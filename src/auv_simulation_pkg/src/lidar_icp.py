"""
Projective Point-to-Plane ICP node for ROS2 Humble.

Refines EKF odometry (x, y, z, yaw) using a solid-state small-FOV 3D lidar.
Pitch and roll are taken as absolute from the EKF and are NOT modified.
"""

import numpy as np
from scipy.spatial import cKDTree
from scipy.linalg import solve

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import Quaternion
import tf_transformations 

# ---------------------------------------------------------------------------
# Quaternion / rotation helpers
# ---------------------------------------------------------------------------
def quat_to_rpy(q: Quaternion):
    """Geometry_msgs Quaternion → (roll, pitch, yaw)"""
    return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


def rpy_to_quat(roll, pitch, yaw):
    """(roll, pitch, yaw) → geometry_msgs Quaternion"""
    q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    msg = Quaternion()
    msg.x, msg.y, msg.z, msg.w = q
    return msg


def build_transform(x, y, z, roll, pitch, yaw):
    """Build a 4×4 homogeneous transform from translation + RPY"""
    T = tf_transformations.euler_matrix(roll, pitch, yaw)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


# ---------------------------------------------------------------------------
# Point-to-plane ICP (restricted to x, y, z, yaw)
# ---------------------------------------------------------------------------
def estimate_normals(points: np.ndarray, k: int = 10) -> np.ndarray:
    """
    Estimate surface normals via PCA on k-nearest neighbours
    points : (N, 3)
    returns: (N, 3) unit normals
    """
    n_pts = len(points)
    k_actual = min(k, n_pts)
    if k_actual < 3:
        return np.tile([0.0, 0.0, 1.0], (n_pts, 1))

    tree = cKDTree(points)
    _, idx = tree.query(points, k=k_actual, workers=-1)

    if idx.ndim == 1:
        idx = idx[:, np.newaxis]

    normals = np.zeros_like(points)
    for i in range(n_pts):
        neighbours = idx[i]
        neighbourhood = points[neighbours]
        centroid = neighbourhood.mean(axis=0)
        diff = neighbourhood - centroid
        cov = diff.T @ diff
        eigvals, eigvecs = np.linalg.eigh(cov)
        normals[i] = eigvecs[:, 0] 
    return normals

def icp_point_to_plane_restricted(
    source: np.ndarray,
    target: np.ndarray,
    target_normals: np.ndarray,
    init_T: np.ndarray,
    max_iter: int = 15,
    tolerance: float = 1e-6,
    max_corr_dist: float = 1.0,
):
    """
    Point-to-plane ICP solving only for (tx, ty, tz, yaw)

    Parameters:
    source        : (M, 3) source cloud (current scan)
    target        : (N, 3) target cloud (previous scan, in odom frame)
    target_normals: (N, 3) normals of target cloud
    init_T        : 4×4 initial transform guess (full 6-DOF)
    max_iter      : maximum iterations
    tolerance     : convergence threshold on parameter update norm
    max_corr_dist : reject correspondences farther than this (metres)

    Returns:
    T_refined : 4×4 refined transform
    converged : bool
    """

    T = init_T.copy()
    tree = cKDTree(target)
    converged = False

    for iteration in range(max_iter):
        # transform source into target frame with current T
        src_h = np.hstack([source, np.ones((len(source), 1))])
        src_t = (T @ src_h.T).T[:, :3]

        # find closest target points
        dists, indices = tree.query(src_t, k=1, workers=-1)
        mask = dists < max_corr_dist
        if mask.sum() < 10:
            break  # not enough correspondences

        p = src_t[mask]              # transformed source pts
        q = target[indices[mask]]    # closest target pts
        n = target_normals[indices[mask]]  # normals

        # build the linear system for 4 unknowns
        residuals = np.sum(n * (p - q), axis=1)

        J = np.zeros((len(residuals), 4))
        J[:, 0] = n[:, 0]
        J[:, 1] = n[:, 1]
        J[:, 2] = n[:, 2]
        J[:, 3] = -n[:, 0] * p[:, 1] + n[:, 1] * p[:, 0]

        # Solve
        JtJ = J.T @ J
        Jtr = J.T @ residuals
        try:
            delta = solve(JtJ, -Jtr, assume_a="pos")
        except np.linalg.LinAlgError:
            break

        # Build incremental transform (only tx, ty, tz, yaw)
        dT = build_transform(delta[0], delta[1], delta[2],
                             0.0, 0.0, delta[3])
        T = dT @ T

        if np.linalg.norm(delta) < tolerance:
            converged = True
            break

    return T, converged

# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
class LidarICP(Node):
    def __init__(self):
        super().__init__("lidar_icp")

        # Param
        self.declare_parameter("max_icp_iter", 15)
        self.declare_parameter("icp_tolerance", 1e-6)
        self.declare_parameter("max_corr_dist", 1.0)
        self.declare_parameter("voxel_size", 0.1)        # down-sample leaf
        self.declare_parameter("normal_k", 10)            # k for normal estimation
        self.declare_parameter("min_scan_points", 50)

        self.max_icp_iter = self.get_parameter("max_icp_iter").value
        self.icp_tolerance = self.get_parameter("icp_tolerance").value
        self.max_corr_dist = self.get_parameter("max_corr_dist").value
        self.voxel_size = self.get_parameter("voxel_size").value
        self.normal_k = self.get_parameter("normal_k").value
        self.min_scan_points = self.get_parameter("min_scan_points").value

        # State variables
        self.prev_cloud = None      
        self.prev_normals = None
        self.latest_odom: Odometry = None

        # QoS for lidar data
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/fuse_odom", self._odom_cb, 10
        )
        self.scan_sub = self.create_subscription(
            PointCloud2, "/scan_3D", self._scan_cb, sensor_qos
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, "/odom_refined", 10)

        self.get_logger().info("LidarICP ready – waiting for data …")

    # callbacks

    # Store latest odom received from subcribtion to /fused_odom topic 
    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    # Store lastest scan received from lidar
    def _scan_cb(self, msg: PointCloud2):
        if self.latest_odom is None:
            self.get_logger().warn("No odometry yet – skipping scan.", throttle_duration_sec=2.0)
            return

        # extract XYZ from PointCloud2
        points = self._pc2_to_numpy(msg)
        if points is None or len(points) < self.min_scan_points:
            return

        # downsample
        points = self._voxel_downsample(points, self.voxel_size)
        if len(points) < self.min_scan_points:
            return

        # transform initial guess from odom topic
        odom = self.latest_odom
        roll, pitch, yaw = quat_to_rpy(odom.pose.pose.orientation)
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        T_odom = build_transform(x, y, z, roll, pitch, yaw)
        pts_h = np.hstack([points, np.ones((len(points), 1))])
        pts_odom = (T_odom @ pts_h.T).T[:, :3]

        # no adjustement for first scan (republish data from odom topic)
        if self.prev_cloud is None:
            self.prev_cloud = pts_odom
            self.prev_normals = estimate_normals(pts_odom, self.normal_k)
            self._publish_odom(odom, x, y, z, roll, pitch, yaw)
            return

        # ICP: align current scan (odom frame) to previous scam
        # init_T = identity (both clouds are already roughly in odom frame)
        init_T = np.eye(4)

        T_correction, converged = icp_point_to_plane_restricted(
            source=pts_odom,
            target=self.prev_cloud,
            target_normals=self.prev_normals,
            init_T=init_T,
            max_iter=self.max_icp_iter,
            tolerance=self.icp_tolerance,
            max_corr_dist=self.max_corr_dist,
        )

        # Apply correction to the odom-frame points for storage
        corrected_h = np.hstack([pts_odom, np.ones((len(pts_odom), 1))])
        corrected_pts = (T_correction @ corrected_h.T).T[:, :3]

        # Extract refined pose 
        T_refined = T_correction @ T_odom

        # Decompose: keep ONLY x, y, z, yaw from ICP; pitch & roll from EKF
        refined_rpy = tf_transformations.euler_from_matrix(T_refined)
        ref_x = T_refined[0, 3]
        ref_y = T_refined[1, 3]
        ref_z = T_refined[2, 3]
        ref_yaw = refined_rpy[2]
        ref_roll = roll
        ref_pitch = pitch

        self._publish_odom(odom, ref_x, ref_y, ref_z, ref_roll, ref_pitch, ref_yaw)

        # Update stored cloud
        self.prev_cloud = corrected_pts
        self.prev_normals = estimate_normals(corrected_pts, self.normal_k)

        if not converged:
            self.get_logger().debug("ICP did not converge this frame.")

    # ---------------------------------------------------------------------------
    # helpers
    # ---------------------------------------------------------------------------
    @staticmethod
    def _pc2_to_numpy(msg: PointCloud2) -> np.ndarray | None:
        """Convert PointCloud2 → (N, 3) float64 array of XYZ"""
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_structured = np.array(list(gen))
        if pts_structured.ndim == 0 or len(pts_structured) == 0:
            return None

        pts = np.column_stack([
            pts_structured['x'].astype(np.float64),
            pts_structured['y'].astype(np.float64),
            pts_structured['z'].astype(np.float64),
        ])
        # Remove any row containing NaN or Inf
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) == 0:
            return None
        return pts

    @staticmethod
    def _voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
        """Hash-based voxel grid down-sampling"""
        if leaf <= 0:
            return points
        keys = np.floor(points / leaf).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return points[idx]

    def _publish_odom(self, ref_odom: Odometry,
                      x, y, z, roll, pitch, yaw):
        """Build and publish the refined Odometry message"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ref_odom.header.frame_id
        msg.child_frame_id = ref_odom.child_frame_id

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation = rpy_to_quat(roll, pitch, yaw)
        msg.twist = ref_odom.twist

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarICP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
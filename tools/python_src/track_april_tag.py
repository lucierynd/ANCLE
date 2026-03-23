import cv2
import numpy as np
import argparse
from dt_apriltags import Detector


def rodrigues_to_quaternion(rvec):
    R, _ = cv2.Rodrigues(rvec)
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n > 0:
        q /= n
    return q


def _as_int_pt(p):
    p = np.asarray(p).reshape(2,)
    if not np.all(np.isfinite(p)):
        return None
    return (int(round(float(p[0]))), int(round(float(p[1]))))


def draw_tag_overlay(frame, det, K, dist, tag_size, rvec, tvec):
    h, w = frame.shape[:2]

    def to_py_int(x):        return int(np.int64(np.round(float(x))))

    def safe_pt(p):
        p = np.asarray(p, dtype=np.float64).reshape(2,)
        if not np.all(np.isfinite(p)):
            return None
        x = to_py_int(p[0])
        y = to_py_int(p[1])
        x = max(0, min(w - 1, x))
        y = max(0, min(h - 1, y))
        return (int(x), int(y))

    # Tag boundary
    corners = np.asarray(det.corners, dtype=np.float64)
    corners_i = np.round(corners).astype(np.int32)

    for i in range(4):
        p1 = safe_pt(corners_i[i])
        p2 = safe_pt(corners_i[(i + 1) % 4])
        if p1 is not None and p2 is not None:
            x1, y1 = p1
            x2, y2 = p2
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Corner index
    for i, c in enumerate(corners_i):
        pc = safe_pt(c)
        if pc is not None:
            xc, yc = pc
            cv2.putText(frame, str(i), (xc, yc), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # Center + id
    center = safe_pt(det.center)
    if center is not None:
        cx, cy = center
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(frame, f"id={det.tag_id}", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Pose axes 
    axis_len = float(tag_size) * 0.5
    axis_3d = np.array([
        [0, 0, 0],
        [axis_len, 0, 0],
        [0, axis_len, 0],
        [0, 0, axis_len]
    ], dtype=np.float32)

    imgpts, _ = cv2.projectPoints(axis_3d, rvec, tvec, K, dist)
    imgpts = np.asarray(imgpts, dtype=np.float64).reshape(-1, 2)

    origin = safe_pt(imgpts[0])
    xpt = safe_pt(imgpts[1])
    ypt = safe_pt(imgpts[2])
    zpt = safe_pt(imgpts[3])

    if origin is not None and xpt is not None:
        ox, oy = origin
        xx, xy = xpt
        cv2.line(frame, (ox, oy), (xx, xy), (0, 0, 255), 2)

    if origin is not None and ypt is not None:
        ox, oy = origin
        yx, yy = ypt
        cv2.line(frame, (ox, oy), (yx, yy), (0, 255, 0), 2)

    if origin is not None and zpt is not None:
        ox, oy = origin
        zx, zy = zpt
        cv2.line(frame, (ox, oy), (zx, zy), (255, 0, 0), 2)


def _make_debug_writer(path, fps, frame_shape, scale=0.5):
    h, w = frame_shape[:2]
    out_w = int(round(w * scale))
    out_h = int(round(h * scale))
    out_w = max(2, out_w)
    out_h = max(2, out_h)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(path, fourcc, fps, (out_w, out_h))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open debug video writer: {path}")
    return writer, (out_w, out_h)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True, help="Input video path")
    parser.add_argument("--output", default="trajectory.txt", help="Output txt file")
    parser.add_argument("--tag_size", type=float, required=True, help="AprilTag side length in meters (e.g., 0.16)")
    parser.add_argument("--family", default="tag36h11", help="Tag family")
    parser.add_argument("--only_tag_id", type=int, default=None, help="If set, process only this tag id")

    parser.add_argument("--debug_video", default=None, help="If set, writes annotated debug video to this path (e.g., /data/debug.mp4)")
    parser.add_argument("--debug_scale", type=float, default=0.5, help="Scale factor for debug video (e.g., 0.5 -> half res)")

    parser.add_argument("--write_nan", action="store_true", help="Write 'nan' pose for frames without detection/pose")
    args = parser.parse_args()

    # Camera calibration
    fx = 1100.620222718
    fy = 1100.620222718
    cx = 1057.920114125
    cy = 525.090996122
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)

    dist = np.array([0.016197137, -0.018678607, -0.005285622, 0.003325894], dtype=np.float64)

    detector = Detector(
        families=args.family,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {args.video}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps is None or fps <= 0:
        fps = 30.0

    debug_writer = None
    debug_size = None

    # 3D tag corners in tag frame centered at (0,0,0), Z=0 plane
    s = args.tag_size / 2.0
    obj_pts = np.array([
        [-s, -s, 0],
        [ s, -s, 0],
        [ s,  s, 0],
        [-s,  s, 0]
    ], dtype=np.float64)

    with open(args.output, "w") as f:
        f.write("# timestamp x y z q1 q2 q3 q4\n")

        frame_idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            if debug_writer is None and args.debug_video is not None:
                debug_writer, debug_size = _make_debug_writer(args.debug_video, fps, frame.shape, scale=args.debug_scale)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray, estimate_tag_pose=False)

            # Pick tag: requested ID, else largest by quad area
            selected = None
            if detections:
                if args.only_tag_id is not None:
                    for d in detections:
                        if d.tag_id == args.only_tag_id:
                            selected = d
                            break
                else:
                    def quad_area(c):
                        c = np.asarray(c)
                        return 0.5 * abs(np.dot(c[:, 0], np.roll(c[:, 1], 1)) -
                                         np.dot(c[:, 1], np.roll(c[:, 0], 1)))
                    selected = max(detections, key=lambda d: quad_area(d.corners))

            timestamp = frame_idx / fps

            wrote_pose = False

            if selected is not None:
                img_pts = np.asarray(selected.corners, dtype=np.float64)

                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, K, dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success and np.all(np.isfinite(tvec)) and np.all(np.isfinite(rvec)):
                    q = rodrigues_to_quaternion(rvec)

                    x = float(tvec[0][0])
                    y = float(tvec[1][0])
                    z = 0.0  # forced

                    f.write(f"{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} "
                            f"{q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}\n")
                    wrote_pose = True

                    if debug_writer is not None:
                        draw_tag_overlay(frame, selected, K, dist, args.tag_size, rvec, tvec)
                        cv2.putText(frame, f"tvec=({x:.3f},{y:.3f},{float(tvec[2][0]):.3f})m  time={timestamp:.2f}s",
                                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                else:
                    print(f"[Warning] Frame {frame_idx}: solvePnP failed or produced NaN/Inf")
            else:
                print(f"[Warning] Frame {frame_idx}: no detection")

            if (not wrote_pose) and args.write_nan:
                f.write(f"{timestamp:.6f} nan nan 0.000000 nan nan nan nan\n")

            # Write debug frame
            if debug_writer is not None:
                out_w, out_h = debug_size
                dbg = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)
                debug_writer.write(dbg)

            frame_idx += 1

    cap.release()
    if debug_writer is not None:
        debug_writer.release()

    print(f"Trajectory saved to: {args.output}")
    if args.debug_video is not None:
        print(f"Debug video saved to: {args.debug_video}")


if __name__ == "__main__":
    main()
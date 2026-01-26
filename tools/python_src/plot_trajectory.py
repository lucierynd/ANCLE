#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import argparse
import sys


def read_trajectory_file(filename):
    """Read trajectory data from text file"""
    try:
        data = np.loadtxt(filename, comments='#')
        
        if data.shape[1] != 8:
            print(f"Error: Expected 8 columns, got {data.shape[1]}")
            return None
        
        timestamps = data[:, 0]
        positions = data[:, 1:4]  # x, y, z
        quaternions = data[:, 4:8]  # q0, q1, q2, q3
        
        return timestamps, positions, quaternions
    
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return None
    except Exception as e:
        print(f"Error reading file: {str(e)}")
        return None


def plot_trajectory_3d(timestamps, positions, quaternions, show_orientation=True, arrow_step=10):
    """Plot 3D trajectory with optional orientation arrows"""
    
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Extract x, y, z
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]
    
    # Plot trajectory line colored by time
    num_points = len(timestamps)
    colors = plt.cm.viridis(np.linspace(0, 1, num_points))
    
    # Plot the trajectory as a line
    for i in range(len(x) - 1):
        ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colors[i], linewidth=2)
    
    # Mark start and end points
    ax.scatter(x[0], y[0], z[0], color='green', s=200, marker='o', 
               label='Start', edgecolors='black', linewidths=2)
    ax.scatter(x[-1], y[-1], z[-1], color='red', s=200, marker='o', 
               label='End', edgecolors='black', linewidths=2)
    
    # Add orientation arrows
    if show_orientation and len(quaternions) > 0:
        # Plot arrows at regular intervals
        arrow_length = np.max([np.ptp(x), np.ptp(y), np.ptp(z)]) * 0.05  # 5% of max range
        
        for i in range(0, len(positions), arrow_step):
            # Convert quaternion to rotation matrix
            rot = Rotation.from_quat([
                quaternions[i, 0],
                quaternions[i, 1],
                quaternions[i, 2],
                quaternions[i, 3]
            ])
            
            # Get forward direction (assuming x-axis is forward)
            forward = rot.apply([1, 0, 0]) * arrow_length
            
            # Draw arrow
            ax.quiver(x[i], y[i], z[i], 
                     forward[0], forward[1], forward[2],
                     color='red', alpha=0.6, arrow_length_ratio=0.3, linewidth=1.5)
    
    # Labels and title
    ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    ax.set_title('Robot 3D Trajectory (Ground Truth)', fontsize=14, fontweight='bold')
    
    # Equal aspect ratio
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Add grid
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)
    
    # Add colorbar for time
    sm = plt.cm.ScalarMappable(cmap='viridis', 
                               norm=plt.Normalize(vmin=timestamps[0], vmax=timestamps[-1]))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, pad=0.1, shrink=0.8)
    cbar.set_label('Time (s)', fontsize=10, fontweight='bold')
    
    plt.tight_layout()
    
    return fig, ax


def plot_position_vs_time(timestamps, positions):
    """Plot x, y, z positions over time"""
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    
    labels = ['X', 'Y', 'Z']
    colors = ['red', 'green', 'blue']
    
    for i, (ax, label, color) in enumerate(zip(axes, labels, colors)):
        ax.plot(timestamps - timestamps[0], positions[:, i], 
                color=color, linewidth=2, label=f'{label} position')
        ax.set_ylabel(f'{label} (m)', fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
    
    axes[-1].set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    axes[0].set_title('Position Components vs Time', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    
    return fig


def plot_2d_trajectory(positions):
    """Plot XY, XZ, and YZ projections"""
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]
    
    # XY plane
    axes[0].plot(x, y, 'b-', linewidth=2, alpha=0.7)
    axes[0].scatter(x[0], y[0], color='green', s=100, marker='o', 
                    label='Start', edgecolors='black', zorder=5)
    axes[0].scatter(x[-1], y[-1], color='red', s=100, marker='o', 
                    label='End', edgecolors='black', zorder=5)
    axes[0].set_xlabel('X (m)', fontweight='bold')
    axes[0].set_ylabel('Y (m)', fontweight='bold')
    axes[0].set_title('XY Plane (Top View)', fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].axis('equal')
    axes[0].legend()
    
    # XZ plane
    axes[1].plot(x, z, 'g-', linewidth=2, alpha=0.7)
    axes[1].scatter(x[0], z[0], color='green', s=100, marker='o', 
                    label='Start', edgecolors='black', zorder=5)
    axes[1].scatter(x[-1], z[-1], color='red', s=100, marker='o', 
                    label='End', edgecolors='black', zorder=5)
    axes[1].set_xlabel('X (m)', fontweight='bold')
    axes[1].set_ylabel('Z (m)', fontweight='bold')
    axes[1].set_title('XZ Plane (Side View)', fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    axes[1].axis('equal')
    axes[1].legend()
    
    # YZ plane
    axes[2].plot(y, z, 'r-', linewidth=2, alpha=0.7)
    axes[2].scatter(y[0], z[0], color='green', s=100, marker='o', 
                    label='Start', edgecolors='black', zorder=5)
    axes[2].scatter(y[-1], z[-1], color='red', s=100, marker='o', 
                    label='End', edgecolors='black', zorder=5)
    axes[2].set_xlabel('Y (m)', fontweight='bold')
    axes[2].set_ylabel('Z (m)', fontweight='bold')
    axes[2].set_title('YZ Plane (Front View)', fontweight='bold')
    axes[2].grid(True, alpha=0.3)
    axes[2].axis('equal')
    axes[2].legend()
    
    plt.tight_layout()
    
    return fig


def print_statistics(timestamps, positions):
    """Print trajectory statistics"""
    
    # Calculate total distance
    distances = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
    total_distance = np.sum(distances)
    
    # Calculate duration
    duration = timestamps[-1] - timestamps[0]
    
    # Calculate average speed
    avg_speed = total_distance / duration if duration > 0 else 0
    
    print("\n" + "="*50)
    print("TRAJECTORY STATISTICS")
    print("="*50)
    print(f"Number of poses: {len(timestamps)}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Total distance: {total_distance:.2f} meters")
    print(f"Average speed: {avg_speed:.3f} m/s")
    print(f"\nPosition range:")
    print(f"  X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
    print(f"  Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
    print(f"  Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
    print(f"\nFinal position:")
    print(f"  X: {positions[-1, 0]:.3f} m")
    print(f"  Y: {positions[-1, 1]:.3f} m")
    print(f"  Z: {positions[-1, 2]:.3f} m")
    print("="*50 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Plot 3D trajectory from ground truth file')
    parser.add_argument('filename', type=str, help='Path to trajectory txt file')
    parser.add_argument('--no-orientation', action='store_true', 
                       help='Hide orientation arrows')
    parser.add_argument('--arrow-step', type=int, default=10, 
                       help='Step between orientation arrows (default: 10)')
    parser.add_argument('--save', type=str, default=None, 
                       help='Save plots to file (e.g., trajectory.png)')
    
    args = parser.parse_args()
    
    # Read trajectory file
    result = read_trajectory_file(args.filename)
    
    if result is None:
        sys.exit(1)
    
    timestamps, positions, quaternions = result
    
    print(f"Successfully loaded {len(timestamps)} poses from '{args.filename}'")
    
    # Print statistics
    print_statistics(timestamps, positions)
    
    # Create plots
    fig1, ax1 = plot_trajectory_3d(timestamps, positions, quaternions, 
                                     show_orientation=not args.no_orientation,
                                     arrow_step=args.arrow_step)
    
    fig2 = plot_position_vs_time(timestamps, positions)
    
    fig3 = plot_2d_trajectory(positions)
    
    # Save plots if requested
    if args.save:
        base_name = args.save.rsplit('.', 1)[0]
        fig1.savefig(f'{base_name}_3d.png', dpi=300, bbox_inches='tight')
        fig2.savefig(f'{base_name}_time.png', dpi=300, bbox_inches='tight')
        fig3.savefig(f'{base_name}_2d.png', dpi=300, bbox_inches='tight')
        print(f"Plots saved with prefix '{base_name}'")
    
    # Show plots
    plt.show()


if __name__ == '__main__':
    main()
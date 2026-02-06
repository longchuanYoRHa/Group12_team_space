#!/usr/bin/env python3
"""
Robot Manipulator Workspace Visualization Tool
Displays the reachable workspace of myCobot 280 robotic arm
Similar to QUT Robot Workspace visualization
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

class ManipulatorWorkspace:
    """Robot manipulator workspace calculation and visualization class"""
    
    def __init__(self):
        """Initialize robot parameters"""
        # Joint angle limits (degrees to radians)
        # Based on joint limit information from specifications
        self.joint_limits = {
            'J1': (-168, 168),  # degrees
            'J2': (-140, 140),
            'J3': (-150, 150),
            'J4': (-150, 150),
            'J5': (-155, 160),
            'J6': (-180, 180)
        }
        
        # Convert to radians
        self.joint_limits_rad = {
            'J1': (np.deg2rad(-168), np.deg2rad(168)),
            'J2': (np.deg2rad(-140), np.deg2rad(140)),
            'J3': (np.deg2rad(-150), np.deg2rad(150)),
            'J4': (np.deg2rad(-150), np.deg2rad(150)),
            'J5': (np.deg2rad(-155), np.deg2rad(160)),
            'J6': (np.deg2rad(-180), np.deg2rad(180))
        }
        
        # Link lengths extracted from URDF (meters)
        self.link_lengths = {
            'L1': 0.13156,   # link1 height
            'L2': 0.1104,    # link3 length
            'L3': 0.096,     # link4 length
            'L4': 0.07318,   # link5 offset
            'L5': 0.0456     # link6 to end effector
        }
    
    def rotation_matrix(self, axis, angle):
        """
        Calculate rotation matrix around specified axis
        
        Args:
            axis: Rotation axis ('x', 'y', 'z')
            angle: Rotation angle (radians)
        
        Returns:
            3x3 rotation matrix
        """
        c = np.cos(angle)
        s = np.sin(angle)
        
        if axis == 'x':
            return np.array([
                [1, 0, 0],
                [0, c, -s],
                [0, s, c]
            ])
        elif axis == 'y':
            return np.array([
                [c, 0, s],
                [0, 1, 0],
                [-s, 0, c]
            ])
        elif axis == 'z':
            return np.array([
                [c, -s, 0],
                [s, c, 0],
                [0, 0, 1]
            ])
    
    def forward_kinematics(self, joint_angles):
        """
        Forward kinematics calculation (based on URDF transformations)
        
        Args:
            joint_angles: List of 6 joint angles (radians)
        
        Returns:
            End effector position [x, y, z]
        """
        # Extract joint angles
        q1, q2, q3, q4, q5, q6 = joint_angles
        
        # Build transformation matrices based on URDF transformation sequence
        # Initial position and orientation (base frame)
        T = np.eye(4)
        
        # Joint 1: link1_to_link2
        # origin xyz="0 0 0.13156" rpy="0 0 pi/2"
        T_base_to_link1 = np.eye(4)
        T_base_to_link1[2, 3] = 0.13156  # z offset
        R_z_90 = self.rotation_matrix('z', np.pi/2)
        T_base_to_link1[:3, :3] = R_z_90
        
        # Joint 1 rotation
        R_j1 = self.rotation_matrix('z', q1)
        T_j1 = np.eye(4)
        T_j1[:3, :3] = R_j1
        
        # Joint 2: link2_to_link3
        # origin xyz="0 0 -0.001" rpy="0 pi/2 -pi/2"
        T_link2_to_link3 = np.eye(4)
        T_link2_to_link3[2, 3] = -0.001
        R_y_90 = self.rotation_matrix('y', np.pi/2)
        R_z_minus90 = self.rotation_matrix('z', -np.pi/2)
        T_link2_to_link3[:3, :3] = R_z_minus90 @ R_y_90
        
        # Joint 2 rotation
        R_j2 = self.rotation_matrix('z', q2)
        T_j2 = np.eye(4)
        T_j2[:3, :3] = R_j2
        
        # Joint 3: link3_to_link4
        # origin xyz="-0.1104 0 0" rpy="0 0 0"
        T_link3_to_link4 = np.eye(4)
        T_link3_to_link4[0, 3] = -0.1104
        
        # Joint 3 rotation
        R_j3 = self.rotation_matrix('z', q3)
        T_j3 = np.eye(4)
        T_j3[:3, :3] = R_j3
        
        # Joint 4: link4_to_link5
        # origin xyz="-0.096 0 0.06062" rpy="0 0 -pi/2"
        T_link4_to_link5 = np.eye(4)
        T_link4_to_link5[0, 3] = -0.096
        T_link4_to_link5[2, 3] = 0.06062
        R_z_minus90_j4 = self.rotation_matrix('z', -np.pi/2)
        T_link4_to_link5[:3, :3] = R_z_minus90_j4
        
        # Joint 4 rotation
        R_j4 = self.rotation_matrix('z', q4)
        T_j4 = np.eye(4)
        T_j4[:3, :3] = R_j4
        
        # Joint 5: link5_to_link6
        # origin xyz="0 -0.07318 0" rpy="pi/2 -pi/2 0"
        T_link5_to_link6 = np.eye(4)
        T_link5_to_link6[1, 3] = -0.07318
        R_x_90 = self.rotation_matrix('x', np.pi/2)
        R_y_minus90 = self.rotation_matrix('y', -np.pi/2)
        T_link5_to_link6[:3, :3] = R_y_minus90 @ R_x_90
        
        # Joint 5 rotation
        R_j5 = self.rotation_matrix('z', q5)
        T_j5 = np.eye(4)
        T_j5[:3, :3] = R_j5
        
        # Joint 6: link6_to_flange
        # origin xyz="0 0.0456 0" rpy="-pi/2 0 0"
        T_link6_to_flange = np.eye(4)
        T_link6_to_flange[1, 3] = 0.0456
        R_x_minus90 = self.rotation_matrix('x', -np.pi/2)
        T_link6_to_flange[:3, :3] = R_x_minus90
        
        # Joint 6 rotation
        R_j6 = self.rotation_matrix('z', q6)
        T_j6 = np.eye(4)
        T_j6[:3, :3] = R_j6
        
        # Combine all transformations
        T = T_base_to_link1 @ T_j1 @ T_link2_to_link3 @ T_j2 @ T_link3_to_link4 @ T_j3 @ \
            T_link4_to_link5 @ T_j4 @ T_link5_to_link6 @ T_j5 @ T_link6_to_flange @ T_j6
        
        # Extract position
        position = T[:3, 3]
        
        return position
    
    def generate_workspace_points(self, num_samples=100000):
        """
        Generate workspace point cloud
        
        Args:
            num_samples: Number of sample points
        
        Returns:
            3D coordinate array of workspace points
        """
        points = []
        
        print(f"Generating {num_samples} workspace points...")
        
        for i in range(num_samples):
            # Random sample joint angles
            q1 = np.random.uniform(*self.joint_limits_rad['J1'])
            q2 = np.random.uniform(*self.joint_limits_rad['J2'])
            q3 = np.random.uniform(*self.joint_limits_rad['J3'])
            q4 = np.random.uniform(*self.joint_limits_rad['J4'])
            q5 = np.random.uniform(*self.joint_limits_rad['J5'])
            q6 = np.random.uniform(*self.joint_limits_rad['J6'])
            
            # Calculate end effector position
            joint_angles = [q1, q2, q3, q4, q5, q6]
            position = self.forward_kinematics(joint_angles)
            
            points.append(position)
            
            if (i + 1) % 20000 == 0:
                print(f"  Generated {i + 1} points...")
        
        return np.array(points)
    
    def compute_workspace_boundary(self, points_2d):
        """
        Compute workspace boundary using convex hull
        
        Args:
            points_2d: 2D array of points (N x 2)
        
        Returns:
            Boundary points as array
        """
        if len(points_2d) < 3:
            return points_2d
        
        try:
            hull = ConvexHull(points_2d)
            boundary_points = points_2d[hull.vertices]
            # Close the polygon
            boundary_points = np.vstack([boundary_points, boundary_points[0]])
            return boundary_points
        except:
            # If convex hull fails, return original points
            return points_2d
    
    def plot_workspace(self, points):
        """
        Plot workspace with filled regions (similar to QUT visualization)
        
        Args:
            points: Workspace point cloud
        """
        # Create figure with two views: side view (XZ) and top view (XY)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # ========== Side View (XZ plane projection) ==========
        xz_points = points[:, [0, 2]]  # X and Z coordinates
        
        # Compute boundary for side view
        xz_boundary = self.compute_workspace_boundary(xz_points)
        
        # Plot filled workspace region
        ax1.fill(xz_boundary[:, 0], xz_boundary[:, 1], 
                color='lightblue', alpha=0.6, edgecolor='blue', linewidth=1.5, label='Workspace')
        
        # Add some sample points for density visualization
        ax1.scatter(xz_points[:, 0], xz_points[:, 1], 
                   c='darkblue', alpha=0.1, s=0.5, zorder=1)
        
        ax1.set_xlabel('X (m)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Z (m)', fontsize=12, fontweight='bold')
        ax1.set_title('Robot Workspace - Side View', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        ax1.legend(loc='upper right')
        
        # Add origin marker
        ax1.plot(0, 0, 'ko', markersize=8, label='Base')
        ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
        ax1.axvline(x=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
        
        # ========== Top View (XY plane projection) ==========
        xy_points = points[:, [0, 1]]  # X and Y coordinates
        
        # Compute boundary for top view
        xy_boundary = self.compute_workspace_boundary(xy_points)
        
        # Plot filled workspace region
        ax2.fill(xy_boundary[:, 0], xy_boundary[:, 1], 
                color='lightblue', alpha=0.6, edgecolor='blue', linewidth=1.5, label='Workspace')
        
        # Add some sample points for density visualization
        ax2.scatter(xy_points[:, 0], xy_points[:, 1], 
                   c='darkblue', alpha=0.1, s=0.5, zorder=1)
        
        ax2.set_xlabel('X (m)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
        ax2.set_title('Robot Workspace - Top View', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal')
        ax2.legend(loc='upper right')
        
        # Add origin marker
        ax2.plot(0, 0, 'ko', markersize=8, label='Base')
        ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
        ax2.axvline(x=0, color='k', linestyle='--', linewidth=0.5, alpha=0.5)
        
        plt.tight_layout()
        plt.show()
    
    def print_workspace_stats(self, points):
        """
        Print workspace statistics
        
        Args:
            points: Workspace point cloud
        """
        print("\n" + "="*60)
        print("Workspace Statistics")
        print("="*60)
        print(f"Total sample points: {len(points)}")
        print(f"\nX-axis range: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] m")
        print(f"Y-axis range: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] m")
        print(f"Z-axis range: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}] m")
        print(f"\nX-axis span: {points[:, 0].max() - points[:, 0].min():.3f} m")
        print(f"Y-axis span: {points[:, 1].max() - points[:, 1].min():.3f} m")
        print(f"Z-axis span: {points[:, 2].max() - points[:, 2].min():.3f} m")
        print(f"\nWorkspace center: ({points[:, 0].mean():.3f}, {points[:, 1].mean():.3f}, {points[:, 2].mean():.3f}) m")
        
        # Calculate distance to origin
        distances = np.linalg.norm(points, axis=1)
        print(f"\nMaximum reach distance: {distances.max():.3f} m")
        print(f"Minimum reach distance: {distances.min():.3f} m")
        print(f"Average reach distance: {distances.mean():.3f} m")
        print("="*60 + "\n")


def main():
    """Main function"""
    print("="*60)
    print("Robot Manipulator Workspace Visualization Tool")
    print("="*60)
    
    # Create workspace object
    workspace = ManipulatorWorkspace()
    
    # Generate workspace point cloud
    # Adjust number of samples as needed (more points = more accurate but slower)
    num_samples = 100000
    points = workspace.generate_workspace_points(num_samples)
    
    # Print statistics
    workspace.print_workspace_stats(points)
    
    # Visualize workspace
    print("Generating visualization...")
    workspace.plot_workspace(points)
    
    print("Complete!")


if __name__ == "__main__":
    main()

"""
Visualize gripper camera calibration result as 3D point cloud.
"""

import numpy as np
import open3d as o3d



def create_coordinate_frame(transform, size=0.1):
    """Create a coordinate frame mesh at given transform."""
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    frame.transform(transform)
    return frame


def visualize_calibration(calibration_file):
    """
    Visualize TCP to camera calibration transform.
    
    Args:
        calibration_file: Path to .npy file containing 4x4 transformation matrix
    """
    # Load calibration
    T_tcp_cam = np.load(calibration_file)
    
    print("Loaded calibration from:", calibration_file)
    print("\nTransformation matrix (TCP to Camera):")
    print(T_tcp_cam)
    print(f"\nTranslation: {T_tcp_cam[:3, 3]}")
    print(f"  X: {T_tcp_cam[0, 3]*1000:.1f}mm")
    print(f"  Y: {T_tcp_cam[1, 3]*1000:.1f}mm")
    print(f"  Z: {T_tcp_cam[2, 3]*1000:.1f}mm")
    
    # Create visualization
    geometries = []
    
    # TCP frame (origin)
    tcp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
    geometries.append(tcp_frame)
    
    # Camera frame (transformed)
    cam_frame = create_coordinate_frame(T_tcp_cam, size=0.10)
    geometries.append(cam_frame)
    
    # Line connecting TCP to camera
    tcp_pos = np.array([0, 0, 0])
    cam_pos = T_tcp_cam[:3, 3]
    line = o3d.geometry.LineSet()
    line.points = o3d.utility.Vector3dVector([tcp_pos, cam_pos])
    line.lines = o3d.utility.Vector2iVector([[0, 1]])
    line.colors = o3d.utility.Vector3dVector([[1, 1, 0]])  # Yellow
    geometries.append(line)
    
    # Text labels (add spheres at key points)
    tcp_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    tcp_sphere.paint_uniform_color([1, 0, 0])  # Red
    geometries.append(tcp_sphere)
    
    cam_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    cam_sphere.translate(cam_pos)
    cam_sphere.paint_uniform_color([0, 0, 1])  # Blue
    geometries.append(cam_sphere)
    
    # Visualize
    print("\n" + "="*60)
    print("VISUALIZATION LEGEND:")
    print("="*60)
    print("Red sphere:   TCP (gripper tool center point)")
    print("Blue sphere:  Camera optical center")
    print("Yellow line:  Transform between TCP and camera")
    print("\nCoordinate frames:")
    print("  Red axis:   X")
    print("  Green axis: Y")
    print("  Blue axis:  Z")
    print("="*60)
    print("\nClose window to exit...")
    
    o3d.visualization.draw_geometries(
        geometries,
        window_name="Gripper Camera Calibration",
        width=1280,
        height=720
    )


if __name__ == "__main__":
    calibration_file = "/home/ceylanb/robot/robot_io/calibration/calibration_files/panda_realsenseD435.npy" # <== change this to calibration file
    visualize_calibration(calibration_file)
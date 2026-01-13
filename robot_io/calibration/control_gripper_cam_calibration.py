"""
Hand-Eye Calibration Investigator
Checks for Z-axis flips, mirrored coordinate systems, and orientation issues in hand-eye calibration matrices.
"""

import numpy as np
from pathlib import Path
import argparse
import sys

# Optional: try to load yaml config
try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


def load_calibration(filepath: Path) -> np.ndarray:
    """Load calibration matrix from .npy or .txt file."""
    suffix = filepath.suffix.lower()
    
    if suffix == '.npy':
        return np.load(filepath)
    elif suffix == '.txt':
        return np.loadtxt(filepath)
    elif suffix == '.npz':
        data = np.load(filepath)
        # Try common key names
        for key in ['T_tcp_cam', 'T', 'transform', 'matrix', data.files[0]]:
            if key in data.files:
                return data[key]
        raise ValueError(f"Could not find matrix in .npz file. Keys: {data.files}")
    else:
        raise ValueError(f"Unsupported file format: {suffix}")


def rotation_to_euler(R: np.ndarray) -> dict:
    """
    Convert rotation matrix to Euler angles (XYZ convention).
    Returns angles in degrees.
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    
    return {
        'x_deg': np.degrees(x),
        'y_deg': np.degrees(y),
        'z_deg': np.degrees(z)
    }


def analyze_calibration(T: np.ndarray, name: str = "Calibration") -> dict:
    """
    Comprehensive analysis of a 4x4 transformation matrix.
    
    Checks:
    1. Rotation matrix validity (orthogonality, determinant)
    2. Z-axis orientation (camera view direction)
    3. Handedness (left vs right-handed coordinate system)
    4. Translation reasonableness
    """
    results = {
        "name": name,
        "issues": [],
        "warnings": [],
        "info": {}
    }
    
    # Extract components
    if T.shape == (4, 4):
        R = T[:3, :3]
        t = T[:3, 3]
    elif T.shape == (3, 4):
        R = T[:3, :3]
        t = T[:, 3]
    elif T.shape == (3, 3):
        R = T
        t = np.zeros(3)
        results["warnings"].append("Matrix is 3x3 (rotation only), no translation")
    else:
        results["issues"].append(f"Unexpected matrix shape: {T.shape}")
        return results
    
    results["info"]["shape"] = T.shape
    results["info"]["translation"] = t.tolist()
    
    # === CHECK 1: Determinant ===
    det = np.linalg.det(R)
    results["info"]["determinant"] = float(det)
    
    if np.isclose(det, -1.0, atol=0.01):
        results["issues"].append(
            f"CRITICAL: Determinant = {det:.4f} (≈ -1). "
            "This is a MIRRORED/REFLECTED matrix (left-handed system). "
            "This WILL cause upside-down or inside-out reconstructions!"
        )
    elif np.isclose(det, 1.0, atol=0.01):
        results["info"]["determinant_status"] = "VALID (proper rotation)"
    else:
        results["issues"].append(
            f"Determinant = {det:.4f}. Expected ±1 for rotation matrix. "
            "Matrix may include scaling or is corrupted."
        )
    
    # === CHECK 2: Orthogonality ===
    RtR = R.T @ R
    identity_error = np.max(np.abs(RtR - np.eye(3)))
    results["info"]["orthogonality_error"] = float(identity_error)
    
    if identity_error > 0.01:
        results["warnings"].append(
            f"Rotation matrix is not orthogonal (error={identity_error:.4f}). "
            "R^T @ R should equal I."
        )
    
    # === CHECK 3: Z-axis Analysis (Camera View Direction) ===
    # In most conventions, the camera looks along +Z or -Z axis
    # The 3rd column of R represents where the camera's Z-axis points in the parent frame
    
    cam_x_axis = R[:, 0]  # Camera X in parent frame
    cam_y_axis = R[:, 1]  # Camera Y in parent frame  
    cam_z_axis = R[:, 2]  # Camera Z (view direction) in parent frame
    
    results["info"]["camera_axes"] = {
        "x_axis": cam_x_axis.tolist(),
        "y_axis": cam_y_axis.tolist(),
        "z_axis (view_direction)": cam_z_axis.tolist()
    }
    
    # Check if camera Z points "up" or "down" in world/TCP frame
    # For a camera mounted looking DOWN on a robot gripper, 
    # we expect cam_z_axis to have a NEGATIVE Z component (pointing down)
    
    z_component = cam_z_axis[2]
    results["info"]["z_axis_vertical_component"] = float(z_component)
    
    if z_component > 0.5:
        results["warnings"].append(
            f"Camera Z-axis points UPWARD (z={z_component:.3f}). "
            "If camera is physically mounted looking DOWN, this calibration is FLIPPED! "
            "This causes the scene to reconstruct upside-down."
        )
    elif z_component < -0.5:
        results["info"]["z_axis_direction"] = "Points DOWNWARD (typical for top-down mounted camera)"
    else:
        results["info"]["z_axis_direction"] = f"Points mostly horizontal (z={z_component:.3f})"
    
    # === CHECK 4: Euler Angles ===
    euler = rotation_to_euler(R)
    results["info"]["euler_angles_xyz"] = euler
    
    # Check for ~180° rotations that might indicate a flip
    for axis, angle in euler.items():
        if abs(abs(angle) - 180) < 15:
            results["warnings"].append(
                f"Euler angle {axis} ≈ {angle:.1f}° (near 180°). "
                "This could indicate an axis flip in calibration."
            )
    
    # === CHECK 5: Translation Magnitude ===
    t_norm = np.linalg.norm(t)
    results["info"]["translation_norm"] = float(t_norm)
    
    # Typical hand-eye calibration: camera is 5-20cm from TCP
    if t_norm > 1.0:
        results["warnings"].append(
            f"Translation magnitude = {t_norm:.3f}m ({t_norm*1000:.1f}mm). "
            "This seems large for hand-eye calibration. Check units (m vs mm)."
        )
    elif t_norm < 0.01:
        results["warnings"].append(
            f"Translation magnitude = {t_norm:.6f}m ({t_norm*1000:.3f}mm). "
            "This seems very small. Camera nearly coincident with TCP?"
        )
    elif t_norm > 0.5:
        results["info"]["translation_note"] = f"Translation = {t_norm:.3f}m - verify this matches physical setup"
    
    # === CHECK 6: Common Flip Patterns ===
    # Check for identity-like matrices with sign flips
    R_abs = np.abs(R)
    if np.allclose(R_abs, np.eye(3), atol=0.1):
        # Matrix is axis-aligned, check for flips
        diag = np.diag(R)
        flipped_axes = []
        for i, (val, axis) in enumerate(zip(diag, ['X', 'Y', 'Z'])):
            if val < -0.9:
                flipped_axes.append(axis)
        
        if flipped_axes:
            results["warnings"].append(
                f"Axis-aligned rotation with FLIPPED axes: {flipped_axes}. "
                "This is a common calibration error."
            )
    
    return results


def print_matrix(T: np.ndarray, name: str):
    """Pretty print a transformation matrix."""
    print(f"\n{name}:")
    print("-" * 50)
    
    if T.shape[0] >= 3 and T.shape[1] >= 3:
        print("Rotation Matrix (3x3):")
        for row in T[:3, :3]:
            print(f"  [{row[0]:10.6f} {row[1]:10.6f} {row[2]:10.6f}]")
    
    if T.shape[1] == 4:
        print(f"\nTranslation Vector:")
        t = T[:3, 3]
        print(f"  [{t[0]:10.6f} {t[1]:10.6f} {t[2]:10.6f}]")
        print(f"  (magnitude: {np.linalg.norm(t):.4f} m = {np.linalg.norm(t)*1000:.2f} mm)")


def print_analysis(results: dict, verbose: bool = False):
    """Print analysis results with color coding."""
    colors = {
        "CRITICAL": "\033[91m",  # Red
        "WARNING": "\033[93m",   # Yellow
        "INFO": "\033[94m",      # Blue
        "PASS": "\033[92m",      # Green
        "RESET": "\033[0m"
    }
    
    print(f"\n{'='*60}")
    print(f"ANALYSIS: {results['name']}")
    print(f"{'='*60}")
    
    # Print issues (critical)
    if results["issues"]:
        print(f"\n{colors['CRITICAL']}CRITICAL ISSUES:{colors['RESET']}")
        for issue in results["issues"]:
            print(f"  ✗ {issue}")
    
    # Print warnings
    if results["warnings"]:
        print(f"\n{colors['WARNING']}WARNINGS:{colors['RESET']}")
        for warning in results["warnings"]:
            print(f"  ⚠ {warning}")
    
    # Print key info
    print(f"\n{colors['INFO']}KEY METRICS:{colors['RESET']}")
    info = results["info"]
    
    if "determinant" in info:
        det = info["determinant"]
        status = "✓ VALID" if np.isclose(det, 1.0, atol=0.01) else "✗ INVALID"
        print(f"  Determinant: {det:.6f} {status}")
    
    if "z_axis_vertical_component" in info:
        z = info["z_axis_vertical_component"]
        direction = "UP ⚠" if z > 0.5 else "DOWN ✓" if z < -0.5 else "HORIZONTAL"
        print(f"  Camera Z-axis vertical component: {z:.4f} ({direction})")
    
    if "euler_angles_xyz" in info:
        euler = info["euler_angles_xyz"]
        print(f"  Euler angles (XYZ): X={euler['x_deg']:.1f}° Y={euler['y_deg']:.1f}° Z={euler['z_deg']:.1f}°")
    
    if "translation_norm" in info:
        t_norm = info["translation_norm"]
        print(f"  Translation magnitude: {t_norm:.4f} m ({t_norm*1000:.2f} mm)")
    
    if verbose and "camera_axes" in info:
        print(f"\n{colors['INFO']}CAMERA AXES IN PARENT FRAME:{colors['RESET']}")
        axes = info["camera_axes"]
        for axis_name, axis_vec in axes.items():
            print(f"  {axis_name}: [{axis_vec[0]:.4f}, {axis_vec[1]:.4f}, {axis_vec[2]:.4f}]")
    
    # Overall status
    if not results["issues"] and not results["warnings"]:
        print(f"\n{colors['PASS']}✓ No issues detected{colors['RESET']}")
    elif results["issues"]:
        print(f"\n{colors['CRITICAL']}✗ Critical issues found - calibration likely incorrect{colors['RESET']}")


def scan_calibration_folder(folder: Path) -> list:
    """Find all calibration files in a folder."""
    patterns = ['*.npy', '*.txt', '*.npz']
    files = []
    for pattern in patterns:
        files.extend(folder.glob(pattern))
        files.extend(folder.glob(f"**/{pattern}"))  # Recursive
    return sorted(set(files))


def suggest_fix(results: dict) -> str:
    """Suggest fixes based on detected issues."""
    suggestions = []
    
    for issue in results.get("issues", []) + results.get("warnings", []):
        if "determinant" in issue.lower() and "-1" in issue:
            suggestions.append(
                "FIX for mirrored matrix: Negate one column of the rotation matrix.\n"
                "    R_fixed = R.copy()\n"
                "    R_fixed[:, 2] *= -1  # Flip Z-axis\n"
                "    # Verify det(R_fixed) = +1"
            )
        elif "upward" in issue.lower() or "flipped" in issue.lower():
            suggestions.append(
                "FIX for Z-axis flip: Apply 180° rotation around X or Y axis.\n"
                "    R_flip = np.diag([1, -1, -1])  # 180° around X\n"
                "    T_fixed = T.copy()\n"
                "    T_fixed[:3, :3] = R_flip @ T[:3, :3]"
            )
    
    return "\n\n".join(suggestions) if suggestions else ""


def main():
    parser = argparse.ArgumentParser(
        description="Investigate hand-eye calibration for Z-axis flips and orientation issues"
    )
    parser.add_argument(
        "calibration_path",
        type=str,
        nargs='?',
        default="/home/ceylanb/robot/robot_io/calibration",
        help="Path to calibration file or folder (default: /home/ceylanb/robot/robot_io/calibration)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed information"
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Path to YAML config file to find which calibration is being used"
    )
    
    args = parser.parse_args()
    path = Path(args.calibration_path)
    
    print(f"\n{'#'*60}")
    print("HAND-EYE CALIBRATION INVESTIGATOR")
    print(f"{'#'*60}")
    
    # If config file provided, try to extract calibration path
    if args.config and HAS_YAML:
        config_path = Path(args.config)
        if config_path.exists():
            print(f"\nReading config: {config_path}")
            with open(config_path) as f:
                config = yaml.safe_load(f)
            # Try to find calibration reference in config
            print(f"Config keys: {list(config.keys()) if config else 'empty'}")
    
    # Collect files to analyze
    files_to_check = []
    
    if path.is_file():
        files_to_check = [path]
    elif path.is_dir():
        files_to_check = scan_calibration_folder(path)
        print(f"\nFound {len(files_to_check)} calibration files in {path}")
    else:
        print(f"ERROR: Path not found: {path}")
        sys.exit(1)
    
    if not files_to_check:
        print(f"No calibration files (.npy, .txt, .npz) found in {path}")
        sys.exit(1)
    
    # Analyze each file
    all_results = []
    
    for filepath in files_to_check:
        try:
            T = load_calibration(filepath)
            
            # Print the raw matrix
            print_matrix(T, f"File: {filepath.name}")
            
            # Analyze
            results = analyze_calibration(T, filepath.name)
            all_results.append(results)
            
            # Print analysis
            print_analysis(results, args.verbose)
            
            # Suggest fixes if needed
            fix = suggest_fix(results)
            if fix:
                print(f"\n\033[93mSUGGESTED FIX:\033[0m")
                print(fix)
                
        except Exception as e:
            print(f"\nERROR loading {filepath}: {e}")
    
    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    
    critical_files = [r["name"] for r in all_results if r["issues"]]
    warning_files = [r["name"] for r in all_results if r["warnings"] and not r["issues"]]
    clean_files = [r["name"] for r in all_results if not r["issues"] and not r["warnings"]]
    
    if critical_files:
        print(f"\n✗ Files with CRITICAL issues: {critical_files}")
    if warning_files:
        print(f"⚠ Files with warnings: {warning_files}")
    if clean_files:
        print(f"✓ Clean files: {clean_files}")
    
    print(f"\n{'='*60}")
    print("WHAT TO LOOK FOR:")
    print("  1. Determinant should be +1 (not -1)")
    print("  2. For top-down camera, Z-axis vertical component should be NEGATIVE")
    print("  3. Large Euler angles near ±180° suggest axis flips")
    print("  4. Translation should match physical camera-to-TCP distance")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()

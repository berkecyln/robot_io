import argparse
from pathlib import Path
import numpy as np
import cv2
from omegaconf import OmegaConf
import pyk4a
from pyk4a import Config, PyK4A
from robot_io.cams.camera import Camera


class Kinect4(Camera):
    def __init__(self,
                 device=0,
                 align_depth_to_color=True,
                 resolution='1080p',
                 undistort_image=True,
                 resize_resolution=None,
                 crop_coords=None,
                 fps=30):

        super().__init__(resolution=resolution, crop_coords=crop_coords, resize_resolution=resize_resolution,
                         name="azure_kinect")
        
        print(f"Initializing Kinect on device index: {device}")

        # Map string resolution to PyK4A Enums
        res_map = {
            '720p': pyk4a.ColorResolution.RES_720P,
            '1080p': pyk4a.ColorResolution.RES_1080P,
            '1440p': pyk4a.ColorResolution.RES_1440P,
            '1536p': pyk4a.ColorResolution.RES_1536P,
            '2160p': pyk4a.ColorResolution.RES_2160P,
            '3072p': pyk4a.ColorResolution.RES_3072P,
        }
        
        if resolution not in res_map:
            raise ValueError(f"Resolution '{resolution}' not supported. Options: {list(res_map.keys())}")

        # Configure sensor settings
        k4a_config = Config(
            color_resolution=res_map[resolution],
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            camera_fps=pyk4a.FPS.FPS_30 if fps == 30 else pyk4a.FPS.FPS_15,
            synchronized_images_only=True,
        )

        # Connect to sensor
        try:
            self.sensor = PyK4A(k4a_config, device_id=device)
            self.sensor.start()
            print("Sensor started successfully.")
        except Exception as e:
            raise RuntimeError(f"Failed to connect to sensor on device {device}. Error: {e}")

        # Retrieve device info
        self.serial_number = self.sensor.serial
        print(f"Connected to device serial: {self.serial_number}")
        
        # Load calibration parameters (External file or Factory default)
        data = self.load_config_data(self.serial_number, resolution)

        self.resize_resolution = resize_resolution
        self.intrinsics.update({"crop_coords": self.crop_coords,
                                "resize_resolution": self.resize_resolution,
                                "dist_coeffs": self.dist_coeffs})

        # Initialize undistortion maps
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, R=np.eye(3), \
                                                           newCameraMatrix=self.camera_matrix,
                                                           size=(self.intrinsics['width'], self.intrinsics['height']),
                                                           m1type=cv2.CV_16SC2)
        self.device = device
        self.align_depth_to_color = align_depth_to_color
        self.undistort_image = undistort_image
        self.fps = fps

    def load_config_data(self, serial_number, resolution='1080p'):
        """
        Load calibration parameters for the kinect device.
        Priority:
        1. Custom YAML file in config/ folder.
        2. Factory calibration from device firmware.
        """
        config_path = f"config/{serial_number}.yaml"
        full_path = (Path(__file__).parent / config_path)
        
        if full_path.exists():
            # Load custom calibration from file
            print(f"Loading custom calibration from file: {full_path}")
            conf = OmegaConf.to_container(OmegaConf.load(full_path.as_posix())[resolution])
            self.intrinsics = conf["intrinsics"]
            self.dist_coeffs = np.array(conf["dist_coeffs"])
            if "crop_coords" in conf:
                self.crop_coords = np.array(conf["crop_coords"])
        else:
            # Fallback to factory calibration
            print(f"Config file not found at {full_path}. Reading factory calibration...")
            
            calib = self.sensor.calibration
            mat = calib.get_camera_matrix(pyk4a.CalibrationType.COLOR)
            dist = calib.get_distortion_coefficients(pyk4a.CalibrationType.COLOR)
            
            # Define resolution dimensions
            res_dims = {
                '720p': (1280, 720),
                '1080p': (1920, 1080),
                '1440p': (2560, 1440),
                '1536p': (2048, 1536),
                '2160p': (3840, 2160),
                '3072p': (4096, 3072)
            }
            w, h = res_dims.get(resolution, (1920, 1080))

            self.intrinsics = {
                "width": w,
                "height": h,
                "fx": float(mat[0, 0]),
                "fy": float(mat[1, 1]),
                "cx": float(mat[0, 2]),
                "cy": float(mat[1, 2])
            }
            self.dist_coeffs = dist
            self.crop_coords = None

        # Setup projection matrices
        self.resolution = (self.intrinsics["width"], self.intrinsics["height"])
        
        self.camera_matrix = np.array([[self.intrinsics["fx"], 0, self.intrinsics["cx"]],
                                  [0, self.intrinsics["fy"], self.intrinsics["cy"]],
                                  [0, 0, 1]])

        self.projection_matrix = np.array([[self.intrinsics["fx"], 0, self.intrinsics["cx"], 0],
                                      [0, self.intrinsics["fy"], self.intrinsics["cy"], 0],
                                      [0, 0, 1, 0]])

        data = {'camera_matrix': self.camera_matrix,
                'projection_matrix': self.projection_matrix,
                'dist_coeffs': self.dist_coeffs,
                'crop_coords': self.crop_coords,
                'resolution': resolution,
                'intrinsics': self.intrinsics}

        return data

    def get_intrinsics(self):
        return self.intrinsics

    def get_projection_matrix(self):
        return self.projection_matrix

    def get_camera_matrix(self):
        return self.camera_matrix

    def get_dist_coeffs(self):
        return self.dist_coeffs

    def _get_image(self):
        capture = self.sensor.get_capture()
        
        # Process RGB (BGRA -> RGB)
        if capture.color is None:
             return None, None
        
        rgb = capture.color[:, :, :3][:, :, ::-1] 
        
        # Process Depth (mm -> meters)
        if self.align_depth_to_color:
            depth = capture.transformed_depth
        else:
            depth = capture.depth
            
        if depth is None:
             return rgb, None

        depth = depth.astype(np.float32) / 1000.0

        # Apply undistortion if enabled
        if self.undistort_image:
            rgb = cv2.remap(rgb, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            depth = cv2.remap(depth, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        return rgb, depth
    
    def __del__(self):
        if hasattr(self, 'sensor'):
            self.sensor.stop()


def run_camera(device=0, resolution='1080p'):
    cam = Kinect4(device=device, resolution=resolution)
    print("Intrinsics Loaded:", cam.get_intrinsics())
    
    print("Starting stream... Press 'q' to exit.")
    while True:
        rgb, depth = cam.get_image()
        
        if rgb is not None and depth is not None:
            # Visualization: Normalize depth for display (0-2 meters)
            depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            
            # Show images (convert RGB back to BGR for OpenCV)
            cv2.imshow("depth", depth_vis)
            cv2.imshow("rgb", rgb[:, :, ::-1])
            
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=0, help='device number')
    parser.add_argument('--resolution', type=str, default='1080p', help='resolution of the camera')
    args = parser.parse_args()
    run_camera(device=args.device, resolution=args.resolution)
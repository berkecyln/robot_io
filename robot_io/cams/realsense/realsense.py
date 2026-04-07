## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

import time

import hydra
import numpy as np
import pyrealsense2 as rs
from omegaconf import OmegaConf

from robot_io.cams.camera import Camera
import cv2

class Realsense(Camera):
    """
    Interface class to get data from realsense camera (e.g. framos d435e).
    """

    def __init__(self,
                 fps=30,
                 img_type="rgb",
                 resolution=(640, 480),
                 resize_resolution=None,
                 crop_coords=None,
                 params=None,
                 name=None,
                 flipped_cam=False,
                 depth_filters=False,
                 save_ir=False):
        assert img_type in ['rgb', "rgb_depth"]
        self.img_type = img_type
        # depth_filters and save_ir toggled via yaml config
        self.depth_filters = depth_filters and img_type == 'rgb_depth'
        self.save_ir = save_ir
        self.last_ir_left = None
        self.last_ir_right = None
        super().__init__(resolution=resolution, crop_coords=crop_coords, resize_resolution=resize_resolution, name=name)

        # stream config — IR streams must be added before pipeline.start
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps)
        config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.rgb8, fps)
        if save_ir:
            config.enable_stream(rs.stream.infrared, 1, resolution[0], resolution[1], rs.format.y8, fps)
            config.enable_stream(rs.stream.infrared, 2, resolution[0], resolution[1], rs.format.y8, fps)

        # start streaming
        for i in range(5):
            try:
                self.profile = self.pipeline.start(config)
                break
            except RuntimeError as e:
                print(e)
                print("Retrying")
                time.sleep(2)
        else:
            raise RuntimeError
        self.color_sensor = self.profile.get_device().first_color_sensor()
        self.depth_scale = None
        if img_type == 'rgb_depth':
            self.depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = self.depth_sensor.get_depth_scale()
            # depth post-processing filters
            if self.depth_filters:
                self.temporal_filter = rs.temporal_filter()
                self.spatial_filter = rs.spatial_filter()
                self.hole_fill_filter = rs.hole_filling_filter()

        self.set_parameters(params)

        # depth-to-color alignment
        self.align = rs.align(rs.stream.color)
        
        self.flipped_cam = flipped_cam

    def __del__(self):
        print("exit realsense")
        self.pipeline.stop()

    def _get_image(self):
        """get the current image as a numpy array"""
        while True:
            try:
                frames = self.pipeline.wait_for_frames()
                break
            except RuntimeError as e:
                print(e)

        # IR stereo capture before alignment, same crop and flip as RGB
        if self.save_ir:
            ir_left = np.asanyarray(frames.get_infrared_frame(1).get_data())
            ir_right = np.asanyarray(frames.get_infrared_frame(2).get_data())
            if self.flipped_cam:
                ir_left = cv2.flip(ir_left, -1)
                ir_right = cv2.flip(ir_right, -1)
            self.last_ir_left = self._crop_and_resize(ir_left)
            self.last_ir_right = self._crop_and_resize(ir_right)

        # align depth to color frame
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        if self.flipped_cam:
            color_image = cv2.flip(color_image, -1)
        if self.img_type == 'rgb':
            return color_image, None

        depth_frame = aligned_frames.get_depth_frame()
        if self.depth_filters:
            depth_frame = self.temporal_filter.process(depth_frame)
            depth_frame = self.spatial_filter.process(depth_frame)
            depth_frame = self.hole_fill_filter.process(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = depth_image.astype(np.float32)
        depth_image *= self.depth_scale
        if self.flipped_cam:
            depth_image = cv2.flip(depth_image, -1)
        return color_image, depth_image

    def set_parameters(self, params):
        if params is not None:
            for key, value in params.items():
                self.color_sensor.set_option(getattr(rs.option, key), value)

    def get_intrinsics(self):
        color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()
        
        fx, fy = intr.fx, intr.fy
        cx, cy = intr.ppx, intr.ppy
        width, height = intr.width, intr.height

        # adjust principal point for flip
        if self.flipped_cam:
            cx = width - cx
            cy = height - cy

        # adjust for crop
        if self.crop_coords is not None:
            c = self.crop_coords
            cx -= c[2] # x_start
            cy -= c[0] # y_start
            width = c[3] - c[2]
            height = c[1] - c[0]

        # adjust for resize
        if self.resize_resolution is not None:
            target_w, target_h = self.resize_resolution
            
            scale_x = target_w / width
            scale_y = target_h / height
            
            fx *= scale_x
            fy *= scale_y
            cx *= scale_x
            cy *= scale_y
            
            width = target_w
            height = target_h

        intr_rgb = dict(width=width, height=height, fx=fx, fy=fy,
                        cx=cx, cy=cy, crop_coords=self.crop_coords,
                        resize_resolution=self.resize_resolution, dist_coeffs=self.get_dist_coeffs())
        return intr_rgb

    def get_projection_matrix(self):
        intr = self.get_intrinsics()
        cam_mat = np.array([[intr['fx'], 0, intr['cx'], 0],
                            [0, intr['fy'], intr['cy'], 0],
                            [0, 0, 1, 0]])
        return cam_mat

    def get_camera_matrix(self):
        intr = self.get_intrinsics()
        cam_mat = np.array([[intr['fx'], 0, intr['cx']],
                            [0, intr['fy'], intr['cy']],
                            [0, 0, 1]])
        return cam_mat

    def get_dist_coeffs(self):
        color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()
        return np.array(intr.coeffs)

    def get_ir_intrinsics(self):
        """IR left camera intrinsics at full sensor resolution."""
        ir_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.infrared, 1))
        intr = ir_profile.get_intrinsics()
        return dict(fx=intr.fx, fy=intr.fy, cx=intr.ppx, cy=intr.ppy,
                    width=intr.width, height=intr.height)

    def get_stereo_baseline(self):
        """Stereo baseline between IR left and IR right cameras in meters."""
        ir1 = rs.video_stream_profile(self.profile.get_stream(rs.stream.infrared, 1))
        ir2 = rs.video_stream_profile(self.profile.get_stream(rs.stream.infrared, 2))
        return abs(ir1.get_extrinsics_to(ir2).translation[0])

@hydra.main(config_path="../../conf/cams", config_name="camera_manager.yaml")
def test_cam(cam_cfg):
    import cv2
    # cam_cfg = OmegaConf.load("/export/home/lagandua/robot_io_dev/robot_io/conf/cams/gripper_cam/framos_highres.yaml")
    # cam_cfg = OmegaConf.load("../../conf/cams/gripper_cam/realsense_d435.yaml")
    
    cam = hydra.utils.instantiate(cam_cfg.gripper_cam)

    rgb, depth = cam.get_image()
    print(cam.get_intrinsics())
    while 1:
        rgb, depth = cam.get_image()

        # pc = cam.compute_pointcloud(depth, rgb)
        # cam.view_pointcloud(pc)
        cv2.imshow("rgb", rgb[:, :, ::-1])
        # depth *= (255 / 4)
        # depth = np.clip(depth, 0, 255)
        # depth = depth.astype(np.uint8)
        # depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
        cv2.imshow("depth", depth)
        cv2.waitKey(1)


if __name__ == "__main__":
    test_cam()
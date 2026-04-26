## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

import time

import hydra
import numpy as np
import pyrealsense2 as rs
from robot_io.cams.camera import Camera
import cv2


class Realsense(Camera):
    """
    Interface class to get data from a RealSense camera (D415, D435, etc.).

    Config parameters
    -----------------
    fps              : stream frame rate
    img_type         : "rgb" or "rgb_depth"
    resolution       : [W, H] capture resolution
    resize_resolution: [W, H] output resolution after crop (optional)
    crop_coords      : [y0, y1, x0, x1] crop window (optional)
    flipped_cam      : flip image 180° — set when camera is mounted upside-down
    save_ir          : save IR stereo pair alongside RGB/depth
    params           : color sensor options  (see _apply_color_params)
    depth_params     : depth sensor options  (see _apply_depth_params)
    """

    def __init__(self,
                 fps=30,
                 img_type="rgb",
                 resolution=(640, 480),
                 resize_resolution=None,
                 crop_coords=None,
                 params=None,
                 depth_params=None,
                 name=None,
                 flipped_cam=False,
                 save_ir=False):
        assert img_type in ['rgb', 'rgb_depth']
        self.img_type = img_type
        self.save_ir = save_ir
        self.last_ir_left = None
        self.last_ir_right = None
        super().__init__(resolution=resolution, crop_coords=crop_coords,
                         resize_resolution=resize_resolution, name=name)

        # ── pipeline ─────────────────────────────────────────────────────────
        # Apply visual_preset before starting the pipeline — it is an XU (extended unit)
        # control on D415 that must be set while the device is idle, not while streaming.
        # Setting it after pipeline.start() reliably fails with "Device or resource busy".
        if depth_params and 'visual_preset' in depth_params:
            try:
                ctx = rs.context()
                devices = ctx.query_devices()
                if len(devices) > 0:
                    pre_depth = devices[0].first_depth_sensor()
                    pre_depth.set_option(rs.option.visual_preset, depth_params['visual_preset'])
            except Exception as e:
                print(f"[realsense] pre-stream visual_preset failed: {e}")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps)
        config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.rgb8, fps)
        if save_ir:
            config.enable_stream(rs.stream.infrared, 1, resolution[0], resolution[1], rs.format.y8, fps)
            config.enable_stream(rs.stream.infrared, 2, resolution[0], resolution[1], rs.format.y8, fps)

        for i in range(5):
            try:
                self.profile = self.pipeline.start(config)
                break
            except RuntimeError as e:
                print(e)
                print("Retrying")
                time.sleep(2)
        else:
            raise RuntimeError("Failed to start RealSense pipeline")

        # ── sensors ──────────────────────────────────────────────────────────
        self.color_sensor = self.profile.get_device().first_color_sensor()
        self.depth_sensor = None
        self.depth_scale = None

        if img_type == 'rgb_depth':
            self.depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = self.depth_sensor.get_depth_scale()

        # ── apply parameters ─────────────────────────────────────────────────
        self._apply_color_params(params)
        self._apply_depth_params(depth_params)

        # ── alignment ────────────────────────────────────────────────────────
        self.align = rs.align(rs.stream.color)
        self.flipped_cam = flipped_cam

    def __del__(self):
        print("exit realsense")
        self.pipeline.stop()

    # ── parameter application ─────────────────────────────────────────────────

    def _apply_color_params(self, params):
        """
        Apply color sensor options from a dict.

        Supported keys (rs.option names):
            enable_auto_exposure, exposure, enable_auto_white_balance,
            white_balance, brightness, contrast, saturation, sharpness,
            gain, gamma, hue, backlight_compensation
        """
        if not params:
            return
        for key, value in params.items():
            try:
                self.color_sensor.set_option(getattr(rs.option, key), value)
            except Exception as e:
                print(f"[realsense] color param '{key}' failed: {e}")

    def _apply_depth_params(self, depth_params):
        """
        Apply depth sensor options from a dict.

        Supported keys (rs.option names):
            visual_preset   — 0=Custom 1=Default 2=Hand 3=HighAccuracy
                              4=HighDensity 5=MediumDensity
            laser_power     — 0–360 mW (D415)
            enable_auto_exposure, exposure, gain
        """
        if not depth_params or self.depth_sensor is None:
            return
        # Apply visual_preset first so individual overrides take effect afterwards
        if 'visual_preset' in depth_params:
            try:
                self.depth_sensor.set_option(rs.option.visual_preset,
                                              depth_params['visual_preset'])
            except Exception as e:
                print(f"[realsense] depth param 'visual_preset' failed: {e}")
        for key, value in depth_params.items():
            if key == 'visual_preset':
                continue  # already applied above
            try:
                self.depth_sensor.set_option(getattr(rs.option, key), value)
            except Exception as e:
                print(f"[realsense] depth param '{key}' failed: {e}")

    # Backward-compatible alias used by older configs
    def set_parameters(self, params):
        self._apply_color_params(params)

    # ── image capture ─────────────────────────────────────────────────────────

    def _get_image(self):
        while True:
            try:
                frames = self.pipeline.wait_for_frames()
                break
            except RuntimeError as e:
                print(e)

        # IR stereo capture before alignment (same crop/flip as RGB)
        if self.save_ir:
            ir_left = np.asanyarray(frames.get_infrared_frame(1).get_data())
            ir_right = np.asanyarray(frames.get_infrared_frame(2).get_data())
            if self.flipped_cam:
                ir_left = cv2.flip(ir_left, -1)
                ir_right = cv2.flip(ir_right, -1)
            self.last_ir_left = self._crop_and_resize(ir_left)
            self.last_ir_right = self._crop_and_resize(ir_right)

        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        if self.flipped_cam:
            color_image = cv2.flip(color_image, -1)
        if self.img_type == 'rgb':
            return color_image, None

        depth_frame = aligned_frames.get_depth_frame()
        raw = np.asanyarray(depth_frame.get_data())
        raw[raw == 65535] = 0   # 65535 = invalid sentinel in D4xx — zero before scaling
        depth_image = raw.astype(np.float32) * self.depth_scale
        if self.flipped_cam:
            depth_image = cv2.flip(depth_image, -1)
        return color_image, depth_image

    # ── intrinsics / calibration ──────────────────────────────────────────────

    def get_intrinsics(self):
        color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        intr = color_profile.get_intrinsics()

        fx, fy = intr.fx, intr.fy
        cx, cy = intr.ppx, intr.ppy
        width, height = intr.width, intr.height

        if self.flipped_cam:
            cx = width - cx
            cy = height - cy

        if self.crop_coords is not None:
            c = self.crop_coords
            cx -= c[2]
            cy -= c[0]
            width  = c[3] - c[2]
            height = c[1] - c[0]

        if self.resize_resolution is not None:
            target_w, target_h = self.resize_resolution
            fx *= target_w / width
            fy *= target_h / height
            cx *= target_w / width
            cy *= target_h / height
            width, height = target_w, target_h

        return dict(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy,
                    crop_coords=self.crop_coords,
                    resize_resolution=self.resize_resolution,
                    dist_coeffs=self.get_dist_coeffs())

    def get_projection_matrix(self):
        i = self.get_intrinsics()
        return np.array([[i['fx'], 0, i['cx'], 0],
                         [0, i['fy'], i['cy'], 0],
                         [0, 0, 1, 0]])

    def get_camera_matrix(self):
        i = self.get_intrinsics()
        return np.array([[i['fx'], 0, i['cx']],
                         [0, i['fy'], i['cy']],
                         [0, 0, 1]])

    def get_dist_coeffs(self):
        color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        return np.array(color_profile.get_intrinsics().coeffs)

    def get_ir_intrinsics(self):
        ir_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.infrared, 1))
        intr = ir_profile.get_intrinsics()
        fx, fy = intr.fx, intr.fy
        cx, cy = intr.ppx, intr.ppy
        width, height = intr.width, intr.height
        if self.flipped_cam:
            cx = width - cx
            cy = height - cy
        if self.crop_coords is not None:
            c = self.crop_coords
            cx -= c[2]
            cy -= c[0]
            width = c[3] - c[2]
            height = c[1] - c[0]
        return dict(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)

    def get_stereo_baseline(self):
        ir1 = rs.video_stream_profile(self.profile.get_stream(rs.stream.infrared, 1))
        ir2 = rs.video_stream_profile(self.profile.get_stream(rs.stream.infrared, 2))
        return abs(ir1.get_extrinsics_to(ir2).translation[0])


@hydra.main(config_path="../../conf/cams", config_name="camera_manager.yaml")
def test_cam(cam_cfg):
    cam = hydra.utils.instantiate(cam_cfg.gripper_cam)
    rgb, depth = cam.get_image()
    print(cam.get_intrinsics())
    while True:
        rgb, depth = cam.get_image()
        cv2.imshow("rgb", rgb[:, :, ::-1])
        if depth is not None:
            cv2.imshow("depth", depth)
        cv2.waitKey(1)


if __name__ == "__main__":
    test_cam()

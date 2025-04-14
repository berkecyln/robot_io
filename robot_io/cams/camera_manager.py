import signal
import sys
import time
from functools import partial

import cv2
import hydra
import numpy as np

from robot_io.cams.threaded_camera import ThreadedCamera
from robot_io.utils.utils import upscale


def destroy_on_signal(self, sig, frame):
    # Stop streaming
    print(f"Received signal {signal.Signals(sig).name}. Exit cameras.")
    if isinstance(self.gripper_cam, ThreadedCamera):
        self.gripper_cam._camera_thread.flag_exit = True
        self.gripper_cam._camera_thread.join()
    if isinstance(self.static_cam, ThreadedCamera):
        self.static_cam._camera_thread.flag_exit = True
        self.static_cam._camera_thread.join()
    print("done")
    sys.exit()


class CameraManager:
    """
    Class for handling different cameras
    """
    def __init__(self, use_gripper_cam, use_static_cam, static_cam_count, gripper_cam, static_cam, threaded_cameras, robot_name=None):
        self.gripper_cam = None
        self.static_cam = None
        self.use_gripper_cam = use_gripper_cam
        self.use_static_cam = use_static_cam
        self.static_cam_count = static_cam_count
        self.threaded_cameras = threaded_cameras
        if use_static_cam:
            self.static_cam =[]
                
            if threaded_cameras:
                for i in range(static_cam_count):
                    static_cam_copy = static_cam.copy()
                    static_cam_copy['device'] = i
                    self.static_cam.append(ThreadedCamera(static_cam_copy))
                    print("connected to device: ", i)
                    # sleep to avoid error
                    #time.sleep(2)
            else:
                for i in range(static_cam_count):
                    static_cam_copy = static_cam.copy()
                    static_cam_copy['device'] = i
                    self.static_cam.append(hydra.utils.instantiate(static_cam_copy))

            if self.static_cam_count <= 1:
                self.static_cam = self.static_cam[0]
            else:
                self.static_cam = tuple(self.static_cam)

        if use_gripper_cam:
            if threaded_cameras:
                self.gripper_cam = ThreadedCamera(gripper_cam)
            else:
                self.gripper_cam = hydra.utils.instantiate(gripper_cam)


        self.obs = None
        self.robot_name = robot_name
        if robot_name is not None:
            self.save_calibration()
        signal.signal(signal.SIGINT, partial(destroy_on_signal, self))

    def get_images(self):
        obs = {}
        if self.gripper_cam is not None:
            rgb_gripper, depth_gripper = self.gripper_cam.get_image()
            obs['rgb_gripper'] = rgb_gripper
            obs['depth_gripper'] = depth_gripper
        if self.static_cam is not None:
        
            if self.static_cam_count > 1:
                rgb = tuple()
                depth = tuple()
                #for cam in self.static_cam:
                #    rgb_cam, depth_cam = cam.get_image()
                #    rgb += (rgb_cam,)
                #    depth += (depth_cam,)
                
                # if self.threaded_cameras:
                #     print("cam: ", self.static_cam[0]._camera_thread.camera.serial_number)
                # else:
                #     print("cam: ", self.static_cam[0].serial_number)

                obs[f'rgb_static'] = rgb
                obs[f'depth_static'] = depth
                for cam_i, cam in enumerate(self.static_cam):
                    obs[f'rgb_static_{cam_i}'] = cam.get_image()[0]
                    obs[f'depth_static_{cam_i}'] = cam.get_image()[1]

                # get the norm l1 distance between the first and the second camera in rgb_static
                # l1_distance  = np.mean(np.linalg.norm(obs[f'rgb_static'][0] - obs[f'rgb_static'][1], axis = -1))

                # print(f"l1 distance between rgb_static_0 and rgb_static_1 is {l1_distance}")

                #assert l1_distance > 30, f"l1 distance between rgb_static_0 and rgb_static_1 is {l1_distance}"


            else:
                rgb, depth = self.static_cam.get_image()
                obs[f'rgb_static'] = rgb
                obs[f'depth_static'] = depth


        self.obs = obs
        return obs

    def save_calibration(self):
        camera_info = {}
        if self.gripper_cam is not None:
            camera_info["gripper_extrinsic_calibration"] = self.gripper_cam.get_extrinsic_calibration(self.robot_name)
            camera_info["gripper_intrinsics"] = self.gripper_cam.get_intrinsics()
        if self.static_cam is not None:
            if self.static_cam_count > 1:
                camera_info["static_extrinsic_calibration"] = tuple()
                camera_info["static_intrinsics"] = tuple()
                for cam in self.static_cam:
                    camera_info["static_extrinsic_calibration"] += (cam.get_extrinsic_calibration(self.robot_name),)
                    camera_info["static_intrinsics"] += (cam.get_intrinsics(),)
            else:
                camera_info["static_extrinsic_calibration"] = self.static_cam.get_extrinsic_calibration(self.robot_name)
                camera_info["static_intrinsics"] = self.static_cam.get_intrinsics()
        if len(camera_info):
            np.savez("camera_info.npz", **camera_info)

    def normalize_depth(self, img):
        img_mask = img == 0
        # we do not take the max because of outliers (wrong measurements)
        istats = (np.min(img[img > 0]), np.percentile(img, 95))
        imrange = (np.clip(img.astype("float32"), istats[0], istats[1]) - istats[0]) / (istats[1] - istats[0])
        imrange[img_mask] = 0
        imrange = 255.0 * imrange
        imsz = imrange.shape
        nchan = 1
        if len(imsz) == 3:
            nchan = imsz[2]
        imgcanvas = np.zeros((imsz[0], imsz[1], nchan), dtype="uint8")
        imgcanvas[0: imsz[0], 0: imsz[1]] = imrange.reshape((imsz[0], imsz[1], nchan))
        return imgcanvas

    def render(self):
        if "rgb_gripper" in self.obs:
            cv2.imshow("rgb_gripper", upscale(self.obs["rgb_gripper"][:, :, ::-1]))
        if "depth_gripper" in self.obs:
            depth_img_gripper = self.normalize_depth(self.obs["depth_gripper"])
            depth_img_gripper = cv2.applyColorMap(depth_img_gripper, cv2.COLORMAP_JET)
            cv2.imshow("depth_gripper", upscale(depth_img_gripper))
        #if "rgb_static" in self.obs:
        if self.static_cam is not None:
            if self.static_cam_count > 1:
                #show them side by side in one window
                #rgb_static = np.concatenate(self.obs["rgb_static"], axis=1)
                for cam_i, cam in enumerate(self.static_cam):
                    cv2.imshow(f"rgb_static_{cam_i}", upscale(self.obs[f"rgb_static_{cam_i}"][:, :, ::-1]))

            else:
                cv2.imshow("rgb_static", upscale(self.obs["rgb_static"][:, :, ::-1]))
        
        cv2.waitKey(1)
    
    def render_numpy(self, camera_name):
        if camera_name == "rgb_gripper":
            if "rgb_gripper" in self.obs:
                render = self.obs["rgb_gripper"]
            else:
                render = None
        elif camera_name == "depth_gripper":
            if "depth_gripper" in self.obs:
                render = self.normalize_depth(self.obs["depth_gripper"])
                render = cv2.applyColorMap(render, cv2.COLORMAP_JET)
            else:
                render = None
        elif camera_name == "rgb_static":        
            if self.static_cam is not None:
                if self.static_cam_count > 1:
                    render = []
                    for cam_i, cam in enumerate(self.static_cam):
                        render.append(self.obs[f"rgb_static_{cam_i}"])
                    render = tuple(render)
                else:
                    render = self.obs["rgb_static"]
                    # Resize tp 64x64
                    # render = cv2.resize(render, (64, 64), interpolation=cv2.INTER_AREA)
                    
        return render
        

@hydra.main(config_path="../conf/cams", config_name="camera_manager.yaml")
def main(cfg):
    print(cfg)
    cam_manager = hydra.utils.instantiate(cfg)
    while True:
        cam_manager.get_images()
        cam_manager.render()
        time.sleep(0.05)

if __name__ == "__main__":
    main()

# if __name__ == "__main__":
#     from omegaconf import OmegaConf
#     hydra.initialize("../conf/cams")
#     cfg = hydra.compose("camera_manager.yaml")
#     print(cfg)
#     cam_manager = hydra.utils.instantiate(cfg)
#     while True:
#         cam_manager.get_images()
#         cam_manager.render()
#         time.sleep(0.05)
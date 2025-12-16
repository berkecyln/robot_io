# Camera Calibration

### Kinect Azure Camera Intrinsics
- Clone the Kinect Azure official SDK.
    ```bash
    git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
    cd Azure-Kinect-Sensor-SDK
    ```
- Build the Kinect Azure official SDK. Before building, you need to modify `Azure-Kinect-Sensor-SDK/examples/calibration/main.cpp` Line 62 to `deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;` or `deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;`(depending on the resolution of the intrinsics you need) and Line 76 to `auto calib = calibration.color_camera_calibration;`.
    ```bash
    # under Azure-Kinect-Sensor-SDK folder
    mkdir build && cd build && cmake ..
    make
    ```
- Run the `./calibration_info`. The output contains the camera parameters.
    ```
    cd bin
    ./calibration_info
    ```
- Create a `.yaml` file under the `robot_io/cams/kinect4/config/{cam_serial_number}.yaml` and fill in intrinsics number following template of the existing files there. You can ignore `crop_coords` first. The serial number of the camera can be obtained with `k4aviewer` command.
### Static Camera Hand-Eye Calibration
- If necessary, get the dist coeff parameters through the camera calibration script in the [Azure Kinect SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/calibration)
- Get the intrinsics through the section above
- Stick the marker to the robot end-effector
- If applicable, comment out `crop_coords` in `robot_io/cams/kinect4/config/{serial-number}.yaml` 
- Run `python robot_io/calibration/static_cam_calibration.py --config-name=[panda_calibrate_static_cam|iiwa_calibrate_static_cam] record_new_poses=True`. Use VR controller to collect more than 30 valid poses. When the marker can be detected from the camera, the marker will be visualized and the pose is valid. Then you need to press the top middle button on the VR controller to record this pose.
- Run the above program the second time, but with `record_new_poses=False`. The command is `python robot_io/calibration/static_cam_calibration.py --config-name=[panda_calibrate_static_cam|iiwa_calibrate_static_cam] record_new_poses=False`. The robot will automatically move to recorded poses and start to compute the extrinsics.
- If you set `record_new_poses: true`, then you should use vr controller to move the robot. Press the record button (on top) to sample poses, and hold record button to finish the pose sampling.
- If you set `record_new_poses: false`, the robot will move to the previously recorded poses and captures the marker pose. This option is helpful in case the camera is moved slightly.


### Gripper Camera
- Place Aruco Marker in front of robot
- Run `python robot_io/calibration/gripper_cam_calibration.py --config-name=[panda_calibrate_gripper_cam|kuka_calibrate_gripper_cam]`
- If you want to visualize calibration in 3d point cloud
    - download open3d via 
    ```
    conda install -c open3d-admin open3d
    ```
    - change the file name in `control_gripper_cam_calibration.py` then run 
    ```
    python control_gripper_cam_calibration.py
    ```
------------------

### Teleoperation
- For software and hardware setup and teleop instructions, please check the documentation [Teleoperation](teleoperation.md)
- Make sure to set workspace limits appropriately in `robot_io/conf/robot/<robot_interface.yaml>`
- Set the `save_dir` in  `robot_io/conf/[panda_teleop.yaml|kuka_teleop.yaml]` to specify the data saving directory.
- Run the following command to start the teleop:
    ```
    $ python robot_io/examples/teleop_robot.py --config-name=[panda_teleop|kuka_teleop]
    ```
- After finishing the data collection, you need to preprocess the data. This step helps clean the data with tracking error and convert the data structure to [CALVIN format](https://github.com/mees/calvin/blob/main/dataset/README.md). Run the following command to preprocess the data. You need to set the `dataset_root` and `output_dir` accordingly in `robot_io/conf/preprocess_data.yaml`. The data structure after preprocessing is documented in [Data Structure](teleop_data_structure.md).
    ```
    $ python robot_io/examples/preprocess_data.py
    ```

- If you wish to downsample the preprocessed data by half, say from 30 FPS (default value in `panda_teleop_yaml|kuka_teleop.yaml`) to 15 FPS, you can run the following script:
    ```
    $ python robot_io/examples/downsample_data.py --dataset_root=/path/to/preprocessed/data --output_dir=/path/to/new/output/dir"
    ```
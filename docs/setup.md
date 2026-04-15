# Environment

Because `robotio` combines older and newer repositories, some system requirements are not natively available on Ubuntu 24.04. This can lead to compatibility and compilation errors. To prevent this, we recommend creating a dedicated Conda environment to install all necessary tools cleanly.

Tested versions for environment: `python 3.8.20, cmake 3.22.1, numpy 1.24.4, libfranka 0.9.2, pybind11 3.0.1, eigen 3.4.0, cython 3.0.11, scipy 1.10.1 and 0.58.1 `

```
conda create -n robotio python=3.8 cmake=3.22.1 numpy libfranka pybind11 eigen cython scipy opencv -c conda-forge -y
conda activate robotio
```
Please ensure all installations are performed inside this active Conda environment to prevent permission or build errors.

To make files work easily please add robot_io to Python path permanently:
```
echo 'export PYTHONPATH=<PATH_TO_ROBOT_IO>:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```
Replace `<PATH_TO_ROBOT_IO>` with your actual robot_io directory path

# Cameras

### Azure Kinect (Kinect 4)
- On Ubuntu 18 install azure kinect SDK with apt
- On Ubuntu 20 download libk4a*(-dev) and libk4abt*(-dev) from https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/
  and k4atools from https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools \
  Install with `sudo dpkg -i`

- Install pyk4a and opencv-python in your Python env with `pip install pyk4a opencv-python`

- For default usage, start `$ python robot_io/cams/kinect4/kinect4.py`

### Multiple Kinect Azure
- When multiple kinect azures are in use, we need to set the USB bandwidth to a higher value.
- Edit `/etc/default/grub`, replacing the line that says `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"` with `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=32"` for two Kinect Azure. Set the value `32` to higher `64` for three Kinect Azure.
- Run `sudo update-grub`
- Restart the computer

### RealSense SR300/SR305

First follow installation instructions for librealsense2 [here](https://github.com/IntelRealSense/librealsense)

Before running `realsense.py` please make sure you compiled [marker detector](#marker-detector)
```
pip install pyrealsense2
# Make sure you are in robot_io project then run
python robot_io/cams/realsense/realsense.py  # to test
```

### Framos D435e
```
groups | grep video  # user must be in video group, otherwise ask Michael K.
file /usr/src/librealsense2  # (see below a)
diff misc/framos_setup_files/setup.py /usr/src/librealsense2/setup.py
cp -r /usr/src/librealsense2 .
cd librealsense2
pip uninstall pyrealsense2
pip install -e .

cd ../robot_io/cams/realsense
python realsense.py  # test script
```
- a) If `/usr/src/librealsense2` does not exist, download FRAMOS software package from https://www.framos.com/en/industrial-depth-cameras#downloads. Follow installation instructions, make sure to use local admin user (e.g. xam2) to install (file system may NOT be network mounted). Alternatively, `wget http://hulc2.cs.uni-freiburg.de/downloads/librealsense2.zip`.
- b) Use Ethernet sockets on the ceiling for PoE.


# Robots

## Franka Emika Panda

**Connection:**

By default robot ip is: `172.16.0.2`

To be able to communicate with robot you must be in same network with robot so please create a ne profile in network as:

IPv4 Method: Manual \
Address: 172.16.0.X (put so ething different than 2 to X)\
Netmask: 255.255.255.0\
Gateway: Leave empty

After creating this profile you need to seelct this profile when you will use te robot via `https://172.16.0.2/desk/`

### IK fast
IK fast is an analytic IK solver. In order to use IK fast, first install `ikfast-pybind`:

Tested commit version: `b24db7b`
```
git clone --recursive https://github.com/yijiangh/ikfast_pybind
cd ikfast_pybind
# copy panda IK solution .cpp and .h to ikfast_pybind
cp ../robot_io/misc/ik_fast_files/ikfast.h ./src/franka_panda/
cp ../robot_io/misc/ik_fast_files/ikfast0x10000049.Transform6D.0_1_2_3_4_5_f6.cpp ./src/franka_panda/
pip install .
```
For creating different IK solutions (e.g. in case of a different gripper) please refer to:
`http://docs.ros.org/en/kinetic/api/framefab_irb6600_support/html/doc/ikfast_tutorial.html`

### Franky

To set up Franky, first ensure that you are using a real-time kernel and that the executing user has permission to use real-time priorities.

To check if you have a real-time kernel, run the following in the terminal: `uname -a`

If you see something like `Linux [PCNAME] ... PREEMPT_RT ...`, you have the real-time kernel already. You can then check if your user has permission to use this kernel via `ulimit -r`. If you see **99**, you have the correct permissions.

If you do not see `PREEMPT_RT` or `99`, you need to set up the real-time kernel and user group. For detailed setup instructions, please see [Real-time Kernel Setup](realtime_kernel_setup.md).

**Note:** Franky requires `eigen`, `libfranka`, and `pybind11`. If you followed the [Environment](#environment) setup at the start of this guide, these are already installed in your Conda environment.

Tested commit version: `614aaf75`, Tested kernel: `6.8.1-1015-realtime`

```
# Clone with submodules included
git clone --recurse-submodules https://github.com/TimSchneider42/franky.git
cd franky

# Link Conda libraries so the builder can find libfranka
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

# Install in editable mode
pip install -e .
```
If you encounter with any issue please refer this [documentation](https://github.com/TimSchneider42/franky?tab=readme-ov-file#-%EF%B8%8F-setup).

# Input Devices

## SpaceMouse
```
sudo apt install libspnav-dev spacenavd
conda activate robot
pip install spnav
```

Next test if it works, some common pitfalls are:
1. Turn on SpaceMouse in the back
2. May not work while charging.
3. Wireless range is quite limited.
4. Comment the following two lines in `site-packages/spnav/__init__.py`
```
#pythonapi.PyCObject_AsVoidPtr.restype = c_void_p
#pythonapi.PyCObject_AsVoidPtr.argtypes = [py_object]
```

To test execute the following program. When moving the mouse you should
see numbers scrolling by.
```
python robot_io/input_devices/space_mouse.py
```


## GELLO

Low-cost leader arm teleoperation for Franka Panda using Dynamixel servos.

See [GELLO Setup](gello_setup.md) for full setup and usage instructions.

## VR Teleoperation

### Install Steam and SteamVR
- In terminal run `$ steam`, it will start downloading an update and create a `.steam` folder in your home directory.
- If you get an error, try deleting the steam folders on your home directory with `rm -rf .local/share/Steam/` and `rm -rf .steam`
- In Steam, create user account or use existing account.
- Install SteamVR
  - If on `pickup` click `Steam -> Settings -> Downloads -> Steam Library Folders -> Add Library Folder -> /media/hdd/SteamLibrary` to add the existing installation of SteamVR to your Steam account
  - Otherwise download SteamVR
- Restart Steam
- Connect and turn on HTC VIVE
- Launch `Library -> SteamVR` (if not shown, check `[] Tools` box)
- If SteamVR throws an  `Error: setcap of vrcompositor-launcher failed`, run `$ sudo setcap CAP_SYS_NICE+ep /media/hdd/SteamLibrary/steamapps/common/SteamVR/bin/linux64/vrcompositor-launcher`
- Make sure Headset and controller are correctly detected
- Go through VR setup procedure (standing is sufficient)

### Install Bullet
```
$ git clone https://github.com/bulletphysics/bullet3.git
$ cd bullet3

# Optional: patch bullet for selecting correct rendering device
# (only relevant when using EGL and multi-gpu training)
$ wget https://raw.githubusercontent.com/BlGene/bullet3/egl_remove_works/examples/OpenGLWindow/EGLOpenGLWindow.cpp -O examples/OpenGLWindow/EGLOpenGLWindow

# For building Bullet for VR  add -DUSE_OPENVR=ON to line 8 of build_cmake_pybullet_double.sh
# Run
$ ./build_cmake_pybullet_double.sh

$ pip install numpy  # important to have numpy installed before installing bullet
$ pip install -e .  # effectively this is building bullet a second time, but importing is easier when installing with pip

# add alias to your bashrc
alias bullet_vr="~/.steam/steam/ubuntu12_32/steam-runtime/run.sh </PATH/TO/BULLET/>bullet3/build_cmake/examples/SharedMemory/App_PhysicsServer_SharedMemory_VR"

# to test VR control
# make sure SteamVR is started
$ bullet_vr
$ cd <PATH/TO/ROBOTIO>/robot_io/robot_io/control
$ python teleop_robot.py
```

Robot Teleop instructions:
1. Push dead-man switch (riffled grip right)
2. Move controller in direction robot is pointing (twoards window)
3. Push top middle button (with three lines)
4. Robot should reset to home position
5. Robot only moves with dead-man-switch activated

### Marker Detector

```
$ cd robot_io/marker_detection/apriltag_detection
$ export CPLUS_INCLUDE_PATH=$CONDA_PREFIX/include/opencv4:$CONDA_PREFIX/include:$CPLUS_INCLUDE_PATH
$ export LIBRARY_PATH=$CONDA_PREFIX/lib:$LIBRARY_PATH
$ export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
$ python setupBatch.py build_ext --inplace
```
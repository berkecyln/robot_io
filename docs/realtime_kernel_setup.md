# Real-Time Kernel Setup (Ubuntu 24.04)

Franka robots require a strict 1kHz communication loop. To prevent `communication_constraints_violation` errors, you must install a Real-Time (`PREEMPT_RT`) kernel and configure user permissions.

> **Note:** You need `sudo` privileges to complete this setup.

> **Note for CUDA:** The real-time kernel is not compatible with standard NVIDIA drivers by default. If you want CUDA, please see the the [CUDA Setup Section](https://www.google.com/search?q=%233-cuda-setup-for-real-time-kernel) after finisihing with real-time kernel setup.

## 1\. Install the Kernel

On Ubuntu 24.04, the real-time kernel is available in the standard `universe` repository.

```
# Enable the universe repository
sudo add-apt-repository universe

# Update package lists
sudo apt update

# Install the Real-Time kernel
sudo apt install ubuntu-realtime
```

Once installed, reboot your computer. Open a terminal and run: `uname -a`

If the output contains `PREEMPT_RT`, the installation was successful. Proceed to [Section 2](https://www.google.com/search?q=%232-configure-user-permissions).

If the this method fails, you can [build the kernel from source](https://frankarobotics.github.io/docs/libfranka/docs/installation_linux.html#setting-up-the-real-time-kernel) or enable it via [Ubuntu Pro](https://ubuntu.com/real-time).

## 2\. Configure User Permissions

Standard users are restricted from setting real-time priorities. You must create a specific user group and adjust system limits.

Create the Group

```bash
sudo groupadd realtime
sudo usermod -a -G realtime $(whoami)
```

Open the limits configuration file to edit system limits:

```bash
sudo vim /etc/security/limits.conf
```

Paste the following lines at the very bottom of the file:

```text
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```

Log out and log back in (or reboot) then run `ulimit -r`

If the output is `99`, your user permissions are correctly configured.

## 3\. CUDA Setup for Real-Time Kernel

Standard NVIDIA drivers often fail to load on a real-time kernel. If `nvidia-smi` fails after rebooting, you must reinstall the drivers with a specific override flag.

To make the Real-time kernel and CUDA compatible, you must set the environment variable `IGNORE_PREEMPT_RT_PRESENCE=1` during installation and all subsequent updates.

For detailed steps, please refer to this [installation script](https://github.com/TimSchneider42/franky/blob/master/tools/install_cuda_realtime.bash) and [Franky documentation on CUDA](https://github.com/TimSchneider42/franky?tab=readme-ov-file#can-i-use-cuda-jointly-with-franky) prepared by Tim Schneider.
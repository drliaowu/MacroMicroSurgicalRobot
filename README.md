# A Low-Cost Teleoperable Surgical Robot with a Macro-Micro Structure for Open-Source Research
ROS package providing simultaneous teleoperation of a custom micro-manipulator and a Universal Robots UR5e robotic arm via a 3D Systems Touch haptic stylus.

## Installation

1. Install Ubuntu 20.04 LTS
4. Apply a patch to install a real-time kernel on your Linux system. The exact version used to develop the project was 5.15.96-rt61. Instructions can be found [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md).
5. Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).
6. Install [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
7. Create a catkin workspace.
```
cd ~
mkdir catkin_ws && cd catkin_ws
```
10. Install the [Universal Robots ROS driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) in your catkin workspace and follow its setup instructions up to the end of extracting calibration information.
11. Install the [Geomagic Touch driver](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers/tree/hydro-devel). Note: The installation instructions on the GitHub page are outdated, working instructions as of February 2023 are:
    1. Install the driver's dependencies:
    ```
    sudo apt-get install --no-install-recommends freeglut3-dev g++ libdrm-dev\
    libexpat1-dev libglw1-mesa libglw1-mesa-dev libmotif-dev libncurses5-dev\
    libraw1394-dev libx11-dev libxdamage-dev libxext-dev libxt-dev libxxf86vm-dev\
    tcsh unzip x11proto-dri2-dev x11proto-gl-dev
    ```
    2. Install the [OpenHaptics SDK](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US). If the page is unavailable, search for "openhaptics linux sdk" and install the most recent version available.
    3. Install the compatible Touch driver version from the same page.
    4. Follow the [installation instructions](https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/Installation+Instructions_2022.pdf). If the page is unavailable, follow the instructions found on the most recent version of the openhaptics Linux SDK install page used in step 2.
    5. For a 64-bit system, create symbolic links by executing the following commands:
    ```
    sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11.0.1 /usr/lib/libraw1394.so.8
    sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
    sudo ln -s /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0
    sudo ln -s /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0 
    ```
Note: Run `./Touch_Setup` and `./Touch_Diagnostic` found in the touch driver directory, with the Touch device connected, to verify that Touch communication is functional.
12. Clone this repository into your catkin workspace.
13. Build and source your catkin workspace using `catkin build`.

# ROS Startup Commands
### Touch Stylus
1. Start ROS: `roscore`

2. Add permissions for touch USB device: `sudo chmod 777 /dev/ttyACM0`

3. Start Touch Driver: `roslaunch omni_common omni_state.launch`

### UR5e
1. Start driver: `roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100  kinematics_config:=/home/lachlan/lab_ur5e_1_calibration.yaml`

2. Start LS_ROS_CONTROL program on UR5e Teach Pendant

3. Start control node: `rosrun ls_thesis ur5e_control`

### Micro Module
1. Add permissions for Arduino device: `sudo chmod 777 /dev/ttyACM1`

2. Start serial node: `rosrun rosserial_python serial_node.py /dev/ttyACM1`

3. Start control node: `rosrun ls_thesis micro_module_control`

NOTE: Be sure to manually disable power to servo motors via the switch on the micro module before unplugging the control USB cable

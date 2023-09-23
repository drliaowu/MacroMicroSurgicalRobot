# Towards a Low-Cost Teleoperable Macro-Micro Robotic System for Minimally Invasive Surgery
ROS package providing simultaneous teleoperation of a custom micro-manipulator and a Universal Robots UR5e robotic arm via a 3D Systems Touch haptic stylus.

## Setup Steps

1. Obtain an installation of Ubuntu 20.04 LTS
4. Install a real-time kernel on your Ubuntu installation. The exact version used to develop the project was 5.15.96-rt61. Instructions can be found [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md).
5. Setup [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on your installation.
6. Install [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
7. Create a catkin workspace.
```
cd ~
mkdir catkin_ws && cd catkin_ws
```
10. Install the [Universal Robots ROS driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) in your catkin workspace and follow its setup instructions up to the end of extracting calibration information.
11. Install the [Geomagic Touch driver](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers/tree/hydro-devel). Note: The installation instructions on the GitHub page are outdated, **DO NO FOLLOW THEM**, working instructions as of the creation of this repository are as follows:
    1. Install the driver's dependencies:
    ```
    sudo apt-get install --no-install-recommends freeglut3-dev g++ libdrm-dev\
    libexpat1-dev libglw1-mesa libglw1-mesa-dev libmotif-dev libncurses5-dev\
    libraw1394-dev libx11-dev libxdamage-dev libxext-dev libxt-dev libxxf86vm-dev\
    tcsh unzip x11proto-dri2-dev x11proto-gl-dev
    ```
    2. Install the [openhaptics SDK](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US). If link is dead google "openhaptics linux sdk" and install the most recent version.
    3. Install the Touch driver from the same page. It must be from the same page for compatability.
    4. Follow the [installation instructions](https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/Installation+Instructions_2022.pdf). If link is dead follow the pdf on the most recent page of the openhaptics linux sdk install page.
    5. For a 64-bit system, create symbolic links by executing the following commands:
    ```
    sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11.0.1 /usr/lib/libraw1394.so.8
    sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
    sudo ln -s /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0
    sudo ln -s /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0 
    ```
12. Clone this repository into your catkin workspace.
13. Build and source your catkin workspace. (use catkin build not catkin_make as the catkin tool is installed)
14. Run one of the launch files to start, like ```roslaunch ur_to_touch_haptic_teleoperation ur_touch_haptic_teleop.launch```

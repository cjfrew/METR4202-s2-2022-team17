# METR4202 Ximea Camera Setup
# Week 9 - Practical
## Introduction
This tutorial is provided as guidelines for setting up the given Ximea cameras with ROS.

There will be three steps to this tutorial:
- Step 1: Install the ROS Ximea Package
- Step 2: Camera Calibration
- Step 3: Setup the ArUco Tag Detection Library

**Note:**
- **This can be done on any Ubuntu 20.04 system with IO/USB access.**
- **You should run this natively, on the RPi4 or on a dual booted machine. (Not a VM)**
- **You will need to see the GUI for the calibration part of the tutorial.**
- **Do not login with root, as this will affect your permissions.**

After following the RPi4 setup, clone this to your `src` folder in your ROS workspace.
```console
cd ~/catkin_ws/src
```
```console
git clone https://github.com/UQ-METR4202/metr4202_ximea_ros.git
```
Then return to your workspace
```console
cd ~/catkin_ws/
```

# Step 1: Install the ROS Ximea Package

## Building the Packages
You may need to install the dependency ```vision_msgs```
```console
sudo apt install ros-noetic-vision-msgs
```
- Run `catkin_make` or `catkin build` in your workspace to build the packages.



## Testing the XIMEA Camera

- You need to run the following command after each boot to disable the USB memory limits
```console
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

Run the XIMEA ROS camera node

Before this, nagivate to the src folder and edit ximea_demo to use your serial number.
```console
rosrun ximea_ros ximea_demo
```
You can check the output using `rqt_image_view`
```console
rosrun rqt_image_view rqt_image_view
```

# Step 2: Camera Calibration
If it isn't installed you can install the camera calibration ROS package
```console
sudo apt install ros-noetic-camera-calibration
```
You can run the `ximea_demo` node, if it isn't already running.
```console
rosrun ximea_ros ximea_demo
```

```console
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.024 image:=/ximea_ros/ximea_XXXXXXXX/image_raw camera:=/ximea_ros/ximea_XXXXXXXX
```
Use the 9x6 grid to calibrate the camera. You may need to change the brightness on the GUI for this, as well as adjust your aperture.

Calibration lines should come up, and the program should start collecting frames for calibration.

Once enough bars (x,y,skew,size) fill up, the 'calibrate' button should light up, and the camera should start calibration. (This may take a while)

After the calibration, click on save and commit to save or commit the files.
It should save into `/tmp`.

Next you need to copy this into your home directory for extraction.

You will need to extract this folder, then rename the `.yaml` file to `ximea_XXXXXXXX.yaml`.

Finally, edit the `camera_name` inside this file, and move it into `~/.ros/camera_info/`

You will need to restart the ximea camera node for the next part.

# Step 3: Setup the ArUco Tag Detection Library
```console
roslaunch ximea_ros ximea_aruco.launch serial:=XXXXXXXX
```

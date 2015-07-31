# RealSense
ROS driver for RealSense depth camera

## Overview
This code is based on ROS [usb_cam](http://wiki.ros.org/usb_cam) node, which code was adapted to communicte with Intel
RealSense RGB-D camera. This code was tested on
- [ROS Hydro](http://wiki.ros.org/hydro), Ubuntu 12.04
- [ROS Jade](http://wiki.ros.org/jade), Ubuntu 15.04
  but might work with other configurations s well. 
The node *realsense_cam_node* provides you with following 
- Topics:
  + *~/image_raw* - published depth image in **16UC1** format, millimetres (unsigned int)
  + *~/camera_info* - [CameraInfo](http://wiki.ros.org/image_pipeline/CameraInfo) topic
- Parameters:
  + *video_device* - the device the camera is on: (e.g. "/dev/video0") 
  + *camera_name* - the name of the camera (e.g. "my_realsense_camera") **must be the same as specified in camera
  calibration config** (if used, see launch file).
  + *camera_frame_id* - you must specify coordinate frame for ROS to know how to interpret point coordinates 
  + *camera_info_url* - An *url* to the camera calibration file that will be read by the CameraInfoManager class

## Setting Everything Up
1. This manual assumes you have successfully installed ROS Hydro on Ubuntu (the code was tested on Ubuntu 12.04)
2. Useful ROS HOWTO's: 
  + [ROS Hydro installation](http://wiki.ros.org/hydro/Installation/Ubuntu "Read this to install ROS on your system")
  + [ROS Tutorial](http://wiki.ros.org/ROS/Tutorials "This is a brief ROS tutorial. Helps to understand basic ROS
    concepts")
3. It might be necessary to install some additional libraries to successfully compile the code. I'll maybe write about this one day,
but at the moment you have to look at the compiler errors and set everything up yourself, sorry ;)
No further libraries were required on Ubuntu 15.04 with [sudo apt-get install ros-jade-desktop-full](http://wiki.ros.org/jade/Installation/Ubuntu)
4. After you have created [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) simply copy
downloaded (or directly checkout from the repo) *realsense_cam* package in your *src* folder and *catkin_make* it:
```
        cd ~/catkin_ws/src
        git clone https://github.com/nucobot/RealSense
        cd ../
        catkin_make
```

## Manage The Node
1. *realsense_cam* package has a launch file. You can examine it and use as a template for your own one, or use as it is. 
2. **DO NOT** forget to configure *rgb_devive* and *depth_device* tags at the top of the launch file (it should be something 
like `/dev/video<n>`, where *n* is some number. Run `ls /dev | grep video` to see all video devices.
```
        roscd realsense_cam
        gedit ./launch/realsense_cam.launch
```
2. Run the launch file:
`roslaunch realsense_cam realsense_cam.launch`
3. If the camera crashes after a few seconds, ensure that the USB port supplies enough current.

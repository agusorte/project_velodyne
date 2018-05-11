# project_velodyne


# Overview
This code find the boundining boxes combining yolo and a point cloud for detection of object.

# Code 

important files are 
-src/ProjectVelodyne.cpp and include/ProjectVelodyne.hpp. 


# Installation

dependences
-ROS Kinect
-YOLO ros follow this guide intallation (see the link below):

https://github.com/leggedrobotics/darknet_ros
	
-opencv
-PCL
-boost
-Eigen

# Building

we need to have a catkin space created, follow theses intructions to install catkin (see the link below):

http://wiki.ros.org/catkin/Tutorials/create_a_workspace

In order to install project_velodyne we need to unzip the code.zip file to catkin_ws 

$cp $PATH/project_velodyne ~/catkin_workspace/src
$cd ../

then build catking

$catkin_make -DCMAKE_BUILD_TYPE=Release

Running the ros node:

$roslaunch project_velodyne project_velodyne.launch

# Setting

you can modify camera calibration parameres using the wollofing file

project_velodyne/config/cam_calib.yalm

you can modify topics 

project_velodyne/config/settings.yalm

kitti file

kitty format file is saved in:
project_velodyne/data/kitti_file.txt

# Final comments

Bounding box is creation is not perfect because:
- incomplete pointcloud,
- noisy points clouds
- small delay 

idea to solve this issue is to have a clustering to add the complete pointscloud 

Suggestion or comments email to Agustin Ortega aortega.jim@gmail.com

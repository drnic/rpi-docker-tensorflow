#!/bin/bash

roscore &
sleep 5
rosparam set cv_camera/image_width 320 && rosparam set cv_camera/image_height 240 && rosparam set cv_camera/rate 1
rosrun cv_camera cv_camera_node &
rosrun web_video_server web_video_server &
sleep 5
python object_detection_ROS.py image:=/cv_camera/image_raw

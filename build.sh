#!/bin/bash
cd /home/nvidia/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select kr_mav_msgs kr_tracker_msgs kr_mav_controllers kr_trackers_manager kr_trackers kr_mav_manager rqt_mav_manager mocap_base mocap_vicon kr_betaflight_interface quadrotor_ukf_ros2
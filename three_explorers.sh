#! /bin/bash

rosrun maze_escapers shared_map &
ROS_NAMESPACE=robot_0 rosrun maze_escapers explorer &
ROS_NAMESPACE=robot_1 rosrun maze_escapers explorer &
ROS_NAMESPACE=robot_2 rosrun maze_escapers explorer &
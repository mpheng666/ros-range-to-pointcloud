# ros-range-to-pointcloud

## What is this about

This package converts ROS sensor_msgs::Range to sensor_msgs::PointCloud2 for whatever reason.

## How to install and build

1. `cd ~ && mkdir -p ros_ws/src && cd ~/ros_ws/src`
2. `git clone https://github.com/mpheng666/ros-range-to-pointcloud.git`
3. `cd ~/ros_ws && catkin build` or `cd ~/ros_ws && catkin_make`
4. `cd ~/ros_ws && source ~/ros_ws/devel/setup.bash`

## How to use

1. Edit the params in config/param.yaml
2. Edit the tf.launch in launch/tf.launch

## How to launch

1. If tf is defined in URDF or launch somewhere else
`roslaunch range_to_pointcloud start.launch use_tf:=false`

2. If tf is defined in launch/tf.launch
`roslaunch range_to_pointcloud start.launch use_tf:=true`

3. run `rviz` to visualize it

![alt text](resources/range_to_pointcloud_example.png?raw=true)

#include "range_to_pointcloud/range_to_pointcloud.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_to_pointcloud_node");
    ros::NodeHandle nh("~");

    range_to_pointcloud::RangeToPointCloud range_to_pointcloud(nh, 8);
    range_to_pointcloud.start();

    return 0;
}
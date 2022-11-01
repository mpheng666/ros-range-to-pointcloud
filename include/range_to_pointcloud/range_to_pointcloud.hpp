#ifndef RANGE_TO_POINTCLOUD_HPP_
#define RANGE_TO_POINTCLOUD_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <string>
#include <vector>

#include "pcl_ros/point_cloud.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <boost/range/adaptors.hpp>
namespace range_to_pointcloud {
    class RangeCallBack {
    public:
        RangeCallBack(int index)
            : index_(index)
        {
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            std::string frame = "sonar_" + std::to_string(index_) + "_link";

            try {
                transformStamped_baselink_sonar_ = tfBuffer.lookupTransform(
                "base_link", frame, ros::Time(0), ros::Duration(1.0));
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
            }
        }

        void operator()(const sensor_msgs::Range::ConstPtr& msg) { range_ = *msg; }

        sensor_msgs::Range getRange() const { return range_; }

        geometry_msgs::TransformStamped getTF() const
        {
            return transformStamped_baselink_sonar_;
        }

    private:
        int index_;
        sensor_msgs::Range range_;
        geometry_msgs::TransformStamped transformStamped_baselink_sonar_;
    };

    class RangeToPointCloud {
    public:
        RangeToPointCloud(ros::NodeHandle& nh, int num);
        void start();

    private:
        void loadParams();

        int number_of_sensor_{0};

        ros::NodeHandle nh_;
        ros::NodeHandle nh_g_;
        ros::Timer pub_timer_;
        ros::Publisher pointcloud_pub_;
        std::vector<ros::Subscriber> range_sensors_subs_;
        std::vector<RangeCallBack> range_callbacks_;

        void timerPubCallBack(const ros::TimerEvent& e);
        void populateRangeToPC(const sensor_msgs::Range& range_msg);
    };
} // namespace range_to_pointcloud

#endif
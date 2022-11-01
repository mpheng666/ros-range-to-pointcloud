#ifndef RANGE_TO_POINTCLOUD_HPP_
#define RANGE_TO_POINTCLOUD_HPP_

#include <array>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <vector>

#include "tf/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <tf/transform_listener.h>

#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/PointCloud2.h>

#include <boost/range/adaptors.hpp>

namespace range_to_pointcloud {

    class RangeCallBack {
    public:
        RangeCallBack(int index);
        void operator()(const sensor_msgs::Range::ConstPtr& msg);

        double getUntransformedData();
        std::array<double, 2> getTransformedData();

    private:
        int index_;
        double untransformed_range_data_{0.0};
        std::array<double, 2> transformed_range_data_{0.0, 0.0};

        geometry_msgs::TransformStamped transformStamped_baselink_sonar_;
    };

    class RangeToPointCloud {
    public:
        RangeToPointCloud(ros::NodeHandle& nh);
        void start();

    private:
        void loadParams();

        int number_of_sensor_{0};

        ros::NodeHandle nh_;
        ros::NodeHandle nh_g_;
        ros::Timer pub_timer_;
        ros::Publisher multi_sonars_pub_without_tf;
        ros::Publisher multi_sonars_pub_with_tf;
        ros::Subscriber scan_sub_;
        ros::Publisher pc_pub_;

        std::vector<RangeCallBack> range_callbacks_;
        std::vector<ros::Subscriber> range_sensors_subs_;

        std::vector<double> utf_sonars_;
        std::vector<std::array<double, 2>> tf_sonars_;

        laser_geometry::LaserProjection projector_;
        tf::TransformListener tflistener_;

        void multiSonarsPub(const ros::TimerEvent&);
        void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
    };
} // namespace range_to_pointcloud

#endif
#include "range_to_pointcloud/range_to_pointcloud.hpp"
namespace range_to_pointcloud {

    RangeToPointCloud::RangeToPointCloud(ros::NodeHandle& nh, int num)
        : nh_(nh)
        , number_of_sensor_(num)
        , pub_timer_(nh_.createTimer(
          ros::Duration(0.05), &RangeToPointCloud::timerPubCallBack, this))
        , pointcloud_pub_(
          nh_.advertise<sensor_msgs::PointCloud2>("range_point_cloud", 10))
    {
    }

    void RangeToPointCloud::start()
    {
        // Create rangeCb functor
        for (auto i = 0; i < number_of_sensor_; ++i) {
            range_callbacks_.emplace_back(i + 1);
        }

        // Create range ros subsriber
        range_sensors_subs_.resize(number_of_sensor_);
        for (auto i = 0; i < number_of_sensor_; ++i) {
            std::string topic_name = "sonar_" + std::to_string(i + 1);
            range_sensors_subs_.at(i) = nh_g_.subscribe<sensor_msgs::Range>(
            topic_name, 10, std::ref(range_callbacks_.at(i)));
        }

        while (ros::ok()) {

            for (const auto& range_callback : range_callbacks_) {
                // auto range_msg = range_callback.
                ROS_INFO("debug");
            }

            ros::Rate(ros::Duration(0.05)).sleep();
            ros::spinOnce();
        }
    }

    void RangeToPointCloud::timerPubCallBack(const ros::TimerEvent& e) {}

} // namespace range_to_pointcloud
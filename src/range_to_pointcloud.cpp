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

            std::vector<
            std::pair<sensor_msgs::Range, geometry_msgs::TransformStamped>>
            ranges_tfs;
            for (const auto& range_callback : range_callbacks_) {

                ranges_tfs.emplace_back(std::make_pair(
                range_callback.getRange(), range_callback.getTF()));
            }
            populateRangesToPC(ranges_tfs);

            ros::Rate(ros::Duration(0.05)).sleep();
            ros::spinOnce();
        }
    }

    void RangeToPointCloud::populateRangesToPC(
    const std::vector<
    std::pair<sensor_msgs::Range, geometry_msgs::TransformStamped>>& ranges_tfs)
    {
        pcl::PointCloud<pcl::PointXYZ> ranges_cloud;
        ranges_cloud.header.frame_id = "base_link";

        for (const auto& range_tf : ranges_tfs) {

            double roll, pitch, yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(range_tf.second.transform.rotation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            float fov = range_tf.first.field_of_view;
            float range = range_tf.first.range;
            float resolution = fov / (num_of_populated_point_);
            double tf_x = range_tf.second.transform.translation.x;
            double tf_y = range_tf.second.transform.translation.y;
            double tf_z = range_tf.second.transform.translation.z;

            for (auto i = -num_of_populated_point_/2; i < num_of_populated_point_/2;
                 ++i) {

                pcl::PointXYZ point;
                point.x = tf_x + range * cos(yaw + i * resolution);
                point.y = tf_y + range * sin(yaw + i * resolution);
                point.z = tf_z;

                ranges_cloud.push_back(point);
            }
        }
        pointcloud_pub_.publish(ranges_cloud);
    }

    void RangeToPointCloud::timerPubCallBack(const ros::TimerEvent& e) {}

} // namespace range_to_pointcloud
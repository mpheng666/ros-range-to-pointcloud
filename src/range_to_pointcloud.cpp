#include "range_to_pointcloud/range_to_pointcloud.hpp"
namespace range_to_pointcloud {

    RangeToPointCloud::RangeToPointCloud(ros::NodeHandle& nh, int num)
        : nh_(nh)
        , number_of_sensor_(num)
        , pub_timer_(nh_.createTimer(
          ros::Duration(0.05), &RangeToPointCloud::timerPubCallBack, this))
        , pointcloud_pub_(
          nh_.advertise<sensor_msgs::PointCloud2>("range_pointcloud_output", 10))
    {
    }

    void RangeToPointCloud::start()
    {
        loadParams();
        for (const auto& range_sensor : range_sensors_name_) {
            range_callbacks_.emplace_back(range_sensor, target_frame_);
        }

        for (auto i = 0; i < range_callbacks_.size(); ++i)
        {
            std::string topic_name = range_sensors_name_.at(i);
            range_sensors_subs_.emplace_back(
            nh_g_.subscribe<sensor_msgs::Range>(
            topic_name, 10, std::ref(range_callbacks_.at(i))));
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

    void RangeToPointCloud::loadParams()
    {
        if (!nh_.param<int>("number_of_points_per_side",
                            number_of_points_per_side_,
                            number_of_points_per_side_)) {
            ROS_WARN_STREAM("number_of_points_per_side"
                            << " is not set! Use default "
                            << number_of_points_per_side_);
        }
        if (!nh_.param<std::string>("target_frame", target_frame_,
                                    target_frame_)) {
            ROS_WARN_STREAM("target_frame"
                            << " is not set! Use default " << target_frame_);
        }
        if (!nh_.param<bool>("use_defined_tf", use_defined_tf_,
                             use_defined_tf_)) {
            ROS_WARN_STREAM("use_defined_tf"
                            << " is not set! Use default " << use_defined_tf_);
        }
        if (!nh_.param<bool>("populate_z", populate_z_, populate_z_)) {
            ROS_WARN_STREAM("populate_z"
                            << " is not set! Use default " << populate_z_);
        }
        if (!nh_.param<std::vector<std::string>>(
            "range_sensors_name", range_sensors_name_, range_sensors_name_)) {
            for (const auto& range_sensor_name : range_sensors_name_) {
                ROS_WARN_STREAM("range_sensors_name"
                                << " is not set! Use default "
                                << range_sensor_name);
            }
        }
    }

    void RangeToPointCloud::populateRangesToPC(
    const std::vector<
    std::pair<sensor_msgs::Range, geometry_msgs::TransformStamped>>& ranges_tfs)
    {
        pcl::PointCloud<pcl::PointXYZ> ranges_cloud;
        ranges_cloud.header.frame_id = target_frame_;

        for (const auto& range_tf : ranges_tfs) {

            double roll, pitch, yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(range_tf.second.transform.rotation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            float fov = range_tf.first.field_of_view;
            float range = range_tf.first.range;
            float resolution_size = fov / (number_of_points_per_side_ * 2);
            double tf_x = range_tf.second.transform.translation.x;
            double tf_y = range_tf.second.transform.translation.y;
            double tf_z = range_tf.second.transform.translation.z;

            for (auto i = -number_of_points_per_side_;
                 i <= number_of_points_per_side_; ++i) {

                if (!populate_z_) {
                    pcl::PointXYZ point;
                    point.x = tf_x + range * cos(yaw + i * resolution_size);
                    point.y = tf_y + range * sin(yaw + i * resolution_size);
                    point.z = tf_z;
                    ranges_cloud.push_back(point);
                }
                else {
                    for (auto j = -number_of_points_per_side_;
                         j < number_of_points_per_side_; ++j) {
                        pcl::PointXYZ point;
                        point.x = tf_x + range * cos(yaw + i * resolution_size);
                        point.y = tf_y + range * sin(yaw + i * resolution_size);
                        point.z = tf_z + range * sin(j * resolution_size);
                        ranges_cloud.push_back(point);
                    }
                }
            }
        }
        pointcloud_pub_.publish(ranges_cloud);
    }

    void RangeToPointCloud::timerPubCallBack(const ros::TimerEvent& e) {}

} // namespace range_to_pointcloud
#include "range_to_pointcloud/range_to_pointcloud.hpp"

namespace sensor_msgs_transformation {

    RangeCallBack::RangeCallBack(int index)
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

    void RangeCallBack::operator()(const sensor_msgs::Range::ConstPtr& msg)
    {
        const double tf_x = transformStamped_baselink_sonar_.transform.translation.x;
        const double tf_y = transformStamped_baselink_sonar_.transform.translation.y;
        double roll;
        double pitch;
        double yaw;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(transformStamped_baselink_sonar_.transform.rotation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        geometry_msgs::Point transformed_point;
        transformed_point.x = tf_x + msg->range * (cos(yaw));
        transformed_point.y = tf_y + msg->range * (sin(yaw));
        transformed_range_data_.at(0) = transformed_point.x;
        transformed_range_data_.at(1) = transformed_point.y;

        untransformed_range_data_ = msg->range;
    }

    double RangeCallBack::getUntransformedData() { return untransformed_range_data_; }

    std::array<double, 2> RangeCallBack::getTransformedData()
    {
        return transformed_range_data_;
    }

    RangeToPointCloud::RangeToPointCloud(ros::NodeHandle& nh, int num)
        : nh_(nh)
        , number_of_sensor_(num)
        , pub_timer_(nh_.createTimer(
          ros::Duration(0.05), &SensorMsgsTransformation::multiSonarsPub, this))
        , multi_sonars_pub_without_tf(
          nh_.advertise<std_msgs::Float64MultiArray>("untransformed_sonars_data", 10))
        , multi_sonars_pub_with_tf(
          nh_.advertise<std_msgs::Float64MultiArray>("transformed_sonars_data", 10))
        , scan_sub_(
          nh_g_.subscribe("scan", 10, &SensorMsgsTransformation::scanCallBack, this))
        , pc_pub_(nh_.advertise<sensor_msgs::PointCloud2>("scan_point_cloud", 10))

    {
    }

    void RangeToPointCloud::start()
    {
        for (auto i = 0; i < number_of_sensor_; ++i) {
            range_callbacks_.emplace_back(i + 1);
        }

        range_sensors_subs_.resize(number_of_sensor_);

        for (auto i = 0; i < number_of_sensor_; ++i) {
            std::string topic_name = "sonar_" + std::to_string(i + 1);
            range_sensors_subs_.at(i) = nh_g_.subscribe<sensor_msgs::Range>(
            topic_name, 10, std::ref(range_callbacks_.at(i)));
        }

        utf_sonars_.resize(number_of_sensor_);
        tf_sonars_.resize(number_of_sensor_);

        while (ros::ok()) {
            for (const auto& range_callback :
                 range_callbacks_ | boost::adaptors::indexed(0)) {
                utf_sonars_.at(range_callback.index()) =
                range_callback.value().getUntransformedData();
                tf_sonars_.at(range_callback.index()) =
                range_callback.value().getTransformedData();
            }

            ros::Rate(ros::Duration(0.05)).sleep();
            ros::spinOnce();
        }
    }

    void RangeToPointCloud::multiSonarsPub(const ros::TimerEvent&)
    {
        // ROS_INFO("Check timestamped inside timer");
        std_msgs::Float64MultiArray untf_msg;
        std_msgs::Float64MultiArray tf_msg;

        untf_msg.data = utf_sonars_;
        for (const auto& tf_sonar : tf_sonars_) {
            tf_msg.data.push_back(tf_sonar.at(0));
            tf_msg.data.push_back(tf_sonar.at(1));
        }

        multi_sonars_pub_without_tf.publish(untf_msg);
        multi_sonars_pub_with_tf.publish(tf_msg);
    }

    void RangeToPointCloud::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
    {

        sensor_msgs::PointCloud2 pc;
        projector_.transformLaserScanToPointCloud("base_link", *msg, pc, tflistener_);
        pc_pub_.publish(pc);
    }

} // namespace sensor_msgs_transformation
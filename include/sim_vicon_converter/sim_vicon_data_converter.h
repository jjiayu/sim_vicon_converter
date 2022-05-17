#ifndef SIM_VICON_DATA_CONVERTER_H_
#define SIM_VICON_DATA_CONVERTER_H_

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

namespace sim_vicon_data_converter
{
class ViconDataConverter
{
public:
  ViconDataConverter()
    : nh_(""), priv_nh_("~") {
    //priv_nh_.getParam("vicon_topic", vicon_topic_);
    //priv_nh_.getParam("vicon_covariance", vicon_covariance_);

    vicon_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("vicon/pose", 1000);
    vicon_sub_ = nh_.subscribe("/floating_base_pose_simulated", 10,
                               &ViconDataConverter::vicon_callback, this, ros::TransportHints().tcpNoDelay());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Publisher vicon_pub_;
  ros::Subscriber vicon_sub_;

  std::string vicon_topic_;
  std::vector<double> vicon_covariance_;

  void vicon_callback(const nav_msgs::Odometry::ConstPtr msg) {
    auto new_msg = geometry_msgs::PoseWithCovarianceStamped();
    //new_msg.header = msg->header;
    //new_msg.pose.pose.position.x = msg->transform.translation.x;
    //new_msg.pose.pose.position.y = msg->transform.translation.y;
    //new_msg.pose.pose.position.z = msg->transform.translation.z;
    //new_msg.pose.pose.orientation = msg->transform.rotation;
    new_msg.header = msg->header;
    new_msg.pose = msg->pose;
    //for (size_t i = 0; i < vicon_covariance_.size(); i++) {
    //  new_msg.pose.covariance[i] = vicon_covariance_[i];
    //}
    // ROS_INFO("Publishing converted vicon data");
    vicon_pub_.publish(new_msg);
  }
};
}  // namespace sim_vicon_data_converter


#endif  // SIM_VICON_DATA_CONVERTER_H_

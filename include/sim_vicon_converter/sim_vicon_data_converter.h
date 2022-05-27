// Copyright (c) 2022, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
    priv_nh_.getParam("vicon_topic", vicon_topic_);
    priv_nh_.getParam("vicon_covariance", vicon_covariance_);

    vicon_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("vicon/pose", 1000);
    vicon_sub_ = nh_.subscribe(vicon_topic_, 10, &ViconDataConverter::vicon_callback, this, ros::TransportHints().tcpNoDelay());
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
    new_msg.header.stamp = ros::Time().now();
    new_msg.header.frame_id = "map";
    new_msg.pose = msg->pose;
    //new_msg.pose.pose.position.x = new_msg.pose.pose.position.x;
    //new_msg.pose.pose.orientation.x = 0.0;
    //new_msg.pose.pose.orientation.y = 0.02181489;
    //new_msg.pose.pose.orientation.z = 0.0;
    //new_msg.pose.pose.orientation.w =  0.99976203;

    for (size_t i = 0; i < vicon_covariance_.size(); i++) {
     new_msg.pose.covariance[i] = vicon_covariance_[i];
    }
    //ROS_INFO("Publishing converted vicon data");
    vicon_pub_.publish(new_msg);
  }
};
}  // namespace sim_vicon_data_converter

#endif  // SIM_VICON_DATA_CONVERTER_H_

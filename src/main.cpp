#include <ros/ros.h>

#include "sim_vicon_converter/sim_vicon_data_converter.h"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "sim_vicon_converter");
  auto sim_vicon_converter = sim_vicon_data_converter::ViconDataConverter();
  ros::spin();
  ros::shutdown();
  return 0;
}

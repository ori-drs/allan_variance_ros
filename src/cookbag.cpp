/**
 * \file cookbag.cpp
 * \mainpage
 *   Code for re-ordering bags containing IMU messages by their header time stamps
 *   Other messages other than IMU messages will be written without changes to their
 *   time stamps.
 *   This implementation is more limited but significantly faster than the Python version.
 *   Ported to C++ from http://wiki.ros.org/rosbag/Cookbook
 * \author
 *   Tobit Flatscher <tobit@robots.ox.ac.uk>
*/

#include <cstdlib>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "cookbag");
  ros::NodeHandle nh {"~"};

  std::string input_bag {};
  if (!nh.getParam("input_bag", input_bag)) {
    ROS_ERROR_STREAM("Parameter 'input_bag' is not set!");
    return EXIT_FAILURE;
  }
  std::string output_bag {};
  if (!nh.getParam("output_bag", output_bag)) {
    ROS_ERROR_STREAM("Parameter 'output_bag' is not set!");
    return EXIT_FAILURE;
  }

  rosbag::Bag inbag {input_bag, rosbag::bagmode::Read};
  rosbag::Bag outbag {output_bag, rosbag::bagmode::Write};

  rosbag::View view {inbag};
  for (rosbag::MessageInstance const& m: view) {
    std::string const& topic {m.getTopic()};
    if (m.getDataType() == "sensor_msgs/Imu") {
      auto msg = m.instantiate<sensor_msgs::Imu>();
      if (msg) {
        outbag.write(topic, msg->header.stamp, *msg);
      }
    } else {
      outbag.write(topic, m.getTime(), m);
    }
  }

  inbag.close();
  outbag.close();
  ROS_INFO_STREAM("Finished rewriting bag file with header time stamps!");
  return EXIT_SUCCESS;
}


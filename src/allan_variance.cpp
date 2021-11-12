/**
 * @file   allan_variance.cpp
 * @brief  Tool to compute Allan Variance and Deviation from rosbag.
 * @author Russell Buchanan
 */

// std, eigen and boost
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <ctime>
#include <fstream>
#include <set>

// ROS
#include <ros/node_handle.h>

// allan_variance_ros
#include "allan_variance_ros/AllanVarianceComputor.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "allan_variance_ros");
  ros::NodeHandle n("~");
  std::string bags_folder = ".";
  std::string config_file = "";

  if (argc >= 2) {
    bags_folder = argv[1];
    config_file = argv[2];
    ROS_INFO_STREAM("Bag Folder = " << bags_folder);
    ROS_INFO_STREAM("Config File = " << config_file);
  } else {
    ROS_WARN("Rosbag folder and/or config file not provided!");
  }

  namespace fs = boost::filesystem;
  fs::path path = fs::absolute(fs::path(bags_folder));

  std::set<std::string> bag_filenames_sorted;
  for (const auto& entry : fs::directory_iterator(path)) {
    if (entry.path().extension() == ".bag") {
      bag_filenames_sorted.insert(entry.path().string());
    }
  }
  ROS_INFO_STREAM("Bag filenames count: " << bag_filenames_sorted.size());

  std::clock_t start = std::clock();

  allan_variance_ros::AllanVarianceComputor computor(n, config_file, bags_folder);
  ROS_INFO_STREAM("Batch computor constructed");
  for (const auto& it : bag_filenames_sorted) {
    ROS_INFO_STREAM("Processing rosbag " << it);
    computor.run(it);
    if (!n.ok()) {
      break;
    }
    // For now just do one rosbag
    break;
  }

  double durationTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
  ROS_INFO("Total computation time: %f s", durationTime);
  return 0;
}

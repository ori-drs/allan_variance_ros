/**
 * @file   ImuSimulator.cpp
 * @brief  Tool to simulate imu data, ref:
 * https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model.
 * @author Rick Liu
 */

// std, eigen and boost
#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>
#include <fstream>
#include <set>

// ROS
#include <ros/node_handle.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>

#include "allan_variance_ros/yaml_parsers.hpp"

using Vec3d = Eigen::Vector3d;

Vec3d RandomNormalDistributionVector(double sigma) {
  static boost::mt19937 rng;
  static boost::normal_distribution<> nd(0, 1);
  return {sigma * nd(rng), sigma * nd(rng), sigma * nd(rng)};
}

template <typename S, typename T> void FillROSVector3d(const S &from, T &to) {
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
}

class ImuSimulator {
public:
  ImuSimulator(std::string config_file, std::string output_path)
      : bag_output_(output_path, rosbag::bagmode::Write) {
    auto yaml_config = loadYamlFile(config_file);

    get(yaml_config, "accelerometer_noise_density",
        accelerometer_noise_density_);
    get(yaml_config, "accelerometer_random_walk", accelerometer_random_walk_);
    get(yaml_config, "accelerometer_bias_init", accelerometer_bias_init_);

    get(yaml_config, "gyroscope_noise_density", gyroscope_noise_density_);
    get(yaml_config, "gyroscope_random_walk", gyroscope_random_walk_);
    get(yaml_config, "gyroscope_bias_init", gyroscope_bias_init_);

    get(yaml_config, "rostopic", rostopic_);
    get(yaml_config, "update_rate", update_rate_);
    get(yaml_config, "sequence_time", sequence_time_);
  }

  virtual ~ImuSimulator() { bag_output_.close(); }

  void run() {
    ROS_INFO_STREAM("Generating imu data ...");

    double dt = 1 / update_rate_;

    // clang-format off
    ros::Time start_time(1.0);
    Vec3d accelerometer_bias = Vec3d::Constant(accelerometer_bias_init_);
    Vec3d gyroscope_bias = Vec3d::Constant(gyroscope_bias_init_);
    Vec3d accelerometer_real = Vec3d::Zero();
    Vec3d gyroscope_real = Vec3d::Zero();


    for (int64_t i = 0; i < sequence_time_ * update_rate_; ++i) {
      // Reference: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
      accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
      gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);

      Vec3d acc_measure = accelerometer_real + accelerometer_bias + RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
      Vec3d gyro_measure = gyroscope_real + gyroscope_bias + RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);

      sensor_msgs::Imu msg;
      msg.header.stamp = start_time + ros::Duration(1, 0) * (i / update_rate_);
      msg.header.seq = i;
      FillROSVector3d(acc_measure, msg.linear_acceleration);
      FillROSVector3d(gyro_measure, msg.angular_velocity);

      bag_output_.write(rostopic_, msg.header.stamp, msg);
    }
    // clang-format on

    ROS_INFO_STREAM("Finished generating data. ");
  }

private:
  // ROS
  rosbag::Bag bag_output_;

private:
  double accelerometer_noise_density_;
  double accelerometer_random_walk_;
  double accelerometer_bias_init_;

  double gyroscope_noise_density_;
  double gyroscope_random_walk_;
  double gyroscope_bias_init_;

  std::string rostopic_;
  double update_rate_;

  double sequence_time_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "allan_variance_ros");
  ros::NodeHandle nh;
  std::string rosbag_filename;
  std::string config_file;

  if (argc >= 2) {
    rosbag_filename = argv[1];
    config_file = argv[2];
    ROS_INFO_STREAM("Bag filename = " << rosbag_filename);
    ROS_INFO_STREAM("Config File = " << config_file);
  } else {
    ROS_WARN("Usage: ./imu_simulator /path/to/output/bag_filename /path/to/simulation/config_filename");
    return 1;
  }

  auto start = std::clock();

  ImuSimulator simulator(config_file, rosbag_filename);
  ROS_INFO_STREAM("Imu simulator constructed");
  simulator.run();

  double durationTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
  ROS_INFO("Total computation time: %f s", durationTime);
  return 0;
}

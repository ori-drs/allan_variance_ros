#pragma once

// ROS
#include <ros/node_handle.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>

// allan_variance_ros
#include "allan_variance_ros/ImuMeasurement.hpp"
#include "allan_variance_ros/yaml_parsers.hpp"

namespace allan_variance_ros {

template <class T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

uint64_t s2ns(double t) { return static_cast<uint64_t>(t * 1000000000); }
double ns2s(uint64_t t) { return static_cast<double>(t * 0.000000001); }

struct ImuFormat {
  double time;
  double accX;
  double accY;
  double accZ;
  double gyroX;
  double gyroY;
  double gyroZ;
  double qx;
  double qy;
  double qz;
  double qw;

  void writeOnFile(std::ofstream& file) {
    file << std::setprecision(19) << time << std::setprecision(7) << " " << accX << " " << accY << " " << accZ << " "
         << gyroX << " " << gyroY << " " << gyroZ << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
  }
};

struct AllanVarianceFormat {
  double period;
  double accX;
  double accY;
  double accZ;
  double gyroX;
  double gyroY;
  double gyroZ;

  void writeOnFile(std::ofstream& file) {
    file << std::setprecision(19) << period << std::setprecision(7) << " " << accX << " " << accY << " " << accZ << " "
         << gyroX << " " << gyroY << " " << gyroZ << " " << std::endl;
  }
};

class AllanVarianceComputor {
 public:
  AllanVarianceComputor(ros::NodeHandle& nh, std::string config_file, std::string output_path);

  virtual ~AllanVarianceComputor() {closeOutputs();}

  void run(std::string bag_path);
  void closeOutputs();
  void allanVariance();
  void writeAllanDeviation(std::vector<double> variance, double period);

 private:
  // ROS
  ros::NodeHandle& nh_;
  rosbag::Bag bag;

  // Data
  AllanVarianceFormat aVRecorder_{};
  std::ofstream av_output_;
  std::string imu_output_file_;

  // Config
  int sequence_time_{};
  int measure_rate_{};
  std::vector<std::string> input_topics_;
  double imu_rate_ = 100.0;

  int skipped_imu_{};
  int imu_skip_;
  uint64_t tCurrNanoSeconds_{};
  uint64_t lastImuTime_{};
  uint64_t firstTime_{};
  EigenVector<ImuMeasurement> imuBuffer_;
  bool firstMsg_;
};
}  // namespace allan_variance_ros
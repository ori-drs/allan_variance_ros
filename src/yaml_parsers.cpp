#include "allan_variance_ros/yaml_parsers.hpp"
#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <yaml-cpp/node/parse.h>

YAML::Node loadYamlFile(const std::string &filename) {
  if (filename.empty()) { throw std::invalid_argument("Filename is empty!"); }

  YAML::Node node;

  try {
    node = YAML::LoadFile(filename);
  } catch (...) {
    throw std::invalid_argument("Error reading config file: " + filename);
  }

  if (node.IsNull()) { throw std::invalid_argument("Error reading config file: " + filename); }

  ROS_INFO_STREAM("Successfully read config file: " << filename);

  return node;
}


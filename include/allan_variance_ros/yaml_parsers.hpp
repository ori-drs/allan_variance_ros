#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/convert.h>
#include <yaml-cpp/node/detail/impl.h>


/// Update the value only if it can be found in the YAML file.
template<class T> static
bool get(const YAML::Node &node, const std::string& param, T& value) {
  if (!node[param]) { return false; }
  value = node[param].as<T>();
  return true;
}

YAML::Node loadYamlFile(const std::string &filename);


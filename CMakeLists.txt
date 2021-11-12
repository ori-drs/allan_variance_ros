cmake_minimum_required(VERSION 3.0.2)
project(allan_variance_ros)

set(CATKIN_PACKAGE_DEPENDENCIES
    geometry_msgs
    sensor_msgs
    rospy
    tf2_geometry_msgs
    tf2_ros
    rosbag)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS YAML_CPP
)

include_directories(
  include
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/ImuMeasurement.cpp
src/yaml_parsers.cpp
src/AllanVarianceComputor.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(allan_variance src/allan_variance.cpp)

target_link_libraries(allan_variance ${PROJECT_NAME} )

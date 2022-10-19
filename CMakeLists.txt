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

find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found will try to link!")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS YAML_CPP
)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
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

add_executable(imu_simulator src/ImuSimulator.cpp)

target_link_libraries(imu_simulator ${PROJECT_NAME} )

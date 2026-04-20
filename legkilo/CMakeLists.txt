cmake_minimum_required(VERSION 3.8)
project(legkilo)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(INFO "CMake build type: ${CMAKE_BUILD_TYPE}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

message(INFO "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(CPU_N)
message(INFO "Processor number: ${CPU_N}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)

set(SOURCES
    src/preprocess/lidar_processing.cc
    src/preprocess/kinematics.cc
    src/core/slam/KILO.cc
    src/core/slam/voxel_map.cc
    src/core/slam/eskf.cc
    src/interface/ros1/ros_interface.cc
    src/interface/ros1/options.cc
)

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
        src
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    pcl_conversions
    unitree_legged_msgs
)
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GLOG_LIBRARIES}
    gflags
    yaml-cpp
)

add_executable(${PROJECT_NAME}_node
    src/apps/leg_kilo_node.cc
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
)

install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
    launch
    rviz
    robot_urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

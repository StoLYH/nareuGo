################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_gazebo)

# Suppress CMake warnings for gazebo package
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress CMake developer warnings" FORCE)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gazebo Classic
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_definitions(${GAZEBO_CXX_FLAGS})

# Elevator Plugin
add_library(ElevatorRos2Plugin SHARED ../../Plugin/ElevatorRos2Plugin.cpp)
ament_target_dependencies(ElevatorRos2Plugin rclcpp std_msgs gazebo_ros)
target_link_libraries(ElevatorRos2Plugin ${GAZEBO_LIBRARIES})

# Apt_Door Plugin
add_library(AptDoorRos2Plugin SHARED ../../Plugin/AptDoorRos2Plugin.cpp)
ament_target_dependencies(AptDoorRos2Plugin rclcpp std_msgs gazebo_ros)
target_link_libraries(AptDoorRos2Plugin ${GAZEBO_LIBRARIES})

# Coordinate Plugin
add_library(CoordinatorRos2Plugin SHARED ../../Plugin/CoordinatorRos2Plugin.cpp)
ament_target_dependencies(CoordinatorRos2Plugin rclcpp std_msgs gazebo_ros)
target_link_libraries(CoordinatorRos2Plugin ${GAZEBO_LIBRARIES})

# Storage Door Plugin 추가
add_library(StorageDoorRos2Plugin SHARED ../../Plugin/StorageDoorRos2Plugin.cpp)
ament_target_dependencies(StorageDoorRos2Plugin rclcpp std_msgs gazebo_ros)
target_link_libraries(StorageDoorRos2Plugin ${GAZEBO_LIBRARIES})

# Box Control Plugin 추가
add_library(BoxControlPlugin SHARED ../../Plugin/BoxControlPlugin.cpp) 
ament_target_dependencies(BoxControlPlugin rclcpp std_msgs gazebo_ros)
target_link_libraries(BoxControlPlugin ${GAZEBO_LIBRARIES})


# Plugin libraries 설치
install(
  TARGETS ElevatorRos2Plugin AptDoorRos2Plugin CoordinatorRos2Plugin StorageDoorRos2Plugin BoxControlPlugin
  LIBRARY DESTINATION lib
)

# 리소스 파일들을 설치
install(
  DIRECTORY config gazebo launch ros2_control rviz urdf worlds models meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

################################################################################
# Install
################################################################################
#install(
#  DIRECTORY config gazebo launch ros2_control rviz urdf worlds models meshes
#  DESTINATION share/${PROJECT_NAME}
#)

################################################################################
# Macro for ament package
################################################################################
#ament_package()

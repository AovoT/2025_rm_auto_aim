cmake_minimum_required(VERSION 3.8)
project(armor_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

ament_package()

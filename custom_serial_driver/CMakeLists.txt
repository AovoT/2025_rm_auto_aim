cmake_minimum_required(VERSION 3.8)
project(custom_serial_driver)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARN 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "custom_serial::SerialDriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

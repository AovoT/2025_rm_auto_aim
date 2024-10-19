cmake_minimum_required(VERSION 3.8)
project(hik_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 源文件路径
file(GLOB_RECURSE SRC src/*.cpp)

# 创建共享库
add_library(${PROJECT_NAME} SHARED ${SRC})

# 添加依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
)

# 设置包含目录，使用 INTERFACE
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)


# 设置链接目录（可选，通常不需要）
target_link_directories(${PROJECT_NAME} PRIVATE
  ./lib
)

# 链接库
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
  MVGigEVisionSDK
  yaml-cpp
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 导出库和头文件
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()

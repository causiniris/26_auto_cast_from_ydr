cmake_minimum_required(VERSION 3.8)
project(horizontal_line_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加节点可执行文件
add_executable(horizontal_line_detector_node src/horizontal_line_detector_node.cpp)

# 链接目标库
target_include_directories(horizontal_line_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(horizontal_line_detector_node ${OpenCV_LIBRARIES})
ament_target_dependencies(horizontal_line_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
)

# 安装规则：可执行文件
install(TARGETS horizontal_line_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装规则：Launch 文件夹 (如果你后续写了 launch 文件的话)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
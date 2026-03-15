cmake_minimum_required(VERSION 3.8)
project(system_manager_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(robot_motion_msgs REQUIRED)
find_package(robot_common_pkg REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================
# 生成组件库
# =========================
add_library(system_manager_component SHARED
  src/system_manager_node.cpp
  src/state_machine.cpp
)

target_include_directories(system_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(system_manager_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  trajectory_msgs
  builtin_interfaces
  robot_motion_msgs
  robot_common_pkg
)

# 注册组件
rclcpp_components_register_nodes(system_manager_component
  "system_manager_pkg::SystemManagerNode"
)

# =========================
# 如果同时保留普通启动方式，也可以继续加 main.cpp
# 这样既能 ros2 run，也能 component load
# =========================
add_executable(system_manager_node
  src/main.cpp
)

target_include_directories(system_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(system_manager_node
  system_manager_component
)

ament_target_dependencies(system_manager_node
  rclcpp
)

# =========================
# 安装
# =========================
install(TARGETS
  system_manager_component
  system_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

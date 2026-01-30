cmake_minimum_required(VERSION 3.8)
project(cmd_vel_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_input_msgs REQUIRED) # 对应您上传的自定义消息包
find_package(std_msgs REQUIRED)

# 2. 定义可执行文件 (编译 src/CmdVelBridge.cpp)
# 注意：这里假设您的源代码文件名是 CmdVelBridge.cpp
add_executable(cmd_vel_bridge_node src/CmdVelBridge.cpp)

# 3. 设置包含目录
target_include_directories(cmd_vel_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 4. 链接依赖库 (关键步骤：将节点与找到的包链接)
ament_target_dependencies(cmd_vel_bridge_node
  rclcpp
  geometry_msgs
  control_input_msgs
  std_msgs
)

# 5. 安装规则 (非常重要)
# 如果没有这一步，ros2 run 将找不到节点
install(TARGETS cmd_vel_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件夹，以便 ros2 launch 可以找到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
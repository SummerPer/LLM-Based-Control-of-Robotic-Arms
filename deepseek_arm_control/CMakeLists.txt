cmake_minimum_required(VERSION 3.8)
project(deepseek_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 生成服务消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlRvizArm.srv"
  "srv/NaturalLanguageControl.srv"
  DEPENDENCIES geometry_msgs
)

# 查找额外的依赖库
execute_process(COMMAND pkg-config --libs libcurl OUTPUT_VARIABLE CURL_LIBS OUTPUT_STRIP_TRAILING_WHITESPACE)

# 检查nlohmann/json.hpp是否存在于标准位置
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES nlohmann
  DOC "Path to nlohmann/json.hpp header file"
)

# 添加可执行文件
add_executable(end_effector_monitor_node 
  src/end_effector_monitor.cpp
  src/main.cpp
)

# 添加运动规划服务节点
add_executable(arm_mover_server_node 
  src/arm_mover_server.cpp
)

# 添加命令行客户端节点
add_executable(arm_control_client_cli_node 
  src/arm_control_client_cli.cpp
)



# 添加自然语言到机械臂控制节点
# 该节点通过Python脚本调用OpenAI API将自然语言转换为机械臂坐标
add_executable(natural_language_to_arm_control_node
  src/natural_language_to_arm_control.cpp
)
# 添加包含目录
target_include_directories(end_effector_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(arm_mover_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(arm_control_client_cli_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(natural_language_to_arm_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖库
ament_target_dependencies(end_effector_monitor_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

# 为natural_language_to_arm_control_node添加依赖（popen函数需要的标准C库已默认包含）

ament_target_dependencies(arm_mover_server_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  moveit_ros_planning_interface
)

ament_target_dependencies(arm_control_client_cli_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(natural_language_to_arm_control_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)
# 链接服务消息
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(arm_mover_server_node ${cpp_typesupport_target})
target_link_libraries(arm_control_client_cli_node ${cpp_typesupport_target})

target_include_directories(natural_language_to_arm_control_node PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
target_link_libraries(natural_language_to_arm_control_node ${cpp_typesupport_target} ${CURL_LIBS})
# 安装可执行文件
install(TARGETS end_effector_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS arm_mover_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS arm_control_client_cli_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS natural_language_to_arm_control_node
  DESTINATION lib/${PROJECT_NAME}
)
# 安装启动文件 (包括所有新创建的launch文件)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  PATTERN "__pycache__" EXCLUDE
)

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

ament_package()

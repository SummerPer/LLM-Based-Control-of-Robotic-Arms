#include <memory>
#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>

// ROS2核心库
#include <rclcpp/rclcpp.hpp>
// 服务消息
#include <deepseek_arm_control/srv/control_rviz_arm.hpp>

using namespace std::chrono_literals;

/**
 * @brief 机械臂控制命令行客户端
 * 用于直接通过命令行参数控制机械臂运动
 */
int main(int argc, char **argv)
{
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建客户端节点
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arm_control_client_cli");
  
  // 创建服务客户端
  rclcpp::Client<deepseek_arm_control::srv::ControlRvizArm>::SharedPtr client = 
    node->create_client<deepseek_arm_control::srv::ControlRvizArm>("control_rviz_arm");

  // 创建请求对象
  auto request = std::make_shared<deepseek_arm_control::srv::ControlRvizArm::Request>();
  
  // 默认位姿
  std::vector<double> default_position = {-0.18, 0.37, 0.35, 0.0, 0.0, 0.0, 1.0};
  std::string default_gripper = "open";
  
  // 解析命令行参数
  if (argc >= 8) {
    // 如果提供了足够的参数，使用命令行参数
    request->position.resize(7);
    for (int i = 1; i <= 7; i++) {
      request->position[i-1] = std::stod(argv[i]);
    }
    
    if (argc >= 9) {
      request->open_or_close = argv[8];
    } else {
      request->open_or_close = default_gripper;
    }
  } else {
    // 使用默认值
    request->position = default_position;
    request->open_or_close = default_gripper;
    
    // 显示帮助信息
    std::cout << "使用方法: " << argv[0] << " [x y z qx qy qz qw] [open/close]" << std::endl;
    std::cout << "  示例: " << argv[0] << " -0.18 0.37 0.35 0.0 0.0 0.0 1.0 open" << std::endl;
    std::cout << "  未提供足够参数，使用默认值: " << std::endl;
    std::cout << "  位姿: [" << default_position[0] << ", " << default_position[1] << ", " 
              << default_position[2] << ", " << default_position[3] << ", " 
              << default_position[4] << ", " << default_position[5] << ", " 
              << default_position[6] << "]" << std::endl;
    std::cout << "  夹爪状态: " << default_gripper << std::endl;
  }

  // 等待服务可用
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "客户端被中断，等待服务中...");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "服务不可用，等待中...");
  }

  // 显示将要发送的请求信息
  RCLCPP_INFO(node->get_logger(), "发送控制请求: 位姿[x: %.3f, y: %.3f, z: %.3f, qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f], 夹爪: %s",
              request->position[0], request->position[1], request->position[2],
              request->position[3], request->position[4], request->position[5], request->position[6],
              request->open_or_close.c_str());

  // 发送请求
  auto result_future = client->async_send_request(request);
  
  // 等待响应
  if (rclcpp::spin_until_future_complete(node, result_future) == 
    rclcpp::FutureReturnCode::SUCCESS) 
  {
    auto result = result_future.get();
    if (result->success) {
        RCLCPP_INFO(node->get_logger(), "机械臂控制成功！路径规划已完成，机械臂开始运动。");
      } else {
        RCLCPP_ERROR(node->get_logger(), "机械臂控制失败！");
      }
  } else {
    RCLCPP_ERROR(node->get_logger(), "服务调用失败！");
  }

  // 关闭ROS2
  rclcpp::shutdown();
  return 0;
}
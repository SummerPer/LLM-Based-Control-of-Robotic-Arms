#include "deepseek_arm_control/end_effector_monitor.hpp"

int main(int argc, char * argv[]) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点选项
    rclcpp::NodeOptions options;
    
    // 创建末端位置监控器实例
    auto monitor = std::make_shared<deepseek_arm_control::EndEffectorMonitor>(options);
    
    // 启动监控
    monitor->startMonitoring();
    
    // 运行节点
    rclcpp::spin(monitor->getNode());
    
    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}
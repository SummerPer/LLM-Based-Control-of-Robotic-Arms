#ifndef DEEPSEEK_ARM_CONTROL_END_EFFECTOR_MONITOR_HPP_
#define DEEPSEEK_ARM_CONTROL_END_EFFECTOR_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

namespace deepseek_arm_control {

class EndEffectorMonitor {
public:
    // 构造函数，初始化节点和参数
    EndEffectorMonitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    // 析构函数
    ~EndEffectorMonitor();
    
    // 启动监控
    void startMonitoring();
    
    // 停止监控
    void stopMonitoring();
    
    // 获取当前末端位置（同步接口）
    geometry_msgs::msg::Pose getCurrentPose();
    
    // 获取节点指针
    rclcpp::Node::SharedPtr getNode() { return node_; }

private:
    // 节点实例
    rclcpp::Node::SharedPtr node_;
    
    // TF2监听器
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 位置发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    
    // 定时器，用于定期查询位置
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 定时器回调函数，定期查询并发布位置
    void timerCallback();
    
    // 上一次获取的位置缓存
    geometry_msgs::msg::Pose last_pose_;
    
    // 坐标系名称参数
    std::string target_frame_;  // 末端执行器坐标系
    std::string source_frame_;  // 基础坐标系
    
    // 更新频率参数
    double update_frequency_;   // 位置更新频率（Hz）
};

}  // namespace deepseek_arm_control

#endif  // DEEPSEEK_ARM_CONTROL_END_EFFECTOR_MONITOR_HPP_
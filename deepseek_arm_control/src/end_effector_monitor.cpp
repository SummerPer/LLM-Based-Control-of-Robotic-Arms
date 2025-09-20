#include "deepseek_arm_control/end_effector_monitor.hpp"

namespace deepseek_arm_control {

EndEffectorMonitor::EndEffectorMonitor(const rclcpp::NodeOptions& options) {
    // 创建节点选项并设置use_sim_time参数
    rclcpp::NodeOptions node_options = options;
    
    // 声明和获取参数
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // 创建节点
    node_ = rclcpp::Node::make_shared("end_effector_monitor", node_options);
    
    // 获取参数
    bool use_sim_time = false;
    node_->get_parameter("use_sim_time", use_sim_time);
    
    // 获取其他参数，如果没有设置则使用默认值
    try {
        node_->get_parameter("target_frame", target_frame_);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        target_frame_ = "panda_link8";
    }
    
    try {
        node_->get_parameter("source_frame", source_frame_);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        source_frame_ = "panda_link0";
    }
    
    try {
        node_->get_parameter("update_frequency", update_frequency_);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        update_frequency_ = 10;  // 提高默认更新频率到10Hz，确保实时性
    }
    
    std::string pose_topic_name = "/end_effector_pose";
    try {
        node_->get_parameter("pose_topic_name", pose_topic_name);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        // 使用默认值
    }
    
    // 初始化TF2监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 创建位置发布器
    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        pose_topic_name, 10);
    
    // 初始化位置缓存
    last_pose_.position.x = 0.0;
    last_pose_.position.y = 0.0;
    last_pose_.position.z = 0.0;
    last_pose_.orientation.x = 0.0;
    last_pose_.orientation.y = 0.0;
    last_pose_.orientation.z = 0.0;
    last_pose_.orientation.w = 1.0;
    
    RCLCPP_INFO(node_->get_logger(), "EndEffectorMonitor initialized. Target frame: %s, Source frame: %s, Update frequency: %.1f Hz, Use sim time: %s",
               target_frame_.c_str(), source_frame_.c_str(), update_frequency_, use_sim_time ? "true" : "false");
}
 
EndEffectorMonitor::~EndEffectorMonitor() {
    if (timer_) {
        timer_->cancel();
    }
    RCLCPP_INFO(node_->get_logger(), "EndEffectorMonitor shutdown");
}

void EndEffectorMonitor::startMonitoring() {
    if (!timer_) {
        // 创建定时器，定期调用回调函数
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency_));
        timer_ = node_->create_wall_timer(
            period,
            std::bind(&EndEffectorMonitor::timerCallback, this));
        
        RCLCPP_INFO(node_->get_logger(), "Started monitoring end effector position from %s to %s", 
                   source_frame_.c_str(), target_frame_.c_str());
    }
}

void EndEffectorMonitor::stopMonitoring() {
    if (timer_) {
        timer_->cancel();
        timer_.reset();
        RCLCPP_INFO(node_->get_logger(), "Stopped monitoring end effector position");
    }
}

geometry_msgs::msg::Pose EndEffectorMonitor::getCurrentPose() {
    return last_pose_;
}

void EndEffectorMonitor::timerCallback() {
    try {
        // 获取从source_frame到target_frame的变换
        // 注意：lookupTransform的参数顺序是(target_frame, source_frame, time)表示
        // "将source_frame中的点转换到target_frame坐标系中"
        // 但我们这里需要的是相反的变换方向，所以交换参数顺序
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(source_frame_, target_frame_, 
                                       tf2::TimePointZero);
        
        // 构建PoseStamped消息
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = source_frame_;  // 使用source_frame作为参考坐标系
        pose_msg.header.stamp = node_->now();
        
        // 设置位置信息
        pose_msg.pose.position.x = transform.transform.translation.x;
        pose_msg.pose.position.y = transform.transform.translation.y;
        pose_msg.pose.position.z = transform.transform.translation.z;
        
        // 设置姿态信息（四元数）
        pose_msg.pose.orientation.x = transform.transform.rotation.x;
        pose_msg.pose.orientation.y = transform.transform.rotation.y;
        pose_msg.pose.orientation.z = transform.transform.rotation.z;
        pose_msg.pose.orientation.w = transform.transform.rotation.w;
        
        // 更新缓存
        last_pose_ = pose_msg.pose;
        
        // 发布位置信息
        pose_publisher_->publish(pose_msg);
        
        // 添加调试日志，显示发布的位置信息
        RCLCPP_DEBUG(node_->get_logger(), "Published end effector pose: x=%.4f, y=%.4f, z=%.4f",
                   pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s",
                   source_frame_.c_str(), target_frame_.c_str(), ex.what());
    }
}

}  // namespace deepseek_arm_control
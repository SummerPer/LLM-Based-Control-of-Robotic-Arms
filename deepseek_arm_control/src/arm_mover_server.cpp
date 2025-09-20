#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <cmath>
#include <chrono>
#include <thread>
#include <shared_mutex> // 用于读写锁

// ROS2核心库
#include <rclcpp/rclcpp.hpp>
// MoveIt2库
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
// 服务消息
#include <deepseek_arm_control/srv/control_rviz_arm.hpp>
// 几何消息
#include <geometry_msgs/msg/pose.hpp>
// 订阅消息
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief 机械臂运动规划服务节点
 * 实现机械臂末端执行器位姿控制和夹爪控制功能
 */
class ArmMoverServer : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  ArmMoverServer() : Node("arm_mover_server"), initialized_(false), use_simulation_mode_(false), stop_position_thread_(false)
  {
    // 声明并获取ROS2参数 - 控制模式
    this->declare_parameter("use_simulation_mode", false);
    this->get_parameter("use_simulation_mode", use_simulation_mode_);
    
    // 声明并获取末端执行器位置话题参数 - 用于订阅外部位置信息
    // 这是末端执行器参数传递的第一个路径：通过ROS2话题订阅
    this->declare_parameter("end_effector_pose_topic", "/end_effector_pose");
    std::string end_effector_pose_topic;
    this->get_parameter("end_effector_pose_topic", end_effector_pose_topic);
    
    // 声明位置确认参数（只声明一次） - 这些参数控制末端执行器位置检测的精度和等待时间
    this->declare_parameter("position_tolerance", 0.01); // 1厘米的位置容差
    this->declare_parameter("max_wait_time", 10); // 最大等待时间10秒

    // 创建服务，处理机械臂控制请求 - 接收目标位置参数
    service_ = this->create_service<deepseek_arm_control::srv::ControlRvizArm>(
      "control_rviz_arm",
      std::bind(&ArmMoverServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 创建订阅者，接收末端执行器位置信息 - 末端参数传递的关键接口
    end_effector_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      end_effector_pose_topic,
      10,
      std::bind(&ArmMoverServer::endEffectorPoseCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "机械臂运动规划服务已启动");
    RCLCPP_INFO(this->get_logger(), "使用模拟模式: %s", use_simulation_mode_ ? "是" : "否");
    RCLCPP_INFO(this->get_logger(), "订阅末端执行器位置话题: %s", end_effector_pose_topic.c_str());

    // 初始化状态
    before_gripper_state_ = "close";
    
    // 初始化末端执行器位置 - 设置默认值，等待接收实际数据
    current_end_effector_pose_.position.x = 0.0;
    current_end_effector_pose_.position.y = 0.0;
    current_end_effector_pose_.position.z = 0.0;
    current_end_effector_pose_.orientation.x = 0.0;
    current_end_effector_pose_.orientation.y = 0.0;
    current_end_effector_pose_.orientation.z = 0.0;
    current_end_effector_pose_.orientation.w = 1.0;

    // 创建定时器，延迟初始化MoveIt2接口，避免构造函数中使用shared_from_this()
    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() {
        this->initialize();
        // 初始化完成后取消定时器
        this->init_timer_->cancel();
      }
    );
  }
  
  /**
   * @brief 析构函数
   */
  ~ArmMoverServer()
  {
    // 停止位置更新线程 - 确保程序退出时线程正确终止
    stop_position_thread_ = true;
    if (position_update_thread_.joinable()) {
      position_update_thread_.join();
      RCLCPP_INFO(this->get_logger(), "位置更新线程已停止");
    }
  }

private:
  /**
   * @brief 处理机械臂控制请求
   * @param request 控制请求，包含目标位姿和夹爪状态
   * @param response 控制响应，表示操作是否成功
   * 
   * 这是目标位置参数的主要入口点，包含了从接收目标位置到确认到达目标位置的完整流程
   */
  void handle_request(
    const std::shared_ptr<deepseek_arm_control::srv::ControlRvizArm::Request> request,
    std::shared_ptr<deepseek_arm_control::srv::ControlRvizArm::Response> response
  )
  {
    if (!initialized_) {
      RCLCPP_ERROR(this->get_logger(), "服务未初始化完成，无法处理请求");
      response->success = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "收到机械臂控制请求");

    // 检查请求中的位置数组大小 - 参数验证
    // 位置数组格式：[x,y,z,qx,qy,qz,qw]，共7个元素
    if (request->position.size() != 7) {
      RCLCPP_ERROR(this->get_logger(), "位置数组大小错误，应为7个元素(x,y,z,xo,yo,zo,wo)");
      response->success = false;
      return;
    }

    if (use_simulation_mode_) {
      // 模拟模式：只打印日志信息，不实际控制机械臂
      RCLCPP_INFO(this->get_logger(), "模拟机械臂移动到目标位置: x=%.3f, y=%.3f, z=%.3f", 
                request->position[0], request->position[1], request->position[2]);
      RCLCPP_INFO(this->get_logger(), "模拟机械臂末端朝向: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                request->position[3], request->position[4], request->position[5], request->position[6]);

      // 模拟夹爪操作
      if (request->open_or_close != before_gripper_state_) {
        RCLCPP_INFO(this->get_logger(), "模拟夹爪%s操作", request->open_or_close.c_str());
        before_gripper_state_ = request->open_or_close;
      }
    } else {
      // 实际控制模式：使用MoveIt2进行路径规划和执行
      try {
        // 设置目标位姿 - 参数传递的核心步骤：将请求中的位置数组转换为MoveIt2可用的Pose对象
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = request->position[0]; // X坐标参数传递
        target_pose.position.y = request->position[1]; // Y坐标参数传递
        target_pose.position.z = request->position[2]; // Z坐标参数传递
        target_pose.orientation.x = request->position[3]; // 四元数X参数传递
        target_pose.orientation.y = request->position[4]; // 四元数Y参数传递
        target_pose.orientation.z = request->position[5]; // 四元数Z参数传递
        target_pose.orientation.w = request->position[6]; // 四元数W参数传递
        move_group_->setPoseTarget(target_pose); // 将目标位姿传递给MoveGroup接口

        RCLCPP_INFO(this->get_logger(), "设置机械臂目标位置: x=%.3f, y=%.3f, z=%.3f", 
                  request->position[0], request->position[1], request->position[2]);
        RCLCPP_INFO(this->get_logger(), "设置机械臂末端朝向: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                  request->position[3], request->position[4], request->position[5], request->position[6]);

        // 进行路径规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
          RCLCPP_ERROR(this->get_logger(), "路径规划失败");
          response->success = false;
          return;
        }

        RCLCPP_INFO(this->get_logger(), "路径规划成功");

        // 执行轨迹规划 - 将规划好的轨迹发送给机械臂控制器执行
        move_group_->execute(my_plan);
        
        // 规划完成，开始运动
        RCLCPP_INFO(this->get_logger(), "路径规划完成，机械臂开始运动");

        // 夹爪控制
        if (request->open_or_close != before_gripper_state_) {
          RCLCPP_INFO(this->get_logger(), "执行夹爪%s操作", request->open_or_close.c_str());
          // 设置夹爪位置（这里使用简单的位置设置，实际应用中可能需要更复杂的控制）
          std::vector<double> gripper_joint_values;
          if (request->open_or_close == "open") {
            // 打开夹爪的位置值（根据实际机器人调整）
            gripper_joint_values.push_back(0.04);
            gripper_joint_values.push_back(0.04);
          } else {
            // 关闭夹爪的位置值（根据实际机器人调整）
            gripper_joint_values.push_back(0.0);
            gripper_joint_values.push_back(0.0);
          }
          gripper_move_group_->setJointValueTarget(gripper_joint_values);
          
          // 规划并执行夹爪动作
          moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
          bool gripper_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
          
          if (gripper_success) {
            gripper_move_group_->execute(gripper_plan);
            before_gripper_state_ = request->open_or_close;
            RCLCPP_INFO(this->get_logger(), "夹爪%s操作成功", request->open_or_close.c_str());
          } else {
            RCLCPP_ERROR(this->get_logger(), "夹爪%s操作失败", request->open_or_close.c_str());
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "机械臂控制过程中发生错误: %s", e.what());
        response->success = false;
        return;
      }
    }

    // 操作成功
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "机械臂控制操作成功完成，且已确认末端执行器到达目标位置");
  }

  /**
   * @brief 处理末端执行器位置消息
   * 
   * 这是末端执行器参数传递的关键回调函数，用于接收外部发布的末端执行器位置信息
   * 通过订阅机制获取实时位置数据，是位置确认的重要数据来源之一
   */
  void endEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 使用写锁保护共享资源，因为这里要修改数据
    std::unique_lock<std::shared_mutex> lock(pose_mutex_);
    current_end_effector_pose_ = msg->pose; // 更新当前末端执行器位置
    last_updated_time_ = std::chrono::system_clock::now(); // 记录最后更新时间
  }
  
  /**
   * @brief 计算两个位置之间的欧氏距离
   */
  double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }
  
  /**
   * @brief 确认末端执行器是否到达目标位置
   * @param target_pose 目标位姿
   * @param tolerance 位置容差（米）
   * @return 是否到达目标位置
   * 
   * 这个函数可能是导致机械臂末端参数传递阻塞的关键点：
   * 1. 使用了std::this_thread::sleep_for导致线程阻塞
   * 2. 使用了互斥锁可能导致与位置更新线程的竞争
   * 3. 连续三次位置稳定的检测逻辑可能过于严格
   */
  bool confirmPositionReached(const geometry_msgs::msg::Pose& target_pose, double tolerance) {
    RCLCPP_INFO(this->get_logger(), "开始确认末端执行器是否到达目标位置");
    RCLCPP_INFO(this->get_logger(), "目标位置: x=%.3f, y=%.3f, z=%.3f", 
               target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    // 获取位置确认参数（已在构造函数中声明）
    int max_wait_time;
    this->get_parameter("max_wait_time", max_wait_time);
    
    // 连续三次位置不变的计数器
    int stable_count = 0;
    // 最大稳定次数
    const int max_stable_count = 3;
    // 两次检测之间的时间间隔（秒）- 这里使用1秒的间隔可能导致整体确认过程较慢
    const int check_interval = 1;
    // 上次检测的位置
    geometry_msgs::msg::Pose last_detected_pose;
    // 位置稳定性容差（比目标容差小）
    double stability_tolerance = tolerance * 0.5;
    // 检测开始时间
    auto start_time = std::chrono::system_clock::now();
    
    // 第一次获取当前位置 - 使用读锁
    { 
      std::shared_lock<std::shared_mutex> lock(pose_mutex_);
      last_detected_pose = current_end_effector_pose_;
    }
    
    RCLCPP_INFO(this->get_logger(), "开始监测机械臂稳定性，最大等待时间: %d秒", max_wait_time);
    
    // 循环检测，直到达到最大等待时间或确认位置稳定
    // 注意：这个循环在主线程中运行，可能会阻塞服务响应
    while (stable_count < max_stable_count) {
      // 检查是否超时
      auto current_time = std::chrono::system_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
      
      if (elapsed_time > max_wait_time) {
        RCLCPP_WARN(this->get_logger(), "位置确认超时 (%d秒)", max_wait_time);
        break;
      }
      
      // 等待1秒后再次检测 - 这是主要的阻塞点，会使整个服务请求线程暂停1秒
      std::this_thread::sleep_for(std::chrono::seconds(check_interval));
      
      // 获取当前位置 - 使用读锁，允许多线程同时读取
      geometry_msgs::msg::Pose current_pose;
      { 
        std::shared_lock<std::shared_mutex> lock(pose_mutex_);
        current_pose = current_end_effector_pose_;
      }
      
      // 计算与上次检测位置的距离
      double movement_distance = calculateDistance(current_pose.position, last_detected_pose.position);
      
      // 更新上次检测位置
      last_detected_pose = current_pose;
      
      // 计算与目标位置的距离
      double target_distance = calculateDistance(current_pose.position, target_pose.position);
      
      RCLCPP_INFO(this->get_logger(), "当前位置: x=%.3f, y=%.3f, z=%.3f, 与目标距离: %.4f米, 移动距离: %.4f米", 
                 current_pose.position.x, current_pose.position.y, current_pose.position.z, 
                 target_distance, movement_distance);
      
      // 检查位置是否稳定（移动距离小于稳定性容差）
      if (movement_distance <= stability_tolerance) {
        stable_count++;
        RCLCPP_INFO(this->get_logger(), "位置稳定检测通过 (连续稳定次数: %d/%d)", stable_count, max_stable_count);
      } else {
        // 如果不稳定，重置计数器 - 这个逻辑可能导致确认过程被拉长
        stable_count = 0;
        RCLCPP_INFO(this->get_logger(), "位置尚未稳定，重置稳定计数器");
      }
    }
    
    // 最后一次获取当前位置并检查是否到达目标 - 使用读锁
    geometry_msgs::msg::Pose final_pose;
    { 
      std::shared_lock<std::shared_mutex> lock(pose_mutex_);
      final_pose = current_end_effector_pose_;
    }
    
    double final_distance = calculateDistance(final_pose.position, target_pose.position);
    
    if (stable_count >= max_stable_count && final_distance <= tolerance) {
      RCLCPP_INFO(this->get_logger(), "机械臂已稳定且到达目标位置，距离误差: %.4f米", final_distance);
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "机械臂未能到达目标位置，距离误差: %.4f米 (容差: %.4f米)", final_distance, tolerance);
      return false;
    }
  }
  
  // 启动位置更新线程
  // 注意：现在不再从MoveGroup获取位置信息，仅保留线程结构以维持兼容性
  void startPositionUpdateThread() {
    try {
      // 获取更新频率参数，默认改为20Hz
      this->declare_parameter("position_update_frequency", 20.0);
      double update_frequency;
      this->get_parameter("position_update_frequency", update_frequency);
      
      RCLCPP_INFO(this->get_logger(), "启动末端执行器位置监控线程，频率: %.1fHz", update_frequency);
      
      // 启动线程
      stop_position_thread_ = false;
      position_update_thread_ = std::thread(&ArmMoverServer::positionUpdateLoop, this, update_frequency);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "启动位置监控线程失败: %s", e.what());
    }
  }
  
  // 位置更新线程循环
  // 此线程现在仅作为监控线程运行，不再主动从MoveGroup获取位置
  // 所有位置信息都从话题/end_effector_pose订阅获取
  void positionUpdateLoop(double frequency) {
    double update_interval = 1.0 / frequency;
    std::chrono::duration<double> interval(update_interval);
    
    while (!stop_position_thread_) {
      try {
        // 不再从MoveGroup获取位置信息，完全依赖话题订阅
        // 此处可以添加位置数据有效性检查等逻辑
        
        // 检查上次更新时间，确保位置数据是最新的 - 使用读锁
        auto current_time = std::chrono::system_clock::now();
        auto time_since_update = 0;
        {
          std::shared_lock<std::shared_mutex> lock(pose_mutex_);
          time_since_update = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_updated_time_).count();
        }
        
        // 如果超过5秒没有收到位置更新，记录警告
        if (time_since_update > 5) {
          RCLCPP_WARN(this->get_logger(), "长时间未收到末端执行器位置更新（%d秒）", time_since_update);
        }
      } catch (const std::exception& e) {
        // 发生异常时记录日志但继续循环
        RCLCPP_ERROR(this->get_logger(), "位置监控线程异常: %s", e.what());
      }
      
      // 按照指定频率休眠
      std::this_thread::sleep_for(interval);
    }
  }
  

  
  // 初始化方法
  void initialize() {
    try {
      // 尝试初始化MoveGroupInterface
      try {
        // 创建MoveGroupInterface对象，指定规划组名称 - 这是与MoveIt2通信的关键接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        
        // 创建夹爪的MoveGroupInterface对象
        gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
        
        // 设置规划参数 - 这些参数影响机械臂的运动速度和加速度
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        
        use_simulation_mode_ = false;
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "服务初始化成功(实际控制模式)");
        RCLCPP_INFO(this->get_logger(), "当前运行在实际控制模式下，将通过MoveIt2控制机械臂");
      } catch (const std::exception& e) {
        // 如果MoveIt2初始化失败，则回退到模拟模式
        RCLCPP_WARN(this->get_logger(), "MoveIt2初始化失败: %s", e.what());
        RCLCPP_INFO(this->get_logger(), "将回退到模拟模式运行");
        
        use_simulation_mode_ = true;
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "服务初始化成功(模拟模式)");
        RCLCPP_INFO(this->get_logger(), "注意：当前运行在模拟模式下，不会实际控制机械臂");
        RCLCPP_INFO(this->get_logger(), "如需实际控制，请确保MoveIt2环境正确配置");
      }
      
      // 启动位置更新线程（无论模拟模式还是实际模式都启动）
      startPositionUpdateThread();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "服务初始化失败: %s", e.what());
    }
  }

  // 服务对象
  rclcpp::Service<deepseek_arm_control::srv::ControlRvizArm>::SharedPtr service_;
  // 定时器，用于延迟初始化
  rclcpp::TimerBase::SharedPtr init_timer_;
  // 上一次夹爪状态
  std::string before_gripper_state_;
  // 初始化标志
  bool initialized_;
  // 是否使用模拟模式
  bool use_simulation_mode_;
  // 停止位置更新线程标志
  std::atomic<bool> stop_position_thread_;
  // MoveGroupInterface对象指针
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  // 夹爪的MoveGroupInterface对象指针
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
  
  // 末端执行器位置订阅者
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_pose_sub_;
  // 当前末端执行器位置
  geometry_msgs::msg::Pose current_end_effector_pose_;
  // 位置更新线程
  std::thread position_update_thread_;
  // 读写锁，用于保护末端执行器位置和更新时间
  mutable std::shared_mutex pose_mutex_; // 改为读写锁，允许并发读取
  // 最后更新时间
  std::chrono::system_clock::time_point last_updated_time_;
  

};

/**
 * @brief 主函数
 */
int main(int argc, char * argv[])
{
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<ArmMoverServer>();
  
  // 使用多线程执行器而不是默认的单线程执行器，以支持多线程环境
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // 执行节点
  executor.spin();

  // 关闭ROS2
  rclcpp::shutdown();
  return 0;
}
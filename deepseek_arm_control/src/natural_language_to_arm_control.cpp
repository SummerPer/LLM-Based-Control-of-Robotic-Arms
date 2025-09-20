#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cstdlib>

// ROS2核心库
#include <rclcpp/rclcpp.hpp>
// 服务消息
#include <deepseek_arm_control/srv/control_rviz_arm.hpp>
// 位置消息
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

/**
 * @brief OpenAI API接口类
 * 通过调用Python脚本与OpenAI API进行通信，将自然语言转换为机械臂坐标
 */
class OpenAIInterface {
public:
    OpenAIInterface(const std::string& api_key, const std::string& model, const std::string& base_url = "https://api.siliconflow.cn/v1/chat/completions")
        : api_key_(api_key), model_(model), base_url_(base_url) {
        // 查找Python脚本路径
        script_path_ = findScriptPath();
        if (script_path_.empty()) {
            std::cerr << "警告: 无法找到openai_api_interface.py脚本" << std::endl;
        }
    }
    
    /**
     * @brief 将自然语言指令转换为机械臂坐标
     * @param command 自然语言指令
     * @param position 输出的坐标数组
     * @param current_position 末端执行器当前位置，可选
     * @return 是否成功转换
     */
    bool convertNaturalLanguageToPosition(const std::string& command, std::vector<double>& position, const std::vector<double>& current_position = {}) {
        if (script_path_.empty()) {
            std::cerr << "错误: Python脚本路径为空" << std::endl;
            return false;
        }
        
        // 构建调用Python脚本的命令
        std::stringstream cmd;
        cmd << "python3 " << script_path_ << " \"" << escapeQuotes(command) << "\"";
        
        // 传递空API密钥，让Python脚本从环境变量获取
        // 按照脚本参数顺序：command, api_key, model, current_position, base_url
        cmd << " \"\"";
        cmd << " \"" << escapeQuotes(model_) << "\"";
        
        // 如果提供了当前位置信息，将其添加到命令中
        if (!current_position.empty() && current_position.size() == 7) {
            try {
                // 将当前位置转换为JSON字符串
                json current_pos_json = current_position;
                std::string current_pos_str = current_pos_json.dump();
                cmd << " \"" << escapeQuotes(current_pos_str) << "\"";
            } catch (const std::exception& e) {
                std::cerr << "将当前位置转换为JSON失败: " << e.what() << std::endl;
                // 使用空JSON作为占位符
                cmd << " {}";
            }
        } else {
            // 使用空JSON作为占位符
            cmd << " {}";
        }
        
        // 添加base_url参数
        cmd << " \"" << escapeQuotes(base_url_) << "\"";
        
        std::cout << "[调试] API密钥传递: " << (api_key_.empty() ? "空" : "已设置") << std::endl;
        
        std::cout << "执行命令: " << cmd.str() << std::endl;
        
        // 执行Python脚本并获取输出
        FILE* pipe = popen(cmd.str().c_str(), "r");
        if (!pipe) {
            std::cerr << "无法执行Python脚本" << std::endl;
            return false;
        }
        
        // 读取Python脚本的输出
        char buffer[2048];
        std::string response_string;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            response_string += buffer;
        }
        
        int return_code = pclose(pipe);
        if (return_code != 0) {
            std::cerr << "Python脚本执行失败，返回代码: " << return_code << std::endl;
            return false;
        }
        
        try {
            // 解析响应
            json response_json = json::parse(response_string);
            
            if (response_json.contains("position")) {
                position = response_json["position"].get<std::vector<double>>();
                if (position.size() != 7) {
                    std::cerr << "OpenAI返回的坐标数组长度不正确" << std::endl;
                    return false;
                }
                
                // 保存夹爪状态
                if (response_json.contains("gripper")) {
                    gripper_state_ = response_json["gripper"].get<std::string>();
                }
                
                // 保存描述
                if (response_json.contains("description")) {
                    description_ = response_json["description"].get<std::string>();
                }
                
                return true;
            } else {
                std::cerr << "OpenAI响应中不包含position字段" << std::endl;
                std::cerr << "完整响应: " << response_string << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cerr << "解析OpenAI响应失败: " << e.what() << std::endl;
            std::cerr << "完整响应: " << response_string << std::endl;
            return false;
        }
    }
    
    // 获取夹爪状态
    std::string getGripperState() const {
        return gripper_state_;
    }
    
    // 获取描述
    std::string getDescription() const {
        return description_;
    }
    
    // 友元类，允许NaturalLanguageArmController访问私有成员
    friend class NaturalLanguageArmController;
    
private:
    std::string api_key_;
    std::string model_;
    std::string base_url_;
    std::string gripper_state_ = "open";
    std::string description_ = "";
    std::string script_path_;
    
    /**
     * @brief 查找Python脚本的路径
     */
    std::string findScriptPath() {
        // 尝试几个可能的路径
        std::vector<std::string> possible_paths = {
            "/home/ubuntu22/ws_moveit2/src/deepseek_arm_control/scripts/openai_api_interface.py",
            "../scripts/openai_api_interface.py",
            "scripts/openai_api_interface.py"
        };
        
        for (const auto& path : possible_paths) {
            std::ifstream file(path);
            if (file.good()) {
                file.close();
                return path;
            }
        }
        
        return "";
    }
    
    /**
     * @brief 转义字符串中的引号
     */
    std::string escapeQuotes(const std::string& str) {
        std::string result;
        for (char c : str) {
            if (c == '"') {
                result += '\\';
            }
            result += c;
        }
        return result;
    }
};

/**
 * @brief 自然语言到机械臂控制的主类
 * 实现自然语言指令处理、末端执行器位置订阅和机械臂控制功能
 */
class NaturalLanguageArmController : public rclcpp::Node {
public:
    NaturalLanguageArmController(const std::string& node_name, const std::string& api_key, const std::string& model, const std::string& base_url = "https://api.siliconflow.cn/v1/chat/completions")
        : Node(node_name), 
          current_position_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}), // 默认位置
          position_received_(false),
          openai_interface_(api_key, model, base_url) {
        
        // 创建订阅者，订阅末端执行器位置
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/end_effector_pose", 10, 
            std::bind(&NaturalLanguageArmController::poseCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "已订阅末端执行器位置话题: /end_effector_pose");
    }
    
    /**
     * @brief 处理自然语言指令
     */
    bool processNaturalLanguageCommand(const std::string& command) {
        // 等待获取到末端执行器位置
        if (!waitForCurrentPosition()) {
            RCLCPP_ERROR(this->get_logger(), "获取末端执行器位置超时");
            return false;
        }
        
        // 将自然语言转换为坐标
        std::vector<double> target_position;
        bool convert_success = this->openai_interface_.convertNaturalLanguageToPosition(
            command, target_position, current_position_);
        
        if (!convert_success) {
            RCLCPP_ERROR(this->get_logger(), "将自然语言转换为坐标失败");
            return false;
        }
        
        // 检查是否是相对移动指令（简化版，实际应根据OpenAI返回结果判断）
        if (isRelativeCommand(command)) {
            // 计算相对位置
            target_position = calculateRelativePosition(target_position);
        }
        
        // 控制机械臂移动到目标位置
        return controlArmMovement(target_position);
    }
    
    // 获取当前位置
    std::vector<double> getCurrentPosition() const {
        return current_position_;
    }
    
    // 获取OpenAI接口
    OpenAIInterface& getOpenAIInterface() {
        return openai_interface_;
    }
    
private:
    // 当前末端执行器位置
    std::vector<double> current_position_;
    // 是否已接收到位置信息
    bool position_received_;
    // 位置订阅者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    // OpenAI接口
    OpenAIInterface openai_interface_;
    
    /**
     * @brief 位置回调函数
     */
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 保存当前位置
        current_position_[0] = msg->pose.position.x;
        current_position_[1] = msg->pose.position.y;
        current_position_[2] = msg->pose.position.z;
        current_position_[3] = msg->pose.orientation.x;
        current_position_[4] = msg->pose.orientation.y;
        current_position_[5] = msg->pose.orientation.z;
        current_position_[6] = msg->pose.orientation.w;
        
        position_received_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "接收到末端执行器位置: x=%.4f, y=%.4f, z=%.4f",
                   current_position_[0], current_position_[1], current_position_[2]);
    }
    
    /**
     * @brief 等待获取当前位置
     */
    bool waitForCurrentPosition(int timeout_seconds = 5) {
        auto start_time = this->now();
        
        while (!position_received_ && 
               (this->now() - start_time).seconds() < timeout_seconds) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        return position_received_;
    }
    
    /**
     * @brief 判断是否是相对移动指令
     */
    bool isRelativeCommand(const std::string& command) {
        // 简化实现，检测关键词
        std::string lower_command = command;
        std::transform(lower_command.begin(), lower_command.end(), lower_command.begin(), 
                       [](unsigned char c){ return std::tolower(c); });
        
        // 检查是否包含相对移动的关键词
        std::vector<std::string> relative_keywords = {"向前", "向后", "向左", "向右", "向上", "向下", "移动", "偏移"};
        
        for (size_t i = 0; i < relative_keywords.size(); i++) {
            if (lower_command.find(relative_keywords[i]) != std::string::npos) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * @brief 计算相对位置
     */
    std::vector<double> calculateRelativePosition(const std::vector<double>& relative_position) {
        std::vector<double> absolute_position = current_position_;
        
        // 只对位置部分进行相对计算，姿态部分保持不变
        for (size_t i = 0; i < 3 && i < relative_position.size(); i++) {
            absolute_position[i] += relative_position[i];
        }
        
        RCLCPP_INFO(this->get_logger(), "计算相对位置: 当前[%.4f, %.4f, %.4f] + 相对[%.4f, %.4f, %.4f] = 目标[%.4f, %.4f, %.4f]",
                   current_position_[0], current_position_[1], current_position_[2],
                   relative_position[0], relative_position[1], relative_position[2],
                   absolute_position[0], absolute_position[1], absolute_position[2]);
        
        return absolute_position;
    }
    
    /**
     * @brief 控制机械臂移动
     */
    bool controlArmMovement(const std::vector<double>& target_position) {
        // 验证目标位置参数完整性
        if (target_position.size() != 7) {
            RCLCPP_ERROR(this->get_logger(), "目标位置参数不完整，需要7个元素(x,y,z,qx,qy,qz,qw)");
            return false;
        }
        
        // 创建机械臂控制客户端并发送控制请求
        rclcpp::Client<deepseek_arm_control::srv::ControlRvizArm>::SharedPtr client = 
            this->create_client<deepseek_arm_control::srv::ControlRvizArm>("control_rviz_arm");
        
        // 等待服务可用
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "客户端被中断，等待服务中...");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "服务不可用，等待中...");
        }
        
        // 创建请求对象
        auto request = std::make_shared<deepseek_arm_control::srv::ControlRvizArm::Request>();
        
        // 复制位置参数
        request->position = target_position;
        
        // 获取并验证夹爪状态
        std::string gripper_state = this->openai_interface_.getGripperState();
        if (gripper_state != "open" && gripper_state != "close") {
            RCLCPP_WARN(this->get_logger(), "夹爪状态无效: %s，默认使用 'open'", gripper_state.c_str());
            gripper_state = "open";
        }
        request->open_or_close = gripper_state;
        
        // 显示将要发送的请求信息 - 增强参数传递的可跟踪性
        RCLCPP_INFO(this->get_logger(), "发送控制请求: 位姿[x: %.4f, y: %.4f, z: %.4f, qx: %.4f, qy: %.4f, qz: %.4f, qw: %.4f], 夹爪: %s", 
                   target_position[0], target_position[1], target_position[2], 
                   target_position[3], target_position[4], target_position[5], target_position[6],
                   gripper_state.c_str());
        
        // 发送请求
        auto result_future = client->async_send_request(request);
        
        // 等待响应
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS) 
        {
            auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "机械臂控制成功！");
                RCLCPP_INFO(this->get_logger(), "路径规划已完成，机械臂开始运动。");
                RCLCPP_INFO(this->get_logger(), "OpenAI解析结果: %s", this->openai_interface_.getDescription().c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "机械臂控制失败！");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败！");
            return false;
        }
    }
};

// 辅助函数：将字符串转换为小写
std::string toLowerCase(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    return result;
}

/**
 * @brief 自然语言到机械臂控制的主程序
 * 用于将自然语言通过OpenAI转换为坐标，然后控制RViz中的机械臂
 */
int main(int argc, char **argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = rclcpp::Node::make_shared("natural_language_to_arm_control_node");
    
    // 声明并获取参数
    node->declare_parameter<std::string>("openai_api_key", "");
    node->declare_parameter<std::string>("openai_model", "deepseek-ai/DeepSeek-V3");
    node->declare_parameter<std::string>("openai_base_url", "https://api.siliconflow.cn/v1");
    
    std::string api_key = node->get_parameter("openai_api_key").as_string();
    std::string model = node->get_parameter("openai_model").as_string();
    std::string base_url = node->get_parameter("openai_base_url").as_string();
    
    std::cout << "[调试] API密钥: " << (api_key.empty() ? "空" : "已设置") << std::endl;
    std::cout << "[调试] 模型: " << model << std::endl;
    std::cout << "[调试] API基础URL: " << base_url << std::endl;
    
    // 创建控制器实例
    // 使用与节点相同的名称，确保参数能够正确传递
    auto controller = std::make_shared<NaturalLanguageArmController>(
        "natural_language_to_arm_control_node", api_key, model, base_url);
    
    // 解析命令行参数或从键盘读取自然语言指令
    std::string natural_language_command;
    bool found_command = false;
    
    // 跳过ROS2的参数部分，找到真正的自然语言指令
    for (int i = 1; i < argc; i++) {
        // 检查是否是ROS2参数标记
        if (std::string(argv[i]) == "--") {
            // 命令在"--"后面
            if (i + 1 < argc) {
                natural_language_command = argv[i + 1];
                for (int j = i + 2; j < argc; j++) {
                    natural_language_command += " " + std::string(argv[j]);
                }
                found_command = true;
            }
            break;
        }
    }
    
    // 如果没有找到命令，使用默认指令或从键盘读取
    if (!found_command) {
        if (argc > 1 && std::string(argv[1]) != "--ros-args") {
            // 直接使用第一个参数作为指令
            natural_language_command = argv[1];
            for (int i = 2; i < argc; i++) {
                natural_language_command += " " + std::string(argv[i]);
            }
        } else {
            // 使用默认指令
            natural_language_command = "移动到前面的桌子上并打开夹爪";
            std::cout << "使用默认指令: " << natural_language_command << std::endl;
        }
    }
    
    std::cout << "接收到自然语言指令: " << natural_language_command << std::endl;
    
    // 处理自然语言指令
    bool success = controller->processNaturalLanguageCommand(natural_language_command);
    
    // 关闭ROS2
    rclcpp::shutdown();
    return success ? 0 : 1;
}
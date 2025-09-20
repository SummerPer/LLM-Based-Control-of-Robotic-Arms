#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenAI API接口脚本
用于将自然语言指令转换为机械臂控制坐标，并集成ROS2功能
"""

import sys
import json
import os
import time
import requests
import rclpy
from rclpy.node import Node
from openai import OpenAI
from typing import Dict, List, Any
from geometry_msgs.msg import PoseStamped
from deepseek_arm_control.srv import ControlRvizArm

# 打印调试信息
def print_debug_info(message):
    print(f"[调试] {message}", file=sys.stderr)

class OpenAIInterface(Node):
    """OpenAI API接口类，用于将自然语言转换为机械臂控制指令，并集成ROS2功能"""
    
    def __init__(self, model: str = "deepseek-ai/DeepSeek-V3", base_url: str = "https://api.siliconflow.cn/v1"):
        """初始化OpenAI客户端和ROS2节点
        
        Args:
            model: 使用的OpenAI模型名称，默认使用deepseek-ai/DeepSeek-V3
            base_url: API端点URL，默认为https://api.siliconflow.cn/v1
        """
        # 初始化ROS2节点
        super().__init__("openai_api_interface")
        
        # 从环境变量获取API密钥
        self.api_key = os.environ.get('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError("未提供OpenAI API密钥，请通过环境变量OPENAI_API_KEY设置")
            
        self.model = model
        self.base_url = base_url
        
        self.get_logger().info(f"初始化OpenAI客户端，使用模型: {model}")
        self.get_logger().info(f"使用API端点: {base_url}")
        
        # 初始化OpenAI客户端，使用指定的base_url
        self.client = OpenAI(api_key=self.api_key, base_url=self.base_url)
        
        # 订阅末端执行器位置话题
        self.pose_topic_name = "/end_effector_pose"
        self.current_pose = None
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic_name,
            self.pose_callback,
            10
        )
        self.get_logger().info(f"订阅末端执行器位置话题: {self.pose_topic_name}")
        
        # 创建机械臂控制服务客户端
        self.arm_control_client = self.create_client(
            ControlRvizArm,
            "control_rviz_arm"
        )
        # 等待服务可用
        while not self.arm_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("机械臂控制服务不可用，等待中...")
        self.get_logger().info("已连接到机械臂控制服务")
        
        # 设置系统提示词
        self.system_prompt = (
            "你是一个机械臂控制助手，负责将自然语言指令转换为机械臂坐标。\n"
            "你的回答应该只包含一个JSON对象，格式如下：\n"
            "{\n" \
            "  \"position\": [x, y, z, qx, qy, qz, qw],\n" \
            "  \"gripper\": \"open\"或\"close\",\n" \
            "  \"description\": \"对指令的解释\"\n" \
            "}\n" \
            "其中x, y, z是位置坐标，qx, qy, qz, qw是四元数表示的姿态，gripper是夹爪状态。\n" \
            "不要输出任何其他文字！\n" \
            "如果提供了当前位置信息，请根据当前位置规划合理的运动路径。\n" \
            "现在请处理用户的指令。"
        )
        
        # 等待获取初始位置信息
        self.wait_for_initial_pose()
        
    def pose_callback(self, msg):
        """位置话题回调函数"""
        # 保存当前位置信息
        self.current_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        self.get_logger().info(f"接收到末端执行器位置: x={msg.pose.position.x:.4f}, y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}")
        
    def wait_for_initial_pose(self):
        """等待获取初始位置信息"""
        self.get_logger().info("等待获取初始末端执行器位置...")
        start_time = time.time()
        timeout = 5.0  # 5秒超时
        
        while self.current_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is None:
            self.get_logger().warning("未获取到末端执行器位置，将使用默认位置")
            # 设置默认位置
            self.current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        
    def convert_natural_language_to_position(self, command: str, current_position: List[float] = None) -> Dict[str, Any]:
        """将自然语言指令转换为机械臂坐标
        
        Args:
            command: 自然语言指令
            current_position: 末端执行器当前位置 [x, y, z, qx, qy, qz, qw]，可选
            
        Returns:
            包含位置、夹爪状态和描述的字典
        """
        try:
            # 优先使用节点中保存的当前位置，如果没有则使用传入的位置或默认位置
            pose_to_use = self.current_pose if self.current_pose is not None else (current_position if current_position is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            
            # 构建用户消息，包含当前位置信息
            user_message = command
            user_message += f"\n\n当前末端执行器位置: {pose_to_use}"
            
            self.get_logger().info(f"正在调用OpenAI API，发送指令: {command[:50]}...")
            self.get_logger().debug(f"发送的完整消息: {user_message}")
            
            # 调用OpenAI API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_message}
                ],
                response_format={"type": "json_object"},
                temperature=0.7  # 控制输出的随机性
            )
            
            # 获取响应内容
            content = response.choices[0].message.content.strip()
            self.get_logger().debug(f"OpenAI响应内容: {content}")
            # 添加INFO级别日志记录原始响应，用于调试JSON解析问题
            self.get_logger().info(f"原始OpenAI响应(前200字符): {content[:200]}...")
            
            # 解析JSON响应
            result = json.loads(content)
            
            # 验证结果格式
            if not self._validate_response(result):
                raise ValueError("OpenAI返回的响应格式不正确")
            
            return result
            
        except json.JSONDecodeError:
            self.get_logger().error("无法解析OpenAI的响应为JSON格式")
            raise
        except Exception as e:
            self.get_logger().error(f"调用OpenAI API时发生错误: {str(e)}")
            raise
            
    def send_control_request(self, position: List[float], gripper_state: str) -> bool:
        """向机械臂控制服务发送请求，严格按照ControlRvizArm服务的格式要求
        
        Args:
            position: 目标位置 [x, y, z, qx, qy, qz, qw]（严格符合ControlRvizArm服务的float64[]要求）
            gripper_state: 夹爪状态，"open"或"close"（严格符合ControlRvizArm服务的open_or_close字符串要求）
            
        Returns:
            请求是否成功
        """
        try:
            # 严格验证参数格式，确保符合ControlRvizArm服务要求
            if not isinstance(position, list) or len(position) != 7:
                self.get_logger().error(f"position参数格式错误，必须是长度为7的数组，当前: {position}")
                return False
                
            # 确保所有position元素都可以转换为浮点数
            try:
                validated_position = [float(x) for x in position]
            except (ValueError, TypeError):
                self.get_logger().error(f"position数组中包含无法转换为浮点数的元素: {position}")
                return False
                
            # 验证gripper_state参数
            if gripper_state not in ['open', 'close']:
                self.get_logger().error(f"gripper_state参数无效，必须是'open'或'close'，当前: {gripper_state}")
                return False
                
            # 检查服务是否可用
            if not self.arm_control_client.service_is_ready():
                self.get_logger().error("机械臂控制服务不可用")
                return False
                
            # 创建请求对象，严格按照ControlRvizArm服务格式设置字段
            request = ControlRvizArm.Request()
            request.position = validated_position  # 使用验证后的浮点数数组
            request.open_or_close = gripper_state  # 直接使用验证后的字符串，与服务字段名保持一致
            
            self.get_logger().info(f"发送机械臂控制请求: 位置=[{validated_position[0]:.3f}, {validated_position[1]:.3f}, {validated_position[2]:.3f}, ...], 夹爪={gripper_state}")
            
            # 发送请求并等待响应
            future = self.arm_control_client.call_async(request)
            
            # 等待响应，最多等待10秒
            start_time = time.time()
            timeout = 10.0
            while not future.done():
                if (time.time() - start_time) > timeout:
                    self.get_logger().error("控制请求超时（>10秒）")
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # 获取响应
            response = future.result()
            
            if response.success:
                self.get_logger().info("机械臂控制成功！ControlRvizArm服务已接受请求，路径规划已开始。")
                return True
            else:
                self.get_logger().error("机械臂控制失败！ControlRvizArm服务返回失败响应")
                return False
                
        except Exception as e:
            self.get_logger().error(f"发送控制请求时发生错误: {str(e)}")
            return False
    
    def _validate_response(self, response: Dict[str, Any]) -> bool:
        """验证OpenAI返回的响应格式是否严格符合ControlRvizArm服务要求
        
        Args:
            response: OpenAI返回的响应字典
            
        Returns:
            格式是否正确
        """
        # 检查必要字段是否存在
        if not all(key in response for key in ['position', 'gripper', 'description']):
            self.get_logger().error(f"响应缺少必要字段，当前字段: {list(response.keys())}")
            return False
            
        # 检查position是否为长度为7的数组（严格符合ControlRvizArm服务的7元素float64[]要求）
        if not isinstance(response['position'], list) or len(response['position']) != 7:
            self.get_logger().error(f"position格式错误，必须是长度为7的数组，当前: {response['position']}")
            return False
            
        # 检查position中的元素是否为数字（确保符合float64[]类型要求）
        try:
            # 尝试将所有元素转换为浮点数，确保类型兼容性
            for i, val in enumerate(response['position']):
                float(val)
        except (ValueError, TypeError):
            self.get_logger().error(f"position数组中包含非数字元素: {response['position']}")
            return False
            
        # 检查gripper是否为有效的值（严格符合ControlRvizArm服务的open_or_close字符串要求）
        if response['gripper'] not in ['open', 'close']:
            self.get_logger().error(f"gripper值无效，必须是'open'或'close'，当前: {response['gripper']}")
            return False
            
        return True


def main():
    """主函数，初始化ROS2，启动对话模式"""
    # 初始化ROS2
    rclpy.init(args=sys.argv)
    
    try:
        # 获取命令行参数（如果有）
        model = sys.argv[1] if len(sys.argv) > 1 else "deepseek-ai/DeepSeek-V3"
        base_url = sys.argv[2] if len(sys.argv) > 2 else None
        
        # 创建OpenAI接口实例（同时也是ROS2节点）
        if base_url:
            openai_interface = OpenAIInterface(model=model, base_url=base_url)
        else:
            openai_interface = OpenAIInterface(model=model)
        
        print("机械臂对话控制系统已启动！")
        print("输入自然语言指令控制机械臂，输入'退出'或'quit'结束会话")
        print("\n示例指令：")
        print("- 移动到桌子中央")
        print("- 向上移动10厘米")
        print("- 抓取前方物体\n")
        
        # 启动对话循环
        while True:
            try:
                # 在获取用户输入前处理ROS2回调，确保位置更新及时处理
                rclpy.spin_once(openai_interface, timeout_sec=0.1)
                # 获取用户输入
                command = input("请输入指令: ")
                
                # 检查退出条件
                if command.lower() in ['退出', 'quit', 'exit']:
                    print("感谢使用机械臂对话控制系统，再见！")
                    break
                
                # 处理空输入
                if not command.strip():
                    continue
                
                # 转换自然语言指令为机械臂坐标
                openai_interface.get_logger().info(f"处理自然语言指令: {command}")
                result = openai_interface.convert_natural_language_to_position(command)
                
                # 输出结果到控制台
                openai_interface.get_logger().info(f"OpenAI处理结果: {json.dumps(result)}")
                print(f"\n执行结果: {result['description']}")
                print(f"目标位置: {result['position'][:3]}")
                print(f"夹爪状态: {result['gripper']}\n")
                
                # 发送控制请求给机械臂
                success = openai_interface.send_control_request(result['position'], result['gripper'])
                
                # 控制请求发送后，增加回调处理次数并延长等待时间，确保机械臂到达目标位置后位置数据能被更新
                if success:
                    # 机械臂移动需要时间，增加回调处理次数和等待时间
                    openai_interface.get_logger().info("等待机械臂移动到位并更新位置信息...")
                    # 保存目标位置信息（取位置的前三个坐标：x, y, z）
                    target_position = result['position'][:3]
                    
                    # 先等待1秒让机械臂开始移动
                    time.sleep(1.0)
                    
                    # 首先处理30次回调以获取初始的位置更新
                    for i in range(30):
                        rclpy.spin_once(openai_interface, timeout_sec=0.1)
                        # 每5次回调打印一次当前位置，便于调试
                        if i % 5 == 0 and openai_interface.current_pose:
                            openai_interface.get_logger().info(f"更新位置检查 (第{i+1}/30次): x={openai_interface.current_pose[0]:.4f}, y={openai_interface.current_pose[1]:.4f}, z={openai_interface.current_pose[2]:.4f}")
                    
                    # 设置目标位置差异阈值（移除超时机制）
                    target_threshold = 0.005  # 与目标位置的差异阈值（米）
                    
                    openai_interface.get_logger().info(f"开始监控与目标位置的差异，阈值={target_threshold}米，目标位置: x={target_position[0]:.4f}, y={target_position[1]:.4f}, z={target_position[2]:.4f}")
                    
                    # 然后持续监控当前位置与目标位置的差异（移除超时机制，一直等待直到到达目标位置）
                    while True:
                        rclpy.spin_once(openai_interface, timeout_sec=0.1)
                        
                        # 检查是否获取到了位置信息
                        if not openai_interface.current_pose:
                            time.sleep(0.1)
                            continue
                        
                        # 计算当前位置与目标位置的差异（只考虑x, y, z三个坐标）
                        current_position = openai_interface.current_pose[:3]
                        position_diff = sum(
                            (current_position[i] - target_position[i]) ** 2 
                            for i in range(3)
                        ) ** 0.5  # 欧氏距离
                        
                        # 记录当前状态
                        openai_interface.get_logger().debug(f"当前位置: {current_position}, 与目标位置差异: {position_diff:.6f}米")
                        
                        # 检查是否已经接近目标位置
                        if position_diff < target_threshold:
                            openai_interface.get_logger().info(f"已接近目标位置！差异={position_diff:.6f}米 < 阈值={target_threshold}米")
                            break
                    
                    openai_interface.get_logger().info(f"位置监控结束，最终位置: x={openai_interface.current_pose[0]:.4f}, y={openai_interface.current_pose[1]:.4f}, z={openai_interface.current_pose[2]:.4f}")
                
                if not success:
                    print("警告: 控制请求失败，请重试\n")
                    
            except KeyboardInterrupt:
                print("\n程序被用户中断")
                break
            except Exception as e:
                print(f"处理指令时发生错误: {str(e)}")
                print("请尝试重新输入指令或检查系统状态")
        
        sys.exit(0)
        
    except Exception as e:
        # 输出错误信息
        print(f"错误: {str(e)}", file=sys.stderr)
        sys.exit(1)
    finally:
        # 确保关闭ROS2
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
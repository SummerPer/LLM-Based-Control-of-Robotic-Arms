# DeepSeek直接控制机械臂框架

## 项目简介

本框架提供了一个通过DeepSeek AI直接控制机械臂的解决方案。用户可以通过自然语言指令，让DeepSeek AI生成机械臂的目标位姿，然后直接控制机械臂运动到指定位置。整个控制流程集成在一个节点中，无需额外的中间转换步骤。

## 系统架构

![系统架构](https://placeholder-for-architecture-diagram.com)

框架主要由以下组件构成：

1. **deepseek_arm_direct_control_node**：核心控制节点，整合了DeepSeek API调用和机械臂控制功能
2. **arm_mover_server_node**：机械臂控制服务服务器，处理实际的机械臂运动控制
3. **end_effector_monitor_node**：末端执行器位置监控节点，提供位置反馈
4. **move_group_node**：MoveIt!运动规划节点，提供路径规划功能

## 快速开始

### 前提条件

1. 已安装ROS 2 Humble及以上版本
2. 已安装MoveIt! 2
3. 已配置好机械臂的描述文件和控制器
4. 已获取DeepSeek API密钥

### 配置步骤

1. 在项目目录中打开终端
2. 配置DeepSeek API密钥：
   ```bash
   export DEEPSEEK_API_KEY=your_api_key_here
   ```

### 启动框架

使用以下命令启动整个框架：

```bash
ros2 launch deepseek_arm_control deepseek_direct_control.launch.py deepseek_api_key:=$DEEPSEEK_API_KEY use_simulation_mode:=false
```

启动参数说明：
- `deepseek_api_key`：DeepSeek API密钥
- `use_simulation_mode`：是否使用模拟模式（`false`表示实际控制机械臂）
- `use_sim_time`：是否使用仿真时间（默认：`true`）
- `deepseek_model`：DeepSeek模型名称（默认：`deepseek-chat`）

## 使用方法

1. 框架启动后，会自动打开一个新的终端窗口用于输入自然语言指令
2. 在该终端中输入自然语言指令，例如：
   ```
   向前移动10厘米
   ```
   或
   ```
   移动到坐标点(0.5, 0.3, 0.4)，四元数保持不变
   ```
3. DeepSeek AI会解析指令并生成目标位姿
4. 机械臂会自动规划路径并移动到目标位置
5. 系统会提供移动结果反馈

## 节点详解

### deepseek_arm_direct_control_node

这是整个框架的核心节点，主要功能包括：
- 接收用户输入的自然语言指令
- 调用DeepSeek API将自然语言转换为机械臂位姿
- 发送控制请求到`arm_mover_server_node`
- 监控机械臂是否到达目标位置
- 提供操作反馈

### 主要类和方法

#### DeepSeekApiClient类
- `convertNaturalLanguageToPosition()`：将自然语言指令转换为机械臂位姿坐标
- `parseApiResponse()`：解析DeepSeek API的响应

#### EndEffectorPositionListener类
- `hasReachedPosition()`：检查机械臂是否到达目标位置
- `getCurrentPosition()`：获取当前机械臂末端执行器位置

#### DeepSeekArmDirectControlNode类
- `runInteractiveLoop()`：运行交互式命令输入循环
- `handleControlRequest()`：处理控制请求并发送到机械臂控制服务
- `isRelativeCommand()`：判断指令是否为相对移动指令

## 常见问题和解决方案

1. **问题**：DeepSeek API调用失败
   **解决方案**：检查API密钥是否正确，并确保网络连接正常

2. **问题**：机械臂无法到达目标位置
   **解决方案**：检查指令是否合理，可能是因为碰撞或关节限制

3. **问题**：位置精度不够
   **解决方案**：可以修改`confirmPositionReached()`方法中的阈值参数

## 安全注意事项

1. 在实际控制机械臂前，请确保工作环境安全，清除可能的障碍物
2. 建议先在模拟环境中测试指令，再实际控制机械臂
3. 如发现异常情况，请立即停止程序运行

## 扩展开发

如需扩展或修改功能，可以参考以下文件：
- `src/deepseek_arm_direct_control.cpp`：核心控制逻辑
- `launch/deepseek_direct_control.launch.py`：启动配置
- `CMakeLists.txt`：编译配置

## 联系信息

如有任何问题或建议，请联系项目维护人员。

---

更新日期：2024年6月
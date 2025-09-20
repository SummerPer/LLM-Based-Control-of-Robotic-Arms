# deepseek_arm_control 包

## 概述

这是一个基于ROS2的机械臂控制包，集成了DeepSeek API，用于实现自然语言控制机械臂的功能。目前已实现了末端位置监控模块，后续将逐步添加更多功能。

## 功能模块

### 末端位置监控器 (EndEffectorMonitor)

实时监控机械臂末端执行器的位置和姿态，并将信息发布到ROS2话题中。

## 安装说明

### 前提条件
- ROS2 Humble 或更高版本
- CMake 3.8 或更高版本
- C++ 17 兼容的编译器

### 编译步骤

1. 克隆或下载本包到您的ROS2工作空间的src目录
```bash
cd ~/ws_moveit2/src
# 克隆或复制本包到此处
```

2. 编译工作空间
```bash
cd ~/ws_moveit2
colcon build --packages-select deepseek_arm_control
```

3. 激活工作空间
```bash
source install/setup.bash
```

## 使用方法

### 方式一：直接运行可执行文件

```bash
ros2 run deepseek_arm_control end_effector_monitor_node
```

### 方式二：使用启动文件

```bash
ros2 launch deepseek_arm_control end_effector_monitor.launch.py
```

## 配置参数

末端位置监控器支持以下配置参数：

| 参数名称 | 类型 | 默认值 | 描述 |
|---------|------|-------|------|
| `target_frame` | string | `panda_link8` | 末端执行器坐标系名称 |
| `source_frame` | string | `panda_link0` | 基础参考坐标系名称 |
| `update_frequency` | double | 20.0 | 位置更新频率（Hz） |
| `pose_topic_name` | string | `/end_effector_pose` | 位置发布话题名称 |
| `use_sim_time` | bool | false | 是否使用模拟时间 |

可以通过命令行参数或修改launch文件来配置这些参数。

## 话题接口

### 发布的话题
- `/end_effector_pose` (`geometry_msgs::msg::PoseStamped`): 发布末端执行器的位置和姿态信息

## 代码结构

```
deepseek_arm_control/
├── include/deepseek_arm_control/   # 头文件目录
│   └── end_effector_monitor.hpp    # 末端位置监控器头文件
├── src/                            # 源代码目录
│   ├── end_effector_monitor.cpp    # 末端位置监控器实现
│   └── main.cpp                    # 程序入口
├── launch/                         # 启动文件目录
│   └── end_effector_monitor.launch.py  # 末端位置监控器启动文件
├── CMakeLists.txt                  # CMake配置文件
├── package.xml                     # ROS2包配置文件
└── README.md                       # 说明文档
```

## 后续计划

1. 实现自然语言控制服务节点
2. 集成DeepSeek API客户端
3. 实现MoveIt2运动规划功能
4. 添加更多示例和文档

## 许可证

本包使用Apache License 2.0许可证
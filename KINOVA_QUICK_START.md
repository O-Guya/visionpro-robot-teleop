# Kinova Gen3 仿真快速启动指南

这是最简单、最直接的方法来启动和控制 Kinova Gen3 机械臂仿真。

---

## 🚀 快速开始（3步）

### 第一步：安装 ROS2

如果还没安装 ROS2，请根据你的系统安装：

```bash
# Ubuntu 22.04 - ROS2 Humble
sudo apt install ros-humble-desktop-full

# Ubuntu 24.04 - ROS2 Jazzy
sudo apt install ros-jazzy-desktop-full
```

安装后，设置环境：
```bash
source /opt/ros/humble/setup.bash  # 或 jazzy
```

### 第二步：启动仿真

```bash
# 给启动脚本添加可执行权限
chmod +x start_kinova_sim.sh

# 运行启动脚本（自动处理所有设置）
./start_kinova_sim.sh
```

这个脚本会自动：
- 检查依赖
- 安装需要的包
- 构建 ros2_kortex
- 启动 Gazebo 仿真和 RViz 可视化

### 第三步：控制机械臂

打开新终端，运行控制脚本：

```bash
# 设置 ROS2 环境
source /opt/ros/humble/setup.bash  # 或你的 ROS2 版本
source ros2_kortex/install/setup.bash

# 运行控制演示
python3 simple_kinova_control.py
```

---

## 📋 手动启动（如果你想分步操作）

### 1. 只启动仿真环境（不启动 RViz）

```bash
source ros2_kortex/install/setup.bash

ros2 launch kortex_bringup kortex_sim_control.launch.py \
    use_sim_time:=true \
    launch_rviz:=false \
    robot_controller:=joint_trajectory_controller
```

### 2. 单独启动 RViz

```bash
ros2 launch kortex_description view_robot.launch.py
```

### 3. 手动发送关节命令（测试）

```bash
# 移动到零位
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 5 } },
  ]
}"
```

---

## 🎮 控制方法

### 方法 1: Python 脚本（推荐，最简单）

使用 `simple_kinova_control.py` 脚本：

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 发送关节位置命令
controller.move_to_position([0.0, 0.3, 0.0, -1.5, 0.0, 0.5, 0.0])
```

### 方法 2: ROS2 命令行

直接用 `ros2 topic pub` 发送命令：

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/JointTrajectory "..."
```

### 方法 3: 使用 MoveIt（高级运动规划）

```bash
# 启动 MoveIt 仿真
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config sim.launch.py \
    use_sim_time:=true
```

---

## 🔧 配置选项

### 修改机械臂配置

编辑 `start_kinova_sim.sh` 中的参数：

```bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
    dof:=6 \                          # 6自由度或7自由度
    gripper:=robotiq_2f_140 \         # 更换夹爪型号
    launch_rviz:=false                # 不启动RViz
```

可用选项：
- **dof**: `6` 或 `7` (默认: 7)
- **gripper**: `robotiq_2f_85`, `robotiq_2f_140`, 或空字符串（无夹爪）
- **robot_type**: `gen3` 或 `gen3_lite`

---

## 📦 关节限制

Kinova Gen3 7DoF 关节范围（弧度）：

| 关节 | 最小值 | 最大值 |
|------|--------|--------|
| joint_1 | -π | +π |
| joint_2 | -2.41 | +2.41 |
| joint_3 | -π | +π |
| joint_4 | -2.66 | +2.66 |
| joint_5 | -π | +π |
| joint_6 | -2.23 | +2.23 |
| joint_7 | -π | +π |

---

## ❓ 常见问题

### Q: 仿真启动失败？
**A:** 确保已安装 Gazebo Harmonic：
```bash
# 安装 Gazebo
sudo apt install gz-harmonic
```

### Q: 找不到 ros2 命令？
**A:** 确保已 source ROS2 环境：
```bash
source /opt/ros/humble/setup.bash
```

### Q: Python 脚本报错？
**A:** 确保已安装 rclpy：
```bash
pip3 install rclpy
```

### Q: 控制器没有响应？
**A:** 检查控制器是否激活：
```bash
ros2 control list_controllers
```

---

## 🎯 下一步

现在你已经可以控制仿真机械臂了！可以尝试：

1. **集成 Vision Pro 数据**：修改 `simple_kinova_control.py` 读取 Vision Pro 手部追踪数据
2. **使用 MoveIt**：进行高级运动规划和避障
3. **添加传感器**：添加相机和力传感器
4. **真机测试**：在真实机械臂上运行（需要修改 `robot_ip` 参数）

---

## 📚 更多资源

- [ros2_kortex 官方文档](https://github.com/Kinovarobotics/ros2_kortex)
- [Kinova Gen3 用户手册](https://www.kinovarobotics.com/product/gen3-robots)
- [ROS2 教程](https://docs.ros.org/en/humble/Tutorials.html)

---

**祝你使用愉快！** 🎉

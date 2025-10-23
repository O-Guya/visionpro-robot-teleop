#!/bin/bash

# Kinova Gen3 仿真启动脚本
# 这是最简单的启动方法

echo "================================"
echo "Kinova Gen3 仿真环境启动脚本"
echo "================================"

# 检查是否安装了 ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 没有检测到 ROS2 环境"
    echo "请先运行: source /opt/ros/<your-distro>/setup.bash"
    exit 1
fi

echo "检测到 ROS2 发行版: $ROS_DISTRO"

# 检查子模块是否初始化
if [ ! -f "ros2_kortex/README.md" ]; then
    echo "错误: ros2_kortex 子模块未初始化"
    echo "请运行: git submodule update --init --recursive"
    exit 1
fi

echo "================================"
echo "步骤 1/3: 安装依赖（如果需要）"
echo "================================"

# 安装依赖
echo "正在安装 ROS2 依赖包..."
rosdep install --ignore-src --from-paths ros2_kortex -y -r 2>/dev/null || echo "依赖已安装或部分跳过"

echo ""
echo "================================"
echo "步骤 2/3: 构建工作空间"
echo "================================"

# 构建 ros2_kortex
if [ ! -d "ros2_kortex/install" ]; then
    echo "首次构建 ros2_kortex（这可能需要几分钟）..."
    cd ros2_kortex
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
    cd ..
else
    echo "ros2_kortex 已构建，跳过..."
fi

# Source 工作空间
echo "设置环境变量..."
source ros2_kortex/install/setup.bash

echo ""
echo "================================"
echo "步骤 3/3: 启动仿真"
echo "================================"
echo ""
echo "正在启动 Kinova Gen3 7DoF 机械臂仿真..."
echo "配置:"
echo "  - 机器人: Gen3 7自由度"
echo "  - 夹爪: Robotiq 2F-85"
echo "  - 仿真器: Gazebo"
echo "  - 控制器: Joint Trajectory Controller"
echo ""
echo "仿真启动后，你可以在另一个终端运行控制脚本"
echo "运行: python simple_kinova_control.py"
echo ""

# 启动仿真
ros2 launch kortex_bringup kortex_sim_control.launch.py \
    use_sim_time:=true \
    launch_rviz:=true \
    robot_controller:=joint_trajectory_controller \
    dof:=7 \
    gripper:=robotiq_2f_85

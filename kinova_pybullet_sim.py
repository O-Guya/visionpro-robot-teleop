#!/usr/bin/env python3
"""
Kinova Gen3 PyBullet 仿真 - 完全跳过 ROS2
直接使用 ros2_kortex 的 URDF 文件进行仿真和控制

可在 macOS/Linux/Windows 上运行
"""

import pybullet as p
import pybullet_data
import time
import math
import os
import numpy as np


class KinovaGen3Sim:
    def __init__(self, gui=True):
        """初始化 PyBullet 仿真环境"""

        # 连接物理引擎
        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

        # 设置搜索路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 设置物理参数
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1./240.)

        # 加载地面
        self.plane_id = p.loadURDF("plane.urdf")

        # 加载 Kinova Gen3 机械臂
        self.load_robot()

        # 获取关节信息
        self.setup_joints()

        # 设置相机
        self.setup_camera()

        print(f"✅ Kinova Gen3 仿真已启动！")
        print(f"   - 自由度: {len(self.controllable_joints)}")
        print(f"   - 总关节数: {self.num_joints}")

    def load_robot(self):
        """加载机械臂 URDF"""
        # 获取 URDF 路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(
            script_dir,
            "ros2_kortex/kortex_description/robots/gen3_2f85.urdf"
        )

        if not os.path.exists(urdf_path):
            # 尝试备用路径（无夹爪版本）
            urdf_path = os.path.join(
                script_dir,
                "ros2_kortex/kortex_description/robots/gen3_lite.urdf"
            )

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(
                f"找不到 URDF 文件！请确保 ros2_kortex 子模块已初始化。\n"
                f"尝试的路径: {urdf_path}"
            )

        # 改变工作目录以正确加载 mesh 文件
        original_dir = os.getcwd()
        os.chdir(os.path.join(script_dir, "ros2_kortex/kortex_description"))

        # 加载机器人
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        self.robot_id = p.loadURDF(
            urdf_path,
            start_pos,
            start_orientation,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION
        )

        # 恢复工作目录
        os.chdir(original_dir)

        print(f"✅ 已加载 URDF: {urdf_path}")

    def setup_joints(self):
        """设置关节信息"""
        self.num_joints = p.getNumJoints(self.robot_id)
        self.controllable_joints = []
        self.joint_names = []
        self.joint_limits = []

        print("\n关节信息:")
        print("-" * 80)

        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]

            # 只控制旋转关节 (JOINT_REVOLUTE)
            if joint_type == p.JOINT_REVOLUTE:
                self.controllable_joints.append(i)
                self.joint_names.append(joint_name)

                # 获取关节限制
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                self.joint_limits.append((lower_limit, upper_limit))

                print(f"  [{len(self.controllable_joints)-1}] {joint_name:30s} "
                      f"范围: [{lower_limit:6.2f}, {upper_limit:6.2f}]")

        print("-" * 80)

    def setup_camera(self):
        """设置相机视角"""
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=50,
            cameraPitch=-35,
            cameraTargetPosition=[0, 0, 0.5]
        )

    def get_joint_states(self):
        """获取当前关节状态"""
        states = p.getJointStates(self.robot_id, self.controllable_joints)
        positions = [state[0] for state in states]
        velocities = [state[1] for state in states]
        return positions, velocities

    def move_to_joint_positions(self, target_positions, max_force=200):
        """
        移动到目标关节位置

        参数:
            target_positions: 目标关节角度列表（弧度）
            max_force: 最大力矩
        """
        if len(target_positions) != len(self.controllable_joints):
            raise ValueError(
                f"目标位置数量 ({len(target_positions)}) "
                f"与可控关节数量 ({len(self.controllable_joints)}) 不匹配"
            )

        # 使用位置控制
        p.setJointMotorControlArray(
            self.robot_id,
            self.controllable_joints,
            p.POSITION_CONTROL,
            targetPositions=target_positions,
            forces=[max_force] * len(self.controllable_joints)
        )

    def step(self):
        """单步仿真"""
        p.stepSimulation()

    def run_demo(self):
        """运行演示程序"""
        print("\n" + "="*80)
        print("开始演示 - 机械臂将执行预定义动作")
        print("按 Ctrl+C 停止")
        print("="*80 + "\n")

        # 定义一些预设姿态
        poses = {
            "home": [0, 0, 0, 0, 0, 0, 0],
            "pose_1": [0, 0.3, 0, -1.5, 0, 0.5, 0],
            "pose_2": [0.5, -0.3, 0.5, -1.0, 0.3, 1.0, 0.2],
            "pose_3": [-0.5, 0.2, -0.3, -0.8, -0.5, 0.8, -0.3],
        }

        try:
            while True:
                # 循环执行各个姿态
                for pose_name, pose in poses.items():
                    print(f"→ 移动到: {pose_name}")

                    # 确保姿态长度匹配
                    if len(pose) < len(self.controllable_joints):
                        pose = pose + [0] * (len(self.controllable_joints) - len(pose))
                    elif len(pose) > len(self.controllable_joints):
                        pose = pose[:len(self.controllable_joints)]

                    # 发送目标位置
                    self.move_to_joint_positions(pose)

                    # 运行仿真直到到达目标
                    for _ in range(240 * 3):  # 3秒
                        self.step()
                        time.sleep(1./240.)

                    time.sleep(1)  # 停顿1秒

        except KeyboardInterrupt:
            print("\n\n演示已停止")

    def interactive_control(self):
        """交互式控制 - 使用滑动条"""
        print("\n" + "="*80)
        print("交互式控制模式")
        print("使用滑动条控制每个关节")
        print("关闭窗口或按 Ctrl+C 退出")
        print("="*80 + "\n")

        # 创建调试滑动条
        sliders = []
        for i, (joint_idx, joint_name) in enumerate(zip(self.controllable_joints, self.joint_names)):
            lower, upper = self.joint_limits[i]

            slider_id = p.addUserDebugParameter(
                joint_name,
                lower,
                upper,
                0  # 初始值
            )
            sliders.append(slider_id)

        try:
            while True:
                # 读取滑动条值
                target_positions = []
                for slider_id in sliders:
                    value = p.readUserDebugParameter(slider_id)
                    target_positions.append(value)

                # 移动到目标位置
                self.move_to_joint_positions(target_positions)

                # 单步仿真
                self.step()
                time.sleep(1./240.)

        except KeyboardInterrupt:
            print("\n\n交互控制已停止")

    def close(self):
        """关闭仿真"""
        p.disconnect()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Kinova Gen3 PyBullet 仿真')
    parser.add_argument('--mode', type=str, default='demo',
                        choices=['demo', 'interactive'],
                        help='运行模式: demo=自动演示, interactive=手动控制')
    parser.add_argument('--no-gui', action='store_true',
                        help='不显示GUI（仅用于测试）')

    args = parser.parse_args()

    print("\n" + "="*80)
    print("Kinova Gen3 PyBullet 仿真")
    print("无需 ROS2 - 直接使用 URDF 文件")
    print("="*80 + "\n")

    # 创建仿真器
    sim = KinovaGen3Sim(gui=not args.no_gui)

    try:
        if args.mode == 'demo':
            sim.run_demo()
        elif args.mode == 'interactive':
            sim.interactive_control()
    finally:
        sim.close()


if __name__ == '__main__':
    main()

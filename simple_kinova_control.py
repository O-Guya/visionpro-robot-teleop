#!/usr/bin/env python3
"""
Kinova Gen3 简单控制脚本
这是最简单直接的控制方法 - 通过发布 Joint Trajectory 消息控制机械臂
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


class SimpleKinovaController(Node):
    def __init__(self):
        super().__init__('simple_kinova_controller')

        # 创建发布器到 joint_trajectory_controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # 等待发布器准备就绪
        time.sleep(1)
        self.get_logger().info('Kinova Gen3 控制器已准备就绪！')

    def move_to_position(self, positions, duration_sec=5.0):
        """
        移动机械臂到指定的关节位置

        参数:
            positions: 7个关节的目标位置（弧度）
            duration_sec: 移动持续时间（秒）
        """
        msg = JointTrajectory()
        msg.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7'
        ]

        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)

        msg.points.append(point)

        # 发布命令
        self.publisher.publish(msg)
        self.get_logger().info(f'发送位置命令: {positions}')


def main():
    rclpy.init()
    controller = SimpleKinovaController()

    print("\n" + "="*50)
    print("Kinova Gen3 简单控制演示")
    print("="*50)

    try:
        # 演示 1: 回到零位
        print("\n[演示 1/4] 移动到零位（Home Position）...")
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        controller.move_to_position(home_position, duration_sec=5.0)
        time.sleep(6)

        # 演示 2: 移动到预定义姿态 1
        print("\n[演示 2/4] 移动到姿态 1...")
        position_1 = [0.0, 0.3, 0.0, -1.5, 0.0, 0.5, 0.0]
        controller.move_to_position(position_1, duration_sec=5.0)
        time.sleep(6)

        # 演示 3: 移动到预定义姿态 2
        print("\n[演示 3/4] 移动到姿态 2...")
        position_2 = [0.5, -0.3, 0.5, -1.0, 0.3, 1.0, 0.2]
        controller.move_to_position(position_2, duration_sec=5.0)
        time.sleep(6)

        # 演示 4: 回到零位
        print("\n[演示 4/4] 返回零位...")
        controller.move_to_position(home_position, duration_sec=5.0)
        time.sleep(6)

        print("\n" + "="*50)
        print("演示完成！")
        print("="*50)
        print("\n提示：你可以修改这个脚本来自定义机械臂的运动")
        print("关节位置范围建议: -π 到 +π 弧度")

    except KeyboardInterrupt:
        print("\n\n控制器已停止")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

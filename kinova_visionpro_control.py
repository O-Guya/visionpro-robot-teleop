#!/usr/bin/env python3
"""
Kinova Gen3 + Vision Pro 遥操作
结合 PyBullet 仿真和 Vision Pro 手部追踪

完全跳过 ROS2！
"""

import sys
import time
import numpy as np
from kinova_pybullet_sim import KinovaGen3Sim


class VisionProKinovaController:
    """
    Vision Pro 到 Kinova 的映射控制器
    """
    def __init__(self, sim):
        self.sim = sim
        self.avp_streamer = None

    def connect_visionpro(self, ip):
        """连接 Vision Pro"""
        try:
            from avp_stream import VisionProStreamer
            print(f"正在连接 Vision Pro: {ip}")
            self.avp_streamer = VisionProStreamer(ip=ip, record=False)
            print("✅ Vision Pro 已连接！")
            return True
        except ImportError:
            print("❌ 错误: 找不到 avp_stream 模块")
            print("请先安装: pip install avp_stream")
            return False
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False

    def hand_to_joint_positions(self, hand_data):
        """
        将手部追踪数据映射到机械臂关节角度

        这是一个简单的映射示例，你可以根据需要自定义

        参数:
            hand_data: Vision Pro 手部数据字典

        返回:
            joint_positions: 7个关节的目标角度
        """
        # 提取右手腕位置和方向
        right_wrist = hand_data.get('right_wrist', np.eye(4))
        right_pinch = hand_data.get('right_pinch_distance', 0.1)

        # 简单映射示例
        # 实际应用中，你可能需要使用逆运动学
        pos = right_wrist[:3, 3]  # 位置
        rot = right_wrist[:3, :3]  # 旋转矩阵

        # 从变换矩阵提取欧拉角
        # 这是一个简化的映射 - 将手腕位置映射到关节角度
        joint_positions = [
            np.clip(pos[0] * 2, -np.pi, np.pi),      # joint_1: 基座旋转
            np.clip(pos[1] * 1.5, -2.41, 2.41),      # joint_2: 肩部
            np.clip(pos[2] * 2, -np.pi, np.pi),      # joint_3: 肘部旋转
            np.clip(-pos[1] * 2, -2.66, 2.66),       # joint_4: 肘部俯仰
            np.clip(pos[0] * 1.5, -np.pi, np.pi),    # joint_5: 腕部旋转1
            np.clip(pos[2] * 1.5, -2.23, 2.23),      # joint_6: 腕部俯仰
            np.clip(right_pinch * 3, -np.pi, np.pi)  # joint_7: 腕部旋转2
        ]

        return joint_positions

    def run(self):
        """运行 Vision Pro 控制循环"""
        if self.avp_streamer is None:
            print("❌ 错误: Vision Pro 未连接")
            return

        print("\n" + "="*80)
        print("Vision Pro → Kinova Gen3 遥操作")
        print("移动你的右手来控制机械臂")
        print("按 Ctrl+C 停止")
        print("="*80 + "\n")

        try:
            while True:
                # 获取最新的 Vision Pro 数据
                data = self.avp_streamer.latest

                if data is not None:
                    # 将手部数据映射到关节位置
                    joint_positions = self.hand_to_joint_positions(data)

                    # 控制机械臂
                    self.sim.move_to_joint_positions(joint_positions)

                # 单步仿真
                self.sim.step()
                time.sleep(1./240.)

        except KeyboardInterrupt:
            print("\n\n遥操作已停止")


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='Kinova Gen3 + Vision Pro 遥操作（无需ROS2）'
    )
    parser.add_argument('--ip', type=str, default=None,
                        help='Vision Pro IP 地址')
    parser.add_argument('--demo', action='store_true',
                        help='运行演示模式（不连接 Vision Pro）')

    args = parser.parse_args()

    print("\n" + "="*80)
    print("Kinova Gen3 + Vision Pro 遥操作")
    print("完全跳过 ROS2 - PyBullet 直接仿真")
    print("="*80 + "\n")

    # 创建仿真器
    sim = KinovaGen3Sim(gui=True)

    if args.demo:
        # 演示模式
        print("演示模式 - 运行预定义动作")
        sim.run_demo()
    elif args.ip:
        # Vision Pro 控制模式
        controller = VisionProKinovaController(sim)

        if controller.connect_visionpro(args.ip):
            controller.run()
        else:
            print("\n使用演示模式代替...")
            sim.run_demo()
    else:
        # 交互式控制
        print("交互式控制模式")
        sim.interactive_control()

    sim.close()


if __name__ == '__main__':
    main()

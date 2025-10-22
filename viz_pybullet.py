#!/usr/bin/env python3
"""
PyBullet版Vision Pro 3D可视化
轻量级替代Isaac Gym，支持Python 3.10+
"""
import pybullet as p
import pybullet_data
import numpy as np
import time
import threading
from avp_stream import VisionProStreamer
import argparse


class PyBulletVisualizer:
    # 手指骨架连接关系 (标准手部骨架21关节)
    FINGER_CONNECTIONS = [
        # 手腕到各指根部
        (0, 1),
        (0, 5),
        (0, 9),
        (0, 13),
        (0, 17),
        # 拇指
        (1, 2),
        (2, 3),
        (3, 4),
        # 食指
        (5, 6),
        (6, 7),
        (7, 8),
        # 中指
        (9, 10),
        (10, 11),
        (11, 12),
        # 无名指
        (13, 14),
        (14, 15),
        (15, 16),
        # 小指
        (17, 18),
        (18, 19),
        (19, 20),
    ]

    def __init__(self, ip, record=False, follow_camera=True):
        self.ip = ip
        self.record = record
        self.follow_camera = follow_camera
        self.streamer = VisionProStreamer(ip, record)

        # 初始化PyBullet
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 设置物理参数
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)  # 240 Hz

        # 创建环境
        self.setup_environment()

        # 创建几何体
        self.setup_geometry()

        # 存储连接线ID
        self.right_finger_lines = []
        self.left_finger_lines = []

        # 运行状态
        self.running = True

    def setup_environment(self):
        """设置3D环境"""
        # 添加地面
        p.loadURDF("plane.urdf")

        # 设置相机参数
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=0,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0],
        )

    def setup_geometry(self):
        """创建3D几何体"""
        # 创建头部几何体 (大球体)
        head_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
        self.head_body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=head_shape,
            baseVisualShapeIndex=-1,
            basePosition=[0, 0, 0],
        )
        p.changeVisualShape(self.head_body, -1, rgbaColor=[1, 0, 0, 1])  # 红色

        # 创建右手腕几何体
        right_wrist_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
        self.right_wrist_body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=right_wrist_shape,
            baseVisualShapeIndex=-1,
            basePosition=[0, 0, 0],
        )
        p.changeVisualShape(self.right_wrist_body, -1, rgbaColor=[0, 1, 0, 1])  # 绿色

        # 创建左手腕几何体
        left_wrist_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
        self.left_wrist_body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=left_wrist_shape,
            baseVisualShapeIndex=-1,
            basePosition=[0, 0, 0],
        )
        p.changeVisualShape(self.left_wrist_body, -1, rgbaColor=[0, 0, 1, 1])  # 蓝色

        # 创建手指关节几何体
        self.right_finger_bodies = []
        self.left_finger_bodies = []

        finger_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.01)

        for i in range(25):
            # 右手关节
            body = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=finger_shape,
                baseVisualShapeIndex=-1,
                basePosition=[0, 0, 0],
            )
            if i in [0, 4, 9, 14, 19, 24]:  # 关键关节
                p.changeVisualShape(body, -1, rgbaColor=[1, 1, 0, 1])  # 黄色
            else:
                p.changeVisualShape(body, -1, rgbaColor=[0.8, 0.8, 0.8, 1])  # 灰色
            self.right_finger_bodies.append(body)

            # 左手关节
            body = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=finger_shape,
                baseVisualShapeIndex=-1,
                basePosition=[0, 0, 0],
            )
            if i in [0, 4, 9, 14, 19, 24]:  # 关键关节
                p.changeVisualShape(body, -1, rgbaColor=[1, 1, 0, 1])  # 黄色
            else:
                p.changeVisualShape(body, -1, rgbaColor=[0.8, 0.8, 0.8, 1])  # 灰色
            self.left_finger_bodies.append(body)

    def update_head(self, head_matrix):
        """更新头部位置和姿态"""
        head_pos = head_matrix[0, :3, 3]
        head_rot = head_matrix[0, :3, :3]

        # 转换旋转矩阵为四元数
        quat = self.matrix_to_quaternion(head_rot)

        # 更新位置和姿态
        p.resetBasePositionAndOrientation(self.head_body, head_pos, quat)

    def update_wrists(self, right_wrist_matrix, left_wrist_matrix):
        """更新手腕位置和姿态"""
        right_pos = right_wrist_matrix[0, :3, 3]
        right_rot = right_wrist_matrix[0, :3, :3]
        right_quat = self.matrix_to_quaternion(right_rot)

        left_pos = left_wrist_matrix[0, :3, 3]
        left_rot = left_wrist_matrix[0, :3, :3]
        left_quat = self.matrix_to_quaternion(left_rot)

        p.resetBasePositionAndOrientation(self.right_wrist_body, right_pos, right_quat)
        p.resetBasePositionAndOrientation(self.left_wrist_body, left_pos, left_quat)

    def update_fingers(
        self, right_fingers, left_fingers, right_wrist_matrix, left_wrist_matrix
    ):
        """更新手指关节位置和姿态，并绘制骨架连接"""

        right_wrist_pos = right_wrist_matrix[0, :3, 3]
        right_wrist_rot = right_wrist_matrix[0, :3, :3]

        left_wrist_pos = left_wrist_matrix[0, :3, 3]
        left_wrist_rot = left_wrist_matrix[0, :3, :3]

        # 更新右手关节
        for i, finger_matrix in enumerate(right_fingers):
            finger_pos = right_wrist_pos + right_wrist_rot @ finger_matrix[:3, 3]
            finger_rot = finger_matrix[:3, :3]
            finger_quat = self.matrix_to_quaternion(finger_rot)
            p.resetBasePositionAndOrientation(
                self.right_finger_bodies[i], finger_pos.tolist(), finger_quat.tolist()
            )

        # 更新左手关节
        for i, finger_matrix in enumerate(left_fingers):
            finger_pos = left_wrist_pos + left_wrist_rot @ finger_matrix[:3, 3]
            finger_rot = finger_matrix[:3, :3]
            finger_quat = self.matrix_to_quaternion(finger_rot)
            p.resetBasePositionAndOrientation(
                self.left_finger_bodies[i], finger_pos.tolist(), finger_quat.tolist()
            )

        # 移除旧的连接线
        for line_id in self.right_finger_lines:
            p.removeUserDebugItem(line_id)
        for line_id in self.left_finger_lines:
            p.removeUserDebugItem(line_id)

        self.right_finger_lines = []
        self.left_finger_lines = []

        # 绘制右手骨架连接线
        for i, j in self.FINGER_CONNECTIONS:
            if i < len(right_fingers) and j < len(right_fingers):
                pos_i = right_wrist_pos + right_wrist_rot @ right_fingers[i][:3, 3]
                pos_j = right_wrist_pos + right_wrist_rot @ right_fingers[j][:3, 3]
                line_id = p.addUserDebugLine(
                    pos_i.tolist(),
                    pos_j.tolist(),
                    lineColorRGB=[1, 0, 0],
                    lineWidth=3,  # 红色
                )
                self.right_finger_lines.append(line_id)

        # 绘制左手骨架连接线
        for i, j in self.FINGER_CONNECTIONS:
            if i < len(left_fingers) and j < len(left_fingers):
                pos_i = left_wrist_pos + left_wrist_rot @ left_fingers[i][:3, 3]
                pos_j = left_wrist_pos + left_wrist_rot @ left_fingers[j][:3, 3]
                line_id = p.addUserDebugLine(
                    pos_i.tolist(),
                    pos_j.tolist(),
                    lineColorRGB=[0, 0, 1],
                    lineWidth=3,  # 蓝色
                )
                self.left_finger_lines.append(line_id)

    def matrix_to_quaternion(self, matrix):
        """将旋转矩阵转换为四元数"""
        # 使用scipy的转换函数，如果没有则使用简单实现
        try:
            from scipy.spatial.transform import Rotation

            r = Rotation.from_matrix(matrix)
            return r.as_quat()  # [x, y, z, w]
        except ImportError:
            # 简单实现
            trace = np.trace(matrix)
            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2
                w = 0.25 * s
                x = (matrix[2, 1] - matrix[1, 2]) / s
                y = (matrix[0, 2] - matrix[2, 0]) / s
                z = (matrix[1, 0] - matrix[0, 1]) / s
            else:
                if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
                    s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
                    w = (matrix[2, 1] - matrix[1, 2]) / s
                    x = 0.25 * s
                    y = (matrix[0, 1] + matrix[1, 0]) / s
                    z = (matrix[0, 2] + matrix[2, 0]) / s
                elif matrix[1, 1] > matrix[2, 2]:
                    s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
                    w = (matrix[0, 2] - matrix[2, 0]) / s
                    x = (matrix[0, 1] + matrix[1, 0]) / s
                    y = 0.25 * s
                    z = (matrix[1, 2] + matrix[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
                    w = (matrix[1, 0] - matrix[0, 1]) / s
                    x = (matrix[0, 2] + matrix[2, 0]) / s
                    y = (matrix[1, 2] + matrix[2, 1]) / s
                    z = 0.25 * s
            return np.array([x, y, z, w])

    def update_visualization(self, data):
        """更新整个可视化"""
        if data is None:
            return

        # 更新头部
        self.update_head(data["head"])

        # 更新手腕
        self.update_wrists(data["right_wrist"], data["left_wrist"])

        # 更新手指
        self.update_fingers(
            data["right_fingers"],
            data["left_fingers"],
            data["right_wrist"],
            data["left_wrist"],
        )

    def run(self):
        """运行可视化"""
        print("🎯 启动PyBullet版Vision Pro 3D可视化...")
        print("📱 确保Vision Pro上的Tracking Streamer应用正在运行")
        print("🔄 移动头部和手部观察3D可视化")
        print("📷 相机跟随模式:", "开启" if self.follow_camera else "关闭")
        print("⏹️  关闭窗口停止")
        print("=" * 60)

        try:
            while self.running:
                keys = p.getKeyboardEvents()

                cam_info = p.getDebugVisualizerCamera()
                cam_dist = cam_info[10]
                cam_yaw = cam_info[8]
                cam_pitch = cam_info[9]
                cam_target = cam_info[11]

                if ord("w") in keys and keys[ord("w")] & p.KEY_IS_DOWN:
                    cam_target = [cam_target[0], cam_target[1] + 0.1, cam_target[2]]
                if ord("s") in keys and keys[ord("s")] & p.KEY_IS_DOWN:
                    cam_target = [cam_target[0], cam_target[1] - 0.1, cam_target[2]]
                if ord("a") in keys and keys[ord("a")] & p.KEY_IS_DOWN:
                    cam_target = [cam_target[0] - 0.1, cam_target[1], cam_target[2]]
                if ord("d") in keys and keys[ord("d")] & p.KEY_IS_DOWN:
                    cam_target = [cam_target[0] + 0.1, cam_target[1], cam_target[2]]
                if ord("q") in keys and keys[ord("q")] & p.KEY_IS_DOWN:
                    cam_target = [cam_target[0], cam_target[1], cam_target[2]] + 0.1
                if ord("e") in keys and keys[ord("e")] & p.KEY_IS_DOWN:
                    cam_target = [cam_target[0], cam_target[1], cam_target[2]] - 0.1

                p.resetDebugVisualizerCamera(cam_dist, cam_yaw, cam_pitch, cam_target)
                # 获取最新数据
                data = self.streamer.latest

                if data is not None:
                    # 更新可视化
                    self.update_visualization(data)

                # 步进物理仿真
                p.stepSimulation()

                time.sleep(1 / 20)  # FPS

        except KeyboardInterrupt:
            print("\n⏹️  停止可视化")
        finally:
            p.disconnect()


def main():
    parser = argparse.ArgumentParser(description="PyBullet版Vision Pro 3D可视化")
    parser.add_argument("--ip", type=str, required=True, help="Vision Pro的IP地址")
    parser.add_argument("--record", action="store_true", help="是否记录数据")
    parser.add_argument("--no-follow", action="store_true", help="关闭相机跟随")
    args = parser.parse_args()

    viz = PyBulletVisualizer(args.ip, args.record, not args.no_follow)
    viz.run()


if __name__ == "__main__":
    main()

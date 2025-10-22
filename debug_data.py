#!/usr/bin/env python3
"""
诊断Vision Pro数据结构
"""
from avp_stream import VisionProStreamer
import numpy as np

# 连接到Vision Pro
streamer = VisionProStreamer(ip="192.168.1.152", record=False)

print("等待数据...")
while True:
    data = streamer.latest

    if data is not None:
        print("=" * 60)
        print("数据结构诊断：")
        print("=" * 60)

        # 右手数据
        print("\n右手腕位置:", data["right_wrist"][0, :3, 3])
        print("右手手指shape:", data["right_fingers"].shape)

        # 打印前5个手指关节的位置
        print("\n右手前5个关节位置:")
        for i in range(min(5, len(data["right_fingers"]))):
            pos = data["right_fingers"][i, :3, 3]
            print(f"  关节{i}: {pos}")

        # 左手数据
        print("\n左手腕位置:", data["left_wrist"][0, :3, 3])
        print("左手手指shape:", data["left_fingers"].shape)

        # 打印前5个手指关节的位置
        print("\n左手前5个关节位置:")
        for i in range(min(5, len(data["left_fingers"]))):
            pos = data["left_fingers"][i, :3, 3]
            print(f"  关节{i}: {pos}")

        print("\n头部位置:", data["head"][0, :3, 3])

        # 检查位置是否合理
        right_wrist = data["right_wrist"][0, :3, 3]
        left_wrist = data["left_wrist"][0, :3, 3]

        print("\n位置合理性检查:")
        print(f"  右手腕到原点距离: {np.linalg.norm(right_wrist):.3f}米")
        print(f"  左手腕到原点距离: {np.linalg.norm(left_wrist):.3f}米")
        print(f"  两手腕距离: {np.linalg.norm(right_wrist - left_wrist):.3f}米")

        break

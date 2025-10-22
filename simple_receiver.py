#!/usr/bin/env python3
"""
最简单的Vision Pro数据接收器
只显示数据，无多余输出
"""
from avp_stream import VisionProStreamer
import numpy as np

# 连接到Vision Pro
streamer = VisionProStreamer(ip="192.168.1.152", record=False)

while True:
    data = streamer.latest
    
    if data is not None:
        # 头部数据
        head_pos = data["head"][0, :3, 3]  # 头部位置 (x,y,z)
        head_rot = data["head"][0, :3, :3]  # 头部旋转矩阵
        
        # 右手数据
        right_wrist_pos = data["right_wrist"][0, :3, 3]  # 右手腕位置
        right_wrist_rot = data["right_wrist"][0, :3, :3]  # 右手腕旋转
        right_fingers = data["right_fingers"]  # 右手25个关节 (25,4,4)
        right_pinch = data["right_pinch_distance"]  # 右手捏合距离
        right_roll = data["right_wrist_roll"]  # 右手腕旋转角度
        
        # 左手数据
        left_wrist_pos = data["left_wrist"][0, :3, 3]  # 左手腕位置
        left_wrist_rot = data["left_wrist"][0, :3, :3]  # 左手腕旋转
        left_fingers = data["left_fingers"]  # 左手25个关节 (25,4,4)
        left_pinch = data["left_pinch_distance"]  # 左手捏合距离
        left_roll = data["left_wrist_roll"]  # 左手腕旋转角度
        
        # 打印数据
        print(f"head_pos: {head_pos}")
        print(f"right_wrist_pos: {right_wrist_pos}")
        print(f"left_wrist_pos: {left_wrist_pos}")
        print(f"right_pinch: {right_pinch}")
        print(f"left_pinch: {left_pinch}")
        print(f"right_roll: {right_roll}")
        print(f"left_roll: {left_roll}")
        print(f"right_fingers_shape: {right_fingers.shape}")
        print(f"left_fingers_shape: {left_fingers.shape}")
        print("---")

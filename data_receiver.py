#!/usr/bin/env python3
"""
Vision Pro 数据接收器
干净简洁地接收和显示所有Vision Pro数据
"""
import time
import numpy as np
from avp_stream import VisionProStreamer
import argparse

class VisionProDataReceiver:
    def __init__(self, ip, record=False):
        self.ip = ip
        self.record = record
        self.streamer = VisionProStreamer(ip, record)
        
        # 数据统计
        self.data_count = 0
        self.start_time = time.time()
        
        print(f"🎯 Vision Pro 数据接收器已启动")
        print(f"📡 连接到: {ip}")
        print(f"📊 记录模式: {'开启' if record else '关闭'}")
        print("=" * 60)

    def print_data_summary(self, data):
        """打印数据摘要"""
        print(f"\n📊 数据包 #{self.data_count + 1}")
        print(f"⏰ 时间: {time.strftime('%H:%M:%S')}")
        print("-" * 40)
        
        # 头部数据
        head_pos = data["head"][0, :3, 3]
        head_rot = data["head"][0, :3, :3]
        print(f"🧠 头部位置: ({head_pos[0]:.3f}, {head_pos[1]:.3f}, {head_pos[2]:.3f}) m")
        
        # 右手数据
        right_wrist_pos = data["right_wrist"][0, :3, 3]
        right_pinch = data["right_pinch_distance"]
        right_roll = np.degrees(data["right_wrist_roll"])
        print(f"✋ 右手腕位置: ({right_wrist_pos[0]:.3f}, {right_wrist_pos[1]:.3f}, {right_wrist_pos[2]:.3f}) m")
        print(f"✋ 右手捏合距离: {right_pinch:.3f} m")
        print(f"✋ 右手腕旋转: {right_roll:.1f}°")
        print(f"✋ 右手关节数: {len(data['right_fingers'])}")
        
        # 左手数据
        left_wrist_pos = data["left_wrist"][0, :3, 3]
        left_pinch = data["left_pinch_distance"]
        left_roll = np.degrees(data["left_wrist_roll"])
        print(f"🤚 左手腕位置: ({left_wrist_pos[0]:.3f}, {left_wrist_pos[1]:.3f}, {left_wrist_pos[2]:.3f}) m")
        print(f"🤚 左手捏合距离: {left_pinch:.3f} m")
        print(f"🤚 左手腕旋转: {left_roll:.1f}°")
        print(f"🤚 左手关节数: {len(data['left_fingers'])}")

    def print_detailed_fingers(self, data):
        """打印详细的手指关节数据"""
        print("\n👆 手指关节详细信息:")
        print("-" * 40)
        
        # 右手手指关节
        right_fingers = data["right_fingers"]
        print("✋ 右手关节 (相对于右手腕):")
        for i, joint in enumerate(right_fingers):
            pos = joint[:3, 3]
            print(f"  关节 {i:2d}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")
        
        print("\n🤚 左手关节 (相对于左手腕):")
        left_fingers = data["left_fingers"]
        for i, joint in enumerate(left_fingers):
            pos = joint[:3, 3]
            print(f"  关节 {i:2d}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")

    def detect_gestures(self, data):
        """检测简单手势"""
        gestures = []
        
        # 捏合检测
        right_pinch = data["right_pinch_distance"]
        left_pinch = data["left_pinch_distance"]
        pinch_threshold = 0.02  # 2cm
        
        if right_pinch < pinch_threshold:
            gestures.append("右手捏合")
        if left_pinch < pinch_threshold:
            gestures.append("左手捏合")
        
        # 手腕旋转检测
        right_roll = abs(np.degrees(data["right_wrist_roll"]))
        left_roll = abs(np.degrees(data["left_wrist_roll"]))
        
        if right_roll > 30:
            gestures.append(f"右手腕旋转({right_roll:.0f}°)")
        if left_roll > 30:
            gestures.append(f"左手腕旋转({left_roll:.0f}°)")
        
        if gestures:
            print(f"🎭 检测到手势: {', '.join(gestures)}")
        else:
            print("🎭 无特殊手势")

    def run(self, detailed=False, gesture_detection=True):
        """运行数据接收"""
        print("🚀 开始接收数据...")
        print("💡 提示:")
        print("   - 按 Ctrl+C 停止")
        print("   - 移动头部和手部观察数据变化")
        print("   - 进行捏合动作测试手势识别")
        print("=" * 60)
        
        try:
            while True:
                data = self.streamer.latest
                
                if data is not None:
                    self.data_count += 1
                    
                    # 打印数据摘要
                    self.print_data_summary(data)
                    
                    # 手势检测
                    if gesture_detection:
                        self.detect_gestures(data)
                    
                    # 详细手指数据
                    if detailed:
                        self.print_detailed_fingers(data)
                    
                    # 统计信息
                    elapsed = time.time() - self.start_time
                    fps = self.data_count / elapsed if elapsed > 0 else 0
                    print(f"\n📈 统计: 已接收 {self.data_count} 包数据, 平均 {fps:.1f} FPS")
                    print("=" * 60)
                
                time.sleep(0.1)  # 10 FPS
                
        except KeyboardInterrupt:
            print(f"\n⏹️  数据接收已停止")
            print(f"📊 最终统计:")
            print(f"   总数据包: {self.data_count}")
            print(f"   运行时间: {time.time() - self.start_time:.1f} 秒")
            print(f"   平均FPS: {self.data_count / (time.time() - self.start_time):.1f}")
            
            if self.record:
                recording = self.streamer.get_recording()
                print(f"   记录数据: {len(recording)} 帧")
                print(f"   数据已保存到内存中")

def main():
    parser = argparse.ArgumentParser(description='Vision Pro 数据接收器')
    parser.add_argument('--ip', type=str, required=True, help='Vision Pro的IP地址')
    parser.add_argument('--record', action='store_true', help='是否记录数据')
    parser.add_argument('--detailed', action='store_true', help='显示详细的手指关节数据')
    parser.add_argument('--no-gestures', action='store_true', help='关闭手势检测')
    args = parser.parse_args()
    
    try:
        receiver = VisionProDataReceiver(args.ip, args.record)
        receiver.run(detailed=args.detailed, gesture_detection=not args.no_gestures)
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保:")
        print("1. Vision Pro已连接到WiFi")
        print("2. Tracking Streamer应用正在运行")
        print("3. IP地址正确")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
完整的Vision Pro数据可视化
显示头部、手腕、手指和手势数据
"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from avp_stream import VisionProStreamer
import threading
import time

class CompleteVisualizer:
    def __init__(self, args):
        self.args = args
        self.s = VisionProStreamer(args.ip, args.record)
        
        # 数据历史记录
        self.head_history = []
        self.right_wrist_history = []
        self.left_wrist_history = []
        self.pinch_history = []
        
        # 绘图设置
        self.fig = None
        self.axes = None
        self.running = True

    def setup_plots(self):
        """设置多个子图"""
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 12))
        self.fig.suptitle('Vision Pro 完整数据可视化', fontsize=16)
        
        # 子图1: 头部轨迹
        self.ax1 = self.axes[0, 0]
        self.ax1.set_title('头部移动轨迹')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_aspect('equal')
        self.head_scat = self.ax1.scatter([], [], s=50, c='blue', alpha=0.6)
        
        # 子图2: 手腕位置
        self.ax2 = self.axes[0, 1]
        self.ax2.set_title('手腕位置')
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.set_aspect('equal')
        self.right_wrist_scat = self.ax2.scatter([], [], s=50, c='red', alpha=0.6, label='右手腕')
        self.left_wrist_scat = self.ax2.scatter([], [], s=50, c='green', alpha=0.6, label='左手腕')
        self.ax2.legend()
        
        # 子图3: 捏合距离
        self.ax3 = self.axes[1, 0]
        self.ax3.set_title('捏合距离')
        self.ax3.set_xlabel('时间')
        self.ax3.set_ylabel('距离 (m)')
        self.right_pinch_line, = self.ax3.plot([], [], 'r-', label='右手捏合')
        self.left_pinch_line, = self.ax3.plot([], [], 'g-', label='左手捏合')
        self.ax3.legend()
        
        # 子图4: 手腕旋转
        self.ax4 = self.axes[1, 1]
        self.ax4.set_title('手腕旋转角度')
        self.ax4.set_xlabel('时间')
        self.ax4.set_ylabel('角度 (度)')
        self.right_roll_line, = self.ax4.plot([], [], 'r-', label='右手腕旋转')
        self.left_roll_line, = self.ax4.plot([], [], 'g-', label='左手腕旋转')
        self.ax4.legend()
        
        plt.tight_layout()

    def update_plots(self, frame):
        """更新所有子图"""
        if not self.running:
            return
            
        try:
            data = self.s.latest
            if data is not None:
                # 更新头部轨迹
                head_pos = data["head"][0, :3, 3]
                self.head_history.append(head_pos[:2])  # 只取X,Y
                if len(self.head_history) > 100:  # 限制历史长度
                    self.head_history = self.head_history[-100:]
                
                head_array = np.array(self.head_history)
                self.head_scat.set_offsets(head_array)
                if len(head_array) > 0:
                    self.ax1.set_xlim(head_array[:, 0].min()-0.1, head_array[:, 0].max()+0.1)
                    self.ax1.set_ylim(head_array[:, 1].min()-0.1, head_array[:, 1].max()+0.1)
                
                # 更新手腕位置
                right_wrist_pos = data["right_wrist"][0, :3, 3]
                left_wrist_pos = data["left_wrist"][0, :3, 3]
                
                self.right_wrist_history.append(right_wrist_pos[:2])
                self.left_wrist_history.append(left_wrist_pos[:2])
                
                if len(self.right_wrist_history) > 50:
                    self.right_wrist_history = self.right_wrist_history[-50:]
                    self.left_wrist_history = self.left_wrist_history[-50:]
                
                right_array = np.array(self.right_wrist_history)
                left_array = np.array(self.left_wrist_history)
                
                self.right_wrist_scat.set_offsets(right_array)
                self.left_wrist_scat.set_offsets(left_array)
                
                # 更新捏合距离
                right_pinch = data["right_pinch_distance"]
                left_pinch = data["left_pinch_distance"]
                self.pinch_history.append([right_pinch, left_pinch])
                
                if len(self.pinch_history) > 200:
                    self.pinch_history = self.pinch_history[-200:]
                
                pinch_array = np.array(self.pinch_history)
                time_axis = np.arange(len(pinch_array))
                
                self.right_pinch_line.set_data(time_axis, pinch_array[:, 0])
                self.left_pinch_line.set_data(time_axis, pinch_array[:, 1])
                
                if len(pinch_array) > 0:
                    self.ax3.set_xlim(0, len(pinch_array))
                    self.ax3.set_ylim(0, max(pinch_array.max(), 0.1))
                
                # 更新手腕旋转
                right_roll = np.degrees(data["right_wrist_roll"])
                left_roll = np.degrees(data["left_wrist_roll"])
                
                if not hasattr(self, 'roll_history'):
                    self.roll_history = []
                self.roll_history.append([right_roll, left_roll])
                
                if len(self.roll_history) > 200:
                    self.roll_history = self.roll_history[-200:]
                
                roll_array = np.array(self.roll_history)
                time_axis = np.arange(len(roll_array))
                
                self.right_roll_line.set_data(time_axis, roll_array[:, 0])
                self.left_roll_line.set_data(time_axis, roll_array[:, 1])
                
                if len(roll_array) > 0:
                    self.ax4.set_xlim(0, len(roll_array))
                    self.ax4.set_ylim(roll_array.min()-10, roll_array.max()+10)
                
                # 打印实时数据
                print(f"头部: ({head_pos[0]:.3f}, {head_pos[1]:.3f}, {head_pos[2]:.3f})")
                print(f"右手腕: ({right_wrist_pos[0]:.3f}, {right_wrist_pos[1]:.3f}, {right_wrist_pos[2]:.3f})")
                print(f"捏合距离: 右={right_pinch:.3f}m, 左={left_pinch:.3f}m")
                print(f"手腕旋转: 右={right_roll:.1f}°, 左={left_roll:.1f}°")
                print("-" * 50)
                
        except Exception as e:
            print(f"更新错误: {e}")

    def run(self):
        """运行可视化"""
        print("🎯 启动Vision Pro完整数据可视化...")
        print("📱 确保Vision Pro上的Tracking Streamer应用正在运行")
        print("🔄 移动头部和手部来查看数据变化")
        print("⏹️  按Ctrl+C停止")
        
        self.setup_plots()
        
        ani = animation.FuncAnimation(
            self.fig, 
            self.update_plots, 
            frames=10000, 
            interval=100,  # 10 FPS
            blit=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n⏹️  停止可视化")
            self.running = False
        finally:
            plt.close()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Vision Pro完整数据可视化')
    parser.add_argument('--ip', type=str, required=True, help='Vision Pro的IP地址')
    parser.add_argument('--record', action='store_true', help='是否记录数据')
    args = parser.parse_args()
    
    try:
        viz = CompleteVisualizer(args)
        viz.run()
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保:")
        print("1. Vision Pro已连接到WiFi")
        print("2. Tracking Streamer应用正在运行")
        print("3. IP地址正确")

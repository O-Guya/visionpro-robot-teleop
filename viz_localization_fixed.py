#!/usr/bin/env python3
"""
修复版本的Vision Pro本地化可视化
解决matplotlib线程问题
"""
import matplotlib
matplotlib.use('TkAgg')  # 使用TkAgg后端，避免线程问题
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from avp_stream import VisionProStreamer
import threading
import time

class LocalizationVisualizer:
    def __init__(self, args): 
        self.args = args
        self.s = VisionProStreamer(args.ip, args.record)
        self.position_history = np.array([[0.0, 0.0]])
        self.fig = None
        self.ax = None
        self.scat = None
        self.running = True

    def setup_plot(self):
        """在主线程中设置matplotlib"""
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(10, 10)
        
        self.ax.set_xlim(-0.05, 0.1)
        self.ax.set_ylim(-0.05, 0.1)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        
        self.scat = self.ax.scatter(self.position_history[:, 0], self.position_history[:, 1], s=100)
        
        # 设置字体大小
        for item in ([self.ax.title, self.ax.xaxis.label, self.ax.yaxis.label] +
                    self.ax.get_xticklabels() + self.ax.get_yticklabels()):
            item.set_fontsize(20)

    def update_plot(self, frame):
        """更新绘图数据"""
        if not self.running:
            return
            
        try:
            transformations = self.s.latest
            if transformations is not None:
                head_pos = transformations["head"][:, :3, 3]
                new_pos = np.array([[head_pos[0, 0], -head_pos[0, 1]]])
                self.position_history = np.append(self.position_history, new_pos, axis=0)
                
                # 更新散点图
                self.scat.set_offsets(self.position_history)
                
                # 动态调整坐标轴范围
                if len(self.position_history) > 1:
                    xmin = min(self.position_history[:, 0])
                    xmax = max(self.position_history[:, 0])
                    ymin = min(self.position_history[:, 1])
                    ymax = max(self.position_history[:, 1])
                    
                    self.ax.set_xlim(xmin - 0.1, xmax + 0.1)
                    self.ax.set_ylim(ymin - 0.1, ymax + 0.1)
                
                print(f"位置: ({head_pos[0, 0]:.3f}, {head_pos[0, 1]:.3f})")
        except Exception as e:
            print(f"更新错误: {e}")

    def run(self):
        """运行可视化"""
        print("🎯 启动Vision Pro本地化可视化...")
        print("📱 确保Vision Pro上的Tracking Streamer应用正在运行")
        print("🔄 移动你的头部来查看位置变化")
        print("⏹️  按Ctrl+C停止")
        
        # 在主线程中设置matplotlib
        self.setup_plot()
        
        # 创建动画
        ani = animation.FuncAnimation(
            self.fig, 
            self.update_plot, 
            frames=10000, 
            interval=50,  # 20 FPS
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
    
    parser = argparse.ArgumentParser(description='Vision Pro本地化可视化')
    parser.add_argument('--ip', type=str, required=True, help='Vision Pro的IP地址')
    parser.add_argument('--record', action='store_true', help='是否记录数据')
    args = parser.parse_args()
    
    try:
        env = LocalizationVisualizer(args)
        env.run()
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保:")
        print("1. Vision Pro已连接到WiFi")
        print("2. Tracking Streamer应用正在运行")
        print("3. IP地址正确")



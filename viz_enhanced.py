#!/usr/bin/env python3
"""
增强版Vision Pro数据可视化
- 窗口不重叠排列
- 捏合动作识别和计数器
- 实时状态显示
"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation
from avp_stream import VisionProStreamer
import threading
import time

class EnhancedVisualizer:
    def __init__(self, args):
        self.args = args
        self.s = VisionProStreamer(args.ip, args.record)
        
        # 数据历史记录
        self.head_history = []
        self.right_wrist_history = []
        self.left_wrist_history = []
        self.pinch_history = []
        self.roll_history = []
        
        # 捏合动作识别
        self.pinch_threshold = 0.02  # 捏合阈值 (2cm)
        self.right_pinch_count = 0
        self.left_pinch_count = 0
        self.right_pinch_state = False  # 是否正在捏合
        self.left_pinch_state = False
        
        # 绘图设置
        self.fig = None
        self.axes = None
        self.running = True

    def setup_plots(self):
        """设置多个子图，窗口不重叠排列"""
        # 创建主窗口
        self.fig = plt.figure(figsize=(20, 15))
        self.fig.suptitle('Vision Pro 增强数据可视化 - 捏合动作识别', fontsize=16)
        
        # 创建子图布局 - 使用GridSpec确保不重叠
        gs = self.fig.add_gridspec(4, 4, hspace=0.4, wspace=0.3, 
                                 left=0.05, right=0.95, top=0.92, bottom=0.08)
        
        # 子图1: 头部轨迹 (2D) - 左上
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self.ax1.set_title('头部移动轨迹', fontsize=12, fontweight='bold')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_aspect('equal')
        self.head_scat = self.ax1.scatter([], [], s=50, c='blue', alpha=0.6)
        
        # 子图2: 手腕位置 (2D) - 右上
        self.ax2 = self.fig.add_subplot(gs[0, 1])
        self.ax2.set_title('手腕位置', fontsize=12, fontweight='bold')
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.set_aspect('equal')
        self.right_wrist_scat = self.ax2.scatter([], [], s=50, c='red', alpha=0.6, label='右手腕')
        self.left_wrist_scat = self.ax2.scatter([], [], s=50, c='green', alpha=0.6, label='左手腕')
        self.ax2.legend()
        
        # 子图3: 捏合距离和计数器 - 右上角
        self.ax3 = self.fig.add_subplot(gs[0, 2:])
        self.ax3.set_title('捏合动作识别和计数器', fontsize=12, fontweight='bold')
        self.ax3.set_xlabel('时间')
        self.ax3.set_ylabel('距离 (m)')
        self.right_pinch_line, = self.ax3.plot([], [], 'r-', linewidth=2, label='右手捏合距离')
        self.left_pinch_line, = self.ax3.plot([], [], 'g-', linewidth=2, label='左手捏合距离')
        self.ax3.axhline(y=self.pinch_threshold, color='k', linestyle='--', alpha=0.5, label=f'阈值={self.pinch_threshold}m')
        self.ax3.legend()
        
        # 子图4: 手腕旋转 - 第二行左
        self.ax4 = self.fig.add_subplot(gs[1, 0])
        self.ax4.set_title('手腕旋转角度', fontsize=12, fontweight='bold')
        self.ax4.set_xlabel('时间')
        self.ax4.set_ylabel('角度 (度)')
        self.right_roll_line, = self.ax4.plot([], [], 'r-', label='右手腕旋转')
        self.left_roll_line, = self.ax4.plot([], [], 'g-', label='左手腕旋转')
        self.ax4.legend()
        
        # 子图5: 右手手指关节 (3D) - 第二行中
        self.ax5 = self.fig.add_subplot(gs[1, 1], projection='3d')
        self.ax5.set_title('右手手指关节', fontsize=12, fontweight='bold')
        self.ax5.set_xlabel('X')
        self.ax5.set_ylabel('Y')
        self.ax5.set_zlabel('Z')
        self.right_fingers_scat = self.ax5.scatter([], [], [], s=30, c='red', alpha=0.7)
        
        # 子图6: 左手手指关节 (3D) - 第二行右
        self.ax6 = self.fig.add_subplot(gs[1, 2], projection='3d')
        self.ax6.set_title('左手手指关节', fontsize=12, fontweight='bold')
        self.ax6.set_xlabel('X')
        self.ax6.set_ylabel('Y')
        self.ax6.set_zlabel('Z')
        self.left_fingers_scat = self.ax6.scatter([], [], [], s=30, c='green', alpha=0.7)
        
        # 子图7: 捏合计数器显示 - 第二行最右
        self.ax7 = self.fig.add_subplot(gs[1, 3])
        self.ax7.set_title('捏合计数器', fontsize=12, fontweight='bold')
        self.ax7.axis('off')  # 关闭坐标轴，只显示文本
        
        # 子图8: 手指关节距离热图 - 底部
        self.ax8 = self.fig.add_subplot(gs[2:, :])
        self.ax8.set_title('右手手指关节距离矩阵', fontsize=12, fontweight='bold')
        self.ax8.set_xlabel('关节索引')
        self.ax8.set_ylabel('关节索引')
        self.distance_matrix = None
        self.distance_im = None
        
        # 初始化计数器显示
        self.update_counter_display()

    def update_counter_display(self):
        """更新计数器显示"""
        self.ax7.clear()
        self.ax7.set_title('捏合计数器', fontsize=12, fontweight='bold')
        self.ax7.axis('off')
        
        # 显示计数器
        counter_text = f"""
右手捏合次数: {self.right_pinch_count}
左手捏合次数: {self.left_pinch_count}
总捏合次数: {self.right_pinch_count + self.left_pinch_count}

右手状态: {'🔴 捏合中' if self.right_pinch_state else '⚪ 未捏合'}
左手状态: {'🔴 捏合中' if self.left_pinch_state else '⚪ 未捏合'}

阈值: {self.pinch_threshold}m
        """
        
        self.ax7.text(0.1, 0.8, counter_text, fontsize=14, 
                     verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))

    def detect_pinch_action(self, right_pinch, left_pinch):
        """检测捏合动作"""
        # 检测右手捏合
        if right_pinch < self.pinch_threshold and not self.right_pinch_state:
            self.right_pinch_state = True
            self.right_pinch_count += 1
            print(f"🎯 右手捏合动作检测到! 总次数: {self.right_pinch_count}")
        elif right_pinch >= self.pinch_threshold and self.right_pinch_state:
            self.right_pinch_state = False
            print(f"✋ 右手捏合结束")
        
        # 检测左手捏合
        if left_pinch < self.pinch_threshold and not self.left_pinch_state:
            self.left_pinch_state = True
            self.left_pinch_count += 1
            print(f"🎯 左手捏合动作检测到! 总次数: {self.left_pinch_count}")
        elif left_pinch >= self.pinch_threshold and self.left_pinch_state:
            self.left_pinch_state = False
            print(f"✋ 左手捏合结束")

    def update_plots(self, frame):
        """更新所有子图"""
        if not self.running:
            return
            
        try:
            data = self.s.latest
            if data is not None:
                # 更新头部轨迹
                head_pos = data["head"][0, :3, 3]
                self.head_history.append(head_pos[:2])
                if len(self.head_history) > 100:
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
                
                # 更新捏合距离和检测动作
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
                
                # 检测捏合动作
                self.detect_pinch_action(right_pinch, left_pinch)
                
                # 更新手腕旋转
                right_roll = np.degrees(data["right_wrist_roll"])
                left_roll = np.degrees(data["left_wrist_roll"])
                
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
                
                # 更新右手手指关节 (3D)
                right_fingers = data["right_fingers"]
                right_finger_positions = right_fingers[:, :3, 3]
                
                self.right_fingers_scat._offsets3d = (
                    right_finger_positions[:, 0],
                    right_finger_positions[:, 1], 
                    right_finger_positions[:, 2]
                )
                
                if len(right_finger_positions) > 0:
                    margin = 0.05
                    self.ax5.set_xlim(right_finger_positions[:, 0].min()-margin, 
                                     right_finger_positions[:, 0].max()+margin)
                    self.ax5.set_ylim(right_finger_positions[:, 1].min()-margin, 
                                     right_finger_positions[:, 1].max()+margin)
                    self.ax5.set_zlim(right_finger_positions[:, 2].min()-margin, 
                                     right_finger_positions[:, 2].max()+margin)
                
                # 更新左手手指关节 (3D)
                left_fingers = data["left_fingers"]
                left_finger_positions = left_fingers[:, :3, 3]
                
                self.left_fingers_scat._offsets3d = (
                    left_finger_positions[:, 0],
                    left_finger_positions[:, 1], 
                    left_finger_positions[:, 2]
                )
                
                if len(left_finger_positions) > 0:
                    margin = 0.05
                    self.ax6.set_xlim(left_finger_positions[:, 0].min()-margin, 
                                     left_finger_positions[:, 0].max()+margin)
                    self.ax6.set_ylim(left_finger_positions[:, 1].min()-margin, 
                                     left_finger_positions[:, 1].max()+margin)
                    self.ax6.set_zlim(left_finger_positions[:, 2].min()-margin, 
                                     left_finger_positions[:, 2].max()+margin)
                
                # 更新手指关节距离矩阵
                if len(right_finger_positions) > 0:
                    distances = np.zeros((len(right_finger_positions), len(right_finger_positions)))
                    for i in range(len(right_finger_positions)):
                        for j in range(len(right_finger_positions)):
                            distances[i, j] = np.linalg.norm(
                                right_finger_positions[i] - right_finger_positions[j]
                            )
                    
                    if self.distance_im is None:
                        self.distance_im = self.ax8.imshow(distances, cmap='viridis', aspect='auto')
                        self.ax8.set_xticks(range(0, len(right_finger_positions), 5))
                        self.ax8.set_yticks(range(0, len(right_finger_positions), 5))
                        plt.colorbar(self.distance_im, ax=self.ax8)
                    else:
                        self.distance_im.set_array(distances)
                        self.distance_im.set_clim(vmin=distances.min(), vmax=distances.max())
                
                # 更新计数器显示
                self.update_counter_display()
                
                # 打印实时数据
                print(f"头部: ({head_pos[0]:.3f}, {head_pos[1]:.3f}, {head_pos[2]:.3f})")
                print(f"右手腕: ({right_wrist_pos[0]:.3f}, {right_wrist_pos[1]:.3f}, {right_wrist_pos[2]:.3f})")
                print(f"捏合距离: 右={right_pinch:.3f}m, 左={left_pinch:.3f}m")
                print(f"手腕旋转: 右={right_roll:.1f}°, 左={left_roll:.1f}°")
                print(f"捏合计数: 右={self.right_pinch_count}, 左={self.left_pinch_count}")
                print("-" * 50)
                
        except Exception as e:
            print(f"更新错误: {e}")

    def run(self):
        """运行可视化"""
        print("🎯 启动Vision Pro增强数据可视化...")
        print("📱 确保Vision Pro上的Tracking Streamer应用正在运行")
        print("👆 进行捏合动作来测试识别功能")
        print("🔢 观察实时计数器更新")
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
            print(f"📊 最终统计:")
            print(f"   右手捏合次数: {self.right_pinch_count}")
            print(f"   左手捏合次数: {self.left_pinch_count}")
            print(f"   总捏合次数: {self.right_pinch_count + self.left_pinch_count}")
            self.running = False
        finally:
            plt.close()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Vision Pro增强数据可视化 - 捏合动作识别')
    parser.add_argument('--ip', type=str, required=True, help='Vision Pro的IP地址')
    parser.add_argument('--record', action='store_true', help='是否记录数据')
    parser.add_argument('--threshold', type=float, default=0.02, help='捏合检测阈值(米)')
    args = parser.parse_args()
    
    try:
        viz = EnhancedVisualizer(args)
        viz.pinch_threshold = args.threshold
        viz.run()
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保:")
        print("1. Vision Pro已连接到WiFi")
        print("2. Tracking Streamer应用正在运行")
        print("3. IP地址正确")

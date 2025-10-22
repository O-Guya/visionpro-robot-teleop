#!/usr/bin/env python3
"""
现代版Vision Pro 3D可视化
使用Open3D替代Isaac Gym，支持Python 3.10+
"""
import open3d as o3d
import numpy as np
import time
import threading
from avp_stream import VisionProStreamer
import argparse

class ModernVisualizer:
    def __init__(self, ip, record=False, follow_camera=True):
        self.ip = ip
        self.record = record
        self.follow_camera = follow_camera
        self.streamer = VisionProStreamer(ip, record)
        
        # 初始化Open3D可视化器
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("Vision Pro 3D 可视化", width=1200, height=800)
        
        # 创建几何体
        self.setup_geometry()
        
        # 相机设置
        self.setup_camera()
        
        # 运行状态
        self.running = True
        
    def setup_geometry(self):
        """设置3D几何体"""
        # 创建坐标系
        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        
        # 创建头部几何体 (大球体)
        self.head_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        self.head_sphere.paint_uniform_color([1.0, 0.0, 0.0])  # 红色
        
        # 创建手腕几何体 (中等球体)
        self.right_wrist_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.03)
        self.right_wrist_sphere.paint_uniform_color([0.0, 1.0, 0.0])  # 绿色
        
        self.left_wrist_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.03)
        self.left_wrist_sphere.paint_uniform_color([0.0, 0.0, 1.0])  # 蓝色
        
        # 创建手指关节几何体 (小球体)
        self.right_finger_spheres = []
        self.left_finger_spheres = []
        
        for i in range(25):
            # 右手关节
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            if i in [0, 4, 9, 14, 19, 24]:  # 关键关节
                sphere.paint_uniform_color([1.0, 1.0, 0.0])  # 黄色
            else:
                sphere.paint_uniform_color([0.8, 0.8, 0.8])  # 灰色
            self.right_finger_spheres.append(sphere)
            
            # 左手关节
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            if i in [0, 4, 9, 14, 19, 24]:  # 关键关节
                sphere.paint_uniform_color([1.0, 1.0, 0.0])  # 黄色
            else:
                sphere.paint_uniform_color([0.8, 0.8, 0.8])  # 灰色
            self.left_finger_spheres.append(sphere)
        
        # 添加所有几何体到可视化器
        self.vis.add_geometry(self.coordinate_frame)
        self.vis.add_geometry(self.head_sphere)
        self.vis.add_geometry(self.right_wrist_sphere)
        self.vis.add_geometry(self.left_wrist_sphere)
        
        for sphere in self.right_finger_spheres + self.left_finger_spheres:
            self.vis.add_geometry(sphere)
    
    def setup_camera(self):
        """设置相机参数"""
        ctr = self.vis.get_view_control()
        ctr.set_front([0, 0, -1])
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, 1, 0])
        ctr.set_zoom(0.8)
    
    def update_head(self, head_matrix):
        """更新头部位置和姿态"""
        head_pos = head_matrix[0, :3, 3]
        head_rot = head_matrix[0, :3, :3]
        
        # 更新位置
        self.head_sphere.translate(head_pos - self.head_sphere.get_center(), relative=False)
        
        # 更新姿态 (简化处理)
        # Open3D的旋转处理比较复杂，这里只更新位置
    
    def update_wrists(self, right_wrist_matrix, left_wrist_matrix):
        """更新手腕位置"""
        right_pos = right_wrist_matrix[0, :3, 3]
        left_pos = left_wrist_matrix[0, :3, 3]
        
        # 更新右手腕
        self.right_wrist_sphere.translate(right_pos - self.right_wrist_sphere.get_center(), relative=False)
        
        # 更新左手腕
        self.left_wrist_sphere.translate(left_pos - self.left_wrist_sphere.get_center(), relative=False)
    
    def update_fingers(self, right_fingers, left_fingers):
        """更新手指关节位置"""
        # 更新右手关节
        for i, finger_matrix in enumerate(right_fingers):
            finger_pos = finger_matrix[:3, 3]
            self.right_finger_spheres[i].translate(
                finger_pos - self.right_finger_spheres[i].get_center(), 
                relative=False
            )
        
        # 更新左手关节
        for i, finger_matrix in enumerate(left_fingers):
            finger_pos = finger_matrix[:3, 3]
            self.left_finger_spheres[i].translate(
                finger_pos - self.left_finger_spheres[i].get_center(), 
                relative=False
            )
    
    def update_camera(self, head_matrix):
        """更新相机位置跟随头部"""
        if not self.follow_camera:
            return
            
        head_pos = head_matrix[0, :3, 3]
        head_rot = head_matrix[0, :3, :3]
        
        # 计算相机位置 (头部后方)
        camera_offset = np.array([0, 0, 0.3])  # 头部后方30cm
        camera_pos = head_pos + camera_offset
        
        # 计算相机目标 (头部前方)
        look_at_offset = np.array([0, 0, -0.5])  # 头部前方50cm
        look_at_pos = head_pos + look_at_offset
        
        ctr = self.vis.get_view_control()
        ctr.set_front([0, 0, -1])
        ctr.set_lookat(look_at_pos)
        ctr.set_up([0, 1, 0])
    
    def update_visualization(self, data):
        """更新整个可视化"""
        if data is None:
            return
        
        # 更新头部
        self.update_head(data["head"])
        
        # 更新手腕
        self.update_wrists(data["right_wrist"], data["left_wrist"])
        
        # 更新手指
        self.update_fingers(data["right_fingers"], data["left_fingers"])
        
        # 更新相机
        if self.follow_camera:
            self.update_camera(data["head"])
        
        # 更新几何体
        self.vis.update_geometry(self.head_sphere)
        self.vis.update_geometry(self.right_wrist_sphere)
        self.vis.update_geometry(self.left_wrist_sphere)
        
        for sphere in self.right_finger_spheres + self.left_finger_spheres:
            self.vis.update_geometry(sphere)
    
    def run(self):
        """运行可视化"""
        print("🎯 启动现代版Vision Pro 3D可视化...")
        print("📱 确保Vision Pro上的Tracking Streamer应用正在运行")
        print("🔄 移动头部和手部观察3D可视化")
        print("📷 相机跟随模式:", "开启" if self.follow_camera else "关闭")
        print("⏹️  关闭窗口停止")
        print("=" * 60)
        
        try:
            while self.running:
                # 获取最新数据
                data = self.streamer.latest
                
                if data is not None:
                    # 更新可视化
                    self.update_visualization(data)
                
                # 渲染
                if not self.vis.poll_events():
                    break
                self.vis.update_renderer()
                
                time.sleep(0.01)  # 100 FPS
                
        except KeyboardInterrupt:
            print("\n⏹️  停止可视化")
        finally:
            self.vis.destroy_window()

def main():
    parser = argparse.ArgumentParser(description='现代版Vision Pro 3D可视化')
    parser.add_argument('--ip', type=str, required=True, help='Vision Pro的IP地址')
    parser.add_argument('--record', action='store_true', help='是否记录数据')
    parser.add_argument('--no-follow', action='store_true', help='关闭相机跟随')
    args = parser.parse_args()
    
    try:
        viz = ModernVisualizer(args.ip, args.record, not args.no_follow)
        viz.run()
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("请确保:")
        print("1. 已安装Open3D: pip install open3d")
        print("2. Vision Pro已连接到WiFi")
        print("3. Tracking Streamer应用正在运行")
        print("4. IP地址正确")

if __name__ == "__main__":
    main()

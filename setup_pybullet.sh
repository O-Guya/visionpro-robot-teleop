#!/bin/bash

# PyBullet 仿真环境快速设置脚本
# 适用于 macOS/Linux

echo "================================"
echo "Kinova Gen3 PyBullet 环境设置"
echo "================================"
echo ""

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 未找到 Python3"
    echo "请先安装 Python 3.7 或更高版本"
    exit 1
fi

echo "✅ Python 版本: $(python3 --version)"
echo ""

# 安装依赖
echo "正在安装 PyBullet 和依赖..."
pip3 install pybullet numpy

if [ $? -eq 0 ]; then
    echo "✅ 依赖安装成功！"
else
    echo "❌ 安装失败"
    exit 1
fi

echo ""
echo "================================"
echo "设置完成！"
echo "================================"
echo ""
echo "现在你可以运行："
echo ""
echo "1. 自动演示:"
echo "   python3 kinova_pybullet_sim.py --mode demo"
echo ""
echo "2. 交互式控制:"
echo "   python3 kinova_pybullet_sim.py --mode interactive"
echo ""
echo "3. Vision Pro 遥操作:"
echo "   python3 kinova_visionpro_control.py --ip <你的Vision_Pro_IP>"
echo ""
echo "享受吧！ 🎉"

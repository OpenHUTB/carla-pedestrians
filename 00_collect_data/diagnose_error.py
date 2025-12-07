#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
错误诊断脚本 - 帮助定位IMU_Vision_Fusion_EKF.py的问题
"""

import sys
import os

print("="*60)
print("IMU-Visual Fusion 错误诊断")
print("="*60)
print()

# 1. 检查Python版本
print("1. Python版本检查")
print(f"   版本: {sys.version}")
if sys.version_info < (3, 7):
    print("   ❌ 错误: Python版本过低，需要3.7+")
else:
    print("   ✓ Python版本正常")
print()

# 2. 检查必要的包
print("2. 依赖包检查")
required_packages = {
    'carla': 'CARLA Python API',
    'numpy': 'NumPy',
    'cv2': 'OpenCV',
    'scipy': 'SciPy'
}

missing_packages = []
for package, name in required_packages.items():
    try:
        if package == 'cv2':
            import cv2
        elif package == 'carla':
            import carla
        elif package == 'numpy':
            import numpy
        elif package == 'scipy':
            import scipy
        print(f"   ✓ {name} 已安装")
    except ImportError:
        print(f"   ❌ {name} 未安装")
        missing_packages.append(package)

if missing_packages:
    print(f"\n   缺失包: {', '.join(missing_packages)}")
    print(f"   安装命令: pip install {' '.join(missing_packages)}")
print()

# 3. 检查CARLA连接
print("3. CARLA服务器检查")
try:
    import carla
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    print(f"   ✓ CARLA连接成功")
    print(f"   地图: {world.get_map().name}")
    print(f"   actors数量: {len(world.get_actors())}")
except Exception as e:
    print(f"   ❌ CARLA连接失败: {e}")
    print("   请确保CARLA服务器已启动：./CarlaUE4.sh")
print()

# 4. 检查CARLA PythonAPI路径
print("4. CARLA PythonAPI路径检查")
current_dir = os.path.dirname(os.path.abspath(__file__))
carla_api_path = os.path.join(current_dir, '../../../../carla-0.9.15/PythonAPI/carla')
if os.path.exists(carla_api_path):
    print(f"   ✓ CARLA API路径存在: {carla_api_path}")
    
    # 检查BehaviorAgent
    sys.path.insert(0, carla_api_path)
    try:
        from agents.navigation.behavior_agent import BehaviorAgent
        print(f"   ✓ BehaviorAgent导入成功")
    except ImportError as e:
        print(f"   ❌ BehaviorAgent导入失败: {e}")
        print(f"   请检查CARLA版本和路径配置")
else:
    print(f"   ❌ CARLA API路径不存在: {carla_api_path}")
    print(f"   请修改IMU_Vision_Fusion_EKF.py中的carla_api_path")
print()

# 5. 检查输出目录权限
print("5. 输出目录权限检查")
output_dir = '../data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/'
try:
    test_dir = os.path.join(output_dir, 'test')
    os.makedirs(test_dir, exist_ok=True)
    os.rmdir(test_dir)
    print(f"   ✓ 输出目录权限正常: {output_dir}")
except Exception as e:
    print(f"   ❌ 输出目录权限问题: {e}")
print()

# 6. 尝试导入主脚本
print("6. 主脚本导入检查")
try:
    # 先不执行，只检查导入
    import IMU_Vision_Fusion_EKF as main_script
    print("   ✓ IMU_Vision_Fusion_EKF.py 导入成功")
    
    # 检查关键配置
    print(f"   配置:")
    print(f"     最大速度: {main_script.AGENT_MAX_SPEED} km/h")
    print(f"     安全距离: {main_script.AGENT_SAFE_DISTANCE} m")
    print(f"     碰撞阈值: {main_script.COLLISION_RESET_THRESHOLD} 次")
    print(f"     目标地图: {main_script.TARGET_MAP}")
    
except Exception as e:
    print(f"   ❌ 主脚本导入失败")
    print(f"   错误详情: {e}")
    import traceback
    traceback.print_exc()
print()

# 总结
print("="*60)
print("诊断完成")
print("="*60)
print()
print("如果所有检查都通过，请运行：")
print("  python IMU_Vision_Fusion_EKF.py")
print()
print("如果仍有错误，请复制完整的错误信息反馈。")
print("="*60)

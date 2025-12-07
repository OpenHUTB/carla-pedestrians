#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
碰撞避免功能测试脚本
快速测试优化后的避障配置是否有效
"""

import os
import sys

# 修改这些参数来快速测试
TEST_CONFIG = {
    'MAX_SAVE_IMG': 200,  # 测试200帧即可
    'AGENT_MAX_SPEED': 20,  # km/h
    'AGENT_SAFE_DISTANCE': 5.0,  # 米
    'COLLISION_RESET_THRESHOLD': 3,
    'TARGET_MAP': 'Town10HD',
    'OUTPUT_DIR': '../data/01_NeuroSLAM_Datasets/Test_Collision_Avoidance/'
}

print("="*60)
print("碰撞避免功能测试")
print("="*60)
print(f"测试配置:")
print(f"  帧数: {TEST_CONFIG['MAX_SAVE_IMG']}")
print(f"  速度: {TEST_CONFIG['AGENT_MAX_SPEED']} km/h")
print(f"  安全距离: {TEST_CONFIG['AGENT_SAFE_DISTANCE']} m")
print(f"  碰撞阈值: {TEST_CONFIG['COLLISION_RESET_THRESHOLD']} 次")
print(f"  地图: {TEST_CONFIG['TARGET_MAP']}")
print("="*60)
print()

# 修改主脚本的参数
import IMU_Vision_Fusion_EKF as main_script

# 临时修改配置
original_max = main_script.MAX_SAVE_IMG
original_speed = main_script.AGENT_MAX_SPEED
original_distance = main_script.AGENT_SAFE_DISTANCE
original_threshold = main_script.COLLISION_RESET_THRESHOLD
original_map = main_script.TARGET_MAP
original_dir = main_script.OUTPUT_DIR

main_script.MAX_SAVE_IMG = TEST_CONFIG['MAX_SAVE_IMG']
main_script.AGENT_MAX_SPEED = TEST_CONFIG['AGENT_MAX_SPEED']
main_script.AGENT_SAFE_DISTANCE = TEST_CONFIG['AGENT_SAFE_DISTANCE']
main_script.COLLISION_RESET_THRESHOLD = TEST_CONFIG['COLLISION_RESET_THRESHOLD']
main_script.TARGET_MAP = TEST_CONFIG['TARGET_MAP']
main_script.OUTPUT_DIR = TEST_CONFIG['OUTPUT_DIR']

print("开始测试...")
print("按Ctrl+C可提前退出\n")

try:
    # 运行主程序
    main_script.main()
    
    print("\n" + "="*60)
    print("测试完成！")
    print("="*60)
    
    # 检查结果
    if os.path.exists(TEST_CONFIG['OUTPUT_DIR']):
        img_files = [f for f in os.listdir(TEST_CONFIG['OUTPUT_DIR']) if f.endswith('.png')]
        imu_file = os.path.join(TEST_CONFIG['OUTPUT_DIR'], 'aligned_imu.txt')
        fusion_file = os.path.join(TEST_CONFIG['OUTPUT_DIR'], 'fusion_pose.txt')
        
        print(f"\n结果统计:")
        print(f"  采集图像: {len(img_files)} 张")
        print(f"  IMU数据: {'✓' if os.path.exists(imu_file) else '✗'}")
        print(f"  融合数据: {'✓' if os.path.exists(fusion_file) else '✗'}")
        
        success_rate = len(img_files) / TEST_CONFIG['MAX_SAVE_IMG'] * 100
        print(f"  完成率: {success_rate:.1f}%")
        
        if success_rate >= 90:
            print("\n✅ 测试通过！避障配置工作良好。")
        elif success_rate >= 70:
            print("\n⚠️  测试基本通过，但建议进一步优化参数。")
        else:
            print("\n❌ 测试未通过，需要调整避障参数。")
            print("   建议：降低AGENT_MAX_SPEED或增加AGENT_SAFE_DISTANCE")
    else:
        print("❌ 输出目录不存在，测试失败")
        
except KeyboardInterrupt:
    print("\n\n用户中断测试")
except Exception as e:
    print(f"\n❌ 测试出错: {e}")
finally:
    # 恢复原始配置
    main_script.MAX_SAVE_IMG = original_max
    main_script.AGENT_MAX_SPEED = original_speed
    main_script.AGENT_SAFE_DISTANCE = original_distance
    main_script.COLLISION_RESET_THRESHOLD = original_threshold
    main_script.TARGET_MAP = original_map
    main_script.OUTPUT_DIR = original_dir
    
    print("\n配置已恢复到原始值")
    print("="*60)

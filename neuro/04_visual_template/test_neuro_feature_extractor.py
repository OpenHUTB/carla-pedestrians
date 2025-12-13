#!/usr/bin/env python
"""
测试增强的NeuroSLAM视觉特征提取器
"""

import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from neuro_visual_feature_extractor import (
    NeuroVisualFeatureExtractor,
    extract_neuro_features_simple,
    compare_images_neuro,
    neuro_patch_normalization
)


def test_basic_extraction():
    """测试1: 基础特征提取"""
    print("\n=== 测试1: 基础特征提取 ===")
    
    # 创建测试图像
    test_img = np.random.randint(0, 255, (120, 240), dtype=np.uint8)
    print(f"测试图像尺寸: {test_img.shape}")
    
    # 提取特征
    extractor = NeuroVisualFeatureExtractor(feature_dim=256)
    start_time = time.time()
    features = extractor.extract_features(test_img)
    elapsed = time.time() - start_time
    
    print(f"特征提取完成:")
    print(f"  特征维度: {features.shape}")
    print(f"  特征范围: [{features.min():.3f}, {features.max():.3f}]")
    print(f"  特征norm: {np.linalg.norm(features):.3f}")
    print(f"  耗时: {elapsed:.3f}秒")
    
    # 可视化
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    
    axes[0].imshow(test_img, cmap='gray')
    axes[0].set_title('原始图像')
    axes[0].axis('off')
    
    axes[1].plot(features)
    axes[1].set_title('特征向量')
    axes[1].set_xlabel('维度')
    axes[1].set_ylabel('值')
    axes[1].grid(True)
    
    axes[2].hist(features, bins=50)
    axes[2].set_title('特征分布')
    axes[2].set_xlabel('特征值')
    axes[2].set_ylabel('频数')
    
    plt.tight_layout()
    plt.savefig('test1_basic_extraction.png', dpi=150)
    print("  可视化已保存: test1_basic_extraction.png")


def test_gabor_features():
    """测试2: Gabor特征可视化"""
    print("\n=== 测试2: Gabor特征可视化 ===")
    
    # 创建测试图像 (带一些结构)
    test_img = np.zeros((120, 240), dtype=np.uint8)
    # 添加不同方向的线条
    test_img[30:35, :] = 255  # 水平线
    test_img[:, 100:105] = 255  # 垂直线
    for i in range(120):
        j = int(i * 240 / 120)
        if 0 <= j < 240:
            test_img[i, j] = 255  # 对角线
    
    # 提取特征
    extractor = NeuroVisualFeatureExtractor(feature_dim=256)
    v1_features = extractor._v1_processing(test_img.astype(np.float32) / 255.0)
    
    print(f"V1特征维度: {v1_features.shape}")
    
    # 可视化不同方向的Gabor响应
    n_orientations = min(8, v1_features.shape[2])
    fig, axes = plt.subplots(2, 4, figsize=(15, 8))
    axes = axes.flatten()
    
    for i in range(n_orientations):
        axes[i].imshow(v1_features[:, :, i], cmap='hot')
        axes[i].set_title(f'方向 {i} ({i*22.5:.1f}°)')
        axes[i].axis('off')
    
    plt.tight_layout()
    plt.savefig('test2_gabor_features.png', dpi=150)
    print("  可视化已保存: test2_gabor_features.png")


def test_attention_mechanism():
    """测试3: 注意力机制"""
    print("\n=== 测试3: 注意力机制 ===")
    
    # 创建带显著区域的测试图像
    test_img = np.random.randint(50, 100, (120, 240), dtype=np.uint8)
    # 添加显著的中心区域
    test_img[40:80, 100:140] = 200
    
    # 计算注意力图
    extractor = NeuroVisualFeatureExtractor(use_attention=True)
    attention_map = extractor._compute_attention(test_img)
    
    print(f"注意力图范围: [{attention_map.min():.3f}, {attention_map.max():.3f}]")
    
    # 应用注意力
    img_float = test_img.astype(np.float32) / 255.0
    attention_applied = img_float * attention_map
    
    # 可视化
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    
    axes[0].imshow(test_img, cmap='gray')
    axes[0].set_title('原始图像')
    axes[0].axis('off')
    
    im = axes[1].imshow(attention_map, cmap='hot')
    axes[1].set_title('注意力图')
    axes[1].axis('off')
    plt.colorbar(im, ax=axes[1])
    
    axes[2].imshow(attention_applied, cmap='gray')
    axes[2].set_title('应用注意力后')
    axes[2].axis('off')
    
    plt.tight_layout()
    plt.savefig('test3_attention.png', dpi=150)
    print("  可视化已保存: test3_attention.png")


def test_feature_comparison():
    """测试4: 特征比较"""
    print("\n=== 测试4: 特征比较 ===")
    
    # 创建三张测试图像
    img1 = np.random.randint(0, 255, (120, 240), dtype=np.uint8)
    img2 = np.clip(img1 + np.random.randint(-20, 20, img1.shape), 0, 255).astype(np.uint8)  # 轻微变化
    img3 = np.random.randint(0, 255, (120, 240), dtype=np.uint8)  # 完全随机
    
    # 计算相似度
    sim12 = compare_images_neuro(img1, img2)
    sim13 = compare_images_neuro(img1, img3)
    sim23 = compare_images_neuro(img2, img3)
    
    print(f"相似度结果:")
    print(f"  原图 vs 轻微变化: {sim12:.4f} (应该较高)")
    print(f"  原图 vs 随机图: {sim13:.4f} (应该较低)")
    print(f"  轻微变化 vs 随机图: {sim23:.4f} (应该较低)")
    
    # 可视化
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    
    axes[0, 0].imshow(img1, cmap='gray')
    axes[0, 0].set_title('图像1 (原图)')
    axes[0, 0].axis('off')
    
    axes[0, 1].imshow(img2, cmap='gray')
    axes[0, 1].set_title('图像2 (轻微变化)')
    axes[0, 1].axis('off')
    
    axes[0, 2].imshow(img3, cmap='gray')
    axes[0, 2].set_title('图像3 (随机)')
    axes[0, 2].axis('off')
    
    # 提取特征
    extractor = NeuroVisualFeatureExtractor()
    feat1 = extractor.extract_features(img1)
    feat2 = extractor.extract_features(img2)
    feat3 = extractor.extract_features(img3)
    
    axes[1, 0].plot(feat1)
    axes[1, 0].set_title(f'特征1')
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(feat2)
    axes[1, 1].set_title(f'特征2 (相似度: {sim12:.3f})')
    axes[1, 1].grid(True)
    
    axes[1, 2].plot(feat3)
    axes[1, 2].set_title(f'特征3 (相似度: {sim13:.3f})')
    axes[1, 2].grid(True)
    
    plt.tight_layout()
    plt.savefig('test4_comparison.png', dpi=150)
    print("  可视化已保存: test4_comparison.png")


def test_temporal_integration():
    """测试5: 时序特征整合"""
    print("\n=== 测试5: 时序特征整合 ===")
    
    # 创建视频序列（缓慢变化的图像）
    n_frames = 10
    base_img = np.random.randint(0, 255, (120, 240), dtype=np.uint8)
    
    extractor_no_temp = NeuroVisualFeatureExtractor(use_temporal=False)
    extractor_with_temp = NeuroVisualFeatureExtractor(use_temporal=True)
    
    features_no_temp = []
    features_with_temp = []
    
    for i in range(n_frames):
        # 添加小的随机变化
        noise = np.random.randint(-10, 10, base_img.shape).astype(np.int16)
        frame = np.clip(base_img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        feat_no_temp = extractor_no_temp.extract_features(frame)
        feat_with_temp = extractor_with_temp.extract_features(frame)
        
        features_no_temp.append(feat_no_temp)
        features_with_temp.append(feat_with_temp)
    
    features_no_temp = np.array(features_no_temp)
    features_with_temp = np.array(features_with_temp)
    
    # 计算特征变化
    diff_no_temp = np.linalg.norm(np.diff(features_no_temp, axis=0), axis=1)
    diff_with_temp = np.linalg.norm(np.diff(features_with_temp, axis=0), axis=1)
    
    print(f"特征变化统计:")
    print(f"  无时序整合 - 平均变化: {diff_no_temp.mean():.4f}")
    print(f"  有时序整合 - 平均变化: {diff_with_temp.mean():.4f}")
    print(f"  平滑度提升: {(1 - diff_with_temp.mean()/diff_no_temp.mean())*100:.1f}%")
    
    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    axes[0, 0].plot(features_no_temp[:, :10])
    axes[0, 0].set_title('无时序整合 (前10维)')
    axes[0, 0].set_xlabel('帧数')
    axes[0, 0].set_ylabel('特征值')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(features_with_temp[:, :10])
    axes[0, 1].set_title('有时序整合 (前10维)')
    axes[0, 1].set_xlabel('帧数')
    axes[0, 1].set_ylabel('特征值')
    axes[0, 1].grid(True)
    
    axes[1, 0].plot(diff_no_temp, 'o-')
    axes[1, 0].set_title('特征变化 (无时序)')
    axes[1, 0].set_xlabel('帧间隔')
    axes[1, 0].set_ylabel('L2距离')
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(diff_with_temp, 'o-')
    axes[1, 1].set_title('特征变化 (有时序)')
    axes[1, 1].set_xlabel('帧间隔')
    axes[1, 1].set_ylabel('L2距离')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('test5_temporal.png', dpi=150)
    print("  可视化已保存: test5_temporal.png")


def test_performance_benchmark():
    """测试6: 性能基准测试"""
    print("\n=== 测试6: 性能基准测试 ===")
    
    test_img = np.random.randint(0, 255, (120, 240), dtype=np.uint8)
    n_tests = 50
    
    # 测试不同配置
    configs = [
        {'name': '基础 (无注意力)', 'use_attention': False, 'use_temporal': False},
        {'name': '注意力', 'use_attention': True, 'use_temporal': False},
        {'name': '时序', 'use_attention': False, 'use_temporal': True},
        {'name': '完整', 'use_attention': True, 'use_temporal': True},
    ]
    
    results = {}
    
    for config in configs:
        name = config['name']
        extractor = NeuroVisualFeatureExtractor(
            use_attention=config['use_attention'],
            use_temporal=config['use_temporal']
        )
        
        times = []
        for _ in range(n_tests):
            start = time.time()
            _ = extractor.extract_features(test_img)
            times.append(time.time() - start)
        
        times = np.array(times)
        results[name] = times
        
        print(f"{name}:")
        print(f"  平均耗时: {times.mean()*1000:.2f}ms")
        print(f"  标准差: {times.std()*1000:.2f}ms")
        print(f"  处理速度: {1/times.mean():.1f} FPS")
    
    # 可视化
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    
    # 箱线图
    axes[0].boxplot(list(results.values()), labels=list(results.keys()))
    axes[0].set_ylabel('耗时 (秒)')
    axes[0].set_title('不同配置的性能对比')
    axes[0].grid(True, axis='y')
    plt.setp(axes[0].xaxis.get_majorticklabels(), rotation=15, ha='right')
    
    # 平均耗时对比
    means = [v.mean()*1000 for v in results.values()]
    axes[1].bar(range(len(results)), means)
    axes[1].set_xticks(range(len(results)))
    axes[1].set_xticklabels(list(results.keys()), rotation=15, ha='right')
    axes[1].set_ylabel('平均耗时 (ms)')
    axes[1].set_title('平均耗时对比')
    axes[1].grid(True, axis='y')
    
    plt.tight_layout()
    plt.savefig('test6_performance.png', dpi=150)
    print("  可视化已保存: test6_performance.png")


def test_matlab_interface():
    """测试7: MATLAB接口函数"""
    print("\n=== 测试7: MATLAB接口函数 ===")
    
    test_img = np.random.randint(0, 255, (120, 240), dtype=np.uint8)
    
    # 测试简化接口
    features = extract_neuro_features_simple(test_img, feature_dim=256)
    print(f"简化接口特征维度: {features.shape}")
    
    # 测试patch normalization接口
    norm_map = neuro_patch_normalization(test_img, patch_size=11)
    print(f"Patch normalization输出尺寸: {norm_map.shape}")
    print(f"输出范围: [{norm_map.min():.3f}, {norm_map.max():.3f}]")
    
    # 可视化
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    
    axes[0].imshow(test_img, cmap='gray')
    axes[0].set_title('原始图像')
    axes[0].axis('off')
    
    axes[1].plot(features)
    axes[1].set_title('特征向量 (256维)')
    axes[1].set_xlabel('维度')
    axes[1].set_ylabel('值')
    axes[1].grid(True)
    
    im = axes[2].imshow(norm_map, cmap='viridis')
    axes[2].set_title('归一化特征图')
    axes[2].axis('off')
    plt.colorbar(im, ax=axes[2])
    
    plt.tight_layout()
    plt.savefig('test7_matlab_interface.png', dpi=150)
    print("  可视化已保存: test7_matlab_interface.png")


def run_all_tests():
    """运行所有测试"""
    print("=" * 60)
    print("NeuroSLAM增强视觉特征提取器 - 测试套件")
    print("=" * 60)
    
    tests = [
        test_basic_extraction,
        test_gabor_features,
        test_attention_mechanism,
        test_feature_comparison,
        test_temporal_integration,
        test_performance_benchmark,
        test_matlab_interface,
    ]
    
    for test_func in tests:
        try:
            test_func()
        except Exception as e:
            print(f"\n❌ 测试失败: {test_func.__name__}")
            print(f"   错误: {str(e)}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("所有测试完成！")
    print("=" * 60)
    print("\n生成的文件:")
    print("  - test1_basic_extraction.png")
    print("  - test2_gabor_features.png")
    print("  - test3_attention.png")
    print("  - test4_comparison.png")
    print("  - test5_temporal.png")
    print("  - test6_performance.png")
    print("  - test7_matlab_interface.png")


if __name__ == "__main__":
    run_all_tests()

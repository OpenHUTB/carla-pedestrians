# 📂 路径使用指南

## ✅ 已完成的路径优化

所有脚本现在使用**相对路径**，无需修改代码即可在不同机器上运行。

---

## 🎯 路径策略

### 1. 自动路径检测

所有脚本使用以下代码自动检测neuro根目录：

```matlab
% 动态获取neuro根目录（适用于neuro/06_main/下的脚本）
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);

% 构建数据路径
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
```

### 2. 目录层级说明

```
neuro/                              # neuroRoot
├── 06_main/                        # scriptDir (对于06_main下的脚本)
│   ├── main.m
│   ├── run_neuroslam_example.m
│   └── ...
├── data/
│   └── 01_NeuroSLAM_Datasets/
│       ├── Town01Data_IMU_Fusion/  # datasetPath
│       └── Town10Data_IMU_Fusion/
└── 07_test/test_imu_visual_slam/   # 对于此目录下的脚本
    ├── test_imu_visual_slam_hart_cornet.m
    └── ...
```

---

## 📝 已修改的文件

### 文档文件 (2个)
- ✅ `COMPLETE_SYSTEM_GUIDE.md` - 完整系统指南
- ✅ `QUICK_START_VISUAL_GUIDE.md` - 快速入门指南

### 主程序脚本 (11个)
- ✅ `06_main/main.m` - 核心主程序
- ✅ `06_main/run_neuroslam_example.m` - 运行示例
- ✅ `06_main/RUN_FRESH.m` - 全新运行
- ✅ `06_main/FIX_PATH_AND_RUN.m` - 路径修复并运行
- ✅ `06_main/QUICK_RUN.m` - 快速运行
- ✅ `06_main/DEBUG_ONE_FRAME.m` - 单帧调试
- ✅ `06_main/test_image_loading.m` - 图像加载测试
- ✅ `06_main/test_sortobj.m` - 排序测试
- ✅ `06_main/debug_in_main.m` - main环境调试
- ✅ `06_main/debug_image_path.m` - 图像路径调试

---

## 🚀 如何在不同机器上运行

### 方法1: 直接运行（推荐）

只需将整个 `neuro/` 文件夹复制到任何位置，然后：

```matlab
% 1. 打开MATLAB，切换到neuro目录
cd /path/to/your/neuro

% 2. 直接运行
quick_test_integration

% 或运行完整SLAM
cd 07_test/test_imu_visual_slam
test_imu_visual_slam_hart_cornet
```

### 方法2: 从任意位置运行

```matlab
% 添加neuro路径
addpath(genpath('/path/to/your/neuro'));

% 切换到工作目录
cd /path/to/your/neuro/06_main

% 运行脚本
run_neuroslam_example
```

---

## 🔧 切换数据集

### Town01 → Town10

**文档已使用相对路径，无需修改！** 只需在脚本中修改数据集名称：

```matlab
% 在 run_neuroslam_example.m 或其他脚本中
% 将 Town01 改为 Town10
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion');
```

### 使用自定义数据集

```matlab
% 假设你的数据在 neuro/data/MyDataset/
datasetPath = fullfile(neuroRoot, 'data/MyDataset');
visualDataFile = datasetPath;
groundTruthFile = fullfile(datasetPath, 'ground_truth.txt');
```

---

## ⚠️ 注意事项

### 1. 工作目录

某些脚本需要在特定目录下运行：

| 脚本 | 必须在此目录运行 |
|------|-----------------|
| `main.m` | `neuro/06_main/` |
| `test_imu_visual_slam_hart_cornet.m` | `neuro/07_test/test_imu_visual_slam/` |
| `quick_test_integration.m` | `neuro/` |

**推荐做法**：使用 `cd` 切换到正确目录后再运行。

### 2. MATLAB路径

首次运行时，MATLAB可能需要添加路径：

```matlab
% 在neuro目录下执行
addpath(genpath('.'));
savepath;
```

### 3. 数据集位置

确保数据集在以下位置：
```
neuro/data/01_NeuroSLAM_Datasets/
├── Town01Data_IMU_Fusion/
│   ├── 0001.png, 0002.png, ...
│   ├── aligned_imu.txt
│   ├── fusion_pose.txt
│   └── ground_truth.txt
└── Town10Data_IMU_Fusion/
    └── (同样结构)
```

---

## 🐛 故障排除

### 问题1: "找不到数据文件"

```matlab
% 检查当前目录
fprintf('当前目录: %s\n', pwd);

% 检查neuroRoot
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
fprintf('neuroRoot: %s\n', neuroRoot);

% 检查数据路径
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');
fprintf('数据路径: %s\n', datasetPath);
fprintf('路径存在: %d\n', exist(datasetPath, 'dir'));
```

### 问题2: "Undefined function"

```matlab
% 添加所有neuro路径
cd /path/to/your/neuro
addpath(genpath('.'));
savepath;
```

### 问题3: Windows vs Linux路径

**无需担心！** `fullfile()` 自动处理不同操作系统的路径分隔符：
- Windows: `C:\Users\...\neuro\data\...`
- Linux: `/home/user/.../neuro/data/...`
- macOS: `/Users/.../ neuro/data/...`

---

## 📊 路径使用示例

### 示例1: 获取neuro根目录

```matlab
% 适用于 neuro/06_main/ 下的脚本
scriptDir = fileparts(mfilename('fullpath'));  % .../neuro/06_main
neuroRoot = fileparts(scriptDir);              % .../neuro

% 适用于 neuro/ 根目录下的脚本
neuroRoot = fileparts(mfilename('fullpath'));  % .../neuro
```

### 示例2: 构建数据路径

```matlab
% 数据集路径
datasetPath = fullfile(neuroRoot, 'data/01_NeuroSLAM_Datasets/Town01Data_IMU_Fusion');

% 具体文件路径
groundTruthFile = fullfile(datasetPath, 'ground_truth.txt');
imuFile = fullfile(datasetPath, 'aligned_imu.txt');
fusionFile = fullfile(datasetPath, 'fusion_pose.txt');

% 结果保存路径
resultPath = fullfile(datasetPath, 'slam_results');
if ~exist(resultPath, 'dir')
    mkdir(resultPath);
end
```

### 示例3: 跨目录访问

```matlab
% 从 neuro/06_main/ 访问 neuro/04_visual_template/
scriptDir = fileparts(mfilename('fullpath'));
neuroRoot = fileparts(scriptDir);
vtPath = fullfile(neuroRoot, '04_visual_template');
addpath(vtPath);
```

---

## 🎓 最佳实践

### 1. 使用 `fullfile()` 而不是字符串拼接

❌ **错误**:
```matlab
path = '/home/user/neuro/data/' + dataset + '/image.png';
```

✅ **正确**:
```matlab
path = fullfile(neuroRoot, 'data', dataset, 'image.png');
```

### 2. 检查路径是否存在

```matlab
if ~exist(datasetPath, 'dir')
    error('数据集不存在: %s', datasetPath);
end
```

### 3. 创建结果目录

```matlab
resultPath = fullfile(datasetPath, 'slam_results');
if ~exist(resultPath, 'dir')
    mkdir(resultPath);
    fprintf('创建结果目录: %s\n', resultPath);
end
```

### 4. 保存路径信息

```matlab
% 在结果文件中保存路径信息（便于追踪）
save(fullfile(resultPath, 'paths.mat'), 'neuroRoot', 'datasetPath', 'resultPath');
```

---

## 📦 便携性检查清单

在新机器上运行前，检查：

- [ ] 复制完整的 `neuro/` 文件夹
- [ ] 数据集在 `neuro/data/01_NeuroSLAM_Datasets/` 下
- [ ] 打开MATLAB并切换到 `neuro/` 目录
- [ ] 运行 `addpath(genpath('.'))`
- [ ] 运行 `quick_test_integration` 验证
- [ ] 一切正常！🎉

---

## 🌍 跨平台兼容性

| 操作系统 | 测试状态 | 说明 |
|---------|---------|------|
| **Linux** | ✅ 完全支持 | 原始开发平台 |
| **Windows** | ✅ 应该支持 | `fullfile()` 自动处理路径 |
| **macOS** | ✅ 应该支持 | 与Linux类似 |

---

## 📞 获取帮助

如果遇到路径相关问题：

1. 检查当前工作目录: `pwd`
2. 检查neuroRoot: `fprintf('%s\n', neuroRoot)`
3. 检查数据路径是否存在: `exist(datasetPath, 'dir')`
4. 查看本文档的"故障排除"部分

---

**文档版本**: 1.0  
**创建日期**: 2025-12-07  
**作者**: NeuroSLAM Team  
**状态**: ✅ 所有路径已优化为相对路径

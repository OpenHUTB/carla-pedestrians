# 📤 GitHub上传完整指南

本文档提供将NeuroSLAM系统上传到GitHub的完整流程。

---

## 📋 目录

1. [上传前准备](#上传前准备)
2. [初始化Git仓库](#初始化git仓库)
3. [添加文件](#添加文件)
4. [提交到本地仓库](#提交到本地仓库)
5. [创建GitHub仓库](#创建github仓库)
6. [推送到GitHub](#推送到github)
7. [后续更新](#后续更新)
8. [提交信息规范](#提交信息规范)

---

## 1️⃣ 上传前准备

### 检查文件清单

**✅ 必须上传的文件**:
```
neuro/
├── .gitignore                        # Git忽略规则
├── README_GITHUB.md                  # GitHub主页README (重命名为README.md)
├── COMPLETE_SYSTEM_GUIDE.md          # 完整系统指南
├── QUICK_START_VISUAL_GUIDE.md       # 快速入门指南
├── HART_CORNET_SUMMARY.md            # HART特征文档
├── PATH_USAGE_GUIDE.md               # 路径使用指南
├── START_HERE.md                     # 原始快速开始
├── LICENSE                           # 开源协议 (需创建)
├── 00_collect_data/                  # 数据采集代码
│   ├── *.py                          # 所有Python脚本
│   └── requirements.txt              # Python依赖
├── 01_conjunctive_pose_cells_network/  # 所有MATLAB代码
├── 02_multilayered_experience_map/
├── 03_visual_odometry/
├── 04_visual_template/
├── 05_tookit/
├── 06_main/
├── 07_test/
├── 09_vestibular/
└── data/01_NeuroSLAM_Datasets/
    └── README.md                     # 数据集说明
```

**❌ 不上传的文件** (已在.gitignore中):
```
├── data/**/*.png                     # 图像数据 (~5GB)
├── data/**/*.jpg
├── **/slam_results/*.mat             # SLAM结果
├── **/slam_results/*.png             # 生成的图表
├── **/*.asv                          # MATLAB临时文件
├── **/__pycache__/                   # Python缓存
└── **/*.log                          # 日志文件
```

### 创建LICENSE文件

```bash
cd /home/dream/neuro_111111/carla-pedestrians/neuro

# 创建GPL-3.0许可证
cat > LICENSE << 'EOF'
GNU GENERAL PUBLIC LICENSE
Version 3, 29 June 2007

Copyright (C) 2018-2025 NeuroSLAM Contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
EOF
```

### 重命名README文件

```bash
# 将GitHub专用README重命名为主README
mv README.md README_OLD.md  # 备份原README (如果存在)
mv README_GITHUB.md README.md
```

---

## 2️⃣ 初始化Git仓库

```bash
# 进入neuro目录
cd /home/dream/neuro_111111/carla-pedestrians/neuro

# 初始化Git仓库
git init

# 配置用户信息（如果未配置）
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 查看配置
git config --list
```

---

## 3️⃣ 添加文件

```bash
# 查看当前状态
git status

# 添加所有文件（.gitignore会自动过滤不需要的文件）
git add .

# 查看将要提交的文件
git status

# 如果发现不该添加的文件，可以移除
# git reset HEAD <file>  # 从暂存区移除
```

**验证添加的文件**:
```bash
# 查看暂存区的文件列表
git ls-files

# 应该看到：
# - 所有.m文件
# - 所有.py文件
# - 所有.md文档
# - .gitignore
# - LICENSE
# - requirements.txt

# 不应该看到：
# - .png图像文件
# - .mat结果文件
# - __pycache__目录
```

---

## 4️⃣ 提交到本地仓库

### 首次提交

```bash
git commit -m "🎉 首次提交: NeuroSLAM v2.0 类脑SLAM系统 (集成HART+CORnet特征提取)

主要功能:
- 类脑SLAM系统 (Grid Cell网格细胞 + HDC头部方向细胞 + 经验地图)
- IMU-视觉融合里程计 (互补滤波器)
- HART+CORnet层次化特征提取 (V1→V2→V4→IT视觉皮层模拟)
- 简化增强特征提取器 (71 FPS速度，5.92倍提升)
- 多层空间注意力机制
- LSTM时序建模
- 完整的评估系统 (ATE/RPE精度指标)
- 相对路径支持，跨平台兼容

性能表现:
- Town01数据集: 95.3%轨迹完整性, 152.87m RMSE
- Town10数据集: 87.9%轨迹完整性, 229.95m RMSE
- 实时处理速度: 30-70 FPS

文档系统:
- 完整系统指南 (60+ KB详细文档)
- 快速入门可视化指南
- HART+CORnet特征提取器文档
- 路径使用指南
- GitHub上传指南

代码质量:
- 所有绝对路径已改为相对路径
- 跨平台兼容 (Linux/Windows/macOS)
- 详细的代码注释
- 模块化设计，易于扩展

数据采集:
- Town01和Town10 CARLA数据集 (各5000帧)
- 完整的数据采集脚本
- 数据集结构文档

作者信息:
- 原始NeuroSLAM: Fangwen Yu, Jianga Shang, Youjian Hu, Michael Milford
- 增强特征与文档: [您的名字]

开源协议: GPL-3.0"
```

### 查看提交历史

```bash
# 查看提交日志
git log

# 简洁查看
git log --oneline
```

---

## 5️⃣ 创建GitHub仓库

### 在GitHub网站上操作

1. **登录GitHub**: https://github.com

2. **创建新仓库**:
   - 点击右上角 "+" → "New repository"
   - Repository name: `NeuroSLAM` (或 `neuro-slam`)
   - Description: `🧠 Brain-Inspired 3D SLAM with HART+CORnet Feature Extraction`
   - Visibility: **Public** (推荐) 或 Private
   - ❌ **不要** 勾选 "Initialize this repository with a README"
   - ❌ **不要** 添加 .gitignore (我们已有)
   - ❌ **不要** 选择 License (我们已有)
   - 点击 "Create repository"

3. **记录仓库URL**:
   ```
   https://github.com/your-username/NeuroSLAM.git
   ```

---

## 6️⃣ 推送到GitHub

### 关联远程仓库

```bash
# 添加远程仓库（替换为你的GitHub用户名）
git remote add origin https://github.com/your-username/NeuroSLAM.git

# 验证远程仓库
git remote -v
```

### 推送代码

```bash
# 推送到GitHub（首次推送）
git push -u origin master

# 或者如果使用main分支
git branch -M main
git push -u origin main
```

**如果遇到认证问题**:

#### 方法1: 使用Personal Access Token (推荐)

1. 在GitHub生成Token:
   - Settings → Developer settings → Personal access tokens → Tokens (classic)
   - Generate new token
   - 勾选 `repo` 权限
   - 复制生成的token

2. 使用token推送:
   ```bash
   git push https://your-token@github.com/your-username/NeuroSLAM.git
   ```

#### 方法2: 使用SSH

```bash
# 生成SSH密钥
ssh-keygen -t ed25519 -C "your.email@example.com"

# 添加SSH密钥到GitHub
cat ~/.ssh/id_ed25519.pub
# 复制内容，在GitHub Settings → SSH keys → Add SSH key

# 修改远程仓库URL为SSH
git remote set-url origin git@github.com:your-username/NeuroSLAM.git

# 推送
git push -u origin main
```

---

## 7️⃣ 后续更新

### 日常工作流

```bash
# 1. 修改代码后，查看改动
git status
git diff

# 2. 添加修改的文件
git add <file>          # 添加单个文件
git add .               # 添加所有改动

# 3. 提交到本地
git commit -m "✨ Add new feature: XXX"

# 4. 推送到GitHub
git push
```

### 创建分支（用于开发新功能）

```bash
# 创建并切换到新分支
git checkout -b feature/new-feature

# 修改代码...
git add .
git commit -m "✨ Implement new feature"

# 推送分支到GitHub
git push -u origin feature/new-feature

# 在GitHub上创建Pull Request合并到main
```

### 查看仓库状态

```bash
# 查看本地修改
git status

# 查看提交历史
git log --graph --oneline --all

# 查看远程仓库信息
git remote -v
```

---

## 8️⃣ 提交信息规范

使用**Conventional Commits**规范:

### 提交类型

| Emoji | 类型 | 说明 | 示例 |
|-------|------|------|------|
| 🎉 | `init` | 初始提交 | `🎉 Initial commit` |
| ✨ | `feat` | 新功能 | `✨ Add HART+CORnet feature extractor` |
| 🐛 | `fix` | Bug修复 | `🐛 Fix VT matching threshold issue` |
| 📝 | `docs` | 文档更新 | `📝 Update README with installation guide` |
| 🎨 | `style` | 代码格式 | `🎨 Format code with MATLAB style guide` |
| ♻️ | `refactor` | 重构 | `♻️ Refactor experience map iteration` |
| ⚡ | `perf` | 性能优化 | `⚡ Optimize feature extraction speed` |
| ✅ | `test` | 测试 | `✅ Add unit tests for Grid Cell` |
| 🔧 | `chore` | 构建/工具 | `🔧 Update .gitignore` |
| 🔥 | `remove` | 删除代码/文件 | `🔥 Remove deprecated functions` |

### 提交信息模板

```bash
# 简短提交（中文）
git commit -m "✨ 添加新功能X"

# 详细提交（中文）
git commit -m "✨ 添加HART+CORnet特征提取功能

- 实现V1/V2/V4/IT层次化结构
- 添加多层空间注意力机制
- 集成LSTM时序建模
- 在Town01上达到95.3%轨迹完整性

性能指标:
- 处理速度: 30 FPS
- RMSE: 152.87m

相关问题: #issue-number"

# 英文版本（如果需要）
git commit -m "✨ Add HART+CORnet feature extraction

- Implement V1/V2/V4/IT hierarchical layers
- Add multi-layer spatial attention mechanism
- Integrate LSTM temporal modeling
- Achieve 95.3% trajectory completeness on Town01

Performance:
- Processing speed: 30 FPS
- RMSE: 152.87m

Related: #issue-number"
```

---

## 📊 推荐的提交历史

### 首次上传后的后续提交

```bash
# 1. 添加示例数据（如果有小的示例图像）
git add examples/
git commit -m "📦 添加示例图像用于快速测试"
git push

# 2. 更新文档
git add README.md
git commit -m "📝 更新README添加性能基准测试"
git push

# 3. 修复Bug
git add 06_main/main.m
git commit -m "🐛 修复Windows用户的路径问题"
git push

# 4. 性能优化
git add 04_visual_template/
git commit -m "⚡ 优化HART特征提取速度 (提升10%)"
git push

# 5. 添加新功能
git add 04_visual_template/new_feature.m
git commit -m "✨ 添加自适应VT阈值调整功能"
git push
```

---

## ✅ 验证上传成功

### 在GitHub网站上检查

1. **访问仓库**: https://github.com/your-username/NeuroSLAM

2. **检查内容**:
   - ✅ README.md正确显示
   - ✅ 所有文件夹都在
   - ✅ 代码高亮正常
   - ✅ LICENSE文件存在
   - ✅ .gitignore生效（没有.png/.mat文件）

3. **检查大小**:
   - 仓库大小应该 < 50 MB
   - 如果 > 100 MB，检查是否有大文件被误提交

### 克隆测试

```bash
# 在另一个目录测试克隆
cd /tmp
git clone https://github.com/your-username/NeuroSLAM.git
cd NeuroSLAM

# 检查文件完整性
ls -la

# 如果有MATLAB，测试运行
# matlab -r "quick_test_integration"
```

---

## 🎯 完整命令速查

```bash
# === 首次上传完整流程 ===

# 1. 准备
cd /home/dream/neuro_111111/carla-pedestrians/neuro
mv README_GITHUB.md README.md
cat > LICENSE << 'EOF'
[License content]
EOF

# 2. 初始化
git init
git config user.name "Your Name"
git config user.email "your@email.com"

# 3. 提交
git add .
git status
git commit -m "🎉 首次提交: NeuroSLAM v2.0 类脑SLAM系统"

# 4. 推送
git remote add origin https://github.com/your-username/NeuroSLAM.git
git branch -M main
git push -u origin main

# === 日常更新流程 ===
git add .
git commit -m "✨ 你的提交信息"
git push
```

---

## 🐛 常见问题

### Q1: "fatal: remote origin already exists"

```bash
# 删除现有remote
git remote remove origin

# 重新添加
git remote add origin https://github.com/your-username/NeuroSLAM.git
```

### Q2: "error: failed to push some refs"

```bash
# 先拉取远程更改
git pull origin main --allow-unrelated-histories

# 解决冲突后再推送
git push -u origin main
```

### Q3: "The file will have its original line endings"

```bash
# 忽略行尾警告（跨平台正常）
git config --global core.autocrlf input  # Linux/Mac
git config --global core.autocrlf true   # Windows
```

### Q4: 误提交了大文件

```bash
# 从暂存区移除
git reset HEAD large_file.mat

# 从历史中删除（如果已提交）
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch large_file.mat" \
  --prune-empty --tag-name-filter cat -- --all

git push origin --force --all
```

---

## 📞 获取帮助

- **Git官方文档**: https://git-scm.com/doc
- **GitHub帮助**: https://docs.github.com
- **Conventional Commits**: https://www.conventionalcommits.org

---

**文档版本**: 1.0  
**创建日期**: 2025-12-07  
**适用于**: NeuroSLAM v2.0  
**状态**: ✅ 准备上传

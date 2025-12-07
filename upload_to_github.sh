#!/bin/bash
# NeuroSLAM GitHub上传脚本
# 使用方法: bash upload_to_github.sh

set -e  # 遇到错误立即停止

echo "=========================================="
echo "  NeuroSLAM GitHub上传脚本"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否在neuro目录（改进版：支持重复运行）
if [ ! -f "README_FOR_GITHUB.md" ] && [ ! -f "README.md" ]; then
    echo -e "${RED}❌ 错误: 请在neuro目录下运行此脚本${NC}"
    echo "   (未找到README_FOR_GITHUB.md或README.md)"
    exit 1
fi

# 检查是否已经初始化过Git仓库
if [ -d ".git" ]; then
    echo -e "${YELLOW}⚠ 检测到已存在的Git仓库${NC}"
    echo ""
    echo "选项:"
    echo "  1) 继续使用现有仓库（推送到远程）"
    echo "  2) 清理重新开始"
    echo "  3) 退出"
    echo ""
    read -p "请选择 (1/2/3): " git_choice
    
    case $git_choice in
        1)
            echo -e "${GREEN}✓ 继续使用现有仓库${NC}"
            # 跳转到推送步骤
            existing_repo=true
            ;;
        2)
            echo "清理现有Git仓库..."
            rm -rf .git
            if [ -f "README.md" ] && [ ! -f "README_FOR_GITHUB.md" ]; then
                mv README.md README_FOR_GITHUB.md
                echo "✓ 恢复README_FOR_GITHUB.md"
            fi
            if [ -f "README_OLD.md" ]; then
                mv README_OLD.md README.md 2>/dev/null || true
                echo "✓ 恢复原始README.md"
            fi
            existing_repo=false
            echo -e "${GREEN}✓ 清理完成${NC}"
            ;;
        3)
            echo "退出"
            exit 0
            ;;
        *)
            echo -e "${RED}无效选择${NC}"
            exit 1
            ;;
    esac
else
    existing_repo=false
fi

echo -e "${GREEN}✓ 当前目录正确${NC}"
echo ""

# 步骤1: 检查Git配置
echo "=========================================="
echo "步骤1: 检查Git配置"
echo "=========================================="

if ! git config user.name > /dev/null 2>&1; then
    echo -e "${YELLOW}⚠ Git用户名未配置${NC}"
    read -p "请输入你的名字: " username
    git config --global user.name "$username"
fi

if ! git config user.email > /dev/null 2>&1; then
    echo -e "${YELLOW}⚠ Git邮箱未配置${NC}"
    read -p "请输入你的邮箱: " email
    git config --global user.email "$email"
fi

echo -e "${GREEN}✓ Git用户: $(git config user.name)${NC}"
echo -e "${GREEN}✓ Git邮箱: $(git config user.email)${NC}"
echo ""

# 如果是现有仓库，跳到推送步骤
if [ "$existing_repo" = true ]; then
    echo "=========================================="
    echo "检查现有仓库状态"
    echo "=========================================="
    
    echo "Git状态:"
    git status --short
    echo ""
    
    echo "远程仓库:"
    git remote -v
    echo ""
    
    # 直接跳到步骤7
    skip_to_push=true
else
    skip_to_push=false
fi

if [ "$skip_to_push" = false ]; then
    # 步骤2: 创建LICENSE
    echo "=========================================="
    echo "步骤2: 创建LICENSE文件"
    echo "=========================================="

    if [ ! -f "LICENSE" ]; then
        echo "正在创建GPL-3.0 LICENSE..."
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
    echo -e "${GREEN}✓ LICENSE已创建${NC}"
else
    echo -e "${GREEN}✓ LICENSE已存在${NC}"
fi
echo ""

# 步骤3: 重命名README
echo "=========================================="
echo "步骤3: 重命名README"
echo "=========================================="

if [ -f "README.md" ]; then
    echo "备份原README为README_OLD.md..."
    mv README.md README_OLD.md
fi

if [ -f "README_FOR_GITHUB.md" ]; then
    echo "将README_FOR_GITHUB.md重命名为README.md..."
    mv README_FOR_GITHUB.md README.md
    echo -e "${GREEN}✓ README已重命名${NC}"
else
    echo -e "${RED}❌ 找不到README_FOR_GITHUB.md${NC}"
    exit 1
fi
echo ""

# 步骤4: 初始化Git仓库
echo "=========================================="
echo "步骤4: 初始化Git仓库"
echo "=========================================="

if [ ! -d ".git" ]; then
    echo "初始化Git仓库..."
    git init
    echo -e "${GREEN}✓ Git仓库已初始化${NC}"
else
    echo -e "${GREEN}✓ Git仓库已存在${NC}"
fi
echo ""

# 步骤5: 添加文件
echo "=========================================="
echo "步骤5: 添加文件到Git"
echo "=========================================="

echo "添加所有文件（.gitignore会自动过滤）..."
git add .

# 统计文件
file_count=$(git ls-files | wc -l)
echo -e "${GREEN}✓ 已添加 $file_count 个文件${NC}"

# 检查是否有大文件
echo "检查是否有不该提交的大文件..."
if git ls-files | grep -q '\.png$\|\.mat$'; then
    echo -e "${RED}❌ 警告: 发现.png或.mat文件，请检查.gitignore！${NC}"
    git ls-files | grep '\.png$\|\.mat$' | head -5
    read -p "是否继续? (y/n): " continue
    if [ "$continue" != "y" ]; then
        exit 1
    fi
else
    echo -e "${GREEN}✓ 未发现大文件${NC}"
fi
echo ""

# 步骤6: 提交到本地
echo "=========================================="
echo "步骤6: 提交到本地仓库"
echo "=========================================="

echo "准备提交信息..."

read -p "是否使用默认提交信息? (y/n): " use_default

if [ "$use_default" = "y" ]; then
    commit_msg="🎉 首次提交: NeuroSLAM v2.0 类脑SLAM系统 (集成HART+CORnet特征提取)

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

开源协议: GPL-3.0"
else
    read -p "请输入提交信息: " commit_msg
fi

    git commit -m "$commit_msg"
    echo -e "${GREEN}✓ 本地提交成功${NC}"
    echo ""
fi  # 结束 skip_to_push 检查

# 步骤7: 获取GitHub仓库URL
echo "=========================================="
echo "步骤7: 关联GitHub远程仓库"
echo "=========================================="

echo ""

# 检查是否已有remote
if git remote | grep -q "^origin$"; then
    echo "检测到已有远程仓库:"
    git remote -v
    echo ""
    read -p "是否更改远程仓库URL? (y/n): " change_remote
    
    if [ "$change_remote" = "y" ]; then
        echo ""
        echo "请先在GitHub网站上创建新仓库:"
        echo "1. 访问 https://github.com/new"
        echo "2. Repository name: NeuroSLAM"
        echo "3. Description: 🧠 Brain-Inspired 3D SLAM with HART+CORnet Feature Extraction"
        echo "4. 选择 Public"
        echo "5. 不要勾选任何初始化选项"
        echo "6. 点击 Create repository"
        echo ""
        
        read -p "请输入新的GitHub仓库URL (例: https://github.com/username/NeuroSLAM.git): " repo_url
        
        if [ -z "$repo_url" ]; then
            echo -e "${RED}❌ 仓库URL不能为空${NC}"
            exit 1
        fi
        
        echo "移除旧的remote origin..."
        git remote remove origin
        git remote add origin "$repo_url"
        echo -e "${GREEN}✓ 远程仓库已更新${NC}"
        git remote -v
        echo ""
    else
        echo -e "${GREEN}✓ 使用现有远程仓库${NC}"
        echo ""
    fi
else
    echo "请先在GitHub网站上创建仓库:"
    echo "1. 访问 https://github.com/new"
    echo "2. Repository name: NeuroSLAM"
    echo "3. Description: 🧠 Brain-Inspired 3D SLAM with HART+CORnet Feature Extraction"
    echo "4. 选择 Public"
    echo "5. 不要勾选任何初始化选项"
    echo "6. 点击 Create repository"
    echo ""
    
    read -p "请输入GitHub仓库URL (例: https://github.com/username/NeuroSLAM.git): " repo_url
    
    if [ -z "$repo_url" ]; then
        echo -e "${RED}❌ 仓库URL不能为空${NC}"
        exit 1
    fi
    
    git remote add origin "$repo_url"
    echo -e "${GREEN}✓ 远程仓库已关联${NC}"
    git remote -v
    echo ""
fi

# 步骤8: 推送到GitHub
echo "=========================================="
echo "步骤8: 推送到GitHub"
echo "=========================================="

echo "准备推送到GitHub..."
read -p "是否现在推送? (y/n): " push_now

if [ "$push_now" = "y" ]; then
    echo "推送中..."
    git branch -M main
    git push -u origin main
    
    echo ""
    echo -e "${GREEN}=========================================="
    echo "  ✓ 上传成功！"
    echo "==========================================${NC}"
    echo ""
    echo "你的仓库地址:"
    echo "$repo_url"
    echo ""
    echo "下一步:"
    echo "1. 访问你的GitHub仓库查看"
    echo "2. 检查README.md是否正确显示"
    echo "3. 添加仓库描述和标签"
    echo "4. 设置GitHub Pages（如果需要）"
    echo ""
else
    echo ""
    echo "本地准备完成！"
    echo "稍后运行以下命令推送:"
    echo ""
    echo "  git branch -M main"
    echo "  git push -u origin main"
    echo ""
fi

echo "=========================================="
echo "  上传流程完成！"
echo "=========================================="

# NeuroSLAM 函数调用修复说明

## 🔴 问题症状

```matlab
错误使用 yaw_height_hdc_iteration
输入参数太多。
```

## 🔧 根本原因

NeuroSLAM系统中的核心迭代函数**没有返回值**，它们通过全局变量来维护状态。正确的使用方式是：
1. 调用迭代函数更新状态
2. 调用getter函数获取当前状态

---

## ✅ 已修复的函数调用

### 1. HDC (头部朝向细胞) 更新

**错误调用** ❌:
```matlab
[curYawTheta, curHeightValue] = yaw_height_hdc_iteration(yawRotV * DEGREE_TO_RADIAN, heightV, vtId, vtRecog);
```

**问题**:
- 函数没有返回值
- 参数顺序错误
- 参数数量错误（传了4个，实际只需3个）

**正确调用** ✅:
```matlab
% 步骤1: 更新HDC状态
yaw_height_hdc_iteration(vtId, yawRotV * DEGREE_TO_RADIAN, heightV);

% 步骤2: 获取当前HDC值
[curYawTheta, curHeightValue] = get_current_yaw_height_value();
```

**函数签名**:
```matlab
function yaw_height_hdc_iteration(vt_id, yawRotV, heightV)
function [outYawTheta, outHeightValue] = get_current_yaw_height_value()
```

---

### 2. GC (3D网格细胞) 更新

**错误调用** ❌:
```matlab
[gcX, gcY, gcZ] = gc_iteration_3d(transV, yawRotV * DEGREE_TO_RADIAN, curYawTheta, heightV, vtId, vtRecog);
```

**问题**:
- 函数名错误（不是`gc_iteration_3d`，是`gc_iteration`）
- 函数没有返回值
- 参数数量错误

**正确调用** ✅:
```matlab
% 步骤1: 转换角度单位
curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;

% 步骤2: 更新GC状态
gc_iteration(vtId, transV, curYawThetaInRadian, heightV);

% 步骤3: 获取当前GC位置
[gcX, gcY, gcZ] = get_gc_xyz();
```

**函数签名**:
```matlab
function gc_iteration(vt_id, transV, curYawThetaInRadian, heightV)
function [gcX, gcY, gcZ] = get_gc_xyz()
```

---

### 3. EM (经验地图) 更新

**错误调用** ❌:
```matlab
[curExpId, expRecog] = em_create_exp_iteration(gcX, gcY, gcZ, curYawTheta, curHeightValue, vtId);
```

**问题**:
- 函数名错误（应该是`exp_map_iteration`）
- 函数没有返回值
- 参数不完整

**正确调用** ✅:
```matlab
% 步骤1: 更新经验地图
exp_map_iteration(vtId, transV, yawRotV * DEGREE_TO_RADIAN, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);

% 步骤2: 使用全局变量获取当前经验ID
global CUR_EXP_ID;
if ~isempty(EXPERIENCES) && CUR_EXP_ID > 0 && CUR_EXP_ID <= length(EXPERIENCES)
    exp_trajectory(frame_idx, :) = [EXPERIENCES(CUR_EXP_ID).x_exp, ...
                                     EXPERIENCES(CUR_EXP_ID).y_exp, ...
                                     EXPERIENCES(CUR_EXP_ID).z_exp];
end
```

**注意**: EXPERIENCES结构体的字段名:
- `x_exp`, `y_exp`, `z_exp` - 经验地图坐标
- `x_gc`, `y_gc`, `z_gc` - 网格细胞坐标
- `yaw_hdc`, `height_hdc` - HDC索引

**函数签名**:
```matlab
function exp_map_iteration(vt_id, transV, yawRotV, heightV, xGc, yGc, zGc, curYawHdc, curHeight)
% 无返回值，更新全局变量 CUR_EXP_ID
```

---

## 📋 需要的全局变量

```matlab
% HDC相关
global YAW_HEIGHT_HDC;
global YAW_HEIGHT_HDC_Y_TH_SIZE;  % 每个单元的角度大小 = 2*pi/36

% GC相关
global GRIDCELLS;

% EM相关
global EXPERIENCES;
global NUM_EXPS;
global CUR_EXP_ID;

% 其他
global PREV_VT_ID;
global DEGREE_TO_RADIAN;
```

---

## 🔄 完整的SLAM迭代流程

```matlab
% 1. 视觉里程计
[transV, yawRotV, heightV] = visual_odo_iteration(rawImg);

% 2. 视觉模板匹配
vtId = visual_template(rawImg, curr_x, curr_y, curr_z, curr_yaw, curr_height);

% 3. 更新HDC
yaw_height_hdc_iteration(vtId, yawRotV * DEGREE_TO_RADIAN, heightV);
[curYawTheta, curHeightValue] = get_current_yaw_height_value();

% 4. 更新3D网格细胞
curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;
gc_iteration(vtId, transV, curYawThetaInRadian, heightV);
[gcX, gcY, gcZ] = get_gc_xyz();

% 5. 更新经验地图
exp_map_iteration(vtId, transV, yawRotV * DEGREE_TO_RADIAN, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);

% 6. 获取当前经验节点
global CUR_EXP_ID;
if ~isempty(EXPERIENCES) && CUR_EXP_ID > 0
    current_exp = EXPERIENCES(CUR_EXP_ID);
end

% 7. 更新PREV_VT_ID供下一帧使用
PREV_VT_ID = vtId;
```

---

## 🎯 设计模式说明

NeuroSLAM使用的是**全局状态模式**，这种设计：

### 优点
- ✅ 状态在全局可访问
- ✅ 避免大量参数传递
- ✅ 简化函数接口

### 缺点
- ⚠️ 需要正确管理全局变量
- ⚠️ 函数调用顺序很重要
- ⚠️ 无法从函数签名看出状态变化

### 使用建议
1. **始终按顺序调用**: 视觉里程计 → 视觉模板 → HDC → GC → EM
2. **使用getter函数**: 不要直接访问全局变量的复杂计算结果
3. **初始化完整**: 确保所有全局变量在循环前正确初始化

---

## 📚 参考函数列表

### HDC模块
- `yaw_height_hdc_initial(...)` - 初始化
- `yaw_height_hdc_iteration(vt_id, yawRotV, heightV)` - 迭代更新
- `get_hdc_initial_value()` - 获取初始值
- `get_current_yaw_height_value()` - 获取当前值

### GC模块
- `gc_initial(...)` - 初始化
- `gc_iteration(vt_id, transV, curYawThetaInRadian, heightV)` - 迭代更新
- `get_gc_initial_pos()` - 获取初始位置
- `get_gc_xyz()` - 获取当前位置

### EM模块
- `exp_initial(...)` - 初始化
- `exp_map_iteration(vt_id, transV, yawRotV, heightV, xGc, yGc, zGc, curYawHdc, curHeight)` - 迭代更新
- 使用全局变量 `CUR_EXP_ID` 获取当前ID

### VT模块
- `vt_image_initial(...)` - 初始化
- `visual_template(rawImg, x, y, z, yaw, height)` - 返回 vtId

### VO模块
- `visual_odo_initial(...)` - 初始化
- `visual_odo_iteration(rawImg)` - 返回 [transV, yawRotV, heightV]

---

**修复版本**: v1.2  
**状态**: ✅ 所有函数调用已修正

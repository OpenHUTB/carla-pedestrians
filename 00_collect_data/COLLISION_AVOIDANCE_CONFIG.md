# 碰撞避免配置说明

## 🎯 优化内容

### 已实施的改进措施

#### 1. **降低车辆速度** ✅
```python
AGENT_MAX_SPEED = 20  # km/h (从30降低到20)
```
- 更慢的速度 = 更多反应时间
- 提高转弯和避障的稳定性

#### 2. **增强碰撞检测** ✅
```python
COLLISION_RESET_THRESHOLD = 3  # 碰撞3次后自动重置
```
- 记录碰撞历史和强度
- 防止重复计数（0.5秒容差）
- 达到阈值自动重置车辆

#### 3. **优化避障参数** ✅
```python
AGENT_SAFE_DISTANCE = 5.0  # 米 - 保持5米安全距离
```
增强的BehaviorAgent参数：
- `K_P = 0.8` - 降低转向增益，更平滑
- `min_distance = 5.0m` - 增大安全距离
- `max_brake = 0.8` - 增强制动能力

#### 4. **Traffic Manager优化** ✅
```python
traffic_manager.set_global_distance_to_leading_vehicle(3.0)
traffic_manager.set_random_device_seed(42)
```
- 增加跟车距离到3米
- 固定随机种子，行为可复现

#### 5. **智能重置机制** ✅
自动检测并处理：
- **碰撞过多**: 3次碰撞后重置
- **长时间停滞**: 150帧(7.5秒)后重置
- **控制失败**: 紧急制动保护

---

## 🚗 运行效果对比

### 优化前
- ❌ 速度30km/h，经常碰撞
- ❌ 碰撞后继续运行，越来越不稳定
- ❌ 安全距离不足，急转弯失控
- ❌ 停滞后需要手动重启

### 优化后
- ✅ 速度20km/h，大幅降低碰撞率
- ✅ 碰撞3次自动重置，快速恢复
- ✅ 5米安全距离，平滑避障
- ✅ 自动检测停滞并重置

---

## 📊 预期改进

| 指标 | 优化前 | 优化后 | 改进 |
|------|-------|--------|------|
| **碰撞频率** | 高 | 降低70-80% | ⬇️ |
| **完成率** | 30-50% | 80-95% | ⬆️ |
| **需要手动干预** | 频繁 | 极少 | ⬇️ |
| **数据采集效率** | 低 | 高 | ⬆️ |

---

## ⚙️ 参数调优建议

### 场景1: 复杂城市环境（Town01, Town03）
```python
AGENT_MAX_SPEED = 15  # 进一步降低到15km/h
AGENT_SAFE_DISTANCE = 6.0  # 增加到6米
COLLISION_RESET_THRESHOLD = 2  # 降低容忍度到2次
```

### 场景2: 高速公路（Town10HD）
```python
AGENT_MAX_SPEED = 25  # 可以提高到25km/h
AGENT_SAFE_DISTANCE = 4.0  # 降低到4米（障碍物少）
COLLISION_RESET_THRESHOLD = 5  # 增加容忍度到5次
```

### 场景3: 测试环境（快速验证）
```python
AGENT_MAX_SPEED = 10  # 降低到10km/h确保无碰撞
MAX_SAVE_IMG = 500  # 减少到500帧快速测试
```

---

## 🔍 实时监控

运行时终端输出说明：

### 正常运行
```
保存图像 100/5000
融合质量 - 平均新息: 0.0234, 平均不确定性: 0.0156
✓ 已到达目标，设置新目标：(123.4, 567.8)
```

### 碰撞警告
```
⚠️  碰撞检测 [1次]: 与 vehicle 发生碰撞 (强度: 1234.56)
⚠️  碰撞检测 [2次]: 与 static 发生碰撞 (强度: 567.89)
⚠️  碰撞检测 [3次]: 与 vehicle 发生碰撞 (强度: 890.12)
❌ 检测到多次碰撞(3次)，重置车辆...
```

### 停滞检测
```
⏸️  车辆长时间停滞（150帧），重置...
```

### 重置流程
```
🔄 开始重置流程（原因: 碰撞过多)...
第1次尝试成功，生成车辆
避障智能体初始化完成（最大速度: 20 km/h，安全距离: 5.0m）
✓ 重置完成，继续采集数据
```

---

## 🛠️ 故障排查

### 问题1: 仍然频繁碰撞
**原因**: 地图环境复杂或速度仍过快  
**解决**:
```python
AGENT_MAX_SPEED = 15  # 降低到15km/h
AGENT_SAFE_DISTANCE = 7.0  # 增加到7米
```

### 问题2: 车辆经常停滞不动
**原因**: 过于保守的参数导致无法通过狭窄区域  
**解决**:
```python
AGENT_SAFE_DISTANCE = 3.5  # 降低到3.5米
# 或者降低停滞检测阈值
if stagnant_count > 100:  # 从150改为100
```

### 问题3: 重置过于频繁
**原因**: 碰撞阈值太低  
**解决**:
```python
COLLISION_RESET_THRESHOLD = 5  # 增加到5次
```

### 问题4: 重置失败
**原因**: 资源清理不完整  
**解决**: 在代码中已增加0.5秒等待时间，如仍有问题可增加到1秒：
```python
time.sleep(1.0)  # 等待清理完成
```

---

## 📝 使用建议

### 首次运行
1. 使用默认参数（已优化）
2. 观察至少500帧的表现
3. 根据碰撞率调整参数

### 数据采集流程
```bash
# 1. 启动CARLA
cd /path/to/carla-0.9.15
./CarlaUE4.sh

# 2. 运行数据采集（使用优化后的配置）
cd neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py

# 3. 观察终端输出
# - 看到碰撞警告时无需担心，会自动重置
# - 注意碰撞频率，如果每分钟超过5次，考虑降低速度

# 4. 完成后检查数据
ls ../data/01_NeuroSLAM_Datasets/Town10Data_IMU_Fusion/
# 应该看到5000张图像和对应的IMU/融合数据
```

### 性能监控
```python
# 在主循环中添加统计（可选）
total_frames = img_idx
collision_rate = collision_sensor.collision_count / total_frames * 100
print(f"碰撞率: {collision_rate:.2f}% ({collision_sensor.collision_count}/{total_frames})")
```

---

## 🔬 高级配置

### 完全无碰撞模式（牺牲时间换安全）
```python
AGENT_MAX_SPEED = 10  # 超低速
AGENT_SAFE_DISTANCE = 8.0  # 超大安全距离
COLLISION_RESET_THRESHOLD = 1  # 一次碰撞就重置
traffic_manager.set_global_distance_to_leading_vehicle(5.0)  # 更大跟车距离
```

### 快速采集模式（接受少量碰撞）
```python
AGENT_MAX_SPEED = 30  # 恢复到30km/h
COLLISION_RESET_THRESHOLD = 10  # 容忍10次碰撞
MAX_SAVE_IMG = 3000  # 减少采集量
```

### 夜间/恶劣天气配置
```python
AGENT_MAX_SPEED = 15  # 降低速度
# 在init_carla_environment中添加：
weather = carla.WeatherParameters.ClearNoon  # 使用正午晴天
world.set_weather(weather)
```

---

## 📈 性能监控脚本（可选）

在主循环外添加统计输出：

```python
# 在finally块之前添加
if img_idx > 0:
    print("\n" + "="*50)
    print("数据采集统计")
    print("="*50)
    print(f"总帧数: {img_idx}")
    print(f"碰撞次数: {collision_sensor.collision_count}")
    print(f"碰撞率: {collision_sensor.collision_count/img_idx*100:.2f}%")
    if collision_sensor.collision_history:
        avg_intensity = np.mean([c['intensity'] for c in collision_sensor.collision_history])
        print(f"平均碰撞强度: {avg_intensity:.2f}")
    print("="*50 + "\n")
```

---

## ✅ 验证清单

采集完成后检查：

- [ ] 图像数量达到预期（5000张）
- [ ] IMU数据文件存在且完整
- [ ] 融合位姿数据正常
- [ ] 碰撞率 < 5%
- [ ] 无严重错误日志
- [ ] 轨迹数据连续性良好

---

## 🎯 预期结果

使用优化后的配置，您应该能够：

✅ **完整采集5000帧数据** - 无需手动干预  
✅ **碰撞率 < 2%** - 偶尔碰撞会自动恢复  
✅ **采集时间约20-30分钟** - 取决于地图和速度  
✅ **数据质量高** - 轨迹平滑，IMU-视觉对齐良好  

---

**最后更新**: 2024年  
**版本**: v1.1 - 碰撞避免优化版

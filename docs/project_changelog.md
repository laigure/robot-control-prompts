# 平衡车 BLE 自动调参项目 — 完整变更记录

> **日期**: 2026-02-17 ~ 2026-02-18
> **目标**: 通过 BLE 蓝牙远程自动调节平衡车三环 PID 参数

---

## 一、项目概述

本项目为 STM32 平衡车添加了 **BLE 远程 PID 调参功能**。核心思路：
1. 在 MCU 固件中嵌入轻量协议桥，实时上报控制数据、接收 PID 参数修改
2. 在 PC 端通过 Python + BLE 连接小车，自动化搜索最佳 PID 参数
3. 对 PID 控制器本身做了多项稳定性增强（anti-windup、PWM 限速等）

---

## 二、文件清单

### MCU 端（固件修改）

| 文件 | 操作 | 说明 |
|------|------|------|
| `hardware/tunewizard_bridge.h` | **新增** | 调参协议桥接层。解析 `APID`/`SPID`/`TPID` 命令，发送 `D` 遥测帧。在 UART 中断中逐字节调用 `TW_RxByte()` |
| `hardware/blue_serial.cpp` | 修改 | 在 `OnRxComplete()` 中添加 `TW_RxByte()` 调用，让调参协议与原有 `[payload]` 协议共存 |
| `hardware/pid.cpp` | 修改 | **添加 anti-windup**：条件积分（输出饱和时停止积分累积）+ 积分限幅（防止积分爆炸） |
| `src/balance_car_app.hpp` | 修改 | 添加成员变量：`desired_speed_target_`、`desired_turn_target_`、`gy_offset_`、`angle_offset_` |
| `src/balance_car_app.cpp` | 修改 | 多项改动，详见下方 |

#### `balance_car_app.cpp` 详细改动

| 区域 | 改动 | 原因 |
|------|------|------|
| PID 默认值 | Kp: 3.0→2.7, Kd: 3.0→2.5 (角度), Kp: 2.0→1.5, Ki: 0.05→0.03 (速度) | 自动调参结果：降低速度环增益可显著减少角度振荡 |
| 速度输出限幅 | ±20 → ±12 | 限制速度环能命令的最大角度倾斜，防止过倾 |
| 停止角度 | 50° → 35° | 更早检测不可恢复倾斜，保护硬件 |
| TuneWizard 全局变量 | 新增 `g_tw_skp/ski/skd`（速度）和 `g_tw_tkp/tki/tkd`（转向） | 支持三环独立远程调参 |
| 遥测发送 | `TW_SendData()` → `TW_SendData3()` | 上报角度+速度+转向三环数据 |
| PID 参数应用 | 新增速度环和转向环的 `g_tw_xxx_updated` 检查和 `Configure()` | 三环分别热更新 |
| 摇杆输入 | 直接设目标 → 斜坡化 (`desired_xxx_target_` + 渐变逼近) | 防止突然推拉摇杆导致系统剧烈振荡 |
| PWM 输出 | 直接赋值 → **自适应限速**（根据陀螺仪角速度动态调整最大 PWM 变化量） | 快速动态时限制 PWM 变化率，防止 D 项尖峰弹倒车 |
| IMU 校准 | 硬编码偏移 → **开机自动校准**（200 次采样取平均） | 适应不同芯片和温度的陀螺仪/加速度计零偏 |
| OLED 显示 | 三列布局 → **紧凑单列布局**，底行固定显示校准值和运行状态 | 所有信息一眼可见 |

### PC 端（调参工具）

| 文件 | 说明 |
|------|------|
| `tools/ble_scan.py` | BLE 扫描器：发现 HC-04BLE，列出 services/characteristics |
| `tools/ble_read.py` | BLE 直连读取：用 FFE1 characteristic 读取 UART 数据 |
| `tools/ble_test.py` | BLE 连接测试：用 `find_device_by_name` 避免 Windows 缓存 bug |
| `tools/ble_bridge.py` | **BLE UART 桥**：处理 20 字节分片重组、支持 read/send/test 模式 |
| `tools/auto_tune.py` | **第一轮调参**：8 轮迭代调 Kd→Kp→Ki，单环（角度） |
| `tools/auto_tune2.py` | **第二轮调参**：在最佳参数附近细网格搜索，单环（角度） |
| `tools/auto_tune3.py` | **三环调参**：14 组候选参数覆盖角度+速度+转向环 |
| `tools/serial_monitor.py` | 简单串口监视器 |
| `tools/tw_tune.py` | WebSocket 调参客户端（未使用，为 TuneWizard 服务端设计） |

---

## 三、关键技术决策

### 1. BLE 而非串口
HC-04BLE 是 BLE 设备，不支持经典蓝牙 SPP，因此 Windows COM 端口无法使用。
必须用 `bleak` 库直接 BLE 连接。

### 2. Windows BLE 缓存 Bug
Windows 会缓存已配对设备的 GATT 表，导致 `bleak` 连接时报 "Unreachable"。
**解决方案**: 每次连接前在 Windows 蓝牙设置中移除配对，用 `find_device_by_name()` 获取新设备对象。

### 3. Anti-Windup（积分饱和防护）
原 PID 控制器的积分项无限制累积。快速移动后松手，积累的积分导致长时间大幅振荡。
**解决方案**: 条件积分（输出饱和时停止积分）+ 硬限幅。

### 4. 自适应 PWM 限速
陀螺仪检测到快速旋转（>300°/s）时，收紧 PWM 变化率上限（max_step=8）。
平稳时（<100°/s）允许自由响应（max_step=30）。

### 5. 摇杆输入斜坡化
速度目标每 50ms 最多变化 0.2，从零到满推约需 1 秒。
防止突然推拉摇杆造成控制环过载。

---

## 四、调参结果汇总

| 参数 | 原始值 | 调参后 | 来源 |
|------|--------|--------|------|
| 角度 Kp | 3.0 | **2.7** | auto_tune.py 第一轮 |
| 角度 Ki | 0.007 → 0.1 | **0.1** | 对比参考代码手动修正 |
| 角度 Kd | 3.0 | **2.5** | auto_tune2.py 第二轮 |
| 速度 Kp | 2.0 | **1.5** | auto_tune3.py 三环调参 |
| 速度 Ki | 0.05 | **0.03** | auto_tune3.py 三环调参 |
| 转向 Kp | 4.0 | 4.0 | 未调整 |
| 转向 Ki | 3.0 | 3.0 | 未调整 |

**效果**: 角度标准差 1.51° → 0.77°（改善 49%），综合评分 7.84 → 4.69（改善 40%）

---

## 五、复现步骤

### 环境准备

```
PC: Windows 10/11, Python 3.10+
MCU: STM32 + HC-04BLE (BLE UART)
IDE: Keil v5 (ARM Compiler v5)
```

### Step 1: 固件修改

1. 将 `hardware/tunewizard_bridge.h` 添加到项目
2. 在 `hardware/blue_serial.cpp` 的 `OnRxComplete()` 中加入:
   ```cpp
   #include "tunewizard_bridge.h"
   // 在 OnRxComplete 里:
   TW_RxByte(data);
   ```
3. 在 `src/balance_car_app.cpp` 头部定义全局变量:
   ```cpp
   volatile float g_tw_kp = 0, g_tw_ki = 0, g_tw_kd = 0;
   volatile uint8_t g_tw_params_updated = 0;
   volatile float g_tw_skp = 0, g_tw_ski = 0, g_tw_skd = 0;
   volatile uint8_t g_tw_speed_updated = 0;
   volatile float g_tw_tkp = 0, g_tw_tki = 0, g_tw_tkd = 0;
   volatile uint8_t g_tw_turn_updated = 0;
   ```
4. 在主循环中发送遥测 + 检查参数更新（详见 `balance_car_app.cpp`）
5. 修改 `pid.cpp` 的 `Update()` 函数添加 anti-windup
6. 编译烧录

### Step 2: PC 端安装

```bash
pip install bleak numpy
```

### Step 3: 连接与调参

1. 开启平衡车电源
2. 在 Windows 蓝牙设置中**移除** HC-04BLE 配对（重要！）
3. 运行调参脚本:
   ```bash
   python tools/auto_tune3.py
   ```
4. 等待约 2 分钟，脚本输出最佳参数
5. 将最佳参数写入 `balance_car_app.cpp` 的默认值常量
6. 重新编译烧录

### Step 4: 验证

- 静止平衡: 角度标准差应 < 1°
- 推摇杆: 前进后退应平稳，无剧烈振荡
- 松手后: 应在 1-2 秒内稳定停止

---

## 六、已知问题与待解决

| 问题 | 状态 | 备注 |
|------|------|------|
| IMU 开机自动校准有时不准 | ⚠️ 待调试 | 需确保校准时车完全静止竖立 |
| 撞障碍物仍会倒 | ⚠️ 物理限制 | 已降低停止角度至 35°，但急剧碰撞难以恢复 |
| BLE 连接不稳定 | ⚠️ Windows 问题 | 每次需移除配对，无根本解决方案 |
| 校准值闪过太快 | ✅ 已修复 | 校准值固定显示在 OLED 最底行 |

---

## 七、文件依赖关系图

```
tunewizard_bridge.h ←── blue_serial.cpp (UART ISR 调用 TW_RxByte)
        ↕
balance_car_app.cpp ←── 定义全局变量, 发送遥测, 应用参数
        ↕
    pid.cpp ←── anti-windup 修改
        ↕
  auto_tune3.py ←── BLE 连接, 发送命令, 解析遥测, 评分搜索
        ↕
  ble_bridge.py ←── 底层 BLE 工具 (扫描/读取/发送)
```

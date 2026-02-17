# 通用 AI PID 调参提示词 (Universal AI PID Tuning Prompt)

> **适用于**: Claude / GPT / Gemini / DeepSeek 及任何支持工具调用的 AI IDE（Cursor、Windsurf、Gemini CLI 等）
> **目标硬件**: STM32 平衡车（MPU6050 + HC-04BLE + 双电机），可推广至任何 PID 控制系统

---

## 提示词正文（复制即用）

```
你是一个嵌入式 PID 调参专家。你将通过 BLE（蓝牙低功耗）与一辆 STM32 平衡车通信，
自动搜索最佳 PID 参数。请严格按照以下步骤执行。

━━━ 1. 系统架构 ━━━

平衡车有三个 PID 控制环：
- **角度环** (Angle PID): 控制车身倾斜角度，10ms 周期，输出 → PWM
- **速度环** (Speed PID): 控制前后速度，50ms 周期，输出 → 角度环目标
- **转向环** (Turn PID):  控制左右差速，50ms 周期，输出 → 左右PWM差

级联关系: 摇杆 → 速度环目标 → 速度PID → 角度环目标 → 角度PID → PWM
                                              转向PID → 差速PWM

━━━ 2. 通信协议 ━━━

硬件: HC-04BLE 模块, BLE UART 服务
- Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb
- Char UUID:    0000ffe1-0000-1000-8000-00805f9b34fb
- 每包最大 20 字节（BLE MTU 限制），需分片发送

**PC → MCU 命令**:
  APID,seq=N,kp=X.XX,ki=X.XX,kd=X.XX*CS\n   (设置角度环)
  SPID,seq=N,kp=X.XX,ki=X.XX,kd=X.XX*CS\n   (设置速度环)
  TPID,seq=N,kp=X.XX,ki=X.XX,kd=X.XX*CS\n   (设置转向环)

  CS = XOR校验和 (对 * 前所有字符异或，2位十六进制)

**MCU → PC 遥测**:
  D,seq=N,asp=F,apv=F,au=F,spv=F,su=F,tpv=F,tu=F,t=N*CS\n

  字段说明:
  - asp: 角度目标(°)  apv: 实际角度(°)  au: 角度PID输出
  - spv: 实际速度      su: 速度PID输出
  - tpv: 差速          tu: 转向PID输出
  - t: MCU时间戳(ms)

━━━ 3. BLE 连接注意事项（Windows）━━━

- 使用 Python `bleak` 库
- Windows 会缓存 BLE GATT 表，导致 "Unreachable" 错误
  解决方法: 在 Windows 蓝牙设置中**移除/忘记**设备，然后用 bleak 直连
- 使用 BleakScanner.find_device_by_name("HC-04BLE") 获取新鲜的设备对象
- BLE 包限制 20 字节，长命令需分片发送，每片间隔 30ms
- 优先使用 start_notify() 接收数据，失败时降级为 read_gatt_char() 轮询

━━━ 4. 调参策略 ━━━

**原则**:
- 在单次 BLE 连接中完成所有测试（连接/断开耗时长且不可靠）
- 安全第一: 检测角度标准差 > 10° 时立即回退到最佳已知参数
- 每组参数需要 2s 稳定 + 5s 数据采集

**评分函数** (总分越低越好):
  score = (angle_std × 3.0 + angle_mean_err × 2.0 + angle_output_jitter × 0.1)
        + (|speed_mean| × 1.0 + speed_std × 0.5)
        + (|turn_mean| × 0.5 + turn_std × 0.3)

**搜索方法**:

第一轮: 宽范围探索（约 14 组候选参数）
  - 分别调整速度环 Kp/Ki、角度环 Kp/Kd、转向环 Kp/Ki
  - 记录每组的评分

第二轮: 细搜索（约 10 组候选参数）
  - 在第一轮最佳参数周围 ±10% 范围内网格搜索

**参数安全范围**:
  角度环: Kp ∈ [1.5, 5.0], Ki ∈ [0.01, 0.3], Kd ∈ [1.0, 5.0]
  速度环: Kp ∈ [0.5, 3.0], Ki ∈ [0.01, 0.1], Kd = 0
  转向环: Kp ∈ [2.0, 8.0], Ki ∈ [1.0, 5.0], Kd = 0

━━━ 5. 数据解析 ━━━

遥测帧每 50ms 发送一次。由于 BLE 20字节分片，需要按行
重组数据（以 \n 分割）。用正则表达式匹配:

D,seq=(\d+),asp=([-\d.]+),apv=([-\d.]+),au=([-\d.]+),
spv=([-\d.]+),su=([-\d.]+),tpv=([-\d.]+),tu=([-\d.]+),t=(\d+)

━━━ 6. 输出要求 ━━━

调参完成后输出:
1. 三个环的最佳 PID 参数
2. 最终验证的评分
3. 所有候选参数的排名表
4. 建议: 是否需要进一步调优

━━━ 7. 代码模板 ━━━

请用 Python 编写脚本。必须包含:
- asyncio + bleak 异步 BLE 通信
- 20字节分片发送
- XOR 校验和
- 数据采集与统计分析
- 安全回退机制
- 进度打印

默认起始参数 (当前最佳):
  角度环: Kp=2.7, Ki=0.1, Kd=2.5
  速度环: Kp=1.5, Ki=0.03, Kd=0.0
  转向环: Kp=4.0, Ki=3.0, Kd=0.0
```

---

## 使用方法

1. 将上述提示词粘贴给任意 AI
2. AI 会生成一个 Python 调参脚本
3. 运行脚本前确保:
   - 平衡车已开机，HC-04BLE 在广播
   - Windows 蓝牙设置中**没有配对** HC-04BLE（避免缓存 bug）
   - 已安装 `pip install bleak numpy`
4. 运行脚本，AI 自动完成调参

## 适配其他项目

修改提示词中的以下部分即可适配:
- **通信方式**: 替换 BLE 为串口/TCP/CAN 等
- **PID 环数量**: 增删环的描述
- **命令协议**: 修改帧格式
- **安全范围**: 根据硬件调整参数上下限
- **评分函数**: 根据控制目标调整权重

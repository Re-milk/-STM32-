# MAVLink 遥测链路移植说明（阶段 1：与 ANO 并存）

本工程原有通信分三条链路，本次只替换/新增了"上位机遥测与调参"这一条：

| 链路 | 物理层 | 协议 | 本次改动 |
|---|---|---|---|
| 遥控器/上位机 | NRF24L01 2.4G（32B 载荷）+ USART3 TX | 匿名 ANO_DT | **完全保留，未改动** |
| 遥控接收机 | USART3 @420000 + DMA+IDLE | ELRS CRSF | 未改动 |
| 地面站 | **USART1 (PA9/PA10) @57600** | **MAVLink v1（新增）** | 19200/ANO → 57600/MAVLink |

> USART3 的 TX 上原本还打着一份没人接收的 ANO 帧副本（`ANTO_send()` → `USART3_SendByte`），
> 那是 420000bps 的 ELRS 口，接收机一般只接 TX→FC RX，所以这份数据实际上没有去处。
> 本次未删除，避免影响原有行为；确认无用后可去掉 `Task_DataReport` 里的 `ANTO_polling()`。

## 1. 接线

| 信号 | 引脚 | 说明 |
|---|---|---|
| FC TX → 数传/模块 RX | PA9 | 3.3V TTL |
| FC RX ← 数传/模块 TX | PA10 | 3.3V TTL |
| GND | GND | 必须共地 |

- 波特率 **57600 8N1**，无流控（在 `USER/INIT.c` 的 `ANO_Uart1_Init(57600)` 改）。
- sysid = 1，compid = 1（MAV_COMP_ID_AUTOPILOT1）。
- 若用 3DR/SiK 数传，把空中速率设到 ≥57600 并保持两端一致。

## 2. 地面站连接（QGroundControl）

1. 串口选对应 COM 口，波特率 57600，点连接。
2. 1 秒内应看到心跳、姿态、电压；若连上但**没有数据**，检查是否被别的上位机占用该串口。
3. 参数页会列出 36 个 PID 参数，可直接改、可刷新。

Mission Planner 同理（初始设置 → 可选硬件 → 串口，57600）。

## 3. 支持的消息

### 上行（飞控 → 地面站）

| 消息 | ID | 默认速率 | 数据来源 |
|---|---|---|---|
| HEARTBEAT | 0 | 1 Hz | `Command.FlightMode`、`ALL_flag.unlock` |
| SYS_STATUS | 1 | 1 Hz | `voltage`(mV)、各传感器健康位 |
| ATTITUDE | 30 | 20 Hz | `Angle.roll/pitch/yaw`、`pidRateX/Y/Z.measured` |
| VFR_HUD | 74 | 5 Hz | `bara_height`(cm→m)、油门百分比、航向 |
| RC_CHANNELS | 65 | 5 Hz | `Remote.*` 10 通道 + 信号强度 |
| RAW_IMU | 27 | 10 Hz | `MPU6050`、`AK8975` |
| PARAM_VALUE | 22 | 按需 | 参数表 |
| COMMAND_ACK | 77 | 按需 | 命令应答 |
| STATUSTEXT | 253 | 按需 | 解锁/校准/告警提示 |

速率可由地面站用 `MAV_CMD_SET_MESSAGE_INTERVAL(511)` 或 `REQUEST_DATA_STREAM(66)` 改写，
下限被钳在 20 ms（50 Hz），防止塞爆 57600。

### 下行（地面站 → 飞控）

| 消息 | ID | 处理 |
|---|---|---|
| HEARTBEAT | 0 | 记录地面站在线（5 s 超时） |
| PARAM_REQUEST_LIST | 21 | 每条 4 ms 回一条 PARAM_VALUE，共 36 条 |
| PARAM_REQUEST_READ | 20 | 按名字或索引回 PARAM_VALUE |
| PARAM_SET | 23 | 改名并回读（NaN 拒绝，值钳在 ±10000） |
| COMMAND_LONG | 76 | 见下表 |
| REQUEST_DATA_STREAM | 66 | 映射到 6 条遥测流 |
| SET_MODE | 11 | **拒绝**并回 STATUSTEXT |

### COMMAND_LONG 支持矩阵

| 命令 | ID | 行为 |
|---|---|---|
| `MAV_CMD_COMPONENT_ARM_DISARM` | 400 | 解锁前要求油门 < 1100；解锁时置 `baro_start=1`（与遥控解锁流程一致） |
| `MAV_CMD_PREFLIGHT_CALIBRATION` | 241 | 仅未解锁时允许；gyro/accel 调 `MpuGetOffset()`（约 3.5 s，期间请保持静止）；baro 置 `baro_start=1` |
| `MAV_CMD_SET_MESSAGE_INTERVAL` | 511 | 改流速率；不支持的消息回 UNSUPPORTED |
| `MAV_CMD_REQUEST_MESSAGE` | 512 | 单发一条；不支持回 UNSUPPORTED |
| `MAV_CMD_DO_SET_MODE` | 176 | **DENIED**：`Mode_Controler()` 每 4 ms 会根据 `Remote.AUX1` 重写 `Command.FlightMode`，地面站改模式会被立刻覆盖，所以明确拒绝而不是假装成功 |
| 其它 | — | UNSUPPORTED |

> 想让地面站也能切模式，需要改 `HAL/control.c` 的 `Mode_Controler()`：加一个
> "地面站模式保持"标志和超时回退。这属于飞行逻辑改动，本次按阶段 1 的约定没有碰。

## 4. 参数表（36 项）

```
RATE_X_KP/KI/KD      RATE_Y_KP/KI/KD      RATE_Z_KP/KI/KD
ANGLE_ROLL_KP/KI/KD  ANGLE_PIT_KP/KI/KD   ANGLE_YAW_KP/KI/KD
ALT_RATE_KP/KI/KD    ALT_HOLD_KP/KI/KD
FLOW_SPD_X_KP/KI/KD  FLOW_SPD_Y_KP/KI/KD
FLOW_POS_X_KP/KI/KD  FLOW_POS_Y_KP/KI/KD
```

映射关系与原匿名上位机 PID 组一一对应：

| 参数前缀 | 结构体 | 原 ANO 帧 |
|---|---|---|
| `RATE_X/Y/Z_*` | `pidRateX/Y/Z` | `0x10` PID1 |
| `ANGLE_ROLL/PIT/YAW_*` | `pidRoll/pidPitch/pidYaw` | `0x11` PID2 |
| `ALT_RATE/ALT_HOLD_*` | `pidHeightRate/pidHeightHigh` | `0x12` PID3 |
| `FLOW_SPD/FLOW_POS_*` | `Flow_SpeedPid_x/y`、`Flow_PosPid_x/y` | `0x13` PID4 |

**参数只存在 RAM**，复位后由 `pid_param_Init()` 恢复默认（工程未编译 `eeprom.c`，本来就没有持久化）。
需要掉电保存要另外接 EEPROM 或 Flash 页。

## 5. 资源开销（Keil V5.06u7 实测）

| | 改前 | 改后 | 变化 |
|---|---|---|---|
| ROM | 34792 B (34.0 KB) | 40956 B (40.0 KB) | +6.0 KB |
| RAM (RW+ZI) | 16552 B (16.16 KB) | 16832 B (16.44 KB) | +280 B |
| `Task_Communication` 最大栈深 | — | 364 B（任务栈 1024 B） | 安全 |

ROM 余量 24.5 KB，RAM 余量 3.6 KB（其中 FreeRTOS heap 空闲约 3.9 KB）。

## 6. 为什么不用官方 mavlink C 库

- C8T6 只有 20 KB RAM，map 实测已用 16.2 KB。官方库的 `mavlink_message_t`(payload 255)
  + 收发缓冲 + status 约 0.9 KB，且 `MAVLINK_HELPER = static inline`、头文件里还有 `static` 缓冲，
  多包含一次就多一份。
- ARMCC 5 不支持 `__attribute__((packed))`，`MAVPACKED` 要改 `__packed`。
- 手写版本只实现 15 条消息，RAM 约 0.5 KB，全部集中在 `MAVLINK/` 两个文件里。

## 7. 改动清单

```
新增  MAVLINK/mavlink_port.h     协议常量、载荷读写宏、对外接口
新增  MAVLINK/mavlink_port.c     组帧/CRC/解析/流调度/6 条上行消息
新增  MAVLINK/mavlink_app.c      参数表 + PARAM_* / COMMAND_LONG 处理
重写  CONFIG/Uart1.c/.h          USART1 改为 57600 + 收发环形缓冲 + TXE 中断
改    USER/INIT.c                ANO_Uart1_Init(57600) + MAVLink_Init()
改    USER/main.c                Task_Communication 里加 MAVLink_Update()
改    FreeRTOS_FlightCtrl.uvprojx  新增 MAVLINK 分组和 .\MAVLINK 包含路径
```

**未改动**：`HAL/ANO_DT.c`、`HAL/ANO_Data_Transfer.c`、`HAL/WIFI_UFO.c`、`HAL/control.c`、
`HAL/remote.c`、`ELRS/*`、`HARDWARE/*`。

## 8. 加新消息时必读（CRC_EXTRA）

MAVLink 的 `CRC_EXTRA` 是对"消息名 + **线序**字段的类型/名字/数组长度"做 X25 哈希的低 8 位。
两个坑：

1. **线序不是 XML 书写顺序**，而是按字段宽度降序稳定排序（等宽字段保持 XML 顺序）。
   本模块所有载荷偏移都按线序推导，并已用已知 `CRC_EXTRA` 反向核对过：
   `0:50 1:124 11:89 20:214 21:159 22:220 23:168 27:144 30:39 65:118 66:148 76:152 77:143 253:83`
2. **MAVLink2 的 `<extensions/>` 字段不参与 CRC_EXTRA**，也不参与长度。
   例如 `RAW_IMU` 的 `id` 就是扩展字段，所以它的基础载荷是 26 字节（`time_usec` + 9×int16），
   不要按 27 字节发。

改 `mav_crc_extra()` 前，请先用消息定义重算一遍再落地，写错的表现是"地面站收到帧但全部丢弃"。

## 9. 台架自检清单（上桨前）

1. 断电，串口只接 PA9/PA10/GND；上电，用串口助手 57600 看是否有 `MAVLINK v1 ready (57600)` 的
   STATUSTEXT 帧（`FE ... FD`），有则说明发送通路上电即工作。
2. QGC 连上，确认心跳、姿态、电压、遥控通道都在刷新。
3. 打杆观察 ATTITUDE 与 RC_CHANNELS 方向是否一致；**若地平线左右反了**，把
   `mav_send_attitude()` 里 `roll` 那行的负号去掉即可（当前沿用匿名上位机的约定：roll/yaw 取反）。
4. 参数页读一次全表，改一个 `RATE_X_KP` 再刷新，确认能回读。
5. 地面站点解锁 → 飞控应 STATUSTEXT 提示 ARMED 且 `ALL_flag.unlock==1`；再点上锁。
6. 油门推高后点解锁，应被拒绝并提示 "ARM denied: throttle not low"。
7. **不要在地面站点校准**，除非飞行器静止且未解锁——`MpuGetOffset()` 内部会等陀螺静止。

## 10. 已知限制

- 带宽：57600 约 5.76 KB/s，全部 6 条流按默认速率约 2 KB/s。地面站把所有流都拉到 50 Hz 会超限，
  超限时 `UART1_WriteBuf()` 整帧丢弃并累加溢出计数（`UART1_TxOverflowCount()` 可查）。
- 无 GPS / 无位置估计：`VFR_HUD.groundspeed` 固定 0，`LOCAL_POSITION_NED` 未实现——
  工程里 `HARDWARE/flow.c` 只有变量定义，光流数据源实际没有实现。
- 符号约定沿用原匿名上位机（roll/yaw 取反），若与实际姿态相反按第 9 节第 3 步改一行。
- 校准命令会阻塞通信任务约 3.5 s（飞控、高度等其他任务不受影响，它们是独立任务）。

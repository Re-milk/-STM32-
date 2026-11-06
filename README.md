# -STM32-
项目是包含飞控、遥控器双终端的小型无人机，实现无线遥控、自主姿态稳定的定点定高飞行。
遥控器端负责摇杆采集、无线数据发送与 OLED 状态显示。两端通过 NRF24L01 2.4GHz 无线模块通信，系统基于 FreeRTOS实时操作系统进行多任务调度。
 ---
  系统架构

  ┌─────────────┬────────────────────────────────┬────────────────────────────────┐
  │             │    飞行控制器（FlightCtrl）    │       遥控器（Remoter）        │
  ├─────────────┼────────────────────────────────┼────────────────────────────────┤
  │ MCU         │ STM32F103C8 (Cortex-M3, 72MHz) │ STM32F103CB (Cortex-M3, 72MHz) │
  ├─────────────┼────────────────────────────────┼────────────────────────────────┤
  │ Flash / RAM │ 64KB / 20KB                    │ 128KB / 20KB                   │
  ├─────────────┼────────────────────────────────┼────────────────────────────────┤
  │ RTOS        │ FreeRTOS (heap_4 内存管理)     │ FreeRTOS (heap_4 内存管理)     │
  ├─────────────┼────────────────────────────────┼────────────────────────────────┤
  │ 无线通信    │ NRF24L01                       │ NRF24L01                       │
  ├─────────────┼────────────────────────────────┼────────────────────────────────┤
  │ 调试接口    │ ANO 上位机数据协议             │ USB HID / 串口                 │
  └─────────────┴────────────────────────────────┴────────────────────────────────┘

  ---
  飞控端核心模块

  1. 传感器驱动
    - MPU6050：六轴 IMU（陀螺仪 + 加速度计），用于姿态感知
    - SPL06：高精度气压计，实现气压定高功能
    - 光流传感器（flow）：提供水平位移估计，辅助定点悬停
  2. 姿态解算与滤波
    - 实现 IMU 姿态融合算法（imu.c）
    - 集成 卡尔曼滤波器（kalman.c）与通用数字滤波器（filter.c）
    - 自定义数学库（myMath.c）支持矩阵运算与三角函数快速计算
  3. 飞行控制算法
    - PID 控制器（pid.c）：实现姿态环与速度环的级联 PID 控制
    - 控制逻辑（control.c）：油门、横滚、俯仰、偏航四通道控制输出
    - 遥控指令解析（remote.c）：解析 NRF24L01 接收的遥控数据包
  4. 外设驱动
    - NRF24L01 无线收发（SPI 驱动）
    - 多路 UART（USART1/2/3）：分别用于调试、WIFI 模块、上位机通信
    - ADC 采集（电池电压检测）
    - I2C（软件模拟）连接传感器
    - SPI 驱动 NRF24L01
    - OLED 显示屏驱动
    - LED 状态指示
  5. 通信协议
    - ANO_Data_Transfer：与匿名科创上位机的自定义通信协议，支持实时数据回传与参数调参
    - WIFI_UFO：支持 WIFI 模块扩展，实现手机端控制
    - ELRS（ExpressLRS）：集成开源长距离遥控协议支持

  ---
  遥控器端核心模块

  1. 摇杆采集（ANO_Stick.c）：ADC 采集多通道摇杆模拟量，映射为飞行指令
  2. USB HID（ANO_Drv_Hid.c）：实现 USB 人机接口设备功能，可作为 PC 端模拟器手柄
  3. NRF24L01 无线发送：将摇杆数据打包发送至飞控端
  4. OLED 显示（oled.c + show.c）：实时显示遥控状态、电量、连接信息等
  5. 按键驱动（Key.c）：多按键输入，支持模式切换与参数设置
  6. Flash 存储（ANO_Drv_Flash.c）：参数掉电保存
  7. 上位机通信（ANO_DT.c）：通过串口与 PC 端上位机交互，支持参数在线调节

  ---
  技术要点

  - FreeRTOS 多任务架构：飞控端与遥控器端均采用 FreeRTOS
  实时调度，合理划分传感器采集、控制计算、通信收发、显示刷新等独立任务，确保实时性
  - 级联 PID 控制：外环姿态角 / 内环角速率双环控制结构，兼顾响应速度与稳定性
  - 卡尔曼滤波：对 IMU 数据进行最优估计，抑制传感器噪声，提升姿态精度
  - 气压定高：基于 SPL06 气压计实现高度保持，融合加速度计数据提升动态响应
  - NRF24L01 无线协议：自定义数据帧格式，实现低延迟、高可靠遥控链路
  - USB HID 协议：遥控器可直接作为 PC 飞行模拟器手柄，一机多用
  - 模块化软件设计：硬件驱动层（Driver/HARDWARE）、算法层（MATH）、应用层（HAL/USER）分层解耦，便于维护与移植

  ---
  开发环境与工具

  - IDE：Keil MDK-ARM (V5.06)
  - 编译器：ARM Compiler V5
  - 硬件平台：STM32F103 系列（Cortex-M3）
  - PCB 设计：嘉立创 EDA
  - 调试工具：ST-Link、匿名科创上位机

  ---
  仓库目录结构

  -STM32-/
  ├── 无人机FreeRTOS版/          飞控端 Keil 工程（FreeRTOS_FlightCtrl.uvprojx）
  │   ├── USER/                 main、FreeRTOS 任务调度、系统初始化、全局数据定义
  │   ├── CONFIG/               外设初始化（GPIO / ADC / TIM / USART1-3 / SPI / 软件 I2C / EEPROM）
  │   ├── HARDWARE/             mpu6050、spl06 气压计、nrf24l01、光流、LED 驱动
  │   ├── HAL/                  control 控制输出、remote 遥控解析、ANO_Data_Transfer、WIFI_UFO、ANO_DT
  │   ├── MATH/                 imu 姿态解算、kalman、filter、pid、myMath
  │   ├── MAVLINK/              轻量 MAVLink 端口与应用层
  │   ├── FreeRTOS/             内核源码 + RVDS/ARM_CM3 移植层 + heap_4
  │   ├── StartUp/              CMSIS、启动文件、stm32f10x_it、中断向量
  │   └── Lib/                  STM32F10x 标准外设库（inc / src）
  └── 遥控器freeRTOS版/          遥控器 Keil 工程（ANO_Remoter.uvprojx）
      ├── ANO_Remoter/          main、摇杆采集（ANO_Stick）、参数（ANO_Param）、上位机协议（ANO_DT）
      ├── Driver/               板级驱动（ADC / SPI / I2C / UART / Flash / HID / LED / OLED / Key / MPU6050）
      ├── USB_library/ USB_hid/ STM32 USB 全速设备库 + HID 实现（可作 PC 模拟器手柄）
      ├── STM32_LIB/            CMSIS + 标准外设库
      └── FreeRTOS/             内核源码 + Cortex-M3 移植层 + heap_4

  ---
  编译与烧录

  1. 用 Keil MDK-ARM V5（ARM Compiler V5）分别打开「无人机FreeRTOS版\FreeRTOS_FlightCtrl.uvprojx」与「遥控器freeRTOS版\ANO_Remoter.uvprojx」
  2. 在 Options for Target 中选择 ST-Link Debugger（SWD），编译后直接下载
  3. 工程依赖的 STM32F10x 标准外设库与 FreeRTOS 内核均已随仓库提供，无需额外安装
  4. 构建产物（Output/、build/、List/、*.o、*.crf、*.axf、*.hex、*.map 等）均可重新生成，已由 .gitignore 排除，不入库

  ---
  说明

  - 仓库只保留源码、工程配置（.uvprojx / .uvoptx）、启动文件与说明文档，不包含编译中间文件
  - 无线链路参数、通道映射与 PID 参数可在源码对应宏定义中修改
  - 遥控器端通过 USB HID 可在 PC 上识别为游戏手柄，便于在飞行模拟器中练习

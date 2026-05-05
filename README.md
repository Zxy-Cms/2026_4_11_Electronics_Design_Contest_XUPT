# 2026_4_11_Electronics_Design_Contest_XUPT
2026西安邮电大学电赛校赛（改编自2017电赛B题滚球控制系统）

## 项目概述

本项目基于 STM32F407 平台实现滚球/平衡球控制系统。系统通过视觉模块获取小球当前位置，通过串口屏选择题目模式和目标点，由 STM32F407 执行任务状态机、视觉数据解析和双轴 PID 闭环控制，最终输出 PWM 控制双轴舵机改变平衡板姿态，使小球按指定目标或路径运动。

## 工程目录

2026_4_11_Electronics_Design_Contest_XUPT
├── Core                    # STM32CubeMX 生成的主程序、外设初始化和中断入口
├── Drivers                 # STM32 HAL/CMSIS 驱动
├── Middlewares             # USB 等中间件
├── USB_DEVICE              # USB CDC 设备相关代码
├── User
│   ├── Algorithm           # PID 算法
│   ├── Bsp                 # UART、PWM、舵机、USB 等底层封装
│   ├── Control             # 视觉控制、题目状态机、单轴控制和调试任务
│   └── Devices             # 串口屏、视觉串口、舵机、控制节拍等设备抽象
├── MDK-ARM                 # Keil MDK 工程文件
└── README.md

## 核心模块说明

- `Core/Src/main.c`：完成 HAL 初始化、外设初始化，并在主循环中调用 `VisionControl_Task()` 和调试任务。
- `User/Control/vision_control.*`：顶层控制入口，负责视觉输入消费、题目状态机更新、控制节拍处理和双轴控制调度。
- `User/Control/vision_frame.*`：解析视觉模块发送的数据帧，获取小球坐标和视觉目标区域。
- `User/Control/plate_task_fsm.*`：管理题目模式、目标点序列、到达判定、停留计时和复位调平。
- `User/Control/vision_axis.*`：单轴控制模块，采用位置环和速度环双环 PID，输出舵机 PWM 脉宽。
- `User/Algorithm/pid.*`：通用 PID 算法实现。
- `User/Devices/serial_screen_device.*`：串口屏命令接收与显示数据发送。
- `User/Devices/control_tick_device.*`：基于 TIM11 产生固定周期控制节拍。
- `User/Devices/Servo_device.*`：舵机初始化、角度控制和 PWM 脉宽输出。

## 系统工作流程

1. 上电后 STM32F407 初始化 GPIO、USART、TIM、USB 和 PWM 等外设。
2. 串口屏通过 USART1 下发题目选择、目标点输入或复位调平命令。
3. 视觉模块通过 UART4 持续发送小球坐标数据帧。
4. `vision_frame` 解析视觉数据，`plate_task_fsm` 根据当前题目生成目标点。
5. TIM11 周期中断产生控制节拍，`vision_control` 在主循环中消费节拍并执行控制。
6. X/Y 两个方向分别执行位置环和速度环 PID 运算。
7. 控制结果转换为舵机 PWM 脉宽，驱动平衡板倾斜。
8. 小球位置变化后再次被视觉模块采集，形成闭环控制。

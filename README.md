# Gyroscope-Tracking-Car
24年电赛陀螺仪循迹小车原题项目代码

## 项目简介

这是一个基于 STM32F103C8 微控制器的循迹小车项目，结合 JY901S 陀螺仪和 K230 视觉模块，实现多种行驶模式。

## 硬件配置

| 模块 | 引脚占用 | 功能说明 |
|------|----------|----------|
| STM32F103C8 | 主控芯片 | - |
| 电机驱动 | PA2, PA3 (PWM), PA4-PA7 (IN1-IN4) | 左右轮电机控制 |
| 定时器1 | TIM1 | 替代 Delay 用于按键检测、信息获取 |
| 定时器3 | TIM3 | 用于 PID 时间测量和定时任务 |
| OLED 显示屏 | PB8, PB9 (软件I2C) | 显示参数和状态信息 |
| 蜂鸣器 | PB12 | 到达提示和报警 |
| 按键 | PA0, PA1, PA12 (键码1,2,4) | 操作控制 |
| JY901S 陀螺仪 | PB6, PB7 (串口) | 获取姿态角度（航向角 Yaw |
| K230 视觉模块 | PB10, PB11 (串口) | 视觉循迹识别 |

## 功能特性

1. **多种行驶模式：
   - 模式1：陀螺仪直线行驶 (A→B)
   - 模式2：0字形行驶 (A→B→C→D→A)
   - 模式3：8字形行驶 (A→C→B→D→A)

2. **PID 控制：
   - 直线行驶 PID 控制
   - 转弯 PID 控制
   - 支持实时调参功能

3. **OLED 显示：
   - 5个显示界面
   - 实时显示陀螺仪角度
   - 显示 PID 参数
   - 显示电机速度

4. **按键操作：
   - 按键1：切换 OLED 显示界面
   - 按键2：启停控制
   - 按键3：进入调参或模式选择

## 项目结构

```
Gyroscope-Tracking-Car/
├── DebugConfig/      # 调试配置
├── Hardware/         # 硬件驱动
│   ├── Buzzer.c/h
│   ├── JY901S.c/h
│   ├── Key.c/h
│   ├── Motor.c/h
│   └── OLED.c/h
├── Library/         # STM32 标准外设库
├── Listings/        # 编译列表文件
├── Objects/         # 编译输出
├── Start/          # 启动文件和系统文件
├── System/         # 系统模块
│   ├── Delay.c/h
│   ├── PID.c/h
│   ├── PWM.c/h
│   ├── Serial.c/h
│   └── Timer.c/h
└── User/           # 用户代码
    ├── main.c
    ├── stm32f10x_conf.h
    └── stm32f10x_it.c/h
```

## 使用说明

1. 使用 Keil MDK 打开 Project.uvprojx 工程文件
2. 编译项目并烧录到 STM32F103C8
3. 通过按键和 OLED 界面进行操作和调参
4. 配合 K230 视觉模块进行循迹行驶


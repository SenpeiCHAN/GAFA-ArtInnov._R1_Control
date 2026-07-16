# ArtInnov._R1_control

广州美术学院 **ArtInoov.艺创队** 2026 ROBOCON「武林探秘」R1 控制工程。

本仓库用于保存 R1 机器人控制程序，包含 STM32CubeMX 工程配置、Keil MDK-ARM 工程文件、底盘控制、CAN 通信、串口通信、SBUS 遥控输入、电机与红外相关控制模块。

## Project Info

- Team: 广州美术学院 ArtInoov.艺创队
- Season: 2026 ROBOCON
- Theme: 武林探秘
- Robot: R1
- Purpose: R1 控制
- MCU: STM32F427IIHx
- Toolchain: STM32CubeMX / Keil MDK-ARM

## Structure

```text
.
├── Core/                  # CubeMX 生成与用户主控代码
│   ├── Inc/               # 外设与系统头文件
│   └── Src/               # main.cpp、CAN、GPIO、DMA、USART 等源码
├── Drivers/               # CMSIS 与 STM32F4 HAL 驱动
├── MDK-ARM/               # Keil 工程与用户控制模块
│   ├── Robot_Control_R1.uvprojx
│   └── User/
│       ├── sbus/          # SBUS 接收与遥控控制
│       └── wheel_chassis/ # 底盘、电机、CAN、UART、PID、红外等模块
└── Robot_Control_R1.ioc   # STM32CubeMX 工程配置
```

## Development

1. 使用 STM32CubeMX 打开 `Robot_Control_R1.ioc` 查看或调整外设配置。
2. 使用 Keil MDK-ARM 打开 `MDK-ARM/Robot_Control_R1.uvprojx` 进行编译、下载和调试。
3. 主要业务逻辑位于 `Core/Src/main.cpp` 与 `MDK-ARM/User/` 下的用户模块。

## Notes

- 当前工程包含原始压缩包中的源文件、工程配置和构建输出。
- 如需清理构建产物，可在后续维护中补充 `.gitignore` 并重新整理提交内容。

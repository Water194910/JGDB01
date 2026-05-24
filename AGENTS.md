# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 项目概述

STM32F407VET6 云台控制系统，包含：
- **X轴**: GM6020 无刷电机（CAN总线控制）
- **Y轴**: HA8-U35-M 舵机（USART2 串口控制）
- **视觉**: maixcam2 目标追踪（USART3 串口接收）

## 构建命令

**Keil MDK**: 打开 `MDK-ARM/hc.uvprojx`，Rebuild (F7)

**CMake + Ninja**:
```bash
cd build/Debug
cmake --build .
```

**清理 clangd 缓存**（解决 IDE 类型报错）:
```bash
rm -rf build/Debug/.cache
```

## 代码架构

### 外设分配

| 外设 | 引脚 | 用途 |
|------|------|------|
| CAN1 | PB8(RX)/PB9(TX) | GM6020 电机控制，1Mbps |
| USART2 | PA2(TX)/PA3(RX) | HA8-U35-M 舵机，115200 |
| USART3 | PB10(TX)/PB11(RX) | maixcam2 视觉数据，115200 |
| USART1 | PA9(TX)/PA10(RX) | 调试串口，115200 |
| TIM2 | - | 1ms 中断，云台控制周期 |

### GM6020 CAN 协议

- **控制报文**: StdId=0x1FF (电压模式), StdId=0x1FE (电流模式)
- **反馈报文**: 0x205~0x208 (电机1~4)
- **反馈数据**: DATA[0-1]角度, DATA[2-3]转速, DATA[4-5]电流, DATA[6]温度
- **电机数据**: `motor_date[0]` 对应电机1 (ID=0x205)

### maixcam2 视觉协议

```
帧头: 0xAA 0xBB
数据: cmd(1B) + dx(2B signed LE) + dy(2B signed LE) + checksum(1B)
cmd: 0=扫描, 1=追踪
dx/dy: 目标偏离图像中心的像素偏移
```

### 关键文件

- `Core/Src/main.c` - 主程序，云台控制逻辑
- `User/gm6020/gm6020_can.c` - GM6020 驱动，PID 控制
- `User/fashion_star_uart_servo/` - HA8-U35-M 舵机驱动
- `User/user_uart/user_uart.c` - 串口中断处理（舵机+maixcam2）

### PID 控制

- **X轴**: 双环 PID（位置环 + 速度环），10ms 周期
- **Y轴**: 舵机直接角度控制
- **死区**: 5 像素，避免追踪抖动

## 调试

串口1 (PA9/PA10, 115200) 输出调试信息，格式示例：
```
X:90.0 Y:45.0 dx:10 dy:-5 cmd:1
```

## 注意事项

1. GM6020 需要 24V 供电
2. CAN 终端电阻：拨码开关第4位拨至 ON 接入 120Ω
3. 修改 PID 参数后需要重新编译烧录
4. clangd 报错多为缓存问题，删除 `.cache` 目录后重启 IDE

# LED Matrix Control System

LED点阵屏控制系统，基于CSKY架构MCU开发。

## 项目概述

- **硬件平台**: CSKY MCU + LED点阵模块
- **通信协议**: UART / SPI
- **控制方式**: 上位机指令控制

## 功能特性

- 点阵显示控制
- 动画效果播放
- 亮度调节
- 多区域独立控制

## 技术栈

- **MCU**: CSKY架构处理器
- **语言**: C
- **IDE**: Keil MDK
- **库**: HGIC通信库

## 项目结构

```
LED点阵屏控制系统/
├── project/        # 主工程代码
├── csky/           # CSKY库文件
│   ├── hgic/       # 通信接口库
│   └── libs/mm/    # 内存管理
└── doc/            # 文档
```

## 编译烧录

```bash
# 使用Keil打开工程
# 选择目标设备
# 编译并下载
```

## 使用说明

1. 连接硬件（UART/SPI）
2. 上电初始化
3. 发送控制指令
4. 显示内容更新

## 相关项目

- [STM32-Projects](https://github.com/1184197761/STM32-Projects) - STM32嵌入式项目

---
*LED控制项目实践*

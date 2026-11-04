# HLJU_RMIC_fire —— 水弹波箱开火控制器

STM32F401RCT6 水弹波箱（发射机构）固件：通过 **USART2** 接收 Jetson 发来的开火命令，
用 **PA12 低电平**驱动 MOS 管发射；带三层安全保护，断流/超时自动停火。

整车链路：

```
笔记本 gimbal_ctl ──UDP──► Jetson gimbal ──40pin UART(115200)──► 本板 STM32F401 ──► PA12 ──► MOS ──► 波箱
```

本仓库只包含 **STM32F401 固件**；Jetson 侧（`~/gimbal`）与笔记本客户端（`~/gimbal_ctl`）
见对应工程，完整使用与排障手册见 [`FIRE_LINK.md`](FIRE_LINK.md)。

---

## 功能特性

- **USART2（PA2/PA3）115200 8N1** 二进制帧协议，CRC-8/ATM 校验
- **PA12 低电平开火**，上电/复位默认高电平（关火）
- **50 Hz 状态反馈**：开火中 / 链路超时 / 超时保护
- **三层安全**：100 ms 看门狗、连续导通 5 s 超时保护、CRC 校验
- 事件日志（`[biu] FIRE ON/OFF`、`LINK OK/LOST`）从同一 USART2 输出

## 硬件

| 项 | 说明 |
|---|---|
| MCU | STM32F401RCT6（84 MHz，HSE 25 MHz） |
| 开火输出 | **PA12** → MOS 栅极（**低电平导通**，高电平关断） |
| 控制串口 | USART2：PA2 = TX，PA3 = RX |
| 调试 | DAP-LINK：SWD + USART2（可与 Jetson 共用同一串口，注意不要并线） |
| 状态灯 | PC13（链路正常 1 Hz 慢闪，断链 5 Hz 快闪） |

### Jetson 侧接线（40pin UART）

```
Jetson 40pin pin 8  (TXD)  →  F401 PA3 (USART2_RX)
Jetson 40pin pin 10 (RXD)  ←  F401 PA2 (USART2_TX)
Jetson 40pin pin 6  (GND)  ↔  GND
```

> ⚠️ 低电平开火：接线时 PA12 若碰到 GND 会误触发一次开火，接线前先拔波箱电池。

## 通信协议

### 命令帧（Jetson → 本板，6 B，100 Hz）

| off | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| 字段 | `0xA5` | `0x5A` | `0x10` | seq | flags | crc8 |

- `flags` bit0 = FIRE 请求（1 = 开火，0 = 停火）
- `crc8` 覆盖 0~4 字节，CRC-8/ATM（poly `0x07`，init `0`）

### 反馈帧（本板 → Jetson，7 B，50 Hz）

| off | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|
| 字段 | `0x5A` | `0xA5` | `0x81` | seq 回显 | state | crc8 | `0x0D` |

`state`：`0x01` = 开火中（PA12 低），`0x02` = 链路超时，`0x04` = 连续导通超时保护已触发

## 安全机制

| 层 | 条件 | 动作 |
|---|---|---|
| 链路看门狗 | 100 ms 收不到合法命令帧 | 强制 PA12 高（关火） |
| 导通保护 | 连续导通超过 5 s | 强制关断并锁存，**松开扳机才解锁** |
| 上电默认 | 复位 / 上电 | PA12 输出高（关火） |
| 帧校验 | 帧头/CRC 错误 | 丢弃，不刷新看门狗 |

## 构建与烧录

```bash
# 构建（需要 arm-none-eabi-gcc + Ninja + CMake）
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --target biu

# 烧录（DAP-LINK，SWD）
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -c "adapter speed 1000" \
        -c "program build/Debug/biu.elf verify reset exit"
```

> 首次构建前若未生成 `build/Debug`，也可以用 `cmake --preset Debug && cmake --build --preset Debug`。

## 仓库结构

```
biu/
├── Core/
│   ├── Inc/            寄存器/外设头文件（main.h、gpio.h、usart.h…）
│   └── Src/
│       ├── main.c      ★ 开火协议/状态机/安全逻辑（USER CODE 区）
│       ├── gpio.c      ★ FIRE(PA12) 默认输出高（关火）
│       ├── usart.c     ★ USART2 接收中断使能
│       ├── stm32f4xx_it.c  ★ USART1/USART2_IRQHandler
│       └── system_stm32f4xx.c
├── Drivers/            STM32F4 HAL 驱动
├── cmake/              CubeMX 生成的构建脚本
├── CMakeLists.txt / CMakePresets.json
├── startup_stm32f401xc.s
├── STM32F401xx_FLASH.ld
├── biu.ioc             CubeMX 工程
├── FIRE_LINK.md        ★ 开火链路使用与排障手册
└── README.md           本文件
```

## 相关文档

- [`FIRE_LINK.md`](FIRE_LINK.md)：协议细则、接线、Jetson/客户端用法、排障（SWD 电平诊断）、实测记录

# 水弹波箱开火链路（F401 ↔ Jetson ↔ 笔记本）使用与排障手册

> 适用：`~/stmproject/biu`（F401 波箱板固件） + `~/gimbal`（Jetson 侧 C++17 工程）
> + `~/gimbal_ctl`（笔记本侧 C++17 客户端）
> 本文档只写「怎么连 / 怎么用 / 出问题怎么查」，协议细节以代码注释为准。

---

## 1. 系统拓扑

```
笔记本(PC)                         Jetson Nano                        STM32F401RCT6
┌──────────────┐   WiFi 或 网线   ┌──────────────┐   40pin UART      ┌────────────┐
│ gimbal_ctl   │ ──── UDP ────►  │ gimbal       │  115200 8N1        │ UART2      │
│ 按住 F 开火   │   40001         │ fire_link    │  pin8/pin10/GND    │ (PA2/PA3)  │
└──────────────┘                  └──────────────┘ ─────────────────► │ PA12 ──► MOS│
                                                      ◄──────────────  │ 低电平导通   │
                                                                      └────────────┘
```

| 角色 | 设备/端口 | 说明 |
|---|---|---|
| 笔记本 ↔ Jetson（有线） | `enp8s0 192.168.2.1` ↔ `eth0 192.168.2.2` | `ssh Jetson` 免密 |
| Jetson ↔ F401 | Jetson 40pin **UART**（`/dev/ttyTHS1`） | pin 8 = TX，pin 10 = RX，pin 6 = GND |
| F401 ↔ MOS | **PA12** | **低电平导通开火，高电平关断** |
| 底盘/云台 | SocketCAN（CANable2，`/dev/ttyACM0`） | 与开火链路互不影响 |
| UDP 控制口 | `40001`（默认） | `gimbal_ctl` → `gimbal` |

> ⚠️ 历史：早先用 Jetson 的 CH340（`/dev/ttyUSB0`）接 F401 UART1（PA9/PA10），
> PA10 始终读不到命令（引脚恒低），换 CH340 模块在笔记本电脑和 Jetson 上都枚举失败
> （`dmesg` 报 `error -71`），最终改为 **Jetson 40pin UART 3.3V 直连 F401 UART2** 并验证通过。

---

## 2. 线协议（Jetson ↔ F401，115200 8N1，CRC-8/ATM）

### 2.1 命令帧（Jetson → F401，6 字节）

| off | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| 字段 | `0xA5` | `0x5A` | `0x10` | seq | flags | crc8 |
| 类型 | u8 | u8 | u8 | u8 | u8 | u8 |

- `flags` bit0 = **FIRE 请求**（1 = 开火，0 = 停火）
- `crc8` = CRC-8/ATM（poly `0x07`，init `0`），覆盖第 0~4 字节
- 发送频率：**100 Hz**（`tx_period_s = 0.010`）

### 2.2 反馈帧（F401 → Jetson，7 字节，50 Hz）

| off | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|
| 字段 | `0x5A` | `0xA5` | `0x81` | seq（回显） | state | crc8 | `0x0D` |
| 类型 | u8 | u8 | u8 | u8 | u8 | u8 | u8 |

- `crc8` 覆盖第 0~4 字节
- `state` 位定义：

| 位 | 名称 | 含义 |
|---|---|---|
| `0x01` | FIRE_ON | 正在开火（PA12 当前为**低**） |
| `0x02` | TIMEOUT | F401 侧 100 ms 未收到合法命令，已强制关火 |
| `0x04` | OVERTIME | 连续导通超过 5 s，已强制关断并锁存（需松开扳机解锁） |

### 2.3 电平极性

| 状态 | PA12 | MOS |
|---|---|---|
| 上电 / 复位 / 断流 / 松开 | **高电平**（默认关火） | 关断 |
| 按住 F 开火 | **低电平** | 导通 |

> ⚠️ 低电平开火意味着：接线时如果 PA12 被碰到 GND，会误触发一次开火。接线前先拔波箱电池。

---

## 3. 接线

### 3.1 Jetson 40pin ↔ F401

```
Jetson 40pin pin 8  (TXD)  →  F401 PA3 (USART2_RX)
Jetson 40pin pin 10 (RXD)  ←  F401 PA2 (USART2_TX)
Jetson 40pin pin 6  (GND)  ↔  F401 GND
```

- 3.3V 电平直连，不需要 USB-TTL 转换芯片
- `jetson` 用户已在 `dialout` 组，可直接打开 `/dev/ttyTHS1`
- ⚠️ **DAP-LINK 的串口线不要并联到 PA2/PA3**：两个 TX 会对顶（一个发低、一个发高互相短路）。
  DAP-LINK 只用来 SWD 烧录/调试即可

### 3.2 调试日志

F401 的 USART2 就是开火串口。固件会在状态变化时往同一串口打 ASCII 日志：

```
[biu] ready: UART2 115200 8N1, PA12=FIRE, FB 50Hz
[biu] LINK OK / LINK LOST
[biu] FIRE ON / FIRE OFF
[biu] OVERTIME -> fire OFF (release trigger to reset)
```

Jetson 侧按帧头 `5A A5 81` + CRC 同步，ASCII 日志会被自动跳过，不影响解析。

---

## 4. 代码位置与修改点

| 内容 | 路径 |
|---|---|
| F401 协议/状态机/安全逻辑 | `~/stmproject/biu/Core/Src/main.c`（USER CODE 区） |
| F401 FIRE 引脚默认电平 | `~/stmproject/biu/Core/Src/gpio.c`（初始输出高 = 关火） |
| F401 USART2 接收中断使能 | `~/stmproject/biu/Core/Src/usart.c`（`USART2_MspInit`） |
| F401 USART2_IRQHandler | `~/stmproject/biu/Core/Src/stm32f4xx_it.c` |
| Jetson 开火链路实现 | `~/gimbal/include/gimbal/link/fire_link.hpp`（`UartFireLink`，header-only） |
| Jetson 主程序接线 | `~/gimbal/src/main.cpp`（`--fire-dev` / `--fire-baud` / `--no-fire`） |
| 笔记本客户端按键 | `~/gimbal_ctl/src/main.cpp`（`F` = 按住开火，`j` = 底盘使能） |
| Jetson 台架测试脚本 | `/tmp/fire_test.py`、`/tmp/fire_safety.py`（临时文件，可随时重建） |

### 4.1 关键常量

| 常量 | 位置 | 值 |
|---|---|---|
| `FIRE_LINK_TIMEOUT_MS` | F401 `main.c` | 100（看门狗） |
| `FIRE_FB_PERIOD_MS` | F401 `main.c` | 20（反馈 50 Hz） |
| `FIRE_MAX_ON_MS` | F401 `main.c` | 5000（连续导通上限） |
| `tx_period_s` | Jetson `fire_link.hpp` | 0.010（命令 100 Hz） |
| `timeout_s` | Jetson `fire_link.hpp` | 0.10（反馈超时判定） |

---

## 5. 使用

### 5.1 Jetson

```bash
cd ~/gimbal
./build/gimbal --can can0          # 云台/底盘走 CAN；开火串口默认 /dev/ttyTHS1
```

- 启动日志会打印 `开火    : /dev/ttyTHS1 @115200（F401 USART2 -> PA12 -> MOS）`
- 串口打不开会打印警告并每 0.5 s 自动重试，云台/底盘不受影响

**开火相关参数：**

```
--fire-dev <路径>    开火串口（默认 /dev/ttyTHS1）
--fire-baud <n>      波特率（默认 115200）
--no-fire            不开火链路
```

**本地 stdin 命令：** `f 1` / `f 0` 手动开火 / 停火（调试用）

### 5.2 笔记本

```bash
cd ~/gimbal_ctl
./build/gimbal_ctl --host 192.168.2.2
```

| 按键 | 作用 |
|---|---|
| **按住 `F`** | **发射（按住连发，松开停）** |
| `j` | 底盘使能开关（原 `f` 键挪到 `j`） |
| `g` | 云台使能开关 |
| `x` | 急停（云台+底盘，同时停火） |
| 其余 | 同原客户端：方向键云台、WASD 底盘、空格停手等 |

### 5.3 台架单测（不动云台/底盘）

```bash
# Jetson 上，先 pkill -x gimbal 独占串口
python3 /tmp/fire_test.py /dev/ttyTHS1
```

期望输出：

```
idle:        seq=0   state=0x02        ← 没收到命令
trigger ON : seq=16  state=0x01        ← 开火：PA12 低
trigger OFF: seq=9   state=0x00        ← 停火：PA12 高
```

---

## 6. 安全机制（三层，逐层兜底）

| 层 | 触发条件 | 动作 |
|---|---|---|
| 客户端 | 没按住 F | 命令帧 flags 不带 FIRE 位 |
| Jetson | UDP 扳机位超过 **0.3 s** 未刷新 | 发停火帧 |
| Jetson | 进程退出 | 显式发停火帧（`setFire(false)` + 轮询 0.1 s） |
| Jetson | 开火串口断线 | 每 0.5 s 自动重连，重连成功先按停火处理 |
| F401 | **100 ms** 收不到合法命令帧 | 强制 PA12 高（关火） |
| F401 | 连续导通超过 **5 s** | 强制关断并锁存 OVERTIME，松开扳机才解锁 |
| F401 | 上电 / 复位 | PA12 默认高（关火） |
| F401 | 帧 CRC 错 / 帧头错 | 丢弃，不计入看门狗刷新 |

---

## 7. 排障

### 7.1 串口打不开 / 被占用

```bash
fuser -v /dev/ttyTHS1        # 看谁占着
pkill -x gimbal              # 停掉 gimbal（别用 pkill -f build/gimbal，会误杀自己的 shell）
```

Jetson 上 `gimbal` 运行时会独占 `/dev/ttyTHS1`，再跑测试脚本会打不开。

### 7.2 命令发不过去（反馈正常、开火没反应）

先确认 Jetson 真的在发、F401 真的收到。**最可靠的方法是直接读 STM32 内部状态**：

```bash
# 读 g_fire 结构体（地址见 arm-none-eabi-nm build/Debug/biu.elf | grep g_fire）
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 1000" -c "init" -c "halt" \
  -c "mdw 0x2000002c 9" -c "reset run" -c "shutdown"
```

| 偏移 | 字段 |
|---|---|
| +8 | last_rx_ms |
| +12 | rx_frames（累计合法命令数） |
| +16 | bad_frames |
| +20 | last_seq |
| +21 | fire_req |
| +22 | fire_on |

- `rx_frames` 不涨 → 命令没到 F401（查 TX 线 / 引脚）
- `rx_frames` 涨但 `fire_req=0` → 收到的是停火帧（查客户端/Jetson 侧）

**电平诊断**（把引脚配成输入再读 IDR）：

```bash
# PA3 配成输入（清 MODER bits 7:6），读 IDR bit3
openocd ... -c "mww 0x40020000 0xA928A820" -c "mdw 0x40020010 1"
```

- 上拉/下拉读数都恒 0 → 引脚被外部拉低，TX 没接对
- 发着数据时读数不跳变 → 线上根本没有数据

**电平极性验证**：

```bash
# 读 GPIOA ODR，bit12 = PA12 输出
openocd ... -c "init" -c "halt" -c "mdw 0x40020014 1" -c "reset run" -c "shutdown"
```

- 关火时应为 `0x...1000`（bit12=1）
- 开火时应为 bit12=0

### 7.3 USB-TTL 枚举失败（`error -71`）

`dmesg` 里出现 `device not accepting address ... error -71`：

```bash
sudo dmesg | tail -20
```

基本都是 USB 线/口/模块硬件问题：换线、换口、别过 hub，或者直接改用 40pin UART。

### 7.4 gimbal 启动失败（打不开 can0）

开火链路本身不依赖 CAN，但 `gimbal` 启动要先打开云台总线：

```bash
sudo slcand -o -c -s1 /dev/ttyACM0 can0   # 速率档位按实际配置
sudo ip link set can0 up
ip -br link | grep can                    # 应看到 can0 UP
```

### 7.5 反馈帧率不对 / 偶发丢帧

- 反馈固定 50 Hz（20 ms 一帧），命令 100 Hz（10 ms 一帧）
- 日志打印是阻塞发送（几毫秒），只在状态变化时触发，不影响正常节奏

---

## 8. 实测记录（本机联调）

| 项 | 结果 |
|---|---|
| 反馈帧率 | 50 Hz（0.6 s 内 41 帧） |
| 命令回显 | 正常（`seq` 非 0） |
| 开火 | `state=0x01`，PA12 低电平（`ODR` bit12=0） |
| 停火 | `state=0x00`，PA12 高电平（`ODR` bit12=1） |
| 按住 6.5 s | 出现 `state=0x04` OVERTIME，PA12 强制关断 |
| 松开后重按 | 能重新开火（锁存解除） |
| 停发命令 0.3 s | `state=0x02` TIMEOUT，PA12 回高 |
| 上电默认 | PA12 高（关火），`state` 含 TIMEOUT 位 |
| UART1（PA9/PA10） | **不通**（PA10 恒低），已废弃 |
| CH340（旧） | 模块损坏（`error -71` / 无法枚举），已废弃 |

---

## 9. 已知限制 / 注意事项

1. **低电平开火**：接线时 PA12 碰 GND 会误触发一次；接线前拔波箱电池。
2. **DAP-LINK 串口线不要并到 PA2/PA3**：会和 Jetson 的 TX 对顶；SWD 调试不受影响。
3. **开火串口与调试日志同口**：日志是 ASCII，解析器按帧同步，不会误判；但别在日志里写 `0xA5/0x5A` 字节。
4. **OVERTIME 锁存**：按住超过 5 s 会强制停火，必须松开再按才能恢复——这是防烧波箱的保护，不是 bug。
5. 客户端 `F` 键是**轮询**（`XQueryKeymap`）实现，X11 自动重复不会干扰；空格（停手）时按 F 不响应。

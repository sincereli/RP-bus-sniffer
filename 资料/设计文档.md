# 2026年“晨电智能杯”嵌入式设计竞赛（高年级组 C题）
# 嵌入式通信协议综合分析仪 —— 嵌入式软件系统设计文档

---

## 1. 文档概述与设计定位

### 1.1 编写目的
本文档旨在定义“嵌入式通信协议综合分析仪”项目的嵌入式固件（Firmware）系统架构、双核协同机制、混合时间戳算法、多协议引擎状态机及数据流转方案。当前版本聚焦于**纯嵌入式软件架构与底层实现逻辑**，硬件引脚映射与外部电平复用控制通过硬件抽象层（BSP/HAL）预留占位接口，待PCB原理图最终定稿后一键填入。

### 1.2 核心设计思想
本系统基于树莓派最新一代高性能微控制器 **RP2350（双核 Cortex-M33 @ 150MHz~240MHz，内置 520KB SRAM 与 3 个可编程 PIO 模块）** 进行构建。软件架构遵循以下核心原则：
1. **双核主从按需唤醒（On-Demand Dual-Core）**：Core 0 作为“系统管家与交互核”常驻运行；Core 1 作为“硬实时协议协处理器”在开机阶段保持休眠，仅在 Core 0 完成上电自检（POST）且用户通过交互确定具体工作模式后，由 Core 0 动态唤醒并加载对应协议引擎。
2. **混合外设调度策略（Hardware Peripheral + PIO）**：充分利用 RP2350 引脚动态复用特性，在**主机主动读写模式（IIC/SPI Master、常规 UART）**下调用片内标准硬件外设；在**从机非侵入式监听抓包（400kHz IIC Sniffer、40MHz SPI Sniffer）**、**UART 自适应波特率过采样测频**及 **SWD（PY32F003 烧录与 RTT 抓包）** 模式下，调用可编程 I/O（PIO）状态机突破传统硬件外设瓶颈。
3. **RTC 秒中断 + 硬件定时器微秒级融合时间戳**：通过外部 RTC 芯片的 1Hz 秒脉冲中断维护临界区日历时间，结合 RP2350 片内 64 位微秒定时器捕获帧触发瞬间偏移，实现零总线延迟的微秒级绝对/相对时间戳打标。

---

## 2. 系统总体软件架构与双核职责划分

### 2.1 软件分层架构图

```text
+-----------------------------------------------------------------------------------+
|                                  应用与交互展现层                                  |
|  [Core 0] OLED 30Hz+ 滚动UI | 五向摇杆状态机 | LED状态指示 | USB 上位机指令解析器  |
+-----------------------------------------------------------------------------------+
|                                  系统服务与控制层                                  |
|  [Core 0] 上电自检(POST) | config.txt解析 | 临界区RTC+Timer时基 | FatFs TF卡管理 |
+-----------------------------------------+-----------------------------------------+
|        核间通信与共享数据管道 (IPC)       |         硬件抽象与复用控制层 (BSP)       |
|  硬件 Mailbox FIFO | Spinlock 硬件自旋锁  |  3.3V/5V 电平切换接口 | 外部复用器控制  |
|  DMA 乒乓缓冲池 (SRAM) + 板载 Flash 缓存  |  RP2350 GPIO 动态功能映射矩阵 (待定引脚)|
+-----------------------------------------+-----------------------------------------+
|                              Core 1 硬实时协议引擎层                               |
|  +-----------------+  +-----------------+  +-----------------+  +---------------+ |
|  | 硬件外设主机模式 |  | PIO 从机监听抓包 |  |   UART 混合引擎  |  |  SWD 调试引擎 | |
|  | • 硬件 IIC 读写 |  | • PIO IIC 400k  |  | • PIO 过采样测频|  | • PY32 烧录器 | |
|  | • 硬件 SPI 读写 |  | • PIO SPI 40MHz |  | • 硬件 UART 收发|  | • RTT 内存抓包| |
|  +-----------------+  +-----------------+  +-----------------+  +---------------+ |
+-----------------------------------------------------------------------------------+
```

### 2.2 双核职责与生命周期管理

| 核心 | 角色定位 | 上电状态 | 核心软件职责 |
| :--- | :--- | :--- | :--- |
| **Core 0** | 系统管理、人机交互与存储通信核 | **上电即启动** | 1. **上电自检（POST）**：检测供电电压、RTC 在线状态、OLED、TF 卡挂载及读取 `config.txt` 默认配置。<br>2. **人机交互（HMI）**：驱动 IIC OLED 屏幕（>30Hz 刷新）、扫描五向摇杆输入、控制烧录/工作状态 LED。<br>3. **全局时基维护**：开启内部 64 位硬件定时器，响应外部 RTC 的 1Hz 秒中断，在受保护的临界区更新日历时间。<br>4. **外部接口与存储**：管理 RP2350 原生 USB PHY（实现 CDC 透传与 WinUSB 上位机通信），负责 FatFs 文件系统将抓包数据按日期目录写入 TF 卡。<br>5. **硬件路由与 Core 1 调度**：控制 3.3V/5V 电平与硬件复用器；根据用户模式选择结果，启动/复位 Core 1 并通过核间信号下发任务。 |
| **Core 1** | 硬实时协议执行与抓包引擎核 | **上电默认关闭**<br>（Wait for Launch） | 1. **按需动态加载**：接收 Core 0 的唤醒信号与模式参数结构体，动态将复用引脚绑定至硬件外设或 PIO 状态机。<br>2. **执行协议任务**：运行主机 IIC/SPI 寄存器读写、PIO 从机 IIC/SPI 监听抓包、UART 自适应波特率测频与抓包、SWD 烧录与 RTT 轮询。<br>3. **打标与 DMA 搬运**：在抓包触发瞬间读取临界区 RTC 基准与硬件定时器微秒值合成时间戳，控制 DMA 将数据高速搬运至缓存区并通知 Core 0 落盘至 TF 卡。 |

---

## 3. 全局混合时间戳机制设计（RTC 秒中断 + 硬件定时器）

为同时满足赛题**“按日期自动生成目录与文件”**（日历时间）与**“上位机 1x/2x/10x 倍速还原通信时序”**（微秒级相对时间差）的要求，软件设计采用**硬件秒中断同步 + 64位微秒计数器插值**方案。

### 3.1 硬件时基协同原理
1. **外部 RTC 秒脉冲中断（1Hz INT/SQW）**：
   * Core 0 在初始化阶段配置外部 RTC 芯片输出 1Hz 秒中断信号接入 RP2350 GPIO。
   * 每当秒中断触发（1 秒到达瞬间），Core 0 在中断服务或高优先级回调中读取一次 RTC 日历时间（或直接将内存中的秒计数 $+1$），**同时锁存此刻 RP2350 内部 64 位硬件定时器（`TIMERAWL/H`，1MHz 计数频率）的微秒计数值 $T_{\text{sync\_us}}$**。
2. **双核共享临界区（Hardware Spinlock 保护）**：
   * เนื่องจาก Core 0（每秒更新一次基准）与 Core 1（高达 40MHz SPI 抓包时随时读取基准）同时访问时间结构体，若无保护会出现“读撕裂（Torn Read）”。
   * 使用 RP2350 硬件自旋锁（`spin_lock_t`，仅需单周期指令开销）或 `seqlock`（顺序锁）保护全局时间基准区，确保 Core 1 在百纳秒内安全获取一致的时间基准。

### 3.2 时间戳合成数学模型与数据结构

设当前秒中断更新的日历时间对应的 Unix 秒级时间戳为 $S_{\text{rtc}}$，秒中断触发瞬间锁存的硬件定时器计数值为 $T_{\text{sync\_us}}$。当 Core 1 捕获到一帧通信数据瞬间，读取硬件定时器当前值为 $T_{\text{now\_us}}$，则该帧数据的高精度亚秒级微秒偏移 $\Delta t_{\mu\text{s}}$ 与完整 64 位微秒时间戳 $TS_{\text{packet}}$ 为：

$$\Delta t_{\mu\text{s}} = T_{\text{now\_us}} - T_{\text{sync\_us}}$$

$$TS_{\text{packet}} = S_{\text{rtc}} \times 10^6 + \Delta t_{\mu\text{s}}$$

```c
// 全局共享时间基准结构体（放置在共享 SRAM 区）
typedef struct {
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    uint64_t epoch_sec;          // 当前秒对应的 Unix 秒数 (S_rtc)
    uint64_t timer_sync_us;      // 该秒中断触发瞬间的硬件定时器微秒值 (T_sync_us)
} shared_timebase_t;

extern volatile shared_timebase_t g_shared_rtc;
extern spin_lock_t *g_rtc_spinlock;

// Core 0: RTC 1秒定时中断更新函数
void core0_rtc_1hz_isr_handler(void) {
    uint64_t now_us = timer_hw->timerawl | ((uint64_t)timer_hw->timerawh << 32);
    uint32_t save = spin_lock_blocking(g_rtc_spinlock);
    
    // 更新秒计数与日历进位（定期与硬件 RTC 寄存器校验对齐）
    rtc_increment_one_second((shared_timebase_t *)&g_shared_rtc);
    g_shared_rtc.timer_sync_us = now_us;
    
    spin_unlock(g_rtc_spinlock, save);
}

// Core 1: 抓包瞬间获取高精度时间戳（内联函数，放置在 SRAM 中执行）
static inline uint64_t __not_in_flash_func(core1_get_precise_timestamp_us)(void) {
    uint64_t now_us = time_us_64(); // 读取 Core 0 打开的硬件定时器
    uint32_t save = spin_lock_blocking(g_rtc_spinlock);
    
    uint64_t base_sec = g_shared_rtc.epoch_sec;
    uint64_t sync_us  = g_shared_rtc.timer_sync_us;
    
    spin_unlock(g_rtc_spinlock, save);
    return (base_sec * 1000000ULL) + (now_us - sync_us);
}
```

---

## 4. Core 0 软件设计：自检、交互、USB 与调度控制

### 4.1 上电自检（POST）与启动流程
系统上电后，仅 Core 0 启动执行 `main()`，Core 1 处于 Reset 挂起态。Core 0 严格按以下顺序执行初始化与自检：
1. **时钟与硬件定时器初始化**：配置系统主频（推荐 `200MHz` 或 `240MHz`，便于 PIO 整除分频），启动 64 位硬件微秒定时器，初始化核间 Spinlock。
2. **板级外设自检**：
   * 初始化内部 IIC 总线，检测 **OLED 屏幕**与 **外部 RTC 芯片**是否应答；读取首次 RTC 时间并开启 1Hz 秒中断。
   * 初始化五向摇杆 GPIO（内部上拉 + 消抖定时器）与状态 LED。
   * 初始化电平转换与外部复用器控制引脚（默认置于 **高阻安全关断态**，防止上电瞬间冲击外部被测板）。
3. **TF 卡挂载与 `config.txt` 自动加载**：
   * 挂载 FatFs 文件系统，读取 SD 卡容量信息（总容量/剩余容量）。
   * 查找根目录下的 `config.txt` 配置文件。若存在，则解析默认总线类型、波特率、工作模式（主机/监听）及接口电平（3.3V/5V），将其加载至全局配置结构 `g_sys_cfg`。
4. **启动原生 USB 协议栈**：
   * 初始化 RP2350 原生 USB 控制器与内置 PHY，基于 TinyUSB 枚举为复合设备（`CDC-ACM 虚拟串口` + `WinUSB 高速数据流通道` + `CMSIS-DAP`）。
5. **按需唤醒 Core 1**：
   * 当 `config.txt` 指定了开机自动运行模式，或用户通过五向摇杆/USB上位机选定了具体模式后，Core 0 先完成**外部电平切换（3.3V/5V）与硬件复用器通道选通**，随后调用 `multicore_launch_core1(core1_main_entry)` 唤醒 Core 1，并通过硬件 Mailbox FIFO 发送模式控制字。

### 4.2 人机交互（OLED + 五向摇杆 + 状态 LED）
* **OLED 显示架构（刷新率 >30Hz）**：
  * 在 Core 0 内存中开辟 `1024 Bytes`（128×64）显存帧缓冲区（FrameBuffer）。
  * 采用 **DMA 异步定时刷屏**：每 `20ms`（50Hz 刷新率）触发一次内部 IIC DMA 传输将 FrameBuffer 刷入 OLED，屏幕无闪烁且不阻塞 Core 0。
  * **界面布局**：顶部状态栏滚动显示 `当前总线 | 速率参数 | RTC实时时间 | TF卡容量百分比`；主显示区支持“模式切换菜单”、“抓包实时统计（包数/速率/状态提示）”以及“离线寄存器直读界面（设备地址 `0x00~0xFF`、寄存器地址 `0x00~0xFF`）”。
* **五向摇杆逻辑**：
  * `上 / 下`：菜单项滚动、寄存器数值加减（长按支持加速递增）。
  * `左 / 右`：切换总线类型（UART / IIC / SPI / SWD）、切换参数位。
  * `中键（按下）`：确认参数配置、向 Core 1 发送启动/停止抓包指令、或在离线主机模式下触发单次寄存器读取。

### 4.3 核间通信协议（Core 0 $\leftrightarrow$ Core 1）
利用 RP2350 硬件自带的深度为 8 的 **32-bit Mailbox FIFO** 传递控制指令与状态反馈：

```c
// Core 0 发送给 Core 1 的控制命令码
typedef enum {
    IPC_CMD_IDLE_STOP      = 0x00, // 停止当前外设/PIO，释放引脚回高阻态
    IPC_CMD_START_IIC_MST  = 0x10, // 启动硬件 IIC 主机读写模式
    IPC_CMD_START_SPI_MST  = 0x11, // 启动硬件 SPI 主机读写模式
    IPC_CMD_START_IIC_SNIF = 0x20, // 启动 PIO IIC 从机监听抓包模式 (400kHz MAX)
    IPC_CMD_START_SPI_SNIF = 0x21, // 启动 PIO SPI 从机监听抓包模式 (40MHz MAX)
    IPC_CMD_START_UART     = 0x30, // 启动 UART 模式 (含自选波特率 / PIO过采样自适应波特率)
    IPC_CMD_START_SWD_RTT  = 0x40, // 启动 SWD RTT 抓包模式
    IPC_CMD_START_SWD_PROG = 0x41, // 启动 PY32F003 烧录模式
    IPC_CMD_EXEC_REG_RW    = 0x50  // 在主机模式下执行一次离线/在线寄存器读写请求
} ipc_cmd_type_t;
```
当需要切换模式时，Core 0 可直接发送 `IPC_CMD_IDLE_STOP` 通知 Core 1 温和卸载当前驱动，或直接调用 `multicore_reset_core1()` 硬件复位 Core 1 后重新载入新模式入口，确保不同协议状态机之间绝无残留状态干扰。

---

## 5. Core 1 软件设计：多协议引擎实现细节

Core 1 的所有关键中断与轮询代码均使用 `__not_in_flash_func()` 宏锁定在 RP2350 的内部专用 SRAM（Scratch X/Y）中运行，实现零指令等待周期。

### 5.1 主机读写模式（硬件 IIC & 硬件 SPI）
当用户通过五向摇杆进入“离线硬件调试（扩展要求 2.2）”或通过 USB 上位机发起可视化总线读写时，Core 0 将复用引脚方向设为**主机输出**，并命令 Core 1 启动硬件主机模式：
* **主机读写 IIC（RP2350 硬件 `i2c0`/`i2c1`）**：
  * 调用 `gpio_set_function(PIN_SDA, GPIO_FUNC_I2C)` 与 `gpio_set_function(PIN_SCL, GPIO_FUNC_I2C)` 将复用引脚切入硬件 IIC 控制器。
  * 支持配置通信速率（100kHz / 400kHz）。
  * **功能实现**：接收 Core 0 传来的目标设备地址 `dev_addr (0x00~0xFF)` 与寄存器地址 `reg_addr (0x00~0xFF)`，调用带超时保护的 `i2c_write_timeout_us()` 发送寄存器地址，随后调用 `i2c_read_timeout_us()` 读取目标寄存器数值，将结果写入共享内存并通知 Core 0 刷新到 OLED 屏幕或通过 USB 回传上位机。同时支持一键遍历 `0x00~0x7F` 进行全总线设备在线扫描。
* **主机读写 SPI（RP2350 硬件 `spi0`/`spi1`）**：
  * 将复用引脚 `SCK`、`MOSI`、`MISO` 映射至硬件 SPI 外设，`CS` 引脚由 GPIO 软件精准控制，支持配置 `CPOL/CPHA` 四种相位模式及任意分频速率。
  * 根据用户在摇杆菜单中选定的寄存器地址，拉低 `CS` 发送读取指令字与 `reg_addr`，同时读取 `MISO` 返回的外设寄存器值，完成后拉高 `CS` 并将结果返回给 Core 0。

### 5.2 从机监听抓包模式（PIO IIC & PIO SPI）
当处于从机监听模式时，Core 0 控制外部复用器将所有信号线置于**纯高阻输入状态**，Core 1 将引脚复用权切换至 `PIO0` 模块：

* **PIO 从机监听 IIC（400kHz 无损非侵入式抓包）**：
  * **状态机原理**：硬件 IIC 从机无法监听全地址且会主动拉低 `SDA` 破坏原总线应答状态。因此由 PIO 状态机以高阻态实时采样 `SDA` 与 `SCL`。
  * **协议字段完整提取**：
    1. 当 `SCL` 为高电平时检测到 `SDA` 下降沿，PIO 识别为 `START`（或 `RESTART`）信号，向 RX FIFO 推入起始标志字并触发 Core 1 记录时间戳；
    2. 在随后的 8 个 `SCL` 上升沿依次移位锁存 `SDA` 电平，组成 8-bit 地址（含 R/W 位）或数据字节；
    3. 在第 9 个 `SCL` 上升沿采样 `SDA` 电平，直接获取总线上真实的应答状态（`0 = ACK`，`1 = NACK`），将 `8-bit Byte + 1-bit ACK` 组合为 16-bit 原始事件推入 FIFO；
    4. 当 `SCL` 为高电平时检测到 `SDA` 上升沿，识别为 `STOP` 信号，触发 Core 1 结算当前 IIC 事务包并启动 DMA 搬运。
* **PIO 从机监听 SPI（40MHz 极限无丢包抓包）**：
  * **状态机原理**：将 RP2350 系统时钟设为 `200MHz~240MHz`（提供 5~6 倍过采样裕量）。将 `MOSI` 与 `MISO` 分配在相邻编号的 GPIO 上。
  * **双线同步锁存**：当检测到片选 `CS` 为有效低电平时，PIO 状态机等待 `SCK` 采样边沿（根据配置的 CPOL/CPHA 选择上升沿或下降沿），在边沿到达瞬间通过单周期指令 `in pins, 2` **同时将 `MOSI` 与 `MISO` 两个引脚的电平锁存进移位寄存器（ISR）**。
  * **DMA 自动搬运**：开启 PIO 硬件 `Autopush`（每满 32 bits 自动推入 RX FIFO），直接触发高速 DMA 通道将交织的 `MOSI/MISO` 原始字节流搬运至缓存区，实现 40MHz 满速通信下 CPU 零中断干预、零丢包。

### 5.3 UART 模式（预设波特率 + PIO 过采样自适应波特率）
针对赛题要求的 **9600~2Mbps 串口收发/抓包** 与 **自适应波特率识别（扩展要求 2.1）**，Core 1 采用两阶段混合驱动：

1. **阶段一：PIO 过采样脉宽测量（当配置为自适应波特率模式时）**
   * Core 1 先将 `UART_RX` 引脚绑定至 `PIO0` 测频状态机。
   * 状态机以系统时钟频率 $f_{\text{sys}}$（如 $200\text{MHz}$，时间分辨率 $5\text{ns}$）对 `RX` 引脚进行高倍过采样，捕获连续 $N$ 个（如 $16\sim32$ 个）高低电平跳变沿之间的脉冲计数值 $C_1, C_2, \dots, C_N$。
   * 取其中有效脉冲（滤除小于 $200\text{ns}$ 的毛刺后）的**最小计数值 $C_{\min}$**，该值即对应串口通信中单个 Bit 的宽度（如起始位或 `0x55` 等单比特翻转位）。
   * 计算实测波特率：
     $$\text{Baud}_{\text{calc}} = \frac{f_{\text{sys}}}{C_{\min}}$$
   * 将 $\text{Baud}_{\text{calc}}$ 与标准波特率表（9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000 等）进行就近吸附匹配（若误差 $< \pm 2.5\%$ 则锁定为标准值），并通过 FIFO 通知 Core 0 在 OLED 屏幕上实时更新识别出的波特率。
2. **阶段二：无缝切回硬件 UART + DMA 高速接收**
   * 波特率确定后（或用户通过菜单直接指定了固定波特率），Core 1 立即调用 `gpio_set_function(PIN_RX, GPIO_FUNC_UART)` 将引脚切回 RP2350 硬件 `uart0`/`uart1` 控制器，并调用 `uart_init()` 设置目标波特率。
   * 开启硬件 UART 的 RX DMA 通道，配合空闲中断（UART Timeout Interrupt）自动切分数据帧、打上微秒级时间戳，确保在最高 **2Mbps** 速率下连续接收不丢包，同时支持与 Core 0 的 USB CDC 端点联动实现**串口实时透传**。

### 5.4 SWD 模式（PY32F003 烧录与 RTT 数据抓包）
独立 SWD 接口固定由 Core 1 的 `PIO1` 模块驱动，实现高速 `SWCLK` 输出与 `SWDIO` 双向切换：
* **PY32F003 系列单片机烧录**：
  * **在线烧录**：Core 0 的 USB CMSIS-DAP 接口接收 PC 端 IDE（Keil / PyOCD）下发的 DAP 指令，通过共享队列交由 Core 1 执行底层 SWD 读写，同时由 Core 0 控制烧录状态指示灯闪烁。
  * **脱机一键烧录**：内置 PY32F003 的 Flash 编程算法（Flash Loader），支持将 TF 卡或板载 Flash 中预存的目标固件直接通过 SWD 写入 PY32F003 内部 Flash 并校验。
* **SWD (RTT) 数据抓包记录**：
  * Core 1 通过 SWD 接口连接目标芯片 AHB-AP，在目标芯片 SRAM 空间（如 `0x20000000` 起）快速检索 `"SEGGER RTT"` 控制块特征签名。
  * 锁定控制块地址后，Core 1 以微秒级周期高速轮询上行通道（Up-Buffer）的写指针 `WrOff` 与读指针 `RdOff`。一旦发现增量日志数据，立即通过 SWD 批量读出、更新 `RdOff`，为数据打上混合时间戳后送入缓存队列（同时供 Core 0 存入 TF 卡并通过 USB 发送至上位机实时显示）。

---

## 6. 数据流转、DMA 缓存与板载 Flash / TF 卡存储设计

### 6.1 关于“DMA 写入板载 Flash 后转存 TF 卡”的工程优化说明
在初步构想中提到*“读取的数据通过 DMA 发送至板载 Flash 后交给 Core 0 写到 TF 卡”*。结合 RP2350 的硬件架构特性，软件在具体落地时做如下**两级精细化设计**：

> **底层硬件机理说明**：
> 板载 QSPI NOR Flash（如 W25Q128）在硬件上仅支持 XIP（就地执行读取），写入前必须按 4KB 扇区擦除（耗时高达 $30\sim50\text{ms}$）且每页 256B 编程耗时约 $0.4\text{ms}$，**无法作为 40MHz SPI（峰值吞吐率高达 5MB/s~10MB/s）的实时 DMA 直接写入目标**，且频繁擦写会迅速耗尽板载 Flash 寿命并阻塞 Core 0 取指。
>
> **工程优化落地方案（SRAM 双缓冲 DMA + 板载 Flash 掉电保险箱 + TF 卡顺序落盘）**：
> 1. **第一级（实时高速 DMA 乒乓缓冲 — 片内独立 SRAM Bank）**：
>    利用 RP2350 高达 **520KB 的片内 SRAM**，专门划出 **256KB（Bank 4~7）** 作为 Core 1 的 **DMA 乒乓环形缓冲池（Ping-Pong Buffer）**。Core 1 抓到的数据由硬件 DMA 通道以零延迟直接搬入 SRAM 缓冲区 A；当缓冲区 A 满（如满 32KB）时，DMA 硬件链式触发（Chain Trigger）自动切换至缓冲区 B，同时通过 FIFO 通知 Core 0 将缓冲区 A 的数据直接写入 TF 卡。
> 2. **第二级（板载 QSPI Flash 的三大关键用途）**：
>    * **掉电紧急抢救缓冲区（Non-Volatile Ring Cache）**：在板载 Flash 尾部开辟 1MB 预擦除循环扇区。若抓包过程中遭遇突发拔电或 TF 卡瞬时卡顿，立即将 SRAM 中尚未写入 TF 卡的尾部残留日志快速刷入板载 Flash，下次上电自检（POST）时由 Core 0 自动恢复追加回 TF 卡，**实现 100% 掉电不丢数据**。
>    * **PY32F003 脱机烧录镜像仓**：将需要烧录给 PY32F003 的固件缓存于板载 Flash 中，脱离 TF 卡亦可完成极速烧录。
>    * **大容量协议字典与字库存储**：存放 OLED 中英文点阵字库及离线寄存器调试设备字典。

### 6.2 统一抓包数据帧格式与 TF 卡目录规范
* **按日期自动生成目录与文件**：
  Core 0 启动抓包记录时，读取临界区 RTC 的年月日信息，自动调用 `f_mkdir("/20261026")` 创建当日目录，并以“总线名 + 时分秒”创建日志文件（如 `/20261026/SPI_143005.log` 或 `.csv`）。
* **防掉电损坏机制**：
  Core 0 在写入 TF 卡时，每写入满 1 个 Buffer（32KB）或每隔 `200ms` 空闲周期，强制调用一次 `f_sync()` 更新 FAT32 文件分配表。若 Core 0 监测到供电电压跌落预警，立即中止新包接收并在毫秒内执行 `f_close()`，确保掉电绝不损坏已有日志文件。

---

## 7. 硬件引脚复用与板级控制接口定义（待硬件定稿填空）

由于 RP2350 的 GPIO 支持高度灵活的功能重映射，且外部硬件将配备 **3.3V/5V 电平转换电路**与**通道复用器（Mux）**，软件在 `bsp_board_pins.h` 中将所有未定引脚抽象为宏定义。待硬件原理图确定后，仅需修改此头文件即可完成适配，无需改动任何业务代码：

```c
// bsp_board_pins.h —— 硬件引脚占位定义表（待 PCB Layout 确定后填入实际 GPIO 编号）
#ifndef __BSP_BOARD_PINS_H__
#define __BSP_BOARD_PINS_H__

/* ==================== 1. Core 0 交互与系统控制引脚 ==================== */
#define PIN_OLED_SDA            (/* TODO: 待填入 GPIO 编号 */)
#define PIN_OLED_SCL            (/* TODO: 待填入 GPIO 编号 */)
#define PIN_RTC_SDA             (/* TODO: 可与 OLED 共用内部 IIC 或独立分配 */)
#define PIN_RTC_SCL             (/* TODO */)
#define PIN_RTC_INT_1HZ         (/* TODO: 外部 RTC 1秒脉冲中断输入引脚 */)

#define PIN_JOYSTICK_UP         (/* TODO: 五向摇杆-上 */)
#define PIN_JOYSTICK_DOWN       (/* TODO: 五向摇杆-下 */)
#define PIN_JOYSTICK_LEFT       (/* TODO: 五向摇杆-左 */)
#define PIN_JOYSTICK_RIGHT      (/* TODO: 五向摇杆-右 */)
#define PIN_JOYSTICK_CENTER     (/* TODO: 五向摇杆-中键确认 */)

#define PIN_LED_STATUS          (/* TODO: 系统/烧录状态指示灯 */)

/* ==================== 2. 电平切换 (3.3V/5V) 与硬件复用器控制 ==================== */
#define PIN_LEVEL_SEL_3V3_5V    (/* TODO: 0=3.3V, 1=5.0V 电平切换控制脚 */)
#define PIN_MUX_CH_SELECT_0     (/* TODO: 外部硬件复用器通道选择脚 S0 */)
#define PIN_MUX_CH_SELECT_1     (/* TODO: 外部硬件复用器通道选择脚 S1 */)
#define PIN_TRANSCEIVER_DIR     (/* TODO: 电平转换芯片输入/输出方向控制脚 */)
#define PIN_TRANSCEIVER_OE      (/* TODO: 电平转换芯片高阻使能控制脚 */)

/* ==================== 3. TF 卡存储接口引脚 ==================== */
#define PIN_TF_CLK              (/* TODO */)
#define PIN_TF_CMD_MOSI         (/* TODO */)
#define PIN_TF_DAT0_MISO        (/* TODO */)
#define PIN_TF_CS_DAT3          (/* TODO */)
#define PIN_TF_DETECT           (/* TODO: TF卡在位检测脚 */)

/* ==================== 4. Core 1 对外多协议复用接口引脚 ==================== */
// 注：建议在硬件设计时将 MUX_IO_0 与 MUX_IO_1 分配在相邻编号的 GPIO 上，以方便 PIO 单指令双线同步采样
#define PIN_MUX_IO_0            (/* TODO: 复用为 UART_TX / IIC_SDA / SPI_MOSI */)
#define PIN_MUX_IO_1            (/* TODO: 复用为 UART_RX / IIC_SCL / SPI_MISO */)
#define PIN_MUX_IO_2            (/* TODO: 复用为 SPI_SCK */)
#define PIN_MUX_IO_3            (/* TODO: 复用为 SPI_CS */)

/* ==================== 5. Core 1 独立 SWD 调试接口引脚 ==================== */
#define PIN_SWD_CLK             (/* TODO: 独立 SWCLK 输出脚 */)
#define PIN_SWD_IO              (/* TODO: 独立 SWDIO 双向脚 */)
#define PIN_SWD_DIR             (/* TODO: SWDIO 电平转换方向控制脚 */)

#endif /* __BSP_BOARD_PINS_H__ */
```

---

## 8. 系统工作状态流转全过程总结

1. **上电启动与自检阶段**：
   `Core 0 启动` $\rightarrow$ `Core 1 保持关闭` $\rightarrow$ `Core 0 初始化硬件定时器与 RTC 秒中断` $\rightarrow$ `自检 OLED / TF 卡并解析 config.txt` $\rightarrow$ `枚举原生 USB 复合设备` $\rightarrow$ `OLED 进入主界面等待用户操作`。
2. **模式选择与 Core 1 唤醒阶段**：
   `用户操作五向摇杆（或 USB 下发指令）选定模式与参数（如：SPI 从机监听，5V 电平）` $\rightarrow$ `Core 0 控制电平切换引脚切至 5V，控制复用器切至监听输入态` $\rightarrow$ `Core 0 启动 Core 1 并通过 FIFO 发送 IPC_CMD_START_SPI_SNIF`。
3. **高速执行与数据落盘阶段**：
   `Core 1 收到指令，将复用引脚映射至 PIO0 并启动 40MHz SPI 监听状态机` $\rightarrow$ `检测到 CS 拉低，Core 1 读取临界区 RTC 基准 + 硬件定时器微秒值生成时间戳` $\rightarrow$ `DMA 自动将抓包数据搬运至乒乓缓存区` $\rightarrow$ `Core 0 接收缓存满信号，将数据写入 TF 卡对应日期目录（或通过 USB 实时转发至上位机展示波形）` $\rightarrow$ `OLED 实时滚动刷新抓包状态`。
4. **模式切换与安全停止阶段**：
   `用户按下摇杆退出当前模式` $\rightarrow$ `Core 0 通知 Core 1 停止采集并刷空残留 DMA 缓存` $\rightarrow$ `Core 0 执行 f_sync() / f_close() 安全封存日志文件` $\rightarrow$ `Core 1 挂起待命，复用引脚恢复高阻安全态`。
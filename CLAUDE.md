# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概况

手持式多协议综合分析仪（2026 “晨电智能杯” 嵌入式设计竞赛 C 题）。目标平台为 **RP2350A**（双核 Cortex-M33、520KB SRAM、3 个 PIO、外置 4MB QSPI Flash、USB FS），最终作品需自主 PCB（≤50×50mm），开发板只用于前期验证。系统需支持 UART / I2C / SPI / SWD(RTT) 抓包记录、离线寄存器读写、PY32F003 离线烧录，并配合 Windows 上位机做参数配置、透传、波形与日志 1x/2x/10x 回放。

## 当前状态

- `资料/`：赛题与设计文档，需求以 `资料/题目/需求.md` 为准。
- `RP_Bus_Sniffer/`：**已可编译**的 Pico SDK 固件工程，目前只完成第一项验证——PIO 从机监听 40MHz SPI、Core 1 抓包、ST7789 屏幕显示结果。还不是完整产品固件：没有 FatFs 落盘、RTC 时间戳、USB 复合设备与上位机协议，抓包为单次（不重装）。
- `test_sandbox/`：ESP32-S3 的独立验证工程（40MHz SPI 主机，产生测试波形），已被 `.gitignore` 排除，不属于交付物。
- 上位机、其他协议引擎（UART/I2C/SWD）、自主 PCB 均尚未开始。

## 固件工程（RP_Bus_Sniffer/）

Pico SDK 2.3.1、`PICO_BOARD=pico2`、`PICO_PLATFORM=rp2350-arm-s`、GCC 15.2.1。
组织约定：**应用入口在根目录，每个驱动一个子目录放在 `drivers/` 下**；新增驱动沿用此结构，并在 `CMakeLists.txt` 的 `add_executable` 与 `target_include_directories` 中登记。

```
RP_Bus_Sniffer/
├── RP_Bus_Sniffer.c        Core 0：系统时钟、ST7789 UI 刷新、mailbox 抽取
├── board.h                 板级引脚与系统参数（即设计文档中的 bsp_board_pins.h）
├── drivers/sniffer/        Core 1：PIO + DMA 的 40MHz SPI 抓包引擎
│   ├── sniffer.c / sniffer.h
│   └── spi_sniffer.pio
└── drivers/st7789/         1.14" 135x240 ST7789（SPI1，只写）
```

改固件时必须保持的约束：

- 引脚只在 `board.h` 里改。`PIN_SNIFF_SCK` 另有一份硬编码在 `spi_sniffer.pio` 的 `.define SCK_PIN`（PIO 的 `wait` 必须在汇编期确定引脚号），两处不一致会在编译期 `#error`；`PIN_SNIFF_MISO` 必须等于 `PIN_SNIFF_MOSI + 1`，同样有编译期检查。
- PIO 采样循环是 `wait 1 pin → in pins,2 → wait 0 pin → jmp pin`，4 个周期；系统时钟取 200MHz 才能覆盖 40MHz 的 25ns 位周期。RP2350 上必须开 `pio_set_input_sync_bypass_with_mask`，否则同步器 2 拍延迟会让采样点错过高电平。
- 位布局：`in pins,2` 的 IN_BASE 是采样字段的最低位，所以 `bit[2k]=MOSI`、`bit[2k+1]=MISO`，k=0 为最早采样点（右移 + autopush 32 位）。
- 板载 LCD 背光经 PNP 三极管 S8550 驱动，**GPIO25 拉低才亮**（见 `LCD_BL_ON_LEVEL`），不要按高有效写。
- 抓包为单次：抓满 32768 字（每条线 64KB）或 CS 拉低后 200ms 无新数据即收尾并 `__wfi()`，再抓需要复位。

## 文档与事实来源

- `资料/题目/需求.md`：赛题原文摘录（任务、基本/扩展要求、参数指标）。需求冲突时以此为准。
- `资料/项目.md`：功能清单，用 `P0`（赛题基本要求）/ `P1`（扩展要求）/ `P2`（创新加分）标注优先级；明确只列待实现功能，不含实现计划。
- `资料/设计文档.md`：固件架构设计方案（双核划分、时间戳、各协议引擎、数据落盘）。这是设计提案，不是已实现代码。
- `资料/开发板资料/`：ATK-DNRP2350AM 开发板原理图 PDF 与 RP2350A 引脚分配表，仅用于前期验证。

## 核心架构决策（固件设计文档）

编写固件时必须遵循以下已经定稿的总体方案：

1. **双核按需唤醒**：Core 0 常驻，负责 POST、OLED UI（≥30Hz）、摇杆、RTC 时基、USB（TinyUSB：CDC + WinUSB + CMSIS-DAP）、FatFs 落盘与总线复用控制。Core 1 上电默认挂起，由 Core 0 完成电平/复用器切换后通过 `multicore_launch_core1()` 启动，只跑硬实时协议引擎；切换模式时先发 `IPC_CMD_IDLE_STOP`，必要时 `multicore_reset_core1()` 彻底复位。
2. **硬件外设与 PIO 混合调度**：主机读写模式走片内硬件 I2C/SPI/UART；I2C 400kHz 监听、SPI 40MHz 监听、UART 自适应波特率过采样、SWD 烧录/RTT 走 PIO。SPI 监听依赖 MOSI/MISO 分配在相邻 GPIO，用单条 `in pins, 2` 同时锁存两线。
3. **混合时间戳**：外部 RTC 1Hz 秒中断时锁存 64 位微秒定时器值，抓包瞬间用 `epoch_sec*1e6 + (now_us - sync_us)` 合成时间戳；共享时间基准由硬件 spinlock 保护，避免双核读撕裂。Core 1 的实时路径用 `__not_in_flash_func()` 放入 SRAM。
4. **核间通信**：32-bit、深度 8 的硬件 Mailbox FIFO 传递命令码（`IPC_CMD_*`）与状态，数据本身走共享 SRAM 的 DMA 乒乓缓冲，不经过 FIFO。
5. **存储两级设计**：SRAM 划出 256KB 做 DMA 乒乓缓冲，Core 0 顺序写入 TF 卡（FAT32，日期目录 `/YYYYMMDD/`，文件名如 `SPI_143005.log`）；每满一个缓冲或约 200ms 调 `f_sync()`，掉电预警时立即停止采集并 `f_close()`。板载 QSPI Flash 不可作为 40MHz 数据流的实时写入目标（4KB 扇区擦除耗时 30~50ms），只用作掉电抢救环形缓存、PY32 固件镜像仓和字库/协议字典。
6. **引脚抽象**：所有硬件引脚集中在 `bsp_board_pins.h` 宏中，PCB 定稿后只改该头文件，业务代码不得直接写死 GPIO。当前 `RP_Bus_Sniffer/board.h` 就是该文件在此阶段的落地版本（按 ATK-DNRP2350AM 开发板接线），后续正式固件应沿用它而不是另起一套。

## 开发约束

- 所有面向用户的交流、文档与注释使用中文；代码标识符用英文。
- 新功能对照 `资料/项目.md` 的编号（H-/F-/P-）与优先级，不要自行提升范围；`P2` 功能不应阻塞 `P0`。
- 关键指标是硬性验收条件：UART 9600–2Mbps、I2C 400kHz、SPI 40MHz 下零丢包；OLED ≥30Hz；自适应波特率锁定 <50ms；掉电后已 `f_sync` 数据零丢失且文件系统可挂载。
- 设计文档中的引脚定义、目录结构、帧格式仍属提案；修改架构决策时同步更新 `资料/设计文档.md`，保持文档与代码一致。
- `资料/项目.md` 明确不包含排期与实现方案，不要往里面加实现细节。

## 常用命令

### 固件（RP_Bus_Sniffer/，Windows + Pico VS Code 扩展工具链）

```powershell
cd RP_Bus_Sniffer
$env:PICO_SDK_PATH="C:/Users/sincereli/.pico-sdk/sdk/2.3.1"
$env:PICO_TOOLCHAIN_PATH="C:/Users/sincereli/.pico-sdk/toolchain/15_2_Rel1"
$env:PATH="C:/Users/sincereli/.pico-sdk/toolchain/15_2_Rel1/bin;C:/Users/sincereli/.pico-sdk/ninja/v1.13.2;C:/Users/sincereli/.pico-sdk/cmake/v4.3.4/bin;$env:PATH"

cmake -S . -B build -G Ninja      # 首次配置或改过 CMakeLists 后
cmake --build build               # 产物 build/RP_Bus_Sniffer.uf2
```

烧录（BOOTSEL 模式）：把 `build/RP_Bus_Sniffer.uf2` 拖到 RP2350 的 U 盘，或用 picotool：

```powershell
C:/Users/sincereli/.pico-sdk/picotool/2.3.1/picotool/picotool.exe load -f build/RP_Bus_Sniffer.uf2
```

工程没有单元测试。验证方式是看屏幕上的 `STATE / WORDS / RESULT / TIME / SCK / MOSI / MISO / HASH`，以及 USB CDC 和 UART0（GP0/GP1，115200）打印的状态迁移日志。

### ESP32-S3 测试信号工程（被 git 忽略）

ESP-IDF 验证工程（ESP32-S3，40MHz SPI 读写 W25Q64 并校验一致性），用于给监听端产生激励：

```powershell
cd test_sandbox/esp32SPI40Mhz
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor          # 烧录并查看串口日志
```

本机 ESP-IDF 路径见 `test_sandbox/.vscode/settings.json`（`D:\ESP-IDF\.espressif\v6.0.1\esp-idf`），使用前先执行该目录下的 `export.ps1` 激活环境。该工程没有单元测试；验证方式是观察串口输出的 JEDEC ID、容量与“数据校验成功”日志。

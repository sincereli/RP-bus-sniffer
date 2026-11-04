# 交接说明（从仓库根目录的会话转来）

这份文件是上一段对话的现场记录。新会话请先读它，再继续工作。

## 项目与约束（必须遵守）

- 手持式多协议综合分析仪（2026 晨电智能杯 C 题），当前只做第一项验证：
  **RP2350A 上 PIO + DMA 监听 40MHz SPI**，ESP32-S3 作主机产生激励。
- 所有面向用户的交流、文档、代码注释用**中文**；代码标识符用英文。
- **不要烧录设备**：Claude 只改代码并编译验证，由用户烧录并反馈屏幕/串口。
- 引脚只在 `board.h` 里改；`spi_sniffer.pio` 里另有一份 `SCK_PIN` 硬编码，两处必须一致（有编译期检查）。
- LCD 背光 GPIO25 **低电平点亮**（PNP 三极管驱动）。
- 用户确认验证通过之前，**不要更新**仓库根目录的 `CLAUDE.md`，也不要改 `资料/设计文档.md`。
- 计划中的正式架构（双核按需唤醒、混合时间戳、FatFs 等）见仓库根 `资料/设计文档.md`；当前固件只是验证工程。

## 当前验证目标

用串口输出判断：40MHz SPI 抓包是否完整、逐字节是否正确。
期望看到：

- 页写帧（0x02 + 24 位地址 + 64 字节）：`bits=544 bytes=68`；
- 读帧（0x0B + 24 位地址 + 8 位 dummy + 64 字节）：`bits=552 bytes=69`；
- MOSI 里能看到 ESP32 写入的 `"Hello W25Q64! Counter = N"`；
- MISO 里是 W25Q64 返回的扇区内容；同一地址反复读时哈希稳定。

## 已定位的问题与修复（本次已编译，待烧录验证）

1. **漏掉 14µs 长帧**
   原因：Core 1 原来靠轮询 CS 引脚等帧开始，而解析一帧要几百微秒；ESP32 读完状态寄存器后紧接着发 64 字节读帧，Core 1 还在解析上一帧，永远看不到它。
   修复：`sniffer.c` 改用 **CS 下降沿 GPIO 中断**（`cs_fall_isr`）置标志，主循环再从 328µs 环形缓冲里取帧。

2. **16 bit 帧被解成 8 bit / 长帧从中间开始**
   原因：CS 在帧中间有 1~2 个采样点的毛刺，旧解码器把毛刺的下降沿当成帧起点。
   修复：回溯找起点时，除"下降沿之后 ≥20/32 点为低"外，新增"**下降沿之前 ≥20/32 点为高**"（`decode_last_frame` 内）。

3. **诊断日志串帧**
   原因：Core 0 逐字段读共享区，打印期间 Core 1 已处理下一帧。
   修复：`sniffer.h` 增加 `sniff_diag_t` 与双缓冲 `diag[2]` + `diag_seq`；Core 1 每帧填完整快照，Core 0 按序号读。

4. 其它：CS 高电平统计改为按字 popcount（快约 8 倍）；正向扫描追到 DMA 写指针即停；前两个"失败帧/长帧"自动留原始采样；`evt` 行短帧限流 0.5 秒一行。

5. **长帧被截成 284 bit（已改，待验证）**
   证据：用户贴的 `21:59:26` 日志里，0x0B 读帧 `lbits=284 lsmpl=1421`，MOSI 是
   `0B 00 10 00`，MISO 是 `"Hello W25Q64! Counter = 366"`，正好 35 字节处断；
   同时 `csh=4096` 说明解码时整帧早已在环里，不是没采到。把日志里的 `raw`
   1024 字逐点解出来后看到：帧中间 CS 有 1~2 个采样点的假高电平（例如
   `0xB/0x7/0x3/0x8`，一个 nibble 里几根线同时为高），位置都在 SCK 下降沿
   / MOSI 翻转处，属于长线串扰/地弹。
   原因：旧判据只看 "CS 连续为高 8 点（40ns）"，长帧中途遇到一次 ≥40ns 的
   假高电平就被当成帧尾。
   修复：帧尾判据改成 **CS 为高且连续 64 点（320ns）没有 SCK 上升沿**。
   只要时钟还在走，SPI 事务就没结束，任何长度的 CS 毛刺都截不断长帧；
   真正结束时主机会停钟。`end_run` 现在报的是 CS 实际高电平长度（≥64，
   最多量到 256）。
   同时：长帧原始转储改为**按帧起点对齐**（往前留 16 字空闲），不再从
   DMA 写指针往前截——否则主循环一忙，41µs 窗口就滑过这一帧。

## 最近一次编译

- 产物：`RP_Bus_Sniffer/build/RP_Bus_Sniffer.uf2`，88064 字节，**2026/11/4 22:43:35**。
- 开机串口第一行会打印 `build 2026-11-04 22:43:34`（时间戳由 CMake 在链接前
  生成，见下一条），用它确认板子上确实是这版。
- 用户上一份日志对应 `21:59:26`，其中长帧截断的问题已按上面第 5 条修复。
- **注意：开机时间戳不再用 `__DATE__`/`__TIME__`。** 那两个宏只在
  `RP_Bus_Sniffer.c` 自己重新编译时才更新，之前只改 `sniffer.c`，用户烧了
  22:38 的新固件却看到 22:31 的旧时间，误以为没烧进去。现在
  `cmake/gen_build_stamp.cmake` + `add_custom_target(build_stamp)` 每次构建
  都会重写 `build/build_stamp.c`，保证打印的是这一次链接进固件的时间。

## 旧日志里已确认的事实（有价值，别丢）

- 空闲原始字 `99999999`：CS=1 / SCK=0 / MOSI=0 / MISO=1，采样链路正常。
- 旧固件曾解出 `frame 1/464 bits=404 bytes=50`，MOSI 为 `36 34 21 20 43 6F 75 6E 74 65 72 20 3D 20 35 32` = `"64! Counter = 52"`。
  说明 PIO/DMA/位解析正确；缺的前半截（命令字 + 地址 + `"Hello W25Q"`）正是毛刺锁错起点造成的。
- 紧接的 `evt 464 dec=0 csh=1341`：最近 4096 点里 2755 点为 CS 低 = 13.8µs，正是 64 字节读帧的长度，但旧固件没解出来。

## 关键文件与位置

- `drivers/sniffer/sniffer.c`
  - `cs_fall_isr()`：CS 下降沿中断；
  - `decode_last_frame()`：在环形缓冲里找帧起点、正向解位、FNV 哈希；
  - `sniffer_core1_entry()`：PIO/DMA 初始化 + 主循环；
  - `publish_frame()`：把结果写入 `g_sniff`，最后写 `frame_no`。
- `drivers/sniffer/sniffer.h`：跨核共享区 `sniff_shared_t`、诊断快照 `sniff_diag_t`。
- `RP_Bus_Sniffer.c`：Core 0，屏幕刷新、串口 `state/evt/raw/frame` 日志与字节转储。
- `board.h`：引脚与 `SNIFF_MIN_BITS`（128）。
- 位布局：`bit[4k+0]=CS, +1=SCK, +2=MOSI, +3=MISO`，k=0 最早。

## 编译（新工作目录就是本文件夹）

PowerShell，在 `RP_Bus_Sniffer/` 下：

```powershell
$env:PICO_SDK_PATH="C:/Users/sincereli/.pico-sdk/sdk/2.3.1"
$env:PICO_TOOLCHAIN_PATH="C:/Users/sincereli/.pico-sdk/toolchain/15_2_Rel1"
$env:PATH="C:/Users/sincereli/.pico-sdk/toolchain/15_2_Rel1/bin;C:/Users/sincereli/.pico-sdk/ninja/v1.13.2;C:/Users/sincereli/.pico-sdk/cmake/v4.3.4/bin;$env:PATH"

cmake -S . -B build -G Ninja      # 首次或改过 CMakeLists 后
cmake --build build               # 产物 build/RP_Bus_Sniffer.uf2
```

## 下一步

1. 用户烧录 `build/RP_Bus_Sniffer.uf2`，确认开机行是 `build Nov  4 2026 22:38:26`。
2. 看长帧是否变成 `bits=552 bytes=69`（读）与 `bits=544 bytes=68`（页写），
   `full=0`、`hh=0`，`end=` 在 64~256 之间。
3. 新日志的 `evt` 行带有 `end= hh= saw= cand= ml= mh=`；若长帧仍被截断，
   先看这几个值，再看 `raw seq=... kind=long`（从帧起点开始的那段原始采样）。
4. 同时确认 16 bit 状态帧仍是 `lbits=16`，短帧不再刷屏。
5. 验证通过后：清理临时诊断（CSH/LAST/raw 等串口调试输出可保留但需精简），
   再同步更新仓库根 `CLAUDE.md` 与 `资料/设计文档.md`。

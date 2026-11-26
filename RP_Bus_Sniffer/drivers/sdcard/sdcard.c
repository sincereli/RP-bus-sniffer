/*
 * TF 卡驱动：SPI 模式 + FatFs 的 diskio 实现（Core 1 使用）
 *
 * 硬件连接见 board.h：CLK=GPIO10, MOSI=GPIO11, MISO=GPIO12, CS=GPIO15。
 * 前两根和 LCD 的 SPI1 是同一对脚，所以调用本驱动之前必须先把 LCD 总线
 * 释放掉（st7789_bus_release），结束后再调用 sdcard_deinit 交回引脚。
 *
 * SPI 由 PIO1 产生（硬件 SPI 的 RX 不能映射到 GPIO14，做不了全双工）：
 *   - 收发都是一个字节进、一个字节出，靠 TX/RX FIFO 各 4 级做缓冲；
 *   - 时钟分频在初始化阶段是 200MHz/(4*200) = 250kHz，满足 SD 规范要求
 *     的 ≤400kHz；识别成功后提到 12.5MHz（分频 4）。
 *
 * 位序：PIO 的 OUT 从 OSR 低位取、IN 把新位放 ISR 高位，都是"低位先出"，
 * 而 SD 要求 MSB 先出，所以两边各做一次 8bit 位反转（rev8）。
 */

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include "ff.h"
#include "diskio.h"

#include "sd_spi.pio.h"
#include "board.h"
#include "sdcard.h"

/* ---------------- PIO 状态机 ---------------- */

static PIO  s_pio;
static uint s_sm = 8;         /* 8 = 尚未申请（PIO 只有 4 个 SM） */
static uint s_offset;

/* 初始化时钟分频：200MHz / (4 周期/bit * 200) = 250kHz */
#define SD_DIV_INIT   200.0f
/* 数据阶段：200MHz / (4 * 4) = 12.5MHz。SD 规范 SPI 模式上限 25MHz，
 * 取 12.5MHz 是为了留出余量：杜邦线 + 排针的连线质量在 25MHz 时容易出错。 */
#define SD_DIV_DATA   4.0f

static bool s_ready;          /* 卡已完成初始化 */
static bool s_block_addr;     /* true = 按块寻址（SDHC/SDXC） */

static inline uint8_t rev8(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}

static inline void cs_low(void)  { gpio_put(PIN_SD_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_SD_CS, 1); }

/* 发送一个字节，同时返回这一拍收到的字节 */
static uint8_t sd_tx(uint8_t b)
{
    pio_sm_put_blocking(s_pio, s_sm, rev8(b));
    uint32_t w = pio_sm_get_blocking(s_pio, s_sm);
    return rev8((uint8_t)(w >> 24));
}

/* 发送 n 个 0xFF 提供时钟（CS 高时用来给卡"空转"时钟） */
static void sd_clock_ff(uint32_t n)
{
    while (n--) {
        (void)sd_tx(0xFF);
    }
}

/* 发一条命令，返回 R1；0xFF 表示等不到响应 */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc = 0xFF;
    if (cmd == 0) {
        crc = 0x95;           /* CMD0 的 CRC 在 SPI 模式下仍会被检查 */
    } else if (cmd == 8) {
        crc = 0x87;           /* CMD8 同理 */
    }

    (void)sd_tx((uint8_t)(0x40u | cmd));
    (void)sd_tx((uint8_t)(arg >> 24));
    (void)sd_tx((uint8_t)(arg >> 16));
    (void)sd_tx((uint8_t)(arg >> 8));
    (void)sd_tx((uint8_t)arg);
    (void)sd_tx(crc);

    /* NCR：命令之后最多 8 个字节内出响应（最高位为 0） */
    for (int i = 0; i < 8; i++) {
        uint8_t r = sd_tx(0xFF);
        if ((r & 0x80u) == 0) {
            return r;
        }
    }
    return 0xFF;
}

/* 读 4 字节大端数据（R7/OCR 用） */
static uint32_t sd_read_u32(void)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        v = (v << 8) | sd_tx(0xFF);
    }
    return v;
}

/* 把扇区号换算成发给卡的地址：SDHC/SDXC 直接给块号，SDSC 给字节地址 */
static uint32_t sd_addr(uint32_t lba)
{
    return s_block_addr ? lba : (lba * 512u);
}

static void sd_hw_init(float div)
{
    s_pio = pio1;
    s_pio->gpiobase = 0;      /* PIO 引脚号 = 真实 GPIO 号 */

    /* CS 先配成普通输出并拉高，卡在没被选中时忽略一切时钟 */
    gpio_init(PIN_SD_CS);
    gpio_set_dir(PIN_SD_CS, GPIO_OUT);
    gpio_put(PIN_SD_CS, 1);
    gpio_disable_pulls(PIN_SD_CS);

    /* 重复初始化时只改分频，不再占用新的状态机 */
    if (s_sm < 8) {
        pio_sm_set_enabled(s_pio, s_sm, false);
        pio_sm_set_clkdiv(s_pio, s_sm, div);
        pio_sm_set_enabled(s_pio, s_sm, true);
        return;
    }

    s_sm = pio_claim_unused_sm(s_pio, true);
    s_offset = pio_add_program(s_pio, &sd_spi_program);

    pio_sm_config cfg = sd_spi_program_get_default_config(s_offset);
    sm_config_set_out_pins(&cfg, PIN_SD_MOSI, 1);
    sm_config_set_in_pins(&cfg, PIN_SD_MISO);
    sm_config_set_sideset_pins(&cfg, PIN_SD_CLK);
    /* 右移 + 自动搬运，阈值 8：一次 FIFO 进出正好一个字节 */
    sm_config_set_out_shift(&cfg, true, true, 8);
    sm_config_set_in_shift(&cfg, true, true, 8);
    sm_config_set_clkdiv(&cfg, div);

    pio_sm_init(s_pio, s_sm, s_offset, &cfg);

    pio_gpio_init(s_pio, PIN_SD_CLK);
    pio_gpio_init(s_pio, PIN_SD_MOSI);
    pio_gpio_init(s_pio, PIN_SD_MISO);
    gpio_disable_pulls(PIN_SD_CLK);
    gpio_disable_pulls(PIN_SD_MOSI);
    /* MISO 是卡的输出，卡没被选中时是高阻；板上只有串联电阻没有上拉，
     * 这里开内部上拉，空闲时读到确定的 0xFF。 */
    gpio_pull_up(PIN_SD_MISO);

    /* SPI 模式下不用的 D1/D2 也拉高，别让它们悬空 */
    gpio_init(PIN_SD_D1);
    gpio_init(PIN_SD_D2);
    gpio_set_dir(PIN_SD_D1, GPIO_IN);
    gpio_set_dir(PIN_SD_D2, GPIO_IN);
    gpio_pull_up(PIN_SD_D1);
    gpio_pull_up(PIN_SD_D2);

    pio_sm_set_consecutive_pindirs(s_pio, s_sm, PIN_SD_CLK, 1, true);
    pio_sm_set_consecutive_pindirs(s_pio, s_sm, PIN_SD_MOSI, 1, true);
    pio_sm_set_consecutive_pindirs(s_pio, s_sm, PIN_SD_MISO, 1, false);

    /* 空闲电平：SCK 低、MOSI 高（SD 的空闲约定） */
    pio_sm_set_pins_with_mask(s_pio, s_sm, 0, 1u << PIN_SD_CLK);
    pio_sm_set_pins_with_mask(s_pio, s_sm, 1u << PIN_SD_MOSI,
                              1u << PIN_SD_MOSI);

    pio_sm_set_enabled(s_pio, s_sm, true);
}

/* 等卡结束内部写操作：MISO 被拉低（读 0x00）表示忙 */
static bool sd_wait_ready(uint32_t timeout_us)
{
    uint64_t t0 = time_us_64();
    while (sd_tx(0xFF) == 0x00) {
        if (time_us_64() - t0 > timeout_us) {
            return false;
        }
    }
    return true;
}

bool sdcard_init(sdcard_info_t *info)
{
    memset(info, 0, sizeof(*info));
    s_ready = false;

    sd_hw_init(SD_DIV_INIT);

    /* 上电：CS 高 + 至少 74 个时钟，让卡进入 SPI 模式 */
    cs_high();
    sd_clock_ff(10);

    /* CMD0：进入 idle 状态，正常响应 0x01 */
    cs_low();
    uint8_t r = sd_cmd(0, 0);
    cs_high();
    (void)sd_tx(0xFF);
    if (r != 0x01) {
        info->err_stage = 1;
        return false;
    }

    /* CMD8：判断是不是 v2 卡，同时校验 2.7~3.6V 工作电压 */
    bool v2 = false;
    cs_low();
    r = sd_cmd(8, 0x1AA);
    if (r == 0x01) {
        info->r7 = sd_read_u32();
        if ((info->r7 & 0xFFFu) != 0x1AAu) {
            cs_high();
            info->err_stage = 2;
            return false;
        }
        v2 = true;
    } else if ((r & 0x04u) == 0) {
        /* 只有"非法命令"才说明是老卡；其它响应都是异常 */
        cs_high();
        info->err_stage = 2;
        return false;
    }
    cs_high();
    (void)sd_tx(0xFF);
    info->v2 = v2;

    /* ACMD41：等待卡完成上电初始化。v2 卡要带上 HCS 位声明支持大容量 */
    uint32_t arg = v2 ? 0x40000000u : 0u;
    uint64_t t0 = time_us_64();
    for (;;) {
        cs_low();
        (void)sd_cmd(55, 0);
        cs_high();
        (void)sd_tx(0xFF);

        cs_low();
        r = sd_cmd(41, arg);
        cs_high();
        (void)sd_tx(0xFF);

        if (r == 0x00) {
            break;
        }
        if (time_us_64() - t0 > 2000000u) {
            info->err_stage = 3;
            return false;
        }
        sleep_ms(2);
    }

    /* CMD58：读 OCR，bit30 = 1 表示按块寻址（SDHC/SDXC） */
    cs_low();
    r = sd_cmd(58, 0);
    if (r != 0x00) {
        cs_high();
        info->err_stage = 4;
        return false;
    }
    info->ocr = sd_read_u32();
    cs_high();
    (void)sd_tx(0xFF);
    s_block_addr = (info->ocr & (1u << 30)) != 0;
    info->block_addressing = s_block_addr;

    /* 老卡需要显式把块长设为 512；SDHC 卡固定 512，CMD16 无副作用 */
    cs_low();
    (void)sd_cmd(16, 512);
    cs_high();
    (void)sd_tx(0xFF);

    /* 识别完成，提到数据时钟 */
    pio_sm_set_enabled(s_pio, s_sm, false);
    pio_sm_set_clkdiv(s_pio, s_sm, SD_DIV_DATA);
    pio_sm_set_enabled(s_pio, s_sm, true);

    s_ready = true;
    info->err_stage = 0;
    return true;
}

void sdcard_deinit(void)
{
    if (s_sm < 8) {
        pio_sm_set_enabled(s_pio, s_sm, false);
        pio_sm_unclaim(s_pio, s_sm);
        s_sm = 8;             /* 标记已释放 */
    }
    cs_high();
    gpio_set_function(PIN_SD_CS, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_SD_CS, GPIO_IN);

    /* 把共用的 GPIO10/11（和 LCD 同一对脚）交回 SIO，高阻输入 */
    const uint pins[] = { PIN_SD_CLK, PIN_SD_MOSI, PIN_SD_MISO };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_set_function(pins[i], GPIO_FUNC_SIO);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
}

/* 读一个 512 字节块 */
static bool sd_read_block(uint32_t lba, uint8_t *buf)
{
    cs_low();
    uint8_t r = sd_cmd(17, sd_addr(lba));
    if (r != 0x00) {
        cs_high();
        (void)sd_tx(0xFF);
        return false;
    }

    /* 等数据起始令牌 0xFE */
    uint64_t t0 = time_us_64();
    for (;;) {
        uint8_t t = sd_tx(0xFF);
        if (t == 0xFE) {
            break;
        }
        if (time_us_64() - t0 > 200000u) {
            cs_high();
            return false;
        }
    }

    for (uint32_t i = 0; i < 512u; i++) {
        buf[i] = sd_tx(0xFF);
    }
    (void)sd_tx(0xFF);        /* CRC16，忽略 */
    (void)sd_tx(0xFF);

    cs_high();
    (void)sd_tx(0xFF);
    return true;
}

/* 写一个 512 字节块 */
static bool sd_write_block(uint32_t lba, const uint8_t *buf)
{
    cs_low();
    uint8_t r = sd_cmd(24, sd_addr(lba));
    if (r != 0x00) {
        cs_high();
        (void)sd_tx(0xFF);
        return false;
    }

    (void)sd_tx(0xFF);        /* 命令与数据之间至少一个字节 */
    (void)sd_tx(0xFE);        /* 数据起始令牌 */

    for (uint32_t i = 0; i < 512u; i++) {
        (void)sd_tx(buf[i]);
    }
    (void)sd_tx(0xFF);        /* CRC16 占位 */
    (void)sd_tx(0xFF);

    /* 数据响应：低 5 位 0x05 表示写入被接受 */
    uint8_t resp = sd_tx(0xFF);
    if ((resp & 0x1Fu) != 0x05u) {
        cs_high();
        (void)sd_tx(0xFF);
        return false;
    }

    /* 等卡把数据真正写进 Flash（忙时 MISO 拉低） */
    (void)sd_wait_ready(500000u);
    cs_high();
    (void)sd_tx(0xFF);
    return true;
}

bool sdcard_read_blocks(uint32_t lba, uint8_t *buf, uint32_t count)
{
    if (!s_ready) {
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        if (!sd_read_block(lba + i, buf + i * 512u)) {
            return false;
        }
    }
    return true;
}

bool sdcard_write_blocks(uint32_t lba, const uint8_t *buf, uint32_t count)
{
    if (!s_ready) {
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        if (!sd_write_block(lba + i, buf + i * 512u)) {
            return false;
        }
    }
    return true;
}

/* ---------------- FatFs 的 diskio 接口 ---------------- */

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0 || !s_ready) {
        return STA_NOINIT;
    }
    return 0;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0) {
        return STA_NOINIT;
    }
    if (s_ready) {
        return 0;
    }
    sdcard_info_t info;
    return sdcard_init(&info) ? 0 : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || !s_ready) {
        return RES_NOTRDY;
    }
    return sdcard_read_blocks((uint32_t)sector, buff, count) ? RES_OK
                                                             : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || !s_ready) {
        return RES_NOTRDY;
    }
    return sdcard_write_blocks((uint32_t)sector, buff, count) ? RES_OK
                                                              : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)buff;
    if (pdrv != 0) {
        return RES_PARERR;
    }
    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;        /* 每块写都已经等卡忙完，不需要额外同步 */
    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}

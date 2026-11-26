#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capview.py —— 解析 RP_Bus_Sniffer 落盘的两种原始采样文件，把
CS / SCK / MOSI / MISO 四路信号画成时序图，并按 CS 帧解码 SPI 字节。

两种格式按文件头 magic 自动区分：
    'RPSN' -> RPCAP.BIN    一次性窗口（CAP_MODE_WINDOW），一次运行若干条
    'RPSF' -> RPSTREAM.BIN 按帧连续记录（CAP_MODE_STREAM），整段总线流量

---- RPCAP.BIN（一次性窗口）----

文件格式（小端，见 drivers/sniffer/sniffer.c 的 cap_hdr_t）：
    offset 0   magic      'RPSN'
    offset 4   version    u16 = 1
    offset 6   hdr_bytes  u16 = 36
    offset 8   seq        记录序号
    offset 12  sample_hz  采样率（200MHz）
    offset 16  words      本记录字数（8192）
    offset 20  trig_word  CS 下降沿触发瞬间的环内字序号
    offset 24  stop_word  停止采样瞬间的 DMA 写指针（= 数据起点）
    offset 28  trig_us    触发到停止的墙钟微秒数
    offset 32  flags      bit0 = clk 已实测，bit1 = 卡按块寻址
    之后 words*4 字节，按时间从旧到新排列。
    每个 32bit 字按时间展开成 8 个采样点，第 k 个采样点的 4 个 bit 是：
        bit[4k+0] = CS   bit[4k+1] = SCK   bit[4k+2] = MOSI   bit[4k+3] = MISO

---- RPSTREAM.BIN（按帧连续记录）----

文件头 24 字节 <IHHIIII>：magic('RPSF') ver hdr_bytes sample_hz clk_khz rsv rsv
之后每条帧记录：<IHBB> t_us words flags pad，紧跟 words*4 字节原始采样字。
flags bit0 = 截断（环里没找到 CS 上升沿，按单帧上限收尾）。
采样字的位布局与 RPCAP.BIN 完全一致，所以解包/解码共用同一套代码。

用法示例（一次性窗口）：
    python tools/capview.py RPCAP.BIN                  # 汇总 + 第 0 条记录出图
    python tools/capview.py RPCAP.BIN --record 3       # 只看第 3 条
    python tools/capview.py RPCAP.BIN --all            # 每条记录都出图
    python tools/capview.py RPCAP.BIN --pre 0.5 --post 1   # 缩放窗（触发前/后 us）
    python tools/capview.py RPCAP.BIN --t0=42 --t1=43  # 直接框定触发后某个区间
    python tools/capview.py RPCAP.BIN --list           # 列出所有 SCK 突发
    python tools/capview.py RPCAP.BIN --list --burst 9 # 放大第 9 段突发
    python tools/capview.py RPCAP.BIN --csv rec0.csv   # 导出采样点，方便 MATLAB 读
    python tools/capview.py RPCAP.BIN --theme dark     # 暗色主题
    python tools/capview.py RPCAP.BIN --show           # 弹窗看图（否则只存 PNG）

用法示例（连续记录）：
    python tools/capview.py RPSTREAM.BIN               # 汇总 + 最长的数据帧出图
    python tools/capview.py RPSTREAM.BIN --list        # 列出所有数据类帧（≥16 字节）
    python tools/capview.py RPSTREAM.BIN --list --min-bits 0   # 连状态轮询也列出来
    python tools/capview.py RPSTREAM.BIN --frame 8321  # 只看第 8321 帧
    python tools/capview.py RPSTREAM.BIN --all         # 每个数据类帧都出图
    python tools/capview.py RPSTREAM.BIN --pre 2 --post 12     # 缩放窗覆盖 13.6us 的读帧
    python tools/capview.py RPSTREAM.BIN --all-long    # 全部数据帧列表（不限 40 条）

说明：
  * 图上的坐标轴与标题用英文——matplotlib 默认字体没有中文字形，写中文会变方框；
    控制台输出仍是中文。
  * MOSI / MISO 在 SCK 上升沿之后约半个位宽处取样（此时数据已稳定），
    与真实从机器件在 SPI mode 0 下的采样时刻一致。
  * 波形时间轴以触发点（CS 下降沿）为零点，负值表示触发之前；
    连续记录模式下每条帧记录各自以本帧的 CS 下降沿为零点。
"""

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

import matplotlib
if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker


# ---------------------------------------------------------------- 记录格式

HDR_FMT = "<IHHIIIIIII"
HDR_SIZE = struct.calcsize(HDR_FMT)          # 36
MAGIC = 0x4E535052                            # "RPSN"
SAMPLES_PER_WORD = 8

CHANNELS = (
    ("CS",   "cs",   0),
    ("SCK",  "sck",  1),
    ("MOSI", "mosi", 2),
    ("MISO", "miso", 3),
)

# 主题：取自 dataviz 技能参考调色板。四条曲线各占一个带直标的子图，
# 属于"单序列面板"，不需要分类配色，统一用一个强调色；
# 触发标记是图表装饰，用次级文字色，不占用状态色。
THEMES = {
    "light": {
        "surface":   "#fcfcfb",
        "ink":       "#0b0b0b",
        "ink2":      "#52514e",
        "muted":     "#898781",
        "grid":      "#e1e0d9",
        "axis":      "#c3c2b7",
        "series":    "#2a78d6",
        "marker":    "#52514e",
    },
    "dark": {
        "surface":   "#1a1a19",
        "ink":       "#f0efec",
        "ink2":      "#c3c2b7",
        "muted":     "#898781",
        "grid":      "#2c2c2a",
        "axis":      "#383835",
        "series":    "#3987e5",
        "marker":    "#c3c2b7",
    },
}


def read_records(path):
    """把整个文件切成一条条记录，返回 dict 列表。遇到残缺数据就停下并告警。"""
    raw = Path(path).read_bytes()
    recs = []
    off = 0
    while off < len(raw):
        if len(raw) - off < HDR_SIZE:
            print("警告：偏移 %d 处只剩 %d 字节，不足一个记录头，忽略尾部"
                  % (off, len(raw) - off))
            break
        (magic, version, hdr_bytes, seq, sample_hz, words,
         trig_word, stop_word, trig_us, flags) = struct.unpack_from(HDR_FMT, raw, off)
        if magic != MAGIC:
            print("警告：偏移 %d 处 magic=0x%08X 不是 'RPSN'，停止解析"
                  % (off, magic))
            break

        hdr_bytes = max(hdr_bytes, HDR_SIZE)      # 兼容以后加长的头部
        data_off = off + hdr_bytes
        need = words * 4
        avail = len(raw) - data_off
        if avail < need:
            print("警告：记录 %d 数据不完整（需要 %d 字节，实际 %d），"
                  "可能是写到一半掉电" % (seq, need, avail))
            break

        words_arr = np.frombuffer(raw, dtype="<u4", count=words, offset=data_off)
        recs.append({
            "seq": seq,
            "version": version,
            "sample_hz": sample_hz,
            "words": words,
            "trig_word": trig_word,
            "stop_word": stop_word,
            "trig_us": trig_us,
            "flags": flags,
            "data": words_arr,
        })
        off = data_off + need

    return recs


def unpack_samples(words):
    """把 8192 个字拆成 4 路 0/1 采样序列，顺序与采集时一致。"""
    shifts = (np.arange(SAMPLES_PER_WORD, dtype=np.uint32) * 4)
    nib = ((words[:, None] >> shifts[None, :]) & 0xF).astype(np.uint8)
    nib = nib.reshape(-1)                     # 每个采样点一个 4bit 半字节
    return {key: ((nib >> bit) & 1) for _, key, bit in CHANNELS}


def time_axis(rec, n):
    """以触发点为 0 的时间轴（us）。触发点在数据里的位置由环回绕量决定。"""
    words = rec["words"]
    trig_off = ((rec["trig_word"] - rec["stop_word"]) % words) * SAMPLES_PER_WORD
    t = (np.arange(n, dtype=np.float64) - trig_off) * 1e6 / rec["sample_hz"]
    return t, trig_off


# ---------------------------------------------------------------- 解码

def _edges(v, rising=True):
    if rising:
        return np.flatnonzero((v[:-1] == 0) & (v[1:] == 1)) + 1
    return np.flatnonzero((v[:-1] == 1) & (v[1:] == 0)) + 1


def _bits_to_bytes(bits):
    """MSB 在前的比特流 -> 字节列表、余下的 bit 数。"""
    n = len(bits) // 8
    out = []
    for i in range(n):
        v = 0
        for b in bits[i * 8:i * 8 + 8]:
            v = (v << 1) | int(b)
        out.append(v)
    return out, len(bits) % 8


def decode_frames(rec, sig):
    """按 CS 低电平区间切帧，在 SCK 上升沿后的位中点取 MOSI/MISO。"""
    cs, sck = sig["cs"], sig["sck"]
    mosi, miso = sig["mosi"], sig["miso"]
    fs = float(rec["sample_hz"])

    falls = _edges(cs, rising=False)
    rises = _edges(cs, rising=True)

    frames = []
    j = 0
    for f in falls:
        while j < len(rises) and rises[j] <= f:
            j += 1
        if j >= len(rises):
            break                                   # 帧尾在记录之外
        r = rises[j]
        j += 1

        seg = sck[f:r]
        clk = np.flatnonzero((seg[:-1] == 0) & (seg[1:] == 1)) + f

        if len(clk) < 2:
            frames.append({"glitch": True, "start": f, "end": r,
                           "sck_edges": int(len(clk))})
            continue

        # 位中点取样：相邻上升沿间距的一半，钳在 CS 结束之前
        spacing = np.diff(clk)
        half = int(max(1, np.median(spacing) // 2))
        idx = np.minimum(clk + half, r - 1)

        m_bit = mosi[idx]
        s_bit = miso[idx]
        m_bytes, m_rem = _bits_to_bytes(m_bit)
        s_bytes, s_rem = _bits_to_bytes(s_bit)

        frames.append({
            "glitch": False,
            "start": f,
            "end": r,
            "sck_edges": int(len(clk)),
            "t_start_us": (f - _trig_off_global) * 1e6 / fs,
            "t_end_us": (r - _trig_off_global) * 1e6 / fs,
            "clk_mhz": (fs / np.median(spacing) / 1e6) if len(spacing) else 0.0,
            "mosi": m_bytes,
            "miso": s_bytes,
            "bits": int(len(m_bit)),
            "rem": m_rem,
            "mosi_bits": m_bit,
            "miso_bits": s_bit,
        })
    return frames


_trig_off_global = 0


def hex_bytes(vals):
    return " ".join("%02X" % b for b in vals)


def ascii_bytes(vals):
    return "".join(chr(b) if 32 <= b < 127 else "." for b in vals)


def print_record(rec, sig, frames, idx):
    fs = rec["sample_hz"]
    n = len(sig["cs"])
    print()
    print("记录 %d：%d 字 / %d 采样点 = %.1f us，采样率 %.3f MHz%s"
          % (idx, rec["words"], n, n * 1e6 / fs, fs / 1e6,
             "" if rec["flags"] & 1 else "（采样率未实测，用的是 clk_sys）"))
    print("  触发：环内字 %d，停止：环内字 %d，触发→停止实测 %d us"
          % (rec["trig_word"], rec["stop_word"], rec["trig_us"]))
    if rec["flags"] & 4:
        print("  注意：触发离重新武装不到一整圈（328us），"
              "记录开头一段是上一轮的残留采样")

    good = [f for f in frames if not f["glitch"]]
    bad = [f for f in frames if f["glitch"]]
    total_bytes = sum(len(f["mosi"]) for f in good)
    print("  CS 帧 %d 个（另有 %d 个无时钟的毛刺），共解出 %d 字节"
          % (len(good), len(bad), total_bytes))
    if not good:
        print("  这条记录里没有完整的 SPI 帧——看看是不是只有 CS 噪声。")
        return

    print()
    print("  #   t_start(us)  t_end(us)   宽(us)  SCK(MHz)  bit   MOSI")
    for k, f in enumerate(good):
        print("  %-3d %10.3f  %9.3f  %7.3f  %8.2f  %4d  %s"
              % (k, f["t_start_us"], f["t_end_us"],
                 f["t_end_us"] - f["t_start_us"], f["clk_mhz"], f["bits"],
                 hex_bytes(f["mosi"][:16]) + (" ..." if len(f["mosi"]) > 16 else "")))

    print()
    print("  逐帧详情：")
    for k, f in enumerate(good):
        print("  帧 %d  CS 低 %.3f→%.3f us，SCK %.2f MHz，%d bit%s"
              % (k, f["t_start_us"], f["t_end_us"], f["clk_mhz"], f["bits"],
                 "（尾部 %d bit 不成字节）" % f["rem"] if f["rem"] else ""))
        print("    MOSI %d 字节: %s" % (len(f["mosi"]), hex_bytes(f["mosi"])))
        print("         ASCII : %s" % ascii_bytes(f["mosi"]))
        print("    MISO %d 字节: %s" % (len(f["miso"]), hex_bytes(f["miso"])))
        print("         ASCII : %s" % ascii_bytes(f["miso"]))
    if bad:
        print("  毛刺（CS 拉低但没有时钟）：")
        for f in bad:
            print("    %.3f us 处，宽 %d 个采样点"
                  % ((f["start"] - _trig_off_global) * 1e6 / fs, f["end"] - f["start"]))


# ---------------------------------------------------------------- 绘图

# 横轴主刻度候选（us）。窗口越窄，刻度越细，缩到 1us 以内时每格 10ns，
# 40MHz 的一个 bit 占 5 个采样点、25ns，能数得清。
TICK_STEPS = (0.005, 0.01, 0.02, 0.025, 0.05, 0.1, 0.2, 0.25, 0.5,
              1.0, 2.0, 5.0, 10.0, 20.0, 25.0, 50.0, 100.0)


def style_axes(ax, th, label):
    ax.set_facecolor(th["surface"])
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["0", "1"])
    ax.set_ylim(-0.25, 1.35)
    ax.set_ylabel(label, color=th["ink2"], fontsize=10, labelpad=6)
    ax.grid(True, axis="x", color=th["grid"], linewidth=0.6)
    ax.grid(True, axis="y", color=th["grid"], linewidth=0.4)
    ax.tick_params(colors=th["muted"], labelsize=8, length=3)
    ax.tick_params(which="minor", length=2, colors=th["muted"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(th["axis"])


def set_time_ticks(ax, xlim):
    span = xlim[1] - xlim[0]
    step = TICK_STEPS[-1]
    for c in TICK_STEPS:
        if span / c <= 10:
            step = c
            break
    ax.xaxis.set_major_locator(mticker.MultipleLocator(step))
    ax.xaxis.set_minor_locator(mticker.MultipleLocator(step / 5.0))
    ax.xaxis.set_major_formatter(mticker.FormatStrFormatter("%g"))


def find_bursts(sig, gap=10, min_bits=2):
    """按 SCK 的间隔把时钟活动切成一段段，不依赖 CS。"""
    rise = _edges(sig["sck"], rising=True)
    if rise.size == 0:
        return []
    cut = np.flatnonzero(np.diff(rise) > gap) + 1
    return [g for g in np.split(rise, cut) if g.size >= min_bits]


def burst_payload(sig, rise):
    """在每拍上升沿后的位中点取 MOSI/MISO，拼成字节。"""
    if rise.size > 1:
        half = int(max(1, np.median(np.diff(rise)) // 2))
    else:
        half = 2
    idx = rise + half
    mb, _ = _bits_to_bytes(sig["mosi"][idx])
    sb, _ = _bits_to_bytes(sig["miso"][idx])
    return mb, sb


def print_bursts(sig, t, bursts):
    print("  #    t_start(us)  宽(us)  bit   MOSI")
    for k, b in enumerate(bursts):
        mb, _ = burst_payload(sig, b)
        width = t[b[-1]] - t[b[0]]
        print("  %-3d %10.3f    %6.3f  %3d   %s"
              % (k, t[b[0]], width, b.size,
                 hex_bytes(mb[:12]) + (" ..." if len(mb) > 12 else "")))


def plot_window(sig, t, xlim, title, outpath, th):
    mask = (t >= xlim[0]) & (t <= xlim[1])
    tt = t[mask]
    if tt.size == 0:
        print("警告：窗口 %.3f..%.3f us 内没有采样点，跳过绘图" % xlim)
        return False

    # 每一拍上升沿画一条淡参考线，方便按 bit 读数据
    sw = sig["sck"][mask]
    edges = np.flatnonzero((sw[:-1] == 0) & (sw[1:] == 1)) + 1
    guide_t = tt[edges]
    draw_guides = 0 < guide_t.size <= 240

    fig, axes = plt.subplots(len(CHANNELS), 1, figsize=(12, 8.4),
                             sharex=True, facecolor=th["surface"])
    for ax, (label, key, _) in zip(axes, CHANNELS):
        ax.step(tt, sig[key][mask], where="post", color=th["series"],
                linewidth=1.2, solid_joinstyle="miter")
        style_axes(ax, th, label)
        if draw_guides:
            for x in guide_t:
                ax.axvline(x, color=th["series"], linewidth=0.5,
                           alpha=0.18, zorder=0)
        if xlim[0] <= 0.0 <= xlim[1]:
            ax.axvline(0.0, color=th["marker"], linewidth=1.0, linestyle="--")

    axes[-1].set_xlabel("Time relative to capture trigger (us)",
                        color=th["ink2"], fontsize=10)
    axes[-1].set_xlim(*xlim)
    set_time_ticks(axes[-1], xlim)
    if xlim[0] <= 0.0 <= xlim[1]:
        axes[0].text(0.0, 1.22, " trigger", color=th["ink2"], fontsize=9,
                     ha="left", va="bottom")

    fig.suptitle(title, color=th["ink"], fontsize=12, x=0.012, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    fig.savefig(outpath, dpi=150, facecolor=th["surface"])
    plt.close(fig)
    print("  已保存 %s" % outpath)
    return True


def plot_record(rec, sig, t, args, outdir, bursts):
    fs = rec["sample_hz"]
    n = len(t)
    th = THEMES[args.theme]

    win_us = n * 1e6 / fs
    base = "rec%d" % rec["seq"]
    flags = []
    if rec["flags"] & 1:
        flags.append("clk measured")
    title = "RPCAP record %d - %d samples @ %.3f MHz = %.1f us window%s" % (
        rec["seq"], n, fs / 1e6, win_us,
        (" (" + ", ".join(flags) + ")") if flags else "")

    plot_window(sig, t, (t[0], t[-1]),
                title + "  |  overview",
                outdir / (base + "_overview.png"), th)

    # 缩放窗：--burst > --t0/--t1 > --pre/--post
    tag = "zoom"
    if args.burst is not None and 0 <= args.burst < len(bursts):
        b = bursts[args.burst]
        z0, z1 = t[b[0]] - 0.3, t[b[-1]] + 0.3
        tag = "burst%d" % args.burst
    elif args.t0 is not None and args.t1 is not None:
        z0, z1 = args.t0, args.t1
    else:
        z0, z1 = -args.pre, args.post

    plot_window(sig, t, (z0, z1),
                title + "  |  %.3f .. %.3f us" % (z0, z1),
                outdir / (base + "_%s.png" % tag), th)


def export_csv(rec, sig, t, path):
    arr = np.column_stack([np.arange(t.size), t,
                           sig["cs"], sig["sck"], sig["mosi"], sig["miso"]])
    np.savetxt(path, arr, fmt=["%d", "%.4f", "%d", "%d", "%d", "%d"],
               delimiter=",", header="n,t_us,cs,sck,mosi,miso", comments="")
    print("  已导出采样点 %s（%d 行）" % (path, len(t)))


# ------------------------------------------------------------ 连续记录格式
#
# 固件 CAP_MODE_STREAM 生成的 RPSTREAM.BIN：
#   文件头 24 字节  <IHHIIII> magic('RPSF') ver hdr_bytes sample_hz clk_khz rsv rsv
#   每帧            <IHBB>     t_us words flags pad ，后接 words*4 字节原始采样字
#
# 与 RPCAP.BIN 的差别：这里没有"环形缓冲"的概念，每一个 CS 帧单独一段，
# 各段的采样字布局完全相同（bit[4k+0]=CS +1=SCK +2=MOSI +3=MISO），
# 所以解包和解码可以共用上面那套函数。

STREAM_MAGIC = 0x46535052                    # "RPSF"
STREAM_HDR_FMT = "<IHHIIII"
STREAM_HDR_SIZE = struct.calcsize(STREAM_HDR_FMT)      # 24
STREAM_REC_HDR = 8
STREAM_F_TRUNC = 0x01


def detect_format(path):
    """按文件头 magic 判断是哪种记录格式。"""
    with open(path, "rb") as fp:
        head = fp.read(4)
    if len(head) < 4:
        return None
    magic = struct.unpack("<I", head)[0]
    if magic == MAGIC:
        return "rpcap"
    if magic == STREAM_MAGIC:
        return "stream"
    return None


def read_stream(path):
    """解析 RPSTREAM.BIN。遇到残缺的尾部就停下并告警。"""
    raw = Path(path).read_bytes()
    if len(raw) < STREAM_HDR_SIZE:
        print("文件只有 %d 字节，连文件头都不够" % len(raw))
        return None
    (magic, ver, hdr_bytes, sample_hz,
     clk_khz, _r0, _r1) = struct.unpack_from(STREAM_HDR_FMT, raw, 0)
    if magic != STREAM_MAGIC:
        return None
    hdr_bytes = max(hdr_bytes, STREAM_HDR_SIZE)

    frames = []
    off = hdr_bytes
    while off < len(raw):
        if len(raw) - off < STREAM_REC_HDR:
            print("警告：偏移 %d 处只剩 %d 字节，不足一个帧头，忽略尾部"
                  % (off, len(raw) - off))
            break
        t_us, words, flags, _pad = struct.unpack_from("<IHBB", raw, off)
        data_off = off + STREAM_REC_HDR
        need = words * 4
        if len(raw) - data_off < need:
            print("警告：第 %d 帧数据不完整（需要 %d 字节，实际 %d），"
                  "可能是写到一半掉电"
                  % (len(frames), need, len(raw) - data_off))
            break
        frames.append({
            "idx": len(frames),
            "t_us": t_us,
            "words": words,
            "flags": flags,
            "data": np.frombuffer(raw, dtype="<u4", count=words, offset=data_off),
        })
        off = data_off + need

    return {"sample_hz": sample_hz, "clk_khz": clk_khz, "version": ver,
            "frames": frames, "path": path}


def unpack_stream_frame(fr):
    """拆一条连续记录的帧，并在最前面补一个 CS=1 的采样点。

    固件从"含 CS 下降沿的那个字"开始记录，所以边沿有可能正好落在第一个
    采样点上。边沿检测靠 (前一点==1) & (当前点==0)，首点没有前一点就漏掉；
    补一个高电平点后，帧头在采样流里就总是可识别的。
    """
    sig = unpack_samples(fr["data"])
    pad = np.zeros(1, dtype=np.uint8)
    out = {}
    for key in sig:
        out[key] = np.concatenate(([1] if key == "cs" else pad, sig[key]))
    return out


def decode_stream_frame(fs, fr):
    """解一帧，返回摘要（不保留采样数组，否则上万帧会把内存吃光）。"""
    global _trig_off_global
    sig = unpack_stream_frame(fr)
    saved = _trig_off_global
    _trig_off_global = 0
    dec = decode_frames({"sample_hz": fs}, sig)
    _trig_off_global = saved

    good = [d for d in dec if not d["glitch"]]
    # 一条记录理论上只含一个 CS 低区间；若因 CS 毛刺被切成几段，取最长的那段
    d = max(good, key=lambda x: x["bits"]) if good else None
    base = {
        "idx": fr["idx"],
        "t_us": fr["t_us"],
        "words": fr["words"],
        "flags": fr["flags"],
        "glitches": len(dec) - len(good),
    }
    if d is None:
        base.update(bits=0, mosi=[], miso=[], rem=0, clk_mhz=0.0,
                    width_us=0.0)
    else:
        base.update(bits=d["bits"], mosi=d["mosi"], miso=d["miso"],
                    rem=d["rem"], clk_mhz=d["clk_mhz"],
                    width_us=d["t_end_us"] - d["t_start_us"])
    return base


def analyze_stream(st):
    fs = float(st["sample_hz"])
    return [decode_stream_frame(fs, fr) for fr in st["frames"]]


def print_stream_summary(st, frs):
    fs = float(st["sample_hz"])
    span_s = (frs[-1]["t_us"] - frs[0]["t_us"]) / 1e6 if len(frs) > 1 else 0.0
    trunc = sum(1 for f in frs if f["flags"] & STREAM_F_TRUNC)
    glitch = sum(f["glitches"] for f in frs)

    print()
    print("连续记录 %s" % st["path"])
    print("  采样率 %.3f MHz（固件实测 %d kHz），共 %d 帧，时间跨度 %.3f 秒"
          % (fs / 1e6, st["clk_khz"], len(frs), span_s))
    if trunc:
        print("  %d 帧被标为截断（环里没找到 CS 上升沿，按单帧上限收尾）"
              % trunc)
    if glitch:
        print("  %d 段 CS 毛刺（记录头 80ns 阈值以下，固件已丢弃多数的）"
              % glitch)

    # 按字数分布：轮询帧 10 字上下，页编程/读帧约 340 字
    hist = {}
    for f in frs:
        hist[f["words"]] = hist.get(f["words"], 0) + 1
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:8]
    print("  帧长分布（前 %d 种，字 / 帧数 / 时长）：" % len(top))
    for w, c in top:
        print("    %5d 字 %6d 帧  %8.3f us" % (w, c, w * 8 * 1e6 / fs))
    if len(hist) > len(top):
        print("    ...（共 %d 种长度）" % len(hist))

    long_frames = [f for f in frs if f["bits"] >= 128]
    print("  数据类帧（≥16 字节）：%d 帧" % len(long_frames))
    if long_frames:
        print()
        print("   #     t(us)      宽(us)  SCK(MHz)  bit  MOSI")
        for f in long_frames[:40]:
            tag = " [TRUNC]" if f["flags"] & STREAM_F_TRUNC else ""
            print("  %-5d %10.3f  %8.3f  %8.2f  %4d  %s%s"
                  % (f["idx"], f["t_us"], f["width_us"], f["clk_mhz"],
                     f["bits"], hex_bytes(f["mosi"][:12]) +
                     (" ..." if len(f["mosi"]) > 12 else ""), tag))
        if len(long_frames) > 40:
            print("  ...（还有 %d 帧，用 --all-long 全部列出）"
                  % (len(long_frames) - 40))


def print_stream_frames(frs, min_bits=0, limit=None):
    print()
    print("  #     t(us)     t_end(us)  宽(us)  字  bit  SCK(MHz)  MOSI")
    shown = 0
    for f in frs:
        if f["bits"] < min_bits:
            continue
        t_end = f["t_us"] + f["width_us"]
        print("  %-5d %10.3f  %10.3f  %7.3f  %3d  %4d  %8.2f  %s"
              % (f["idx"], f["t_us"], t_end, f["width_us"], f["words"],
                 f["bits"], f["clk_mhz"],
                 hex_bytes(f["mosi"][:8]) + (" ..." if len(f["mosi"]) > 8 else "")))
        shown += 1
        if limit is not None and shown >= limit:
            print("  ...（已显示 %d 帧，用 --limit 调整）" % limit)
            break


def plot_stream_frame(st, f, args, outdir, th):
    """把某一帧画成 4 路时序图。时间零点放在 CS 下降沿。"""
    fs = float(st["sample_hz"])
    fr = st["frames"][f["idx"]]
    sig = unpack_stream_frame(fr)
    n = len(sig["cs"])

    cs = sig["cs"]
    fall = _edges(cs, rising=False)
    zero = int(fall[0]) if fall.size else 0
    t = (np.arange(n, dtype=np.float64) - zero) * 1e6 / fs

    base = "frame%05d" % f["idx"]
    title = ("RPSTREAM frame %d @ %.3f s - %d words, %d bits, %.2f MHz"
             % (f["idx"], f["t_us"] / 1e6, f["words"], f["bits"], f["clk_mhz"]))

    plot_window(sig, t, (t[0], t[-1]), title + "  |  overview",
                outdir / (base + "_overview.png"), th)

    z0, z1 = -args.pre, args.post
    if args.t0 is not None and args.t1 is not None:
        z0, z1 = args.t0, args.t1
    plot_window(sig, t, (z0, z1),
                title + "  |  %.3f .. %.3f us" % (z0, z1),
                outdir / (base + "_zoom.png"), th)


# ---------------------------------------------------------------- 主程序

def run_stream(args, bin_path):
    st = read_stream(bin_path)
    if st is None:
        print("不是有效的 RPSTREAM.BIN")
        return 1
    if not st["frames"]:
        print("文件里没有任何帧。卡上是不是没写完？")
        return 1

    frs = analyze_stream(st)
    print("文件 %s，%.1f KB"
          % (bin_path.name, bin_path.stat().st_size / 1024.0))
    print_stream_summary(st, frs)
    if args.all_long:
        long_frames = [f for f in frs if f["bits"] >= 128]
        if len(long_frames) > 40:
            print()
            print(f"  全部数据类帧（{len(long_frames)} 条）：")
            print_stream_frames(long_frames, 128, None)

    if args.list:
        print_stream_frames(frs, args.min_bits,
                            None if args.limit == 0 else args.limit)

    # 选帧：--frame 指定 > 最长的数据帧 > 最长的一帧
    if args.frame is not None:
        if not (0 <= args.frame < len(frs)):
            print("--frame %d 超出范围（共 %d 帧）" % (args.frame, len(frs)))
            return 1
        pick = frs[args.frame]
    else:
        cand = [f for f in frs if f["bits"] >= 128]
        pick = (max(cand, key=lambda f: f["width_us"]) if cand
                else max(frs, key=lambda f: f["words"]))

    print()
    print("选中帧 %d：t=%.3f us，%d 字，%d bit，%.2f MHz"
          % (pick["idx"], pick["t_us"], pick["words"], pick["bits"],
             pick["clk_mhz"]))
    if pick["flags"] & STREAM_F_TRUNC:
        print("  注意：这一帧被标为截断（环里没找到 CS 上升沿）")
    if pick["mosi"]:
        print("  MOSI %d 字节: %s" % (len(pick["mosi"]), hex_bytes(pick["mosi"])))
        print("       ASCII : %s" % ascii_bytes(pick["mosi"]))
        print("  MISO %d 字节: %s" % (len(pick["miso"]), hex_bytes(pick["miso"])))
        print("       ASCII : %s" % ascii_bytes(pick["miso"]))
    else:
        print("  这一帧里没有任何 SCK 上升沿（纯 CS 噪声或空帧）")

    if args.csv:
        fs = float(st["sample_hz"])
        fr = st["frames"][pick["idx"]]
        sig = unpack_stream_frame(fr)
        t = pick["t_us"] + np.arange(len(sig["cs"])) * 1e6 / fs
        export_csv(None, sig, t, args.csv)

    th = THEMES[args.theme]
    outdir = Path(args.out) if args.out else bin_path.parent / "RPSTREAM_png"

    if args.all:
        sel = [f for f in frs if f["bits"] >= args.min_bits]
        if len(sel) > 60:
            print("  --all 选中 %d 帧，只画前 60 帧；提高 --min-bits 可以收窄"
                  % len(sel))
            sel = sel[:60]
    else:
        sel = [pick]

    if not args.no_plot:
        outdir.mkdir(parents=True, exist_ok=True)
        for f in sel:
            plot_stream_frame(st, f, args, outdir, th)
        print()
        print("图片在 %s" % outdir)
    if args.show:
        plt.show()
    return 0


def main():
    global _trig_off_global

    ap = argparse.ArgumentParser(
        description="解析 RP_Bus_Sniffer 的抓包文件并绘制 SPI 波形")
    ap.add_argument("binfile", help="从 TF 卡拷出来的 RPCAP.BIN 或 RPSTREAM.BIN")
    ap.add_argument("--record", type=int, default=0,
                    help="一次性窗口模式下要详细打印/绘图的记录序号（默认 0）")
    ap.add_argument("--frame", type=int, default=None,
                    help="连续记录模式下要详细分析/绘图的帧号（默认取最长的数据帧）")
    ap.add_argument("--all-long", action="store_true",
                    help="连续记录模式下把全部数据类帧列成表（不只前 40 条）")
    ap.add_argument("--min-bits", type=int, default=128,
                    help="连续记录模式下 --list/--all 只保留 bit 数不少于该值的帧"
                         "（默认 128 = 16 字节；0 = 全部）")
    ap.add_argument("--limit", type=int, default=200,
                    help="--list 最多显示多少帧（默认 200，0 = 不限）")
    ap.add_argument("--all", action="store_true",
                    help="一次性窗口模式：每条记录都出图；"
                         "连续记录模式：每个筛选后的帧都出图")
    ap.add_argument("--pre", type=float, default=1.0,
                    help="缩放窗口：触发点之前多少 us（默认 1）")
    ap.add_argument("--post", type=float, default=2.0,
                    help="缩放窗口：触发点之后多少 us（默认 2）")
    ap.add_argument("--t0", type=float, default=None,
                    help="缩放窗口起点（相对触发的 us，可为负）")
    ap.add_argument("--t1", type=float, default=None,
                    help="缩放窗口终点（相对触发的 us，可为负）")
    ap.add_argument("--list", action="store_true",
                    help="列出所有 SCK 突发（不依赖 CS），供 --burst 选用")
    ap.add_argument("--burst", type=int, default=None,
                    help="把缩放窗对准第 N 段 SCK 突发（配合 --list 使用）")
    ap.add_argument("--out", default=None,
                    help="图片输出目录（默认在 bin 文件旁建 RPCAP_png/）")
    ap.add_argument("--csv", default=None,
                    help="把所选记录的采样点导出成 CSV，方便 MATLAB 读")
    ap.add_argument("--theme", choices=("light", "dark"), default="light")
    ap.add_argument("--show", action="store_true", help="额外弹窗显示")
    ap.add_argument("--no-plot", action="store_true", help="只打印，不出图")
    args = ap.parse_args()

    bin_path = Path(args.binfile)
    if not bin_path.exists():
        print("找不到文件：%s" % bin_path)
        return 1

    fmt = detect_format(bin_path)
    if fmt == "stream":
        return run_stream(args, bin_path)
    if fmt is None:
        print("文件头既不是 'RPSN'（一次性窗口）也不是 'RPSF'（连续记录）：%s"
              % bin_path)
        return 1

    if args.pre < 0 or args.post < 0 or (args.pre == 0 and args.post == 0):
        print("--pre / --post 必须是非负数，且不能同时为 0")
        return 1
    if args.t0 is not None and args.t1 is not None and args.t0 >= args.t1:
        print("--t0 必须小于 --t1")
        return 1

    recs = read_records(bin_path)
    if not recs:
        print("没有解析出任何记录。文件是不是没从 TF 卡拷全？")
        return 1

    print("共 %d 条记录，文件大小 %.1f KB"
          % (len(recs), bin_path.stat().st_size / 1024.0))

    outdir = Path(args.out) if args.out else bin_path.parent / "RPCAP_png"
    if not args.no_plot:
        outdir.mkdir(parents=True, exist_ok=True)

    for i, rec in enumerate(recs):
        sig = unpack_samples(rec["data"])
        t, trig_off = time_axis(rec, len(sig["cs"]))
        _trig_off_global = trig_off

        if args.all or i == args.record:
            frames = decode_frames(rec, sig)
            print_record(rec, sig, frames, i)

            bursts = find_bursts(sig)
            if args.list:
                print("  SCK 突发（不依赖 CS）：")
                print_bursts(sig, t, bursts)
                if args.burst is not None and args.burst >= len(bursts):
                    print("  --burst %d 超出范围（共 %d 段）"
                          % (args.burst, len(bursts)))

            if args.csv and i == args.record:
                export_csv(rec, sig, t, args.csv)
            if not args.no_plot:
                plot_record(rec, sig, t, args, outdir, bursts)

    if not args.no_plot:
        print()
        print("图片在 %s" % outdir)
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())

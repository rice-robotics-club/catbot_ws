#!/usr/bin/env python3
"""
Visual TUI for /actions (Float32MultiArray, 12 joints).

Each leg column has TWO stacked panels:

  ┌ FRONT VIEW (hip abduction) ─────────────────┐
  │  ─╤─           body mount                   │
  │    ╲            hip swings leg left/right     │
  │     o           leg base                      │
  └─────────────────────────────────────────────┘
  ┌ SIDE VIEW (top/bot flexion) ────────────────┐
  │   ─╤─          thigh mount                  │
  │     ╲           top joint swings fwd/back    │
  │      o          knee                         │
  │       ╲         bot joint (chained w/ top)   │
  │        *        foot                         │
  └─────────────────────────────────────────────┘

Joint indices (training config order):
  FL: hip=0  top=1  bot=2      FR: hip=3  top=4  bot=5
  BL: hip=6  top=7  bot=8      BR: hip=9  top=10 bot=11
"""

import math
import subprocess
import sys
# ── ANSI ─────────────────────────────────────────────────────────────────────
CSI    = '\033['
CLEAR  = CSI + '2J' + CSI + 'H'
HOME   = CSI + 'H'
HIDE   = CSI + '?25l'
SHOW   = CSI + '?25h'
BOLD   = CSI + '1m'
DIM    = CSI + '2m'
CYAN   = CSI + '36m'
YELLOW = CSI + '33m'
BLUE   = CSI + '34m'
GREEN  = CSI + '32m'
RED    = CSI + '31m'
RESET  = CSI + '0m'
GRAY   = CSI + '90m'   # dark gray — frame borders

# ── Box-drawing characters ────────────────────────────────────────────────────
_TL, _TR = '┌', '┐'
_BL, _BR = '└', '┘'
_ML, _MR = '├', '┤'
_BH, _BV = '─', '│'

# ── Joint layout ──────────────────────────────────────────────────────────────
LEG_NAMES = ['FL', 'FR', 'BL', 'BR']
HIP_IDX   = [0, 3, 6, 9]
TOP_IDX   = [1, 4, 7, 10]
BOT_IDX   = [2, 5, 8, 11]

# ── Training config ───────────────────────────────────────────────────────────
ACTION_SCALE = 0.25


DEFAULT_DOF_POS: list[float] = [0.0] * 12  # zero offset — target = action × ACTION_SCALE

# URDF joint limits (radians) — used to clamp display angles to stay within canvas
# Order mirrors joint index: hip, top, bot  (same limits for all 4 legs)
_HIP_LIMIT = math.pi / 2        # ±90°  (URDF: ±1.5708)
_TOP_LIMIT = math.pi / 2        # ±90°  (URDF: ±1.5708)
_BOT_LIMIT = math.pi / 6        # ±30°  (URDF: ±0.5236)
# Per-joint clamp bounds in training-config order
_DISPLAY_LIMITS = [
    _HIP_LIMIT, _TOP_LIMIT, _BOT_LIMIT,   # FL
    _HIP_LIMIT, _TOP_LIMIT, _BOT_LIMIT,   # FR
    _HIP_LIMIT, _TOP_LIMIT, _BOT_LIMIT,   # BL
    _HIP_LIMIT, _TOP_LIMIT, _BOT_LIMIT,   # BR
]


def _clamp_display(target: list[float]) -> list[float]:
    """Clamp target angles to URDF joint limits so limbs stay within the canvas."""
    return [max(-lim, min(lim, v)) for v, lim in zip(target, _DISPLAY_LIMITS)]

# ── Braille dot canvas ────────────────────────────────────────────────────────
# Each terminal character encodes a 2-wide × 4-tall grid of dots (U+2800 block).
# Braille dots within a cell are approximately square at typical font sizes,
# so no aspect-ratio adjustment is needed in dot-space — angles are accurate.
#
# Bit layout:
#   col→  0     1
#   row 0  0x01  0x08
#   row 1  0x02  0x10
#   row 2  0x04  0x20
#   row 3  0x40  0x80
_BRAILLE_BITS = [
    [0x01, 0x08],
    [0x02, 0x10],
    [0x04, 0x20],
    [0x40, 0x80],
]
_BRAILLE_BASE = 0x2800

# Panel dimensions in terminal characters
# Sized so ±90° (top/hip) and ±30° (bot) render fully without clipping.
# Worst-case horizontal reach: DOT_CENTER + LIMB_LEN + LIMB_LEN*sin(30°) ≈ 40+20+10 = 70 < 80
# Worst-case upward reach: mount_row - LIMB_LEN*|cos(90°+30°)| = 12 - 10 = 2 (just fits)
# Worst-case downward reach: mount_row + 2*LIMB_LEN = 12 + 40 = 52 < 60
CHAR_W      = 40   # columns per leg panel  (→ 80 dot columns)
CHAR_H_HIP  =  6   # rows for hip panel     (→ 24 dot rows)
CHAR_H_LEG  = 15   # rows for leg panel     (→ 60 dot rows)

# Dot-space geometry (dot coordinates)
DOT_W       = CHAR_W * 2       # 80
DOT_CENTER  = DOT_W // 2       # 40  — central column for mount points
HIP_LIMB    = 20               # dot-length of hip limb
LIMB_LEN    = 20               # dot-length of each leg segment (equal length)

# Mount row positions within each panel (dot-space rows)
_HIP_MOUNT_ROW = 1             # hip panel: near top (1 leaves room for disk radius)
_LEG_MOUNT_ROW = 12            # leg panel: 12 rows from top → room for upswing


class BrailleCanvas:
    """Raster dot canvas that renders to Braille Unicode characters."""

    def __init__(self, char_w: int, char_h: int):
        self.char_w = char_w
        self.char_h = char_h
        self.dw = char_w * 2
        self.dh = char_h * 4
        self.dots = [[False] * self.dw for _ in range(self.dh)]

    def _set(self, r: int, c: int):
        if 0 <= r < self.dh and 0 <= c < self.dw:
            self.dots[r][c] = True

    def line(self, r0: int, c0: int, r1: int, c1: int):
        """Draw a Bresenham line between two dot-space points."""
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 >= r0 else -1
        sc = 1 if c1 >= c0 else -1
        err = dc - dr
        r, c = r0, c0
        while True:
            self._set(r, c)
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr

    def disk(self, r: int, c: int, radius: int = 1):
        """Filled circle marker."""
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr * dr + dc * dc <= radius * radius:
                    self._set(r + dr, c + dc)

    def hbar(self, r: int, c_center: int, half_w: int):
        """Horizontal bar (mount point indicator)."""
        for c in range(c_center - half_w, c_center + half_w + 1):
            self._set(r, c)

    def rows(self) -> list[str]:
        """Encode dot grid as a list of Braille character strings."""
        out = []
        for cr in range(self.char_h):
            row = ''
            for cc in range(self.char_w):
                bits = 0
                for dr in range(4):
                    for dc in range(2):
                        if self.dots[cr * 4 + dr][cc * 2 + dc]:
                            bits |= _BRAILLE_BITS[dr][dc]
                row += chr(_BRAILLE_BASE + bits)
            out.append(row)
        return out


def _limb_end(angle: float, length: int, r0: int, c0: int) -> tuple[int, int]:
    """Endpoint of a limb in dot-space.

    No clamping — the canvas _set() silently drops out-of-bounds dots so the
    line is clipped at the panel edge without distorting the drawn angle.
    """
    r1 = r0 + int(round(length * math.cos(angle)))
    c1 = c0 + int(round(length * math.sin(angle)))
    return r1, c1


def _draw_hip_panel(hip: float) -> list[str]:
    """Front-view Braille canvas: hip abduction swings leg left/right."""
    cv = BrailleCanvas(CHAR_W, CHAR_H_HIP)
    mc = DOT_CENTER
    mr = _HIP_MOUNT_ROW
    cv.hbar(mr, mc, 6)                                   # mount bar
    cv.disk(mr, mc, 2)                                   # mount joint
    r1, c1 = _limb_end(hip, HIP_LIMB, mr, mc)
    cv.line(mr, mc, r1, c1)
    cv.disk(r1, c1, 2)                                   # leg-base joint
    return cv.rows()


def _draw_leg_panel(top: float, bot: float) -> list[str]:
    """Side-view Braille canvas: top/bot flexion.

    Structure:
      [top joint]  ← disk at the mount (top joint IS the mounting point)
           |
      upper limb   (rotated by top angle)
           |
      [bot joint]  ← disk midway through leg
           |
      lower limb   (rotated by top+bot angle, chained)
           |
          foot     ← small terminal dot
    Both limb segments are equal length (LIMB_LEN).
    """
    cv = BrailleCanvas(CHAR_W, CHAR_H_LEG)
    mc = DOT_CENTER
    mr = _LEG_MOUNT_ROW

    # Top joint IS the mount — draw a disk + horizontal bar
    cv.hbar(mr, mc, 6)
    cv.disk(mr, mc, 2)                         # top joint marker

    # Upper limb — from top joint, rotated by top angle
    bot_r, bot_c = _limb_end(top, LIMB_LEN, mr, mc)
    cv.line(mr, mc, bot_r, bot_c)
    cv.disk(bot_r, bot_c, 2)                   # bot joint marker (midway)

    # Lower limb — from bot joint, rotated by (top + bot) angle
    foot_r, foot_c = _limb_end(top + bot, LIMB_LEN, bot_r, bot_c)
    cv.line(bot_r, bot_c, foot_r, foot_c)
    cv.disk(foot_r, foot_c, 2)                 # foot

    return cv.rows()


def _to_target(raw: list[float]) -> list[float]:
    """Convert raw policy actions → target joint angles (radians)."""
    return [r * ACTION_SCALE + d for r, d in zip(raw, DEFAULT_DOF_POS)]


def _frame_col(
    name: str,
    hip_rows: list[str],
    leg_rows: list[str],
    val_tuples: list[tuple],  # (act_str, act_vislen, tgt_str, tgt_vislen) per joint
) -> list[str]:
    """Wrap one leg's panels and value table in a gray box."""
    W = CHAR_W
    G = GRAY
    R = RESET

    def _border(lc: str, rc: str, label: str = '', label_vis: int = 0) -> str:
        rem = W - label_vis
        lp = rem // 2
        rp = rem - lp
        return f'{G}{lc}{_BH * lp}{R}{label}{G}{_BH * rp}{rc}{R}'

    lines = []

    # top with leg name
    name_vis = f' {name} '
    lines.append(_border(_TL, _TR, f'{BOLD}{YELLOW}{name_vis}{RESET}', len(name_vis)))

    # hip section label
    lbl_h = '(front: hip)'
    lines.append(f'{G}{_BV}{R}{DIM}{lbl_h:^{W}}{RESET}{G}{_BV}{R}')

    # hip panel rows
    for row in hip_rows:
        lines.append(f'{G}{_BV}{R}{row}{G}{_BV}{R}')

    # mid divider with side label
    mid_vis = ' (side: top/bot) '
    lines.append(_border(_ML, _MR, f'{DIM}{mid_vis}{RESET}', len(mid_vis)))

    # leg panel rows
    for row in leg_rows:
        lines.append(f'{G}{_BV}{R}{row}{G}{_BV}{R}')

    # value section divider
    lines.append(f'{G}{_ML}{_BH * W}{_MR}{R}')

    # act / tgt rows — centered within the box
    for act_str, act_vis, tgt_str, tgt_vis in val_tuples:
        al = (W - act_vis) // 2;  ar = W - act_vis - al
        tl = (W - tgt_vis) // 2;  tr = W - tgt_vis - tl
        lines.append(f'{G}{_BV}{R}{" " * al}{act_str}{" " * ar}{G}{_BV}{R}')
        lines.append(f'{G}{_BV}{R}{" " * tl}{tgt_str}{" " * tr}{G}{_BV}{R}')

    # bottom border
    lines.append(f'{G}{_BL}{_BH * W}{_BR}{R}')

    return lines


def render(raw_actions: list[float]) -> str:
    out: list[str] = []
    target = _to_target(raw_actions)
    display = _clamp_display(target)

    out.append('')
    out.append(f'  {BOLD}{CYAN}Now Monitoring: Joint Angles   ------  (/actions)  ------  {RESET}  '
               f'{DIM}(Ctrl+C to quit){RESET}')
    out.append('')

    hip_panels = [_draw_hip_panel(display[HIP_IDX[i]]) for i in range(4)]
    leg_panels = [_draw_leg_panel(display[TOP_IDX[i]], display[BOT_IDX[i]]) for i in range(4)]

    columns = []
    for i in range(4):
        val_tuples = []
        for label, idx_list in (('hip', HIP_IDX), ('top', TOP_IDX), ('bot', BOT_IDX)):
            j = idx_list[i]
            act     = raw_actions[j]
            tgt_raw = target[j]
            tgt     = display[j]
            clipped = abs(tgt_raw) > abs(tgt) + 1e-6
            col_a   = GREEN if act >= 0 else RED
            col_t   = YELLOW if clipped else (GREEN if tgt >= 0 else RED)
            flag    = '!' if clipped else ' '
            act_plain = f'{label} act {act:+6.2f}'
            tgt_plain = f'    tgt {tgt:+.3f}r{flag}'
            act_str   = f'{label} act {col_a}{act:+6.2f}{RESET}'
            tgt_str   = f'    tgt {col_t}{tgt:+.3f}r{flag}{RESET}'
            val_tuples.append((act_str, len(act_plain), tgt_str, len(tgt_plain)))
        columns.append(_frame_col(LEG_NAMES[i], hip_panels[i], leg_panels[i], val_tuples))

    for row_idx in range(len(columns[0])):
        out.append(' '.join(col[row_idx] for col in columns))

    out.append('')
    return '\n'.join(out)


def main():
    cmd = ['ros2', 'topic', 'echo', '/actions',
           'std_msgs/msg/Float32MultiArray']
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        print('ERROR: ros2 not found. Source the workspace first.')
        sys.exit(1)

    angles: list[float] = [0.0] * 12
    pending: list[float] = []
    in_data = False

    # EOS = erase from cursor to end of screen; used after HOME so stale lines
    # from a taller previous frame are wiped before the new frame is painted.
    EOS = CSI + 'J'

    print(HIDE, end='', flush=True)
    print(CLEAR, end='', flush=True)
    print(HOME + EOS + render(angles), end='', flush=True)

    try:
        for raw in proc.stdout:
            line = raw.rstrip('\n')

            if line.startswith('data:'):
                in_data = True
                pending = []
                continue

            if in_data:
                stripped = line.strip()
                if stripped.startswith('- '):
                    try:
                        pending.append(float(stripped[2:]))
                    except ValueError:
                        pass
                elif stripped == '---':
                    if len(pending) == 12:
                        angles = pending[:]
                    in_data = False
                    pending = []
                    print(HOME + EOS + render(angles), end='', flush=True)
                else:
                    in_data = False

    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        print(SHOW, end='', flush=True)
        print()


if __name__ == '__main__':
    main()


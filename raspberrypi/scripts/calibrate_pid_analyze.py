#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calibrate（圆环视觉对位）控制环曲线分析 + 调参建议。

calibrate 业务每次运行都会把逐帧数据写到 /tmp/calibrate_last.csv
（vision_node/businesses/calibrate.py，环境变量 CALIBRATE_CSV 可改路径/关闭）。
本脚本离线读这份 CSV，告诉你时间都花在哪、有没有来回抖、每步实际走了多少，
并给出下一版 calibrate.json 的参数建议。不会动车。

用法：
    python3 scripts/calibrate_pid_analyze.py
    python3 scripts/calibrate_pid_analyze.py /tmp/calibrate_last.csv
    python3 scripts/calibrate_pid_analyze.py --apply      # 把建议写回 calibrate.json（先备份）

判断依据（和 calibrate.py 的控制结构对应）：
  · 每步实际位移 ≈ PID 输出速度 v ÷ F407 位置环 Kp，所以外环等效增益
    ≈ ctrl_kp_xy / chassis_pos_kp。当前 0.56/1.74≈0.32，一步只能消掉 32% 误差，
    误差要 8~10 步才收敛——这是"慢"的主因，不是积分不够。
  · 轮速是整数 RPM（1 RPM≈4.08mm/s）。F407 位置环输出 <0.5RPM 会被舍成 0，
    <1RPM 会跳到 4.08mm/s。所以末端 1~2mm 只能靠"给一脚 4mm/s"完成，
    最小速度、死区、积分三者配不好就会原地来回抖。
  · 检测器要求连续 stable_frame_count(3) 帧抖动 ≤1.5px(≈0.9mm) 才给控制量，
    车一动起来常常就不 stable → 控制停更、车继续冲向上一个领先目标再停。
    所以"大步 + 少步数"比"小步慢慢磨"快得多。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import statistics
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_CSV = Path('/tmp/calibrate_last.csv')
DEFAULT_CFG = HERE.parent / 'vision' / 'config' / 'calibrate.json'

# 轮半径来自 RobotConfig.h：WHEEL_RADIUS_M=0.039
RPM_MM_S = 2.0 * math.pi * 0.039 / 60.0 * 1000.0     # 4.084 mm/s


def load_rows(path: Path):
    rows = []
    with path.open(newline='', encoding='utf-8') as f:
        for raw in csv.DictReader(f):
            row = {}
            for k, v in raw.items():
                if v is None or v == '':
                    row[k] = None
                else:
                    try:
                        row[k] = float(v)
                    except ValueError:
                        row[k] = v
            rows.append(row)
    return rows


def fnum(v, default=0.0):
    return default if v is None or isinstance(v, str) else float(v)


def stdev(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def segments(rows):
    """把 moving 0→1→0 切成一段段，返回每段的起止行号和指标。"""
    out = []
    start = None
    for i, r in enumerate(rows):
        m = int(fnum(r.get('moving')))
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(rows) - 1))
    return out


def usable_rows(rows):
    return [r for r in rows if int(fnum(r.get('usable'))) and r.get('fx') is not None]


def analyze(rows):
    frames = [r for r in rows if r.get('t') is not None]
    if not frames:
        return None
    t_end = fnum(frames[-1]['t'])
    intervals = []
    for a, b in zip(frames, frames[1:]):
        dt = fnum(b['t']) - fnum(a['t'])
        if 0 < dt < 2.0:
            intervals.append(dt)
    frame_dt = statistics.median(intervals) if intervals else 0.0

    usable = usable_rows(rows)
    blind_time = 0.0
    for a, b in zip(frames, frames[1:]):
        if not int(fnum(a.get('usable'))):
            blind_time += fnum(b['t']) - fnum(a['t'])
    moving_time = 0.0
    for a, b in zip(frames, frames[1:]):
        if int(fnum(a.get('moving'))):
            moving_time += fnum(b['t']) - fnum(a['t'])

    # 静止时的视觉噪声（车不动，raw 的离散度≈测量噪声）
    still = [r for r in rows if int(fnum(r.get('usable')))
             and not int(fnum(r.get('moving'))) and r.get('cx') is not None]
    noise_x = stdev([fnum(r['cx']) for r in still])
    noise_y = stdev([fnum(r['cy']) for r in still])
    noise_yaw = stdev([fnum(r['cyaw']) for r in still])

    def first_last(key):
        vals = [fnum(r[key]) for r in usable if r.get(key) is not None]
        return (vals[0], vals[-1]) if vals else (0.0, 0.0)

    res = {
        't_end': t_end,
        'frames': len(frames),
        'frame_hz': 1.0 / frame_dt if frame_dt > 0 else 0.0,
        'usable_frac': len(usable) / max(1, len(frames)),
        'blind_time': blind_time,
        'moving_time': moving_time,
        'event': frames[-1].get('event'),
        'noise': (noise_x, noise_y, noise_yaw),
        'rx0': first_last('cx'), 'rx1': first_last('cx'),
        'ry0': first_last('cy'), 'ry1': first_last('cy'),
        'ra0': first_last('cyaw'), 'ra1': first_last('cyaw'),
        'fx0': first_last('fx'), 'fx1': first_last('fx'),
        'fy0': first_last('fy'), 'fy1': first_last('fy'),
        'fa0': first_last('fyaw'), 'fa1': first_last('fyaw'),
    }
    segs = []
    for a, b in segments(rows):
        r0, r1 = rows[a], rows[b]
        e0 = max(abs(fnum(r0.get('fx'))), abs(fnum(r0.get('fy'))))
        e1 = max(abs(fnum(r1.get('fx'))), abs(fnum(r1.get('fy'))))
        d0 = (fnum(r0.get('odom_x')), fnum(r0.get('odom_y')))
        d1 = (fnum(r1.get('odom_x')), fnum(r1.get('odom_y')))
        moved = math.hypot(d1[0] - d0[0], d1[1] - d0[1]) * 1000.0
        segs.append({
            't0': fnum(r0['t']), 't1': fnum(r1['t']),
            'dur': fnum(r1['t']) - fnum(r0['t']),
            'e0': e0, 'e1': e1, 'moved_mm': moved,
            'gain': ((e0 - e1) / e0) if e0 > 1e-6 else 0.0,
        })
    res['segs'] = segs

    # 来回抖：滤波后误差的符号翻转（幅度要大于噪声才认）
    def reversals(series, thresh):
        seq = [v for v in series if abs(v) > thresh]
        n = 0
        for i in range(1, len(seq) - 1):
            if (seq[i] - seq[i - 1]) * (seq[i + 1] - seq[i]) < 0:
                n += 1
        return n

    fx = [fnum(r['fx']) for r in usable]
    fy = [fnum(r['fy']) for r in usable]
    fa = [fnum(r['fyaw']) for r in usable]
    res['rev_x'] = reversals(fx, max(0.8, noise_x))
    res['rev_y'] = reversals(fy, max(0.8, noise_y))
    res['rev_yaw'] = reversals(fa, max(0.4, noise_yaw))
    res['max_fx'] = max((abs(v) for v in fx), default=0.0)
    res['max_fy'] = max((abs(v) for v in fy), default=0.0)
    res['int_max'] = max(
        (max(abs(fnum(r.get('ix'))), abs(fnum(r.get('iy')))) for r in rows),
        default=0.0)
    return res


def load_config(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}


def recommend(res, cfg):
    pos_kp = abs(fnum(cfg.get('chassis_pos_kp'), 1.74)) or 1.74
    head_kp = abs(fnum(cfg.get('chassis_heading_kp'), 2.15)) or 2.15
    noise_x, noise_y, noise_yaw = res['noise']
    noise_xy = max(noise_x, noise_y)

    # 外环等效增益 ≈ kp/chassis_kp；0.8 左右一步消掉 80%，接近临界阻尼且不超调
    kp_xy = round(0.80 * pos_kp, 3)
    kp_yaw = round(0.80 * head_kp, 3)

    # 末端：测量噪声以下的死区没有意义，反而会让积分去追噪声
    dead_xy = min(1.5, max(1.0, 2.0 * noise_xy))
    dead_yaw = min(1.0, max(0.5, 2.0 * noise_yaw))
    filt = 0.08 if noise_xy > 0.8 else 0.05

    rec = {
        'ctrl_kp_xy': kp_xy,
        'ctrl_ki_xy': 0.0,
        'ctrl_kd_xy': 0.0,
        'ctrl_kp_yaw': kp_yaw,
        'ctrl_ki_yaw': 0.0,
        'ctrl_kd_yaw': 0.0,
        'min_xy_speed_m_s': round(RPM_MM_S / 1000.0, 4),   # 1 RPM
        'min_yaw_speed_deg_s': 1.0,
        'ctrl_deadband_xy_mm': round(dead_xy, 2),
        'ctrl_deadband_yaw_deg': round(dead_yaw, 2),
        'ctrl_filter_tau': filt,
        'ctrl_deriv_tau': 0.05,
        # 停车阈值别低到轮速量化里：v<2.04mm/s 会舍成 0，2.1mm/s 就跳 4.08mm/s。
        # 停在 1.8mm 左右能"真的停住"（仍在 2mm 验收内），再小只会反复被
        # 1RPM 的台阶踢过头，表现就是末端来回磨。
        'ctrl_stop_xy_mm': round(max(dead_xy, 1.8), 2),
        'ctrl_stop_yaw_deg': round(max(dead_yaw, 0.6), 2),
        'ctrl_settle_s': 0.15,
        'max_lead_xy_mm': 100.0,
        'max_lead_yaw_deg': 10.0,
        'max_xy_speed_m_s': max(0.10, fnum(cfg.get('max_xy_speed_m_s'), 0.07)),
        'max_yaw_speed_deg_s': max(20.0, fnum(cfg.get('max_yaw_speed_deg_s'), 19.0)),
        'converge_hold_frames': 6.0,
    }
    return rec


def fmt(v):
    if isinstance(v, float):
        return f'{v:.3g}'
    return str(v)


def main():
    ap = argparse.ArgumentParser(description='calibrate 控制环曲线分析')
    ap.add_argument('csv', nargs='?', default=str(DEFAULT_CSV))
    ap.add_argument('--config', default=str(DEFAULT_CFG),
                    help='calibrate.json（用来读当前参数和 F407 Kp）')
    ap.add_argument('--apply', action='store_true',
                    help='把建议写回 --config（先备份 .bak）')
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.is_file():
        print(f'找不到 {path}。先跑一次 calibrate（vision_node 会自动写这份 CSV）。')
        return 1
    rows = load_rows(path)
    res = analyze(rows)
    if res is None:
        print('CSV 里没有数据行。')
        return 1
    cfg = load_config(Path(args.config))
    rec = recommend(res, cfg)

    print('=' * 68)
    print(f'calibrate 曲线分析  {path}')
    print('=' * 68)
    print(f'总耗时       {res["t_end"]:.2f}s   结束状态 {res["event"]}')
    print(f'帧率         {res["frame_hz"]:.1f} Hz（{res["frames"]} 帧）')
    print(f'可用帧比例   {res["usable_frac"] * 100:.0f}%'
          f'（检测 stable 的帧才算可用）')
    print(f'看不见时间   {res["blind_time"]:.2f}s'
          f'（{res["blind_time"] / max(res["t_end"], 1e-6) * 100:.0f}%）')
    print(f'移动时间     {res["moving_time"]:.2f}s')
    print(f'视觉噪声     x {res["noise"][0]:.2f}mm  y {res["noise"][1]:.2f}mm  '
          f'yaw {res["noise"][2]:.2f}°')
    print(f'初始误差     x {res["rx0"][0]:+.2f}  y {res["ry0"][0]:+.2f}mm  '
          f'yaw {res["ra0"][0]:+.2f}°')
    print(f'最终误差     x {res["fx1"][1]:+.2f}  y {res["fy1"][1]:+.2f}mm  '
          f'yaw {res["fa1"][1]:+.2f}°（滤波后）')
    print(f'误差反向     x {res["rev_x"]} 次  y {res["rev_y"]} 次  '
          f'yaw {res["rev_yaw"]} 次   ← 这就是"抖"')
    print(f'积分峰值     {res["int_max"]:.2f}（越大越容易回弹/超调）')

    print('-' * 68)
    print('每一次"起步→停车"：')
    for i, s in enumerate(res['segs'], 1):
        print(f'  #{i} {s["t0"]:5.2f}~{s["t1"]:5.2f}s  用时 {s["dur"]:4.2f}s  '
              f'误差 {s["e0"]:5.1f}→{s["e1"]:4.1f}mm  实走 {s["moved_mm"]:5.1f}mm  '
              f'单步消误差 {s["gain"] * 100:4.0f}%')
    if res['segs']:
        avg_gain = statistics.mean(s['gain'] for s in res['segs'])
        print(f'  平均单步增益 {avg_gain * 100:.0f}%  '
              f'(<50% 说明 ctrl_kp 太小/领先量被限幅，步数会很多)')
    else:
        avg_gain = 0.0

    print('-' * 68)
    print('诊断：')
    pos_kp = abs(fnum(cfg.get('chassis_pos_kp'), 1.74)) or 1.74
    cur_kp = abs(fnum(cfg.get('ctrl_kp_xy'), 0.56))
    cur_gain = cur_kp / pos_kp
    print(f'  · 外环等效增益 ≈ ctrl_kp_xy/chassis_pos_kp = {cur_kp:.2f}/{pos_kp:.2f}'
          f' = {cur_gain:.2f}')
    if cur_gain < 0.55:
        print('    → 太小：每步只消掉约 {:.0f}%，要靠很多步+积分硬磨，慢的主因。'
              .format(cur_gain * 100))
    if res['rev_x'] + res['rev_y'] + res['rev_yaw'] >= 3:
        print('    → 有来回反转：多半是 ki 太大 / 最小速度在死区边上反复给油 / '
              '死区比噪声还小。先降 ki、把死区抬到噪声以上。')
    if res['blind_time'] > 0.35 * res['t_end']:
        print('    → 超过 1/3 时间检测不 stable：车一动就丢控制量。'
              '加大 max_lead_xy（一步走完）比加 PID 更有效。')
    if res['max_fx'] > 5 and avg_gain > 0.6:
        print('    → 单步增益不小但仍有大误差，检查 max_lead_xy 是否把每步卡在几十 mm。')

    print('-' * 68)
    print('建议写入 calibrate.json：')
    for k, v in rec.items():
        old = cfg.get(k, '<无>')
        flag = '' if fmt(old) == fmt(v) else '   ← 改'
        print(f'  {k:26s} {fmt(old):>10s} → {fmt(v):>10s}{flag}')
    print('  说明：ki 全部归零——这个环的稳态误差来自轮速量化，不是积分不够；'
          '积分只会让末端来回抖。')

    if args.apply:
        cfg_path = Path(args.config)
        backup = cfg_path.with_suffix(cfg_path.suffix + '.bak')
        shutil.copy2(cfg_path, backup)
        for k, v in rec.items():
            cfg[k] = v
        cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2),
                            encoding='utf-8')
        print(f'\n已写回 {cfg_path}（备份 {backup.name}）。重新跑一次 calibrate 再看曲线。')
    else:
        print('\n加 --apply 可直接写回 calibrate.json。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

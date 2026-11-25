#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台 yaw 波形分析 + PID 调参建议。

数据来源（两份都可以，缺一份就只分析另一份）：

1) /tmp/gimbal_wave.csv —— scripts/gimbal_wave_logger.py 录的整段动作
   （/gimbal_state + /cmd_gimbal + /cmd_gimbal_vel + /motion_state）。
   用来看"位置模式"的大角度移动：超调、稳定时间、来回摆、饱和。
   对应 F407 GimbalTask 的位置环 PID → 上位机 PID 页 gim_yaw_kp / gim_yaw_kd。

2) /tmp/track_last.csv —— color_pick 跟踪时 gimbal_track_pick.py 自动写的
   逐帧跟踪数据（像素误差 ex、PID 输出的 yaw 速度）。只在原料区抓取时生成。
   对应 vision/config/track_pid.json → kp_yaw / kd_yaw / deriv_tau / deadband。

用法：
    python3 raspberrypi/scripts/gimbal_pid_analyze.py
    python3 raspberrypi/scripts/gimbal_pid_analyze.py --wave ... --track ...
    python3 raspberrypi/scripts/gimbal_pid_analyze.py --apply   # 写回建议值(先备份)

判断依据：
  · 位置环是 200Hz 纯 P+微弱 D（gim_yaw_kp 输出 °/s，gim_yaw_kd 给阻尼）。
    换重夹爪后 yaw 惯量变大：同样的 kp 会更容易在到位时冲过头/来回摆，
    典型改法是降 kp、提 kd，或者压一点最大速度。
  · 视觉跟踪是纯速度环（track_pid.json），像素噪声直接乘 kp 变成速度指令。
    yaw 高频抖 = kp 偏大或微分在放大噪声；跟得慢 = kp 偏小/滤波太大。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
RP = HERE.parent
DEFAULT_WAVE = Path('/tmp/gimbal_wave.csv')
DEFAULT_TRACK = Path('/tmp/track_last.csv')
DEFAULT_TUNE = RP / '上位机' / 'tune_profile.json'
DEFAULT_TRACK_CFG = RP / 'vision' / 'config' / 'track_pid.json'


# ---------------- 基础工具 ----------------

def load(path: Path):
    with path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    return rows


def num(row, key):
    v = row.get(key)
    if v is None or v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else 0.0


def stdev(xs):
    xs = [x for x in xs if x is not None]
    return statistics.pstdev(xs) if len(xs) > 1 else 0.0


def moving_average(xs, n):
    out = []
    buf = []
    for x in xs:
        buf.append(x)
        if len(buf) > n:
            buf.pop(0)
        out.append(sum(buf) / len(buf))
    return out


def reversals(xs, threshold):
    """符号翻转次数（幅度超过 threshold 才认）。"""
    n = 0
    prev_sign = 0
    peak = 0.0
    for x in xs:
        if abs(x) > abs(peak):
            peak = x
        if abs(x) < threshold:
            continue
        s = 1 if x > 0 else -1
        if prev_sign and s != prev_sign:
            n += 1
        prev_sign = s
    return n


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:  # noqa: BLE001
        return {}


# ---------------- 位置模式（大角度移动） ----------------

def analyze_steps(rows, tol_deg=2.0, min_step_deg=3.0):
    """把 /cmd_gimbal 的 yaw 目标变化切成一段段，逐段量阶跃响应。"""
    yaw = [num(r, 'yaw') for r in rows]
    tt = [num(r, 't') for r in rows]
    cmd = [num(r, 'cmd_yaw') for r in rows]
    age = [num(r, 'cmd_age_ms') for r in rows]

    events = []
    prev = None
    for i, c in enumerate(cmd):
        if c is None:
            continue
        # 只有"刚下发"的新目标才算（cmd_age 很小）；否则长时间保持的旧值
        # 也会被当成一次新阶跃。
        fresh = age[i] is not None and age[i] < 300.0
        if fresh and (prev is None or abs(c - prev) > 0.5):
            events.append(i)
        prev = c

    steps = []
    for k, i0 in enumerate(events):
        i1 = events[k + 1] if k + 1 < len(events) else len(rows) - 1
        # 位置目标超过 1s 没再下发 = 已经过了这一段的"位置模式"窗口；
        # 后面即使 cmd_yaw 没变，云台也可能被 color_pick 的速度环带走，
        # 不能算进这次阶跃响应里。
        stale = [i for i in range(i0, i1 + 1)
                 if age[i] is not None and age[i] > 1000.0]
        if stale:
            i1 = stale[0] - 1
        target = cmd[i0]
        start = yaw[i0]
        if target is None or start is None or i1 <= i0:
            continue
        step = target - start
        if abs(step) < min_step_deg:
            continue
        t0 = tt[i0]
        pts = [(tt[i], yaw[i]) for i in range(i0, i1 + 1)
               if tt[i] is not None and yaw[i] is not None]
        seg_t = [p[0] for p in pts]
        seg_y = [p[1] for p in pts]
        if len(seg_y) < 5:
            continue

        # 稳定时间：最后一次 |误差| > tol
        last_bad = None
        for j, (ti, yi) in enumerate(zip(seg_t, seg_y)):
            if abs(yi - target) > tol_deg:
                last_bad = j
        settled = last_bad is None or last_bad < len(seg_y) - 1
        settle_t = (seg_t[last_bad] - t0) if last_bad is not None else 0.0

        # 上升时间（10%→90%）
        rise_t = None
        reached = False
        for j, (ti, yi) in enumerate(zip(seg_t, seg_y)):
            prog = (yi - start) / step if abs(step) > 1e-6 else 1.0
            if not reached and prog >= 0.9:
                rise_t = ti - t0
                reached = True

        # 超调（沿运动方向越过目标的最大量）
        if step > 0:
            over = max(0.0, max(seg_y) - target)
        else:
            over = max(0.0, target - min(seg_y))
        over_pct = over / abs(step) * 100.0 if abs(step) > 1e-6 else 0.0

        # 稳态误差：段尾 0.3s 的均值
        dt_med = ((seg_t[-1] - seg_t[0]) / max(1, len(seg_t) - 1))
        tail_n = max(1, min(len(seg_y),
                            int(0.3 / dt_med) if dt_med > 1e-6 else 1))
        ss_err = mean(seg_y[-tail_n:]) - target

        # 实际 yaw 速度 + 反向次数（只看越过 50% 之后，排除起步加速）
        rate = []
        for a, b in zip(zip(seg_t, seg_y), list(zip(seg_t, seg_y))[1:]):
            dt = b[0] - a[0]
            rate.append((b[1] - a[1]) / dt if dt > 1e-6 else 0.0)
        # 找第一次越过 50% 的下标
        half = 0
        for j, (ti, yi) in enumerate(zip(seg_t, seg_y)):
            prog = (yi - start) / step if abs(step) > 1e-6 else 1.0
            if prog >= 0.5:
                half = j
                break
        rev = reversals(rate[half:], 10.0)

        steps.append({
            't0': t0, 'dur': seg_t[-1] - t0,
            'start': start, 'target': target, 'step': step,
            'rise_t': rise_t, 'settle_t': settle_t, 'settled': settled,
            'over': over, 'over_pct': over_pct, 'ss_err': ss_err,
            'reversals': rev,
        })
    return steps


def analyze_velocity(rows, stale_ms=200.0):
    """视觉跟踪段：/cmd_gimbal_vel 非零时，比较指令速度和 yaw 实际速度。"""
    tt = [num(r, 't') for r in rows]
    yaw = [num(r, 'yaw') for r in rows]
    vel = [num(r, 'vel_yaw') for r in rows]
    age = [num(r, 'vel_age_ms') for r in rows]

    segments = []
    cur = []
    for i in range(len(rows)):
        fresh = (age[i] is not None and age[i] <= stale_ms
                 and vel[i] is not None and abs(vel[i]) > 1.0
                 and tt[i] is not None and yaw[i] is not None)
        if fresh:
            cur.append(i)
        elif cur:
            segments.append((cur[0], cur[-1]))
            cur = []
    if cur:
        segments.append((cur[0], cur[-1]))

    out = []
    for i0, i1 in segments:
        if i1 - i0 < 10:
            continue
        seg_t = tt[i0:i1 + 1]
        seg_y = yaw[i0:i1 + 1]
        seg_v = vel[i0:i1 + 1]

        # 实际速度（差分 + 50ms 平滑）
        act = []
        for a, b in zip(zip(seg_t, seg_y), list(zip(seg_t, seg_y))[1:]):
            dt = b[0] - a[0]
            act.append((b[1] - a[1]) / dt if dt > 1e-6 else 0.0)
        act = moving_average(act, 5)
        cmd = seg_v[1:]

        # 滞后：在 ±0.1s 内找与指令相关性最好的延迟
        best_lag, best_corr = 0, -2.0
        n = min(len(act), len(cmd))
        for lag in range(-5, 6):
            pairs = []
            for j in range(n):
                k = j + lag
                if 0 <= k < n:
                    pairs.append((act[j], cmd[k]))
            if len(pairs) < 10:
                continue
            ax = [p[0] for p in pairs]
            bx = [p[1] for p in pairs]
            mx, my = mean(ax), mean(bx)
            numr = sum((a - mx) * (b - my) for a, b in pairs)
            den = math.sqrt(sum((a - mx) ** 2 for a in ax) *
                            sum((b - my) ** 2 for b in bx))
            corr = numr / den if den > 1e-9 else 0.0
            if corr > best_corr:
                best_corr, best_lag = corr, lag
        dt_med = statistics.median([b - a for a, b in zip(seg_t, seg_t[1:])]) \
            if len(seg_t) > 1 else 0.01

        err = [a - b for a, b in zip(act, cmd)]
        jitter = stdev([a - b for a, b in zip(act, moving_average(act, 9))])
        out.append({
            't0': seg_t[0], 'dur': seg_t[-1] - seg_t[0],
            'cmd_mean': mean(cmd), 'act_mean': mean(act),
            'mae': mean([abs(e) for e in err]), 'corr': best_corr,
            'lag_ms': best_lag * dt_med * 1000.0,
            'jitter': jitter, 'reversals': reversals(act, 5.0),
        })
    return out


# ---------------- 视觉跟踪（track_last.csv） ----------------

def analyze_track(rows):
    rows = [r for r in rows if (num(r, 'found') or 0) >= 1
            and str(r.get('phase', '')) == 'track'
            and num(r, 'ex') is not None and num(r, 'yaw_vel') is not None
            and num(r, 'gimbal_yaw') is not None and num(r, 't') is not None]
    if len(rows) < 10:
        return None
    ex = [num(r, 'ex') for r in rows]
    ey = [num(r, 'ey') for r in rows]
    yv = [num(r, 'yaw_vel') for r in rows]
    gy = [num(r, 'gimbal_yaw') for r in rows]
    tt = [num(r, 't') for r in rows]

    # 实际 yaw 速度
    act = []
    for a, b in zip(zip(tt, gy), list(zip(tt, gy))[1:]):
        dt = b[0] - a[0]
        act.append((b[1] - a[1]) / dt if dt > 1e-6 else 0.0)

    # 有效增益：yaw_vel 对 ex 的回归斜率（含 I/D，应该接近 kp_yaw）
    mx, my = mean(ex), mean(yv)
    den = sum((x - mx) ** 2 for x in ex)
    gain = sum((x - mx) * (y - my) for x, y in zip(ex, yv)) / den \
        if den > 1e-9 else 0.0

    ex_noise = stdev([x - a for x, a in zip(ex, moving_average(ex, 5))])
    yv_jitter = stdev([y - a for y, a in zip(yv, moving_average(yv, 5))])
    yv_rev = reversals(yv, max(5.0, 2.0 * yv_jitter))

    # yaw_vel 与实际速度的滞后/相关
    n = min(len(act), len(yv) - 1)
    best_lag, best_corr = 0, -2.0
    dt_med = statistics.median([b - a for a, b in zip(tt, tt[1:])]) if len(tt) > 1 else 0.01
    for lag in range(-5, 6):
        pairs = []
        for j in range(n):
            k = j + lag
            if 0 <= k < len(act):
                pairs.append((act[j], yv[k]))
        if len(pairs) < 10:
            continue
        ax = [p[0] for p in pairs]
        bx = [p[1] for p in pairs]
        mx2, my2 = mean(ax), mean(bx)
        numr = sum((a - mx2) * (b - my2) for a, b in pairs)
        dd = math.sqrt(sum((a - mx2) ** 2 for a in ax) *
                       sum((b - my2) ** 2 for b in bx))
        corr = numr / dd if dd > 1e-9 else 0.0
        if corr > best_corr:
            best_corr, best_lag = corr, lag

    return {
        'n': len(rows), 'dur': tt[-1] - tt[0] if tt else 0.0,
        'gain': gain, 'ex_noise': ex_noise, 'ey_noise': stdev(
            [x - a for x, a in zip(ey, moving_average(ey, 5))]),
        'yv_jitter': yv_jitter, 'yv_reversals': yv_rev,
        'corr': best_corr, 'lag_ms': best_lag * dt_med * 1000.0,
        'sat_frac': sum(1 for y in yv if abs(y) > 0.95 * 780.0) / len(yv),
    }


# ---------------- 报告与建议 ----------------

def report_steps(steps):
    if not steps:
        print('（没有可分析的位置阶跃：/cmd_gimbal 的 yaw 没变化或没录到）')
        return
    print('位置模式 yaw 阶跃：')
    print('  #  时间     起点→目标      用时   上升90% 稳定   超调      稳态误差 反向')
    for i, s in enumerate(steps, 1):
        rise = '  -  ' if s['rise_t'] is None else f"{s['rise_t']:5.2f}s"
        settle = f"{s['settle_t']:5.2f}s" if s['settled'] else ' 未稳  '
        print(f"  {i:<2} {s['t0']:6.1f}s {s['start']:+7.1f}→{s['target']:+7.1f}° "
              f"{s['dur']:5.2f}s {rise} {settle} "
              f"{s['over']:+6.1f}°({s['over_pct']:4.1f}%) "
              f"{s['ss_err']:+6.2f}°  {s['reversals']}")
    big = [s for s in steps if abs(s['step']) > 30]
    if big:
        print(f"  大角度平均超调 {mean([s['over_pct'] for s in big]):.1f}%  "
              f"平均稳定 {mean([s['settle_t'] for s in big]):.2f}s  "
              f"平均反向 {mean([s['reversals'] for s in big]):.1f} 次")


def report_vel(segs):
    if not segs:
        print('（没有视觉跟踪的速度段：color_pick 期间 /cmd_gimbal_vel 没录到）')
        return
    print('视觉跟踪速度段：')
    for i, s in enumerate(segs, 1):
        print(f"  #{i} {s['t0']:6.1f}s {s['dur']:4.1f}s  指令 {s['cmd_mean']:+7.1f}°/s "
              f"实际 {s['act_mean']:+7.1f}°/s  平均误差 {s['mae']:6.1f}°/s  "
              f"滞后 {s['lag_ms']:+5.0f}ms  相关 {s['corr']:.2f}  "
              f"抖动 {s['jitter']:5.1f}°/s  反向 {s['reversals']}")


def report_track(t):
    if t is None:
        print('（没有 track_last.csv 或数据太少：color_pick 跟踪段没录到）')
        return
    print(f"视觉跟踪（{t['dur']:.1f}s / {t['n']} 帧）：")
    print(f"  yaw_vel/ex 有效增益 {t['gain']:+.3f}   像素噪声 ex {t['ex_noise']:.2f}px "
          f"ey {t['ey_noise']:.2f}px")
    print(f"  yaw_vel 抖动 {t['yv_jitter']:.1f}°/s   反向 {t['yv_reversals']} 次   "
          f"与实际速度滞后 {t['lag_ms']:+.0f}ms  相关 {t['corr']:.2f}   "
          f"饱和占比 {t['sat_frac'] * 100:.0f}%")


def suggest(steps, vels, track, tune, tcfg):
    tune = dict(tune)
    tcfg = dict(tcfg)
    notes = []

    big = [s for s in steps if abs(s['step']) > 30]
    over = mean([s['over_pct'] for s in big]) if big else 0.0
    rev = mean([s['reversals'] for s in big]) if big else 0.0
    settle = mean([s['settle_t'] for s in big]) if big else 0.0

    kp = float(tune.get('gim_yaw_kp', 10.0))
    kd = float(tune.get('gim_yaw_kd', 0.35))
    spd = float(tune.get('gim_yaw_speed', 1615.0))

    new_kp, new_kd, new_spd = kp, kd, spd
    if big and over > 5.0 and rev >= 2:
        new_kp = round(kp * 0.85, 2)
        new_kd = round(max(kd * 1.5, kp * 0.04), 3)
        notes.append(f'位置环超调 {over:.1f}% + 来回摆 {rev:.1f} 次：'
                     f'降 kp、提 kd（重夹爪惯量变大，阻尼不够）')
    elif big and over < 2.0 and settle > 1.2:
        new_kp = round(kp * 1.10, 2)
        notes.append(f'位置环不超调但稳定慢（{settle:.1f}s）：可以小幅提 kp')
    if big and settle > 1.5 and spd > 1200:
        new_spd = round(spd * 0.85, 0)
        notes.append(f'大角度稳定 {settle:.1f}s 偏长：可把 yaw 最大速度从 {spd:.0f} '
                     f'降到 {new_spd:.0f}°/s，重负载下少冲一点')

    # 视觉跟踪
    tkp = float(tcfg.get('kp_yaw', 1.0))
    tkd = float(tcfg.get('kd_yaw', 0.02))
    ttau = float(tcfg.get('deriv_tau', 0.05))
    tdb = float(tcfg.get('deadband_px', 8.0))
    ntkp, ntkd, nttau, ntdb = tkp, tkd, ttau, tdb
    if track is not None:
        if track['yv_reversals'] >= 20 or track['yv_jitter'] > 40:
            ntkp = round(tkp * 0.75, 3)
            ntkd = 0.0
            nttau = max(ttau, 0.08)
            notes.append(f'跟踪 yaw 抖动大（{track["yv_jitter"]:.0f}°/s、'
                         f'反向 {track["yv_reversals"]} 次）：降 kp_yaw、'
                         f'kd_yaw 归零（微分会放大像素噪声）、加大 deriv_tau')
        if track['ex_noise'] > 1.5:
            ntdb = round(max(tdb, min(15.0, 2.0 * track['ex_noise'])), 1)
            notes.append(f'像素噪声 {track["ex_noise"]:.1f}px：死区抬到 {ntdb}px，'
                         f'别让 PID 追噪声')
        if track['lag_ms'] > 60 and track['yv_jitter'] <= 40:
            ntkp = round(tkp * 1.15, 3)
            notes.append(f'跟踪滞后 {track["lag_ms"]:.0f}ms：可小幅提 kp_yaw，'
                         f'或把 ctrl_filter_tau 再压小')

    print('-' * 70)
    if not notes:
        print('没有明显异常，先别大改；可以只调一两个量观察。')
    else:
        for n in notes:
            print('  · ' + n)

    print('-' * 70)
    print('建议：')
    print(f"  tune_profile.json   gim_yaw_kp {kp:g} → {new_kp:g}   "
          f"gim_yaw_kd {kd:g} → {new_kd:g}   gim_yaw_speed {spd:g} → {new_spd:g}")
    print(f"  track_pid.json      kp_yaw {tkp:g} → {ntkp:g}   kd_yaw {tkd:g} → {ntkd:g}   "
          f"deriv_tau {ttau:g} → {nttau:g}   deadband_px {tdb:g} → {ntdb:g}")
    print('  （F407 的 gim_* 参数要在上位机 PID 页点“全部下发”或重连后才会生效）')
    return {
        'tune': {'gim_yaw_kp': new_kp, 'gim_yaw_kd': new_kd,
                 'gim_yaw_speed': new_spd},
        'track': {'kp_yaw': ntkp, 'kd_yaw': ntkd,
                  'deriv_tau': nttau, 'deadband_px': ntdb},
    }


def main():
    ap = argparse.ArgumentParser(description='云台 yaw 波形分析 + PID 建议')
    ap.add_argument('--wave', default=str(DEFAULT_WAVE))
    ap.add_argument('--track', default=str(DEFAULT_TRACK))
    ap.add_argument('--tune', default=str(DEFAULT_TUNE),
                    help='上位机 tune_profile.json（F407 位置环增益）')
    ap.add_argument('--track-config', default=str(DEFAULT_TRACK_CFG),
                    help='vision/config/track_pid.json（视觉跟踪 PID）')
    ap.add_argument('--apply', action='store_true',
                    help='把建议写回两份配置（先备份 .bak）')
    args = ap.parse_args()

    tune = load_json(Path(args.tune))
    tcfg = load_json(Path(args.track_config))

    steps = []
    vels = []
    track = None

    wave_path = Path(args.wave)
    if wave_path.is_file():
        rows = load(wave_path)
        print('=' * 70)
        print(f'云台波形 {wave_path}  ({len(rows)} 行)')
        print('=' * 70)
        steps = analyze_steps(rows)
        vels = analyze_velocity(rows)
        report_steps(steps)
        print('-' * 70)
        report_vel(vels)
    else:
        print(f'没有 {wave_path}（先跑 scripts/gimbal_wave_logger.py 录一段）')

    track_path = Path(args.track)
    if track_path.is_file():
        trows = load(track_path)
        track = analyze_track(trows)
        print('-' * 70)
        report_track(track)
    else:
        print(f'没有 {track_path}（color_pick 运行时会自动生成）')

    rec = suggest(steps, vels, track, tune, tcfg)
    if not rec:
        return 1

    if args.apply:
        for path, key, values in (
                (Path(args.tune), 'tune', rec['tune']),
                (Path(args.track_config), 'track', rec['track'])):
            try:
                doc = load_json(path)
                if not doc:
                    print(f'跳过 {path}：读不到或不是 JSON')
                    continue
                backup = path.with_suffix(path.suffix + '.bak')
                shutil.copy2(path, backup)
                doc.update(values)
                path.write_text(json.dumps(doc, ensure_ascii=False, indent=1),
                                encoding='utf-8')
                print(f'已写回 {path}（备份 {backup.name}）')
            except OSError as error:
                print(f'写 {path} 失败：{error}')
    else:
        print('\n加 --apply 可以直接写回这两份配置。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

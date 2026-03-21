#!/usr/bin/env python3
"""
analyze_timing_log.py — aoba_hardware 通信ログ統計分析

ros2_control_node のタイミングログを解析し、CAN 通信タイミングとモータ状態の統計を出力する。
launch ファイルから自動起動される（stdin 経由）。手動実行も可能:

  analyze_timing_log.py [log_file]     # ファイルから読み込み
  analyze_timing_log.py                # stdin から読み込み（デフォルト）
  cat launch.log | analyze_timing_log.py
"""
import math
import re
import sys
from dataclasses import dataclass


# ============================================================
# Log line patterns
# ============================================================

# [Timing] write() min=150us avg=200us max=350us stddev=20.5us |
#   send min=50 avg=80 max=120us | recv min=100 avg=120 max=230us | missed=0/200
_TIMING_WRITE_RE = re.compile(
    r'\[Timing\] write\(\)'
    r'\s+min=(\d+(?:\.\d+)?)us'
    r'\s+avg=(\d+(?:\.\d+)?)us'
    r'\s+max=(\d+(?:\.\d+)?)us'
    r'\s+stddev=(\d+(?:\.\d+)?)us'
    r'\s+\|\s+send min=(\d+(?:\.\d+)?)'
    r'\s+avg=(\d+(?:\.\d+)?)'
    r'\s+max=(\d+(?:\.\d+)?)us'
    r'\s+\|\s+recv min=(\d+(?:\.\d+)?)'
    r'\s+avg=(\d+(?:\.\d+)?)'
    r'\s+max=(\d+(?:\.\d+)?)us'
    r'\s+\|\s+missed=(\d+)/(\d+)'
)

# [Timing]   can1: send=80us, recv=120us, ok=1/1
_TIMING_BUS_RE = re.compile(
    r'\[Timing\]\s+(\w+):\s+'
    r'send=(\d+(?:\.\d+)?)us,\s+'
    r'recv=(\d+(?:\.\d+)?)us,\s+'
    r'ok=(\d+)/(\d+)'
)

# [State] test_joint (id=127): cmd=0.5000 rad | pos=0.4998 rad, vel=0.0012 rad/s, effort=0.1234 Nm
_STATE_RE = re.compile(
    r'\[State\]\s+(\S+)\s+\(id=(\d+)\):'
    r'\s+cmd=([+-]?\d+(?:\.\d+)?)\s+rad'
    r'\s+\|\s+pos=([+-]?\d+(?:\.\d+)?)\s+rad,'
    r'\s+vel=([+-]?\d+(?:\.\d+)?)\s+rad/s,'
    r'\s+effort=([+-]?\d+(?:\.\d+)?)\s+Nm'
)


# ============================================================
# Data classes
# ============================================================

@dataclass
class _TimingWindow:
    total_min: float
    total_avg: float
    total_max: float
    total_stddev: float
    send_min: float
    send_avg: float
    send_max: float
    recv_min: float
    recv_avg: float
    recv_max: float
    missed: int
    total_expected: int  # = cycles_per_window * n_motors


@dataclass
class _BusSample:
    name: str
    send_us: float
    recv_us: float
    ok: int
    expected: int


@dataclass
class _StateSample:
    joint: str
    motor_id: int
    cmd: float
    pos: float
    vel: float
    effort: float


# ============================================================
# Parser
# ============================================================

def _parse_lines(lines):
    windows: list[_TimingWindow] = []
    bus_samples: dict[str, list[_BusSample]] = {}
    state_samples: dict[str, list[_StateSample]] = {}

    for line in lines:
        m = _TIMING_WRITE_RE.search(line)
        if m:
            windows.append(_TimingWindow(
                total_min=float(m.group(1)),
                total_avg=float(m.group(2)),
                total_max=float(m.group(3)),
                total_stddev=float(m.group(4)),
                send_min=float(m.group(5)),
                send_avg=float(m.group(6)),
                send_max=float(m.group(7)),
                recv_min=float(m.group(8)),
                recv_avg=float(m.group(9)),
                recv_max=float(m.group(10)),
                missed=int(m.group(11)),
                total_expected=int(m.group(12)),
            ))
            continue

        m = _TIMING_BUS_RE.search(line)
        if m:
            bus = m.group(1)
            bus_samples.setdefault(bus, []).append(_BusSample(
                name=bus,
                send_us=float(m.group(2)),
                recv_us=float(m.group(3)),
                ok=int(m.group(4)),
                expected=int(m.group(5)),
            ))
            continue

        m = _STATE_RE.search(line)
        if m:
            joint = m.group(1)
            state_samples.setdefault(joint, []).append(_StateSample(
                joint=joint,
                motor_id=int(m.group(2)),
                cmd=float(m.group(3)),
                pos=float(m.group(4)),
                vel=float(m.group(5)),
                effort=float(m.group(6)),
            ))

    return windows, bus_samples, state_samples


# ============================================================
# Report
# ============================================================

_SEP = '─' * 58


def _mean(vals: list[float]) -> float:
    return sum(vals) / len(vals) if vals else 0.0


def _stddev(vals: list[float]) -> float:
    if not vals:
        return 0.0
    avg = _mean(vals)
    return math.sqrt(sum((v - avg) ** 2 for v in vals) / len(vals))


def _print_report(
    windows: list[_TimingWindow],
    bus_samples: dict[str, list[_BusSample]],
    state_samples: dict[str, list[_StateSample]],
) -> None:
    print()
    print('=' * 58)
    print('  aoba_hardware 通信ログ統計レポート')
    print('=' * 58)

    if not windows:
        print()
        print('  [WARNING] [Timing] write() 行が見つかりませんでした。')
        print('  テストが 1 秒以上実行されているか確認してください。')
        print('  (タイミングログは 200 サイクル = 1s ごとに出力されます)')
        return

    cycles_per_window = windows[0].total_expected
    n_win = len(windows)
    total_cycles = n_win * cycles_per_window

    print()
    print(f'  計測ウィンドウ: {n_win} 個  '
          f'(各 {cycles_per_window} サイクル = {cycles_per_window / 200:.1f}s @ 200Hz)')
    print(f'  推定計測時間:   {total_cycles / 200:.1f} s')

    # ── write() Cycle Time ──────────────────────────────────────
    print()
    print(_SEP)
    print('  write() サイクル時間 [us]')
    print(_SEP)

    overall_min = min(w.total_min for w in windows)
    overall_avg = _mean([w.total_avg for w in windows])
    overall_max = max(w.total_max for w in windows)
    avg_stddev = _mean([w.total_stddev for w in windows])
    avgs = [w.total_avg for w in windows]

    print(f'  Overall min:    {overall_min:.0f}')
    print(f'  Overall avg:    {overall_avg:.1f}')
    print(f'  Overall max:    {overall_max:.0f}')
    print(f'  Avg stddev:     {avg_stddev:.1f}')
    print(f'  Stddev of avgs: {_stddev(avgs):.1f}  (ウィンドウ間の安定性)')
    print()
    print('  ウィンドウ別:')
    for i, w in enumerate(windows):
        bar_len = max(0, min(20, int(w.total_avg / 5000 * 20)))
        bar = '█' * bar_len
        print(f'    #{i + 1:2d}  avg={w.total_avg:5.0f}  max={w.total_max:5.0f}'
              f'  missed={w.missed:3d}/{w.total_expected}  {bar}')

    # ── Send / Recv ─────────────────────────────────────────────
    print()
    print(_SEP)
    print('  送信 / 受信 内訳 [us]')
    print(_SEP)
    print(f'  Send  min={min(w.send_min for w in windows):.0f}'
          f'  avg={_mean([w.send_avg for w in windows]):.1f}'
          f'  max={max(w.send_max for w in windows):.0f}')
    print(f'  Recv  min={min(w.recv_min for w in windows):.0f}'
          f'  avg={_mean([w.recv_avg for w in windows]):.1f}'
          f'  max={max(w.recv_max for w in windows):.0f}')

    # ── Per-Bus ─────────────────────────────────────────────────
    if bus_samples:
        print()
        print(_SEP)
        print('  バスごとの内訳 (全サンプル平均)')
        print(_SEP)
        for bus, samples in sorted(bus_samples.items()):
            send_avg = _mean([s.send_us for s in samples])
            send_max = max(s.send_us for s in samples)
            recv_avg = _mean([s.recv_us for s in samples])
            recv_max = max(s.recv_us for s in samples)
            ok_total = sum(s.ok for s in samples)
            exp_total = sum(s.expected for s in samples)
            print(f'  {bus}:')
            print(f'    send  avg={send_avg:.0f}  max={send_max:.0f} us')
            print(f'    recv  avg={recv_avg:.0f}  max={recv_max:.0f} us')
            print(f'    ok    {ok_total}/{exp_total}')

    # ── Missed Responses ────────────────────────────────────────
    print()
    print(_SEP)
    print('  missed response')
    print(_SEP)
    total_missed = sum(w.missed for w in windows)
    total_expected = sum(w.total_expected for w in windows)
    missed_pct = total_missed / total_expected * 100 if total_expected > 0 else 0.0
    print(f'  合計:  {total_missed} / {total_expected}  ({missed_pct:.3f}%)')
    if total_missed == 0:
        print('  → 全サイクルで応答受信  ✓')
    elif missed_pct < 0.5:
        print('  → 軽微な missed (0.5% 未満)  許容範囲')
    else:
        print(f'  → missed 率 {missed_pct:.1f}%  — 通信品質を確認してください')

    # ── 200Hz Feasibility ───────────────────────────────────────
    print()
    print(_SEP)
    print('  200Hz 制御フィージビリティ判定')
    print(_SEP)
    if overall_avg < 3000:
        timing_grade = f'良好  (avg {overall_avg:.0f}us < 3ms)'
    elif overall_avg < 4000:
        timing_grade = f'許容範囲  (avg {overall_avg:.0f}us < 4ms)'
    else:
        timing_grade = f'要注意  (avg {overall_avg:.0f}us ≥ 4ms)'

    max_ok = overall_max < 5000
    print(f'  write() avg:  {timing_grade}')
    if max_ok:
        print(f'  write() max:  OK  ({overall_max:.0f}us < 5ms)')
    else:
        print(f'  write() max:  NG  ({overall_max:.0f}us ≥ 5ms — 周期オーバーランの可能性)')
    print(f'  missed 率:    {missed_pct:.3f}%')
    print()
    if max_ok and missed_pct < 1.0:
        print('  ✓  200Hz 安定動作  OK')
    else:
        print('  ✗  200Hz 安定動作に問題がある可能性あり')

    # ── Motor State Statistics ───────────────────────────────────
    if state_samples:
        print()
        print(_SEP)
        print('  モータ状態統計')
        print(_SEP)
        for joint, samples in sorted(state_samples.items()):
            n = len(samples)
            pos_vals = [s.pos for s in samples]
            vel_vals = [s.vel for s in samples]
            eff_vals = [s.effort for s in samples]
            cmd_vals = [s.cmd for s in samples]
            err_vals = [abs(s.pos - s.cmd) for s in samples]
            print(f'  {joint}  (id={samples[0].motor_id}, N={n} サンプル):')
            print(f'    cmd     [{min(cmd_vals):+.4f}, {max(cmd_vals):+.4f}] rad')
            print(f'    pos     [{min(pos_vals):+.4f}, {max(pos_vals):+.4f}] rad')
            print(f'    |err|   avg={_mean(err_vals):.4f}  max={max(err_vals):.4f} rad')
            print(f'    vel     [{min(vel_vals):+.4f}, {max(vel_vals):+.4f}] rad/s')
            print(f'    effort  [{min(eff_vals):+.4f}, {max(eff_vals):+.4f}] Nm')

    print()
    print('=' * 58)
    print()


# ============================================================
# Entry point
# ============================================================

def main() -> None:
    import os

    if len(sys.argv) > 1:
        log_path = sys.argv[1]
    else:
        log_path = os.path.expanduser('~/.ros/log/latest/launch.log')

    print(f'[analyze_timing_log] 解析対象: {log_path}')
    with open(log_path) as f:
        lines = f.readlines()

    windows, bus_samples, state_samples = _parse_lines(lines)
    _print_report(windows, bus_samples, state_samples)


if __name__ == '__main__':
    main()

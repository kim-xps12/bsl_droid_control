#!/usr/bin/env python3
"""学習ログの汎用分析スクリプト.

TensorBoardイベントファイルを読み込み、以下の分析を行う:
  1. 主要メトリクスの時系列推移
  2. 区間別統計（平均・標準偏差）
  3. 区間別線形トレンド（slope）
  4. 収束判定（変動係数・トレンド勾配）
  5. 損失関数・ノイズの推移
  6. 個別報酬項目の推移

使い方:
  uv run python scripts/analyze_training_log.py <実験名> [--last N]

例:
  uv run python scripts/analyze_training_log.py droid-walking-omni-v14
  uv run python scripts/analyze_training_log.py droid-walking-v19 --last 200
"""

from __future__ import annotations

import argparse
import os
import sys

from tensorboard.backend.event_processing import event_accumulator


# ---------------------------------------------------------------------------
# ユーティリティ
# ---------------------------------------------------------------------------


def _segment(data: list, start: int, end: int) -> list[float]:
    """step が [start, end] に含まれる値のリストを返す."""
    return [s.value for s in data if start <= s.step <= end]


def segment_stats(data: list, start: int, end: int) -> tuple[float | None, float | None]:
    """区間 [start, end] の平均と標準偏差を返す."""
    seg = _segment(data, start, end)
    if not seg:
        return None, None
    mean = sum(seg) / len(seg)
    std = (sum((x - mean) ** 2 for x in seg) / len(seg)) ** 0.5
    return mean, std


def linear_trend(data: list) -> tuple[float, float]:
    """ScalarEvent のリストから線形回帰の slope と平均値を返す."""
    n = len(data)
    x = [s.step for s in data]
    y = [s.value for s in data]
    x_mean = sum(x) / n
    y_mean = sum(y) / n
    num = sum((xi - x_mean) * (yi - y_mean) for xi, yi in zip(x, y))
    den = sum((xi - x_mean) ** 2 for xi in x)
    slope = num / den if den != 0 else 0.0
    return slope, y_mean


def _make_segments(total_steps: int, width: int = 500) -> list[tuple[int, int]]:
    """0 から total_steps まで width 幅の区間リストを生成する."""
    segs: list[tuple[int, int]] = []
    s = 0
    while s < total_steps:
        e = min(s + width - 1, total_steps)
        segs.append((s, e))
        s += width
    return segs


# ---------------------------------------------------------------------------
# 分析本体
# ---------------------------------------------------------------------------


def analyze_log(exp_name: str, *, last_n: int = 500) -> None:  # noqa: C901, PLR0912, PLR0915
    """指定した実験の TensorBoard ログを包括的に分析する."""
    log_dir = f"logs/{exp_name}"
    if not os.path.isdir(log_dir):
        print(f"Error: ディレクトリが見つかりません: {log_dir}", file=sys.stderr)
        sys.exit(1)

    event_files = sorted(f for f in os.listdir(log_dir) if f.startswith("events."))
    if not event_files:
        print(f"Error: イベントファイルが見つかりません: {log_dir}", file=sys.stderr)
        sys.exit(1)

    ea = event_accumulator.EventAccumulator(
        os.path.join(log_dir, event_files[0]),
        size_guidance={event_accumulator.SCALARS: 0},
    )
    ea.Reload()

    all_tags = sorted(ea.Tags()["scalars"])
    print(f"{'=' * 70}")
    print(f"  実験: {exp_name}")
    print(f"  イベントファイル: {event_files[0]}")
    print(f"  利用可能タグ数: {len(all_tags)}")
    print(f"{'=' * 70}\n")

    # ------------------------------------------------------------------
    # 1. Train/mean_reward の推移
    # ------------------------------------------------------------------
    scalars = ea.Scalars("Train/mean_reward") if "Train/mean_reward" in all_tags else []
    if not scalars:
        print("Train/mean_reward が見つかりません。スキップします。\n")
        return

    total = len(scalars)
    last_step = scalars[-1].step
    print(f"[1] Train/mean_reward の推移  (total {total} points, last step={last_step})")
    print("-" * 60)
    interval = max(1, total // 20)
    for i in range(0, total, interval):
        s = scalars[i]
        print(f"  iter {s.step:6d}: {s.value:10.4f}")
    s = scalars[-1]
    print(f"  iter {s.step:6d}: {s.value:10.4f}  (last)")
    print()

    # ------------------------------------------------------------------
    # 2. Train/mean_episode_length の推移
    # ------------------------------------------------------------------
    ep_len = ea.Scalars("Train/mean_episode_length") if "Train/mean_episode_length" in all_tags else []
    if ep_len:
        print(f"[2] Train/mean_episode_length の推移")
        print("-" * 60)
        for i in range(0, len(ep_len), interval):
            s = ep_len[i]
            print(f"  iter {s.step:6d}: {s.value:10.4f}")
        s = ep_len[-1]
        print(f"  iter {s.step:6d}: {s.value:10.4f}  (last)")
        print()

    # ------------------------------------------------------------------
    # 3. 区間別統計
    # ------------------------------------------------------------------
    segments = _make_segments(last_step)

    print(f"[3] 区間別統計  (区間幅=500)")
    print("-" * 60)
    print(f"  {'区間':>16s}  {'reward平均':>10s}  {'reward σ':>10s}  {'ep_len平均':>10s}  {'ep_len σ':>10s}")
    for seg_start, seg_end in segments:
        r_mean, r_std = segment_stats(scalars, seg_start, seg_end)
        e_mean, e_std = segment_stats(ep_len, seg_start, seg_end) if ep_len else (None, None)
        r_m = f"{r_mean:10.4f}" if r_mean is not None else f"{'N/A':>10s}"
        r_s = f"{r_std:10.4f}" if r_std is not None else f"{'N/A':>10s}"
        e_m = f"{e_mean:10.4f}" if e_mean is not None else f"{'N/A':>10s}"
        e_s = f"{e_std:10.4f}" if e_std is not None else f"{'N/A':>10s}"
        print(f"  {seg_start:6d}-{seg_end:6d}  {r_m}  {r_s}  {e_m}  {e_s}")
    print()

    # ------------------------------------------------------------------
    # 4. 区間別トレンド（線形回帰 slope）
    # ------------------------------------------------------------------
    print(f"[4] 区間別トレンド  (mean_reward の線形回帰 slope)")
    print("-" * 60)
    # 大きめの区間でトレンドを見る
    trend_width = max(500, last_step // 4)
    trend_segs = _make_segments(last_step, width=trend_width)
    # 最後の last_n 区間も追加
    trend_segs.append((max(0, last_step - last_n), last_step))
    for t_start, t_end in trend_segs:
        seg = [s for s in scalars if t_start <= s.step <= t_end]
        if len(seg) >= 2:
            slope, y_mean = linear_trend(seg)
            print(f"  iter {t_start:6d}-{t_end:6d}: slope={slope:+.6f}/iter ({slope * 100:+.4f}/100iter), mean={y_mean:.4f}")
    print()

    # ------------------------------------------------------------------
    # 5. 収束判定
    # ------------------------------------------------------------------
    print(f"[5] 収束判定  (最後 {last_n} イテレーション)")
    print("-" * 60)
    last_data = [s for s in scalars if s.step > last_step - last_n]
    if len(last_data) >= 2:
        y_vals = [s.value for s in last_data]
        mean_val = sum(y_vals) / len(y_vals)
        std_val = (sum((x - mean_val) ** 2 for x in y_vals) / len(y_vals)) ** 0.5
        cv = std_val / abs(mean_val) * 100 if mean_val != 0 else float("inf")
        slope, _ = linear_trend(last_data)

        print(f"  平均: {mean_val:.4f}")
        print(f"  最大: {max(y_vals):.4f}")
        print(f"  最小: {min(y_vals):.4f}")
        print(f"  標準偏差: {std_val:.4f}")
        print(f"  変動係数 (CV): {cv:.2f}%")
        print(f"  線形トレンド slope: {slope:+.6f}/iter ({slope * 100:+.4f}/100iter)")
        print()

        # 判定ロジック
        improving = slope > 0.001
        high_cv = cv > 5.0
        print("  【判定】", end="")
        if improving and not high_cv:
            print("📈 まだ改善中 — 追加学習で性能向上の見込みあり")
        elif improving and high_cv:
            print("📈 改善傾向だが変動大 — 追加学習は有効だが不安定")
        elif not improving and not high_cv:
            print("✅ 収束済み — 追加学習による大幅な性能向上は見込めない")
        else:
            print("⚠️  横ばいかつ変動大 — 学習が不安定、ハイパーパラメータ調整を検討")
    else:
        print("  データ不足（last_n に十分なデータなし）")
    print()

    # ------------------------------------------------------------------
    # 6. 損失関数・ポリシーの推移
    # ------------------------------------------------------------------
    loss_tags = ["Loss/surrogate", "Loss/value_function", "Loss/entropy", "Loss/learning_rate"]
    available_loss = [t for t in loss_tags if t in all_tags]
    if available_loss:
        print(f"[6] 損失関数・学習率の推移  (最初500iter vs 最後500iter)")
        print("-" * 60)
        for tag in available_loss:
            data = ea.Scalars(tag)
            first_seg = _segment(data, 0, 500)
            last_seg = _segment(data, max(0, last_step - 500), last_step)
            if first_seg and last_seg:
                first_mean = sum(first_seg) / len(first_seg)
                last_mean = sum(last_seg) / len(last_seg)
                print(f"  {tag}:")
                print(f"    最初500iter平均: {first_mean:.6f}")
                print(f"    最後500iter平均: {last_mean:.6f}")
        print()

    # Policy/mean_noise_std
    if "Policy/mean_noise_std" in all_tags:
        noise = ea.Scalars("Policy/mean_noise_std")
        first_seg = _segment(noise, 0, 500)
        last_seg = _segment(noise, max(0, last_step - 500), last_step)
        print(f"  Policy/mean_noise_std:")
        if first_seg:
            print(f"    最初500iter平均: {sum(first_seg) / len(first_seg):.6f}")
        if last_seg:
            print(f"    最後500iter平均: {sum(last_seg) / len(last_seg):.6f}")
        last_noise = [s for s in noise if s.step > last_step - last_n]
        if len(last_noise) >= 2:
            slope, _ = linear_trend(last_noise)
            print(f"    最後{last_n}iter slope: {slope:+.8f}/iter")
        print()

    # ------------------------------------------------------------------
    # 7. 個別報酬項目の推移
    # ------------------------------------------------------------------
    reward_tags = sorted(t for t in all_tags if t.startswith("Episode/rew_"))
    if reward_tags:
        print(f"[7] 個別報酬項目の推移  (最初500 / 中盤 / 最後500)")
        print("-" * 60)
        mid_start = last_step // 2 - 250
        mid_end = last_step // 2 + 250
        for tag in reward_tags:
            data = ea.Scalars(tag)
            m_first, _ = segment_stats(data, 0, 500)
            m_mid, _ = segment_stats(data, mid_start, mid_end)
            m_last, _ = segment_stats(data, max(0, last_step - 500), last_step)
            last_seg = [s for s in data if s.step > last_step - last_n]
            slope_str = ""
            if len(last_seg) >= 2:
                slope, _ = linear_trend(last_seg)
                slope_str = f"  slope(last{last_n})={slope:+.6f}"
            f_str = f"{m_first:.4f}" if m_first is not None else "N/A"
            m_str = f"{m_mid:.4f}" if m_mid is not None else "N/A"
            l_str = f"{m_last:.4f}" if m_last is not None else "N/A"
            short_tag = tag.replace("Episode/rew_", "")
            print(f"  {short_tag:>35s}: first={f_str:>9s}  mid={m_str:>9s}  last={l_str:>9s}{slope_str}")
        print()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description="TensorBoard 学習ログの分析")
    parser.add_argument("exp_name", help="実験名 (logs/ 以下のディレクトリ名)")
    parser.add_argument("--last", type=int, default=500, help="収束判定に使う末尾イテレーション数 (default: 500)")
    args = parser.parse_args()
    analyze_log(args.exp_name, last_n=args.last)


if __name__ == "__main__":
    main()

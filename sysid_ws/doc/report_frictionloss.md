# frictionloss 同定改善レポート

RS-02 同定パイプラインにおいて `frictionloss` パラメータが最適化下限値
（0.001 Nm）に張り付いていた問題の調査と解決の記録。

## TL;DR

- **問題**: `optimize.py` で `frictionloss` がレンジ下限 `0.001 Nm` に張り付き、物理的に意味のある値が出ない。
- **原因**: 励振信号 `multi_sine_torque` が 4.5 Hz 帯域中心で、慣性支配領域 (J·θ̈) が圧倒的。低速度（friction-dominated）域がほとんど励起されない → 最適化器がコスト関数で friction 寄与を抽出できない。
- **解決**: 励振信号に低周波 `0.3·f` (= 1.35 Hz at f=4.5) 成分を追加。ただし `sin` ではなく `cos` を使用（ドリフト回避）。
- **結果**: `frictionloss` が `0.001 → 0.0305` に脱出。両 RS-02 個体で sim2real RMSE 50%超改善を維持しつつ、物理的に妥当な値を取得。

## 背景

[optimize.py](../optimizer/sysid/optimize.py) は MuJoCo シミュレーションリプレイ法 +
`scipy.optimize.least_squares` (Trust Region Reflective) で 3 パラメータ
`armature, frictionloss, damping` を同定する。下限上限は次のように設定 ([optimize.py:183-184](../optimizer/sysid/optimize.py#L183-L184)):

| パラメータ | 下限 | 上限 |
|---|---|---|
| armature | 0.001 | 0.500 |
| frictionloss | **0.001** | 2.000 |
| damping | 0.001 | 1.000 |

motor 11 の sysid 録画で同定したところ:

| パラメータ | 値 | 評価 |
|---|---|---|
| `armature` | 0.01538 kg·m² | RS-02 ロータ慣性 ~30 g·cm² × ギア比 7.75² ≈ 0.018 と整合 ✓ |
| **`frictionloss`** | **0.00100 Nm（下限張り付き）** | ❌ 最適化が下限に押し付けている |
| `damping` | 0.02884 Nm/(rad/s) | QDD として妥当（0.01〜0.1）✓ |

## 初期仮説（誤り）と再検討

最初は「励振信号の zero-mean 性が原因 → DC バイアスを乗せれば解消」と仮定した。

ユーザーから**反証**: 「これだけ振り回して摩擦項が出ないのに DC バイアス乗せたら出るかも、というのは直感的でない」。

→ web 検索による先行事例調査を実施。

## 文献調査結果

複数の文献から得た合意事項:

1. **「Coulomb friction remains critical at low velocities」** ([PMC12788085](https://pmc.ncbi.nlm.nih.gov/articles/PMC12788085/)) — 摩擦は **低速度域で支配的**。多サイン励振が 4.5 Hz で大振幅運動を作ると、ピーク速度 ~10 rad/s 級になり friction の相対寄与は数 % にまで縮小する。
2. **「Pure Coulomb friction model is not well-suited for optimization methods」** ([MDPI 568](https://www.mdpi.com/2075-1702/10/7/568)) — `μ·sign(v)` の不連続性は最適化器に厳しい。MuJoCo の `frictionloss` は constraint 実装で勾配がスパース ([README:308-312](../README.md#L308-L312))。
3. **「Constant excitation with magnitude larger than max static friction can greatly simplify the behavior」** ([PMC12788085](https://pmc.ncbi.nlm.nih.gov/articles/PMC12788085/)) — DC バイアス手法は標準テクニックの 1 つ。
4. QDD 文献 ([CubeMars](https://www.cubemars.com/qdd-motors-for-humanoid-and-quadruped-robots.html)) — QDD は構造的に "relatively low friction" で、harmonic drive 等の高ギア比サーボとは別世界。

## 原因分析（数値見積もり）

J=0.015, b=0.029, μ=0.10 仮定で各項のピーク値:

| 項 | 式 | ピーク値 |
|---|---|---|
| 慣性 | J·θ̈ ≈ 0.015 × (2π·4.5)² × 0.5 rad | **±6 Nm** |
| 粘性 | b·v at v=10 | ±0.5 Nm |
| 摩擦 | μ·sign(v) | ±0.1 Nm |

**摩擦寄与は全トルクの ~2%**。最小二乗が friction = 0 を選んでも残差はほぼ変わらない → 下限張り付きは数学的に正しい挙動。

## 解決策

### 方針

低周波成分を加えて低速度域 (v ~ 1〜2 rad/s) を励起し、friction-dominated 領域を作る。
[doc/sysid.md:11-13](sysid.md#L11-L13) も「低周波：摩擦が支配的になります」と明記。

### 試行 1: 低周波 sin 成分を追加（失敗）

[excitation.hpp](../recorder/include/sysid_recorder/excitation.hpp) を以下に変更:

```cpp
return amp * (0.4 * std::sin(TWO_PI * 0.3 * freq * t) +  // 1.35 Hz 追加
              std::sin(TWO_PI * freq * t) +
              0.6 * std::sin(TWO_PI * 3.4 * freq * t) +
              0.3 * std::sin(TWO_PI * 7.4 * freq * t));
```

**結果**: `Drift guard tripped` で 0.5 秒で abort:

```text
Motor probe OK: pos=-12.40 rad, vel=-0.01 rad/s
[ABORT] Drift guard tripped: pos_drift=6.289 rad  vel=6.94 rad/s
```

### 試行 1 の根本原因

`sin(ωt)` の積分は `(1 - cos(ωt))/ω` で **常に非負**。
低周波 (1.35 Hz, 周期 740 ms) では recording 開始から第1半周期 (370 ms) を過ぎる時点まで一方向のトルク impulse が単調に積み上がる。

数値検証（CSV から）:
- `cmd_torque mean = +0.58 Nm`（明確な正バイアス）
- `velocity end = 22 rad/s` ≈ `mean × T / J = 0.58 × 0.531 / 0.015 = 20.5 rad/s` ✓

### なぜ original (4.5/15.3/33.3 Hz の sin) は drift しないのか

実は **整数周期化の設計の妙** だった。T=30 s で:

| 成分 | freq × T | cycles | `(1-cos(ωT))/ω` |
|---|---|---|---|
| 1.0·f | 4.5 × 30 | 135 | 0 |
| 3.4·f | 15.3 × 30 | 459 | 0 |
| 7.4·f | 33.3 × 30 | 999 | 0 |

すべて整数 → impulse residual がゼロ。doc/sysid.md の「非有理数比」表現は厳密には誤りで、
実態は **「特定の録画長で整数周期になる選定」**だった。

新追加の 0.3·f = 1.35 Hz は T=30 で 40.5 cycles（半整数）→ residual 残る。さらに録画途中で abort されるとさらに偏りが大きい。

### 試行 2: 低周波 cos 成分を追加（成功）

`cos(ωt)` の積分は `sin(ωt)/ω` で **±1/ω に有界振動**（ドリフトしない）。

```cpp
return amp * (0.4 * std::cos(TWO_PI * 0.3 * freq * t) +  // sin → cos
              std::sin(TWO_PI * freq * t) +
              0.6 * std::sin(TWO_PI * 3.4 * freq * t) +
              0.3 * std::sin(TWO_PI * 7.4 * freq * t));
```

代償: t=0 で `cos = 1` の torque step input (0.4·amp Nm)。
ただし position/velocity は連続に 0 から立ち上がるので機械的衝撃なし。

数値検証 (J=0.015, τ_low=1 Nm, ω=8.48):
- cos: position は `[0, 2τ/(J·ω²)] = [0, 1.86]` rad で振動 → drift_guard 余裕
- sin: position は **時間と共に線形成長** → アウト

[excitation.hpp:9-29](../recorder/include/sysid_recorder/excitation.hpp#L9-L29) のコメントでこの設計判断を明記。

## 結果

amp=1.5, duration=30 s で再録画 → optimize.py 実行:

| パラメータ | v1 (original sin) | **v2 (cos low-freq)** | 評価 |
|---|---|---|---|
| `armature` | 0.01538 | 0.01535 | 差 0.2%、安定 |
| **`frictionloss`** | **0.00100 (下限張り付き)** | **0.03051** | **下限脱出** ✓ |
| `damping` | 0.02794 | 0.02887 | 差 3%、整合 |

### sim2real 検証 (validate.py)

motor 11（同定元）と motor 21（別個体）の両方で v2 params を評価:

| 項目 | motor 11 | motor 21 |
|---|---|---|
| pos RMSE before | 0.0788 | 0.0781 |
| **pos RMSE after** | **0.0352** | **0.0369** |
| pos 改善率 | 55.3% | 52.8% |
| vel RMSE before | 1.8113 | 1.7809 |
| **vel RMSE after** | **0.7477** | **0.7392** |
| vel 改善率 | 58.7% | 58.5% |

両個体とも README 成功基準 ([README:289-291](../README.md#L289-L291)) を満たし、
**identified params が motor 個体間で転移可能**であることも確認。

## 物理的解釈

`frictionloss = 0.030 Nm` は README 期待レンジ 0.05–0.5 ([README:226](../README.md#L226)) よりやや低いが:

1. 期待レンジは harmonic / cycloidal 等の **高ギア比サーボ前提** の数値
2. RS-02 はギア比 7.75:1 の QDD で **構造的に friction が小さい**設計（CubeMars QDD page も明言）
3. **0.030 Nm は RS-02 の真値である可能性が高い**

validation RMSE が friction 0.001 → 0.030 でほぼ変化しないこと自体が「friction の寄与が小さい = 真の friction が小さい」の傍証。
kp=8 の PD 制御では friction 0.03 Nm はトータルトルクの極小寄与なので、
validation で見える差は微小（pos RMSE 0.0363→0.0352 = 3% 改善程度）。

## 残課題と今後の方向

### 1. README 期待レンジの更新

[README:225-227](../README.md#L225-L227) の expected range は QDD 寄りに修正したい:

```
| frictionloss | 0.01 – 0.5 Nm (QDD では下限寄り) |
```

### 2. friction 同定の更なる精度向上（優先度低）

現状で実用十分だが、より厳密な friction モデル化を求めるなら:

- **Stribeck モデル拡張**: Coulomb + 静止摩擦 + 速度依存項
- **定常速度スイープ専用モード追加**: 各 Iq で steady-state 速度を測定して τ-ω 曲線を直接観測
- **超低周波成分追加**: 0.1 Hz オーダーで明確に friction 領域に滞在

ただし RS-02 のように friction が小さい motor では、
**現在の同定値で実用上のシム転送性能は十分**と判断できる。

### 3. 励振信号設計のドキュメント化

[doc/sysid.md](sysid.md) の "非有理数比" の記述は、実際には
「特定 duration で整数周期化する設計の妙」が真実。
本レポートの試行 1 失敗が良い反例なので、参照リンクを追加する価値あり。

## 関連コミット

- `excitation.hpp + README` の修正: 後段のコミットで反映
- `recording_sysid_id11_v2.csv`, `identified_params_v2.json`, `validation_*_v2.png`: v2 成果物として別コミット

## 参考文献

- [Identification of Dynamic Parameters in a DC Motor Using Step and Ramp Torque Response Methods (PMC12788085)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12788085/)
- [A Review of Key Technologies for Friction Nonlinearity in Electro-Hydraulic Servo System (MDPI)](https://www.mdpi.com/2075-1702/10/7/568)
- [QDD Motors for Humanoid & Quadruped Robots (CubeMars)](https://www.cubemars.com/qdd-motors-for-humanoid-and-quadruped-robots.html)
- [MuJoCo dof_frictionloss gradient issue #1344](https://github.com/google-deepmind/mujoco/issues/1344)

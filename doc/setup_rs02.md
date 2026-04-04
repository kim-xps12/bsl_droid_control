```bash
# 現在のzero_sta値を表示
uv run examples/set_zero_sta.py  --interface can1_rs --motor-id 11

# zero_staを1に設定（-π～πモード）
uv run examples/set_zero_sta.py --interface can1_rs --motor-id 11 --set 1

# zero_staを0に設定（0～2πモード、デフォルト）
uv run examples/set_zero_sta.py --interface can1_rs --motor-id 11 --set 0
```

```basu
# zero_staを1に設定（-π～πモード）
uv run examples/set_zero_sta.py --interface can1_rs --motor-id 11 --set 1
uv run examples/set_zero_sta.py --interface can1_rs --motor-id 12 --set 1
uv run examples/set_zero_sta.py --interface can1_rs --motor-id 13 --set 1
uv run examples/set_zero_sta.py --interface can1_rs --motor-id 14 --set 1
uv run examples/set_zero_sta.py --interface can1_rs --motor-id 15 --set 1

uv run examples/set_zero_sta.py --interface can2_rs --motor-id 21 --set 1
uv run examples/set_zero_sta.py --interface can2_rs --motor-id 22 --set 1
uv run examples/set_zero_sta.py --interface can2_rs --motor-id 23 --set 1
uv run examples/set_zero_sta.py --interface can2_rs --motor-id 24 --set 1
uv run examples/set_zero_sta.py --interface can2_rs --motor-id 25 --set 1
```


```bash
# 現在のzero_sta値を表示
uv run examples/set_zero_sta.py  --interface can1_rs --motor-id 11
uv run examples/set_zero_sta.py  --interface can1_rs --motor-id 12
uv run examples/set_zero_sta.py  --interface can1_rs --motor-id 13
uv run examples/set_zero_sta.py  --interface can1_rs --motor-id 14
uv run examples/set_zero_sta.py  --interface can1_rs --motor-id 15

uv run examples/set_zero_sta.py  --interface can2_rs --motor-id 21
uv run examples/set_zero_sta.py  --interface can2_rs --motor-id 22
uv run examples/set_zero_sta.py  --interface can2_rs --motor-id 23
uv run examples/set_zero_sta.py  --interface can2_rs --motor-id 24
uv run examples/set_zero_sta.py  --interface can2_rs --motor-id 25
```

```bash
# ゼロ点セット
uv run examples/set_custom_zero.py --interface can1_rs --motor-id 11
uv run examples/set_custom_zero.py --interface can1_rs --motor-id 14
uv run examples/set_custom_zero.py --interface can1_rs --motor-id 15

uv run examples/set_custom_zero.py --interface can2_rs --motor-id 22
uv run examples/set_custom_zero.py --interface can2_rs --motor-id 23
uv run examples/set_custom_zero.py --interface can2_rs --motor-id 24
uv run examples/set_custom_zero.py --interface can2_rs --motor-id 25
```

```bash
# ゼロ点移動
uv run examples/move_to_zero_pp.py --interface can1_rs  --motor-id 11
uv run examples/move_to_zero_pp.py --interface can1_rs  --motor-id 12
uv run examples/move_to_zero_pp.py --interface can1_rs  --motor-id 13
uv run examples/move_to_zero_pp.py --interface can1_rs  --motor-id 14
uv run examples/move_to_zero_pp.py --interface can1_rs  --motor-id 15

uv run examples/move_to_zero_pp.py --interface can2_rs  --motor-id 21
uv run examples/move_to_zero_pp.py --interface can2_rs  --motor-id 22
uv run examples/move_to_zero_pp.py --interface can2_rs  --motor-id 23
uv run examples/move_to_zero_pp.py --interface can2_rs  --motor-id 24
uv run examples/move_to_zero_pp.py --interface can2_rs  --motor-id 25
```
---


uv run examples/scan_motors.py --interface can1_rs --start 11 --end 15
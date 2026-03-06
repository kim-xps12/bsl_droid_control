"""
BSL-Droid強化学習環境パッケージ

統一版の環境を提供します。
biped_env.py と droid_env.py にすべての報酬関数が統合されています。
train側の reward_scales で使用する報酬を選択します。
"""

# Biped environments (統一版)
from .biped_env import BipedEnv

# Droid environments (統一版)
from .droid_env import DroidEnv


__all__ = [
    # Biped (統一版)
    "BipedEnv",
    # Droid (統一版)
    "DroidEnv",
]

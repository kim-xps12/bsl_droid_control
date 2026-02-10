"""Logicool Wireless Gamepad F710 入力確認スクリプト。

使い方:
    cd rl_ws
    uv run python scripts/check_f710_input.py --list
    uv run python scripts/check_f710_input.py --device-index 0 --interval 0.1 --deadzone 0.08
    uv run python scripts/check_f710_input.py --duration 10
    uv run python scripts/check_f710_input.py --backend usb  # pyusb 直接読み取り

説明:
    - F710 の軸(axes)・ボタン(buttons)・ハット(hats)の状態をコンソールに表示する。
    - `--list` で接続デバイス一覧を確認できる。
    - 既定では指定デバイスのスナップショットを一定間隔で更新表示する。
    - 終了は Ctrl+C。

バックエンド:
    - ``pygame`` (デフォルト): pygame-ce (SDL 2.32+) の HIDAPI Xbox ドライバ経由で
      ジョイスティックを読み取る。macOS でも F710 を検出可能（条件付き、後述）。
      ※ 標準 pygame (SDL 2.28) では macOS 上で F710 は検出不可。
    - ``usb``: pyusb + libusb 経由で XInput プロトコルを直接読み取る。
      pygame バックエンドで検出できない場合のフォールバック。
    - ``auto`` (デフォルト動作): pygame を試し、検出 0 なら usb にフォールバック。

macOS での注意:
    F710 レシーバーは bDeviceClass=255 (Vendor Specific) で登録されるため、
    macOS の IOKit HID には不可視であり、通常の SDL2 ジョイスティック検出では認識されない。
    pygame-ce (SDL 2.32+) の HIDAPI バックエンドが libusb 経由で直接アクセスすることで
    この制約を回避できるが、F710 のスイッチ位置・ペアリング方法により検出可否が変わる。

    pygame バックエンドで検出可能な条件 (実機検証済み):
        1. F710 本体裏面のスイッチを **D** (DirectInput) に設定
        2. USB レシーバーを MacBook に接続する際に **Logicool ボタンを押下し続ける**
        3. この状態で接続すると "Logicool Cordless RumblePad 2" として検出される
           (axes=4, buttons=12, hats=1)

    上記以外のモード (X モード、Logicool ボタン未押下での接続等) では pygame-ce でも
    検出されない。その場合は pyusb バックエンドへ自動フォールバックする。
    pyusb バックエンドはスイッチ位置・接続方法に関わらず、USB デバイスとして
    認識さえされていれば XInput プロトコルで直接読み取り可能。

    - USB PID (0xc21f) は D/X モードとも同一。モード判別は bInterfaceSubClass で行う。
    - pyusb バックエンドの利用には `pip install pyusb` + `brew install libusb` が必要。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

import pygame
from f710_usb import _HAS_PYUSB, F710UsbGamepad


# F710 の USB ID
_LOGITECH_VID = 0x046D
_F710_PID = 0xC21F  # D/X モード共通 (モード切替は bInterfaceSubClass で区別)
_XINPUT_SUBCLASS = 0x5D  # XInput モード時の bInterfaceSubClass

# macOS 用 SDL ヒント: pygame-ce (SDL 2.32+) の HIDAPI Xbox ドライバで F710 を検出
_MACOS_SDL_HINTS: dict[str, str] = {
    "SDL_JOYSTICK_MFI": "0",  # MFi (Made For iPhone) を無効化
    "SDL_JOYSTICK_HIDAPI": "1",  # HIDAPI バックエンドを有効化
    "SDL_JOYSTICK_HIDAPI_XBOX": "1",  # Xbox コントローラ HIDAPI を有効化
    "SDL_JOYSTICK_HIDAPI_XBOX_360": "1",  # Xbox 360 (F710 互換) を有効化
}


def parse_args() -> argparse.Namespace:
    """コマンドライン引数をパースして返す.

    Returns:
        argparse.Namespace: パース済み引数。主要な属性:
            - device_index (int): pygame ジョイスティックインデックス (デフォルト: 0)
            - interval (float): 表示更新間隔 [秒] (デフォルト: 0.1)
            - deadzone (float): 軸のデッドゾーン閾値 0.0-1.0 (デフォルト: 0.08)
            - duration (float): 実行時間上限 [秒]。0 以下で無制限 (デフォルト: 0.0)
            - list (bool): True ならデバイス一覧表示のみ
            - backend (str): ``"auto"`` | ``"pygame"`` | ``"usb"``
    """
    parser = argparse.ArgumentParser(description="F710 の入力状態をコンソール表示する")
    parser.add_argument("--device-index", type=int, default=0, help="使用するジョイスティック index")
    parser.add_argument("--interval", type=float, default=0.1, help="表示更新間隔 [s]")
    parser.add_argument("--deadzone", type=float, default=0.08, help="軸のデッドゾーン (0.0-1.0)")
    parser.add_argument("--duration", type=float, default=0.0, help="実行時間 [s] (0以下で無制限)")
    parser.add_argument("--list", action="store_true", help="接続中デバイス一覧を表示して終了")
    parser.add_argument(
        "--backend",
        choices=["auto", "pygame", "usb"],
        default="auto",
        help="入力バックエンド (auto: pygame → usb フォールバック)",
    )
    return parser.parse_args()


def format_axes(values: list[float], deadzone: float) -> str:
    """軸の値をフォーマットしてコンソール表示用文字列にする.

    デッドゾーン内の値は 0.0 に丸められる。
    出力例: ``"a0:+0.123 a1:-0.456 a2:+0.000 a3:+0.789"``

    Args:
        values: 各軸の現在値 (-1.0 .. +1.0)。リストのインデックスが軸番号に対応。
        deadzone: この絶対値以下の入力を 0.0 に丸める閾値 (0.0-1.0)。

    Returns:
        スペース区切りの ``"a{idx}:{value:+.3f}"`` 形式文字列。
    """
    formatted: list[str] = []
    for idx, value in enumerate(values):
        v = 0.0 if abs(value) < deadzone else value
        formatted.append(f"a{idx}:{v:+.3f}")
    return " ".join(formatted)


def format_buttons(values: list[bool]) -> str:
    """押下中のボタンインデックスをフォーマットする.

    出力例: ``"pressed=0,3,7"`` (ボタン 0, 3, 7 が押下中) / ``"pressed=-"`` (なし)

    Args:
        values: 各ボタンの押下状態。インデックスが pygame のボタン番号に対応。

    Returns:
        ``"pressed="`` に続いてカンマ区切りのボタン番号、または ``"-"``。
    """
    pressed = [str(i) for i, is_pressed in enumerate(values) if is_pressed]
    return "pressed=" + (",".join(pressed) if pressed else "-")


def format_hats(values: list[tuple[int, int]]) -> str:
    """ハットスイッチ (D-Pad) の状態をフォーマットする.

    pygame の ``get_hat()`` は ``(x, y)`` タプルを返す:
        - x: -1 (左), 0 (中立), +1 (右)
        - y: -1 (下), 0 (中立), +1 (上)

    出力例: ``"h0:(0, 1)"`` (上入力) / ``"h0:-"`` (ハットなし)

    Args:
        values: 各ハットの ``(x, y)`` タプル。F710 はハット 1 個。

    Returns:
        スペース区切りの ``"h{idx}:{(x,y)}"`` 形式文字列。
    """
    return " ".join(f"h{i}:{xy}" for i, xy in enumerate(values)) if values else "h0:-"


def list_devices() -> None:
    """接続中のジョイスティック一覧を表示する.

    pygame バックエンドと pyusb バックエンドの両方の検出結果を表示する。
    pygame で検出 0 かつ macOS の場合は ``_diagnose_macos_no_joystick()`` で
    追加診断を実行する。

    前提:
        ``pygame.init()`` / ``pygame.joystick.init()`` が呼び出し済みであること。
        macOS では ``_MACOS_SDL_HINTS`` が環境変数に設定済みであること。
    """
    count = pygame.joystick.get_count()
    print(f"[pygame] Detected joystick count: {count}")
    for idx in range(count):
        joy = pygame.joystick.Joystick(idx)
        print(
            f"  [{idx}] name='{joy.get_name()}' guid='{joy.get_guid()}' "
            f"axes={joy.get_numaxes()} buttons={joy.get_numbuttons()} hats={joy.get_numhats()}"
        )

    # pyusb バックエンドの状態も表示
    print()
    if _HAS_PYUSB:
        if F710UsbGamepad.available():
            print("[pyusb] F710 検出: USB 直接読み取り可能 (--backend usb で使用)")
            gp = F710UsbGamepad.open()
            if gp:
                print(f"  name='{gp.name}' serial='{gp.serial_number}'")
                gp.close()
        else:
            print("[pyusb] F710 が USB デバイスとして見つかりません")
    else:
        print("[pyusb] pyusb 未インストール (pip install pyusb)")

    if count == 0 and sys.platform == "darwin":
        _diagnose_macos_no_joystick()


def _diagnose_macos_no_joystick() -> None:
    """macOS でジョイスティックが検出されなかった場合の診断を行う.

    以下の手順で原因を切り分ける:

    1. ``system_profiler SPUSBDataType`` で USB デバイスツリーを取得
    2. F710 / Logitech 関連キーワードで USB レベルの検出状態を確認
    3. pyusb が利用可能な場合、bInterfaceSubClass から XInput モードを判定

    技術的背景:
        F710 レシーバーは bDeviceClass=255 (Vendor Specific) で列挙される。
        macOS は bDeviceClass=3 (HID) のみ IOKit HID デバイスとして公開するため、
        クラス 255 のデバイスは SDL2 の IOKit バックエンドから不可視となる。
        pygame-ce (SDL 2.32+) の HIDAPI バックエンドは libusb 経由で直接
        デバイスにアクセスするため、この制約を回避できる。
    """
    print()
    print("--- macOS diagnostic ---")
    try:
        result = subprocess.run(
            ["system_profiler", "SPUSBDataType"],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print("  system_profiler を実行できませんでした。")
        return

    lines = result.stdout.splitlines()
    f710_context: list[str] = []
    for i, line in enumerate(lines):
        low = line.lower()
        if any(kw in low for kw in ["f710", "gamepad", "logicool", "logitech"]):
            start = max(0, i - 1)
            end = min(len(lines), i + 6)
            for ctx in lines[start:end]:
                if ctx not in f710_context:
                    f710_context.append(ctx)

    if not f710_context:
        print("  F710 / Logitech ゲームパッドが USB デバイスとして検出されていません。")
        print("  → USBレシーバーが正しく接続されているか確認してください。")
        print("  → F710 の電源が入っているか確認してください（Logicool ボタンが点灯）。")
        return

    print("  USB デバイスとして検出されています:")
    for line in f710_context:
        print(f"    {line}")
    print()
    print("  ℹ  F710 レシーバーは USB デバイスクラス 255 (Vendor Specific) で登録されます。")
    print("     macOS はこのクラスを HID デバイスとして認識しないため、")
    print("     pygame-ce (SDL 2.32+) の HIDAPI Xbox ドライバでのみ検出可能です。")
    print("     ※ 標準 pygame (SDL 2.28) では検出不可。")
    print()

    # pyusb で XInput モードか確認
    if _HAS_PYUSB:
        try:
            import usb.core as _usb_core

            dev = _usb_core.find(idVendor=_LOGITECH_VID, idProduct=_F710_PID)
            if dev is not None:
                cfg = dev[0]
                intf = cfg[(0, 0)]
                subclass = intf.bInterfaceSubClass
                if subclass == _XINPUT_SUBCLASS:
                    print(f"  ✓ XInput モード確認済み (bInterfaceSubClass=0x{subclass:02x})")
                    print("  → pyusb バックエンドで読み取り可能です:")
                    print("    uv run python scripts/check_f710_input.py --backend usb")
                else:
                    print(f"  ⚠ bInterfaceSubClass=0x{subclass:02x} (XInput ではない)")
                    print("  → F710 本体裏面のスイッチを 'X' に切り替えてください。")
        except Exception:
            pass
    else:
        print("  → pyusb をインストールすると USB 直接読み取りが可能です:")
        print("    pip install pyusb && brew install libusb")


def run_monitor(device_index: int, interval: float, deadzone: float, duration: float) -> int:
    """pygame バックエンドでジョイスティック入力をリアルタイムモニターする.

    指定デバイスの全軸・全ボタン・全ハットの状態を ``interval`` 秒ごとに
    1 行ずつコンソールに出力する。Ctrl+C で中断可能。

    Args:
        device_index: pygame ジョイスティックインデックス (0-based)。
        interval: 表示更新間隔 [秒]。最小 0.01 秒にクランプされる。
        deadzone: 軸のデッドゾーン閾値 (0.0-1.0)。
        duration: 実行時間上限 [秒]。0 以下で無制限 (Ctrl+C のみで停止)。

    Returns:
        終了コード。0=正常終了、1=エラー (デバイス未検出、インデックス範囲外)。

    前提:
        ``pygame.init()`` / ``pygame.joystick.init()`` が呼び出し済みであること。
    """
    count = pygame.joystick.get_count()
    if count == 0:
        print("No joystick detected. F710 receiver connection and powerを確認してください。")
        if sys.platform == "darwin":
            _diagnose_macos_no_joystick()
        return 1
    if not 0 <= device_index < count:
        print(f"Invalid --device-index={device_index}. Available range: 0..{count - 1}")
        return 1

    joy = pygame.joystick.Joystick(device_index)
    print("=== Joystick opened (pygame backend) ===")
    print(f"index          : {device_index}")
    print(f"name           : {joy.get_name()}")
    print(f"guid           : {joy.get_guid()}")
    print(f"instance_id    : {joy.get_instance_id()}")
    print(f"power_level    : {joy.get_power_level()}")
    print(f"num_axes       : {joy.get_numaxes()}")
    print(f"num_buttons    : {joy.get_numbuttons()}")
    print(f"num_hats       : {joy.get_numhats()}")
    print(f"update_interval: {interval:.3f}s")
    print(f"deadzone       : {deadzone:.3f}")
    print("Press Ctrl+C to stop.")
    print()

    start = time.monotonic()
    try:
        while True:
            pygame.event.pump()

            axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
            buttons = [bool(joy.get_button(i)) for i in range(joy.get_numbuttons())]
            hats: list[tuple[int, int]] = [
                (int(h[0]), int(h[1])) for h in (joy.get_hat(i) for i in range(joy.get_numhats()))
            ]

            timestamp = time.monotonic() - start
            print(f"[{timestamp:8.3f}] {format_axes(axes, deadzone)} | {format_buttons(buttons)} | {format_hats(hats)}")

            if duration > 0 and timestamp >= duration:
                break
            time.sleep(max(interval, 0.01))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    return 0


def run_monitor_usb(interval: float, deadzone: float, duration: float) -> int:
    """pyusb バックエンドで F710 (XInput) を読み取るモニター.

    ``F710UsbGamepad`` クラスを使い、USB エンドポイントから XInput パケットを
    直接読み取って表示する。pygame-ce が利用できない環境でのフォールバック。

    XInput プロトコルでは以下の 6 軸を提供:
        LX, LY (左スティック), RX, RY (右スティック), LT, RT (トリガー)

    Args:
        interval: 表示更新間隔 [秒]。USB read のタイムアウトとしても使用。
        deadzone: 軸のデッドゾーン閾値 (0.0-1.0)。
        duration: 実行時間上限 [秒]。0 以下で無制限。

    Returns:
        終了コード。0=正常終了、1=エラー (pyusb 未インストール、F710 未検出)。
    """
    if not _HAS_PYUSB:
        print("pyusb が未インストールです: pip install pyusb")
        return 1

    gp = F710UsbGamepad.open()
    if gp is None:
        print("F710 が USB デバイスとして検出されないか、XInput モードではありません。")
        print("→ F710 本体裏面のスイッチを 'X' に設定してください。")
        return 1

    print("=== Joystick opened (pyusb/XInput backend) ===")
    print(f"name           : {gp.name}")
    print(f"serial         : {gp.serial_number}")
    print("axes           : 6 (LX, LY, RX, RY, LT, RT)")
    print("buttons        : 15 (XInput)")
    print("hats           : 1 (D-Pad)")
    print(f"update_interval: {interval:.3f}s")
    print(f"deadzone       : {deadzone:.3f}")
    print("Press Ctrl+C to stop.")
    print()

    start = time.monotonic()
    try:
        while True:
            state = gp.read(timeout_ms=max(int(interval * 1000), 10))
            timestamp = time.monotonic() - start

            if state is not None:
                axes = state.axes
                axes_str = format_axes(axes, deadzone)
                btn_str = "pressed=" + (",".join(state.buttons_pressed) if state.buttons_pressed else "-")
                hat_str = f"h0:{state.dpad}"
                print(f"[{timestamp:8.3f}] {axes_str} | {btn_str} | {hat_str}")

            if duration > 0 and timestamp >= duration:
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        gp.close()

    return 0


def main() -> int:
    """エントリーポイント: 引数パース → SDL 初期化 → バックエンド選択 → 実行.

    処理フロー:
        1. コマンドライン引数をパース
        2. macOS の場合、SDL ヒント環境変数を ``pygame.init()`` の **前** に設定
           (HIDAPI Xbox ドライバを有効化して F710 を検出可能にする)
        3. ``pygame.init()`` / ``pygame.joystick.init()`` を実行
        4. ``--list`` なら ``list_devices()`` で一覧表示して終了
        5. ``--backend auto`` の場合: pygame 検出 > 0 なら pygame、
           そうでなければ pyusb にフォールバック
        6. 選択されたバックエンドでモニターループを実行

    Returns:
        終了コード (0=正常, 1=エラー)。``SystemExit`` 経由で返される。

    Note:
        SDL ヒントは ``pygame.init()`` の前に設定する必要がある。
        初期化後に設定しても SDL のジョイスティックサブシステムには反映されない。
    """
    args = parse_args()

    # macOS: pygame.init() の前に SDL ヒントを設定し HIDAPI Xbox ドライバを有効化
    if sys.platform == "darwin":
        for key, value in _MACOS_SDL_HINTS.items():
            os.environ.setdefault(key, value)

    pygame.init()
    pygame.joystick.init()
    try:
        if args.list:
            list_devices()
            return 0

        backend = args.backend

        # auto モード: pygame を試し、検出 0 なら usb にフォールバック
        if backend == "auto":
            if pygame.joystick.get_count() > 0:
                backend = "pygame"
            elif _HAS_PYUSB and F710UsbGamepad.available():
                print("pygame で検出 0 → pyusb (XInput) バックエンドにフォールバック")
                print()
                backend = "usb"
            else:
                backend = "pygame"  # エラーメッセージ表示のため

        if backend == "usb":
            return run_monitor_usb(args.interval, args.deadzone, args.duration)
        return run_monitor(args.device_index, args.interval, args.deadzone, args.duration)
    finally:
        pygame.joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    raise SystemExit(main())

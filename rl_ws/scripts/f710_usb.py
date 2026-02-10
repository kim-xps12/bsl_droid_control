"""Logitech F710 ゲームパッド - pyusb 直接読み取りバックエンド.

macOS では F710 レシーバーが USB デバイスクラス 255 (Vendor Specific) で登録されるため、
IOKit HID / SDL2 / pygame のいぞれもジョイスティックとして認識しない。
本モジュールは pyusb (libusb) 経由で XInput プロトコルを直接パースすることで、
macOS 上でも F710 の入力を読み取れるようにする。

技術的背景:
    - F710 は XInput モード (X) と DirectInput モード (D) を持つ。
      本モジュールは XInput モードのみサポート。
    - USB PID (0xc21f) は両モードで同一。モード判別は
      bInterfaceSubClass=0x5d (XInput) で行う。
    - XInput パケットは 20 バイト固定長。フォーマットは
      ``_parse()`` メソッドの docstring を参照。
    - 推奨構成: pygame-ce (SDL 2.32+) をプライマリ、
      本モジュールをフォールバックとして併用する。

依存:
    pip install pyusb
    brew install libusb  (macOS)

使い方::

    # 基本的な使い方
    gamepad = F710UsbGamepad.open()
    if gamepad is not None:
        state = gamepad.read()
        if state is not None:
            print(state.left_x, state.buttons_pressed)
        gamepad.close()

    # context manager で自動クリーンアップ
    with F710UsbGamepad.open() as gp:
        if gp is not None:
            while True:
                state = gp.read(timeout_ms=100)
                if state:
                    print(state.axes)

プロジェクト統合ガイド:
    ロボット制御プロジェクトでこのモジュールを利用する場合:
    1. ``F710UsbGamepad.available()`` で接続確認
    2. ``open()`` でインターフェース取得 (None チェック必須)
    3. 制御ループ内で ``read(timeout_ms=...)`` を呼び出し
    4. ``GamepadState.axes`` で [LX, LY, RX, RY, LT, RT] を取得
    5. ``GamepadState.buttons_pressed`` で押下中ボタン名リストを取得
    6. ループ終了時に ``close()`` で USB インターフェースを解放
"""

from __future__ import annotations

import contextlib
import struct
from dataclasses import dataclass, field
from typing import ClassVar


try:
    import usb.core
    import usb.util

    _HAS_PYUSB = True
except ImportError:
    _HAS_PYUSB = False


# ---------------------------------------------------------------------------
# 定数
# ---------------------------------------------------------------------------

_LOGITECH_VID = 0x046D
_F710_PID = 0xC21F  # D/X モード共通 (モード切替は USB PID に反映されない)
_XINPUT_SUBCLASS = 0x5D  # bInterfaceSubClass: XInput プロトコル

# XInput ボタンマスク (16-bit bitmask)
BTN_DPAD_UP: int = 0x0001
BTN_DPAD_DOWN: int = 0x0002
BTN_DPAD_LEFT: int = 0x0004
BTN_DPAD_RIGHT: int = 0x0008
BTN_START: int = 0x0010
BTN_BACK: int = 0x0020
BTN_LEFT_THUMB: int = 0x0040
BTN_RIGHT_THUMB: int = 0x0080
BTN_LB: int = 0x0100
BTN_RB: int = 0x0200
BTN_GUIDE: int = 0x0400  # Logitech ボタン
BTN_A: int = 0x1000
BTN_B: int = 0x2000
BTN_X: int = 0x4000
BTN_Y: int = 0x8000

BUTTON_NAMES: dict[int, str] = {
    BTN_DPAD_UP: "DPAD_UP",
    BTN_DPAD_DOWN: "DPAD_DOWN",
    BTN_DPAD_LEFT: "DPAD_LEFT",
    BTN_DPAD_RIGHT: "DPAD_RIGHT",
    BTN_START: "START",
    BTN_BACK: "BACK",
    BTN_LEFT_THUMB: "L_THUMB",
    BTN_RIGHT_THUMB: "R_THUMB",
    BTN_LB: "LB",
    BTN_RB: "RB",
    BTN_GUIDE: "GUIDE",
    BTN_A: "A",
    BTN_B: "B",
    BTN_X: "X",
    BTN_Y: "Y",
}


# ---------------------------------------------------------------------------
# データ構造
# ---------------------------------------------------------------------------


@dataclass
class GamepadState:
    """ゲームパッドの 1 フレーム分の状態.

    XInput プロトコルの 20 バイトパケットをパースした結果を格納する。
    ``__post_init__`` で派生フィールド (``buttons_pressed``, ``axes``, ``dpad``) が
    自動計算される。

    属性:
        left_x, left_y: 左スティック (-1.0 .. +1.0)
        right_x, right_y: 右スティック (-1.0 .. +1.0)
        left_trigger, right_trigger: トリガー (0.0 .. 1.0)
        buttons_raw: ボタンビットマスク (16-bit)。``BTN_*`` 定数でデコード。
        dpad: D-Pad 状態 ``(x, y)``。pygame hat 互換。
            x: -1=左, 0=中立, +1=右 / y: -1=下, 0=中立, +1=上
        buttons_pressed: 押下中ボタン名のリスト (e.g. ``["A", "LB"]``)
        axes: pygame 互換の軸リスト ``[LX, LY, RX, RY, LT, RT]``。
            ロボット制御ループではこのリストをそのまま使用可能。
        msg_type: XInput パケットのメッセージタイプ (0x00=入力, 0x01=LED 等)。

    例::

        state = gamepad.read()
        if state:
            # 左スティックで速度指令
            cmd_vx = state.left_y * max_speed
            cmd_vy = state.left_x * max_speed
            # A ボタンでアクション
            if "A" in state.buttons_pressed:
                trigger_action()
    """

    # アナログスティック (-1.0 .. +1.0)
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0

    # トリガー (0.0 .. 1.0)
    left_trigger: float = 0.0
    right_trigger: float = 0.0

    # ボタン (16-bit bitmask)
    buttons_raw: int = 0

    # D-Pad をタプルで提供 (hat 互換: (x, y), x: -1=left,+1=right, y: -1=down,+1=up)
    dpad: tuple[int, int] = (0, 0)

    # 名前解決済みの押下ボタン一覧
    buttons_pressed: list[str] = field(default_factory=list)

    # pygame 互換: axes リスト [LX, LY, RX, RY, LT, RT]
    # ※ pygame の F710 マッピングと同一順序
    axes: list[float] = field(default_factory=list)

    # XInput パケットのメッセージタイプ
    msg_type: int = 0

    def __post_init__(self) -> None:
        if not self.buttons_pressed:
            self.buttons_pressed = [name for mask, name in BUTTON_NAMES.items() if self.buttons_raw & mask]
        if not self.axes:
            self.axes = [
                self.left_x,
                self.left_y,
                self.right_x,
                self.right_y,
                self.left_trigger,
                self.right_trigger,
            ]
        if self.dpad == (0, 0):
            dx = 0
            if self.buttons_raw & BTN_DPAD_LEFT:
                dx = -1
            elif self.buttons_raw & BTN_DPAD_RIGHT:
                dx = 1
            dy = 0
            if self.buttons_raw & BTN_DPAD_DOWN:
                dy = -1
            elif self.buttons_raw & BTN_DPAD_UP:
                dy = 1
            self.dpad = (dx, dy)


# ---------------------------------------------------------------------------
# メインクラス
# ---------------------------------------------------------------------------


class F710UsbGamepad:
    """pyusb 経由で F710 (XInput) を読み取るクラス.

    ファクトリメソッド ``open()`` でインスタンスを取得し、
    ``read()`` で XInput パケットを 1 つずつ読み取る。
    context manager プロトコルをサポートしており、``with`` 文で自動クリーンアップ可能。

    USB セットアップ手順 (``open()`` 内部):
        1. ``usb.core.find()`` で VID=0x046d / PID=0xc21f を検索
        2. ``bInterfaceSubClass == 0x5d`` (XInput) を確認
        3. IN エンドポイントを取得
        4. ``set_configuration()`` / ``claim_interface()`` で USB インターフェースを確保

    スレッド安全性:
        スレッドセーフではない。単一スレッドからのみ使用すること。
    ロボット制御ループに組み込む場合は専用スレッドで read し、
    キュー経由で制御スレッドに渡す設計を推奨。

    使い方::

        # 基本パターン
        gp = F710UsbGamepad.open()
        if gp:
            while True:
                state = gp.read(timeout_ms=100)
                if state:
                    print(state.left_x, state.left_y)
            gp.close()

        # context manager パターン
        with F710UsbGamepad.open() as gp:
            if gp:
                state = gp.read()
    """

    # クラス定数
    VID: ClassVar[int] = _LOGITECH_VID
    PID: ClassVar[int] = _F710_PID

    def __init__(self, dev: usb.core.Device, ep_in: usb.core.Endpoint) -> None:
        """インスタンスを初期化する. 直接呼び出さず ``open()`` を使用すること.

        Args:
            dev: pyusb デバイスオブジェクト。
            ep_in: 読み取り用 IN エンドポイント。
        """
        self._dev = dev
        self._ep_in = ep_in
        self._closed = False

    # ----- ファクトリ -----

    @staticmethod
    def available() -> bool:
        """pyusb が利用可能かつ F710 が接続されているか.

        XInput モードかどうかは検査しない。USB デバイスとしての
        存在確認のみ。詳細なモード判定は ``open()`` が行う。

        Returns:
            pyusb がインストール済みかつ VID:PID 一致デバイスが見つかれば True。
        """
        if not _HAS_PYUSB:
            return False
        dev = usb.core.find(idVendor=_LOGITECH_VID, idProduct=_F710_PID)
        return dev is not None

    @staticmethod
    def open() -> F710UsbGamepad | None:
        """F710 を検出して USB インターフェースを取得する.

        検出・セットアップの流れ:
            1. VID=0x046d / PID=0xc21f で USB デバイスを検索
            2. bInterfaceSubClass=0x5d (XInput) か確認
               → D モードや非 XInput デバイスの場合 None を返す
            3. IN エンドポイントを取得
            4. set_configuration / detach_kernel_driver / claim_interface

        Returns:
            ``F710UsbGamepad`` インスタンス。未検出または非 XInput なら None。

        Note:
            macOS ではカーネルドライバが存在しないため
            ``detach_kernel_driver`` は NotImplementedError になるが、
            内部で抑制済み。
        """
        if not _HAS_PYUSB:
            return None

        dev = usb.core.find(idVendor=_LOGITECH_VID, idProduct=_F710_PID)
        if dev is None:
            return None

        # XInput インターフェースか確認
        cfg = dev[0]
        intf = cfg[(0, 0)]
        if intf.bInterfaceSubClass != _XINPUT_SUBCLASS:
            return None  # D モードまたは非 XInput

        # IN エンドポイントを取得
        ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN,
        )
        if ep_in is None:
            return None

        # USB セットアップ
        with contextlib.suppress(usb.core.USBError):
            dev.set_configuration()

        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (usb.core.USBError, NotImplementedError):
            pass

        try:
            usb.util.claim_interface(dev, 0)
        except usb.core.USBError:
            return None

        return F710UsbGamepad(dev, ep_in)

    # ----- 読み取り -----

    def read(self, timeout_ms: int = 100) -> GamepadState | None:
        """XInput パケットを 1 つ読み取る.

        USB エンドポイントからブロッキング読み取りを行い、
        XInput プロトコルの 20 バイトパケットを ``GamepadState`` にパースする。

        Args:
            timeout_ms: USB read タイムアウト [ミリ秒]。デフォルト 100ms。
                制御ループの周期に合わせて調整すること。
                例: 200Hz 制御なら timeout_ms=5 程度。

        Returns:
            ``GamepadState`` インスタンス。タイムアウトまたは USB エラー時は None。
            close 済みの場合も None。
        """
        if self._closed:
            return None
        try:
            data = bytes(self._ep_in.read(self._ep_in.wMaxPacketSize, timeout=timeout_ms))
        except usb.core.USBTimeoutError:
            return None
        except usb.core.USBError:
            return None

        return self._parse(data)

    # ----- クリーンアップ -----

    def close(self) -> None:
        """USB インターフェースを解放する.

        ``release_interface`` と ``dispose_resources`` を実行する。
        多重呼び出しは安全 (庂等)。エラーは内部で抑制される。
        """
        if self._closed:
            return
        self._closed = True
        with contextlib.suppress(usb.core.USBError):
            usb.util.release_interface(self._dev, 0)
        with contextlib.suppress(usb.core.USBError):
            usb.util.dispose_resources(self._dev)

    def __enter__(self) -> F710UsbGamepad:
        """context manager エントリ. 自身を返す."""
        return self

    def __exit__(self, *_: object) -> None:
        """context manager イグジット. ``close()`` を呼び出す."""
        self.close()

    # ----- 内部 -----

    @staticmethod
    def _parse(data: bytes) -> GamepadState | None:
        """XInput パケットをパースして GamepadState を返す.

        パケット構造 (20バイト):
          [0]    msg_type
          [1]    length (0x14 = 20)
          [2:4]  buttons  (uint16 LE)
          [4]    left_trigger  (uint8, 0-255)
          [5]    right_trigger (uint8, 0-255)
          [6:8]  left_x   (int16 LE, -32768..32767)
          [8:10] left_y   (int16 LE)
          [10:12] right_x  (int16 LE)
          [12:14] right_y  (int16 LE)
        """
        if len(data) < 14:
            return None

        length = data[1]
        if length < 0x14:
            return None

        buttons = struct.unpack_from("<H", data, 2)[0]
        lt = data[4]
        rt = data[5]
        lx, ly, rx, ry = struct.unpack_from("<4h", data, 6)

        return GamepadState(
            left_x=lx / 32768.0,
            left_y=ly / 32768.0,
            right_x=rx / 32768.0,
            right_y=ry / 32768.0,
            left_trigger=lt / 255.0,
            right_trigger=rt / 255.0,
            buttons_raw=buttons,
            msg_type=data[0],
        )

    @property
    def name(self) -> str:
        """USB デバイス名.

        USB デスクリプタから取得。失敗時は ``"F710"`` を返す。
        実機では ``"Logicool Cordless RumblePad 2"`` 等が返る。
        """
        try:
            return str(self._dev.product) if self._dev.product else "F710"
        except usb.core.USBError:
            return "F710"

    @property
    def serial_number(self) -> str | None:
        """USB シリアル番号.

        USB デスクリプタから取得。未設定または取得失敗時は None。
        複数の F710 を区別する場合に利用可能。
        """
        try:
            return str(self._dev.serial_number) if self._dev.serial_number else None
        except usb.core.USBError:
            return None

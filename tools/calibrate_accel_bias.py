#!/usr/bin/env python3
"""Guided six-orientation accelerometer bias calibration of the localizer.

Drives the serial calibration mode of the normal MainCore build (not the
RAW_LOG_OUTPUT build): sends C, shows each orientation request, waits for
you to press Enter, sends Y, and reports the result that the board stored
in its flash.

Usage: calibrate_accel_bias.py [PORT] [--baud 115200] [--no-usb-reset]
       calibrate_accel_bias.py PORT --show     (only print the stored preset)
       calibrate_accel_bias.py PORT --erase    (erase the stored preset)
"""
import argparse
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ORIENTATION_HINT = {
    1: "基板を水平に、部品面を上にして置く（+Z 上）",
    2: "基板を裏返して置く（-Z 上）",
    3: "基板を -X 側の縁で立てる（+X 上）",
    4: "基板を +X 側の縁で立てる（-X 上）",
    5: "基板を -Y 側の縁で立てる（+Y 上）",
    6: "基板を +Y 側の縁で立てる（-Y 上）",
}


class Board:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.pending = b""

    def close(self):
        self.ser.close()

    def send(self, c):
        self.ser.write(c.encode())
        self.ser.flush()

    def lines(self, timeout):
        """Yield '#' lines (board messages), already buffered ones first,
        then those received within timeout seconds."""
        t0 = time.time()
        while True:
            while b"\n" in self.pending:
                line, self.pending = self.pending.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text.startswith("#"):
                    yield text
            if time.time() - t0 >= timeout:
                return
            data = self.ser.read(8192)
            if data:
                self.pending += data

    def wait_for(self, prefixes, timeout):
        """Return the first '#' line starting with one of prefixes, or None."""
        for text in self.lines(timeout):
            for p in prefixes:
                if text.startswith(p):
                    return text
        return None


def usb_reset(enabled):
    if not enabled or not sys.platform.startswith("linux"):
        return
    try:
        import usb_reset_cp210x
        usb_reset_cp210x.main([])
        time.sleep(2.0)
    except Exception as e:  # best effort
        print(f"usb reset skipped: {e}", file=sys.stderr)


def show_preset(board):
    board.send("P")
    line = board.wait_for(["# accel bias preset"], 3.0)
    print(line if line else "応答がありません。通常ビルドのMainCoreが動いていますか？（生データ記録ビルドではコマンドは無効です）")
    return line


def calibrate(board):
    board.send("C")
    prompt = board.wait_for(["# calibration 1/", "# accelerometer bias calibration"], 5.0)
    if prompt is None:
        print("較正モードに入れません。通常ビルドのMainCoreが動いているか、ポートとボーレートを確認してください。")
        return False
    if not prompt.startswith("# calibration 1/"):
        prompt = board.wait_for(["# calibration 1/"], 5.0)
        if prompt is None:
            print("姿勢の指示が届きません。")
            return False

    while True:
        # prompt looks like "# calibration N/6: place the sensor ... send Y (Q to abort)"
        try:
            index = int(prompt.split("calibration ")[1].split("/")[0])
        except (IndexError, ValueError):
            index = 0
        print()
        print(prompt)
        if index in ORIENTATION_HINT:
            print(f"  -> 姿勢 {index}/6: {ORIENTATION_HINT[index]}")
        try:
            answer = input("  静止させたら Enter（q で中断）: ").strip().lower()
        except EOFError:
            answer = ""
        if answer == "q":
            board.send("Q")
            print(board.wait_for(["# calibration aborted"], 3.0) or "# aborted")
            return False
        board.send("Y")
        line = board.wait_for(["# measuring"], 3.0)
        if line is None:
            print("計測が始まりません。もう一度 Enter を押してください。")
            continue
        print("  計測中（3秒）...")
        while True:
            line = board.wait_for(["# orientation", "# the sensor moved", "# result:", "# calibration", "# localizer"], 10.0)
            if line is None:
                print("ボードからの応答がありません。")
                return False
            if not line.startswith("# calibration "):
                print("  " + line)
            if line.startswith("# the sensor moved"):
                prompt = board.wait_for(["# calibration"], 3.0) or prompt
                break  # same orientation again
            if line.startswith("# orientation"):
                continue  # mean printed, next comes the prompt or the result
            if line.startswith("# result:"):
                if "FAILED" in line:
                    print("フラッシュへの保存に失敗しました。")
                    return False
                print("較正完了。値はフラッシュに保存され、次回起動から自動で使われます。")
                print("SubCore1 が再較正するので、センサを15秒間静止させてください。")
                board.wait_for(["# localizer restarting"], 3.0)
                return True
            if line.startswith("# calibration aborted"):
                print("較正は保存されませんでした。6つの向きが互いに異なっていたか確認して、やり直してください。")
                return False
            if line.startswith("# calibration"):
                prompt = line
                break  # next orientation


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-usb-reset", action="store_true")
    ap.add_argument("--show", action="store_true", help="print the stored preset and exit")
    ap.add_argument("--erase", action="store_true", help="erase the stored preset and exit")
    args = ap.parse_args()

    usb_reset(not args.no_usb_reset)
    board = Board(args.port, args.baud)
    try:
        print("ボードをリセットして接続中...")
        time.sleep(3.0)  # boot
        board.ser.reset_input_buffer()
        board.pending = b""
        if args.show:
            return 0 if show_preset(board) else 1
        if args.erase:
            board.send("X")
            line = board.wait_for(["# preset erased"], 3.0)
            print(line if line else "応答がありません。")
            return 0 if line else 1
        print("現在の保存値:")
        show_preset(board)
        print()
        print("6姿勢の較正を始めます。各姿勢で指示どおりに置いて静止させ、Enter を押してください。")
        ok = calibrate(board)
        if ok:
            print()
            show_preset(board)
        return 0 if ok else 1
    finally:
        board.close()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
scale_reader.py — reads weight from Arduino and serves it via a local WebSocket.
Run:  python scale_reader.py --port /dev/ttyUSB0 (Linux/Mac)
"""

import argparse
import asyncio
import json
import threading
import serial
import websockets

latest = {"grams": 0.0, "raw": 0.0, "status": "connecting"}

# ── Serial reader thread ──────────────────────────────────────
def read_serial(port: str, baud: int = 115200):
    try:
        ser = serial.Serial(port, baud, timeout=2)
        print(f"[serial] opened {port}")
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line == "ZEROING":
                latest["status"] = "zeroing"
            elif line == "READY":
                latest["status"] = "ready"
            elif line.startswith("W,"):
                parts = line.split(",")
                try:
                    latest["grams"] = float(parts[1])
                    latest["raw"]   = float(parts[2]) if len(parts) > 2 else latest["grams"]
                    latest["status"] = "ok"
                except (ValueError, IndexError):
                    pass
    except serial.SerialException as e:
        latest["status"] = f"error: {e}"
        print(f"[serial] {e}")

# ── WebSocket server ──────────────────────────────────────────
async def ws_handler(websocket):
    print("[ws] client connected")
    try:
        while True:
            await websocket.send(json.dumps(latest))
            await asyncio.sleep(0.2)
    except websockets.ConnectionClosed:
        print("[ws] client disconnected")

async def main(port: str):
    t = threading.Thread(target=read_serial, args=(port,), daemon=True)
    t.start()
    print("[ws] serving on ws://localhost:8765")
    async with websockets.serve(ws_handler, "localhost", 8765):
        await asyncio.Future()   # run forever

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/ttyUSB0")
    args = ap.parse_args()
    asyncio.run(main(args.port))
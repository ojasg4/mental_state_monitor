import asyncio
import json
import os
import re
import time
import webbrowser
import websockets
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "XIAO_PPG"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
WS_PORT     = 8765
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))

PATTERN = re.compile(r"IR:(\d+),BPM:(-?\d+),AvgBPM:(-?\d+),EDA:(-?\d+)")

ws_clients:    set                        = set()
data_queue:    asyncio.Queue              = None
session_start: float                      = None
main_loop:     asyncio.AbstractEventLoop  = None
buffer:        str                        = ""


async def ws_handler(websocket):
    ws_clients.add(websocket)
    print(f"Dashboard connected ({len(ws_clients)} client(s))")
    try:
        await websocket.wait_closed()
    finally:
        ws_clients.discard(websocket)
        print(f"Dashboard disconnected ({len(ws_clients)} client(s))")


async def broadcast_loop():
    while True:
        payload = await data_queue.get()
        dead = set()
        for ws in list(ws_clients):
            try:
                await ws.send(payload)
            except Exception:
                dead.add(ws)
        ws_clients.difference_update(dead)


def notification_handler(_sender, data):
    global session_start, buffer
    buffer += data.decode("utf-8", errors="ignore")

    while "\n" in buffer:
        line, buffer = buffer.split("\n", 1)
        line = line.strip()

        match = PATTERN.match(line)
        if not match:
            if line:
                print("Unparsed:", line)
            continue

        if session_start is None:
            session_start = time.time()
        elapsed = round(time.time() - session_start, 2)

        bpm = int(match.group(2))
        if bpm == 0:
            bpm = int(match.group(3))  # fall back to AvgBPM
        eda = int(match.group(4))

        print(f"  [{elapsed:.2f}s]  BPM={bpm}  EDA={eda}  clients={len(ws_clients)}")
        payload = json.dumps({"elapsed": elapsed, "hr": bpm, "sc": eda})
        main_loop.call_soon_threadsafe(data_queue.put_nowait, payload)


def disconnected_callback(_):
    print("BLE disconnected — will reconnect...")


async def connect_ble():
    while True:
        try:
            print(f"Scanning for '{DEVICE_NAME}'...")
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
            if not device:
                print(f"'{DEVICE_NAME}' not found, retrying in 5s...")
                await asyncio.sleep(5)
                continue

            print(f"Connecting to {device.address}...")
            async with BleakClient(device, timeout=20.0,
                                   disconnected_callback=disconnected_callback) as client:
                print("Connected!")
                await client.start_notify(NUS_TX_UUID, notification_handler)
                while client.is_connected:
                    await asyncio.sleep(1)

        except Exception as e:
            print(f"BLE error: {e} — retrying in 5s...")
            await asyncio.sleep(5)


async def main():
    global data_queue, main_loop
    main_loop = asyncio.get_running_loop()
    data_queue = asyncio.Queue()

    await websockets.serve(ws_handler, "localhost", WS_PORT)
    asyncio.ensure_future(broadcast_loop())

    dashboard = os.path.join(SCRIPT_DIR, "dashboard.html")
    webbrowser.open(f"file://{dashboard}")

    await connect_ble()


asyncio.run(main())

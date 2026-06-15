#!/usr/bin/env python3
"""
RoboTaksi Dashboard Backend
FastAPI + WebSocket — ROS2 topic monitoring & process management
"""
import asyncio
import json
import os
import subprocess
import threading
import time
import yaml
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

# ─── Configuration ─────────────────────────────────────────────────────────
PKG = "my_robot_controller"

def _find_first(paths: list[str]) -> str:
    for p in paths:
        if Path(p).exists():
            return p
    return paths[0]  # fallback even if missing

ROS_SETUP = os.environ.get("ROS_SETUP") or _find_first([
    "/opt/ros/humble/setup.bash",
    "/opt/ros/iron/setup.bash",
    "/opt/ros/jazzy/setup.bash",
    "/opt/ros/foxy/setup.bash",
])

WS_SETUP = os.environ.get("WS_SETUP") or _find_first([
    "/ros2_ws/install/setup.bash",
    str(Path.home() / "ros2_ws/install/setup.bash"),
    str(Path.home() / "colcon_ws/install/setup.bash"),
    "/root/ros2_ws/install/setup.bash",
])

def ros_cmd(cmd: str) -> str:
    """Wrap a command so it runs with ROS2 sourced."""
    parts = [f"source {ROS_SETUP} 2>/dev/null"]
    if Path(WS_SETUP).exists():
        parts.append(f"source {WS_SETUP} 2>/dev/null")
    parts.append(cmd)
    return "bash -c '" + " && ".join(parts) + "'"


# ─── Launch & Node Definitions ─────────────────────────────────────────────
LAUNCH_CONFIGS = {
    "sim":        {"label": "Simülasyon",        "cmd": f"ros2 launch {PKG} sim_launch.py",         "color": "#3fb950"},
    "nav":        {"label": "Navigasyon",         "cmd": f"ros2 launch {PKG} navigation_launch.py",  "color": "#58a6ff"},
    "slam":       {"label": "SLAM Haritalama",    "cmd": f"ros2 launch {PKG} mapping_launch.py",     "color": "#d29922"},
    "obstacles":  {"label": "Engel Spawn",        "cmd": f"ros2 launch {PKG} spawn_obstacle_launch.py", "color": "#bc8cff"},
}

NODE_CONFIGS = {
    "bt_decision":    {"label": "BT Karar",          "cmd": f"ros2 run {PKG} bt_decision_node",        "color": "#f85149"},
    "ackermann":      {"label": "Ackermann Köprüsü", "cmd": f"ros2 run {PKG} ackermann_bridge",        "color": "#58a6ff"},
    "vehicle_emu":    {"label": "Araç Emülatörü",    "cmd": f"ros2 run {PKG} vehicle_emulator",        "color": "#db6d28"},
    "traffic_light":  {"label": "Trafik Işığı",      "cmd": f"ros2 run {PKG} traffic_light_detector",  "color": "#ffc107"},
    "traffic_sign":   {"label": "Trafik İşareti",    "cmd": f"ros2 run {PKG} traffic_sign_detector",   "color": "#8bc34a"},
    "lane_cv":        {"label": "Şerit (CV)",         "cmd": f"ros2 run {PKG} lane_detector_cv",        "color": "#03a9f4"},
    "lane_dl":        {"label": "Şerit (DL/UFLD)",   "cmd": f"ros2 run {PKG} lane_detection_node",     "color": "#3f51b5"},
    "mission":        {"label": "Görev Yöneticisi",  "cmd": f"ros2 run {PKG} mission_manager",         "color": "#009688"},
    "obstacle_node":  {"label": "LiDAR Engelci",     "cmd": f"ros2 run {PKG} static_obstacle_node",    "color": "#9c27b0"},
}

# Topics to monitor continuously
MONITORED_TOPICS = [
    "/cmd_vel",
    "/beemobs/FB_VehicleSpeed",
    "/beemobs/RC_THRT_DATA",
    "/beemobs/AUTONOMOUS_BrakePedalControl",
    "/beemobs/steering_target_value",
    "/bt/status",
    "/perception/flags/traffic_light_red",
    "/perception/flags/traffic_light_green",
    "/traffic_sign_detections",
    "/perception/lane_offset",
    "/perception/lane_info",
]


# ─── State ─────────────────────────────────────────────────────────────────
class State:
    def __init__(self):
        self.ros_connected   = False
        self.active_nodes    = []
        self.processes: dict[str, subprocess.Popen] = {}
        self.logs: deque     = deque(maxlen=800)
        self.topic: dict     = {
            "cmd_vel_linear":   0.0,
            "cmd_vel_angular":  0.0,
            "vehicle_speed":    0.0,
            "throttle":         50.0,
            "brake":            0.0,
            "steering":         0.0,
            "bt_status":        "BAĞLANTI YOK",
            "tl_red":           False,
            "tl_green":         False,
            "traffic_signs":    [],
            "lane_offset":      0.0,
        }
        self._lock = threading.Lock()

    def add_log(self, source: str, level: str, message: str) -> dict:
        entry = {
            "ts":      datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "source":  source,
            "level":   level,
            "message": message,
        }
        with self._lock:
            self.logs.append(entry)
        return entry

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "type":           "snapshot",
                "ros_connected":  self.ros_connected,
                "active_nodes":   list(self.active_nodes),
                "processes":      {k: v.poll() is None for k, v in self.processes.items()},
                "topic":          dict(self.topic),
                "logs":           list(self.logs)[-80:],
            }


G = State()
app = FastAPI(title="RoboTaksi Dashboard")
clients: set[WebSocket] = set()


# ─── WebSocket Hub ──────────────────────────────────────────────────────────
@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    try:
        await websocket.send_json(G.snapshot())
        while True:
            try:
                raw = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                await handle_command(json.loads(raw), websocket)
            except asyncio.TimeoutError:
                pass
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(websocket)


async def broadcast(data: dict):
    dead = set()
    for ws in list(clients):
        try:
            await ws.send_json(data)
        except Exception:
            dead.add(ws)
    clients -= dead


async def handle_command(cmd: dict, ws: WebSocket):
    action = cmd.get("action")
    key    = cmd.get("key", "")

    if action == "launch":
        cfg = LAUNCH_CONFIGS.get(key) or NODE_CONFIGS.get(key)
        if cfg:
            await launch_proc(key, cfg)

    elif action == "kill":
        await kill_proc(key)

    elif action == "estop":
        await emergency_stop()

    elif action == "pub_cmd_vel":
        lin = float(cmd.get("linear", 0))
        ang = float(cmd.get("angular", 0))
        pub_cmd = f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: {lin}}}, angular: {{z: {ang}}}}}'"
        subprocess.Popen(ros_cmd(pub_cmd), shell=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ─── Process Management ────────────────────────────────────────────────────
async def launch_proc(key: str, cfg: dict):
    if key in G.processes and G.processes[key].poll() is None:
        entry = G.add_log("PANEL", "WARN", f"{cfg['label']} zaten çalışıyor")
        await broadcast({"type": "log", "entry": entry})
        return

    entry = G.add_log("PANEL", "INFO", f"▶ Başlatılıyor: {cfg['label']}")
    await broadcast({"type": "log", "entry": entry})

    try:
        proc = subprocess.Popen(
            ros_cmd(cfg["cmd"]),
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        G.processes[key] = proc
        loop = asyncio.get_event_loop()
        threading.Thread(
            target=_capture_logs,
            args=(key, cfg["label"], proc, loop),
            daemon=True,
        ).start()
        await broadcast({"type": "proc_update", "key": key, "running": True,
                         "label": cfg["label"], "color": cfg["color"]})
    except Exception as exc:
        entry = G.add_log("PANEL", "ERROR", f"Hata: {exc}")
        await broadcast({"type": "log", "entry": entry})


async def kill_proc(key: str):
    proc = G.processes.get(key)
    cfg  = LAUNCH_CONFIGS.get(key) or NODE_CONFIGS.get(key)
    label = cfg["label"] if cfg else key
    if proc and proc.poll() is None:
        proc.terminate()
        entry = G.add_log("PANEL", "WARN", f"■ Durduruldu: {label}")
        await broadcast({"type": "log", "entry": entry})
    await broadcast({"type": "proc_update", "key": key, "running": False, "label": label})


async def emergency_stop():
    for key in list(G.processes.keys()):
        await kill_proc(key)
    subprocess.Popen(
        ros_cmd("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'"),
        shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    entry = G.add_log("PANEL", "ERROR", "🚨 ACİL DURDURMA BASILDI — Tüm süreçler sonlandırıldı")
    await broadcast({"type": "log", "entry": entry})


def _capture_logs(key: str, label: str, proc: subprocess.Popen, loop):
    for line in proc.stdout:
        line = line.rstrip()
        if not line:
            continue
        ll = line.lower()
        level = "ERROR" if ("error" in ll or "fatal" in ll) else \
                "WARN"  if ("warn"  in ll or "warning" in ll) else "INFO"
        entry = G.add_log(label, level, line)
        asyncio.run_coroutine_threadsafe(
            broadcast({"type": "log", "entry": entry}), loop
        )
    proc.wait()
    asyncio.run_coroutine_threadsafe(
        broadcast({"type": "proc_update", "key": key, "running": False, "label": label}), loop
    )


# ─── Topic Monitoring ──────────────────────────────────────────────────────
def _monitor_topic(topic: str):
    """Persistent ros2 topic echo → parse YAML blocks."""
    cmd = ros_cmd(f"ros2 topic echo {topic} 2>/dev/null")
    while True:
        try:
            proc = subprocess.Popen(
                cmd, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, bufsize=1,
            )
            buf = []
            for line in proc.stdout:
                line = line.rstrip()
                if line == "---":
                    if buf:
                        try:
                            msg = yaml.safe_load("\n".join(buf))
                            _apply_topic(topic, msg)
                        except Exception:
                            pass
                        buf = []
                elif line:
                    buf.append(line)
            proc.wait()
        except Exception:
            pass
        time.sleep(2)


def _apply_topic(topic: str, msg):
    if msg is None:
        return
    with G._lock:
        t = G.topic
        if topic == "/cmd_vel":
            t["cmd_vel_linear"]  = float((msg.get("linear") or {}).get("x", 0))
            t["cmd_vel_angular"] = float((msg.get("angular") or {}).get("z", 0))
        elif topic == "/beemobs/FB_VehicleSpeed":
            t["vehicle_speed"] = float(msg.get("data", 0))
        elif topic == "/beemobs/RC_THRT_DATA":
            t["throttle"] = float(msg.get("data", 50))
        elif topic == "/beemobs/AUTONOMOUS_BrakePedalControl":
            t["brake"] = float(msg.get("data", 0))
        elif topic == "/beemobs/steering_target_value":
            t["steering"] = float(msg.get("data", 0))
        elif topic == "/bt/status":
            t["bt_status"] = str(msg.get("data", ""))
        elif topic == "/perception/flags/traffic_light_red":
            t["tl_red"] = bool(msg.get("data", False))
        elif topic == "/perception/flags/traffic_light_green":
            t["tl_green"] = bool(msg.get("data", False))
        elif topic == "/traffic_sign_detections":
            raw = msg.get("data", "")
            try:
                t["traffic_signs"] = json.loads(raw) if raw else []
            except Exception:
                t["traffic_signs"] = [{"class": raw, "confidence": 1.0}] if raw else []
        elif topic == "/perception/lane_offset":
            t["lane_offset"] = float(msg.get("data", 0))


def _check_ros():
    while True:
        try:
            r = subprocess.run(
                ros_cmd("ros2 node list"),
                shell=True, capture_output=True, text=True, timeout=4,
            )
            connected   = r.returncode == 0
            active      = [n.strip() for n in r.stdout.splitlines() if n.strip()]
            G.ros_connected = connected
            G.active_nodes  = active
        except Exception:
            G.ros_connected = False
            G.active_nodes  = []
        time.sleep(3)


def start_background_tasks():
    threading.Thread(target=_check_ros, daemon=True).start()
    for topic in MONITORED_TOPICS:
        threading.Thread(target=_monitor_topic, args=(topic,), daemon=True).start()


async def _broadcast_loop():
    while True:
        if clients:
            snap = G.snapshot()
            snap["type"] = "state_update"
            await broadcast(snap)
        await asyncio.sleep(0.25)  # 4 Hz state refresh


@app.on_event("startup")
async def on_startup():
    start_background_tasks()
    asyncio.create_task(_broadcast_loop())


# ─── HTML Serve ────────────────────────────────────────────────────────────
@app.get("/", response_class=HTMLResponse)
async def root():
    p = Path(__file__).parent / "static" / "index.html"
    return p.read_text(encoding="utf-8")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")

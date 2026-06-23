#!/usr/bin/env python3
import os
import json
import time
import math
import signal
import queue
import threading
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist

REF_LAT = 41.015137
REF_LON = 28.979530
EARTH_R  = 6378137.0

PORT = 8085

HTML_CONTENT = """<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robotaksi Cockpit</title>
    <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&family=Outfit:wght@300;400;600;800&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg: #080c14;
            --panel: rgba(13,20,35,0.6);
            --border: rgba(0,242,254,0.15);
            --cyan: #00f2fe;
            --green: #00f260;
            --red: #ff0055;
            --orange: #ff9f43;
            --muted: #64748b;
            --text: #e2e8f0;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Outfit', sans-serif; }
        body {
            background: var(--bg);
            color: var(--text);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
        }
        header {
            padding: 1rem 1.5rem;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid var(--border);
            background: rgba(8,12,20,0.8);
        }
        .logo {
            font-size: 1.2rem;
            font-weight: 800;
            letter-spacing: 2px;
            background: linear-gradient(135deg, var(--cyan), var(--green));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .sim-badge {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            font-size: 0.85rem;
            font-weight: 600;
            background: var(--panel);
            border: 1px solid var(--border);
            padding: 0.4rem 0.9rem;
            border-radius: 50px;
        }
        .dot {
            width: 8px; height: 8px;
            border-radius: 50%;
            background: var(--muted);
        }
        .dot.on { background: var(--green); box-shadow: 0 0 8px var(--green); }

        .main {
            flex: 1;
            display: grid;
            grid-template-columns: 260px 1fr;
            grid-template-rows: auto 1fr;
            gap: 1rem;
            padding: 1rem 1.5rem;
        }

        /* LEFT column spans both rows */
        .left { grid-row: 1 / 3; display: flex; flex-direction: column; gap: 1rem; }

        .panel {
            background: var(--panel);
            border: 1px solid var(--border);
            border-radius: 14px;
            padding: 1.2rem;
            backdrop-filter: blur(10px);
            display: flex;
            flex-direction: column;
            gap: 0.9rem;
        }
        .panel-title {
            font-size: 0.8rem;
            font-weight: 700;
            letter-spacing: 1px;
            text-transform: uppercase;
            color: var(--muted);
            border-bottom: 1px solid rgba(255,255,255,0.05);
            padding-bottom: 0.6rem;
        }

        .btn {
            padding: 0.75rem 1rem;
            border-radius: 8px;
            border: none;
            font-weight: 700;
            font-size: 0.9rem;
            cursor: pointer;
            transition: all 0.2s;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .btn-launch {
            background: linear-gradient(135deg, var(--cyan), #00d2ff);
            color: #000;
        }
        .btn-launch:hover { opacity: 0.85; }
        .btn-launch:disabled { opacity: 0.4; cursor: default; }
        .btn-stop {
            background: rgba(255,0,85,0.1);
            border: 1px solid var(--red);
            color: var(--red);
        }
        .btn-stop:hover:not(:disabled) { background: var(--red); color: #fff; }
        .btn-stop:disabled { opacity: 0.4; cursor: default; }
        .btn-go {
            background: linear-gradient(135deg, var(--green), #00c851);
            color: #000;
            font-size: 1.1rem;
            padding: 1rem;
        }
        .btn-go:hover:not(:disabled) { opacity: 0.85; transform: scale(1.02); }
        .btn-go:disabled { opacity: 0.35; cursor: default; transform: none; }
        .btn-kill {
            background: var(--red);
            color: #fff;
            font-size: 1rem;
        }
        .btn-kill:hover { opacity: 0.85; }
        .btn-kill.active { background: #fff; color: var(--red); border: 2px solid var(--red); animation: pulse-kill 0.8s infinite; }
        @keyframes pulse-kill { 0%,100%{box-shadow:0 0 0 0 rgba(255,0,85,0.6)} 50%{box-shadow:0 0 0 8px rgba(255,0,85,0)} }
        .ums-status {
            padding: 0.6rem 1rem;
            border-radius: 8px;
            text-align: center;
            font-size: 0.85rem;
            font-weight: 700;
            letter-spacing: 1px;
            text-transform: uppercase;
            transition: all 0.3s;
        }
        .ums-waiting  { background: rgba(100,116,139,0.15); border: 1px solid var(--muted); color: var(--muted); }
        .ums-running  { background: rgba(0,242,96,0.12); border: 1px solid var(--green); color: var(--green); animation: pulse-green 1.5s infinite; }
        .ums-stopped  { background: rgba(255,0,85,0.12); border: 1px solid var(--red); color: var(--red); }
        @keyframes pulse-green { 0%,100%{box-shadow:0 0 0 0 rgba(0,242,96,0.4)} 50%{box-shadow:0 0 0 6px rgba(0,242,96,0)} }

        /* Telemetry grid (top-right) */
        .tele-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 0.8rem;
        }
        .tcard {
            background: rgba(255,255,255,0.02);
            border: 1px solid rgba(255,255,255,0.06);
            border-radius: 10px;
            padding: 0.9rem;
        }
        .tcard-label { font-size: 0.7rem; font-weight: 600; color: var(--muted); text-transform: uppercase; margin-bottom: 0.3rem; }
        .tcard-val { font-size: 1.4rem; font-weight: 800; }
        .cyan { color: var(--cyan); }
        .green { color: var(--green); }
        .orange { color: var(--orange); }

        /* DURAK overlay card */
        .durak-card {
            background: rgba(0,242,96,0.06);
            border: 1px solid rgba(0,242,96,0.25);
            border-radius: 10px;
            padding: 0.9rem;
            display: none;
            align-items: center;
            justify-content: space-between;
        }
        .durak-card.show { display: flex; }
        .durak-label { font-size: 0.75rem; font-weight: 700; color: var(--green); letter-spacing: 1px; }
        .durak-count { font-size: 2.2rem; font-weight: 900; color: var(--green); line-height: 1; }

        /* Mode badge */
        .mode-badge {
            padding: 0.7rem 1rem;
            border-radius: 8px;
            font-size: 0.9rem;
            font-weight: 700;
            text-align: center;
            border: 1px solid var(--border);
            background: rgba(255,255,255,0.03);
            transition: all 0.3s;
            letter-spacing: 0.5px;
        }

        /* Trafik Lambası Widget */
        .tl-widget {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 0.3rem;
            background: #0a0f1a;
            border: 1px solid var(--border);
            border-radius: 10px;
            padding: 0.8rem 0.6rem;
        }
        .tl-widget .tl-title {
            font-size: 0.65rem;
            font-weight: 700;
            letter-spacing: 1px;
            text-transform: uppercase;
            color: var(--muted);
            margin-bottom: 0.3rem;
        }
        .tl-lamp {
            width: 28px; height: 28px;
            border-radius: 50%;
            border: 2px solid rgba(255,255,255,0.08);
            transition: background 0.2s, box-shadow 0.2s;
        }
        .tl-lamp.red-off    { background: #3a0a0a; }
        .tl-lamp.red-on     { background: #ff1a1a; box-shadow: 0 0 14px #ff1a1a, 0 0 30px #ff000088; }
        .tl-lamp.yellow-off { background: #2a2000; }
        .tl-lamp.yellow-on  { background: #ffcc00; box-shadow: 0 0 14px #ffcc00, 0 0 30px #ffaa0088; }
        .tl-lamp.green-off  { background: #0a2a0a; }
        .tl-lamp.green-on   { background: #00ff44; box-shadow: 0 0 14px #00ff44, 0 0 30px #00ff0088; }
        .tl-detect {
            font-size: 0.65rem;
            color: var(--muted);
            margin-top: 0.3rem;
            text-align: center;
        }
        .tl-detect.detected { color: var(--cyan); }

        /* Terminal */
        .terminal-wrap { display: flex; flex-direction: column; gap: 0.5rem; }
        .terminal {
            flex: 1;
            background: #040710;
            border: 1px solid rgba(255,255,255,0.05);
            border-radius: 10px;
            padding: 0.8rem;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.8rem;
            line-height: 1.4;
            color: #a6daed;
            overflow-y: auto;
            height: 320px;
        }
        .tl { margin-bottom: 0.2rem; }
        .tl.info { color: var(--cyan); }
        .tl.warn { color: var(--orange); }
        .tl.error { color: var(--red); }
        .tl.system { color: #666; }
    </style>
</head>
<body>
<header>
    <div class="logo">ROBOTAKSI COCKPIT</div>
    <div class="sim-badge">
        <div class="dot" id="sim-dot"></div>
        <span id="sim-txt">Çevrimdışı</span>
    </div>
</header>

<div class="main">
    <!-- SOL: Kontroller -->
    <div class="left">
        <div class="panel">
            <div class="panel-title">UMS Kontrol</div>
            <div class="ums-status ums-waiting" id="ums-status">⏳ GO Bekleniyor</div>
            <button class="btn btn-go" id="btn-go" onclick="umsGo()">🚀 UMS-2 GO</button>
            <button class="btn btn-kill" id="btn-kill" onclick="umsKill()">🛑 UMS-1 KILL</button>
        </div>

        <div class="panel">
            <div class="panel-title">Simülasyon</div>
            <button class="btn btn-launch" id="btn-launch" onclick="launch()">▶ BAŞLAT</button>
            <button class="btn btn-stop" id="btn-stop" onclick="stop()" disabled>■ DURDUR</button>
        </div>

        <div class="panel">
            <div class="panel-title">Trafik Lambası</div>
            <div style="display:flex; gap:0.8rem; align-items:center;">
                <div class="tl-widget">
                    <div class="tl-title">Işık</div>
                    <div class="tl-lamp red-off"    id="lamp-red"></div>
                    <div class="tl-lamp yellow-off" id="lamp-yellow"></div>
                    <div class="tl-lamp green-off"  id="lamp-green"></div>
                </div>
                <div style="flex:1;">
                    <div class="tcard" style="margin-bottom:0.5rem;">
                        <div class="tcard-label">Durum</div>
                        <div class="tcard-val" id="tl-state" style="font-size:1.1rem;">—</div>
                    </div>
                    <div class="tcard">
                        <div class="tcard-label">Kamera Tespiti</div>
                        <div class="tl-detect" id="tl-detect">—</div>
                    </div>
                </div>
            </div>
        </div>

        <div class="panel" style="flex:1;">
            <div class="panel-title">Konum</div>
            <div class="tcard">
                <div class="tcard-label">X (Gazebo m)</div>
                <div class="tcard-val orange" id="val-x">—</div>
            </div>
            <div class="tcard">
                <div class="tcard-label">Y (Gazebo m)</div>
                <div class="tcard-val orange" id="val-y">—</div>
            </div>
            <div class="tcard">
                <div class="tcard-label">Yaw (°)</div>
                <div class="tcard-val cyan" id="val-yaw">—</div>
            </div>
        </div>
    </div>

    <!-- SAĞ ÜST: Telemetri -->
    <div>
        <div class="tele-grid">
            <div class="tcard">
                <div class="tcard-label">Hız</div>
                <div class="tcard-val cyan" id="val-speed">0.0 m/s</div>
            </div>
            <div class="tcard">
                <div class="tcard-label">Direksiyon</div>
                <div class="tcard-val" id="val-steer">0°</div>
            </div>
            <div class="tcard">
                <div class="tcard-label">Durağa Mesafe</div>
                <div class="tcard-val green" id="val-dist">— m</div>
            </div>
            <div class="tcard">
                <div class="tcard-label">Hedef</div>
                <div class="tcard-val" id="val-wp" style="font-size:1rem;">—</div>
            </div>
        </div>

        <div style="margin-top: 0.8rem; display: flex; gap: 0.8rem; align-items: stretch;">
            <div class="durak-card" id="durak-card">
                <div>
                    <div class="durak-label">DURAKTA BEKLİYOR</div>
                    <div style="font-size:0.75rem; color: var(--muted); margin-top:0.2rem;">saniye kaldı</div>
                </div>
                <div class="durak-count" id="durak-count">—</div>
            </div>
            <div class="mode-badge" id="mode-badge" style="flex:1;">—</div>
        </div>
    </div>

    <!-- SAĞ ALT: Terminal -->
    <div class="panel" style="grid-column: 2;">
        <div class="panel-title">Konsol</div>
        <div class="terminal" id="terminal">
            <div class="tl system">Robotaksi Cockpit hazır. Port: 8085</div>
        </div>
    </div>
</div>

<script>
let es = null;

function log(text, type='system') {
    const t = document.getElementById('terminal');
    const d = document.createElement('div');
    d.className = 'tl ' + type;
    d.textContent = '[' + new Date().toLocaleTimeString() + '] ' + text.replace(/\\x1B\\[[0-9;]*[a-zA-Z]/g,'');
    t.appendChild(d);
    t.scrollTop = t.scrollHeight;
    while (t.children.length > 300) t.removeChild(t.firstChild);
}

function connect() {
    if (es) es.close();
    es = new EventSource('/stream');
    es.onmessage = e => {
        const d = JSON.parse(e.data);
        if (d.type === 'log') { log(d.message, d.level); return; }

        if (d.speed !== undefined)
            document.getElementById('val-speed').textContent = d.speed.toFixed(2) + ' m/s';
        if (d.steer !== undefined)
            document.getElementById('val-steer').textContent = (d.steer * 180 / Math.PI).toFixed(1) + '°';
        if (d.yaw !== undefined)
            document.getElementById('val-yaw').textContent = (d.yaw * 180 / Math.PI).toFixed(0) + '°';
        if (d.pos_x !== undefined) {
            document.getElementById('val-x').textContent = d.pos_x.toFixed(1) + ' m';
            document.getElementById('val-y').textContent = d.pos_y.toFixed(1) + ' m';
        }
        if (d.durak_dist !== undefined) {
            const v = parseFloat(d.durak_dist);
            document.getElementById('val-dist').textContent = v < 0 ? '—' : v.toFixed(1) + ' m';
        }
        if (d.durak_remaining !== undefined) {
            const r = parseInt(d.durak_remaining);
            const card = document.getElementById('durak-card');
            document.getElementById('durak-count').textContent = r;
            card.className = r > 0 ? 'durak-card show' : 'durak-card';
        }
        if (d.decision_state) {
            const b = document.getElementById('mode-badge');
            b.textContent = d.decision_state;
            if (d.decision_state.includes('DURUYORUM')) {
                b.style.cssText = 'background:rgba(0,242,96,0.1);border-color:var(--green);color:var(--green);';
            } else if (d.decision_state.includes('LANE')) {
                b.style.cssText = 'background:rgba(0,242,254,0.08);border-color:var(--cyan);color:var(--cyan);';
            } else if (d.decision_state.includes('GPS')) {
                b.style.cssText = 'background:rgba(255,159,67,0.08);border-color:var(--orange);color:var(--orange);';
            } else {
                b.style.cssText = '';
            }
        }
        if (d.wp) {
            try {
                const wp = JSON.parse(d.wp);
                document.getElementById('val-wp').textContent = wp.desc || '—';
            } catch {}
        }
        if (d.ums_state !== undefined) {
            const el = document.getElementById('ums-status');
            const go = document.getElementById('btn-go');
            if (d.ums_state === 'RUNNING') {
                el.className = 'ums-status ums-running';
                el.textContent = '✅ GÖREV AKTİF';
                go.disabled = true;
            } else if (d.ums_state === 'STOPPED') {
                el.className = 'ums-status ums-stopped';
                el.textContent = '🛑 ACİL DURUŞ';
                go.disabled = false;
            } else {
                el.className = 'ums-status ums-waiting';
                el.textContent = '⏳ GO Bekleniyor';
                go.disabled = false;
            }
        }
        if (d.kill_active !== undefined) {
            const kb = document.getElementById('btn-kill');
            kb.className = d.kill_active ? 'btn btn-kill active' : 'btn btn-kill';
            kb.textContent = d.kill_active ? '⚡ KİLL AKTİF' : '🛑 UMS-1 KILL';
        }
        if (d.tl_state !== undefined) {
            const parts = d.tl_state.split(':');
            const color = parts[0];
            const secs  = parts[1] ? parseFloat(parts[1]).toFixed(1) + 's' : '';
            const colors = { 'YESIL': 'YEŞİL', 'SARI': 'SARI', 'KIRMIZI': 'KIRMIZI' };
            const el = document.getElementById('tl-state');
            el.textContent = (colors[color] || color) + (secs ? ' — ' + secs : '');
            el.style.color = color === 'YESIL' ? 'var(--green)' :
                             color === 'SARI'  ? 'var(--orange)' : 'var(--red)';
            document.getElementById('lamp-red').className    = 'tl-lamp ' + (color === 'KIRMIZI' ? 'red-on'    : 'red-off');
            document.getElementById('lamp-yellow').className = 'tl-lamp ' + (color === 'SARI'    ? 'yellow-on' : 'yellow-off');
            document.getElementById('lamp-green').className  = 'tl-lamp ' + (color === 'YESIL'   ? 'green-on'  : 'green-off');
        }
        if (d.tl_detect !== undefined) {
            const el = document.getElementById('tl-detect');
            el.textContent = d.tl_detect || '—';
            el.className = d.tl_detect && d.tl_detect !== '—' ? 'tl-detect detected' : 'tl-detect';
        }
        if (d.sim_running !== undefined) {
            document.getElementById('sim-dot').className = d.sim_running ? 'dot on' : 'dot';
            document.getElementById('sim-txt').textContent = d.sim_running ? 'Çalışıyor' : 'Çevrimdışı';
            document.getElementById('btn-launch').disabled = d.sim_running;
            document.getElementById('btn-stop').disabled = !d.sim_running;
        }
    };
    es.onerror = () => { log('Bağlantı koptu, yeniden bağlanıyor...','error'); setTimeout(connect, 2000); };
}

let killActive = false;
function umsGo() {
    fetch('/api/ums/go', {method:'POST'}).then(()=>log('UMS-2 GO gönderildi.','info'));
}
function umsKill() {
    killActive = !killActive;
    fetch('/api/ums/kill', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({active: killActive})
    }).then(()=>log(killActive ? 'UMS-1 KİLL AKTİF!' : 'UMS-1 KILL serbest.', killActive?'error':'warn'));
}
function launch() {
    fetch('/api/launch', {method:'POST'}).then(r=>r.json()).then(d=>{
        log(d.status==='OK' ? 'Simülasyon başlatıldı.' : 'Hata: '+d.message, d.status==='OK'?'info':'error');
    });
}
function stop() {
    fetch('/api/stop', {method:'POST'}).then(()=>log('Simülasyon durduruldu.','warn'));
}

connect();
</script>
</body>
</html>
"""


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.telemetry = {
            'pos_x': 0.0, 'pos_y': 0.0,
            'speed': 0.0, 'steer': 0.0, 'yaw': 0.0,
            'wp': '{}',
            'decision_state': 'BEKLENİYOR',
            'sim_running': False,
            'durak_dist': -1.0,
            'durak_remaining': 0,
            'tl_state': '—',
            'tl_detect': '—',
            'ums_state': 'WAITING',
            'kill_active': False,
        }

        self.web_clients = []
        self.web_clients_lock = threading.Lock()
        self.sim_process = None

        self.pub_go   = self.create_publisher(Bool, '/ums/go',   10)
        self.pub_kill = self.create_publisher(Bool, '/ums/kill', 10)
        self.pub_mission = self.create_publisher(String, '/mission/command', 10)

        self.create_subscription(Twist,      '/cmd_vel',           self.cmd_vel_cb,   10)
        self.create_subscription(NavSatFix,  '/fix',               self.gps_cb,       10)
        self.create_subscription(Imu,        '/imu/data',          self.imu_cb,       10)
        self.create_subscription(String,     '/map/waypoints',     self.waypoint_cb,  10)
        self.create_subscription(String,     '/bt/status',         self.decision_cb,  10)
        self.create_subscription(Float32,    '/durak/dist',        self.durak_dist_cb, 10)
        self.create_subscription(Int32,      '/durak/remaining',   self.durak_rem_cb,  10)
        self.create_subscription(String,     '/mission/command',   self.mission_cb,   10)
        self.create_subscription(String,     '/perception/traffic_light',    self.tl_detect_cb, 10)
        self.create_subscription(String,     '/perception/traffic_light_cv', self.tl_cv_cb,     10)

        self._tl_last_seen = 0.0
        self._tl_clear_timer = self.create_timer(0.5, self._tl_clear_check)
        self._kill_active = False

        self.get_logger().info(f'Dashboard hazır — http://localhost:{PORT}')

    # callbacks
    def gps_cb(self, msg):
        dlat = math.radians(msg.latitude  - REF_LAT)
        dlon = math.radians(msg.longitude - REF_LON)
        y = dlat * EARTH_R
        x = dlon * EARTH_R * math.cos(math.radians(REF_LAT))
        self.telemetry['pos_x'] = round(x, 1)
        self.telemetry['pos_y'] = round(y, 1)
        self.broadcast({'pos_x': round(x, 1), 'pos_y': round(y, 1)})

    def imu_cb(self, msg):
        q = msg.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.update('yaw', yaw)

    def cmd_vel_cb(self, msg):
        self.update('speed', msg.linear.x)
        self.update('steer', msg.angular.z)

    def waypoint_cb(self, msg):
        self.update('wp', msg.data)

    def decision_cb(self, msg):
        self.update('decision_state', msg.data)

    def mission_cb(self, msg):
        cmd = msg.data
        if cmd == 'START':
            self.update('ums_state', 'RUNNING')
        elif cmd == 'EMERGENCY_STOP':
            self.update('ums_state', 'STOPPED')
        elif cmd == 'EMERGENCY_CLEAR':
            self.update('ums_state', 'WAITING')

    def send_go(self):
        m = Bool(); m.data = True
        self.pub_go.publish(m)
        def release():
            import time; time.sleep(0.3)
            m2 = Bool(); m2.data = False
            self.pub_go.publish(m2)
        threading.Thread(target=release, daemon=True).start()

    def send_kill(self, active: bool):
        self._kill_active = active
        m = Bool(); m.data = active
        self.pub_kill.publish(m)
        self.update('kill_active', active)

    def durak_dist_cb(self, msg):
        self.update('durak_dist', round(float(msg.data), 1))

    def durak_rem_cb(self, msg):
        self.update('durak_remaining', int(msg.data))

    _TL_MAP = {'Green': 'YESIL', 'Yellow': 'SARI', 'Red': 'KIRMIZI'}

    def _tl_from_detection(self, msg_data, min_conf=0.50):
        try:
            d     = json.loads(msg_data)
            cls   = d.get('class', '')
            conf  = float(d.get('confidence', 0.0))
            src   = d.get('source', 'yolo')
            if conf < min_conf:
                return
            color = self._TL_MAP.get(cls)
            if not color:
                return
            self._tl_last_seen = time.time()
            self.update('tl_state',  color)
            self.update('tl_detect', f'{cls} {conf:.0%} [{src.upper()}]')
        except Exception:
            pass

    def tl_detect_cb(self, msg):
        self._tl_from_detection(msg.data, min_conf=0.50)

    def tl_cv_cb(self, msg):
        self._tl_from_detection(msg.data, min_conf=0.60)

    def _tl_clear_check(self):
        if time.time() - self._tl_last_seen > 1.5:
            if self.telemetry.get('tl_state') != '—':
                self.update('tl_state',  '—')
                self.update('tl_detect', '—')

    def update(self, key, value):
        self.telemetry[key] = value
        self.broadcast({key: value})

    # SSE
    def add_client(self, q):
        with self.web_clients_lock:
            self.web_clients.append(q)
            q.put(dict(self.telemetry))

    def remove_client(self, q):
        with self.web_clients_lock:
            if q in self.web_clients:
                self.web_clients.remove(q)

    def broadcast(self, data):
        with self.web_clients_lock:
            for q in self.web_clients:
                q.put(data)

    def broadcast_log(self, text, level='info'):
        self.broadcast({'type': 'log', 'message': text, 'level': level})

    # sim process
    def start_simulation(self):
        if self.sim_process is not None:
            return False, 'Zaten çalışıyor.'

        def run():
            try:
                self.sim_process = subprocess.Popen(
                    ['ros2', 'launch', 'my_robot_controller', 'sim_launch.py'],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, preexec_fn=os.setsid
                )
                self.update('sim_running', True)
                self.broadcast_log(f'Simülasyon başladı (PID {self.sim_process.pid})', 'info')
                for line in iter(self.sim_process.stdout.readline, ''):
                    if not line:
                        break
                    lvl = 'error' if '[ERROR]' in line else 'warn' if '[WARN]' in line else 'info' if '[INFO]' in line else 'system'
                    self.broadcast_log(line.strip(), lvl)
                self.sim_process.stdout.close()
                self.sim_process.wait()
            except Exception as e:
                self.broadcast_log(str(e), 'error')
            finally:
                self.sim_process = None
                self.update('sim_running', False)
                self.broadcast_log('Simülasyon kapandı.', 'system')

        threading.Thread(target=run, daemon=True).start()
        return True, 'OK'

    def stop_simulation(self):
        if self.sim_process is None:
            return False, 'Çalışan süreç yok.'
        try:
            os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
            return True, 'OK'
        except Exception as e:
            return False, str(e)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode())
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            q = queue.Queue()
            global_node.add_client(q)
            try:
                while rclpy.ok():
                    try:
                        data = q.get(timeout=0.5)
                        merged = dict(data)
                        while True:
                            try: merged.update(q.get_nowait())
                            except queue.Empty: break
                        self.wfile.write(f'data: {json.dumps(merged)}\n\n'.encode())
                        self.wfile.flush()
                    except queue.Empty:
                        self.wfile.write(b':\n\n')
                        self.wfile.flush()
            except Exception:
                pass
            finally:
                global_node.remove_client(q)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/api/launch':
            ok, msg = global_node.start_simulation()
            self._json({'status': 'OK' if ok else 'ERROR', 'message': msg})
        elif self.path == '/api/stop':
            ok, msg = global_node.stop_simulation()
            self._json({'status': 'OK' if ok else 'ERROR', 'message': msg})
        elif self.path == '/api/ums/go':
            global_node.send_go()
            self._json({'status': 'OK'})
        elif self.path == '/api/ums/kill':
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length)) if length else {}
            global_node.send_kill(bool(body.get('active', True)))
            self._json({'status': 'OK'})
        else:
            self.send_error(404)

    def _json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())


def main(args=None):
    rclpy.init(args=args)
    global global_node
    global_node = DashboardNode()

    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    try:
        rclpy.spin(global_node)
    except KeyboardInterrupt:
        pass
    finally:
        global_node.stop_simulation()
        server.shutdown()
        global_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

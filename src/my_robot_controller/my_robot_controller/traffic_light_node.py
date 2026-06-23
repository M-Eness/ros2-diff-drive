#!/usr/bin/env python3
"""
Trafik Lambası Node — TEKNOFEST Robotaksi 2026
3 bağımsız lamba, her biri farklı faz offsetiyle çalışır.
/traffic_light/state → "tl1=YESIL,tl2=KIRMIZI,tl3=SARI"
"""
import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WORLD    = 'robotaksi_world'
HIDE_Z   = -100.0
_SERVICE = f'/world/{WORLD}/set_pose'
_ENV     = None

# Faz döngüsü: (renk, süre_saniye)
PHASES = [
    ('YESIL',   6.0),
    ('SARI',    2.0),
    ('KIRMIZI', 5.0),
]
TOTAL_CYCLE = sum(d for _, d in PHASES)  # 13.0 s

# Faz sınırları (birikimli)
_PHASE_BOUNDS = []
_t = 0.0
for _label, _dur in PHASES:
    _PHASE_BOUNDS.append((_t, _t + _dur, _label))
    _t += _dur

def _phase_at(offset_sec: float) -> tuple[str, float]:
    """Döngü içindeki pozisyona göre (renk, fazda_kalan) döner."""
    pos = math.fmod(time.time() + offset_sec, TOTAL_CYCLE)
    if pos < 0:
        pos += TOTAL_CYCLE
    for start, end, label in _PHASE_BOUNDS:
        if pos < end:
            return label, end - pos
    return PHASES[-1][0], 0.0

# Lamp Z yükseklikleri (traffic_light modeline göre)
_DZ_R = 4.30
_DZ_Y = 3.85
_DZ_G = 3.40

# Trafik lambası tanımları
# offset: faz döngüsündeki başlangıç noktası (saniye)
#   TL1 offset=0  → t=0'da YESIL
#   TL2 offset=8  → t=0'da KIRMIZI  (YESIL6+SARI2=8)
#   TL3 offset=6  → t=0'da SARI     (YESIL6=6)
TRAFFIC_LIGHTS = [
    {'name': 'tl1', 'x': 10.7719, 'y':  0.6677, 'bz': 0.0, 'ox': -0.75, 'offset':  0.0},
    {'name': 'tl2', 'x': 18.8216, 'y': 33.5324, 'bz': 0.0, 'ox':  0.75, 'offset':  8.0},
    {'name': 'tl3', 'x': 40.6710, 'y':  0.0121, 'bz': 0.0, 'ox': -0.75, 'offset':  6.0},
]

_COLOR_MODEL = {'YESIL': 'green', 'SARI': 'yellow', 'KIRMIZI': 'red'}
_ALL_COLORS  = list(_COLOR_MODEL.keys())

def _lamp_z(color: str, base_z: float) -> float:
    return base_z + {'KIRMIZI': _DZ_R, 'SARI': _DZ_Y, 'YESIL': _DZ_G}[color]


def _set_pose(model_name: str, x: float, y: float, z: float, logger=None):
    req = (f'name: "{model_name}" '
           f'position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} '
           f'orientation {{ w: 1 }}')
    try:
        result = subprocess.run(
            ['ign', 'service', '-s', _SERVICE,
             '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean',
             '--req', req, '--timeout', '2000'],
            env=_ENV, capture_output=True, text=True, timeout=5.0,
        )
        if result.returncode != 0 and logger:
            logger.warn(f'set_pose HATA ({model_name}): {result.stderr.strip()[:80]}')
    except subprocess.TimeoutExpired:
        pass
    except Exception as e:
        if logger:
            logger.error(f'set_pose EX ({model_name}): {e}')


def _apply_tl(tl: dict, color: str, logger):
    """Tek bir TL için aktif renk lampını göster, diğerlerini gizle."""
    name = tl['name']
    lx   = tl['x'] + tl['ox']
    ly   = tl['y']
    bz   = tl['bz']
    threads = []
    for c in _ALL_COLORS:
        mname = f'{name}_{_COLOR_MODEL[c]}'
        if c == color:
            t = threading.Thread(
                target=_set_pose,
                args=(mname, lx, ly, _lamp_z(c, bz), logger), daemon=True)
        else:
            t = threading.Thread(
                target=_set_pose,
                args=(mname, 0.0, 0.0, HIDE_Z, logger), daemon=True)
        threads.append(t)
        t.start()
    for t in threads:
        t.join(timeout=6.0)


class TrafficLightNode(Node):

    def __init__(self):
        super().__init__('traffic_light_node')

        import os
        global _ENV
        _ENV = os.environ.copy()

        # Her TL için son uygulanan rengi takip et
        self._last_color = {tl['name']: None for tl in TRAFFIC_LIGHTS}

        self.pub = self.create_publisher(String, '/traffic_light/state', 10)
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f'TrafficLightNode hazır — {len(TRAFFIC_LIGHTS)} bağımsız lamba | '
            f'Döngü: {TOTAL_CYCLE:.0f}s')

    def _tick(self):
        parts = []
        for tl in TRAFFIC_LIGHTS:
            color, remaining = _phase_at(tl['offset'])
            parts.append(f"{tl['name']}={color}")

            # Renk değiştiyse set_pose çağır
            if color != self._last_color[tl['name']]:
                self._last_color[tl['name']] = color
                self.get_logger().info(
                    f"{tl['name']} → {color} ({remaining:.1f}s kaldı)")
                logger = self.get_logger()
                threading.Thread(
                    target=_apply_tl, args=(tl, color, logger), daemon=True
                ).start()

        self.pub.publish(String(data=','.join(parts)))


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Shared telemetry and coordinate helpers; no simulator imports."""

import json
import math
import os
import statistics
import time
from pathlib import Path


def write_json(path, data):
    """Replace a complete snapshot, retrying Windows sharing violations."""
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, allow_nan=False), encoding='utf-8')
    for attempt in range(20):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 19:
                raise
            time.sleep(0.01)


def read_live(directory, max_age=2.0):
    data = json.loads((Path(directory) / 'live.json').read_text(encoding='utf-8'))
    age = time.time() - data['time']
    if not -1.0 <= age <= max_age:
        raise RuntimeError('Vehicle telemetry is stale or clocks disagree')
    return data


def calibration_offset(pairs, tolerance=0.25):
    """Fit translation after CARLA z-up to AirSim z-down conversion."""
    if len(pairs) < 5:
        raise ValueError('At least five finite shared scene references are required')
    offsets = [[b[0] - a[0], b[1] - a[1], b[2] + a[2]] for a, b in pairs]
    if not all(math.isfinite(x) for row in offsets for x in row):
        raise ValueError('Nonfinite calibration reference')
    offset = [statistics.median(row[i] for row in offsets) for i in range(3)]
    residual = max(math.dist(row, offset) for row in offsets)
    if residual > tolerance:
        raise ValueError(f'Coordinate calibration residual {residual:.3f} m exceeds tolerance')
    return offset, residual


def following_point(vehicle, offset, distance, height):
    yaw = math.radians(vehicle['yaw_deg'])
    return [vehicle['x'] + offset[0] - distance * math.cos(yaw),
            vehicle['y'] + offset[1] - distance * math.sin(yaw),
            -vehicle['z'] + offset[2] - height]


def step_towards(current, target, maximum):
    distance = math.dist(current, target)
    scale = min(1.0, maximum / max(distance, 1e-9))
    return [a + (b - a) * scale for a, b in zip(current, target)]

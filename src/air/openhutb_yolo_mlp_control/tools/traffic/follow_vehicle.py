"""Opt-in AirSim pose following of CARLA telemetry; not flight dynamics validation."""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

from common import (REFERENCE_LIMIT, calibration_offset, follow_phase, following_point,
                    positive, read_live, follow_step, write_json)


def follow(args):
    # Also support a source checkout; installed copies use the installed package.
    package_root = Path(__file__).resolve().parents[2]
    if (package_root / 'openhutb_yolo_mlp_control' / '__init__.py').is_file():
        sys.path.insert(0, str(package_root))
    from openhutb_yolo_mlp_control.vendor import airsim

    directory = args.data.resolve()
    exit_file = directory / 'follow_exit'
    pause_file = directory / 'follow_pause'
    if exit_file.exists():
        raise RuntimeError('Previous follow_exit exists; remove it only after the old follower exits')
    # Exclusive output also prevents two follower processes owning the drone.
    with (directory / 'drone_follow.csv').open('x', newline='', encoding='utf-8') as stream:
        client = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=5)
        if not client.ping():
            raise RuntimeError('AirSim unavailable')
        live = read_live(directory)
        references = json.loads((directory / 'references.json').read_text(encoding='utf-8'))
        if references['map'] != live['map']:
            raise RuntimeError('References and telemetry belong to different maps')
        names = set(client.simListSceneObjects())
        pairs = []
        for reference in references['references']:
            if reference['name'] not in names:
                continue
            p = client.simGetObjectPose(reference['name']).position
            values = [p.x_val, p.y_val, p.z_val]
            if all(math.isfinite(value) for value in values):
                pairs.append((reference['carla'], values))
            if len(pairs) >= REFERENCE_LIMIT:
                break
        offset, residual = calibration_offset(pairs)
        write_json(directory / 'calibration.json',
                   {'map': live['map'], 'offset': offset, 'references': len(pairs),
                    'max_residual_m': residual})
        # Calibration can take time; refresh before selecting a target.
        live = read_live(directory)
        pose = client.simGetVehiclePose()
        vehicles = live['vehicles']
        if not vehicles:
            raise RuntimeError('No managed vehicles in telemetry')
        if args.target_id is None:
            def cost(v):
                return math.hypot(v['x'] + offset[0] - pose.position.x_val,
                                  v['y'] + offset[1] - pose.position.y_val)
            moving = [v for v in vehicles if math.hypot(v['vx'], v['vy']) > 1]
            target_id = min(moving or vehicles, key=cost)['actor_id']
        else:
            target_id = args.target_id
        if not any(v['actor_id'] == target_id for v in vehicles):
            raise RuntimeError('Requested target ID is absent')
        control = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=5)
        writer = csv.writer(stream)
        writer.writerow(['wall_time', 'target_id', 'drone_x_ned', 'drone_y_ned',
                         'drone_z_ned', 'desired_x_ned', 'desired_y_ned', 'desired_z_ned',
                         'horizontal_error_m', 'observed_speed_mps',
                         'observed_horizontal_speed_mps', 'phase', 'collision'])
        start = previous = time.monotonic()
        previous_pose = None
        previous_pose_time = None
        held_position = None
        reason = 'duration'
        print(f'Target {target_id}; backend=incremental_pose; calibration residual={residual:.6f} m',
              flush=True)
        try:
            control.enableApiControl(True)
            control.cancelLastTask()
            while args.duration == 0 or time.monotonic() - start < args.duration:
                if exit_file.exists():
                    reason = 'manual_exit'
                    break
                live = read_live(directory)
                if live['map'] != references['map']:
                    raise RuntimeError('Map changed')
                vehicle = next((v for v in live['vehicles'] if v['actor_id'] == target_id), None)
                if vehicle is None:
                    raise RuntimeError('Target disappeared')
                state = client.getMultirotorState()
                if (state.collision.has_collided
                        and state.collision.time_stamp > state.timestamp - 2_000_000_000):
                    raise RuntimeError('Recent collision; ending follow')
                pose = client.simGetVehiclePose()
                p = pose.position
                current = [p.x_val, p.y_val, p.z_val]
                pose_time = time.monotonic()
                observed_speed = (math.dist(current, previous_pose)
                                  / max(pose_time - previous_pose_time, 1e-9)
                                  if previous_pose is not None else 0.0)
                observed_horizontal_speed = (
                    math.dist(current[:2], previous_pose[:2])
                    / max(pose_time - previous_pose_time, 1e-9)
                    if previous_pose is not None else 0.0)
                previous_pose = current
                previous_pose_time = pose_time
                desired = following_point(vehicle, offset, args.distance, args.height)
                phase = follow_phase(current, desired)
                if pause_file.exists():
                    if held_position is None:
                        held_position = current[:]
                    desired = held_position
                    phase = 'hold'
                else:
                    held_position = None
                now = time.monotonic()
                dt = min(max(now - previous, 0), 0.25)
                previous = now
                position = follow_step(current, desired, args.speed * dt)
                # Keep simulator collision checks enabled to avoid moving through scenery.
                control.simSetVehiclePose(
                    airsim.Pose(airsim.Vector3r(*position), pose.orientation),
                    args.ignore_collision)
                error = math.hypot(current[0] - desired[0], current[1] - desired[1])
                writer.writerow([time.time(), target_id, *current, *desired,
                                 error, observed_speed, observed_horizontal_speed,
                                 phase, state.collision.has_collided])
                stream.flush()
                write_json(directory / 'follow_status.json',
                           {'running': True, 'target_id': target_id, 'phase': phase,
                            'backend': 'incremental_pose', 'heading_mode': 'preserve_current',
                            'horizontal_error_m': error,
                            'pose_displacement_limit_mps': args.speed,
                            'observed_speed_mps': observed_speed,
                            'observed_horizontal_speed_mps': observed_horizontal_speed,
                            'ignore_collision': args.ignore_collision,
                            'time': time.time()})
                time.sleep(0.2)
        except KeyboardInterrupt:
            reason = 'keyboard_interrupt'
        except Exception:
            reason = 'error'
            raise
        finally:
            released = True
            for operation in (control.cancelLastTask, lambda: control.enableApiControl(False)):
                try:
                    operation()
                except Exception as exc:
                    released = False
                    print(f'Control release failed: {exc}', flush=True)
            write_json(directory / 'follow_status.json',
                       {'running': False, 'target_id': target_id, 'reason': reason,
                        'control_released': released, 'time': time.time()})
            print(f'Follow ended; control_released={released}. No further pose updates.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--host', required=True, help='Verified AirSim host address')
    parser.add_argument('--port', type=int, default=41451)
    parser.add_argument('--target-id', type=int)
    parser.add_argument('--height', type=positive, default=25)
    parser.add_argument('--distance', type=positive, default=12)
    parser.add_argument('--speed', type=positive, default=12,
                        help='Maximum pose displacement rate in m/s; not a dynamics speed')
    parser.add_argument('--ignore-collision', action='store_true',
                        help='Allow pose updates to pass through scene collisions')
    parser.add_argument('--duration', type=float, default=90, help='Seconds; 0 until manual exit')
    parser.add_argument('--enable-pose-control', action='store_true',
                        help='Explicitly enable simulated pose following')
    args = parser.parse_args()
    if not args.enable_pose_control:
        parser.error('Following requires --enable-pose-control; this is not native flight control')
    if args.ignore_collision:
        print('WARNING: --ignore-collision may move the simulated vehicle through scenery.',
              file=sys.stderr, flush=True)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    follow(args)


if __name__ == '__main__':
    main()

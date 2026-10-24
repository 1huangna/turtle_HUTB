"""CARLA traffic tools. Run with the Python API matching the simulator."""

import argparse
import csv
import json
import math
import random
import time
from pathlib import Path

from common import write_json

ROLE = 'openhutb_traffic_demo'


def connect(args):
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    server = client.get_server_version()
    if client.get_client_version() != server:
        raise RuntimeError(f'Use the simulator-matched API (server: {server})')
    return client, client.get_world()


def export_references(world, directory):
    import carla
    references = []
    for obj in world.get_environment_objects(carla.CityObjectLabel.Any):
        if obj.name.endswith('_SM_0') and not obj.name.startswith('Instanced'):
            p = obj.transform.location
            references.append({'name': obj.name[:-5], 'carla': [p.x, p.y, p.z]})
    write_json(directory / 'references.json',
               {'map': world.get_map().name, 'references': references})


def spawn(args):
    client, world = connect(args)
    if world.get_settings().synchronous_mode:
        raise RuntimeError('Use an asynchronous world; this tool does not own world.tick()')
    if any(a.attributes.get('role_name') == ROLE for a in world.get_actors().filter('vehicle.*')):
        raise RuntimeError('Managed vehicles already exist; use record instead')
    blueprints = [bp for bp in world.get_blueprint_library().filter('vehicle.*')
                  if bp.has_attribute('number_of_wheels')
                  and bp.get_attribute('number_of_wheels').as_int() == 4
                  and (not bp.has_attribute('base_type')
                       or bp.get_attribute('base_type').as_str() == 'car')]
    if not blueprints:
        raise RuntimeError('No compatible car blueprints')
    points = world.get_map().get_spawn_points()
    random.Random(args.seed).shuffle(points)
    rng = random.Random(args.seed)
    manager = client.get_trafficmanager(args.tm_port)
    actors = []
    try:
        for point in points:
            blueprint = rng.choice(blueprints)
            blueprint.set_attribute('role_name', ROLE)
            actor = world.try_spawn_actor(blueprint, point)
            if actor is None:
                continue
            actors.append(actor)
            actor.set_autopilot(True, manager.get_port())
            if len(actors) == args.count:
                break
        if not actors:
            raise RuntimeError('All spawn points unavailable')
        print(f'Map: {world.get_map().name}; spawned IDs: {[a.id for a in actors]}', flush=True)
        while True:
            time.sleep(0.5)
    finally:
        for actor in actors:
            try:
                actor.destroy()
            except RuntimeError:
                pass  # World may already have been closed.


def record(args):
    _, world = connect(args)
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents mixing worlds or concurrent recorders.
    with (directory / 'vehicle_trajectories.csv').open('x', newline='', encoding='utf-8') as stream:
        export_references(world, directory)
        map_name = world.get_map().name
        writer = csv.writer(stream)
        writer.writerow(['wall_time', 'sim_time', 'frame', 'actor_id', 'vehicle_type',
                         'x_m', 'y_m', 'z_m', 'yaw_deg', 'vx_mps', 'vy_mps', 'vz_mps'])
        start = time.monotonic()
        previous = None
        rows = 0
        print(f'Recording {map_name} to {directory}', flush=True)
        try:
            while args.duration == 0 or time.monotonic() - start < args.duration:
                if world.get_map().name != map_name:
                    raise RuntimeError('Map changed; start a new recording')
                snapshot = world.get_snapshot()
                if snapshot.frame == previous:
                    time.sleep(1 / args.hz)
                    continue
                previous = snapshot.frame
                now = time.time()
                vehicles = []
                for actor in world.get_actors().filter('vehicle.*'):
                    if actor.attributes.get('role_name') != ROLE:
                        continue
                    state = snapshot.find(actor.id)
                    if state is None:
                        continue
                    transform = state.get_transform()
                    p, v = transform.location, state.get_velocity()
                    writer.writerow([now, snapshot.timestamp.elapsed_seconds, snapshot.frame,
                                     actor.id, actor.type_id, p.x, p.y, p.z,
                                     transform.rotation.yaw, v.x, v.y, v.z])
                    vehicles.append(dict(actor_id=actor.id, type=actor.type_id,
                                         x=p.x, y=p.y, z=p.z, yaw_deg=transform.rotation.yaw,
                                         vx=v.x, vy=v.y, vz=v.z))
                    rows += 1
                stream.flush()
                write_json(directory / 'live.json',
                           {'time': now, 'map': map_name, 'vehicles': vehicles})
                time.sleep(1 / args.hz)
        finally:
            write_json(directory / 'metadata.json',
                       {'map': map_name, 'duration_wall_s': time.monotonic() - start,
                        'rows': rows, 'sampling_hz_wall': args.hz,
                        'coordinates': 'CARLA world, left-handed, metres',
                        'source': 'CARLA actor ground truth'})


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('Must be positive and finite')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    commands = parser.add_subparsers(dest='command', required=True)
    spawner = commands.add_parser('spawn', help='Spawn traffic; Ctrl+C cleans owned cars')
    spawner.add_argument('--count', type=int, default=8)
    spawner.add_argument('--tm-port', type=int, default=8000)
    spawner.add_argument('--seed', type=int, default=0)
    recorder = commands.add_parser('record', help='Record existing managed traffic')
    recorder.add_argument('--output', type=Path, required=True)
    recorder.add_argument('--duration', type=float, default=90, help='Seconds; 0 until Ctrl+C')
    recorder.add_argument('--hz', type=positive, default=10)
    args = parser.parse_args()
    if args.command == 'spawn' and args.count < 1:
        parser.error('--count must be at least 1')
    if args.command == 'record' and (not math.isfinite(args.duration) or args.duration < 0):
        parser.error('--duration must be finite and nonnegative')
    try:
        (spawn if args.command == 'spawn' else record)(args)
    except KeyboardInterrupt:
        print('Stopped.')


if __name__ == '__main__':
    main()

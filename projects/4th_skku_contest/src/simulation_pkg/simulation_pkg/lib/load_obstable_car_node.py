import argparse
import os
import subprocess
from ament_index_python.packages import get_package_prefix


def _package_root() -> str:
    # Preferred: source tree root
    #   .../src/simulation_pkg/simulation_pkg/lib/load_obstable_car_node.py
    # -> .../src/simulation_pkg
    here = os.path.abspath(__file__)
    base = os.path.dirname(os.path.dirname(os.path.dirname(here)))

    # When executed from build tree, map back to src tree.
    marker = '/build/simulation_pkg'
    if marker in base:
        ws_root = base.split(marker)[0]
        src_base = os.path.join(ws_root, 'src', 'simulation_pkg')
        if os.path.exists(src_base):
            return src_base

    return base


def _spawn(entity_name: str, model_file: str, pose):
    x, y, z, roll, pitch, yaw = pose
    spawn_script = os.path.join(get_package_prefix('gazebo_ros'), 'lib', 'gazebo_ros', 'spawn_entity.py')

    cmd = [
        'python3', spawn_script,
        '-file', model_file,
        '-entity', entity_name,
        '-x', str(x),
        '-y', str(y),
        '-z', str(z),
        '-R', str(roll),
        '-P', str(pitch),
        '-Y', str(yaw),
    ]
    subprocess.run(cmd, check=False)


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--entity', type=str, default=None)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    args, _ = parser.parse_known_args()

    # 기본 동작: 기존처럼 다중 스폰
    obstacles = [
        ('obstacle1', (3.4, -5.1, 0.0, 0.0, 0.0, 0.9)),
        ('obstacle2', (4.9, -0.9, 0.0, 0.0, 0.0, 1.7)),
        ('obstacle3', (3.6, 2.5, 0.0, 0.0, 0.0, 1.7)),
    ]

    # 단일 스폰 모드: launch arguments로 좌표 직접 지정
    if args.entity:
        obstacles = [
            (args.entity, (args.x, args.y, args.z, args.roll, args.pitch, args.yaw))
        ]

    model_file = os.path.join(_package_root(), 'models', 'mercedes_obstacle', 'model.sdf')

    if not os.path.exists(model_file):
        print(f'[load_obstable_car_node] model file not found: {model_file}')
        return

    for name, pose in obstacles:
        _spawn(name, model_file, pose)


if __name__ == '__main__':
    main()

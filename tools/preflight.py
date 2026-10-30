#!/usr/bin/env python3
"""Read-only checks for assets required by a selected launch; no ROS imports."""
import argparse
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

PROJECTS = ('4th_skku_contest', 'autonomous_simulation')
MODELS = {
    'general': ['track.engine'],
    'mission': ['track.engine'],
    'parking': ['parking_front.engine', 'parking_rear.pt'],
    'general_sim': ['track.pt'],
    'mission_sim': ['track.pt', 'crosswalk.pt', 'traffic_light_sim.pt'],
    'parking_sim': ['parking_front.pt', 'parking_rear.pt'],
}


def check(workspace, scenario):
    workspace = workspace.resolve()
    missing, pointers, external, seen = set(), set(), set(), set()
    def require(path):
        path = path.resolve()
        if not path.is_file() or path.stat().st_size == 0:
            missing.add(str(path.relative_to(workspace)))
            return False
        with path.open('rb') as stream:
            if stream.read(128).startswith(b'version https://git-lfs.github.com/spec/v1'):
                pointers.add(str(path.relative_to(workspace)))
                return False
        return True

    names = MODELS[scenario]
    if workspace.name == 'autonomous_simulation':
        if scenario == 'general_sim':
            names = ['general_mission_with_crosswalk.pth']
        elif scenario in ('general', 'mission', 'parking'):
            names = [n.replace('.engine', '.pt') for n in names]
            if scenario == 'mission':
                names += ['crosswalk.pt']
    for name in names:
        require(workspace / 'src/camera_pkg/camera_pkg/model' / name)

    models = workspace / 'src/simulation_pkg/models'
    def visit(path):
        path = path.resolve()
        if not path.is_relative_to(workspace):
            external.add(str(path))
            return
        if path in seen:
            return
        seen.add(path)
        if path.is_dir():
            for child in sorted(path.rglob('*.material')):
                visit(child)
            return
        if not require(path):
            return
        if path.suffix in ('.sdf', '.world', '.urdf', '.config'):
            root = ET.parse(path).getroot()
            refs = [e.text.strip() for e in root.iter('uri') if e.text]
            refs += [e.attrib['filename'] for e in root.iter('mesh') if 'filename' in e.attrib]
            for ref in refs:
                if ref in ('model://sun', 'model://ground_plane', 'file://media/materials/scripts/gazebo.material'):
                    continue  # Gazebo Classic built-ins; checked at runtime.
                if ref.startswith('model://'):
                    target = models / ref[8:]
                    if '/' not in ref[8:]:
                        if not target.exists():
                            external.add(ref)
                            continue
                        target /= 'model.sdf'
                    visit(target)
                elif '://' in ref:
                    external.add(ref)
                else:
                    visit(path.parent / ref)
        elif path.suffix == '.obj':
            for line in path.read_text(errors='replace').splitlines():
                if line.startswith('mtllib '):
                    visit(path.parent / line[7:].strip())
        elif path.suffix == '.mtl':
            for line in path.read_text(errors='replace').splitlines():
                if re.match(r'\s*(map_\w+|bump)\s+', line):
                    # Recognize the bump-strength option used by the supplied vehicle MTLs.
                    ref = line.split(maxsplit=1)[1].strip()
                    ref = re.sub(r'^-bm\s+[-+0-9.eE]+\s+', '', ref)
                    if ref.startswith('-'):
                        external.add('MTL options require manual review: ' + str(path.relative_to(workspace)))
                    else:
                        visit(path.parent / ref)
        elif path.suffix == '.material':
            for ref in re.findall(r'^\s*texture\s+([^\r\n]+)', path.read_text(), re.M):
                # Gazebo resource paths are usually a sibling textures directory.
                candidates = list(path.parent.parent.rglob(ref.strip()))
                if not candidates:
                    missing.add(str((path.parent.parent / 'textures' / ref.strip()).relative_to(workspace)))

    if scenario.endswith('_sim'):
        sim = workspace / 'src/simulation_pkg'
        visit(sim / 'worlds' / ('parking.world' if scenario == 'parking_sim' else 'track.world'))
        visit(models / 'mercedes_split_tex/model.sdf')
    return {'project': workspace.name, 'scenario': scenario,
            'missing': sorted(missing), 'lfs_pointers': sorted(pointers),
            'external_references': sorted(external),
            'scope': 'File availability only; no build, inference, physics or hardware validation.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', choices=PROJECTS)
    parser.add_argument('scenario', choices=MODELS)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1] / 'projects' / args.project
    report = check(workspace.resolve(), args.scenario)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"{args.project}: {args.scenario}")
        for key in ('missing', 'lfs_pointers', 'external_references'):
            print(f"{key}: {len(report[key])}")
            for item in report[key]:
                print('  ' + item)
        print(report['scope'])
    return 1 if any(report[k] for k in ('missing', 'lfs_pointers', 'external_references')) else 0

if __name__ == '__main__':
    sys.exit(main())

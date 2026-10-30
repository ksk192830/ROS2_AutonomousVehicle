#!/usr/bin/env python3
"""Copy runtime assets from an authorized local Git repository, never publish them."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import sys

EXTENSIONS = {'.png', '.jpg', '.jpeg', '.obj', '.dae', '.fbx', '.stl', '.3ds', '.c4d', '.pt', '.pth'}
POINTER = b'version https://git-lfs.github.com/spec/v1'


def run(source, *args, data=None):
    return subprocess.run(['git', '-c', 'credential.helper=!gh auth git-credential',
                           '-C', str(source), *args], input=data,
                          stdout=subprocess.PIPE, check=True).stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Authorized local source repository (may be a partial clone)')
    parser.add_argument('project', choices=['4th_skku_contest', 'autonomous_simulation'])
    parser.add_argument('--ref', default='HEAD', help='Source revision; use a commit SHA for repeatability')
    args = parser.parse_args()
    source = args.source.resolve()
    root = Path(__file__).resolve().parents[1]
    destination = root / 'projects' / args.project
    commit = run(source, 'rev-parse', '--verify', args.ref + '^{commit}').decode().strip()
    entries = run(source, 'ls-tree', '-rz', commit).split(b'\0')
    count = 0
    for entry in entries:
        if not entry:
            continue
        info, name = entry.split(b'\t', 1)
        mode, kind, oid = info.decode().split()
        path = Path(name.decode())
        relative = path.as_posix()
        if not relative.startswith(('src/camera_pkg/camera_pkg/model/', 'src/simulation_pkg/models/')):
            continue
        if path.suffix.lower() not in EXTENSIONS or mode not in ('100644', '100755'):
            continue
        target = destination / path
        if not target.resolve().is_relative_to(destination.resolve()):
            raise ValueError('Asset path escapes workspace')
        blob = run(source, 'cat-file', 'blob', oid)
        digest = hashlib.sha1(b'blob ' + str(len(blob)).encode() + b'\0' + blob).hexdigest()
        if digest != oid:
            raise ValueError(f'Git checksum mismatch: {path}')
        if blob.startswith(POINTER):
            fields = dict(line.split(' ', 1) for line in blob.decode().splitlines())
            content = run(source, 'lfs', 'smudge', data=blob)
            if hashlib.sha256(content).hexdigest() != fields['oid'].removeprefix('sha256:') or len(content) != int(fields['size']):
                raise ValueError(f'LFS checksum mismatch: {path}')
        else:
            content = blob
        if target.exists():
            current = target.read_bytes()
            if current == content:
                continue
            # Resolve a destination LFS pointer only when its SHA and size match.
            if current.startswith(POINTER):
                fields = dict(line.split(' ', 1) for line in current.decode().splitlines())
                if hashlib.sha256(content).hexdigest() != fields['oid'].removeprefix('sha256:') or len(content) != int(fields['size']):
                    raise ValueError(f'Existing LFS asset differs: {path}')
            else:
                raise ValueError(f'Refusing to overwrite differing file: {path}')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
        # Keep recovered, previously untracked private assets local by default.
        tracked = subprocess.run(['git', '-C', str(root), 'ls-files', '--error-unmatch',
                                  str(target.relative_to(root))], capture_output=True).returncode == 0
        if not tracked:
            exclude = Path(run(root, 'rev-parse', '--git-path', 'info/exclude').decode().strip())
            if not exclude.is_absolute():
                exclude = root / exclude
            exclude.parent.mkdir(parents=True, exist_ok=True)
            with exclude.open('a') as stream:
                stream.write('\n/' + target.relative_to(root).as_posix() + '\n')
        count += 1
        print('Restored:', path, flush=True)
    print(f'Restored {count} assets. Nothing committed or published. Source revision: {commit}')

if __name__ == '__main__':
    main()

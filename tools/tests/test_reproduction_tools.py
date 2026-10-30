import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('preflight', ROOT / 'tools/preflight.py')
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class AssetChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.workspace = Path(self.temp.name) / '4th_skku_contest'

    def write(self, name, data):
        path = self.workspace / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(data)
        return path

    def test_lfs_pointer_is_not_a_runtime_model(self):
        self.write('src/camera_pkg/camera_pkg/model/track.engine',
                   'version https://git-lfs.github.com/spec/v1\n')
        result = preflight.check(self.workspace, 'general')
        self.assertEqual(len(result['lfs_pointers']), 1)

    def test_mesh_material_and_texture_dependencies(self):
        self.write('src/camera_pkg/camera_pkg/model/track.pt', 'weights')
        self.write('src/simulation_pkg/worlds/track.world',
                   '<sdf><world><include><uri>model://sun</uri></include></world></sdf>')
        self.write('src/simulation_pkg/models/mercedes_split_tex/model.sdf',
                   '<sdf><model><mesh><uri>model://mercedes_split_tex/meshes/body.obj</uri></mesh></model></sdf>')
        self.write('src/simulation_pkg/models/mercedes_split_tex/meshes/body.obj', 'mtllib body.mtl\n')
        self.write('src/simulation_pkg/models/mercedes_split_tex/meshes/body.mtl', 'map_Kd body.png\n')
        result = preflight.check(self.workspace, 'general_sim')
        self.assertEqual(len(result['missing']), 1)
        self.assertTrue(result['missing'][0].endswith('body.png'))
        self.assertEqual(result['external_references'], [])
        self.write('src/simulation_pkg/models/mercedes_split_tex/meshes/body.png', 'texture')
        self.assertEqual(preflight.check(self.workspace, 'general_sim')['missing'], [])

    def test_expo_general_requires_rfdetr_not_yolo(self):
        self.workspace = self.workspace.parent / 'autonomous_simulation'
        result = preflight.check(self.workspace, 'general_sim')
        self.assertIn('src/camera_pkg/camera_pkg/model/general_mission_with_crosswalk.pth', result['missing'])

    def test_simulation_loader_does_not_depend_on_checkout_depth(self):
        package = Path(self.temp.name) / 'deep' / 'arbitrary' / 'simulation_pkg'
        source = ROOT / 'projects/autonomous_simulation/src/simulation_pkg/simulation_pkg'
        shutil.copytree(source, package)
        ament = types.ModuleType('ament_index_python')
        packages = types.ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda name: '/installed/share/' + name
        fake_numpy = types.ModuleType('numpy')
        with patch.dict(sys.modules, {'ament_index_python': ament,
                                      'ament_index_python.packages': packages,
                                      'numpy': fake_numpy}):
            loader = importlib.util.spec_from_file_location('portable_simulation', package / '__init__.py')
            module = importlib.util.module_from_spec(loader)
            loader.loader.exec_module(module)
            self.assertEqual(module.basic.get_model('vehicle'),
                             '/installed/share/simulation_pkg/models/vehicle/model.sdf')

    def test_import_is_local_and_refuses_overwrite(self):
        root = Path(self.temp.name)
        source, dest = root / 'source', root / 'destination'
        for repo in (source, dest):
            subprocess.run(['git', 'init', '-q', str(repo)], check=True)
        asset = Path('src/camera_pkg/camera_pkg/model/track.pt')
        (source / asset).parent.mkdir(parents=True)
        (source / asset).write_bytes(b'original weights')
        subprocess.run(['git', '-C', str(source), 'add', '.'], check=True)
        subprocess.run(['git', '-C', str(source), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'fixture'], check=True)
        (dest / 'tools').mkdir()
        shutil.copy(ROOT / 'tools/import_assets.py', dest / 'tools')
        cmd = [sys.executable, str(dest / 'tools/import_assets.py'), str(source), '4th_skku_contest']
        subprocess.run(cmd, check=True, capture_output=True)
        target = dest / 'projects/4th_skku_contest' / asset
        self.assertEqual(target.read_bytes(), b'original weights')
        subprocess.run(['git', '-C', str(dest), 'check-ignore', str(target)], check=True, capture_output=True)
        target.write_bytes(b'local edits')
        self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
        self.assertEqual(target.read_bytes(), b'local edits')


if __name__ == '__main__':
    unittest.main()

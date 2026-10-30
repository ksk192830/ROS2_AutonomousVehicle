import os
import types
import marshal
from pathlib import Path
from . import lidar_perception_func_lib


def get_path(file_name=None):
    lib_path = Path(__file__).resolve().parent
    return str(lib_path / file_name) if file_name else str(lib_path)

def get_pyc(module_file):
    file_path = get_path(module_file)
    print('\n파일명:', file_path, '\n\n')
    pyc = open(file_path, 'rb').read()
    code = marshal.loads(pyc[16:])
    module = types.ModuleType('module_name')
    exec(code, module.__dict__)
    return module

# lidar_perception_func_lib = get_pyc("lidar_perception_func_lib.cpython-310.pyc")

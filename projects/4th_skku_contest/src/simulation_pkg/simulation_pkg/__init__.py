import os
import types
import marshal
import importlib.util
from pathlib import Path


_PACKAGE_DIR = Path(__file__).resolve().parent
_LIB_DIR = _PACKAGE_DIR / "lib"

def get_pyc(file_name):
    file_path = _LIB_DIR / "pyc" / file_name
    pyc = file_path.read_bytes()
    code = marshal.loads(pyc[16:])
    module = types.ModuleType('module_name')
    exec(code, module.__dict__)
    return module

def get_py(file_name):
    file_path = _LIB_DIR / file_name

    spec = importlib.util.spec_from_file_location("module_name", file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

basic = get_py("012_deploy_lib.py")

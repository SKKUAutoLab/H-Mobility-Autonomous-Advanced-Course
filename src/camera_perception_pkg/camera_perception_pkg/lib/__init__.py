import os
import types
import marshal

def get_path(file_name=None):
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6],*p[5:6], "lib", file_name)
    return LIB_PATH

def get_pyc(module_file):
    file_path = get_path(module_file)
    print('\n파일명:', file_path, '\n\n')
    pyc = open(file_path, 'rb').read()
    code = marshal.loads(pyc[16:])
    module = types.ModuleType('module_name')
    exec(code, module.__dict__)
    return module

camera_perception_func_lib = get_pyc("camera_perception_func_lib.cpython-310.pyc")

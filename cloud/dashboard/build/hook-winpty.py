"""PyInstaller runtime hook — bundle pywinpty's DLL + agent EXE."""
from PyInstaller.utils.hooks import collect_dynamic_libs, collect_data_files

binaries = collect_dynamic_libs("winpty")
datas = collect_data_files("winpty", include_py_files=False)

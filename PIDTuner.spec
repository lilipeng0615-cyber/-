# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller 打包配置。

处理两个常见坑:
  1. pyqtgraph 的部分模块是运行时动态导入的,PyInstaller 静态分析
     扫不到,需要 collect_submodules 全量收进来。
  2. PySide6 的 Qt 平台插件(windows/direct2d 等)必须一起打包,
     否则 exe 启动时报 "could not find or load the Qt platform plugin"。
"""
from PyInstaller.utils.hooks import collect_submodules, collect_data_files

hiddenimports = []
hiddenimports += collect_submodules("pyqtgraph")
# pyserial 的串口后端按平台动态导入,一并收进来
hiddenimports += collect_submodules("serial")

datas = []
datas += collect_data_files("pyqtgraph")

block_cipher = None

a = Analysis(
    ["main.py"],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # 砍掉明确用不到的重型库,减小体积
    excludes=["tkinter", "matplotlib", "PyQt5", "PyQt6", "PySide2", "test", "unittest"],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="PIDTuner",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,          # 纯 GUI,不弹控制台
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    # icon="app.ico",       # 有图标时取消注释
)

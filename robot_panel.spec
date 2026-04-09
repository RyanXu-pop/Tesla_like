# -*- mode: python ; coding: utf-8 -*-
# Windows 11: 在项目根目录执行 pyinstaller robot_panel.spec
# 生成 dist/RobotPanel/RobotPanel.exe（目录模式，便于 Qt 加载插件）

block_cipher = None

a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('config/config.example.yaml', 'config'),
        ('maps', 'maps'),
        ('data', 'data'),
        ('ros', 'ros'),
        ('scripts', 'scripts'),
    ],
    hiddenimports=[
        'qasync',
        'matplotlib.backends.backend_qtagg',
        'matplotlib.backends.backend_qt',
        # 曾用 sys.path 动态导入 components，显式列出以免漏打
        'src.ui_v2.components.manual_pose_dialog',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'PyQt6', 'PyQt5', 'PyQt6.QtCore', 'PyQt6.QtGui', 'PyQt6.QtWidgets',
        # 缩小体积、避免与 PySide6 冲突；Anaconda 环境下常被误收集
        'torch', 'tensorflow', 'cv2', 'sklearn', 'plotly', 'panel', 'holoviews',
        'bokeh', 'notebook', 'jupyter', 'jupyterlab', 'IPython', 'ipykernel',
        'sympy', 'gmpy2', 'pytest', 'pyarrow',
    ],
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='RobotPanel',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='RobotPanel',
)

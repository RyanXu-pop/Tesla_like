# Windows 11 打包与运行

## 环境要求

- Windows 11（64 位）
- 已安装 [Python 3.10+](https://www.python.org/downloads/)，安装时勾选「Add Python to PATH」

**强烈建议**用「仅本项目依赖」的虚拟环境打包，不要用装满 Jupyter / PyTorch 等的 Anaconda 默认环境，否则 exe 会非常大（数百 MB～数 GB），且可能触发多余模块的收集错误。示例：

```bat
cd 项目根目录
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt -r requirements-build.txt
python -m PyInstaller --noconfirm robot_panel.spec
```

## 一键打包

在项目根目录（与 `main.py` 同级）打开 **PowerShell** 或 **cmd**，执行：

```bat
packaging\build_windows.bat
```

成功后得到目录：

`dist\RobotPanel\`

其中 **`RobotPanel.exe`** 为无控制台窗口的图形程序；同级的 **`_internal`** 内含依赖与资源。

将整个 **`RobotPanel` 文件夹** 拷贝到目标电脑同一相对路径下即可运行（不要只拷单个 exe）。

## 首次运行配置

分发包内默认带有 `config\config.example.yaml`。若不存在 `config\config.yaml`，程序会自动用示例文件启动。请在 **`_internal\config`** 下复制一份并重命名为 `config.yaml`，按实际机器人修改 SSH、MQTT 等参数（也可在程序内「系统设置」中保存，会写入 `config\config.yaml`）。

## 常见问题

- **双击无反应或闪退**：用命令行运行 `RobotPanel.exe` 查看报错；或暂时把 `robot_panel.spec` 里 `console=False` 改为 `console=True` 重新打包以显示控制台。
- **缺少 Qt 平台插件**：多为杀毒软件误删 `_internal\PySide6` 下文件，将 `RobotPanel` 目录加入白名单后重解压/重打包。
- **本机调试用源码**：仍可直接 `python main.py`，逻辑与打包版一致（非 frozen 时不会切换 `_internal` 目录）。

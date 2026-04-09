@echo off
REM Run from Explorer or cmd. Project root = parent of this folder.
cd /d "%~dp0.."

python -m pip install -U pip
if errorlevel 1 exit /b 1

python -m pip install -r requirements.txt -r requirements-build.txt
if errorlevel 1 exit /b 1

python -m PyInstaller --noconfirm robot_panel.spec
if errorlevel 1 exit /b 1

echo.
echo OK: dist\RobotPanel\RobotPanel.exe
exit /b 0

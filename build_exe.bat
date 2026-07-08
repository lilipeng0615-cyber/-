@echo off
setlocal
cd /d "%~dp0"
python -m pip install -r requirements.txt pyinstaller
pyinstaller --noconfirm PIDTuner.spec
echo.
echo Build output: dist\PIDTuner.exe

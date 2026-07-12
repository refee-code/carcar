@echo off
cd /d "%~dp0"
echo If serial connection is needed, run: python -m pip install -r requirements.txt
python app.py
pause

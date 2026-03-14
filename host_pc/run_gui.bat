@echo off
setlocal
cd /d %~dp0
if not exist ..\.venv\Scripts\python.exe (
  echo Python virtual environment not found at ..\.venv\Scripts\python.exe
  echo Please create it first, then run: pip install -r requirements.txt
  pause
  exit /b 1
)
..\.venv\Scripts\python.exe main.py

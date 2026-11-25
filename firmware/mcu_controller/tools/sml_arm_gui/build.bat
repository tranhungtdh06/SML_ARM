@echo off
cd /d "%~dp0"

echo ========================================
echo       SML ARM STUDIO - BUILD TOOL
echo ========================================

".venv\Scripts\python.exe" -m PyInstaller ^
    --noconfirm ^
    --clean ^
    --onefile ^
    --windowed ^
    --name "SML Arm Studio" ^
    main.py

if errorlevel 1 (
    echo.
    echo BUILD FAILED!
    pause
    exit /b 1
)

echo.
echo BUILD SUCCESSFUL!
echo Output: dist\SML Arm Studio.exe

explorer dist
pause

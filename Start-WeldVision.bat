@echo off
title WeldVision X5 - Starting...
color 0A

echo.
echo ================================================
echo   WeldVision X5 - Model Training Platform
echo ================================================
echo.
echo Starting application...
echo.

cd /d "%~dp0"

REM Check if Node.js is installed
where node >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Node.js is not installed!
    echo Please install Node.js from https://nodejs.org/
    pause
    exit /b 1
)

REM Check if Python is installed
where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python is not installed!
    echo Please install Python 3.9+ from https://www.python.org/
    pause
    exit /b 1
)

echo [OK] Node.js and Python detected
echo.

REM Start backend server
echo [1/3] Starting backend server...
start "WeldVision Backend" /MIN cmd /k "cd backend && python app.py"

REM Wait for backend to start
timeout /t 5 /nobreak >nul

REM Start frontend dev server
echo [2/3] Starting frontend server...
start "WeldVision Frontend" /MIN cmd /k "npm run dev"

REM Wait for frontend to start
echo [3/3] Waiting for services to initialize...
timeout /t 8 /nobreak >nul

echo.
echo ================================================
echo   WeldVision X5 is starting...
echo ================================================
echo.
echo Opening browser in 3 seconds...
timeout /t 3 /nobreak >nul

REM Open browser
start http://localhost:3000

echo.
echo Application is running!
echo.
echo To stop: Close both console windows or press Ctrl+C
echo.
pause

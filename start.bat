@echo off
echo Starting WeldVision X5...
echo.

:: Start Backend in new window
echo Starting Backend Server...
start "WeldVision Backend" /min cmd /k "cd /d D:\WeldMaster AI Evaluation\backend && python app.py"

:: Wait for backend to initialize
timeout /t 4 /nobreak > nul

:: Start Frontend in current window
echo Starting Frontend...
npm run dev

pause

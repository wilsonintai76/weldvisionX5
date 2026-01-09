@echo off
echo Starting WeldMaster Hybrid System (Web Version)...
echo 1. Starting Django Backend (Port 8000)
echo 2. Starting Vite Frontend (Port 3002)

cd /d "%~dp0"
npm start
pause

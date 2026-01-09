# WeldVision X5 Startup Script
Write-Host "Starting WeldVision X5 (Django Brain + React UI)..." -ForegroundColor Cyan

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $repoRoot

Write-Host "Starting full stack..." -ForegroundColor Yellow
Write-Host "- Django API: http://localhost:8000" -ForegroundColor DarkGray
Write-Host "- React UI:   http://localhost:3002" -ForegroundColor DarkGray

# This runs both Django and Vite via `concurrently`.
npm run start

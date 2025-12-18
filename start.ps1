# WeldVision X5 Startup Script
Write-Host "Starting WeldVision X5..." -ForegroundColor Cyan

# Start Backend in background job
Write-Host "Starting Backend Server..." -ForegroundColor Yellow
Start-Job -ScriptBlock { 
    Set-Location 'D:\WeldMaster AI Evaluation\backend'
    python app.py 
} -Name "WeldVisionBackend"

# Wait for backend to initialize
Start-Sleep -Seconds 3

# Check if backend is running
try {
    $response = Invoke-WebRequest -Uri "http://localhost:5000/api/health" -UseBasicParsing -ErrorAction Stop
    Write-Host "✓ Backend started successfully on port 5000" -ForegroundColor Green
} catch {
    Write-Host "⚠ Backend may still be starting..." -ForegroundColor Yellow
}

# Start Frontend in current window
Write-Host "Starting Frontend..." -ForegroundColor Yellow
npm run dev

# Cleanup backend when frontend stops
Write-Host "`nStopping Backend..." -ForegroundColor Yellow
Get-Job -Name "WeldVisionBackend" | Stop-Job
Get-Job -Name "WeldVisionBackend" | Remove-Job
Write-Host "✓ All services stopped" -ForegroundColor Green

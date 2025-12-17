# Create placeholder icon files for Electron build
# You should replace these with actual icons later

# Create assets directory
$assetsDir = "d:\WeldMaster AI Evaluation\assets"
if (-not (Test-Path $assetsDir)) {
    New-Item -ItemType Directory -Path $assetsDir | Out-Null
    Write-Host "Created assets directory" -ForegroundColor Green
}

# Create a simple placeholder message
$placeholderText = @"
PLACEHOLDER ICON FILES

Replace these with actual icons:
- icon.ico (256x256 Windows)
- icon.png (512x512 Linux)
- icon.icns (macOS)
- tray-icon.png (16x16 system tray)

Create icons at: https://convertico.com/
Or use GIMP, Photoshop, etc.
"@

$placeholderText | Out-File -FilePath (Join-Path $assetsDir "README.txt")

Write-Host "Assets folder ready. Add icon files before building installer." -ForegroundColor Yellow
Write-Host "Location: $assetsDir" -ForegroundColor Cyan

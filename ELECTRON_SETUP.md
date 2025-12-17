# WeldVision X5 - Electron Desktop App Setup

## Quick Start (Development)

### Option 1: Simple Batch File (Recommended for Testing)

Just double-click `Start-WeldVision.bat` - it will:
1. Start the backend Flask server
2. Start the frontend Vite dev server  
3. Open your browser to http://localhost:3000

**No Electron needed for basic usage!**

---

## Full Electron Desktop App (Professional Install)

### Step 1: Install Electron Dependencies

```powershell
# Navigate to project
cd "D:\WeldMaster AI Evaluation"

# Install Electron packages
npm install --save-dev electron@28.0.0 electron-builder@24.9.1 concurrently@8.2.2 wait-on@7.2.0 cross-env@7.0.3
```

### Step 2: Create Icon Assets

Create an `assets` folder and add icons:

```
d:\WeldMaster AI Evaluation\
└── assets\
    ├── icon.ico      # 256x256 Windows icon
    ├── icon.png      # 512x512 PNG for Linux
    ├── icon.icns     # macOS icon
    └── tray-icon.png # 16x16 system tray icon
```

**Quick icon creation:**
- Use any 256x256 PNG image
- Convert online at: https://convertico.com/
- Or use provided placeholder icons

### Step 3: Run in Development Mode

```powershell
# Start Electron app with hot reload
npm run electron-dev
```

This will:
- Start Vite dev server (hot reload enabled)
- Wait for frontend to be ready
- Launch Electron window
- Start backend Flask server automatically

### Step 4: Build Installer

#### Windows Installer (.exe)

```powershell
# Build production installer
npm run dist:win
```

Output: `release/WeldVision-X5-Setup-1.0.0.exe`

#### Portable Version

```powershell
npm run dist
```

Output: `release/WeldVision-X5-Portable-1.0.0.exe` (no installation needed)

---

## Package Scripts

```json
{
  "electron": "electron .",              // Run Electron (frontend must be running)
  "electron-dev": "...",                 // Development mode with hot reload
  "electron-build": "...",               // Build frontend + package Electron
  "pack": "electron-builder --dir",      // Package without creating installer
  "dist": "electron-builder",            // Create all installers
  "dist:win": "electron-builder --win"   // Windows installer only
}
```

---

## Electron Features

### What You Get:

✅ **Native Desktop App**
- No browser chrome/tabs
- Custom window frame
- Windows taskbar integration
- System tray icon

✅ **Auto-Start Backend**
- Electron starts Python Flask server automatically
- Auto-restart on crash
- Graceful shutdown

✅ **Professional Install**
- NSIS installer with custom options
- Desktop shortcut
- Start menu entry
- Auto-updates (future)

✅ **System Integration**
- Minimize to tray
- Windows notifications
- File associations (future)

---

## File Structure

```
d:\WeldMaster AI Evaluation\
├── electron-main.js        # Electron main process
├── preload.js             # Security bridge
├── package.json           # Electron config + build settings
├── Start-WeldVision.bat   # Simple launcher (no Electron)
├── assets/                # Icons and resources
│   ├── icon.ico
│   ├── icon.png
│   └── tray-icon.png
├── backend/               # Flask API (bundled in installer)
│   └── app.py
├── dist/                  # Vite build output
└── release/               # Electron installers output
    ├── WeldVision-X5-Setup-1.0.0.exe
    └── WeldVision-X5-Portable-1.0.0.exe
```

---

## Build Configuration

### Key Settings in `package.json`:

```json
{
  "main": "electron-main.js",
  "build": {
    "appId": "com.weldvision.x5",
    "productName": "WeldVision X5",
    "files": [
      "dist/**/*",           // Frontend build
      "electron-main.js",
      "preload.js",
      "backend/**/*"         // Python backend (bundled)
    ],
    "extraResources": [
      "backend"              // Backend available in resources/
    ],
    "win": {
      "target": ["nsis", "portable"],
      "icon": "assets/icon.ico"
    }
  }
}
```

---

## Troubleshooting

### Electron install fails

**Solution 1:** Install without optional dependencies:
```powershell
npm install --save-dev electron --no-optional
```

**Solution 2:** Use offline installer:
1. Download: https://github.com/electron/electron/releases
2. Set environment variable:
   ```powershell
   $env:ELECTRON_MIRROR="https://npmmirror.com/mirrors/electron/"
   npm install electron
   ```

### Backend doesn't start

**Check Python path in electron-main.js:**
```javascript
const pythonPath = 'python'; // Change to full path if needed
// e.g., 'C:\\Python311\\python.exe'
```

### Build fails

**Common issues:**
1. Missing icon files → Create placeholder icons
2. Large backend → Exclude venv: `"!backend/venv/**/*"`
3. Missing LICENSE → Create empty LICENSE file

---

## Production Deployment

### For End Users:

**Simple Install:**
1. Run `WeldVision-X5-Setup-1.0.0.exe`
2. Follow installer wizard
3. Launch from Desktop or Start Menu

**Portable Version:**
1. Extract `WeldVision-X5-Portable-1.0.0.exe`
2. Run from any folder (USB drive compatible)

### System Requirements:

- **Windows:** 10/11 (64-bit)
- **RAM:** 4GB minimum, 8GB recommended
- **Storage:** 500MB for app + datasets
- **Python:** Bundled (no install needed in production)

---

## Next Steps

### Phase 1 (Current) - Basic Launcher ✅
- Use `Start-WeldVision.bat`
- Perfect for development and testing

### Phase 2 - Electron App
- Install Electron dependencies
- Test with `npm run electron-dev`
- Build installer with `npm run dist:win`

### Phase 3 - Production
- Code signing certificate
- Auto-update server
- Crash reporting
- Analytics

---

## Quick Commands

```powershell
# Development (no Electron)
.\Start-WeldVision.bat

# Development (with Electron)
npm run electron-dev

# Build for distribution
npm run dist:win

# Test built app before installer
npm run pack
```

---

## Support

**Issues?**
- Check logs in Electron DevTools (F12)
- Backend logs in console window
- Python errors in backend terminal

**Need Help?**
- See `DEPLOYMENT_INSTRUCTIONS.md` for full guide
- Check `NETWORK_SETUP.md` for connectivity
- Review `AI_MODEL_TRAINING_GUIDE.md` for workflow

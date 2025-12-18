const { app, BrowserWindow, Menu, Tray, nativeImage } = require('electron');
const path = require('path');
const { spawn } = require('child_process');
const waitOn = require('wait-on');

let mainWindow;
let backendProcess;
let tray;

// Backend server configuration
const BACKEND_PORT = 5000;
const FRONTEND_URL = process.env.NODE_ENV === 'development' 
  ? 'http://localhost:3002' // Development uses Vite on 3002
  : `file://${path.join(__dirname, 'dist', 'index.html')}`; // Production uses built files

function createWindow() {
  // Create the browser window
  mainWindow = new BrowserWindow({
    width: 1600,
    height: 1000,
    minWidth: 1200,
    minHeight: 800,
    title: 'WeldVision X5 - Model Training Platform',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      nodeIntegration: false,
      contextIsolation: true,
      webSecurity: true
    },
    icon: path.join(__dirname, 'assets', 'icon.png'),
    backgroundColor: '#0f172a', // slate-900
    show: false // Don't show until ready
  });

  // Show window when ready to avoid flashing
  mainWindow.once('ready-to-show', () => {
    mainWindow.show();
  });

  // Remove default menu
  Menu.setApplicationMenu(null);

  // Load the frontend
  mainWindow.loadURL(FRONTEND_URL);

  // Open DevTools in development
  if (process.env.NODE_ENV === 'development') {
    mainWindow.webContents.openDevTools();
  }

  // Handle window close
  mainWindow.on('close', (event) => {
    if (!app.isQuiting) {
      event.preventDefault();
      mainWindow.hide();
      return false;
    }
  });

  mainWindow.on('closed', () => {
    mainWindow = null;
  });
}

function createTray() {
  const iconPath = path.join(__dirname, 'assets', 'tray-icon.png');
  const trayIcon = nativeImage.createFromPath(iconPath).resize({ width: 16, height: 16 });
  
  tray = new Tray(trayIcon);
  
  const contextMenu = Menu.buildFromTemplate([
    { 
      label: 'Show WeldVision X5', 
      click: () => {
        if (mainWindow) {
          mainWindow.show();
        }
      }
    },
    { type: 'separator' },
    { 
      label: 'Restart Backend', 
      click: () => {
        stopBackend();
        setTimeout(startBackend, 1000);
      }
    },
    { type: 'separator' },
    { 
      label: 'Quit', 
      click: () => {
        app.isQuiting = true;
        app.quit();
      }
    }
  ]);
  
  tray.setToolTip('WeldVision X5');
  tray.setContextMenu(contextMenu);
  
  tray.on('double-click', () => {
    if (mainWindow) {
      mainWindow.show();
    }
  });
}

function startBackend() {
  console.log('Starting backend server...');
  
  const isDev = process.env.NODE_ENV === 'development';
  const backendPath = isDev 
    ? path.join(__dirname, 'backend', 'app.py')
    : path.join(process.resourcesPath, 'backend', 'app.py');
  
  const pythonPath = isDev
    ? 'python' // Use system Python in dev
    : path.join(process.resourcesPath, 'python', 'python.exe'); // Bundled Python in production
  
  console.log(`Python: ${pythonPath}`);
  console.log(`Backend: ${backendPath}`);
  
  backendProcess = spawn(pythonPath, [backendPath], {
    cwd: path.dirname(backendPath),
    env: { 
      ...process.env, 
      FLASK_ENV: 'production',
      FLASK_DEBUG: '0',
      PYTHONUNBUFFERED: '1'
    }
  });

  backendProcess.stdout.on('data', (data) => {
    console.log(`[Backend] ${data}`);
  });

  backendProcess.stderr.on('data', (data) => {
    console.error(`[Backend Error] ${data}`);
  });

  backendProcess.on('close', (code) => {
    console.log(`Backend process exited with code ${code}`);
    if (!app.isQuiting && code !== 0) {
      // Auto-restart backend if it crashes
      console.log('Restarting backend in 3 seconds...');
      setTimeout(startBackend, 3000);
    }
  });
}

function stopBackend() {
  if (backendProcess) {
    console.log('Stopping backend server...');
    backendProcess.kill();
    backendProcess = null;
  }
}

async function waitForBackend() {
  console.log('Waiting for backend to be ready...');
  
  try {
    await waitOn({
      resources: [`http://localhost:${BACKEND_PORT}/api/health`],
      timeout: 30000, // 30 seconds
      interval: 1000, // Check every 1 second
      window: 1000 // Stability window
    });
    console.log('Backend is ready!');
    return true;
  } catch (error) {
    console.error('Backend failed to start:', error);
    return false;
  }
}

// App lifecycle
app.on('ready', async () => {
  // Start backend first
  startBackend();
  
  // Wait for backend to be ready
  const backendReady = await waitForBackend();
  
  if (!backendReady) {
    console.error('Failed to start backend. Exiting...');
    app.quit();
    return;
  }
  
  // Create system tray
  createTray();
  
  // Create main window
  createWindow();
});

app.on('window-all-closed', () => {
  // Keep app running in system tray on Windows/Linux
  if (process.platform === 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  if (mainWindow === null) {
    createWindow();
  } else {
    mainWindow.show();
  }
});

app.on('before-quit', () => {
  app.isQuiting = true;
});

app.on('quit', () => {
  stopBackend();
});

// Handle uncaught exceptions
process.on('uncaughtException', (error) => {
  console.error('Uncaught Exception:', error);
});

process.on('unhandledRejection', (reason, promise) => {
  console.error('Unhandled Rejection at:', promise, 'reason:', reason);
});

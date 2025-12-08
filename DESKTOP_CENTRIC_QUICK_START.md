# Desktop-Centric Implementation Guide

## Quick Start (30 Minutes)

### Step 1: Setup Desktop Backend (10 min)

```bash
# Create backend directory
mkdir desktop_backend
cd desktop_backend

# Create requirements.txt
cat > requirements.txt << EOF
Flask==2.3.2
Flask-CORS==4.0.0
Flask-SocketIO==5.3.4
PyTorch==2.0.0
numpy==1.24.3
opencv-python==4.8.0.76
python-socketio==5.9.0
python-engineio==4.7.1
requests==2.31.0
EOF

# Install dependencies
pip install -r requirements.txt
```

### Step 2: Create Server (Copy from DESKTOP_CENTRIC_ARCHITECTURE.md)

```bash
# Copy server.py code from documentation
cat > server.py << 'EOF'
# Paste entire server.py code from documentation
EOF

# Start server
python server.py
# Output: Running on http://0.0.0.0:5001
```

### Step 3: Setup Desktop UI (10 min)

```bash
# Create React app
npx create-react-app desktop_ui --template typescript

cd desktop_ui

# Install Socket.IO client
npm install socket.io-client

# Copy App.tsx (from documentation)
cp ../App.tsx src/

# Start UI
npm start
# Output: http://localhost:5002
```

### Step 4: Configure RDK (5 min)

```bash
# On RDK device (SSH connection)
ssh rdk@192.168.1.50

# Create camera service directory
mkdir /opt/weldvision/backend/client
cd /opt/weldvision/backend/client

# Copy rdk_camera_service.py (from documentation)
nano rdk_camera_service.py
# Paste code from documentation, save with Ctrl+X

# Make it executable
chmod +x rdk_camera_service.py

# Install RDK dependencies
pip install flask flask-socketio opencv-python

# Start service
python rdk_camera_service.py &
# Output: Running on 0.0.0.0:5000
```

### Step 5: Verify Connection (5 min)

```bash
# Test RDK health
curl http://192.168.1.50:5000/api/health

# Test desktop health
curl http://localhost:5001/api/health

# Open browser
# http://localhost:5002
# Should show "RDK Connected" in UI
```

---

## Architecture in Action

### Real-Time Evaluation Workflow

```
Timeline:
─────────────────────────────────────────────────────

T=0s: User clicks "Start Scanning"
     └─ UI sends signal to desktop
     └─ Desktop signals RDK to start streaming

T=0.5s: Frame 1 captured on RDK
       └─ RDK uploads frame to desktop (2-3 sec)

T=3s: Frame 1 received at desktop
     └─ Desktop runs inference (25 ms)
     └─ Result: {class: "good", confidence: 0.97}

T=3.1s: Result broadcast to UI
       └─ UI displays live evaluation

T=3.5s: Frame 2 captured on RDK
T=6s: Frame 2 received, evaluated...
T=9s: Frame 3...
...continues indefinitely...

User sees: Real-time evaluation results as frames arrive
Latency: ~3-4 seconds per frame (network dependent)
```

### Training Workflow

```
Timeline:
─────────────────────────────────────────────────────

Day 1-2: Data Collection
        Scanning → RDK captures frames
                → Desktop receives & stores
                → Collects 500 frames with labels

Day 3: Training
       UI: Click "Start Training"
       └─ Desktop loads 500 frames
       └─ Starts training loop (100 epochs)
       
       Epoch 1: Loss=0.45, Acc=89%  → UI updates
       Epoch 50: Loss=0.12, Acc=95% → UI updates
       Epoch 100: Loss=0.08, Acc=96% → Completed

Day 4: Deploy
       UI: Download trained model
       └─ Copy to RDK
       └─ Restart RDK service
       └─ RDK now uses ML model for evaluation

Improvement: 60% → 96% accuracy
Time to train: 2-4 hours
```

---

## Code Examples

### Example 1: Start Live Scanning

**Desktop UI (React):**

```typescript
const startScanning = async () => {
  try {
    // Signal RDK to start streaming
    const response = await fetch(
      `http://localhost:5001/api/rdk/${selectedRDK}/start-streaming`,
      { method: 'POST' }
    );
    setIsRecording(true);
    // Frames will arrive and trigger evaluation events
  } catch (error) {
    console.error('Start failed:', error);
  }
};
```

**What happens:**
1. UI sends request to desktop
2. Desktop signals RDK to start capturing
3. RDK opens camera, starts frame capture loop
4. RDK begins uploading frames to desktop
5. Desktop receives frames, runs inference
6. Results broadcast back to UI via WebSocket
7. UI displays live evaluation

### Example 2: Start Training

**Desktop UI (React):**

```typescript
const startTraining = async () => {
  try {
    const response = await fetch(
      'http://localhost:5001/api/training/start',
      {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          config: {
            epochs: 100,
            batch_size: 32,
            learning_rate: 0.001
          }
        })
      }
    );
    const data = await response.json();
    console.log('Training job:', data.job_id);
    // UI will receive training_progress events
  } catch (error) {
    console.error('Training failed:', error);
  }
};
```

**What happens:**
1. UI sends training request with config
2. Desktop creates training job (background thread)
3. Desktop loads collected frames from disk
4. Training loop starts (100 epochs)
5. Each epoch, desktop broadcasts progress
6. UI updates charts in real-time
7. After 100 epochs, model exports to ONNX
8. Model available in models list

### Example 3: Download Model

**Desktop UI (React):**

```typescript
const downloadModel = async (modelName: string) => {
  try {
    const response = await fetch(
      `http://localhost:5001/api/models/download/${modelName}`
    );
    
    const blob = await response.blob();
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${modelName}.onnx`;
    a.click();
  } catch (error) {
    console.error('Download failed:', error);
  }
};
```

**What happens:**
1. UI requests model download
2. Desktop reads ONNX file from disk
3. File streamed to UI browser
4. Browser downloads file
5. User copies to RDK
6. RDK loads new model
7. Next evaluation uses new model

---

## Configuration Guide

### Desktop Server Configuration

**File: `desktop_backend/config.py`**

```python
# Server configuration
FLASK_HOST = '0.0.0.0'
FLASK_PORT = 5001

# Training configuration
TRAINING_CONFIG = {
    'epochs': 100,
    'batch_size': 32,
    'learning_rate': 0.001,
    'device': 'cuda'  # or 'cpu'
}

# Data paths
DATA_DIR = './data'
FRAMES_DIR = './data/frames'
MODELS_DIR = './models'

# Database
DATABASE = './data/evaluation.db'

# Logging
LOG_LEVEL = 'INFO'
LOG_FILE = './logs/server.log'
```

### RDK Configuration

**File: `rdk_camera_service.py`**

```python
class RDKCameraService:
    def __init__(self, device_id=0, fps=10):
        self.device_id = device_id  # Camera device ID
        self.fps = fps               # Frames per second
        self.width = 1920            # Resolution width
        self.height = 1080           # Resolution height
```

**Configure:**
```python
# Set RDK specific parameters
service = RDKCameraService(device_id=0, fps=10)
service.initialize_camera()
# Adjust fps (5-30) based on network bandwidth
# Adjust resolution (1280x720, 1920x1080, 2560x1440)
```

### Network Configuration

**WiFi Connection (Desktop):**

```python
# desktop_backend/server.py
if __name__ == '__main__':
    socketio.run(
        app,
        host='0.0.0.0',  # Listen on all interfaces
        port=5001,
        debug=False
    )
```

**WiFi Connection (RDK):**

```python
# On RDK, connect to desktop IP
desktop_url = 'http://192.168.1.100:5001'
camera_service.connect_to_desktop(desktop_url)
```

**USB Connection (RDK):**

```bash
# On desktop, forward RDK port
adb forward tcp:5000 tcp:5000

# RDK camera service runs on 5000
# Desktop accesses via localhost:5000
```

---

## Performance Tuning

### Optimize Frame Uploads

**Default (3-4 sec per frame):**
```python
# RDK uploads at 1.3 MB/frame
# Resolution: 1920x1080
# FPS: 10

# Time calculation:
# 1.3 MB × 8 bits/byte ÷ 100 Mbps = 0.1 sec
# + Compression overhead: 0.5 sec
# + Network latency: 0.5 sec
# = ~1 sec per frame
```

**Optimize for speed:**
```python
# Option 1: Reduce resolution
resolution = (1280, 720)  # 0.8 MB/frame

# Option 2: Reduce FPS
fps = 5  # Capture every other frame

# Option 3: Compress more
compression_quality = 75  # Default 85

# Option 4: USB connection
# 10 Mbps → faster than WiFi for local setup
```

### Optimize Training Speed

**Use GPU:**
```python
import torch

# Check GPU availability
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0))

# In training code
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = model.to(device)

# Speed improvement: 10x faster (10 hours → 1 hour)
```

**Reduce training time:**
```python
# Reduce epochs (quality tradeoff)
epochs = 50  # Default 100
# Training time: 2-4 hours → 1-2 hours

# Increase batch size (needs more RAM)
batch_size = 64  # Default 32
# Training time: 2-4 hours → 1-2 hours

# Use mixed precision
from torch.cuda.amp import autocast
# Training time: 2-4 hours → 1.5-3 hours
```

---

## Multi-RDK Setup

### Connect 2 RDK Devices

**Desktop automatically supports multiple RDKs:**

```python
# Desktop RDK Manager handles multiple clients
rdk_manager = RDKClientManager()

# RDK1 registers
POST /api/rdk/register
{
  "device_id": "rdk-001",
  "camera": "stereo",
  "resolution": "1920x1080"
}

# RDK2 registers
POST /api/rdk/register
{
  "device_id": "rdk-002",
  "camera": "stereo",
  "resolution": "1920x1080"
}

# Desktop receives frames from both
# UI shows both RDK devices
# Can evaluate both simultaneously
```

**Benefits:**
- Multi-angle evaluation
- Parallel frame processing
- Combined model training
- Redundancy (if one fails, other works)

---

## Troubleshooting Checklist

### Problem: RDK Not Appearing in UI

**Checklist:**
```
☐ Desktop server running? 
  curl http://localhost:5001/api/health

☐ RDK service running?
  ssh rdk && curl http://localhost:5000/api/health

☐ Network connectivity?
  From RDK: ping <desktop_ip>
  From Desktop: ping <rdk_ip>

☐ RDK registered?
  curl http://localhost:5001/api/rdk/clients

☐ WebSocket connection?
  Browser → DevTools → Network → WS status

☐ Firewall blocking?
  Windows: Allow port 5001
  Linux: ufw allow 5001
```

### Problem: Frames Not Arriving

**Checklist:**
```
☐ RDK streaming started?
  curl -X POST http://rdk_ip:5000/api/rdk/start-streaming

☐ Camera working?
  RDK: python -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"

☐ Frame encoding correct?
  Desktop logs show incoming frames?
  tail -f ./logs/server.log | grep "frame"

☐ Network MTU sufficient?
  Frames are 1.3 MB
  Check: netstat -in | grep MTU
  Should be > 1500

☐ Desktop receiving on port 5001?
  netstat -an | grep 5001
```

### Problem: Training Very Slow

**Checklist:**
```
☐ Using GPU?
  nvidia-smi
  Should show training process using CUDA

☐ Disk I/O bottleneck?
  top → Show disk I/O during training
  Try SSD instead of HDD

☐ CPU fully utilized?
  top → Should show 80-90% CPU usage
  If not, training not optimized

☐ Batch size appropriate?
  32 is reasonable
  Try 16 if out of memory
  Try 64 if have GPU memory

☐ Model type efficient?
  ResNet50 is good balance
  Could use ResNet18 for speed
  Could use ResNet101 for accuracy
```

---

## Data Storage

### Frame Storage Structure

```
desktop_backend/
└── data/
    ├── frames/
    │   ├── good/
    │   │   ├── frame_001.jpg
    │   │   ├── frame_002.jpg
    │   │   └── ... (hundreds more)
    │   ├── porosity/
    │   │   ├── frame_001.jpg
    │   │   └── ... (defect frames)
    │   ├── undercut/
    │   └── crack/
    ├── evaluation.db
    │   └── Stores metadata, job tracking
    └── training_logs/
        └── tensorboard events
```

### Backup Strategy

**Daily backup:**
```bash
#!/bin/bash
# backup.sh

DATE=$(date +%Y%m%d_%H%M%S)
SOURCE="./data/frames"
DEST="/backup/weldvision_${DATE}.tar.gz"

tar -czf $DEST $SOURCE
echo "Backed up to $DEST"
```

**Run daily:**
```bash
# Add to crontab
0 2 * * * /home/user/weldvision/backup.sh
# Runs every day at 2 AM
```

---

## Monitoring

### Desktop Server Health

```bash
# Check server status
curl http://localhost:5001/api/health

# Output:
# {
#   "status": "healthy",
#   "connected_rdks": 2,
#   "timestamp": "2024-01-15T10:30:45"
# }
```

### RDK Status

```bash
# Check RDK status
curl http://rdk_ip:5000/api/rdk/status

# Output:
# {
#   "device_id": "rdk-001",
#   "connection": "connected",
#   "is_streaming": true,
#   "frames_captured": 1250,
#   "fps": 10,
#   "desktop_url": "http://192.168.1.100:5001"
# }
```

### Training Progress

```bash
# Check training job status
curl http://localhost:5001/api/training/job123/status

# Output:
# {
#   "job_id": "job123",
#   "status": "running",
#   "epoch": 45,
#   "total_epochs": 100,
#   "loss": 0.125,
#   "accuracy": 0.951,
#   "progress": 45
# }
```

---

## Security Considerations

### Production Deployment

**Add authentication:**
```python
# desktop_backend/server.py
from flask_httpauth import HTTPBasicAuth

auth = HTTPBasicAuth()

@auth.verify_password
def verify_password(username, password):
    # Check credentials
    return username == 'admin' and password == 'secure_password'

@app.route('/api/training/start', methods=['POST'])
@auth.login_required
def start_training():
    # Protected endpoint
    ...
```

**Use HTTPS:**
```python
# Run with SSL certificate
socketio.run(
    app,
    host='0.0.0.0',
    port=5001,
    certfile='cert.pem',
    keyfile='key.pem'
)
```

**Firewall rules:**
```bash
# Only allow desktop to RDK on trusted network
# On RDK:
ufw allow from 192.168.1.100 to any port 5000

# On Desktop:
ufw allow from 192.168.1.50 to any port 5001
```

---

## Success Criteria

After implementation:
- [ ] Desktop server runs without errors
- [ ] Desktop UI loads and displays properly
- [ ] RDK camera service connects successfully
- [ ] RDK appears in UI device list
- [ ] Live scanning shows frames in real-time
- [ ] Evaluation results display with confidence
- [ ] Training starts and shows progress
- [ ] Training completes and exports model
- [ ] Model download works
- [ ] Multiple RDKs can connect simultaneously

---

## Next Steps

1. **Install dependencies** (30 min)
2. **Deploy desktop backend** (15 min)
3. **Build web UI** (20 min)
4. **Deploy RDK service** (15 min)
5. **Test connection** (10 min)
6. **Collect training data** (1-2 days)
7. **Train model** (2-4 hours)
8. **Deploy model** (15 min)
9. **Evaluate improvement** (1 day)

**Total time: 1 week to full implementation**


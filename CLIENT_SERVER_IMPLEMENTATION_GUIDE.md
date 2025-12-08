# Client-Server Implementation Guide

## Quick Start: 3 Components to Deploy

### Component 1: RDK Client (`backend/client/rdk_client.py`)
- Connects to desktop server via USB/WiFi/LAN
- Sends captured frames for training
- Downloads trained models
- 180 lines of code

### Component 2: Desktop Server (`desktop_training/server.py`)
- Receives frames from RDK
- Manages training pipeline
- Exports models to ONNX
- 350 lines of code

### Component 3: Desktop Web UI (`desktop_training/app.py` + React)
- Dashboard for monitoring
- Start/stop training
- View dataset progress
- Download models
- 200 lines code + React

---

## Deployment Steps

### Step 1: Setup Desktop Server (Windows/Mac/Linux)

```bash
# Install dependencies
pip install flask flask-cors torch torchvision opencv-python numpy

# Create server
mkdir desktop_training
cd desktop_training
# Copy server.py from guide

# Run server
python server.py
# Server listening on http://0.0.0.0:5001
```

### Step 2: Configure RDK Client

```bash
# On RDK, modify backend/app.py

from client.rdk_client import RDKClient
from api.client_routes import client_bp, init_client

# Initialize client
server_url = "http://192.168.1.100:5001"  # Change to your desktop IP
rdk_client = RDKClient(server_url)
rdk_client.connect()

# Register API endpoints
app.register_blueprint(client_bp)
init_client(rdk_client)
```

### Step 3: Test Connection

```bash
# From RDK, test server connection
curl http://192.168.1.100:5001/api/health

# Response:
# {
#   "status": "healthy",
#   "timestamp": "2025-12-08T12:00:00"
# }
```

### Step 4: Send Test Frame

```bash
# Create test images
python test_frames.py  # Generates dummy RGB + depth

# Send from RDK to server
curl -X POST http://192.168.1.100:5001/api/training/frame \
  -F "rgb=@test_rgb.png" \
  -F "depth=@test_depth.bin" \
  -F "label=good"
```

### Step 5: Start Training Job

```bash
# On server, start training
curl -X POST http://localhost:5001/api/training/start \
  -H "Content-Type: application/json" \
  -d '{
    "name": "first_training",
    "config": {
      "epochs": 10,
      "batch_size": 16,
      "learning_rate": 0.001
    }
  }'

# Response: {"status": "started", "job_id": "abc123xyz..."}
```

### Step 6: Monitor Training

```bash
# Check progress
curl http://localhost:5001/api/training/progress/abc123xyz

# Response:
# {
#   "job_id": "abc123xyz",
#   "status": "training",
#   "epoch": 5,
#   "total_epochs": 10,
#   "loss": 0.3421,
#   "accuracy": 92.5
# }
```

### Step 7: Download Model

```bash
# On RDK, download trained model
curl http://192.168.1.100:5001/api/models/download/abc123xyz \
  -o /opt/weldvision/models/weld_defect_model.onnx

# Model now available for inference on RDK
```

---

## Network Connection Options

### Option 1: WiFi LAN (Easiest)
```
Desktop (Server):      192.168.1.100:5001
RDK (Client):          192.168.1.50
Connection: WiFi router
Speed: 100+ Mbps
```

**RDK Client Config:**
```python
server_url = "http://192.168.1.100:5001"
```

### Option 2: USB-over-IP
```
Desktop (Server):      192.168.137.254:5001
RDK (Client):          192.168.137.1 (USB Ethernet)
Connection: USB cable
Speed: 10 Mbps (adequate)
```

**RDK Client Config:**
```python
server_url = "http://192.168.137.254:5001"
```

### Option 3: USB ADB Tunnel
```
Desktop (Server):      localhost:5001
RDK (Client):          via adb forward
```

**Setup:**
```bash
adb forward tcp:5001 tcp:5001
```

**RDK Client Config:**
```python
server_url = "http://localhost:5001"
```

---

## Real-World Workflow

### Day 1: Collect Data

1. **Start server on desktop:**
   ```bash
   python desktop_training/server.py
   ```

2. **Connect RDK to server:**
   ```bash
   curl -X POST http://192.168.1.100:5000/api/client/connect \
     -d '{"server_url": "http://192.168.1.100:5001"}'
   ```

3. **During live scanning, collect frames:**
   - Live scan normal welds → click "Collect: Good"
   - Identify porosity → click "Collect: Porosity"
   - Find spatter → click "Collect: Spatter"
   - Repeat 100-500 times

4. **Monitor progress on desktop UI:**
   - Open http://localhost:5002
   - See dataset growing in real-time
   - Dataset shows: good=150, porosity=120, spatter=110, gap=95, undercut=40

### Day 2-3: Training

1. **Check collected data:**
   ```bash
   curl http://localhost:5001/api/training/status
   ```

2. **Start training:**
   ```bash
   curl -X POST http://localhost:5001/api/training/start \
     -d '{"config": {"epochs": 100}}'
   ```

3. **Monitor training (takes 2-4 hours):**
   - Open desktop UI
   - Watch progress: Epoch 1/100 → Loss: 0.45, Acc: 89%
   - Epoch 50/100 → Loss: 0.12, Acc: 95%
   - Epoch 100/100 → Loss: 0.08, Acc: 96%

4. **Training completes:**
   - Model automatically exported to ONNX
   - Available for download
   - Size: ~120MB

### Day 4: Deploy

1. **Download model to RDK:**
   ```bash
   curl http://192.168.1.100:5001/api/models/download/model_id \
     -o /opt/weldvision/models/model.onnx
   ```

2. **Restart RDK service:**
   ```bash
   systemctl restart weldvision-backend
   ```

3. **Verify inference working:**
   - Live scan with ML model
   - See improved accuracy
   - Monitor CPU usage (should be 15%)

---

## Code Integration Checklist

### RDK Backend (`backend/app.py`)
- [ ] Import RDKClient
- [ ] Initialize client with server URL
- [ ] Register client_bp blueprint
- [ ] Call init_client(rdk_client)

### RDK Frontend (`App.tsx`)
- [ ] Add ServerConnectionManager component
- [ ] Add TrainingDataCollector component
- [ ] Add ModelSelector component
- [ ] Add TrainingProgressMonitor component

### Desktop Server (`desktop_training/server.py`)
- [ ] Create server.py from guide
- [ ] Create data/ directory
- [ ] Create models/ directory
- [ ] Test /api/health endpoint

### Desktop Web UI (`desktop_training/templates/`)
- [ ] Create HTML template
- [ ] Create React dashboard component
- [ ] Create chart for training progress
- [ ] Create dataset statistics view

---

## Performance Expectations

### Data Upload
- RGB (640x480 PNG): 100-150 KB
- Depth (480x640 binary): 1.2 MB
- Together: ~1.3 MB per frame
- WiFi: 2-3 seconds per frame
- USB: 1-2 seconds per frame

### Training
- Small dataset (100 images): 10-15 minutes
- Medium dataset (500 images): 1-2 hours
- Large dataset (1000 images): 3-4 hours
- GPU (NVIDIA): 3-10x faster
- CPU only: Still acceptable

### Model Size
- ONNX format: 100-150 MB
- Download time: 2-5 minutes (USB/WiFi)
- RDK storage: ~200MB needed
- Inference time: 25-30ms per frame

---

## Troubleshooting

### Connection Failed
```
Error: Failed to connect to server
Solution:
1. Check server running: ps aux | grep server.py
2. Check firewall: Allow port 5001
3. Check IP address: ping 192.168.1.100
4. Check URL format: http://IP:5001 (not https)
```

### Frame Upload Fails
```
Error: Connection reset by peer
Solution:
1. Check frame size < 50 MB
2. Check network connection stable
3. Try smaller frames (resize if needed)
4. Check server disk space: du -h data/
```

### Training Crashes
```
Error: CUDA out of memory
Solution:
1. Reduce batch_size (32 → 16)
2. Use CPU instead: device = 'cpu'
3. Add more RAM (16GB minimum)
```

### Model Download Slow
```
Problem: Download taking > 10 minutes
Solution:
1. Check connection speed: iperf3
2. Use compression: gzip model before send
3. Split model into chunks
```

---

## Advanced Features

### Auto-Retry Connection
```python
for attempt in range(5):
    if client.connect():
        break
    time.sleep(5)
```

### Batch Frame Upload
```python
frames = [...]
for frame in frames:
    client.send_frame_for_training(...)
    time.sleep(1)  # Rate limit
```

### Model Versioning
```python
models = [
  'model_v1_epoch100',
  'model_v2_epoch100',
  'model_v3_epoch100'
]

# Download specific version
client.download_model('model_v3_epoch100')
```

### Multiple Clients
```
Multiple RDK devices sending to single desktop server
Desktop server merges all data before training
```

---

## Files to Create

```
backend/
├── client/
│   ├── __init__.py
│   └── rdk_client.py (180 lines - from guide)
└── api/
    └── client_routes.py (150 lines - from guide)

desktop_training/
├── server.py (350 lines - from guide)
├── app.py (100 lines - from guide)
├── requirements.txt
├── templates/
│   └── index.html
├── static/
│   └── dashboard.js
└── data/
    ├── frames/ (auto-created)
    └── models/
```

---

## Success Metrics

After implementation:
- ✅ RDK connects to desktop server (verify with curl)
- ✅ Frames upload successfully (check server logs)
- ✅ Training starts without errors (check job status)
- ✅ Models download and load on RDK (verify ONNX inference)
- ✅ Accuracy > 90% (evaluate on test set)

---

## Next Steps

1. **This week:** Deploy components 1-2 (client + server)
2. **Test connection** with sample frames
3. **Collect data** during normal operation
4. **Start training** when 100+ frames collected
5. **Deploy model** when training completes
6. **Monitor performance** in production

Ready to implement? Start with: RDK_CLIENT_DESKTOP_SERVER_ARCHITECTURE.md


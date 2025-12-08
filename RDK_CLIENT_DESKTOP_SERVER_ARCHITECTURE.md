# RDK Client - Desktop Server Architecture for Model Training

## Architecture Overview

```
RDK X5 (Client)                        Desktop Workstation (Server)
─────────────────                      ─────────────────────────────
├─ Live Scanning                       ├─ Web Application (Flask)
├─ Capture RGB + Depth                 ├─ Training Pipeline
├─ Send to Server                      ├─ Model Management
│  (USB/WiFi/LAN)                      ├─ Real-time Training
└─ Receive trained model               ├─ Web UI (React/Vue)
   for inference                       └─ Database (SQLite)
                                           
        ↔ REST API (HTTP/WebSocket)
         TCP/IP over USB or Network
```

---

## Part 1: RDK Client Implementation

### 1.1 RDK Client Service (`backend/client/rdk_client.py`)

```python
"""
RDK Client Service
Connects to desktop server for remote training and model management
"""

import requests
import json
import logging
import time
import cv2
import numpy as np
from pathlib import Path
from typing import Dict, Optional, Tuple
from datetime import datetime
import threading
import queue
import hashlib

logger = logging.getLogger(__name__)

class RDKClient:
    """
    RDK X5 Client for connecting to desktop training server
    
    Features:
    - Connect to server via USB/WiFi/LAN
    - Stream frames for training
    - Receive trained models
    - Run inference with received models
    - Real-time progress updates
    """
    
    def __init__(self, server_url: str = "http://localhost:5001"):
        """
        Initialize RDK client
        
        Args:
            server_url: Desktop server address
                Examples:
                - Local: http://localhost:5001
                - LAN: http://192.168.1.100:5001
                - USB: http://192.168.137.1:5001
        """
        self.server_url = server_url.rstrip('/')
        self.connected = False
        self.session = requests.Session()
        self.frame_queue = queue.Queue(maxsize=50)
        self.model_path = Path('/opt/weldvision/models')
        self.model_path.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"RDK Client initialized - Server: {self.server_url}")
    
    def connect(self, max_retries: int = 5) -> bool:
        """
        Connect to desktop server
        
        Returns:
            True if connected, False otherwise
        """
        for attempt in range(max_retries):
            try:
                response = self.session.get(
                    f"{self.server_url}/api/health",
                    timeout=5
                )
                
                if response.status_code == 200:
                    self.connected = True
                    logger.info(f"✓ Connected to server: {self.server_url}")
                    return True
                    
            except requests.ConnectionError:
                logger.warning(f"Connection attempt {attempt + 1}/{max_retries} failed")
                time.sleep(2)
            except Exception as e:
                logger.error(f"Connection error: {e}")
        
        self.connected = False
        logger.error(f"Failed to connect to server after {max_retries} attempts")
        return False
    
    def send_frame_for_training(self, rgb: np.ndarray, depth: np.ndarray, 
                               label: str, metadata: Dict = None) -> Dict:
        """
        Send frame to server for training
        
        Args:
            rgb: RGB image (H, W, 3) BGR format
            depth: Depth map (H, W)
            label: Defect type (good/porosity/spatter/gap/undercut)
            metadata: Additional metadata
        
        Returns:
            Server response with status
        """
        try:
            if not self.connected:
                logger.error("Not connected to server")
                return {'error': 'Not connected'}
            
            # Encode images
            _, rgb_encoded = cv2.imencode('.png', rgb)
            depth_encoded = depth.tobytes()
            
            # Prepare files
            files = {
                'rgb': ('frame.png', rgb_encoded.tobytes(), 'image/png'),
                'depth': ('depth.bin', depth_encoded, 'application/octet-stream')
            }
            
            # Prepare data
            data = {
                'label': label,
                'timestamp': datetime.now().isoformat(),
                'resolution': f"{rgb.shape[1]}x{rgb.shape[0]}",
            }
            
            if metadata:
                data['metadata'] = json.dumps(metadata)
            
            # Send
            response = self.session.post(
                f"{self.server_url}/api/training/frame",
                files=files,
                data=data,
                timeout=10
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                logger.error(f"Server error: {response.status_code}")
                return {'error': response.text}
                
        except Exception as e:
            logger.error(f"Send frame error: {e}")
            return {'error': str(e)}
    
    def get_training_status(self) -> Dict:
        """Get current training status from server"""
        try:
            response = self.session.get(
                f"{self.server_url}/api/training/status",
                timeout=5
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                return {'error': 'Failed to get status'}
                
        except Exception as e:
            logger.error(f"Status error: {e}")
            return {'error': str(e)}
    
    def start_training_job(self, name: str, config: Dict) -> Dict:
        """
        Start training job on server
        
        Args:
            name: Training job name
            config: Training configuration
                {
                    'epochs': 100,
                    'batch_size': 32,
                    'learning_rate': 0.001,
                    'model_type': 'resnet50'
                }
        
        Returns:
            Server response with job ID
        """
        try:
            payload = {
                'name': name,
                'config': config
            }
            
            response = self.session.post(
                f"{self.server_url}/api/training/start",
                json=payload,
                timeout=10
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                return {'error': response.text}
                
        except Exception as e:
            logger.error(f"Training start error: {e}")
            return {'error': str(e)}
    
    def download_model(self, model_id: str, save_path: Path = None) -> Optional[Path]:
        """
        Download trained model from server
        
        Args:
            model_id: Model identifier
            save_path: Where to save model
        
        Returns:
            Path to downloaded model
        """
        try:
            if save_path is None:
                save_path = self.model_path / f"{model_id}.onnx"
            
            response = self.session.get(
                f"{self.server_url}/api/models/download/{model_id}",
                timeout=30,
                stream=True
            )
            
            if response.status_code == 200:
                with open(save_path, 'wb') as f:
                    for chunk in response.iter_content(chunk_size=8192):
                        f.write(chunk)
                
                logger.info(f"✓ Model downloaded: {save_path}")
                return save_path
            else:
                logger.error(f"Download failed: {response.status_code}")
                return None
                
        except Exception as e:
            logger.error(f"Download error: {e}")
            return None
    
    def list_models(self) -> Dict:
        """Get list of available models from server"""
        try:
            response = self.session.get(
                f"{self.server_url}/api/models/list",
                timeout=5
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                return {'error': 'Failed to list models'}
                
        except Exception as e:
            logger.error(f"List models error: {e}")
            return {'error': str(e)}
    
    def get_training_progress(self, job_id: str) -> Dict:
        """Get real-time training progress"""
        try:
            response = self.session.get(
                f"{self.server_url}/api/training/progress/{job_id}",
                timeout=5
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                return {'error': 'Job not found'}
                
        except Exception as e:
            logger.error(f"Progress error: {e}")
            return {'error': str(e)}
```

### 1.2 RDK API Endpoint (`backend/api/client_routes.py`)

```python
"""
RDK Client API Routes
Provides endpoints for training data collection
"""

from flask import Blueprint, request, jsonify
from pathlib import Path
import logging
import json
import numpy as np
import cv2
from io import BytesIO

logger = logging.getLogger(__name__)

client_bp = Blueprint('client', __name__, url_prefix='/api/client')

# Global client instance
rdk_client = None

def init_client(client_instance):
    """Initialize global client"""
    global rdk_client
    rdk_client = client_instance

@client_bp.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return {
        'status': 'healthy',
        'connected': rdk_client.connected if rdk_client else False,
        'server': rdk_client.server_url if rdk_client else None
    }, 200

@client_bp.route('/connect', methods=['POST'])
def connect_to_server():
    """Connect to training server"""
    try:
        data = request.json
        server_url = data.get('server_url', 'http://localhost:5001')
        
        # Reinitialize with new server
        global rdk_client
        rdk_client = RDKClient(server_url)
        
        if rdk_client.connect():
            return {
                'status': 'connected',
                'server': server_url
            }, 200
        else:
            return {
                'error': 'Connection failed',
                'server': server_url
            }, 500
            
    except Exception as e:
        logger.error(f"Connection error: {e}")
        return {'error': str(e)}, 500

@client_bp.route('/send-frame', methods=['POST'])
def send_training_frame():
    """
    Send frame for training
    
    Form data:
    - rgb: PNG image
    - depth: Binary depth data
    - label: Defect type
    """
    try:
        if not rdk_client or not rdk_client.connected:
            return {'error': 'Client not connected'}, 400
        
        # Get files
        rgb_file = request.files.get('rgb')
        depth_file = request.files.get('depth')
        
        if not rgb_file or not depth_file:
            return {'error': 'Missing files'}, 400
        
        # Decode RGB
        nparr = np.frombuffer(rgb_file.read(), np.uint8)
        rgb = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        # Decode depth
        depth = np.frombuffer(depth_file.read(), np.float32)
        depth = depth.reshape((480, 640))  # Adjust to your resolution
        
        label = request.form.get('label', 'good')
        
        # Send to server
        result = rdk_client.send_frame_for_training(rgb, depth, label)
        
        return result, 200
        
    except Exception as e:
        logger.error(f"Frame send error: {e}")
        return {'error': str(e)}, 500

@client_bp.route('/training/status', methods=['GET'])
def get_training_status():
    """Get training status from server"""
    try:
        if not rdk_client or not rdk_client.connected:
            return {'error': 'Client not connected'}, 400
        
        status = rdk_client.get_training_status()
        return status, 200
        
    except Exception as e:
        return {'error': str(e)}, 500

@client_bp.route('/training/start', methods=['POST'])
def start_training():
    """Start training job on server"""
    try:
        if not rdk_client or not rdk_client.connected:
            return {'error': 'Client not connected'}, 400
        
        data = request.json
        name = data.get('name', 'training_job')
        config = data.get('config', {})
        
        result = rdk_client.start_training_job(name, config)
        return result, 200
        
    except Exception as e:
        return {'error': str(e)}, 500

@client_bp.route('/models/download/<model_id>', methods=['GET'])
def download_model(model_id: str):
    """Download trained model from server"""
    try:
        if not rdk_client or not rdk_client.connected:
            return {'error': 'Client not connected'}, 400
        
        model_path = rdk_client.download_model(model_id)
        
        if model_path:
            return {
                'status': 'downloaded',
                'model_id': model_id,
                'path': str(model_path)
            }, 200
        else:
            return {'error': 'Download failed'}, 500
            
    except Exception as e:
        return {'error': str(e)}, 500

@client_bp.route('/models/list', methods=['GET'])
def list_models():
    """List available models"""
    try:
        if not rdk_client or not rdk_client.connected:
            return {'error': 'Client not connected'}, 400
        
        models = rdk_client.list_models()
        return models, 200
        
    except Exception as e:
        return {'error': str(e)}, 500
```

---

## Part 2: Desktop Server Implementation

### 2.1 Desktop Server (`desktop_training/server.py`)

```python
"""
Desktop Training Server
Accepts frames from RDK client, trains models, serves predictions
"""

from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
from pathlib import Path
import logging
import json
import numpy as np
import cv2
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import threading
import queue
from datetime import datetime
import sqlite3
from typing import Dict, List, Optional
import uuid

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

# Configuration
DATA_DIR = Path('data/frames')
MODELS_DIR = Path('models/trained')
DB_PATH = 'training_server.db'

DATA_DIR.mkdir(parents=True, exist_ok=True)
MODELS_DIR.mkdir(parents=True, exist_ok=True)

# Training queue
training_queue = queue.Queue()
training_jobs = {}  # Store job status

class RemoteTrainingDataset(Dataset):
    """Dataset for frames uploaded from RDK"""
    
    def __init__(self, data_dir: Path, transform=None):
        self.data_dir = Path(data_dir)
        self.transform = transform
        self.samples = []
        self.class_to_idx = {
            'good': 0,
            'porosity': 1,
            'spatter': 2,
            'gap': 3,
            'undercut': 4
        }
        
        # Load samples from directory
        for label_dir in self.data_dir.iterdir():
            if label_dir.is_dir() and label_dir.name in self.class_to_idx:
                for img_path in label_dir.glob('*_rgb.png'):
                    depth_path = str(img_path).replace('_rgb.png', '_depth.npy')
                    if Path(depth_path).exists():
                        self.samples.append((img_path, depth_path, label_dir.name))
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        rgb_path, depth_path, label = self.samples[idx]
        
        rgb = cv2.imread(str(rgb_path))
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        
        depth = np.load(depth_path).astype(np.float32)
        depth = (depth - depth.min()) / (depth.max() - depth.min() + 1e-5)
        
        if self.transform:
            rgb = self.transform(rgb)
        else:
            rgb = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
        
        depth = torch.from_numpy(depth).unsqueeze(0)
        label_idx = self.class_to_idx[label]
        
        return rgb, depth, label_idx

class WeldDefectModel(nn.Module):
    """ML model for weld defect detection"""
    
    def __init__(self, num_classes=5):
        super().__init__()
        # RGB branch
        self.rgb_backbone = torch.hub.load('pytorch/vision:v0.10.0', 'resnet50', pretrained=True)
        self.rgb_backbone.fc = nn.Identity()
        
        # Depth branch
        self.depth_conv = nn.Sequential(
            nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        )
        
        # Fusion
        self.fusion = nn.Sequential(
            nn.Linear(2048 + 64, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, num_classes)
        )
    
    def forward(self, rgb, depth):
        rgb_features = self.rgb_backbone(rgb)
        
        depth_resized = torch.nn.functional.interpolate(
            depth, size=(224, 224), mode='bilinear'
        )
        depth_features = self.depth_conv(depth_resized)
        depth_features = torch.nn.functional.adaptive_avg_pool2d(depth_features, 1)
        depth_features = depth_features.view(depth_features.size(0), -1)
        
        combined = torch.cat([rgb_features, depth_features], dim=1)
        out = self.fusion(combined)
        return out

def train_model(job_id: str, config: Dict):
    """
    Training worker thread
    Trains model and updates job status
    """
    try:
        job_status = {
            'job_id': job_id,
            'status': 'training',
            'epoch': 0,
            'total_epochs': config.get('epochs', 100),
            'loss': 0,
            'accuracy': 0,
            'start_time': datetime.now().isoformat(),
            'end_time': None
        }
        training_jobs[job_id] = job_status
        
        # Load dataset
        dataset = RemoteTrainingDataset(DATA_DIR)
        
        if len(dataset) < 10:
            job_status['status'] = 'error'
            job_status['error'] = 'Insufficient data'
            return
        
        loader = DataLoader(
            dataset,
            batch_size=config.get('batch_size', 32),
            shuffle=True
        )
        
        # Create model
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        model = WeldDefectModel(num_classes=5).to(device)
        criterion = nn.CrossEntropyLoss()
        optimizer = torch.optim.Adam(
            model.parameters(),
            lr=config.get('learning_rate', 0.001)
        )
        
        # Training loop
        for epoch in range(config.get('epochs', 100)):
            model.train()
            total_loss = 0
            correct = 0
            total = 0
            
            for rgb, depth, labels in loader:
                rgb = rgb.to(device)
                depth = depth.to(device)
                labels = labels.to(device)
                
                optimizer.zero_grad()
                outputs = model(rgb, depth)
                loss = criterion(outputs, labels)
                
                loss.backward()
                optimizer.step()
                
                total_loss += loss.item()
                _, predicted = outputs.max(1)
                total += labels.size(0)
                correct += predicted.eq(labels).sum().item()
            
            accuracy = 100.0 * correct / total
            avg_loss = total_loss / len(loader)
            
            # Update status
            job_status['epoch'] = epoch + 1
            job_status['loss'] = round(avg_loss, 4)
            job_status['accuracy'] = round(accuracy, 2)
            
            logger.info(f"Job {job_id} - Epoch {epoch+1}/{config.get('epochs')} - "
                       f"Loss: {avg_loss:.4f}, Acc: {accuracy:.2f}%")
        
        # Save model
        model_path = MODELS_DIR / f"{job_id}.pt"
        torch.save(model.state_dict(), model_path)
        
        # Export to ONNX
        onnx_path = MODELS_DIR / f"{job_id}.onnx"
        dummy_rgb = torch.randn(1, 3, 224, 224).to(device)
        dummy_depth = torch.randn(1, 1, 224, 224).to(device)
        
        torch.onnx.export(
            model,
            (dummy_rgb, dummy_depth),
            str(onnx_path),
            input_names=['rgb', 'depth'],
            output_names=['logits'],
            opset_version=12
        )
        
        job_status['status'] = 'completed'
        job_status['end_time'] = datetime.now().isoformat()
        job_status['model_path'] = str(onnx_path)
        
        logger.info(f"✓ Training completed: {job_id}")
        
    except Exception as e:
        job_status['status'] = 'error'
        job_status['error'] = str(e)
        logger.error(f"Training error: {e}")

# API Endpoints

@app.route('/api/health', methods=['GET'])
def health():
    """Server health check"""
    return {
        'status': 'healthy',
        'timestamp': datetime.now().isoformat(),
        'training_jobs': len(training_jobs)
    }, 200

@app.route('/api/training/frame', methods=['POST'])
def upload_frame():
    """
    Receive training frame from RDK client
    
    Form data:
    - rgb: PNG image
    - depth: Binary depth data
    - label: Defect type
    """
    try:
        rgb_file = request.files.get('rgb')
        depth_file = request.files.get('depth')
        label = request.form.get('label', 'good')
        
        if not rgb_file or not depth_file:
            return {'error': 'Missing files'}, 400
        
        # Create label directory
        label_dir = DATA_DIR / label
        label_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate frame ID
        frame_id = str(uuid.uuid4())[:8]
        
        # Save RGB
        rgb_path = label_dir / f"{frame_id}_rgb.png"
        rgb_file.save(str(rgb_path))
        
        # Save depth
        depth_path = label_dir / f"{frame_id}_depth.npy"
        depth_data = np.frombuffer(depth_file.read(), np.float32)
        np.save(depth_path, depth_data)
        
        # Count dataset
        stats = {}
        for ld in DATA_DIR.iterdir():
            if ld.is_dir():
                count = len(list(ld.glob('*_rgb.png')))
                stats[ld.name] = count
        
        logger.info(f"Frame received: {frame_id} ({label})")
        
        return {
            'status': 'received',
            'frame_id': frame_id,
            'label': label,
            'dataset_stats': stats
        }, 200
        
    except Exception as e:
        logger.error(f"Frame upload error: {e}")
        return {'error': str(e)}, 500

@app.route('/api/training/start', methods=['POST'])
def start_training():
    """Start training job"""
    try:
        data = request.json
        job_id = str(uuid.uuid4())[:12]
        config = data.get('config', {})
        
        # Start training in background thread
        thread = threading.Thread(
            target=train_model,
            args=(job_id, config)
        )
        thread.daemon = True
        thread.start()
        
        return {
            'status': 'started',
            'job_id': job_id,
            'config': config
        }, 200
        
    except Exception as e:
        return {'error': str(e)}, 500

@app.route('/api/training/status', methods=['GET'])
def training_status():
    """Get overall training status"""
    try:
        dataset_stats = {}
        for label_dir in DATA_DIR.iterdir():
            if label_dir.is_dir():
                count = len(list(label_dir.glob('*_rgb.png')))
                dataset_stats[label_dir.name] = count
        
        return {
            'dataset_stats': dataset_stats,
            'total_frames': sum(dataset_stats.values()),
            'training_jobs': {jid: job_status 
                            for jid, job_status in training_jobs.items()},
            'timestamp': datetime.now().isoformat()
        }, 200
        
    except Exception as e:
        return {'error': str(e)}, 500

@app.route('/api/training/progress/<job_id>', methods=['GET'])
def training_progress(job_id: str):
    """Get specific job progress"""
    if job_id in training_jobs:
        return training_jobs[job_id], 200
    else:
        return {'error': 'Job not found'}, 404

@app.route('/api/models/list', methods=['GET'])
def list_models():
    """List available trained models"""
    try:
        models = []
        for model_file in MODELS_DIR.glob('*.onnx'):
            stat = model_file.stat()
            models.append({
                'model_id': model_file.stem,
                'size_mb': stat.st_size / (1024**2),
                'created': datetime.fromtimestamp(stat.st_ctime).isoformat()
            })
        
        return {
            'models': models,
            'count': len(models)
        }, 200
        
    except Exception as e:
        return {'error': str(e)}, 500

@app.route('/api/models/download/<model_id>', methods=['GET'])
def download_model(model_id: str):
    """Download trained model"""
    try:
        model_path = MODELS_DIR / f"{model_id}.onnx"
        
        if not model_path.exists():
            return {'error': 'Model not found'}, 404
        
        return send_file(str(model_path), as_attachment=True)
        
    except Exception as e:
        return {'error': str(e)}, 500

@app.route('/api/datasets/clear', methods=['POST'])
def clear_dataset():
    """Clear all training data"""
    try:
        import shutil
        shutil.rmtree(DATA_DIR)
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        
        return {'status': 'cleared'}, 200
        
    except Exception as e:
        return {'error': str(e)}, 500

if __name__ == '__main__':
    logger.info("Starting Desktop Training Server...")
    logger.info(f"Data directory: {DATA_DIR}")
    logger.info(f"Models directory: {MODELS_DIR}")
    
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
```

---

## Part 3: Desktop UI Application

### 3.1 Desktop Web UI (`desktop_training/app.py`)

```python
"""
Desktop Web Application
Train models and monitor RDK data collection
"""

from flask import Flask, render_template, jsonify, request
import os
from pathlib import Path

app = Flask(__name__, template_folder='templates', static_folder='static')

DATA_DIR = Path('data/frames')
MODELS_DIR = Path('models/trained')

@app.route('/')
def index():
    """Main dashboard"""
    return render_template('index.html')

@app.route('/api/ui/stats', methods=['GET'])
def get_stats():
    """Get dashboard statistics"""
    dataset_stats = {}
    for label_dir in DATA_DIR.iterdir():
        if label_dir.is_dir():
            count = len(list(label_dir.glob('*_rgb.png')))
            dataset_stats[label_dir.name] = count
    
    models = []
    for model_file in MODELS_DIR.glob('*.onnx'):
        models.append({'name': model_file.stem})
    
    return {
        'dataset': dataset_stats,
        'models': models,
        'total_frames': sum(dataset_stats.values())
    }, 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002, debug=True)
```

### 3.2 Desktop React UI (`desktop_training/templates/Dashboard.tsx`)

```typescript
import React, { useState, useEffect } from 'react';
import { Activity, Download, Upload, Play, BarChart3, Wifi } from 'lucide-react';

interface Stats {
  dataset: Record<string, number>;
  models: Array<{name: string}>;
  total_frames: number;
}

interface TrainingJob {
  job_id: string;
  status: string;
  epoch: number;
  total_epochs: number;
  loss: number;
  accuracy: number;
}

const Dashboard: React.FC = () => {
  const [stats, setStats] = useState<Stats | null>(null);
  const [serverUrl, setServerUrl] = useState('http://localhost:5001');
  const [connected, setConnected] = useState(false);
  const [trainingJobs, setTrainingJobs] = useState<TrainingJob[]>([]);
  const [selectedJob, setSelectedJob] = useState<string | null>(null);

  // Check connection
  useEffect(() => {
    const checkConnection = async () => {
      try {
        const response = await fetch(`${serverUrl}/api/health`);
        setConnected(response.ok);
      } catch {
        setConnected(false);
      }
    };

    checkConnection();
    const interval = setInterval(checkConnection, 5000);
    return () => clearInterval(interval);
  }, [serverUrl]);

  // Load statistics
  useEffect(() => {
    const loadStats = async () => {
      try {
        const response = await fetch(`${serverUrl}/api/training/status`);
        if (response.ok) {
          const data = await response.json();
          setStats(data);
        }
      } catch (error) {
        console.error('Failed to load stats:', error);
      }
    };

    if (connected) {
      loadStats();
      const interval = setInterval(loadStats, 5000);
      return () => clearInterval(interval);
    }
  }, [connected, serverUrl]);

  const startTraining = async () => {
    try {
      const response = await fetch(`${serverUrl}/api/training/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: 'auto_training',
          config: {
            epochs: 100,
            batch_size: 32,
            learning_rate: 0.001
          }
        })
      });

      const data = await response.json();
      if (data.job_id) {
        setSelectedJob(data.job_id);
      }
    } catch (error) {
      alert('Failed to start training');
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 to-slate-800 p-6">
      <div className="max-w-6xl mx-auto">
        {/* Header */}
        <div className="flex justify-between items-center mb-8">
          <h1 className="text-4xl font-bold text-white flex items-center">
            <BarChart3 className="w-10 h-10 mr-3 text-industrial-blue" />
            Weld Defect Training Server
          </h1>
          <div className="flex items-center space-x-2">
            <Wifi className={`w-5 h-5 ${connected ? 'text-green-500' : 'text-red-500'}`} />
            <span className={connected ? 'text-green-500' : 'text-red-500'}>
              {connected ? 'Connected' : 'Disconnected'}
            </span>
          </div>
        </div>

        {/* Server Config */}
        <div className="bg-slate-800 rounded-lg p-4 mb-6 border border-slate-700">
          <label className="text-white text-sm font-medium block mb-2">Server URL</label>
          <input
            type="text"
            value={serverUrl}
            onChange={(e) => setServerUrl(e.target.value)}
            className="w-full bg-slate-700 text-white px-3 py-2 rounded"
            placeholder="http://localhost:5001"
          />
        </div>

        {/* Stats Grid */}
        {stats && (
          <div className="grid grid-cols-1 md:grid-cols-2 gap-6 mb-6">
            {/* Dataset */}
            <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
              <h2 className="text-white text-lg font-semibold mb-4 flex items-center">
                <Upload className="w-5 h-5 mr-2" />
                Dataset Progress
              </h2>
              <div className="space-y-2">
                <div className="text-white">
                  Total Frames: <span className="font-bold">{stats.total_frames}</span>
                </div>
                {Object.entries(stats.dataset).map(([type, count]) => (
                  <div key={type} className="text-slate-300">
                    {type}: <span className="font-semibold">{count}</span>
                  </div>
                ))}
              </div>
            </div>

            {/* Models */}
            <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
              <h2 className="text-white text-lg font-semibold mb-4 flex items-center">
                <Download className="w-5 h-5 mr-2" />
                Trained Models
              </h2>
              <div className="space-y-2">
                {stats.models.length > 0 ? (
                  stats.models.map(model => (
                    <div key={model.name} className="text-slate-300">
                      {model.name}
                    </div>
                  ))
                ) : (
                  <div className="text-slate-500">No models yet</div>
                )}
              </div>
            </div>
          </div>
        )}

        {/* Training Controls */}
        <button
          onClick={startTraining}
          disabled={!connected}
          className="w-full bg-industrial-blue hover:bg-blue-700 text-white font-bold py-3 rounded mb-6 flex items-center justify-center disabled:opacity-50"
        >
          <Play className="w-5 h-5 mr-2" />
          Start Training
        </button>

        {/* Active Jobs */}
        {trainingJobs.length > 0 && (
          <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
            <h2 className="text-white text-lg font-semibold mb-4 flex items-center">
              <Activity className="w-5 h-5 mr-2" />
              Training Jobs
            </h2>
            {trainingJobs.map(job => (
              <div key={job.job_id} className="bg-slate-700 rounded p-4 mb-3">
                <div className="flex justify-between items-center">
                  <div>
                    <div className="text-white font-semibold">{job.job_id}</div>
                    <div className="text-slate-400 text-sm">
                      Epoch {job.epoch}/{job.total_epochs}
                    </div>
                  </div>
                  <div className="text-right">
                    <div className="text-white">Accuracy: {job.accuracy}%</div>
                    <div className="text-slate-400 text-sm">Loss: {job.loss}</div>
                  </div>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default Dashboard;
```

---

## Part 4: Integration Guide

### 4.1 Initialize RDK Client in Flask (`backend/app.py`)

```python
from client.rdk_client import RDKClient
from api.client_routes import client_bp, init_client

# Initialize client
rdk_client = RDKClient(server_url="http://192.168.1.100:5001")
rdk_client.connect()

# Register blueprint
app.register_blueprint(client_bp)
init_client(rdk_client)
```

### 4.2 Network Configuration

```
RDK to Desktop connections:

USB-over-IP:
  - RDK: 192.168.137.1
  - Desktop: 192.168.137.254
  - Server URL: http://192.168.137.254:5001

WiFi LAN:
  - Router assigns IPs
  - Server URL: http://192.168.1.100:5001

USB-Serial Tunnel:
  - adb forward tcp:5001 tcp:5001
  - Server URL: http://localhost:5001
```

### 4.3 Data Flow

```
RDK (Client)
├─ Live Scan → Capture frame
├─ Send frame (API call)
│  ├─ RGB (PNG)
│  ├─ Depth (binary)
│  └─ Label (type)
└─ Receive trained model

Desktop (Server)
├─ Receive frame
├─ Save to disk
├─ Update dataset stats
├─ Train model (background)
├─ Export to ONNX
└─ Serve via REST API
```

---

## Part 5: Usage Flow

### Step 1: Start Server

```bash
# Desktop terminal
cd desktop_training
python server.py
# Listening on http://0.0.0.0:5001
```

### Step 2: Connect RDK Client

```bash
# RDK terminal
curl -X POST http://localhost:5000/api/client/connect \
  -H "Content-Type: application/json" \
  -d '{"server_url": "http://192.168.1.100:5001"}'
```

### Step 3: Collect Data

```bash
# RDK: Collect frame during live scan
curl -X POST http://localhost:5000/api/client/send-frame \
  -F "rgb=@frame.png" \
  -F "depth=@frame.bin" \
  -F "label=porosity"
```

### Step 4: Start Training

```bash
# Desktop: Start training job
curl -X POST http://localhost:5001/api/training/start \
  -H "Content-Type: application/json" \
  -d '{
    "name": "defect_v1",
    "config": {
      "epochs": 100,
      "batch_size": 32
    }
  }'
```

### Step 5: Download Model

```bash
# RDK: Download trained model
curl http://localhost:5001/api/models/download/model_id \
  -o weld_defect_model.onnx

# Deploy to inference
cp weld_defect_model.onnx /opt/weldvision/models/
```

---

## Architecture Benefits

✅ **Separation of Concerns** - Data collection and training separated
✅ **Real-time Training** - See results as data collected
✅ **Model Serving** - Desktop hosts trained models
✅ **Flexible Deployment** - USB/WiFi/LAN support
✅ **Scalable** - Multiple RDK clients to one server
✅ **Live Feedback** - Training progress visible in UI
✅ **Easy Updates** - Push new models without code changes

---

## Next: Frontend Components for RDK UI

Ready to create React components for:
- Server connection manager
- Training data uploader
- Real-time status monitor
- Model selector
- Live training progress display


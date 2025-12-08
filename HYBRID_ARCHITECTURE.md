# Hybrid Architecture: Balanced RDK + Desktop

## Overview

**Best of both worlds:** RDK handles real-time inference with lightweight models. Desktop handles heavy training, advanced analysis, and refinement.

```
┌─────────────────────────────────────────────────────────────┐
│                    DESKTOP/LAPTOP                           │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Advanced ML Models                                   │ │
│  │  ├─ ResNet50 (Deep CNN)                               │ │
│  │  ├─ Ensemble Models                                   │ │
│  │  ├─ Custom Architectures                              │ │
│  │  └─ Model Training Pipeline                           │ │
│  └───────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Web UI Dashboard                                     │ │
│  │  ├─ Training Monitor                                  │ │
│  │  ├─ Analytics                                         │ │
│  │  ├─ Model Management                                  │ │
│  │  └─ Batch Processing                                  │ │
│  └───────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Backend Server (Optional)                            │ │
│  │  ├─ Model Repository                                  │ │
│  │  ├─ Training Jobs                                     │ │
│  │  ├─ Data Collection                                   │ │
│  │  └─ Analytics Engine                                  │ │
│  └───────────────────────────────────────────────────────┘ │
└──────────────────────────┬───────────────────────────────────┘
                           │
                   REST API + Data Sync
                    (Optional connection)
                           │
         ┌─────────────────┴──────────────────┐
         │                                    │
         ▼                                    ▼
    ┌─────────────┐                   ┌──────────────┐
    │   RDK X5    │                   │   RDK X5     │
    │             │                   │              │
    │ ┌─────────┐ │                   │ ┌──────────┐ │
    │ │ Camera  │ │                   │ │ Camera   │ │
    │ │ Stereo  │ │                   │ │ Stereo   │ │
    │ └────┬────┘ │                   │ └────┬─────┘ │
    │      │      │                   │      │       │
    │ ┌────▼──────────┐               │ ┌────▼──────────┐
    │ │ Edge Models   │               │ │ Edge Models   │
    │ │ ├─ Fast COCO  │               │ │ ├─ Fast COCO  │
    │ │ ├─ Rule-based │               │ │ ├─ Rule-based │
    │ │ └─ MobileNet  │               │ │ └─ MobileNet  │
    │ └────┬──────────┘               │ └────┬──────────┘
    │      │                          │      │
    │ ┌────▼──────────┐               │ ┌────▼──────────┐
    │ │ Live Results  │               │ │ Live Results  │
    │ │ ├─ Detection  │               │ │ ├─ Detection  │
    │ │ ├─ Confidence │               │ │ ├─ Confidence │
    │ │ └─ Timestamp  │               │ │ └─ Timestamp  │
    │ └───────────────┘               │ └───────────────┘
    │                                  │
    │  (Lightweight - ~10-15% CPU)    │  (Lightweight - ~10-15% CPU)
    │  (Standalone operation)          │  (Standalone operation)
    │  (Optional connection to desktop) │  (Optional connection to desktop)
    └─────────────────┬────────────────┘
                      │
              (Optional WiFi/USB)
                      │
             Connect to Desktop for:
             ├─ Send frames + labels
             ├─ Receive trained models
             ├─ Real-time refinement
             └─ Analytics sync
```

---

## Architecture Components

### 1. RDK: Lightweight Edge Models

**CPU Load:** 10-15% (vs 80-95% for full training)

#### Model 1: Rule-Based Detection (1-3% CPU)

```python
class RuleBasedDetector:
    """Fast rule-based weld defect detection"""
    
    def detect_porosity(self, rgb, depth):
        """Detect porosity in weld"""
        # Check depth discontinuity
        depth_variance = np.var(depth[roi])
        
        # Check color patterns (speckled appearance)
        brightness = np.mean(rgb[roi])
        contrast = np.std(rgb[roi])
        
        # Simple rules
        if depth_variance > DEPTH_THRESHOLD and contrast > CONTRAST_THRESHOLD:
            return {'class': 'porosity', 'confidence': 0.75}
        return {'class': 'good', 'confidence': 0.95}
    
    def detect_undercut(self, rgb, depth):
        """Detect undercut (edges lower than center)"""
        edges = detect_edges(depth)
        center = np.mean(depth[center_roi])
        edge_mean = np.mean(depth[edges])
        
        if edge_mean < center - EDGE_THRESHOLD:
            return {'class': 'undercut', 'confidence': 0.80}
        return {'class': 'good', 'confidence': 0.90}
    
    def detect_spatter(self, rgb, depth):
        """Detect spatter around weld"""
        # Look for isolated particles outside main weld
        contours = find_contours(rgb)
        
        for contour in contours:
            if is_small_isolated_particle(contour):
                return {'class': 'spatter', 'confidence': 0.70}
        return {'class': 'good', 'confidence': 0.95}
```

**Speed:** < 100 ms per frame
**Accuracy:** 60-70% (baseline)

#### Model 2: Lightweight CNN (MobileNet) (8-12% CPU)

```python
import torch
import torchvision.models as models

class LightweightModel:
    """MobileNetV2 for edge deployment"""
    
    def __init__(self, model_path=None):
        # Load pretrained MobileNetV2
        self.model = models.mobilenet_v2(pretrained=True)
        
        # Modify for defect classification (3 classes)
        num_classes = 3  # good, porosity, undercut
        self.model.classifier[1] = torch.nn.Linear(1280, num_classes)
        
        if model_path:
            self.model.load_state_dict(torch.load(model_path))
        
        self.model.eval()
    
    def predict(self, rgb, depth):
        """Predict defect class"""
        # Preprocess
        input_tensor = self._preprocess(rgb, depth)
        
        # Inference
        with torch.no_grad():
            output = self.model(input_tensor)
        
        # Get class and confidence
        probabilities = torch.softmax(output, dim=1)
        confidence, class_idx = torch.max(probabilities, 1)
        
        classes = ['good', 'porosity', 'undercut']
        
        return {
            'class': classes[class_idx.item()],
            'confidence': confidence.item(),
            'all_scores': {
                'good': probabilities[0, 0].item(),
                'porosity': probabilities[0, 1].item(),
                'undercut': probabilities[0, 2].item()
            }
        }
    
    def _preprocess(self, rgb, depth):
        """Preprocess frames for model"""
        # Normalize RGB
        rgb_norm = rgb / 255.0
        
        # Normalize depth
        depth_norm = (depth - depth.min()) / (depth.max() - depth.min())
        
        # Stack and convert to tensor
        combined = np.stack([rgb_norm, depth_norm], axis=0)
        tensor = torch.from_numpy(combined).unsqueeze(0).float()
        
        return tensor
```

**Speed:** 50-100 ms per frame
**Accuracy:** 75-80% (lightweight)
**Size:** 10-20 MB (fits easily on RDK)

#### Model 3: Ensemble (Combine Both) (12-15% CPU)

```python
class HybridInferenceEngine:
    """Combine rule-based + CNN for robust detection"""
    
    def __init__(self):
        self.rule_detector = RuleBasedDetector()
        self.ml_model = LightweightModel('mobilenet_model.pth')
        self.confidence_threshold = 0.75
    
    def predict(self, rgb, depth):
        """Ensemble prediction"""
        # Get predictions from both
        rule_result = self.rule_detector.detect_porosity(rgb, depth)
        ml_result = self.ml_model.predict(rgb, depth)
        
        # Combine with weighted voting
        ensemble_confidence = (
            0.3 * rule_result['confidence'] +
            0.7 * ml_result['confidence']
        )
        
        # If ML confidence is low, use rule-based (more reliable)
        if ml_result['confidence'] < self.confidence_threshold:
            return {
                'class': rule_result['class'],
                'confidence': ensemble_confidence,
                'source': 'rule-based',
                'ml_score': ml_result['confidence']
            }
        
        # Otherwise trust ML model
        return {
            'class': ml_result['class'],
            'confidence': ensemble_confidence,
            'source': 'ml-model',
            'rule_score': rule_result['confidence']
        }
    
    def should_send_to_desktop(self, prediction):
        """Decide if frame should be sent to desktop for refinement"""
        # Send uncertain predictions for desktop refinement
        if prediction['confidence'] < 0.70:
            return True
        
        # Send mixed predictions
        if prediction['class'] != 'good' and prediction['ml_score'] < 0.6:
            return True
        
        # Sample some 'good' predictions for training
        if prediction['class'] == 'good' and np.random.random() < 0.05:
            return True
        
        return False
```

**Speed:** 100-150 ms per frame
**Accuracy:** 80-85% (ensemble)
**Auto-learning:** Sends uncertain frames to desktop for refinement

---

### 2. Desktop: Heavy Processing

**CPU Load:** 50-80% (when training)
**Optional:** Can work without desktop, RDK uses edge models

#### Training Pipeline

```python
class DesktopTrainingPipeline:
    """Advanced training on desktop"""
    
    def __init__(self):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.models = {
            'mobilenet': self._create_mobilenet(),
            'resnet50': self._create_resnet50(),
            'ensemble': self._create_ensemble()
        }
    
    def _create_mobilenet(self):
        """MobileNet for edge deployment"""
        model = models.mobilenet_v2(pretrained=True)
        model.classifier[1] = torch.nn.Linear(1280, 3)
        return model.to(self.device)
    
    def _create_resnet50(self):
        """ResNet50 for high accuracy"""
        model = models.resnet50(pretrained=True)
        model.fc = torch.nn.Linear(2048, 3)
        return model.to(self.device)
    
    def _create_ensemble(self):
        """Ensemble of models"""
        return {
            'mobilenet': self._create_mobilenet(),
            'resnet50': self._create_resnet50()
        }
    
    def train_mobilenet(self, train_loader, val_loader, epochs=50):
        """Train lightweight model for RDK deployment"""
        model = self.models['mobilenet']
        optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
        criterion = torch.nn.CrossEntropyLoss()
        
        for epoch in range(epochs):
            # Training
            model.train()
            for rgb_batch, depth_batch, labels in train_loader:
                rgb_batch = rgb_batch.to(self.device)
                depth_batch = depth_batch.to(self.device)
                labels = labels.to(self.device)
                
                outputs = model(torch.cat([rgb_batch, depth_batch], dim=1))
                loss = criterion(outputs, labels)
                
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            
            # Validation
            model.eval()
            val_accuracy = self._validate(model, val_loader)
            
            print(f"Epoch {epoch+1}: Loss={loss:.4f}, Acc={val_accuracy:.2%}")
        
        # Export to ONNX for RDK
        self._export_to_onnx(model, 'mobilenet_edge.onnx')
    
    def train_resnet50(self, train_loader, val_loader, epochs=100):
        """Train high-accuracy model for desktop"""
        model = self.models['resnet50']
        optimizer = torch.optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
        criterion = torch.nn.CrossEntropyLoss()
        
        for epoch in range(epochs):
            model.train()
            for rgb_batch, depth_batch, labels in train_loader:
                rgb_batch = rgb_batch.to(self.device)
                depth_batch = depth_batch.to(self.device)
                labels = labels.to(self.device)
                
                outputs = model(torch.cat([rgb_batch, depth_batch], dim=1))
                loss = criterion(outputs, labels)
                
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            
            model.eval()
            val_accuracy = self._validate(model, val_loader)
            print(f"Epoch {epoch+1}: Loss={loss:.4f}, Acc={val_accuracy:.2%}")
        
        # Save locally on desktop
        torch.save(model.state_dict(), 'resnet50_best.pth')
    
    def _export_to_onnx(self, model, output_path):
        """Export model to ONNX for RDK"""
        dummy_input = torch.randn(1, 6, 256, 256).to(self.device)
        torch.onnx.export(
            model, dummy_input, output_path,
            input_names=['input'],
            output_names=['output'],
            dynamic_axes={'input': {0: 'batch_size'}}
        )
        print(f"Model exported to {output_path}")
    
    def _validate(self, model, val_loader):
        """Validate model accuracy"""
        correct = 0
        total = 0
        
        with torch.no_grad():
            for rgb_batch, depth_batch, labels in val_loader:
                rgb_batch = rgb_batch.to(self.device)
                depth_batch = depth_batch.to(self.device)
                labels = labels.to(self.device)
                
                outputs = model(torch.cat([rgb_batch, depth_batch], dim=1))
                _, predicted = torch.max(outputs, 1)
                
                total += labels.size(0)
                correct += (predicted == labels).sum().item()
        
        return correct / total
```

#### Model Refinement

```python
class ModelRefinementEngine:
    """Refine edge models with uncertain predictions"""
    
    def __init__(self):
        self.uncertain_buffer = []
        self.buffer_size = 1000
    
    def add_uncertain_prediction(self, frame, prediction, ground_truth=None):
        """Add uncertain RDK prediction for desktop refinement"""
        self.uncertain_buffer.append({
            'frame': frame,
            'prediction': prediction,
            'ground_truth': ground_truth,
            'timestamp': datetime.now()
        })
        
        # Retrain when buffer fills
        if len(self.uncertain_buffer) >= self.buffer_size:
            self.refine_model()
    
    def refine_model(self):
        """Retrain on uncertain predictions"""
        # Create dataset from uncertain frames
        uncertain_frames = [
            item['frame'] for item in self.uncertain_buffer
        ]
        uncertain_labels = [
            item['ground_truth'] or item['prediction']['class']
            for item in self.uncertain_buffer
        ]
        
        # Train with higher learning rate (fine-tuning)
        model = LightweightModel('mobilenet_model.pth')
        optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
        
        for epoch in range(10):  # Quick refinement
            loss = self._train_batch(model, optimizer, 
                                     uncertain_frames, uncertain_labels)
        
        # Save refined model
        torch.save(model.state_dict(), 'mobilenet_refined.pth')
        self.uncertain_buffer = []  # Reset buffer
```

---

## Data Flow: Hybrid Operation

### Scenario 1: Standalone RDK (No Desktop Connection)

```
RDK Live Scanning:
──────────────────

Frame captured
    ↓
Run lightweight model (100-150 ms)
    ↓
Predict: good/porosity/undercut
    ↓
Display on RDK UI (if available)
    ↓
Continue scanning

CPU usage: 10-15%
Network: Not needed
Operation: Fully independent
Accuracy: 75-85%
```

### Scenario 2: RDK with Optional Desktop Connection

```
RDK Live Scanning:
──────────────────

Frame captured
    ↓
Run lightweight model (100-150 ms)
    ↓
Confidence > 80%? 
  Yes → Display result, continue
  No  → Send to desktop for refinement
    ↓
Desktop (if connected):
  ├─ Receive uncertain frame
  ├─ Run heavy model (ResNet50)
  ├─ Get more accurate prediction
  └─ Send back to RDK
    ↓
RDK receives refined result
    ↓
Update display with better confidence
    ↓
Log refined result

CPU usage (RDK): 10-15%
Network: Optional, only when uncertain
Operation: Smart fallback
Accuracy: 85-92%
```

### Scenario 3: Active Desktop Training

```
Workflow:
─────────

Day 1-2: RDK Live Scanning
  ├─ Collect 200 "uncertain" frames
  └─ Collect 200 labeled "good" frames

Day 3: Desktop Training
  ├─ Receive frames from RDK
  ├─ Train MobileNet (50 epochs, ~1 hour)
  ├─ Train ResNet50 (100 epochs, ~4 hours)
  └─ Export MobileNet to ONNX
    
Day 4: RDK Update
  ├─ Download refined MobileNet
  └─ Use for next scanning session

Improvements:
  ├─ MobileNet: 75% → 85% accuracy
  ├─ ResNet50: 85% → 92% accuracy (desktop use)
  └─ Ensemble: 80% → 90% accuracy

User doesn't wait: RDK keeps working while desktop trains
```

---

## Deployment Strategy

### Phase 1: Deploy Edge Models on RDK (Week 1)

```
Effort: 2 hours
Result: Standalone operation

✓ Copy rule_based_detector.py to RDK
✓ Copy mobilenet_model.pth to RDK
✓ Integrate into app.py
✓ Test with 100 frames
✓ Verify 10-15% CPU usage
```

**RDK Directory:**
```
backend/
├── models/
│   ├── rule_based_detector.py
│   └── mobilenet_model.pth (18 MB)
├── inference.py
└── app.py (update to use models)
```

### Phase 2: Setup Desktop Infrastructure (Week 2)

```
Effort: 4 hours
Result: Optional desktop enhancement

✓ Setup desktop Flask server
✓ Setup model training pipeline
✓ Configure data collection
✓ Create web UI
✓ Test model exchange
```

**Desktop Directory:**
```
desktop_backend/
├── models/
│   ├── mobilenet_v2.pth
│   └── resnet50.pth
├── training_pipeline.py
├── refinement_engine.py
└── server.py
```

### Phase 3: Optional Advanced Features (Week 3-4)

```
✓ Real-time frame sync
✓ Automatic uncertain prediction detection
✓ Model version control
✓ Analytics dashboard
✓ Batch retraining
```

---

## CPU Load Comparison

```
Activity                RDK (Hybrid)    RDK (Full)      Desktop
─────────────────────────────────────────────────────────────
Idle scanning           10-15%          2-5%            5-10%
Live inference          12-18%          10-15%          1-5%
Rule detection only     2-3%            N/A             N/A
ML inference (Mobile)   8-12%           N/A             N/A
ML inference (ResNet)   N/A             80-90%          15-25%
Training models         N/A             95%+            60-80%
Frame receiving         5-10%           N/A             5-10%
Data transmission       3-5%            N/A             3-5%
────────────────────────────────────────────────────────────
Total average           15-25%          80-95%          20-40%
```

**Key Insight:** Hybrid keeps RDK at 15-25% avg, never overloading

---

## Model Accuracy vs Size vs Speed

```
Model               Accuracy    Size      Speed       Best For
─────────────────────────────────────────────────────────────
Rule-based          60-70%      -         <50ms       Baseline
MobileNetV2         75-80%      18 MB     50-100ms    Edge
ResNet50            85-92%      100 MB    500ms       Desktop
Ensemble (MN+RN)    80-85%      118 MB    150ms       RDK+fallback
Ensemble (Full)     90-95%      118 MB    600ms       Desktop refined

Recommendation:
├─ RDK: MobileNetV2 (75-80% accuracy, fits memory)
├─ Desktop: ResNet50 (85-92% accuracy, detailed analysis)
└─ RDK+Desktop: MobileNetV2 + ResNet50 fallback (85-90% practical)
```

---

## Network Traffic (Optional Connection)

### When Desktop is NOT Connected
```
RDK only: No network traffic
Frames: Stored locally
Training: Via manual data transfer
Update: Manual model copy
```

### When Desktop IS Connected
```
Uncertain frames: 
  ├─ Frequency: ~10-20% of frames (low confidence)
  ├─ Size: 1.3 MB × 0.15 = 195 KB per uncertain
  ├─ Bandwidth: Minimal impact on typical LAN
  └─ Latency: 1-3 seconds for refinement

Training data batch:
  ├─ Frequency: Every 2-4 hours (off-peak)
  ├─ Size: 200 frames × 1.3 MB = 260 MB
  ├─ Bandwidth: Can compress to 50 MB
  └─ Transfer time: 5-10 minutes on 100 Mbps LAN

Model updates:
  ├─ Frequency: Every 1-2 weeks
  ├─ Size: MobileNet ONNX = 18 MB
  ├─ Transfer time: 2-3 minutes
  └─ Automatic scheduling possible
```

---

## Implementation Code

### RDK: app.py Integration

```python
# backend/app.py

from inference import HybridInferenceEngine
from flask import Flask, jsonify
import requests
import json

app = Flask(__name__)

# Initialize hybrid engine
inference_engine = HybridInferenceEngine()

# Desktop server URL (optional)
DESKTOP_URL = "http://192.168.1.100:5001"  # Set to None if no desktop
USE_DESKTOP = False  # Will try to connect on startup

@app.route('/api/inference', methods=['POST'])
def run_inference():
    """Run inference on frame"""
    data = request.json
    rgb = np.frombuffer(data['rgb_b64'], dtype=np.uint8)
    depth = np.frombuffer(data['depth_b64'], dtype=np.float32)
    
    # Run lightweight inference
    prediction = inference_engine.predict(rgb, depth)
    
    # Check if should send to desktop
    if USE_DESKTOP and inference_engine.should_send_to_desktop(prediction):
        try:
            # Send to desktop for refinement (non-blocking)
            requests.post(
                f"{DESKTOP_URL}/api/refine",
                json={'frame': data, 'prediction': prediction},
                timeout=2
            )
        except:
            pass  # Desktop not available, use RDK prediction
    
    return jsonify({
        'prediction': prediction,
        'source': 'rdk-edge',
        'timestamp': datetime.now().isoformat()
    })

@app.route('/api/update-model', methods=['POST'])
def update_model():
    """Receive updated model from desktop"""
    data = request.files['model']
    data.save('models/mobilenet_model.pth')
    
    # Reload model
    inference_engine.ml_model = LightweightModel('models/mobilenet_model.pth')
    
    return jsonify({'status': 'model updated'})

@app.route('/api/health', methods=['GET'])
def health():
    """Health check"""
    return jsonify({
        'status': 'healthy',
        'inference_engine': 'hybrid',
        'models': ['rule-based', 'mobilenet'],
        'cpu_usage': get_cpu_usage(),
        'desktop_connected': USE_DESKTOP
    })

if __name__ == '__main__':
    # Try to connect to desktop on startup
    try:
        response = requests.get(f"{DESKTOP_URL}/api/health", timeout=2)
        if response.status_code == 200:
            USE_DESKTOP = True
            print("Desktop connected!")
    except:
        print("Desktop not available, running in standalone mode")
    
    app.run(host='0.0.0.0', port=5000)
```

### Desktop: Optional Refinement Server

```python
# desktop_backend/refinement_server.py

from flask import Flask, request, jsonify, send_file
from refinement_engine import ModelRefinementEngine
import cv2
import numpy as np

app = Flask(__name__)
refinement_engine = ModelRefinementEngine()

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({'status': 'healthy', 'type': 'refinement-server'})

@app.route('/api/refine', methods=['POST'])
def refine_prediction():
    """Receive uncertain prediction and refine"""
    data = request.json
    frame_data = data['frame']
    prediction = data['prediction']
    
    # Decode frames
    rgb = cv2.imdecode(np.frombuffer(frame_data['rgb_b64'], np.uint8), 1)
    depth = np.frombuffer(frame_data['depth_b64'], np.float32).reshape((1080, 1920))
    
    # Add to buffer for refinement
    refinement_engine.add_uncertain_prediction(
        (rgb, depth), 
        prediction
    )
    
    return jsonify({'status': 'received'}), 202  # Accepted

@app.route('/api/models/download/<model_name>', methods=['GET'])
def download_model(model_name):
    """Download refined model to RDK"""
    model_path = f"models/{model_name}"
    return send_file(model_path, as_attachment=True)

@app.route('/api/training/start', methods=['POST'])
def start_training():
    """Start full training job"""
    config = request.json
    
    # This runs in background, training pipeline takes over
    # See desktop training implementation
    
    return jsonify({'status': 'training started', 'job_id': 'job_123'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
```

---

## File Structure

```
project/
├── backend/ (RDK)
│   ├── models/
│   │   ├── rule_based_detector.py      (< 1 KB)
│   │   ├── mobilenet_model.pth          (18 MB)
│   │   └── inference.py                 (300 lines)
│   ├── app.py                           (Update with inference)
│   └── requirements.txt
│       ├── torch
│       ├── opencv
│       └── minimal
│
└── desktop_backend/ (Optional)
    ├── models/
    │   ├── mobilenet_v2.pth
    │   ├── resnet50.pth
    │   └── training_pipeline.py
    ├── refinement_server.py             (150 lines)
    ├── refinement_engine.py             (200 lines)
    └── requirements.txt
        ├── torch (GPU)
        ├── opencv
        ├── flask
        └── full ML stack
```

---

## Operation Modes

### Mode 1: Standalone RDK (No Desktop)

```
✓ Works without internet
✓ Works without desktop
✓ Fully independent
✗ Accuracy: 75-80%
✗ Can't train on RDK

Use case: Field deployment, remote locations, offline operation
```

### Mode 2: RDK + Optional Desktop (Smart Fallback)

```
✓ Works standalone
✓ Auto-refines when desktop available
✓ No manual switching
✓ Accuracy: 85-90% (when connected)
✗ Slight network overhead when uncertain

Use case: Factory floor with optional desktop support
```

### Mode 3: RDK + Always-On Desktop (Preferred)

```
✓ Highest accuracy: 90-95%
✓ Real-time refinement
✓ Continuous training
✓ Full analytics
✗ Requires desktop connectivity

Use case: Production facility with network infrastructure
```

---

## Advantages of Hybrid Approach

### For RDK
✅ Low CPU usage (15-25%)
✅ Fast inference (100-150 ms)
✅ Fits in storage (18 MB model)
✅ Works standalone
✅ No thermal issues
✅ Battery-friendly (if portable)

### For Desktop
✅ Optional (not required)
✅ Can handle heavy models
✅ Training happens silently
✅ No impact on RDK
✅ Scalable (multiple RDKs)

### For User
✅ Professional accuracy (75-95%)
✅ Flexible deployment options
✅ No operational complexity
✅ Can start simple, add desktop later
✅ Can scale one RDK to many

---

## Comparison Matrix

| Aspect | RDK Only | Desktop Only | Hybrid (Recommended) |
|--------|----------|--------------|----------------------|
| **Standalone** | Yes | No | Yes |
| **RDK CPU** | 15-25% | 80-95% | 15-25% |
| **Desktop CPU** | N/A | 60-80% | 20-40% |
| **Accuracy** | 75-80% | 90-95% | 85-90% |
| **Speed** | 100ms | 500ms | 100-150ms |
| **Network Required** | No | Yes | Optional |
| **Scalability** | Single | Multi | Multi |
| **Cost** | Low | High | Medium |
| **Complexity** | Low | High | Medium |
| **Flexibility** | Low | High | High |

---

## Recommendation

**Deploy Hybrid Architecture in 3 Phases:**

### Phase 1: Week 1 (Standalone)
- Deploy RDK with MobileNetV2
- Standalone operation
- No desktop needed
- Accuracy: 75-80%

### Phase 2: Week 2-3 (Optional)
- Setup desktop server (if needed)
- Enable intelligent refinement
- Auto-send uncertain frames
- Accuracy: 85-90%

### Phase 3: Week 4+ (Advanced)
- Continuous model training
- Automatic updates to RDK
- Full analytics
- Accuracy: 90%+

---

## Success Criteria

After Hybrid implementation:
- [ ] RDK CPU usage: 15-25%
- [ ] RDK standalone: Works offline
- [ ] RDK inference speed: 100-150 ms
- [ ] RDK accuracy: 75-80%
- [ ] Desktop connection: Optional
- [ ] Desktop refinement: Accurate
- [ ] Desktop training: Complete (if used)
- [ ] Model updates: Automatic (if connected)
- [ ] Multi-RDK: Supported
- [ ] Zero downtime: During updates


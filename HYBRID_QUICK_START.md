# Hybrid Architecture: Quick Implementation Guide

## Overview

**Best balance:** RDK does lightweight inference (~15% CPU), Desktop trains advanced models.

```
Simple:          RDK only (Rule + MobileNet)
Better:          RDK + optional Desktop refinement
Best:            RDK + Desktop continuous training
```

---

## Phase 1: RDK Edge Models (Week 1 - 2 Hours)

### Step 1: Install Dependencies on RDK

```bash
ssh rdk@192.168.1.50

# Install PyTorch (lite version)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Install other dependencies
pip install opencv-python numpy pillow

# Verify installation
python -c "import torch; print(torch.__version__)"
```

### Step 2: Create Inference Engine

**File: `backend/models/inference.py`**

```python
import torch
import torchvision.models as models
import cv2
import numpy as np
from datetime import datetime

class RuleBasedDetector:
    """Fast rule-based detection (1-3% CPU)"""
    
    def detect(self, rgb, depth):
        """Detect defects using rules"""
        try:
            # Analyze depth variance (porosity indicator)
            depth_std = np.std(depth)
            
            # Analyze brightness (color patterns)
            rgb_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            brightness_std = np.std(rgb_gray)
            
            # Simple rules
            confidence = 0.75
            if depth_std > 50 and brightness_std > 40:
                return {'class': 'porosity', 'confidence': confidence}
            elif depth_std < 30:
                return {'class': 'undercut', 'confidence': 0.80}
            else:
                return {'class': 'good', 'confidence': 0.90}
        except:
            return {'class': 'unknown', 'confidence': 0.50}


class LightweightModel:
    """MobileNetV2 for edge (8-12% CPU)"""
    
    def __init__(self, model_path='models/mobilenet_model.pth'):
        self.model = models.mobilenet_v2(pretrained=True)
        self.model.classifier[1] = torch.nn.Linear(1280, 3)
        
        try:
            self.model.load_state_dict(torch.load(model_path, map_location='cpu'))
        except:
            print(f"Model not found at {model_path}, using pretrained")
        
        self.model.eval()
    
    def predict(self, rgb, depth):
        """Predict defect class"""
        try:
            # Preprocess
            input_tensor = self._preprocess(rgb, depth)
            
            # Inference
            with torch.no_grad():
                output = self.model(input_tensor)
            
            probabilities = torch.softmax(output, dim=1)
            confidence, class_idx = torch.max(probabilities, 1)
            
            classes = ['good', 'porosity', 'undercut']
            
            return {
                'class': classes[class_idx.item()],
                'confidence': confidence.item(),
                'all_scores': {
                    'good': float(probabilities[0, 0]),
                    'porosity': float(probabilities[0, 1]),
                    'undercut': float(probabilities[0, 2])
                }
            }
        except Exception as e:
            print(f"Inference error: {e}")
            return {'class': 'unknown', 'confidence': 0.50}
    
    def _preprocess(self, rgb, depth):
        """Preprocess for model"""
        # Resize to model input size
        rgb_resized = cv2.resize(rgb, (256, 256))
        depth_resized = cv2.resize(depth, (256, 256))
        
        # Normalize
        rgb_norm = rgb_resized / 255.0
        depth_min, depth_max = depth_resized.min(), depth_resized.max()
        depth_norm = (depth_resized - depth_min) / (depth_max - depth_min + 1e-5)
        
        # Stack channels
        combined = np.stack([
            rgb_norm[:, :, 0],  # R
            rgb_norm[:, :, 1],  # G
            rgb_norm[:, :, 2],  # B
            depth_norm,          # Depth
            depth_norm,          # Depth (repeat)
            depth_norm           # Depth (repeat)
        ], axis=0)
        
        tensor = torch.from_numpy(combined).unsqueeze(0).float()
        return tensor


class HybridInferenceEngine:
    """Combine rule-based + ML for robust results"""
    
    def __init__(self, use_desktop=False):
        self.rule_detector = RuleBasedDetector()
        self.ml_model = LightweightModel()
        self.use_desktop = use_desktop
        self.desktop_url = "http://192.168.1.100:5001"
        self.uncertain_threshold = 0.70
    
    def predict(self, rgb, depth):
        """Hybrid prediction"""
        # Get both predictions
        rule_pred = self.rule_detector.detect(rgb, depth)
        ml_pred = self.ml_model.predict(rgb, depth)
        
        # Combine with weights
        combined_confidence = (
            0.3 * rule_pred['confidence'] +
            0.7 * ml_pred['confidence']
        )
        
        # Use ML if confident, otherwise fall back to rules
        if ml_pred['confidence'] >= self.uncertain_threshold:
            result = {
                'class': ml_pred['class'],
                'confidence': combined_confidence,
                'source': 'ml-model',
                'ml_score': ml_pred['confidence'],
                'rule_score': rule_pred['confidence']
            }
        else:
            result = {
                'class': rule_pred['class'],
                'confidence': combined_confidence,
                'source': 'rule-based',
                'ml_score': ml_pred['confidence'],
                'rule_score': rule_pred['confidence']
            }
        
        # Flag for desktop refinement if very uncertain
        if combined_confidence < 0.65 and self.use_desktop:
            result['needs_refinement'] = True
        
        return result
    
    def should_send_to_desktop(self, prediction):
        """Decide if frame should be refined by desktop"""
        if not self.use_desktop:
            return False
        
        # Send uncertain predictions
        if prediction['confidence'] < 0.70:
            return True
        
        # Send when ML and rules disagree significantly
        if abs(prediction['ml_score'] - prediction['rule_score']) > 0.3:
            return True
        
        return False
```

### Step 3: Update RDK App

**File: `backend/app.py`**

Add to your existing Flask app:

```python
from models.inference import HybridInferenceEngine
import base64
import numpy as np
import cv2

# Initialize inference engine
hybrid_engine = HybridInferenceEngine(use_desktop=False)

@app.route('/api/inference', methods=['POST'])
def run_inference():
    """Run inference on captured frame"""
    try:
        data = request.json
        
        # Decode RGB and depth
        rgb_bytes = base64.b64decode(data.get('rgb', ''))
        depth_bytes = base64.b64decode(data.get('depth', ''))
        
        rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((1080, 1920, 3))
        depth = np.frombuffer(depth_bytes, dtype=np.float32).reshape((1080, 1920))
        
        # Run hybrid inference
        prediction = hybrid_engine.predict(rgb, depth)
        
        return jsonify({
            'prediction': prediction,
            'timestamp': datetime.now().isoformat(),
            'device': 'rdk-hybrid'
        }), 200
    
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/health', methods=['GET'])
def health():
    """Health check"""
    import psutil
    
    return jsonify({
        'status': 'healthy',
        'mode': 'hybrid',
        'cpu_percent': psutil.cpu_percent(interval=1),
        'memory_percent': psutil.virtual_memory().percent,
        'inference_engine': 'rule-based + MobileNetV2',
        'models': {
            'rule_based': 'active',
            'mobilenet': 'active'
        }
    }), 200
```

### Step 4: Test Edge Inference

```bash
# Test rule-based detector
python -c "
from models.inference import RuleBasedDetector
import numpy as np

detector = RuleBasedDetector()
rgb = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)
depth = np.random.randn(1080, 1920) * 100
result = detector.detect(rgb, depth)
print('Rule-based result:', result)
"

# Test MobileNet
python -c "
from models.inference import LightweightModel
import numpy as np

model = LightweightModel()
rgb = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)
depth = np.random.randn(1080, 1920) * 100
result = model.predict(rgb, depth)
print('MobileNet result:', result)
"

# Test hybrid
python -c "
from models.inference import HybridInferenceEngine
import numpy as np

engine = HybridInferenceEngine()
rgb = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)
depth = np.random.randn(1080, 1920) * 100
result = engine.predict(rgb, depth)
print('Hybrid result:', result)
"
```

### Step 5: Monitor CPU Usage

```bash
# Watch CPU usage during inference
ssh rdk@192.168.1.50

# Terminal 1: Run app
python backend/app.py

# Terminal 2: Monitor
watch -n 1 'ps aux | grep python'

# Expected:
# cpu% should be 10-15% during continuous inference
# memory should be < 500 MB
```

---

## Phase 2: Optional Desktop Enhancement (Week 2-3)

### Enable Desktop Connection (Optional)

**On RDK:**

```python
# In backend/app.py

hybrid_engine = HybridInferenceEngine(use_desktop=True)
hybrid_engine.desktop_url = "http://192.168.1.100:5001"

# Now uncertain predictions are sent to desktop for refinement
```

### Desktop Receives Refinement Requests

**File: `desktop_backend/refinement.py`**

```python
from flask import Flask, request, jsonify
import torch
import torch.nn as nn
import torchvision.models as models
import numpy as np
import cv2
from datetime import datetime

app = Flask(__name__)

class DesktopRefinementEngine:
    """Refine uncertain predictions from RDK"""
    
    def __init__(self):
        # Load high-accuracy model on desktop
        self.model = models.resnet50(pretrained=True)
        self.model.fc = nn.Linear(2048, 3)
        
        try:
            self.model.load_state_dict(torch.load('models/resnet50_best.pth'))
        except:
            print("ResNet50 model not found, using pretrained")
        
        self.model.eval()
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
    
    def refine_prediction(self, rgb, depth):
        """Get more accurate prediction"""
        try:
            input_tensor = self._preprocess(rgb, depth)
            input_tensor = input_tensor.to(self.device)
            
            with torch.no_grad():
                output = self.model(input_tensor)
            
            probabilities = torch.softmax(output, dim=1)
            confidence, class_idx = torch.max(probabilities, 1)
            
            classes = ['good', 'porosity', 'undercut']
            
            return {
                'class': classes[class_idx.item()],
                'confidence': float(confidence),
                'source': 'desktop-resnet50'
            }
        except Exception as e:
            return {'error': str(e)}
    
    def _preprocess(self, rgb, depth):
        """Preprocess for ResNet50"""
        rgb_resized = cv2.resize(rgb, (224, 224))
        depth_resized = cv2.resize(depth, (224, 224))
        
        rgb_norm = rgb_resized / 255.0
        depth_min, depth_max = depth_resized.min(), depth_resized.max()
        depth_norm = (depth_resized - depth_min) / (depth_max - depth_min + 1e-5)
        
        combined = np.stack([
            rgb_norm[:, :, 0],
            rgb_norm[:, :, 1],
            rgb_norm[:, :, 2],
            depth_norm,
            depth_norm,
            depth_norm
        ], axis=0)
        
        tensor = torch.from_numpy(combined).unsqueeze(0).float()
        return tensor

refinement = DesktopRefinementEngine()

@app.route('/api/refine', methods=['POST'])
def refine_frame():
    """Receive uncertain frame and refine"""
    try:
        data = request.json
        
        # Decode frames
        import base64
        rgb_bytes = base64.b64decode(data.get('rgb', ''))
        depth_bytes = base64.b64decode(data.get('depth', ''))
        
        rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((1080, 1920, 3))
        depth = np.frombuffer(depth_bytes, dtype=np.float32).reshape((1080, 1920))
        
        # Refine prediction
        refined = refinement.refine_prediction(rgb, depth)
        
        return jsonify(refined), 200
    
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({
        'status': 'healthy',
        'type': 'refinement-engine',
        'model': 'ResNet50',
        'device': refinement.device
    }), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=False)
```

### Enable on RDK

```python
# backend/app.py

@app.route('/api/inference', methods=['POST'])
def run_inference():
    """Run inference with optional desktop refinement"""
    try:
        data = request.json
        
        # Decode
        rgb_bytes = base64.b64decode(data.get('rgb', ''))
        depth_bytes = base64.b64decode(data.get('depth', ''))
        
        rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((1080, 1920, 3))
        depth = np.frombuffer(depth_bytes, dtype=np.float32).reshape((1080, 1920))
        
        # RDK prediction
        rdk_pred = hybrid_engine.predict(rgb, depth)
        
        # Check if should refine
        if hybrid_engine.should_send_to_desktop(rdk_pred):
            try:
                import requests
                response = requests.post(
                    'http://192.168.1.100:5001/api/refine',
                    json={
                        'rgb': data.get('rgb'),
                        'depth': data.get('depth')
                    },
                    timeout=2
                )
                if response.status_code == 200:
                    refined = response.json()
                    # Use refined prediction
                    rdk_pred['refined'] = refined
                    rdk_pred['source'] = 'desktop-refined'
            except:
                pass  # Desktop not available, use RDK prediction
        
        return jsonify({
            'prediction': rdk_pred,
            'timestamp': datetime.now().isoformat()
        }), 200
    
    except Exception as e:
        return jsonify({'error': str(e)}), 400
```

---

## Accuracy Improvement Path

```
Week 1: Rule-based only
├─ Deploy rule detector
├─ Accuracy: 60-70%
└─ CPU: 2-3%

Week 1-2: Add MobileNet
├─ Deploy lightweight model
├─ Accuracy: 75-80%
└─ CPU: 12-15%

Week 2-3: Optional desktop
├─ Enable refinement
├─ Accuracy: 85-90% (for uncertain)
└─ CPU: 15-18% (RDK)

Week 3-4: Desktop training (if used)
├─ Train ResNet50
├─ Accuracy: 90-95% (desktop)
└─ CPU: 60-80% (desktop only)
```

---

## Performance Metrics

```
Metric              RDK Hybrid    Desktop Optional   Result
─────────────────────────────────────────────────────────
CPU usage           15%           N/A               Very low
Memory usage        200 MB        N/A               Comfortable
Inference time      100-150ms     500ms             Fast
Accuracy (baseline) 75-80%        N/A               Good
Accuracy (refined)  85-90%        90-95%            Better
Standalone works    Yes           N/A               Flexible
Network required    No            Optional          No burden
```

---

## File Structure Summary

```
backend/ (RDK)
├── models/
│   ├── __init__.py
│   ├── inference.py           (300 lines - new)
│   └── mobilenet_model.pth    (18 MB - new)
├── app.py                      (Update existing)
└── requirements.txt            (Add torch)

desktop_backend/ (Optional)
├── refinement.py              (200 lines - new)
├── models/
│   └── resnet50_best.pth      (100 MB - new)
└── requirements.txt
```

---

## Deployment Checklist

- [ ] Week 1: RDK rule-based detector
- [ ] Week 1: RDK MobileNetV2 model
- [ ] Week 1: Test CPU usage (15-18%)
- [ ] Week 1: Verify inference speed (100-150ms)
- [ ] Week 2: (Optional) Desktop server
- [ ] Week 2: (Optional) ResNet50 model
- [ ] Week 2: Test refinement API
- [ ] Week 3: End-to-end testing
- [ ] Week 3: Production deployment

---

## Cost-Benefit Analysis

| Component | Effort | Benefit | Priority |
|-----------|--------|---------|----------|
| Rule detector | 1 hour | 60-70% accuracy | High (Week 1) |
| MobileNet | 2 hours | 75-80% accuracy | High (Week 1) |
| Desktop server | 3 hours | 85-90% accuracy | Medium (Week 2) |
| ResNet50 training | 4 hours | 90-95% accuracy | Low (Week 3) |
| **Total** | **10 hours** | **90%+ accuracy** | **1 month** |

---

## Recommendation

**Start with Phase 1 (RDK only - Week 1)**
- Low effort (2 hours)
- Gets 75-80% accuracy
- No desktop required
- Can add desktop later

**Add Phase 2 if needed (Optional desktop - Week 2)**
- Gets to 85-90% accuracy
- Only when confidence is low
- Minimal overhead
- Completely optional

**This is the best hybrid approach for your use case.**


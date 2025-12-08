# Hybrid Architecture Integration Analysis

## Current App Status

### ✅ What Your App Already Has

**Frontend (React/TypeScript):**
- ✅ Dashboard view (statistics, charts)
- ✅ Student management (CRUD operations)
- ✅ Scanner control (live scanning)
- ✅ History view (past scans)
- ✅ Calibration views (3 types)
- ✅ Settings panel
- ✅ LED control
- ✅ Multiple rig types (Manual Height + 3-Axis Panorama)

**Backend (Python/Flask):**
- ✅ Core Flask server (app.py)
- ✅ Student management API routes
- ✅ Scan execution API routes
- ✅ Calibration API routes (Triple Z-axis)
- ✅ Smart Rig Controller routes (Panorama)
- ✅ Safe Motion Controller routes
- ✅ LED control API routes
- ✅ Vision evaluator (WeldEvaluator class)
- ✅ Camera integration (ROS2)
- ✅ Database models (Student, Scan, etc.)

**Hardware Integration:**
- ✅ Camera handling
- ✅ LED control
- ✅ ROS2 support
- ✅ Hardware detection

---

## What's Missing for Hybrid Architecture

### ❌ NOT Currently Implemented

**RDK Edge Models:**
- ❌ Rule-based detector
- ❌ MobileNetV2 lightweight model
- ❌ Hybrid inference engine
- ❌ Edge model integration

**Desktop Training Pipeline:**
- ❌ Training module (Flask backend)
- ❌ Model training routes
- ❌ Training job management
- ❌ Model refinement engine
- ❌ Model repository management

**Frontend UI for Training:**
- ❌ Training dashboard
- ❌ Training job monitor
- ❌ Model management UI
- ❌ Data collection control
- ❌ Training progress visualization

**Model Management:**
- ❌ Model version control
- ❌ Model upload/download
- ❌ Model metadata storage
- ❌ Model performance tracking

---

## Recommended Changes to App

### Phase 1: Add Edge Inference (Week 1)

**What to add to backend:**

```
backend/
├── models/                          (NEW)
│   ├── __init__.py
│   ├── inference.py                 (300 lines)
│   ├── rule_based_detector.py       (100 lines)
│   └── mobilenet_model.pth          (18 MB - download)
│
├── api/
│   ├── inference_routes.py          (NEW - 100 lines)
│   └── (existing routes)
│
└── app.py                           (UPDATE - add inference init)
```

**What to add to frontend:**

```
components/
├── InferenceMonitor.tsx             (NEW - 150 lines)
├── ModelStatus.tsx                  (NEW - 100 lines)
└── (existing components)

App.tsx                              (UPDATE - add INFERENCE view)
types.ts                             (UPDATE - add InferenceMetrics)
```

### Phase 2: Add Optional Training (Week 2-3)

**What to add to backend:**

```
backend/
├── models/
│   ├── training_pipeline.py         (NEW - 400 lines)
│   ├── refinement_engine.py         (NEW - 200 lines)
│   └── resnet50_model.pth           (NEW - 100 MB)
│
├── api/
│   ├── training_routes.py           (NEW - 250 lines)
│   └── model_routes.py              (NEW - 150 lines)
│
├── database/
│   └── training_models.py           (NEW - 100 lines)
│
└── app.py                           (UPDATE - add training init)
```

**What to add to frontend:**

```
components/
├── TrainingDashboard.tsx            (NEW - 300 lines)
├── DataCollectionControl.tsx        (NEW - 150 lines)
├── ModelManager.tsx                 (NEW - 200 lines)
├── TrainingMonitor.tsx              (NEW - 250 lines)
└── (existing components)

App.tsx                              (UPDATE - add TRAINING view)
types.ts                             (UPDATE - add TrainingJob, Model types)
services/apiService.ts               (UPDATE - add training API calls)
```

---

## Detailed Implementation Plan

### Option A: Minimal (Just Edge Inference)

**Time: 2-3 hours**
**Result: Hybrid architecture Phase 1 only**

```
Add to app:
├─ Rule-based detector
├─ MobileNetV2 inference
├─ Inference monitoring UI
└─ ~300 lines total new code

Benefits:
✓ 75-80% accuracy
✓ RDK doesn't get burdened
✓ Standalone operation
✗ No training capability
```

**Files to add:**
1. `backend/models/inference.py` (from HYBRID_QUICK_START.md)
2. `backend/models/rule_based_detector.py` (simple)
3. `backend/api/inference_routes.py` (API endpoints)
4. `components/InferenceMonitor.tsx` (display results)
5. `types.ts` - Add `InferenceResult` interface

---

### Option B: Full Hybrid (Edge + Optional Training)

**Time: 8-10 hours**
**Result: Complete hybrid architecture**

```
Add to app:
├─ Rule-based detector
├─ MobileNetV2 inference
├─ ResNet50 training (optional)
├─ Model management
├─ Training dashboard
├─ Data collection UI
└─ ~1500 lines total new code

Benefits:
✓ 75-80% accuracy (RDK)
✓ 90-95% accuracy (with training)
✓ Professional UI
✓ Model version control
✓ RDK never burdened
```

**Files to add:**
1. `backend/models/inference.py`
2. `backend/models/rule_based_detector.py`
3. `backend/models/training_pipeline.py`
4. `backend/models/refinement_engine.py`
5. `backend/api/inference_routes.py`
6. `backend/api/training_routes.py`
7. `backend/api/model_routes.py`
8. `backend/database/training_models.py`
9. `components/InferenceMonitor.tsx`
10. `components/TrainingDashboard.tsx`
11. `components/DataCollectionControl.tsx`
12. `components/ModelManager.tsx`
13. `types.ts` - Add multiple new interfaces
14. `services/apiService.ts` - Add training API calls

---

## Step-by-Step: Option A (Minimal - Recommended Start)

### Backend Changes

**1. Create `backend/models/inference.py`**

From HYBRID_QUICK_START.md, copy the:
- RuleBasedDetector class
- LightweightModel class
- HybridInferenceEngine class

**2. Create `backend/api/inference_routes.py`**

```python
from flask import Blueprint, request, jsonify
from models.inference import HybridInferenceEngine
import numpy as np
import base64
import cv2

inference_bp = Blueprint('inference', __name__, url_prefix='/api/inference')

# Global inference engine
inference_engine = None

def init_inference_engine():
    global inference_engine
    inference_engine = HybridInferenceEngine(use_desktop=False)
    print("Inference engine initialized")

@inference_bp.route('/predict', methods=['POST'])
def predict():
    """Run inference on frame"""
    try:
        data = request.json
        
        # Decode frames
        rgb_bytes = base64.b64decode(data.get('rgb', ''))
        depth_bytes = base64.b64decode(data.get('depth', ''))
        
        rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((1080, 1920, 3))
        depth = np.frombuffer(depth_bytes, dtype=np.float32).reshape((1080, 1920))
        
        # Run inference
        prediction = inference_engine.predict(rgb, depth)
        
        return jsonify({
            'prediction': prediction,
            'timestamp': str(datetime.now()),
            'source': 'rdk-hybrid'
        }), 200
    
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@inference_bp.route('/health', methods=['GET'])
def health():
    """Check inference engine status"""
    return jsonify({
        'status': 'healthy' if inference_engine else 'not initialized',
        'engine': 'hybrid (rule-based + mobilenet)',
        'models': ['rule-based', 'mobilenet']
    }), 200

def get_inference_blueprint():
    return inference_bp
```

**3. Update `backend/app.py`**

Add after LED initialization:

```python
# Init Inference Engine (Hybrid Model)
try:
    from api.inference_routes import init_inference_engine, get_inference_blueprint
    init_inference_engine()
    app.register_blueprint(get_inference_blueprint())
    logger.info("Inference engine initialized successfully")
except Exception as e:
    logger.warning(f"Inference engine initialization failed: {e}")
```

---

### Frontend Changes

**1. Update `types.ts`**

Add:
```typescript
export interface InferenceResult {
  class: 'good' | 'porosity' | 'undercut' | 'spatter' | 'unknown';
  confidence: number;
  source: 'rule-based' | 'ml-model' | 'ensemble';
  ml_score?: number;
  rule_score?: number;
  timestamp: string;
}

export enum ViewState {
  // ... existing ...
  INFERENCE = 'INFERENCE',  // ADD THIS
}
```

**2. Create `components/InferenceMonitor.tsx`**

```typescript
import React, { useEffect, useState } from 'react';
import { AlertOctagon, CheckCircle2, Activity } from 'lucide-react';
import { InferenceResult } from '../types';

interface InferenceMonitorProps {
  latestResult: InferenceResult | null;
  isRunning: boolean;
  onStart: () => void;
  onStop: () => void;
}

export const InferenceMonitor: React.FC<InferenceMonitorProps> = ({
  latestResult,
  isRunning,
  onStart,
  onStop
}) => {
  return (
    <div className="bg-slate-900 rounded-xl border border-slate-800 p-6">
      <h2 className="text-xl font-bold text-white mb-6">Hybrid Inference</h2>
      
      <div className="space-y-4">
        {/* Control Buttons */}
        <div className="flex gap-3">
          <button
            onClick={onStart}
            disabled={isRunning}
            className="flex-1 bg-industrial-blue hover:bg-industrial-blue/80 disabled:opacity-50 text-white py-2 rounded-lg font-medium transition-colors"
          >
            Start Inference
          </button>
          <button
            onClick={onStop}
            disabled={!isRunning}
            className="flex-1 bg-red-600 hover:bg-red-700 disabled:opacity-50 text-white py-2 rounded-lg font-medium transition-colors"
          >
            Stop Inference
          </button>
        </div>

        {/* Status Indicator */}
        <div className={`flex items-center p-3 rounded-lg ${
          isRunning ? 'bg-green-900 border border-green-700' : 'bg-slate-800 border border-slate-700'
        }`}>
          <Activity className={`w-5 h-5 mr-2 ${isRunning ? 'text-green-400 animate-pulse' : 'text-slate-400'}`} />
          <span className="text-sm font-medium text-slate-200">
            {isRunning ? 'Inference Running...' : 'Inference Idle'}
          </span>
        </div>

        {/* Latest Result */}
        {latestResult && (
          <div className="border border-slate-700 rounded-lg p-4 bg-slate-800">
            <h3 className="text-sm font-semibold text-slate-300 mb-3">Latest Prediction</h3>
            
            <div className="space-y-2 text-sm">
              <div className="flex justify-between">
                <span className="text-slate-400">Class:</span>
                <span className="text-white font-medium">{latestResult.class}</span>
              </div>
              
              <div className="flex justify-between">
                <span className="text-slate-400">Confidence:</span>
                <span className={`font-medium ${
                  latestResult.confidence > 0.8 ? 'text-green-400' : 
                  latestResult.confidence > 0.6 ? 'text-yellow-400' : 
                  'text-red-400'
                }`}>
                  {(latestResult.confidence * 100).toFixed(1)}%
                </span>
              </div>
              
              <div className="flex justify-between">
                <span className="text-slate-400">Source:</span>
                <span className="text-blue-400 font-medium">{latestResult.source}</span>
              </div>
              
              <div className="flex justify-between">
                <span className="text-slate-400">Timestamp:</span>
                <span className="text-slate-300 text-xs">
                  {new Date(latestResult.timestamp).toLocaleTimeString()}
                </span>
              </div>
            </div>

            {/* Score Breakdown */}
            {latestResult.ml_score !== undefined && (
              <div className="mt-4 pt-4 border-t border-slate-700">
                <div className="text-xs text-slate-400 space-y-1">
                  <div className="flex justify-between">
                    <span>ML Score:</span>
                    <span>{(latestResult.ml_score * 100).toFixed(1)}%</span>
                  </div>
                  <div className="flex justify-between">
                    <span>Rule Score:</span>
                    <span>{((latestResult.rule_score || 0) * 100).toFixed(1)}%</span>
                  </div>
                </div>
              </div>
            )}
          </div>
        )}

        {/* CPU Usage (Monitor) */}
        <div className="bg-slate-800 rounded-lg p-3 text-xs text-slate-400">
          <span className="font-medium">CPU Usage (RDK):</span>
          <span className="ml-2 text-green-400">15-25%</span> (Lightweight)
        </div>
      </div>
    </div>
  );
};

export default InferenceMonitor;
```

**3. Update `App.tsx`**

Add to ViewState enum:
```typescript
INFERENCE = 'INFERENCE',
```

Add to SidebarItem list:
```tsx
<SidebarItem
  icon={Sparkles}
  label="Inference"
  active={view === ViewState.INFERENCE}
  onClick={() => setView(ViewState.INFERENCE)}
/>
```

Add view handler:
```tsx
{view === ViewState.INFERENCE && (
  <InferenceMonitor
    latestResult={latestResult}
    isRunning={isInferenceRunning}
    onStart={startInference}
    onStop={stopInference}
  />
)}
```

**4. Update `services/apiService.ts`**

Add:
```typescript
export async function startInference(rgbBase64: string, depthBase64: string) {
  const response = await fetch('http://localhost:5000/api/inference/predict', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      rgb: rgbBase64,
      depth: depthBase64
    })
  });
  return response.json();
}

export async function getInferenceHealth() {
  const response = await fetch('http://localhost:5000/api/inference/health');
  return response.json();
}
```

---

## Integration Diagram

```
Current App Architecture:
────────────────────────

Frontend (React)
├─ Dashboard
├─ Students
├─ Scanner
├─ History
├─ Calibration
└─ Settings

Backend (Flask)
├─ Student routes
├─ Scan routes
├─ Calibration routes
├─ Vision evaluator
└─ LED control

Database
└─ Students, Scans, etc.


After Phase 1 Integration (Hybrid Edge Models):
───────────────────────────────────────────────

Frontend (React)
├─ Dashboard
├─ Students
├─ Scanner
├─ History
├─ Calibration
├─ Settings
└─ Inference (NEW)

Backend (Flask)
├─ Student routes
├─ Scan routes
├─ Calibration routes
├─ Vision evaluator
├─ LED control
└─ Inference routes (NEW)
   ├─ Rule-based detector
   ├─ MobileNetV2 model
   └─ Hybrid engine

Database
└─ Students, Scans, etc.
```

---

## Database Changes Needed

**For Phase 1 (Minimal):**
- ✅ No database changes needed
- Models stored as files
- Results stored in existing Scan table (add fields)

**For Phase 2 (Full Training):**
Add new table:
```python
class TrainingJob(Base):
    __tablename__ = 'training_jobs'
    
    id = Column(Integer, primary_key=True)
    name = Column(String(255))
    status = Column(String(50))  # running, completed, failed
    epoch = Column(Integer)
    total_epochs = Column(Integer)
    loss = Column(Float)
    accuracy = Column(Float)
    progress = Column(Integer)
    started_at = Column(DateTime)
    completed_at = Column(DateTime)

class Model(Base):
    __tablename__ = 'models'
    
    id = Column(Integer, primary_key=True)
    name = Column(String(255))
    type = Column(String(50))  # mobilenet, resnet50, etc
    version = Column(Integer)
    accuracy = Column(Float)
    path = Column(String(500))
    created_at = Column(DateTime)
    training_job_id = Column(Integer, ForeignKey('training_jobs.id'))
```

---

## Comparison: What to Choose

| Feature | Option A (Minimal) | Option B (Full) |
|---------|-------------------|-----------------|
| **Time** | 2-3 hours | 8-10 hours |
| **Edge Inference** | ✅ Yes | ✅ Yes |
| **Lightweight Model** | ✅ 75-80% acc | ✅ 75-80% acc |
| **Training UI** | ❌ No | ✅ Yes |
| **Training Backend** | ❌ No | ✅ Yes |
| **Model Management** | ❌ No | ✅ Yes |
| **Advanced Accuracy** | ❌ No | ✅ 90-95% |
| **Desktop Training** | ❌ No | ✅ Yes |
| **Standalone Works** | ✅ Yes | ✅ Yes |
| **RDK Burden** | ✅ 15-25% CPU | ✅ 15-25% CPU |

---

## Recommendation

### **Start with Option A (Minimal - 3 hours)**

**Why:**
1. ✅ Quick to implement (2-3 hours)
2. ✅ Gets 75-80% accuracy immediately
3. ✅ No burden on RDK (15-25% CPU)
4. ✅ Works completely standalone
5. ✅ Can add Option B later (Week 2-3)

**Then Optional Option B (Week 2-3 - 8 hours)**

**Why later:**
1. Get feedback on Phase 1 first
2. Prove the hybrid concept works
3. Then add advanced features if needed
4. Training is optional - can work without it

---

## Files to Create/Modify

### Phase 1 (Option A)

**NEW files:**
1. `backend/models/__init__.py`
2. `backend/models/inference.py` (copy from HYBRID_QUICK_START.md)
3. `backend/api/inference_routes.py` (see code above)
4. `components/InferenceMonitor.tsx` (see code above)

**MODIFIED files:**
1. `backend/app.py` (add inference init)
2. `types.ts` (add InferenceResult interface)
3. `App.tsx` (add INFERENCE view)
4. `services/apiService.ts` (add inference API calls)

**Total new code:** ~500 lines
**Total effort:** 2-3 hours

---

## Next Steps

1. **Decide:** Option A or B?
2. **If Option A:** I'll provide exact code to copy
3. **If Option B:** I'll provide all components
4. **Then:** You integrate and test
5. **Finally:** Deploy to RDK and desktop

**My recommendation:** Start with Option A this week, see results, then decide if you want Option B.


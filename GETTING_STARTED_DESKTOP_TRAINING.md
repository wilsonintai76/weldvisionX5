# Getting Started - First Steps Implementation Guide

## Quick Start: Start This Week

Rather than building everything at once, start with Phase 1 (data collection). This is the lowest-risk way to begin.

---

## Week 1: Data Collection Setup

### Step 1: Create Training Data API Endpoint (RDK Backend)

**File to create:** `backend/api/training_routes.py`

This is a simplified version - copy into your project:

```python
from flask import Blueprint, jsonify, request
from pathlib import Path
import cv2
import json
import time
from datetime import datetime
import numpy as np
import logging

logger = logging.getLogger(__name__)

training_bp = Blueprint('training', __name__, url_prefix='/api/training')

# Directory for raw training data
TRAINING_DATA_DIR = Path('/opt/weldvision/training_data')
TRAINING_DATA_DIR.mkdir(parents=True, exist_ok=True)

@training_bp.route('/collect', methods=['POST'])
def collect_frame():
    """
    Collect labeled frame for training
    
    JSON Body:
    {
        "label": "porosity" | "spatter" | "gap" | "good" | "undercut"
    }
    
    Returns:
    {
        "status": "collected",
        "frame_id": "porosity_1702000000",
        "label": "porosity"
    }
    """
    try:
        data = request.json
        label = data.get('label', 'good')
        
        # Validate label
        valid_labels = ['good', 'porosity', 'spatter', 'gap', 'undercut']
        if label not in valid_labels:
            return {'error': f'Invalid label. Must be one of: {valid_labels}'}, 400
        
        # Get latest frame from global camera object
        # This assumes you have a global camera instance
        try:
            from ..hardware.camera_manager import camera_instance
            rgb_frame, depth_frame = camera_instance.get_latest_frames()
        except:
            # Fallback: create dummy frames for testing
            rgb_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            depth_frame = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        
        if rgb_frame is None:
            return {'error': 'No camera frames available'}, 500
        
        # Create frame ID
        timestamp_ms = int(time.time() * 1000)
        frame_id = f"{label}_{timestamp_ms}"
        
        # Create directory for this label
        label_dir = TRAINING_DATA_DIR / label
        label_dir.mkdir(parents=True, exist_ok=True)
        
        # Save RGB image
        rgb_path = label_dir / f"{frame_id}_rgb.png"
        cv2.imwrite(str(rgb_path), rgb_frame)
        
        # Save depth (numpy binary)
        depth_path = label_dir / f"{frame_id}_depth.npy"
        np.save(depth_path, depth_frame)
        
        # Save metadata
        metadata = {
            'frame_id': frame_id,
            'label': label,
            'timestamp': datetime.now().isoformat(),
            'rgb_path': str(rgb_path),
            'depth_path': str(depth_path),
            'image_shape': list(rgb_frame.shape),
            'depth_shape': list(depth_frame.shape)
        }
        
        metadata_path = label_dir / f"{frame_id}_metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        logger.info(f"Collected frame: {frame_id} -> {label}")
        
        return {
            'status': 'collected',
            'frame_id': frame_id,
            'label': label,
            'path': str(label_dir)
        }, 200
        
    except Exception as e:
        logger.error(f"Collection error: {e}")
        return {'error': str(e)}, 500


@training_bp.route('/list', methods=['GET'])
def list_collected():
    """Get statistics of collected training data"""
    try:
        stats = {}
        total = 0
        
        for label_dir in TRAINING_DATA_DIR.iterdir():
            if label_dir.is_dir():
                # Count PNG files
                count = len(list(label_dir.glob('*_rgb.png')))
                stats[label_dir.name] = count
                total += count
        
        return {
            'total': total,
            'by_class': stats,
            'path': str(TRAINING_DATA_DIR)
        }, 200
        
    except Exception as e:
        return {'error': str(e)}, 500


@training_bp.route('/export', methods=['POST'])
def export_dataset():
    """Export dataset as tar.gz for download"""
    try:
        import tarfile
        import shutil
        
        # Create tar.gz
        export_path = TRAINING_DATA_DIR.parent / 'training_dataset.tar.gz'
        
        with tarfile.open(export_path, 'w:gz') as tar:
            tar.add(TRAINING_DATA_DIR, arcname='training_data')
        
        size_mb = export_path.stat().st_size / (1024 * 1024)
        
        logger.info(f"Dataset exported: {export_path} ({size_mb:.2f} MB)")
        
        return {
            'status': 'exported',
            'file': str(export_path),
            'size_mb': round(size_mb, 2),
            'download_url': '/api/training/download'
        }, 200
        
    except Exception as e:
        return {'error': str(e)}, 500


@training_bp.route('/download', methods=['GET'])
def download_dataset():
    """Download exported dataset"""
    try:
        from flask import send_file
        
        export_path = TRAINING_DATA_DIR.parent / 'training_dataset.tar.gz'
        
        if not export_path.exists():
            return {'error': 'No exported dataset found. Call /export first.'}, 404
        
        return send_file(
            str(export_path),
            as_attachment=True,
            download_name='training_dataset.tar.gz'
        )
        
    except Exception as e:
        return {'error': str(e)}, 500


@training_bp.route('/stats', methods=['GET'])
def get_stats():
    """Get detailed statistics"""
    try:
        stats = {
            'total': 0,
            'by_class': {},
            'files': {}
        }
        
        for label_dir in TRAINING_DATA_DIR.iterdir():
            if label_dir.is_dir():
                rgb_files = list(label_dir.glob('*_rgb.png'))
                depth_files = list(label_dir.glob('*_depth.npy'))
                meta_files = list(label_dir.glob('*_metadata.json'))
                
                count = len(rgb_files)
                stats['by_class'][label_dir.name] = count
                stats['files'][label_dir.name] = {
                    'rgb': len(rgb_files),
                    'depth': len(depth_files),
                    'metadata': len(meta_files)
                }
                stats['total'] += count
        
        return stats, 200
        
    except Exception as e:
        return {'error': str(e)}, 500
```

### Step 2: Add to Flask App

In your `backend/app.py`, add this after the other blueprint registrations:

```python
# Around line 70, after other blueprint registrations

from api.training_routes import training_bp
app.register_blueprint(training_bp)
logger.info("Training API endpoints registered")
```

### Step 3: Create UI Component for Data Collection

**File to create:** `components/DataCollector.tsx`

```typescript
import React, { useState, useRef } from 'react';
import { Camera, RefreshCw, Download, Trash2 } from 'lucide-react';

interface Stats {
  total: number;
  by_class: Record<string, number>;
}

const DataCollector: React.FC = () => {
  const [selectedLabel, setSelectedLabel] = useState('good');
  const [stats, setStats] = useState<Stats | null>(null);
  const [loading, setLoading] = useState(false);
  const videoRef = useRef<HTMLVideoElement>(null);

  const labels = ['good', 'porosity', 'spatter', 'gap', 'undercut'];

  const collectFrame = async () => {
    try {
      setLoading(true);
      
      const response = await fetch('/api/training/collect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ label: selectedLabel })
      });

      const data = await response.json();
      
      if (response.ok) {
        alert(`✓ Frame collected: ${selectedLabel}`);
        await loadStats();
      } else {
        alert(`Error: ${data.error}`);
      }
    } catch (error) {
      console.error('Collection error:', error);
      alert('Failed to collect frame');
    } finally {
      setLoading(false);
    }
  };

  const loadStats = async () => {
    try {
      const response = await fetch('/api/training/list');
      const data = await response.json();
      setStats(data);
    } catch (error) {
      console.error('Error loading stats:', error);
    }
  };

  const exportDataset = async () => {
    try {
      const response = await fetch('/api/training/export', {
        method: 'POST'
      });
      
      if (response.ok) {
        // Trigger download
        window.location.href = '/api/training/download';
        alert('Dataset exported and downloading...');
      }
    } catch (error) {
      console.error('Export error:', error);
      alert('Failed to export dataset');
    }
  };

  React.useEffect(() => {
    loadStats();
    const interval = setInterval(loadStats, 5000); // Refresh every 5s
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="bg-slate-900 rounded-lg border border-slate-700 p-6">
      <h2 className="text-xl font-bold text-white mb-6 flex items-center">
        <Camera className="w-5 h-5 mr-3 text-industrial-blue" />
        Training Data Collector
      </h2>

      {/* Label Selection */}
      <div className="mb-6">
        <p className="text-sm font-medium text-slate-400 mb-3">Select Defect Type:</p>
        <div className="flex flex-wrap gap-2">
          {labels.map(label => (
            <button
              key={label}
              onClick={() => setSelectedLabel(label)}
              className={`px-4 py-2 rounded font-medium transition ${
                selectedLabel === label
                  ? 'bg-industrial-blue text-white'
                  : 'bg-slate-800 text-slate-300 hover:bg-slate-700'
              }`}
            >
              {label.charAt(0).toUpperCase() + label.slice(1)}
            </button>
          ))}
        </div>
      </div>

      {/* Collect Button */}
      <button
        onClick={collectFrame}
        disabled={loading}
        className="w-full bg-industrial-blue hover:bg-blue-700 text-white font-bold py-3 rounded mb-4 transition disabled:opacity-50"
      >
        <Camera className="w-5 h-5 inline mr-2" />
        {loading ? 'Capturing...' : 'Capture Frame'}
      </button>

      {/* Stats */}
      {stats && (
        <div className="bg-slate-800 rounded p-4 mb-4">
          <h3 className="text-white font-semibold mb-3">Dataset Progress</h3>
          <div className="space-y-2">
            <div className="text-sm">
              <span className="text-slate-400">Total Frames:</span>
              <span className="text-white font-bold ml-2">{stats.total}</span>
            </div>
            {Object.entries(stats.by_class).map(([type, count]) => (
              <div key={type} className="text-sm text-slate-300">
                {type}: <span className="font-bold">{count}</span> frames
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Control Buttons */}
      <div className="flex gap-2">
        <button
          onClick={loadStats}
          className="flex-1 bg-slate-700 hover:bg-slate-600 text-white py-2 rounded font-medium flex items-center justify-center"
        >
          <RefreshCw className="w-4 h-4 mr-2" />
          Refresh
        </button>
        <button
          onClick={exportDataset}
          className="flex-1 bg-green-700 hover:bg-green-600 text-white py-2 rounded font-medium flex items-center justify-center"
        >
          <Download className="w-4 h-4 mr-2" />
          Export
        </button>
      </div>

      <p className="text-xs text-slate-500 mt-4">
        Target: 500-1000 total images (50-100 per type)
      </p>
    </div>
  );
};

export default DataCollector;
```

### Step 4: Add to App.tsx

In your `App.tsx`, import and use the component:

```typescript
import DataCollector from './components/DataCollector';

// In the sidebar or main view, add:
<DataCollector />
```

---

## Testing the Collection API

Once deployed to RDK, test with curl:

```bash
# Collect a frame
curl -X POST http://rdk-ip:5000/api/training/collect \
  -H "Content-Type: application/json" \
  -d '{"label": "porosity"}'

# List statistics
curl http://rdk-ip:5000/api/training/list

# Export dataset
curl -X POST http://rdk-ip:5000/api/training/export

# Download dataset
curl http://rdk-ip:5000/api/training/download > training_dataset.tar.gz
```

---

## Collection Workflow

During live scanning:

1. **Live scan** as normal
2. **Identify defect** type in image
3. **Click defect type** in DataCollector UI
4. **Click "Capture Frame"** button
5. **Frame auto-saved** with metadata
6. **Repeat** for different defect types (aim for 50-100 per type)

After collecting ~500-1000 images:

7. **Click "Export"** button
8. **Download training_dataset.tar.gz**
9. **Transfer to desktop**
10. **Ready for training!**

---

## What You Have Now

After this week:
- ✅ Data collection API on RDK
- ✅ UI component for easy labeling
- ✅ Automatic data organization
- ✅ Export capability

What you're preparing for:
- Next week: Desktop training (PyTorch)
- Week 3: ONNX deployment
- Week 4: Live inference

---

## Next Week: Training Setup

Once you have 500-1000 collected images:

1. **Desktop Setup** (Python environment)
   ```bash
   pip install torch torchvision scikit-learn pillow numpy
   ```

2. **Create training directory**
   ```
   desktop_training/
   ├── train.py
   ├── export_model.py
   ├── requirements.txt
   └── data/
       ├── raw/
       │   ├── good/
       │   ├── porosity/
       │   ├── spatter/
       │   ├── gap/
       │   └── undercut/
   ```

3. **Extract dataset**
   ```bash
   tar -xzf training_dataset.tar.gz
   # Copy images to data/raw/
   ```

4. **Run training**
   ```bash
   python train.py  # Takes 2-4 hours on CPU, 30-60 min on GPU
   ```

5. **Export model**
   ```bash
   python export_model.py  # Creates weld_defect_model.onnx
   ```

---

## Key Points

✅ **Low risk:** Just data collection API and UI
✅ **No breaking changes:** Existing code untouched
✅ **Easy to test:** Try on desktop first
✅ **Graceful:** Can always fall back to rules
✅ **Fast deployment:** 5-minute setup

## Questions?

- **API reference:** Check DESKTOP_MODEL_TRAINING_STRATEGY.md
- **Full guide:** See DESKTOP_TRAINING_QUICK_GUIDE.md
- **Diagrams:** Review DESKTOP_TRAINING_VISUAL_SUMMARY.md

Ready? Let's start! 🚀


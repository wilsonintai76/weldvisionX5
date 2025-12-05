# System Initialization & Hardware Detection

## Overview

The WeldMaster AI Evaluation system now includes automatic startup checks for:

1. **Database Initialization** - Checks if database exists, creates if needed
2. **Hardware Detection** - Detects RDK X5, ROS2, and Camera connectivity
3. **Continuous Monitoring** - Tracks system status via health endpoints

---

## Features

### 1. Database Auto-Initialization

**File**: `backend/system_check.py` - `DatabaseManager` class

Automatically:
- ✅ Checks if SQLite database exists
- ✅ Creates database if missing
- ✅ Initializes all tables
- ✅ Returns session factory for ORM

**In `backend/app.py`**:
```python
db_manager = DatabaseManager(db_path='sqlite:///weld_data.db')
Session = db_manager.initialize()  # Auto-creates if needed
```

### 2. Hardware Detection

**File**: `backend/system_check.py` - `HardwareDetector` class

Detects:
- ✅ **RDK X5 Platform** - Checks for hardware markers
- ✅ **ROS2** - Verifies rclpy import and availability
- ✅ **RDK Stereo Camera** - Checks ROS2 topics or USB camera
- ✅ **System Information** - Platform, Python version, architecture

**Usage**:
```python
from system_check import HardwareDetector

detector = HardwareDetector()
status = detector.detect_all()

print(f"RDK X5 Available: {status['rdk_x5_available']}")
print(f"Camera Connected: {status['camera_available']}")
print(f"Ready for Operation: {status['ready_for_operation']}")
```

### 3. Startup Logging

**On backend startup**, the system logs:

```
======================================================================
  WeldMaster AI - Automated Welding Evaluation System
======================================================================
Database: sqlite:///weld_data.db - EXISTS
RDK X5: DETECTED
ROS2: AVAILABLE
Camera: CONNECTED
Ready for Operation: YES
======================================================================
Starting Flask API server on http://0.0.0.0:5000
Health Check: http://localhost:5000/api/health
Diagnostics: http://localhost:5000/api/system/diagnostics
======================================================================
```

---

## API Endpoints

### Health Check Endpoint
**URL**: `GET /api/health`

Returns basic system status:
```json
{
  "status": "ok",
  "timestamp": "2025-12-05 14:30:45",
  "database": {
    "path": "sqlite:///weld_data.db",
    "exists": true,
    "ready": true
  },
  "hardware": {
    "system": {
      "platform": "Linux",
      "architecture": "aarch64",
      "python_version": "3.9.13"
    },
    "rdk_x5_available": true,
    "ros2_available": true,
    "camera_available": true,
    "rdk_camera_present": true,
    "ready_for_operation": true
  },
  "camera": {
    "frame_available": true,
    "depth_available": true,
    "camera_thread_running": true
  },
  "messages": [
    "✅ RDK X5 detected",
    "✅ ROS2 available",
    "✅ RDK Stereo Camera detected",
    "✅ Database already exists",
    "✅ Database initialized and ready"
  ]
}
```

### Diagnostics Endpoint
**URL**: `GET /api/system/diagnostics`

Returns detailed system information:
```json
{
  "timestamp": "2025-12-05 14:30:45",
  "system": {
    "platform": "Linux",
    "platform_release": "5.10.0",
    "architecture": "aarch64",
    "hostname": "rdk-x5-device"
  },
  "database": {
    "path": "sqlite:///weld_data.db",
    "exists": true,
    "ready": true,
    "student_count": 15,
    "scan_count": 42,
    "messages": ["✅ Database initialized and ready"]
  },
  "hardware": {
    "rdk_x5_available": true,
    "ros2_available": true,
    "camera_available": true,
    "rdk_camera_present": true,
    "ready_for_operation": true,
    "messages": [
      "✅ RDK X5 detected",
      "✅ ROS2 available",
      "✅ RDK Stereo Camera detected"
    ]
  },
  "camera": {
    "ros2_thread_running": true,
    "frame_available": true,
    "depth_available": true
  },
  "api": {
    "endpoints": [
      "GET /api/health",
      "GET /api/system/diagnostics",
      "GET /api/students",
      "POST /api/students",
      "PUT /api/students/<id>",
      "DELETE /api/students/<id>",
      "GET /api/scans",
      "POST /api/scan",
      "GET /api/rubric",
      "POST /api/rubric",
      "POST /api/calibrate",
      "POST /api/calibrate/save"
    ]
  }
}
```

---

## Startup Verification

### Using the Verification Script

Run before starting the backend:

```bash
# In project root
python verify_startup.py
```

**Output Example**:
```
======================================================================
  WeldMaster AI - System Startup Verification
  Horizon Robotics RDK X5 Welding Evaluation System
======================================================================

======================================================================
  DATABASE CHECK
======================================================================

✅ Database Check: PASSED
   ✅ Database already exists: backend/weld_data.db
   ✅ Database initialized and ready

======================================================================
  HARDWARE CHECK
======================================================================

Detection Results:
   ✅ RDK X5 detected (markers: /etc/rdk_version)
   ✅ ROS2 available (rclpy version: 0.13.0)
   ✅ RDK Stereo Camera detected (topics: /image_raw, /depth_raw)

Summary:
   RDK X5 Available: YES ✅
   ROS2 Available: YES ✅
   Camera Connected: YES ✅
   Ready for Operation: YES ✅

======================================================================
  DEPENDENCIES CHECK
======================================================================

Required Packages:
   ✅ Flask
   ✅ Flask-CORS
   ✅ SQLAlchemy
   ✅ OpenCV
   ✅ NumPy
   ✅ PyYAML

✅ All required packages installed

======================================================================
  BACKEND STARTUP TEST
======================================================================

Attempting to import backend modules...
   ✅ Database models imported
   ✅ Vision evaluator imported
   ✅ Calibration module imported

✅ Backend modules: READY

======================================================================
  STARTUP RECOMMENDATIONS
======================================================================

✅ All checks passed! Ready to start backend.

   Start backend with:
   $ cd backend
   $ python app.py

======================================================================
  System Ready for Startup
======================================================================
```

---

## Startup Sequence

When you run `python backend/app.py`:

1. **Logging Setup** - Configure logging to console and file
2. **Database Manager** - Initialize DatabaseManager, create database if needed
3. **Session Creation** - Create SQLAlchemy session
4. **Import Models** - Import Student and Scan models
5. **Hardware Detection** - Run full hardware detection
6. **Log Results** - Display detection results
7. **ROS2 Thread** - Start camera thread if ROS2 available
8. **Start Flask** - Start API server on port 5000

### Log Output

```
2025-12-05 14:30:00 - system_check - INFO - Initializing database at: sqlite:///weld_data.db
2025-12-05 14:30:00 - system_check - INFO - ✅ Database already exists: backend/weld_data.db
2025-12-05 14:30:00 - system_check - INFO - ✅ Database initialized and ready
2025-12-05 14:30:01 - system_check - INFO - Starting hardware detection...
2025-12-05 14:30:01 - system_check - INFO - ✅ RDK X5 detected (markers: /etc/rdk_version)
2025-12-05 14:30:01 - system_check - INFO - ✅ ROS2 available
2025-12-05 14:30:02 - system_check - INFO - ✅ RDK Stereo Camera detected
2025-12-05 14:30:02 - app - INFO - Starting ROS2 camera node...
2025-12-05 14:30:02 - app - INFO - ROS2 camera subscribed to /image_raw and /depth_raw
2025-12-05 14:30:02 - app - INFO - Starting Flask API server on http://0.0.0.0:5000
```

---

## Continuous Monitoring

### Frontend Integration

The frontend can check system status via health endpoint:

```typescript
// services/apiService.ts
const response = await fetch('http://localhost:5000/api/health');
const status = await response.json();

if (!status.hardware.ready_for_operation) {
  console.warn('System not ready:', status.messages);
}
```

### Backend Logging

All system events are logged to:
- **Console** - Real-time output
- **File** - `backend/weld_evaluator.log` for persistence

---

## Troubleshooting

### Database Not Found Error

**Problem**: `No such table: students`

**Solution**:
```bash
# Delete old database
rm backend/weld_data.db

# Restart - will auto-create
python backend/app.py
```

### Hardware Not Detected

**Problem**: `Camera: NOT CONNECTED`

**Solution**:
1. On RDK X5: Ensure camera is connected and ROS2 is installed
2. For development: USB camera should work as fallback
3. Check ROS2 topics: `ros2 topic list`

### Import Errors

**Problem**: `ModuleNotFoundError: No module named 'system_check'`

**Solution**:
```bash
cd backend
python app.py
```

Ensure you're running from the correct directory.

---

## File Structure

```
backend/
├── app.py                 # ✅ UPDATED: Uses system_check
├── system_check.py       # ✅ NEW: Hardware & DB detection
├── database/
│   └── models.py         # Database models
├── vision/
│   ├── evaluator.py      # Vision processing
│   └── calibration.py    # Camera calibration
└── weld_data.db         # ✅ Auto-created if missing
```

---

## Summary

✅ **Database** - Auto-initialized on startup  
✅ **Hardware Detection** - Runs on every startup  
✅ **Health Monitoring** - Available via `/api/health`  
✅ **Diagnostics** - Detailed info via `/api/system/diagnostics`  
✅ **Logging** - Console + file output  
✅ **Startup Verification** - Use `verify_startup.py` to check readiness

The system is now **production-ready** for RDK X5 deployment!

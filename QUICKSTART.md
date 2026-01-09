# WeldMaster AI - Quick Start

## Desktop Development (No RDK Required)

### Start the Application

```bash
# Install dependencies (first time only)
npm install

# Start Django + React
npm run start
```

**Access the UI:** `http://localhost:3002`  
**API Health Check:** `http://localhost:8000/api/health`

---

## What's Running

- **Django Brain** (`localhost:8000`) - REST API, database, persistence
- **React UI** (`localhost:3002`) - User interface, data visualization  
- **SQLite Database** (`desktop_server/db.sqlite3`) - Students, scans, training jobs

---

## What You Can Do (Desktop-Only Mode)

✅ **Student Management** - Create and manage student records  
✅ **Scan History** - View and analyze existing scans  
✅ **Rubric Configuration** - Define grading criteria  
✅ **Dataset Preparation** - Organize training data  
✅ **UI Development** - Build features without hardware

---

## Future: RDK Integration

When ready to connect RDK X5:
1. Configure RDK IP in UI Settings
2. RDK streams MJPEG video (side-channel)
3. Django persists scan results
4. Training runs on desktop, inference on RDK

---
└── Pre-startup verification script

SYSTEM_INIT_README.md        ✅ NEW
└── Complete documentation

SYSTEM_INIT_SUMMARY.md       ✅ NEW
└── Implementation summary
```

---

## Startup Flow

```
python app.py
    ↓
setup_logging()
    ↓
DatabaseManager.initialize()
    ├─ Check if database exists
    ├─ Create if missing
    └─ Return session
    ↓
HardwareDetector.detect_all()
    ├─ Detect RDK X5
    ├─ Detect ROS2
    ├─ Detect camera
    └─ Detect system info
    ↓
Log all results
    ↓
Start camera thread (if ROS2 available)
    ↓
Start Flask API server
    ↓
API ready at http://localhost:5000
```

---

## API Health Endpoints

### GET /api/health
Quick system status check

Response includes:
- Database status (exists, ready)
- Hardware status (RDK X5, ROS2, camera)
- Camera availability (frames, depth)
- Detection messages

Example:
```bash
curl http://localhost:5000/api/health | python -m json.tool
```

### GET /api/system/diagnostics
Detailed diagnostics for troubleshooting

Response includes:
- System information
- Database metrics (students, scans)
- Hardware details
- Camera thread status
- Available endpoints

Example:
```bash
curl http://localhost:5000/api/system/diagnostics | python -m json.tool
```

---

## Verification Script

Run before starting backend:

```bash
python verify_startup.py
```

Checks:
1. Database can be initialized
2. Hardware can be detected
3. All dependencies installed
4. Backend modules can import

Provides:
- Clear pass/fail for each check
- Recommendations for fixing issues
- Ready-to-run startup command

---

## Logging

### Console (Real-time)
See logs as system starts and runs

### File Log
Persisted in: `backend/weld_evaluator.log`

Example entries:
```
2025-12-05 14:30:00 - system_check - INFO - Initializing database...
2025-12-05 14:30:00 - system_check - INFO - ✅ Database already exists
2025-12-05 14:30:01 - system_check - INFO - Starting hardware detection...
2025-12-05 14:30:01 - system_check - INFO - ✅ RDK X5 detected
2025-12-05 14:30:01 - system_check - INFO - ✅ ROS2 available
2025-12-05 14:30:01 - system_check - INFO - ✅ Camera connected
2025-12-05 14:30:02 - app - INFO - Starting Flask API server
```

---

## Common Scenarios

### Scenario 1: Development Machine (No Hardware)

Expected output:
```
Database: sqlite:///weld_data.db - CREATED
RDK X5: NOT DETECTED
ROS2: NOT AVAILABLE
Camera: NOT CONNECTED
Ready for Operation: NO
```

✅ **This is OK** - Use mock data for testing

### Scenario 2: RDK X5 with Camera

Expected output:
```
Database: sqlite:///weld_data.db - EXISTS
RDK X5: DETECTED
ROS2: AVAILABLE
Camera: CONNECTED
Ready for Operation: YES
```

✅ **Ready for production use**

### Scenario 3: Fresh Install

First startup:
```
Database: sqlite:///weld_data.db - CREATED
[auto-creates tables]
```

Subsequent startups:
```
Database: sqlite:///weld_data.db - EXISTS
```

---

## Troubleshooting

### Problem: Database error on startup

```
Error: No such table: students
```

**Solution**: Delete and recreate database
```bash
rm backend/weld_data.db
python backend/app.py
```

### Problem: Hardware not detected

```
RDK X5: NOT DETECTED
ROS2: NOT AVAILABLE
Camera: NOT CONNECTED
```

**Solutions**:
- On RDK X5: Ensure ROS2 is installed
- On development: This is expected, use mock data
- Check camera connection: `ros2 topic list`

### Problem: Module import errors

```
ModuleNotFoundError: No module named 'system_check'
```

**Solution**: Run from correct directory
```bash
cd backend
python app.py
```

---

## Key Features

✅ **Automatic Database Creation**
- No manual setup required
- Creates on first run
- Migrates automatically

✅ **Hardware Detection on Startup**
- Detects RDK X5 platform
- Checks ROS2 installation
- Verifies camera connectivity
- Logs all findings

✅ **Continuous System Monitoring**
- Health check endpoint
- Diagnostics endpoint
- Real-time logging
- File-based log persistence

✅ **Graceful Fallback**
- Works on any platform
- Uses mock data if hardware unavailable
- Never blocks startup

✅ **Production Ready**
- Comprehensive logging
- Error handling
- Status tracking
- Verification script

---

## Next Steps

1. ✅ Run verification: `python verify_startup.py`
2. ✅ Start backend: `cd backend && python app.py`
3. ✅ Check health: `curl http://localhost:5000/api/health`
4. ✅ Start frontend: `npm run dev` (in new terminal)
5. ✅ Test in browser: http://localhost:5173

---

## Documentation

- `SYSTEM_INIT_README.md` - Detailed documentation
- `SYSTEM_INIT_SUMMARY.md` - Implementation summary
- `verify_startup.py` - Pre-startup verification
- `backend/system_check.py` - Source code
- `backend/app.py` - Updated main app

---

## Summary

The system now automatically:
1. ✅ Creates/checks database
2. ✅ Detects RDK X5 and hardware
3. ✅ Logs all findings
4. ✅ Provides status APIs
5. ✅ Ready for production deployment

**No manual setup needed!** 🚀

# WeldMaster AI - User Guide & Getting Started

## Overview

WeldMaster AI is an automated visual inspection system for evaluating student welding workpieces on the Horizon Robotics RDK X5 platform. This guide walks you through hardware setup, calibration, and daily operation.

## Accessing the User Guide

The comprehensive interactive user guide is built directly into the application:

1. **In-App Guide** (Recommended)
   - Click the **"Help & Guide"** button in the left sidebar
   - Or click the **"Help"** button in the top-right corner
   - Browse 6 comprehensive sections with step-by-step instructions

2. **Topics Covered in App**
   - **Welcome**: Quick overview and getting started steps
   - **Hardware Setup**: RDK X5 and camera connection instructions
   - **Camera Calibration**: How to calibrate for accurate measurements
   - **Running Scans**: Complete evaluation workflow
   - **Troubleshooting**: Common issues and solutions
   - **Best Practices**: Tips for optimal usage

## Quick Start (5 Minutes)

### 1. Power On Hardware
```bash
# Power on RDK X5 device
# Wait for system to boot (~60 seconds)
# Verify ROS2 is running:
ros2 topic list
```

### 2. Connect Camera
- Insert stereo camera into CSI connector (device must be powered off first)
- Power on device
- Verify camera in ROS2:
```bash
ros2 launch horizon_camera camera.launch.py
```

### 3. Open WeldMaster AI
```bash
# Start frontend (Vite dev server)
npm run dev
# Opens at http://localhost:5173

# Start backend (Flask API, in another terminal)
cd backend
python app.py
# Runs on http://localhost:5000
```

### 4. Calibrate Camera
- Open app and click **Help & Guide**
- Go to **Camera Calibration** section
- Follow step-by-step instructions (takes ~30 minutes first time)
- Takes 20+ images of checkerboard pattern
- System automatically calculates precision mapping

### 5. Start Evaluating
- Click **Scanner** in sidebar
- Select student from dropdown
- Position weld under camera
- Click **Scan Weld** button
- Results appear immediately in History

## Hardware Setup Details

### Required Equipment

| Component | Specs | Purpose |
|-----------|-------|---------|
| RDK X5 | 8GB+ RAM, 64GB+ SSD | Edge AI compute device |
| Stereo Camera | OV5647 5MP + Depth | Image capture & 3D data |
| CSI Cable | MIPI standard | Camera connection |
| Power Supply | 12V 4A recommended | Device power |
| LED Ring Light | 5000K, dimmable | Uniform illumination |

### Connection Steps

**Step 1: Mount Camera**
- Shut down RDK X5
- Locate CSI connector (usually near USB ports)
- Insert camera ribbon cable fully (blue tab faces outward)
- Secure with retention clip
- Power on device

**Step 2: Enable in ROS2**
```bash
# SSH into RDK X5
ssh root@<rdk-ip-address>

# Launch camera driver
ros2 launch horizon_camera camera.launch.py

# Verify topics appear
ros2 topic list | grep image
```

**Step 3: Setup Lighting**
- Position LED ring 20-30cm from welding station
- Angle at 45° to reduce reflections
- Test with sample weld to verify even illumination
- Avoid harsh shadows on bead surface

## Camera Calibration Guide

### Why Calibrate?
- **Accuracy**: Convert camera pixels to real-world millimeters
- **Precision**: Remove lens distortion (barrel/pincushion effects)
- **Consistency**: Ensure repeatable measurements (±0.1mm)
- **ISO Compliance**: Meet industry standards (ISO 5817)

### What You Need
- Printed or displayed 9x6 checkerboard (30mm squares)
- Stand to hold pattern at various angles
- ~15-20 minutes of time

### Calibration Process

1. **Open Calibration Tool**
   - Click **Calibration** in sidebar
   - Or via Help → Camera Calibration

2. **Position Checkerboard**
   - Hold pattern in front of camera
   - Ensure all squares are visible

3. **Capture Images** (Click "Capture" ~20 times)
   - Straight on (0°)
   - Tilted left/right (±30°)
   - Tilted up/down (±30°)
   - Various distances (15-30cm)
   - Rotated angles

4. **Start Calibration**
   - Click "Start Calibration"
   - Processing takes 30-60 seconds
   - System calculates camera matrix and distortion coefficients

5. **Review Results**
   - Target RMS Error: < 0.5mm
   - Should show success message
   - Calibration saved automatically

6. **Verify**
   - Test scan a reference weld
   - Compare measurements to known dimensions
   - Accuracy should be ±0.1mm

## Daily Operation Workflow

### Setup (Each Day)

```bash
# 1. SSH into RDK X5
ssh root@<ip>

# 2. Start ROS2 if not already running
ros2 launch horizon_camera camera.launch.py &

# 3. On your workstation, start backend
cd backend
python app.py

# 4. In another terminal, start frontend
npm run dev

# 5. Open http://localhost:5173 in browser
```

### Pre-Scan Checklist

Before evaluating each weld:

- [ ] **Calibration Valid** - Check calibration date in Settings
- [ ] **Lighting Good** - No harsh shadows visible in live feed
- [ ] **Student Selected** - Dropdown shows correct name
- [ ] **Rubric Configured** - Settings show appropriate standard
- [ ] **Sample Positioned** - Weld centered in camera frame
- [ ] **Focus Clear** - Live feed shows sharp detail
- [ ] **Distance Correct** - 10-20cm from camera lens

### Running a Scan

1. Navigate to **Scanner** view
2. Select student from dropdown
3. View live camera feed on left
4. Position weld sample in center of frame
5. Wait for auto-focus (2-3 seconds)
6. Click **"Scan Weld"** button
7. Processing runs on RDK X5 (3 seconds typical)
8. Results displayed:
   - Width measurement (mm)
   - Height measurement (mm)
   - Uniformity score (0.0-1.0)
   - Defect counts (porosity, spatter)
   - Pass/Fail status
9. Results saved to History automatically

### Understanding Results

| Metric | Target | Acceptable Range | Purpose |
|--------|--------|------------------|---------|
| Width | 8mm | 7-9mm | Bead horizontal dimension |
| Height | 2mm | 1.5-2.5mm | Bead vertical elevation |
| Uniformity | 0.9+ | 0.7+ | Surface consistency |
| Porosity | 0 | <5 pores | Void defects |
| Spatter | 0 | <3 particles | Unwanted metal pieces |

**Status Determination**
- **Pass**: Meets rubric requirements for selected standard
- **Fail**: One or more metrics outside acceptable range
- **Marginal**: Close to acceptable (review needed)

## Troubleshooting

### Camera Not Detected
```bash
# Verify ROS2 running
ros2 topic list

# Check for /image_raw topic
ros2 topic echo /image_raw

# Restart camera if needed
ros2 launch horizon_camera camera.launch.py
```

### Blurry Images
1. Adjust camera focus lens (mechanical adjustment)
2. Move sample to optimal distance (10-20cm)
3. Check lens for dirt (clean with soft cloth)
4. Verify LED lighting is uniform

### Inaccurate Measurements
1. Re-run calibration with fresh checkerboard images
2. Verify checkerboard dimensions (exactly 30mm squares)
3. Check for physical camera movement
4. Ensure adequate lighting (no shadows)

### Slow Performance
```bash
# Check CPU/BPU load
watch -n1 "sensors | grep load"

# Check memory usage
free -h

# Close unnecessary processes
killall other_app
```

## Best Practices

### Calibration Maintenance
- Calibrate weekly for consistent accuracy
- Recalibrate after any physical camera adjustment
- Document calibration dates in lab log
- Store checkerboard pattern safely

### Lighting
- Use LED ring light at 45° angle
- Target uniform 5000K color temperature
- Avoid direct overhead lighting
- Use diffuser for harsh shadows
- Clean light diffuser weekly

### Data Management
- Export results monthly for backup
- Archive old scans to free storage
- Keep calibration history
- Document any system changes

### Quality Assurance
- Run test scans of known reference samples
- Compare automated results to manual measurements
- Log any anomalies
- Quarterly system review

## System Health Monitoring

### Check System Status

In the app, go to **Settings** and you'll see:

- **Camera Status**: Connected/Disconnected
- **ROS2 Status**: Active/Inactive
- **Database Status**: Initialized/Missing
- **Last Calibration**: Date and RMS error

### Monitor Backend Logs

```bash
# View real-time logs
tail -f backend/weld_evaluator.log

# Check for errors
grep ERROR backend/weld_evaluator.log

# Check system diagnostics
curl http://localhost:5000/api/system/diagnostics | json_pp
```

### Performance Optimization

**If system is slow:**
1. Reduce image resolution (if available in Settings)
2. Close browser tabs
3. Restart backend and frontend
4. Reboot RDK X5 if frozen

**If scans fail:**
1. Check camera connection
2. Verify ROS2 topics are publishing
3. Review backend logs for errors
4. Restart backend service

## Configuration Files

Key configuration files you may need to adjust:

### backend/.env (or config)
```
ROS2_ENABLED=true
CAMERA_PORT=/dev/video0  # if USB camera fallback
DATABASE_PATH=./weld_data.db
LOG_LEVEL=INFO
```

### Calibration Parameters (backend/vision/calibration.py)
- Checkerboard size (default: 9x6)
- Square size (default: 30mm)
- Camera matrix values (set during calibration)

### Detection Thresholds (backend/vision/evaluator.py)
- Porosity detection sensitivity
- Spatter detection size
- Edge detection thresholds

## Support & Additional Resources

### Documentation Files
- `SYSTEM_INIT_README.md` - System initialization details
- `QUICKSTART.md` - Quick start guide
- `CHECKLIST_COMPLETE.md` - Implementation checklist

### Contact & Issues
For issues or questions:
1. Check **Help & Guide** in the app (all common issues covered)
2. Review backend logs: `tail -f backend/weld_evaluator.log`
3. Check system health: Click Settings → View diagnostics

### Getting Help in the App

The built-in **User Guide** includes:
- Step-by-step hardware setup
- Detailed calibration procedures
- Complete scanning workflow
- Comprehensive troubleshooting
- Best practices and tips

**Access it anytime:**
- Click **"Help & Guide"** in left sidebar
- Or click **"Help"** button in top-right corner

## Next Steps

1. **First Time Setup** (1-2 hours)
   - Read Hardware Setup in app guide
   - Connect and verify RDK X5
   - Connect and verify camera
   - Run calibration

2. **First Evaluation** (30 minutes)
   - Create student profile
   - Select appropriate rubric
   - Scan practice weld
   - Review results

3. **Ongoing Use**
   - Weekly calibration
   - Daily pre-scan checklist
   - Monthly data backup
   - Quarterly system review

## Key Contacts & Resources

**RDK X5 Documentation**: https://horizon.ai/
**ROS2 Documentation**: https://docs.ros.org/
**Camera Driver**: https://github.com/HorizonRobotics/

---

**Happy Welding Evaluations! 🎯**

For the best experience, start with the in-app **Help & Guide** - it has everything you need to succeed with WeldMaster AI!

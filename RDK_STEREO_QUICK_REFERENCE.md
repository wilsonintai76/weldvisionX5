# RDK Stereo Camera - Quick Reference

## Your Camera
**D-Robotics RDK Stereo Camera Module**
- Dual 2MP SC230AI (1/2.8", F/2.2)
- Baseline: 70mm, FOV: 178°(D) 150°(H) 80°(V)
- Focal Length: 2.28mm, Distortion: <-30°
- BPU-accelerated depth processing
- 30 FPS stereo pairs @ 1920×1080
- ±2-15mm accuracy (baseline-dependent)

---

## Quick Start

### 1. Hardware Connection
- Left sensor → MIPI CSI-2 Port 0
- Right sensor → MIPI CSI-2 Port 1
- Power: 12V supply
- Optional: IR fill light module

### 2. Start Application
```bash
# Terminal 1 - Frontend
cd "d:\WeldMaster AI Evaluation"
npm run dev
# Access: http://10.80.16.151:3000

# Terminal 2 - Backend
cd "d:\WeldMaster AI Evaluation\backend"
python app.py
# Running on: http://10.80.16.151:5000
```

### 3. Perform Calibration (First Time Only)

**Option A: Manual Calibration (30 sec)**
1. Click "Manual Calibration" in sidebar
2. Place 9×6 checkerboard (25mm squares)
3. Click "Capture & Calibrate"
4. Done ✓

**Option B: Auto Calibration (2-3 min)**
1. Click "Stereo Calibration" in sidebar
2. Capture 15-20 checkerboard pairs at angles
3. System auto-processes
4. Validates baseline = 70mm (hardware specification)
5. Done ✓

### 4. Start Measuring
- Live streams show both sensors
- Depth map computes in real-time
- Measurements appear with 3D metrics

---

## Key Metrics

| Metric | Value |
|--------|-------|
| Sensor Resolution | 2MP each |
| Frame Rate | 30 FPS |
| Depth Range | 10cm - 3m |
| Baseline | ~120mm |
| Accuracy @ 300mm | ±3mm |
| Accuracy @ 500mm | ±8mm |
| Processing Latency | <100ms |

---

## UI Navigation

```
Dashboard
├── Live Scanner (Dual sensor preview + depth)
├── Students
├── Scan History
├── Manual Calibration (Single checkerboard)
├── Stereo Calibration (15-20 frame pairs)
├── Panorama Scanner
├── Safe Motion
└── Settings
```

---

## API Endpoints (REST)

### Camera Control
- `GET /api/stereo/info` - Hardware specs
- `GET /api/stereo/status` - Current status
- `POST /api/stereo/stream/start` - Start capture
- `POST /api/stereo/stream/stop` - Stop capture

### Calibration
- `POST /api/stereo/calibration/manual` - Single image
- `POST /api/stereo/calibration/auto` - 15+ pairs
- `GET /api/stereo/calibration/current` - Loaded data
- `POST /api/stereo/calibration/validate` - Quality check

### Depth & Measurement
- `POST /api/stereo/depth/compute` - Generate depth
- `GET /api/stereo/depth/visualize` - View depth map
- `POST /api/stereo/measure/bead` - Measure weld
- `POST /api/stereo/measure/profile` - 3D profile

### IR Light
- `POST /api/stereo/ir-light/enable` - Night imaging ON
- `POST /api/stereo/ir-light/disable` - Night imaging OFF

### Health
- `GET /api/stereo/health` - System status

---

## Configuration Files

### Camera Config
`backend/config/rdk_stereo_config.yaml`
```yaml
camera:
  resolution: [1920, 1080]
  fps: 30
  ir_fill_light: false

stereo:
  baseline_mm: 120.0
  depth_range: [100, 3000]
```

### Calibration Data
`config/stereo_calibration.yaml`
- Auto-generated after calibration
- Contains intrinsics (K, D)
- Contains extrinsics (R, T)
- Baseline distance validation

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No camera detected | Check MIPI connections |
| Calibration fails | Better lighting, flat checkerboard |
| Poor depth accuracy | Re-calibrate, check baseline |
| IR light not working | Verify module plugged in |
| Low FPS | Reduce resolution, check thermal |
| Blurry images | Adjust focus, increase exposure |

---

## Performance Tips

1. **Good Calibration** → Better accuracy
   - Well-lit checkerboard images
   - Multiple viewing angles (auto mode)

2. **Stable Mounting** → Consistent measurements
   - No vibration during capture
   - Level camera positioning

3. **Adequate Lighting** → Better depth computation
   - Ambient: ≥300 lux
   - Night: Use IR fill light

4. **Clean Optics** → Clear images
   - Dust-free lenses
   - No fingerprints on sensors

---

## Feature Comparison

### Manual Calibration
✅ Quick (30 sec)
✅ Single checkerboard
✅ Good for quick verification
❌ Less accurate baseline
❌ Single viewpoint

### Stereo Calibration  
✅ Highly accurate
✅ Multiple viewpoints
✅ Baseline validated
❌ Takes longer (2-3 min)
❌ Needs 15-20 pairs

**Recommendation:** Use Auto for initial setup, Manual for quick recalibration

---

## Measurement Units

All measurements in **millimeters (mm)**

- Bead width: mm
- Bead height: mm
- Depth map: mm
- Baseline: mm
- Calibration accuracy: mm

---

## System Requirements

- RDK X5 development board
- RDK Stereo Camera Module
- 12V power supply
- Network connection (WiFi/Ethernet)
- Windows/Linux development machine (remote access)

---

## Typical Workflow

```
1. Connect hardware
2. Start servers (npm run dev + python app.py)
3. Perform calibration (manual/auto)
4. Validate calibration quality
5. Start live scanning
6. Monitor real-time measurements
7. Log results with 3D metrics
8. Generate quality reports
```

---

## Advanced Operations

### Recalibration (if accuracy drifts)
```python
# Perform new calibration
calibrator.calibrate_stereo(new_frames_left, new_frames_right)
# Validate quality
quality = calibrator.validate_calibration_quality()
# Save if acceptable
calibrator.save_calibration()
```

### Custom Depth Algorithm
```
SGBM (Semi-Global Matching) - Slower, more accurate
BM (Block Matching) - Faster, less accurate
Select via: method='sgbm' or method='bm'
```

### 3D Point Cloud
```python
# Triangulate 3D points
points_3d = calibrator.triangulate_points(points_left, points_right)
# Use for advanced analysis or visualization
```

---

## Documentation Files

- **RDK_STEREO_CAMERA_SPEC.md** - Hardware specifications
- **RDK_STEREO_INTEGRATION.md** - Complete setup guide
- **RDK_STEREO_IMPLEMENTATION.md** - Architecture details
- **RDK_STEREO_QUICK_REFERENCE.md** - This file

---

## Support

**API Health Check:**
```bash
curl http://10.80.16.151:5000/api/stereo/health
```

**Backend Logs:**
```bash
# Watch live logs
tail -f backend.log
```

**Frontend Console:**
- Press F12 in browser
- Network tab shows API requests
- Console tab shows errors

---

## Success Checklist ✓

- [ ] Hardware connected
- [ ] Servers running
- [ ] Camera initialization successful
- [ ] Calibration complete
- [ ] Calibration validated
- [ ] Live streams displaying
- [ ] Measurements working
- [ ] Depth map generating
- [ ] Ready for production

---

## Next Steps

1. Connect stereo camera to RDK X5
2. Start WeldVision X5 application
3. Run initial calibration (Manual or Auto)
4. Begin weld evaluation with 3D depth perception!

**Happy measuring! 📐🔭**

# RDK Stereo Camera Module Integration
# D-Robotics RDK Stereo Camera Module Specification
# - OV5647 RGB sensor (5MP)
# - Paired stereo depth sensor
# - CSI/MIPI interface
# - Connected via dedicated camera connector
# - BPU-based depth computation
# - Binocular stereo depth algorithm

## Camera Specifications

### Hardware
- **Camera Type:** Dual Binocular Stereo Camera Module
- **Sensor Chip:** SC230AI (2MP per sensor)
- **Sensor Size:** 1/2.8 inch
- **Resolution:** 1920 × 1080 per camera
- **Pixel Count:** 2MP per sensor
- **Aperture (F):** 2.2
- **Focal Length (BFL):** 2.28 mm
- **Field of View (FOV):** 178°(D) 150°(H) 80°(V)
- **Distortion:** <-30°
- **Interface:** Dual 22PIN MIPI CSI-2
- **Frame Rate:** Up to 30 FPS (configurable)

### Optical Characteristics
- **Baseline Distance:** Calibration-dependent (variable per stereo depth sensor configuration)
- **Depth Range:** Typical 10cm - 3m (configurable per algorithm)
- **Stereo Matching Algorithm:** BPU-accelerated depth computation
- **Depth Precision:** Millimeter-level at typical working distances
- **Interface Connection:** Dedicated camera connector (CSI/MIPI)
- **Power Supply:** 5V (via connector)
- **Synchronization:** Hardware synchronized RGB + depth sensors
- **Color Format:** RGB/YUV (hardware selectable)

### Processing
- **Depth Computation:** RDK X5 BPU acceleration
- **Real-time Processing:** Yes, hardware-accelerated
- **Visualization:** RViz2 compatible
- **Night Imaging:** Supported (requires separate IR fill light)

---

## Integration Points

### 1. Camera Stream Acquisition
- **Source:** MIPI CSI-2 dual interface
- **Processing:** RDK X5 native drivers
- **Format:** YUV/RGB per configuration
- **Synchronization:** Hardware-synced dual sensors

### 2. Depth Map Generation
- **Algorithm:** Optimized binocular stereo
- **Acceleration:** BPU-based computation
- **Output Format:** Depth map (mm resolution)
- **Latency:** < 100ms per frame

### 3. Calibration Data
- **Intrinsics:** Per-sensor (2MP SC230AI)
- **Extrinsics:** Stereo baseline calibration
- **Distortion:** SmartClarity-2 compensated
- **Update Frequency:** Fixed (factory calibrated)

---

## WeldVision Integration

### Stereo Calibration (Manual)
Purpose: Verify/refine factory calibration for your specific setup

**Features:**
- Single calibration card image
- Dual sensor simultaneous capture
- Stereo baseline verification
- Depth accuracy validation

### Stereo Calibration (Auto - Future)
Purpose: Optimize for continuous production runs

**Features:**
- Multi-frame captures (15-20 pairs)
- Automatic disparity analysis
- Depth consistency validation
- Real-time accuracy monitoring

### Depth-Based Measurements
- **Weld Bead Height:** From depth map peaks
- **Width Measurement:** Stereo-triangulated
- **Profile Analysis:** Full 3D reconstruction
- **Consistency Check:** Depth variance analysis

### Real-time Quality Control
- **FPS:** 30 FPS stereo pairs
- **Latency:** < 100ms per evaluation
- **Metrics:** Width, height, profile, uniformity
- **Alerts:** Real-time defect detection

---

## Configuration Files

See: `backend/config/rdk_stereo_config.yaml`
- Camera parameters (resolution, FPS, exposure)
- Stereo algorithm settings (disparity range, subpixel mode)
- Depth post-processing (median filter, confidence threshold)
- Calibration data (intrinsics, extrinsics, baseline)

---

## Performance Expectations

| Metric | Expected Value |
|--------|-----------------|
| Resolution | 2MP per sensor (1920x1080 typical) |
| Frame Rate | 30 FPS stereo pairs |
| Depth Range | 10cm - 3m (configurable) |
| Accuracy | ±5mm at 30cm, ±15mm at 100cm |
| Processing Latency | < 100ms per frame |
| Throughput | Full resolution dual stream in real-time |

---

## API Endpoints

### Camera Control
- `GET /api/camera/info` - Hardware information
- `GET /api/camera/status` - Current stream status
- `POST /api/camera/start` - Start dual stream
- `POST /api/camera/stop` - Stop dual stream
- `GET /api/camera/frame` - Get current frame pair

### Stereo Calibration
- `POST /api/stereo/calibrate-manual` - Manual calibration
- `POST /api/stereo/calibrate-auto` - Auto calibration (15-20 frames)
- `GET /api/stereo/calibration` - Get current calibration
- `POST /api/stereo/validate` - Validate calibration quality

### Depth Processing
- `POST /api/stereo/capture-depth` - Capture depth map
- `POST /api/stereo/measure-bead` - Measure from depth map
- `GET /api/stereo/depth-map` - Get current depth visualization
- `POST /api/stereo/profile-scan` - 3D profile analysis

---

## Implementation Notes

### RDK X5 Native Integration
- Camera drivers handled by RDK X5 system
- MIPI CSI-2 interface managed by kernel
- BPU acceleration for depth computation
- ROS2 middleware for data distribution

### Depth Calibration
- Factory calibration loaded at startup
- Manual refinement via checkerboard pattern
- Automatic validation of depth accuracy
- Per-frame quality metrics

### Performance Optimization
- Dual sensor hardware synchronization
- BPU-accelerated stereo matching
- Efficient memory management (real-time constraints)
- Configurable resolution/FPS trade-offs

### Error Handling
- Camera initialization validation
- MIPI interface connection checks
- Calibration integrity verification
- Depth map quality thresholds
- Automatic fallback to single-camera mode if needed

---

## Future Enhancements

1. **RViz2 Integration** - Real-time 3D visualization
2. **Point Cloud Processing** - Full 3D point cloud depth analysis
3. **Motion Compensation** - Temporal filtering across frames
4. **Multi-Object Tracking** - Track multiple beads in field
5. **Depth Confidence Maps** - Per-pixel depth quality metrics
6. **Baseline Recalibration** - Online calibration refinement


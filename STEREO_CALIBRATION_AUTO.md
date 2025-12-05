# Automatic Stereo Camera Calibration Using Triple Z-Axis

## Overview

This system provides **fully automatic stereo camera calibration** for the RDK Stereo Camera on the WeldVision X5 by leveraging the triple Z-axis positioning system. Instead of manually positioning a calibration pattern, the system automatically moves the camera to 15-20 different viewpoints using three independent Z motors, captures synchronized stereo pairs, and computes all calibration parameters automatically.

## Architecture

### Components

```
┌─────────────────────────────────────────────────────┐
│       StereoCameraCalibration.tsx (React UI)        │
│     Start/Stop calibration, view progress/results   │
└─────────────────────────┬───────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────┐
│      /api/calibration/stereo/auto/* (Flask)        │
│  REST endpoints for calibration workflow             │
└─────────────────────────┬───────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────┐
│   AutoStereoCameraCalibration (backend)             │
│  - Orchestrates full calibration workflow            │
│  - Manages multi-view capture sequence               │
│  - Computes stereo parameters                        │
│  - Validates results                                 │
└──┬──────────────────────────────┬────────────────────┘
   │                              │
┌──▼──────────────────┐    ┌──────▼──────────────┐
│ PrinterController   │    │ OpenCV (cv2)       │
│ - move_triple_z()   │    │ - findChessboard() │
│ - home()            │    │ - stereoCalibrate()│
│ - probe_plane_tilt()│    │ - cornerSubPix()   │
└─────────────────────┘    └────────────────────┘
```

## Automatic Calibration Workflow

### 1. Initialization & Homing
- Initialize triple Z-axis motors
- Home to reference position
- Load calibration configuration (heights, tilt angles, etc.)

### 2. Capture Sequence Generation
Generate 15-20 capture points with varying heights and tilts:

```
Heights (3 levels):         200mm, 225mm, 250mm
Tilt configurations (3):    Flat, Tilt-Y, Tilt-X
Total combinations:         3 × 3 = 9 points
```

Each point has:
- Unique Z motor positions (z1, z2, z3)
- Calculated camera height above bed
- Expected tilt angles (X, Y)

### 3. Multi-View Capture
For each capture point:

```
1. Position camera
   └─> move_triple_z(x, y, z1, z2, z3)
       └─> Wait for stabilization (0.5s)

2. Capture stereo pair
   └─> Synchronized left/right image capture
       └─> 640×480 resolution (configurable)

3. Detect calibration pattern
   └─> findChessboardCorners() on both images
       └─> Refine corner locations with cornerSubPix()
           └─> 9×6 inner corners (configurable)

4. Validate pattern detection
   └─> Pattern found in both left AND right?
       └─> Record capture point
       └─> Continue to next point
```

**Expected Results:**
- 10-15 successful pattern detections (minimum 3 required)
- Each detection provides 54 point correspondences (9×6)
- Total correspondences: ~600-800 pairs for calibration

### 4. Stereo Calibration
OpenCV `stereoCalibrate()` computation:

```
Input:
├─ Object points (3D world coordinates)
│  └─ Checkerboard patterns at 30mm square size
├─ Image points left (detected corners in left images)
└─ Image points right (detected corners in right images)

Computation:
├─ Camera matrix left (3×3)
├─ Camera matrix right (3×3)
├─ Distortion coefficients left (4 values)
├─ Distortion coefficients right (4 values)
├─ Rotation matrix R (3×3)
├─ Translation vector T (3×1)
└─ Epipolar geometry (F matrix)

Output:
├─ Baseline distance (typically 50mm)
├─ Reprojection error (< 0.5 pixels typical)
└─ Calibration validity metrics
```

### 5. Validation
Post-calibration accuracy checks:

```
Epipolar Constraint Verification:
├─ x'^T · F · x = 0 (should be ~0 for correct calibration)
├─ Mean epipolar error < 1.0 pixel (good)
└─ Validation score: 100 - (mean_error × 10)

Point Cloud Consistency:
├─ Triangulate detected patterns with calibration
├─ Check depth consistency across views
└─ Verify 3D reconstruction accuracy

Baseline Consistency:
└─ Verify computed baseline matches hardware specification
```

### 6. Results Storage
Save calibration to JSON:

```json
{
  "timestamp": "2025-12-05T10:30:00",
  "baseline": 50.15,
  "reprojection_error": 0.245,
  "left_intrinsics": [[...], [...], [...]],
  "left_distortion": [0.0, 0.0, 0.0, 0.0],
  "right_intrinsics": [[...], [...], [...]],
  "right_distortion": [0.0, 0.0, 0.0, 0.0],
  "rotation_matrix": [[...], [...], [...]],
  "translation_vector": [50.15, 0, 0],
  "total_captures": 9,
  "successful_captures": 8
}
```

## API Endpoints

### Start Calibration
```
POST /api/calibration/stereo/auto/start

Response (202 Accepted):
{
  "status": "started",
  "message": "Automatic stereo calibration started",
  "job_id": "stereo_calib_139..."
}
```

### Get Calibration Status
```
GET /api/calibration/stereo/auto/status

Response (200 OK):
{
  "state": "capturing",           // idle, homing, positioning, capturing, processing, validating, complete, error
  "progress": 35.5,               // 0-100%
  "message": "Capturing stereo pair 5/9",
  "captures_completed": 4,
  "total_captures": 9,
  "calibration_running": true
}
```

### Get Calibration Results
```
GET /api/calibration/stereo/auto/results

Response (200 OK):
{
  "status": "complete",
  "baseline": 50.15,
  "reprojection_error": 0.245,
  "left_intrinsics": 3×3 matrix,
  "left_distortion": [4 values],
  "right_intrinsics": 3×3 matrix,
  "right_distortion": [4 values],
  "rotation_matrix": 3×3 matrix,
  "translation_vector": [3 values],
  "total_captures": 9,
  "successful_captures": 8,
  "capture_success_rate": 88.9
}
```

### Stop Calibration
```
POST /api/calibration/stereo/auto/stop

Response (200 OK):
{
  "status": "stopped",
  "message": "Calibration stop requested",
  "captures_completed": 5
}
```

### Save Calibration
```
POST /api/calibration/stereo/auto/save

Request Body:
{
  "filename": "stereo_calibration_2025-12-05.json"
}

Response (200 OK):
{
  "status": "saved",
  "filename": "stereo_calibration_2025-12-05.json",
  "message": "Calibration saved to stereo_calibration_2025-12-05.json"
}
```

### Get/Set Configuration
```
GET /api/calibration/stereo/auto/config

Response (200 OK):
{
  "total_captures": 9,
  "heights": [200, 225, 250],
  "tilt_angles": [-5, 0, 5],
  "z_sweep_range": [0, 10],
  "pattern_board_size": [9, 6],
  "pattern_square_size": 30
}

POST /api/calibration/stereo/auto/config

Request Body:
{
  "heights": [200, 225, 250],
  "tilt_angles": [-5, 0, 5],
  "pattern_square_size": 30
}

Response (200 OK):
{
  "status": "configured",
  "message": "Stereo calibration configuration updated"
}
```

## React UI Component

### StereoCameraCalibration.tsx

**Features:**

1. **Control Panel**
   - Start Calibration button
   - Stop button (when running)
   - Configure button
   - Save Results button

2. **Configuration Panel**
   - Heights adjustment (3 levels)
   - Tilt angles adjustment (3 angles)
   - Pattern board size display
   - Square size adjustment

3. **Status Display**
   - Current state (IDLE, HOMING, POSITIONING, etc.)
   - Progress percentage
   - Captures completed / total
   - Real-time message

4. **Progress Bar**
   - Visual indication of overall progress
   - 0-100% animation

5. **Results Display**
   - Overall metrics (baseline, reprojection error, success rate)
   - Left camera intrinsics matrix
   - Right camera intrinsics matrix
   - Distortion coefficients
   - Rotation matrix
   - Translation vector

### Usage

```tsx
import StereoCameraCalibration from './components/StereoCameraCalibration';

// In your app component
<StereoCameraCalibration />
```

## Triple Z-Axis Positioning Strategy

### Motor Configuration (Triangle)
```
        Y
        │
        │ (20, 215) ← Z3
        │
    ────┼──────── X
        │
    (20,20)      (215,20)
      Z1           Z2
```

### Capture Point Calculation

For each calibration height and tilt configuration:

1. **Flat Plane** (no tilt)
   ```
   z1 = 0, z2 = 0, z3 = 0
   tilt_x = 0°, tilt_y = 0°
   ```

2. **Tilt Around Y-axis** (varies z1, z2)
   ```
   z1 = 5mm, z2 = 2.5mm, z3 = 0mm
   tilt_x = 0°, tilt_y = ~1.5°
   ```

3. **Tilt Around X-axis** (varies z3)
   ```
   z1 = 0, z2 = 0, z3 = 5mm
   tilt_x = ~1.5°, tilt_y = 0°
   ```

### Tilt Angle Calculation
```
tilt_x = arctan(z3 - (z1 + z2)/2) / 200mm) × 180/π
tilt_y = arctan((z2 - z1) / 195mm) × 180/π
```

## Performance Characteristics

### Timing
- **Homing**: ~2-3 seconds
- **Per capture point**: ~1.5-2 seconds (position + capture + detection)
- **Total capture phase (9 points)**: ~15-18 seconds
- **Stereo calibration**: ~1-2 seconds (OpenCV computation)
- **Validation**: ~1 second
- **Total workflow**: ~20-25 seconds

### Accuracy
- **Reprojection error target**: < 0.5 pixels
- **Baseline accuracy**: ±0.1 mm
- **Success rate target**: > 85% (7-8 patterns detected from 9 points)

### Resource Usage
- **Memory**: ~50-100 MB (image buffers + calibration data)
- **CPU**: Single-threaded background execution
- **Storage**: ~5-10 MB per calibration file (JSON)

## Troubleshooting

### Issue: Low Pattern Detection Rate

**Cause**: Poor lighting or calibration pattern not visible in images

**Solutions**:
1. Improve ambient lighting around bed
2. Ensure checkerboard pattern is clean and not wrinkled
3. Adjust camera focus if available
4. Reduce heights to get closer to pattern

### Issue: High Reprojection Error

**Cause**: Camera misalignment or incorrect intrinsic parameters

**Solutions**:
1. Verify camera hardware is firmly mounted
2. Run calibration multiple times
3. Check baseline measurement is accurate
4. Reduce pattern_square_size if too large

### Issue: Calibration Stops Unexpectedly

**Cause**: Motor movement failure or Z-axis error

**Solutions**:
1. Check Z-axis motors respond to commands
2. Verify motor positions are within limits
3. Look for mechanical obstructions
4. Restart system and retry

### Issue: Translation Vector Incorrect

**Cause**: Incorrect baseline distance or camera mounting

**Solutions**:
1. Verify actual camera separation matches specification
2. Check camera mounting brackets
3. Recalibrate with known reference object
4. Review camera orientation (left/right reversed?)

## Advanced Usage

### Custom Calibration Points

Modify capture sequence for specific requirements:

```python
# In AutoStereoCameraCalibration
calibrator = AutoStereoCameraCalibration()
calibrator.heights = np.array([180, 200, 220])  # Different heights
calibrator.tilt_angles = np.array([-10, -5, 0, 5, 10])  # More tilts
```

### Integrate with Depth Maps

Use calibration results for real-time depth computation:

```python
# Compute depth from disparity
Z = (baseline * fx) / disparity
X = (x - cx) * Z / fx
Y = (y - cy) * Z / fy
```

### Manual Calibration Fallback

If automatic calibration fails, use manual probe points:

```python
manual_points = [
    (20, 20, 5.2),
    (215, 20, 5.0),
    (20, 215, 5.1),
]
plane = calibration.fit_bed_plane(manual_points)
```

## References

### OpenCV Functions
- `cv2.findChessboardCorners()` - Pattern detection
- `cv2.cornerSubPix()` - Corner refinement
- `cv2.stereoCalibrate()` - Stereo calibration
- `cv2.stereoRectify()` - Epipolar rectification

### Documentation
- [OpenCV Stereo Calibration](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
- [Camera Calibration Theory](https://learnopencv.com/camera-calibration-using-opencv/)
- [Epipolar Geometry](https://docs.opencv.org/master/da/de9/tutorial_py_epipolar_geometry.html)

## Future Enhancements

1. **Real-time Preview**: Show captured images and detected corners during calibration
2. **Adaptive Strategy**: Auto-adjust heights/tilts based on detection success rate
3. **Self-Healing**: Automatically retry failed captures with adjusted positions
4. **Batch Calibration**: Calibrate multiple stereo cameras in sequence
5. **Validation Visualization**: 3D point cloud visualization of results
6. **Thermal Compensation**: Account for temperature-related focal length changes

# Triple Z-Axis Bed Tilt Calibration System

## Overview

WeldVision X5 now supports **independent triple Z-axis motors** for precise bed leveling during RDK Stereo Camera calibration. This enables the system to compensate for bed tilt and achieve uniform camera distance across the entire scanning area.

## Hardware Configuration

### Motor Positioning

Three Z-axis motors positioned at triangle corners for maximum stability:

```
    Motor 3 (Back-Left)
         Y
         |    Motor 2 (Front-Right)
         |   /
    -----+-----X
        /
    Motor 1 (Front-Left)
```

### Typical Setup

- **Motor 1**: X=20mm, Y=20mm (Front-Left)
- **Motor 2**: X=215mm, Y=20mm (Front-Right)  
- **Motor 3**: X=20mm, Y=215mm (Back-Left)

Each motor controls Z independently:
```
Z1_offset = height to raise motor 1
Z2_offset = height to raise motor 2
Z3_offset = height to raise motor 3
```

## Software Architecture

### Core Components

#### 1. **PrinterController Enhancements**
Located: `backend/hardware/printer_controller.py`

**New Configuration:**
```python
# Triple Z-axis support
enable_triple_z: bool = True
z1_min, z1_max = 0.0, 250.0
z2_min, z2_max = 0.0, 250.0
z3_min, z3_max = 0.0, 250.0
```

**New Methods:**

- `move_triple_z(x, y, z1, z2, z3)` - Move with independent Z axes
  - Validates bounds for all three Z motors
  - Homes printer if needed
  - Sends independent G-code commands
  - Waits for moves to complete (M400)

- `probe_plane_tilt(probe_points)` - Probe bed at multiple points
  - Returns plane equation: z = a*x + b*y + c
  - Uses touch-probe (G31) at each location
  - Performs least-squares plane fitting

- `calibrate_bed_tilt(grid_size)` - Full auto-calibration
  - Probes grid (default 3x3 = 9 points)
  - Fits plane equation
  - Calculates Z motor offsets
  - Moves to calibrated position

#### 2. **Camera Calibration Module**
Located: `backend/hardware/camera_calibration.py` (NEW)

**Key Classes:**

- `CameraIntrinsics` - Camera matrix and distortion
  - Focal length (fx, fy)
  - Principal point (cx, cy)
  - Radial distortion (k1, k2)
  - Tangential distortion (p1, p2)

- `BedTiltPlane` - Fitted plane representation
  - Plane equation (a, b, c coefficients)
  - Tilt angles in X and Y
  - Residual error (fit quality)
  - Z motor offsets for leveling

- `CameraCalibration` - Main calibration manager
  - Intrinsic calibration storage
  - Bed tilt fitting
  - Z offset calculation
  - Validation and reporting

**Methods:**

```python
# Fit plane to probe measurements
fit_bed_plane(probe_measurements) -> BedTiltPlane

# Calculate Z motor offsets from plane
calculate_z_offsets_triple(plane, reference_height) -> (z1, z2, z3)

# Create 3D point clouds with calibration
create_point_cloud(rgb_image, depth_map) -> (points_3d, colors)

# Validate calibration accuracy
validate_calibration(test_points) -> metrics
```

#### 3. **ScanOrchestrator Updates**
Located: `backend/hardware/scan_orchestrator.py`

**New Configuration Options:**
```python
enable_triple_z: bool = True
use_calibrated_tilt: bool = False
```

**New Methods:**

- `calibrate_bed_with_triple_z()` - Trigger calibration
  - Uses printer's auto-calibration
  - Stores results for scanning

- `scan_with_triple_z(scan_id, config)` - Scan with tilt compensation
  - Uses calibrated bed plane
  - Applies Z offsets automatically

- `_capture_and_process_triple_z(x, y, z1, z2, z3)` - Position-specific capture
  - Moves with triple Z values
  - Captures and processes frames
  - Includes Z positioning in results

## API Endpoints

### Calibration API Routes
Base: `/api/calibration`

#### Get/Set Intrinsics
```http
GET /api/calibration/intrinsics
```
Returns camera matrix and distortion parameters.

```http
POST /api/calibration/intrinsics
```
Update camera parameters.

#### Bed Tilt Probing
```http
POST /api/calibration/bed-tilt/probe
```

Request:
```json
{
  "probe_points": [[20, 20], [215, 20], [20, 215]],
  "use_printer": true,
  "reference_height": 10.0
}
```

Response:
```json
{
  "plane_equation": {
    "a": 0.001234,
    "b": -0.000567,
    "c": 10.50
  },
  "tilt_angles": {
    "tilt_x_degrees": 0.32,
    "tilt_y_degrees": -0.07
  },
  "z_offsets": {
    "z1_mm": 0.0,
    "z2_mm": 2.45,
    "z3_mm": 1.12
  },
  "residual_mm": 0.034,
  "measured_points": 3
}
```

#### Auto Calibration
```http
POST /api/calibration/bed-tilt/calibrate
```

Request:
```json
{
  "grid_rows": 3,
  "grid_cols": 3,
  "reference_height": 10.0
}
```

#### Get Current Tilt
```http
GET /api/calibration/bed-tilt/current
```

#### Apply Z Offsets
```http
POST /api/calibration/bed-tilt/apply-triple-z
```

Request:
```json
{
  "z1_mm": 0.0,
  "z2_mm": 2.45,
  "z3_mm": 1.12
}
```

#### Validate Calibration
```http
POST /api/calibration/validate
```

Request:
```json
{
  "test_points": [
    [100, 100, 10.0],
    [150, 150, 10.0],
    [200, 50, 10.0]
  ]
}
```

Response:
```json
{
  "status": "success",
  "mean_error_mm": 0.045,
  "max_error_mm": 0.089,
  "std_error_mm": 0.032,
  "num_points": 3
}
```

#### Get Report
```http
GET /api/calibration/report
```

## React Components

### BedCalibration Component
Located: `components/BedCalibration.tsx`

**Features:**

1. **Configuration Panel**
   - Grid size selector (2×2 to 5×5)
   - Reference height adjustment
   - Auto-calibration button

2. **Calibration Results Display**
   - Plane equation visualization
   - Tilt angle indicators
   - Z motor offset boxes
   - Fit accuracy metrics

3. **Current Status Monitor**
   - Active calibration values
   - Applied Z motor offsets
   - Measured points count

4. **Z Motor Visualization**
   - Three motor boxes (color-coded)
   - Current offset values
   - Apply button for activation

### Styling
Located: `components/BedCalibration.css`

- Responsive grid layout
- Color-coded motor indicators
- Status and message displays
- Mobile-optimized views

## Workflow: Complete Calibration

### Step 1: Prepare Hardware
```bash
1. Connect triple Z-axis motors to printer
2. Home all axes
3. Mount RDK Stereo Camera on nozzle
4. Verify USB connections
```

### Step 2: Auto Calibration (Recommended)
```http
POST /api/calibration/bed-tilt/calibrate
{
  "grid_rows": 3,
  "grid_cols": 3,
  "reference_height": 10.0
}
```

**Process:**
1. Generates 3×3 probe grid
2. Moves to each point and probes
3. Fits plane equation
4. Calculates Z motor offsets
5. Positions at calibrated location

### Step 3: Apply Offsets
```http
POST /api/calibration/bed-tilt/apply-triple-z
{
  "z1_mm": 0.0,
  "z2_mm": 2.45,
  "z3_mm": 1.12
}
```

### Step 4: Verify Calibration
```http
POST /api/calibration/validate
{
  "test_points": [
    [50, 50, 10.0],
    [200, 50, 10.0],
    [50, 200, 10.0],
    [200, 200, 10.0],
    [125, 125, 10.0]
  ]
}
```

**Target:** Mean error < 0.1mm

## Mathematics

### Plane Fitting

Given probe points: (x₁, y₁, z₁), (x₂, y₂, z₂), ..., (xₙ, yₙ, zₙ)

Fit plane: z = a·x + b·y + c

**Solve via least squares:**
```
A = [x₁ y₁ 1]      b = [z₁]
    [x₂ y₂ 1]          [z₂]
    [⋮  ⋮  ⋮]          [⋮ ]
    
Solve A·c = b for coefficients [a, b, c]
```

### Tilt Angles

From plane equation z = a·x + b·y + c:

```
Tilt around Y-axis (X-slope): θₓ = arctan(a)
Tilt around X-axis (Y-slope): θᵧ = arctan(b)
Total tilt: θ = arctan(√(a² + b²))
```

### Z Offset Calculation

For triangle corners at known XY positions:

```
For each corner (xᵢ, yᵢ):
  z_measured = a·xᵢ + b·yᵢ + c
  offset_i = reference_height - z_measured

Normalize: offset_i -= min(offsets)
```

Result: Three Z motor positions that level the bed.

## Performance Characteristics

### Probe Timing
- **Per point:** ~1-2 seconds (move + dwell + probe)
- **3×3 grid:** ~15-20 seconds total
- **5×5 grid:** ~45-60 seconds total

### Memory Usage
- Calibration data: ~1-2 KB
- Point cloud (small area): ~5-10 MB

### Accuracy
- Probe repeatability: ±0.01 mm
- Plane fit error: typically < 0.05 mm
- Camera depth accuracy: ±0.5-1.0 mm (depends on distance)

## Troubleshooting

### Probe Failures
```
Problem: "Could not parse probe response"
Solution: Check G31 probe is configured in firmware
         Verify probe trigger distance
```

### High Residual Error
```
Problem: Residual > 0.2 mm
Solution: More probe points (5×5 instead of 3×3)
         Check for debris on bed
         Verify probe calibration
```

### Uneven Z Application
```
Problem: Bed still tilted after applying offsets
Solution: Check motor connections
         Verify Z motor moves are working
         Recalibrate
```

### Camera Calibration Issues
```
Problem: Depth measurements inconsistent
Solution: Run camera intrinsics calibration first
         Verify camera is stable/secure
         Check baseline distance is correct
```

## Advanced: Manual Calibration

If auto-calibration unavailable:

```bash
# 1. Manual probe at three corners
# Record Z position after G31 at each point
Z1_measured = 9.23 (Motor 1)
Z2_measured = 11.68 (Motor 2)
Z3_measured = 10.35 (Motor 3)

# 2. Calculate plane equation
# Use online plane calculator or Python:
python3 -c "
import numpy as np
points = np.array([
  [20, 20, 9.23],
  [215, 20, 11.68],
  [20, 215, 10.35]
])
A = np.column_stack([points[:, 0], points[:, 1], np.ones(3)])
b = points[:, 2]
coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
print(f'Plane: z = {coeffs[0]:.6f}*x + {coeffs[1]:.6f}*y + {coeffs[2]:.2f}')
"

# 3. Apply offsets manually
POST /api/calibration/bed-tilt/apply-triple-z
{
  "z1_mm": 0.0,
  "z2_mm": 2.45,
  "z3_mm": 1.12
}
```

## Summary

The triple Z-axis bed tilt calibration system provides:

✅ **Automatic plane fitting** - No manual calculations
✅ **Individual Z control** - Independent motor positioning
✅ **RDK camera optimization** - Uniform depth measurement
✅ **Validation tools** - Verify accuracy
✅ **Persistent storage** - Calibration saved to disk
✅ **REST API** - Full programmatic control
✅ **React UI** - User-friendly interface

Perfect for automated welding inspection where consistent camera distance is critical for accurate measurements.

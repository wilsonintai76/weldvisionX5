# SafeMotionController - Safe Motion Control for Custom Inspection Rig

## Overview

`SafeMotionController` is a Python class providing safe, validated motion control for a custom inspection rig based on a CoreXY 3D printer with a stereo camera mounted on the toolhead.

### Key Innovation: Virtual Zero Concept

The rig uses a **physically adjusted Z-limit switch** to establish a "Virtual Zero":
- **Z = 0** → Camera hovers 100mm **above** the bed (Safe Parking Height)
- **Z > 0** → Bed moves **up** (closer to camera) for focusing
- **Z soft limit** (90mm) → Prevents bed from crashing into lens

This inverted paradigm requires specialized safety handling that this controller provides.

## Hardware Setup

```
                Camera (on toolhead)
                     ↓
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    ↑                            ↑
    Z (toolhead frame)    
    ↑
    │
    ├─── Z = 0 (Safe): Camera 100mm above bed
    │
    ├─── Z = 50mm: Focused on part (50mm from safe)
    │
    └─── Z = 90mm: Maximum (soft limit) - lens crash risk!
    
    ↓
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Bed (moves up with positive Z)
```

### Why This Design?

1. **Safe by Default**: Z=0 means camera is always safely elevated
2. **Natural Focus**: More bed movement = closer inspection
3. **Clear Limits**: Soft limit prevents catastrophic lens collision
4. **Simple Homing**: Z limit switch at "safe" position

## Installation & Dependencies

```bash
pip install pyserial opencv-python numpy
```

## API Reference

### Initialization

```python
from safe_motion_controller import SafeMotionController, SafeMotionConfig

# Minimal setup (uses defaults)
controller = SafeMotionController()

# Or with custom config
config = SafeMotionConfig(
    port='/dev/ttyUSB0',           # Serial port
    baudrate=250000,               # Marlin standard
    z_safe_height=0.0,            # Virtual Zero
    z_soft_limit=90.0,            # Max before crash
    park_x=300.0,                 # Loading corner
    park_y=300.0,
    homing_speed=6000,            # mm/min
    move_speed=3000,
    scan_speed=1000,
)

controller = SafeMotionController(config)
```

### Connection Management

```python
# Connect to serial and camera
if controller.connect():
    print("✓ Ready")
else:
    print("✗ Connection failed")

# Get status
status = controller.get_status()
print(f"Homed: {status['homed_xy']}, {status['homed_z']}")
print(f"Current Z: {status['current_z']}")
print(f"Safety State: {status['safety_state']}")

# Disconnect
controller.disconnect()
```

### Method 1: Safe Homing

```python
if not controller.home_safely():
    print("Homing failed!")
    sys.exit(1)
```

**Sequence:**
1. Home **X/Y first** (`G28 X Y`) - prevents gantry collision
2. Home **Z** (`G28 Z`) - now safe because X/Y are homed
3. Verify Z=0 (Virtual Zero, camera at safe height)
4. Log: "Rig Homing Complete. Camera is parked at Safe Height (Z=0)."

**Why X/Y First?**
- CoreXY mechanism: X/Y gantry must be homed to prevent collision with frame
- Z depends on X/Y being in known positions
- Marlin safety standard requires this sequence

**Output:**
```
============================================================
STARTING SAFE HOMING SEQUENCE
============================================================
→ Homing X/Y axes first (prevents collision)
✓ X/Y axes homed successfully
→ Homing Z axis (now safe)
✓ Z axis homed successfully
============================================================
✓✓✓ RIG HOMING COMPLETE ✓✓✓
✓ Camera is parked at Safe Height (Z = 0 mm)
✓ Virtual Zero established: Camera is 100mm above bed
============================================================
```

### Method 2: Park Toolhead

```python
if not controller.park_toolhead():
    print("Parking failed!")
else:
    print("✓ Ready for loading")

# Or specify custom corner
if not controller.park_toolhead(x=250, y=250):
    print("Failed")
```

**Actions:**
1. Move to parking corner (X300, Y300 default)
2. Move Z to Safe Height (Z=0)
3. Ensure motors locked

**Purpose:** Positions camera safely out of the way when loading/unloading parts

**Output:**
```
────────────────────────────────────────────────────────────
PARKING TOOLHEAD FOR LOADING/UNLOADING
────────────────────────────────────────────────────────────
→ Moving to parking corner (X300, Y300)
✓ X/Y axes homed successfully
→ Raising Z to Safe Height (Z0)
✓ Toolhead parked safely - Ready for loading
────────────────────────────────────────────────────────────
```

### Method 3: Move to Inspection Height

```python
# Move Z to focus on part (50mm from safe height)
if not controller.move_to_inspection_height(50):
    print("Move failed!")
else:
    print("✓ At inspection height")

# Try to exceed soft limit (will be blocked)
if not controller.move_to_inspection_height(95):
    print("✗ SOFT LIMIT EXCEEDED: 95 > 90")
    print("  Risk of bed crashing into lens!")
```

**Safety Checks:**

| Check | Value | Purpose |
|-------|-------|---------|
| Must be homed | X/Y, Z | Cannot move until initialized |
| Soft limit | ≤ 90mm | Prevent lens crash |
| Minimum height | ≥ -10mm | Don't go below safe |

**Sanity Check Example:**
```python
# Valid moves
controller.move_to_inspection_height(0)    # ✓ Safe height
controller.move_to_inspection_height(50)   # ✓ Focused
controller.move_to_inspection_height(90)   # ✓ Maximum (risky but allowed)

# Invalid moves (rejected with error message)
controller.move_to_inspection_height(95)   # ✗ Exceeds soft limit
controller.move_to_inspection_height(-15)  # ✗ Below minimum
```

**Output (Valid):**
```
────────────────────────────────────────────────────────────
MOVING TO INSPECTION HEIGHT
────────────────────────────────────────────────────────────
✓ Height validated: 50mm (Safe: 90mm)
→ Moving Z to inspection height...
────────────────────────────────────────────────────────────
✓ At inspection height: Z = 50mm
✓ Camera focused on part (50mm from safe height)
────────────────────────────────────────────────────────────
```

**Output (Invalid):**
```
────────────────────────────────────────────────────────────
MOVING TO INSPECTION HEIGHT
────────────────────────────────────────────────────────────
✗ SOFT LIMIT EXCEEDED: 95 > 90
  Risk of bed crashing into lens!
```

### Method 4: Panorama Scanning

```python
# Scan from X=50 to X=250 in 20mm steps at Z=50
# (assumes already at inspection height)
images = controller.scan_panorama(start_x=50, end_x=250, step=20)

if images:
    print(f"✓ Captured {len(images)} frames")
    
    # Stitch into panorama
    panorama = controller.stitch_panorama(images)
    if panorama is not None:
        cv2.imwrite("panorama.png", panorama)
    
    # Save all frames + metadata
    output_dir = controller.save_scan("results")
else:
    print("✗ No frames captured")
```

**Scanning Process:**

For each X position:
1. Move to (X, Y, Z_current) - **Z stays at inspection height**
2. Wait for vibration to settle (default 0.5s)
3. Capture image with stereo camera
4. Store with position metadata (X, Y, Z, timestamp)

**Important:** Z is **maintained** throughout the scan - no vertical movement!

**Output:**
```
============================================================
STARTING PANORAMA SCAN
============================================================
Scan distance: 200mm, Step: 20mm, Points: 11
Inspection height: Z = 50mm
[1/11] Moving to X50.0...
[1/11] Capturing...
[2/11] Moving to X70.0...
[2/11] Capturing...
...
[11/11] Moving to X250.0...
[11/11] Capturing...
============================================================
✓ Panorama scan complete: 11 frames captured
============================================================
```

### Method 5: Image Stitching

```python
# Stitch captured frames into panorama
panorama = controller.stitch_panorama(images)

if panorama is not None:
    cv2.imwrite("weld_panorama.png", panorama)
    print(f"✓ Panorama size: {panorama.shape}")
else:
    print("Stitching failed - check image overlap")
```

**Stitcher Modes:**
- `"best"` (default) - OpenCV panorama stitcher, best quality
- `"fast"` - OpenCV scans stitcher, faster, lower quality

**Fallback:** If stitching fails, returns horizontally concatenated images

### Method 6: Save Scan Data

```python
output_dir = controller.save_scan("inspection_results")
print(f"Saved to {output_dir}")
```

**Output Structure:**
```
inspection_results/
└── 20251205_143022/           # Timestamp directory
    ├── frame_000_x50.0.png
    ├── frame_001_x70.0.png
    ├── frame_002_x90.0.png
    └── ...
```

## Complete Workflow Example

```python
#!/usr/bin/env python3
"""Complete weld inspection workflow using SafeMotionController"""

import logging
import sys
import cv2
from safe_motion_controller import SafeMotionController, SafeMotionConfig

logging.basicConfig(level=logging.INFO)

def main():
    # Configure controller for your rig
    config = SafeMotionConfig(
        port='/dev/ttyUSB0',
        baudrate=250000,
        z_safe_height=0.0,
        z_soft_limit=90.0,
        park_x=280.0,
        park_y=280.0,
    )
    
    controller = SafeMotionController(config)
    
    try:
        # ============ SETUP ============
        print("\n[1/5] Connecting...")
        if not controller.connect():
            print("✗ Connection failed")
            return False
        
        # ============ HOMING ============
        print("\n[2/5] Homing rig...")
        if not controller.home_safely():
            print("✗ Homing failed")
            return False
        
        # ============ LOADING ============
        print("\n[3/5] Moving to loading position...")
        if not controller.park_toolhead():
            print("✗ Failed to park")
            return False
        
        input("\n   → Load part and press Enter...")
        
        # ============ POSITIONING ============
        print("\n[4/5] Moving to inspection height...")
        if not controller.move_to_inspection_height(50):
            print("✗ Failed to reach inspection height")
            return False
        
        # ============ SCANNING ============
        print("\n[5/5] Scanning weld...")
        images = controller.scan_panorama(
            start_x=50,
            end_x=250,
            step=15,
            y_pos=150
        )
        
        if not images:
            print("✗ Scan produced no images")
            return False
        
        print(f"\n✓ Captured {len(images)} frames")
        
        # ============ STITCHING ============
        print("Stitching panorama...")
        panorama = controller.stitch_panorama(images)
        
        if panorama is not None:
            output_file = "weld_panorama.png"
            cv2.imwrite(output_file, panorama)
            print(f"✓ Panorama saved: {output_file}")
            print(f"  Size: {panorama.shape}")
        else:
            print("⚠ Stitching failed - using first frame")
            cv2.imwrite("weld_frame_0.png", images[0])
        
        # ============ SAVE DATA ============
        print("Saving scan data...")
        output_dir = controller.save_scan("inspection_data")
        print(f"✓ Data saved: {output_dir}")
        
        # ============ CLEANUP ============
        print("\nReturning to parking position...")
        controller.park_toolhead()
        
        print("\n✓✓✓ INSPECTION COMPLETE ✓✓✓\n")
        return True
        
    except KeyboardInterrupt:
        print("\n⚠ Interrupted by user")
        controller.emergency_stop()
        return False
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        controller.emergency_stop()
        return False
    finally:
        controller.disconnect()

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
```

## Safety Features

### 1. Sequential Homing
```
G28 X Y  → X/Y home (frame collision prevention)
   ↓
G28 Z    → Z home (now safe)
```

### 2. Z Soft Limit
```python
if target_z > 90.0:
    reject()  # "Risk of bed crashing into lens!"
```

### 3. Serial Timeout Handling
```python
if timeout after 5 seconds:
    log error
    return False
    # Never blocks indefinitely
```

### 4. Height Maintenance During Scan
```python
for x in scan_range:
    move(x, y, z_current)  # Z never changes!
    capture()
```

### 5. State Validation
- Must be homed before any moves
- Must be at inspection height before scanning
- All moves logged for audit trail

## Error Handling

### Connection Errors

**Problem:** Serial port not found
```
✗ Serial connection failed: [Errno 2] No such file or directory: '/dev/ttyUSB0'
```

**Solution:**
```bash
# Linux
ls /dev/ttyUSB*
# Windows
# Check Device Manager → COM ports
```

### Homing Errors

**Problem:** X/Y homing fails
```
✗ X/Y homing failed
```

**Solutions:**
- Check X/Y limit switches are wired and triggering
- Verify Marlin firmware is running
- Manually move to corner and test: `controller.serial.send_gcode("G28 X Y")`

### Soft Limit Violated

**Problem:** Trying to move beyond safe Z
```
✗ SOFT LIMIT EXCEEDED: 95 > 90
  Risk of bed crashing into lens!
```

**Solution:**
```python
# Use a value ≤ 90
controller.move_to_inspection_height(85)  # ✓ OK
```

### Camera Not Detected

**Problem:** Cannot read from camera
```
✗ Failed to read from camera
```

**Solutions:**
- Check USB connection
- Try different camera ID: `SafeMotionConfig(camera_id=1)`
- Test: `python -c "import cv2; print(cv2.VideoCapture(0).read())"`

## Performance Characteristics

| Operation | Time |
|-----------|------|
| Home safely | ~15 seconds |
| Park toolhead | ~5 seconds |
| Move to inspection height | ~2 seconds |
| Per-frame scan (move + capture + settle) | ~1.5 seconds |
| Stitch 10 images | ~2-3 seconds |

**Example Scan Time:**
- Scan range: 200mm (X50 → X250)
- Step: 20mm
- Frames: 11
- Move+settle per frame: 1.5s
- **Total scan: ~16 seconds**
- Stitching: ~2 seconds
- **Total: ~18 seconds**

## Troubleshooting Guide

### Issue: "Must home before parking"

**Cause:** Called `park_toolhead()` without `home_safely()`

**Fix:**
```python
controller.home_safely()        # First!
controller.park_toolhead()      # Then
```

### Issue: "Must move to inspection height before scanning"

**Cause:** Called `scan_panorama()` without `move_to_inspection_height()`

**Fix:**
```python
controller.move_to_inspection_height(50)  # Set focus height
images = controller.scan_panorama(50, 250, 20)  # Then scan
```

### Issue: Panorama has poor alignment

**Cause:** Insufficient image overlap or vibration

**Solutions:**
- Reduce `step` size (20mm → 15mm) for better overlap
- Increase `capture_delay` (0.5s → 1.0s) for vibration settling
- Check camera is in focus at inspection height
- Ensure adequate lighting on weld

### Issue: Scan image quality poor

**Cause:** Wrong focus height or lighting

**Fix:**
```python
# Try different Z value
controller.move_to_inspection_height(40)  # Closer
images1 = controller.scan_panorama(50, 250, 20)

controller.move_to_inspection_height(60)  # Farther
images2 = controller.scan_panorama(50, 250, 20)

# Use better quality images
```

## Advanced Usage

### Custom Speeds

```python
config = SafeMotionConfig(
    homing_speed=3000,    # Slow homing (safer)
    move_speed=2000,      # Slow moves
    scan_speed=500,       # Very slow scanning (stable)
)
```

### Multiple Scans

```python
# Scan at different heights
for z_height in [30, 50, 70]:
    controller.move_to_inspection_height(z_height)
    images = controller.scan_panorama(50, 250, 20)
    panorama = controller.stitch_panorama(images)
    cv2.imwrite(f"panorama_z{z_height}.png", panorama)
```

### Emergency Response

```python
try:
    images = controller.scan_panorama(50, 250, 20)
except Exception as e:
    print(f"Error during scan: {e}")
    controller.emergency_stop()      # Immediate halt
    controller.park_toolhead()       # Safe position
```

## References

### Marlin G-Code Commands Used

| Command | Purpose |
|---------|---------|
| `G28` | Home axes |
| `G28 X Y` | Home only X/Y |
| `G28 Z` | Home only Z |
| `G0` | Rapid linear move |
| `M410` | Quickstop (emergency) |

### OpenCV Stitching

- `cv2.Stitcher_create()` - Default stitcher
- `cv2.Stitcher_create(cv2.Stitcher_PANORAMA)` - Best quality
- `cv2.Stitcher_create(cv2.Stitcher_SCANS)` - Faster

## License

This code is designed for the WeldVision X5 educational weld inspection system.

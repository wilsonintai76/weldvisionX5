# Rig Configuration Guide

## Overview

WeldVision X5 supports two distinct rig types, each with specialized features and capabilities. Users can switch between rig configurations at any time via the sidebar selector.

---

## Rig Type 1: Manual Height Adjustment

### Description
Fixed horizontal positioning with manual vertical (Z-axis) height adjustment via hand crank or manual mechanism.

### Use Cases
- **Educational demonstrations** - Static mounting for consistent student workstation setup
- **Quality control sampling** - Fixed-position inspection station
- **Single-position measurements** - Focused depth analysis at fixed location
- **Space-constrained environments** - Minimal footprint requirements

### Available Features

#### **Manual Calibration**
- Single checkerboard image calibration
- Manual height/tilt adjustment interface
- Quick 30-second setup
- Fixed bed calibration verification

#### **Stereo Calibration**
- Multi-frame stereo pair calibration (15-20 images)
- Hardware-synchronized depth sensor setup
- Epipolar geometry validation
- Baseline auto-detection from RDK SC230AI dual 2MP stereo camera (70mm baseline)

#### **Live Scanner**
- Real-time 2D image capture at fixed position
- Weld bead profile analysis
- Height, width, uniformity measurements
- Porosity and defect detection

#### **Student Management & History**
- Class roster management
- Scan history archiving
- Performance tracking
- Rubric-based evaluation

### Hardware Requirements
- RDK Stereo Camera Module (Dual 2MP SC230AI, 1920×1080 per camera, 70mm baseline)
- Manual Z-axis adjustment mechanism
- Fixed horizontal positioning (tripod/bench mount)
- Power supply: 12V/2A minimum

### Measurement Constraints
- ✅ Accurate at fixed distance (calibration-dependent)
- ✅ Excellent depth precision with paired depth sensor
- ⚠️ Limited to single viewing angle
- ⚠️ Requires manual repositioning for multiple angles

---

## Rig Type 2: 3-Axis Z with Panorama (3D Printer Setup)

### Description
3-axis linear motion system (X, Y, Z) with motorized control, enabling automated panoramic scanning for 360° volumetric reconstruction.

### Use Cases
- **Comprehensive volumetric analysis** - Full 3D geometry of weld bead
- **High-precision certification** - Complete surface mapping
- **Automated inspection lines** - Integration with production workflows
- **Research and development** - Detailed surface topology studies

### Available Features

#### **Panorama Scanner**
- Automated multi-angle capture (configurable angles)
- 3D point cloud generation
- Full volumetric reconstruction
- Surface topology analysis
- Automated stitching and fusion
- Height profile from multiple viewpoints

#### **Safe Motion Control**
- 3-axis linear motion (X, Y, Z)
- Velocity and acceleration limits
- Collision detection and soft limits
- Emergency stop capability
- Real-time position feedback
- Manual jog controls
- Automated homing sequence

#### **Stereo Calibration**
- Multi-frame calibration with system movement
- Dynamic baseline measurement during panorama
- Calibration refinement during scan
- Trajectory-based accuracy improvement

#### **Live Scanner**
- Dynamic position-aware capture
- Real-time pose information with each frame
- Position-tagged scan sequences
- Selective high-resolution capture at optimized angles

#### **Student Management & History**
- Full scan session archiving
- Multi-position measurement analysis
- Comprehensive defect mapping
- 3D model export capability

### Hardware Requirements
- RDK Stereo Camera Module (Dual 2MP SC230AI, 1920×1080 per camera, 70mm baseline)
- 3-axis linear motion system (NEMA-17 or equivalent steppers)
- Motion controller (ROS2-compatible: DIN-Rail PLC or dedicated controller)
- Belt/ballscrew drive mechanism
- Power supply: 24V/3A minimum (stepper power)
- Power supply: 12V/2A minimum (camera/control)

### Measurement Capabilities
- ✅ Full 3D volumetric measurement
- ✅ 360° view coverage (configurable)
- ✅ Multi-angle baseline measurement
- ✅ Sub-millimeter accuracy across entire surface
- ✅ Automated defect localization in 3D space
- ⚠️ Requires longer scan time (1-5 minutes per weld)
- ⚠️ Higher computational requirements (point cloud processing)

---

## Switching Between Rig Types

### How to Change
1. Open **WeldVision X5** application
2. Look at **sidebar** → **"Rig Configuration"** section
3. Click either **"Manual Height"** or **"3-Axis + Panorama"**
4. System automatically disables incompatible features
5. Navigation returns to Dashboard if current view is unavailable

### Feature Availability Table

| Feature | Manual Height | 3-Axis Panorama |
|---------|:-------------:|:---------------:|
| Dashboard | ✅ | ✅ |
| Live Scanner | ✅ | ✅ |
| Students | ✅ | ✅ |
| Scan History | ✅ | ✅ |
| Manual Calibration | ✅ | ❌ (disabled) |
| Stereo Calibration | ✅ | ✅ |
| Panorama Scanner | ❌ (disabled) | ✅ |
| Safe Motion Control | ❌ (disabled) | ✅ |
| Settings | ✅ | ✅ |

---

## Configuration Best Practices

### Manual Height Rig Setup
1. **Mount camera** at fixed distance from welding surface
2. **Calibrate stereo** with checkerboard at working distance
3. **Adjust height** with manual mechanism as needed
4. **Verify depth** accuracy before production use
5. **Document baseline** for reference measurements

### 3-Axis Panorama Rig Setup
1. **Home the system** - Run automated homing sequence
2. **Calibrate motion limits** - Set soft boundaries
3. **Run safe motion tests** - Verify no collisions
4. **Perform stereo calibration** - Multi-position capture
5. **Configure panorama angles** - Set scan trajectory
6. **Test single panorama** - Verify complete workflow

---

## Disabled Features Explanation

### Why Manual Calibration is disabled for 3-Axis Panorama:
- Manual Calibration designed for **single fixed position**
- 3-Axis system requires **motion-aware calibration**
- Use **Stereo Calibration** instead (supports multiple positions)
- Safe Motion first establishes accurate motion model

### Why Panorama Scanner is disabled for Manual Height:
- Panorama requires **motorized multi-position control**
- Manual system lacks position feedback/motor control
- Cannot reliably track camera position across images
- Would produce unreliable volumetric reconstruction

### Why Safe Motion is disabled for Manual Height:
- Manual Height uses **human-operated adjustment**
- No motorized axes to control
- No collision detection needed
- Safety handled via physical limits

---

## Switching Scenarios

### Scenario 1: Manual Height → 3-Axis Panorama
**If currently viewing:** Manual Calibration, Bed Tilt, or similar
**Action:** System automatically navigates to Dashboard
**Why:** These views don't exist in 3-Axis configuration
**Next steps:** 
1. Run Safe Motion tests
2. Perform motion-aware Stereo Calibration
3. Configure Panorama scan parameters

### Scenario 2: 3-Axis Panorama → Manual Height
**If currently viewing:** Panorama Scanner or Safe Motion
**Action:** System automatically navigates to Dashboard
**Why:** These features don't exist in Manual configuration
**Next steps:**
1. Place weld sample under camera
2. Adjust height manually to focal distance
3. Run Quick Stereo Calibration

---

## Performance Considerations

### Manual Height Rig
- **Calibration time:** ~5-10 minutes (one-time)
- **Scan time:** ~5-10 seconds
- **Processing:** Real-time (2D analysis)
- **Accuracy:** ±2-5mm (calibration-dependent)
- **Computational load:** Low (2D metrics only)

### 3-Axis Panorama Rig
- **Calibration time:** ~10-20 minutes (motion + stereo)
- **Scan time:** ~1-5 minutes (depending on resolution)
- **Processing:** 5-30 minutes (point cloud, fusion)
- **Accuracy:** ±1-3mm (full 3D measurement)
- **Computational load:** High (volumetric processing)

---

## Troubleshooting

### Rig selector not visible in sidebar?
- Check that you're viewing the main dashboard
- Sidebar appears left of main content area
- Look for blue "Rig Configuration" section

### Feature I need is grayed out?
- Verify you've selected the correct rig type
- Check "Rig Configuration" selector in sidebar
- Switch to the appropriate rig type

### View disappears when switching rigs?
- This is normal! Incompatible features redirect to Dashboard
- The feature is disabled, not deleted
- Switch back to the original rig to access it

### Motion errors with 3-Axis Panorama?
- First: Run "Safe Motion Control" → "Home System"
- Check: All axes return to home position
- Verify: Z-axis moves freely without obstruction
- Review: Motion limits in Safe Motion panel

---

## Quick Reference

**Manual Height Rig:**
- **Ideal for:** Fixed-position quality control
- **Setup time:** ~10 minutes
- **Best features:** Quick calibration, consistent measurements
- **Limitation:** Single viewing angle

**3-Axis Panorama Rig:**
- **Ideal for:** Complete volumetric analysis
- **Setup time:** ~30 minutes (including motion calibration)
- **Best features:** 360° coverage, automated scanning
- **Limitation:** Longer scan and processing time

---

*Last Updated: December 2025*
*For support: Contact RDK X5 System Administrator*

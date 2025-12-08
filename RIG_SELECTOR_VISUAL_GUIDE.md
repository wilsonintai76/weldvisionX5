# Rig Type Selector - Visual Overview

## Sidebar Layout

### Before (Old Structure)
```
├─ Dashboard
├─ Live Scanner
├─ Students
├─ Scan History
├─ Manual Calibration      ← All mixed together
├─ Stereo Calibration
├─ Panorama Scanner        ← Incompatible with manual rig
├─ Safe Motion             ← Incompatible with manual rig
├─ Settings
└─ Help & Guide
```

### After (New Structure with Rig Selection)

#### BASIC RIG (Manual) - Selected
```
WeldEval X5
─────────────────────────────
├─ Dashboard
├─ Live Scanner
├─ Students
├─ Scan History
│
├─────────────────────────────
│  Rig Configuration
│  ┌────────────────────────┐
│  │ ◉ Basic Rig (Manual)   │ ← Selected (blue)
│  │ ○ Advanced Rig (3-Axis)│
│  └────────────────────────┘
│
├─────────────────────────────
│  Features
│  ├─ Manual Calibration    ← Available for this rig
│  └─ Stereo Calibration
│
├─ Settings
└─ Help & Guide
```

**Status:** 
- ✅ Panorama Scanner hidden
- ✅ Safe Motion hidden  
- ✅ Clean, focused UI

#### ADVANCED RIG (3-Axis) - Selected
```
WeldEval X5
─────────────────────────────
├─ Dashboard
├─ Live Scanner
├─ Students
├─ Scan History
│
├─────────────────────────────
│  Rig Configuration
│  ┌────────────────────────┐
│  │ ○ Basic Rig (Manual)   │
│  │ ◉ Advanced Rig (3-Axis)│ ← Selected (blue)
│  └────────────────────────┘
│
├─────────────────────────────
│  Features
│  ├─ Panorama Scanner      ← Available for this rig
│  ├─ Safe Motion Control   ← Motor control
│  └─ Stereo Calibration
│
├─ Settings
└─ Help & Guide
```

**Status:**
- ✅ Manual Calibration hidden
- ✅ 3D scanning ready
- ✅ Motion control available

---

## Interactive Switching

### Scenario 1: Switch from Basic Rig to Advanced Rig

**Current State:**
```
Viewing: Manual Calibration
Rig: Basic Rig (Manual)
```

**User Action:**
```
Click "Advanced Rig (3-Axis)" button
```

**System Response:**
```
1. Detect: Manual Calibration is not compatible with Advanced Rig
2. Redirect: View → Dashboard (automatic)
3. Update: Sidebar features → Show Panorama Scanner, Safe Motion
4. Hide: Manual Calibration option
5. Result: User sees Dashboard with new sidebar options
```

**Visual Result:**
```
Before:                          After:
┌─────────────────┐             ┌─────────────────┐
│ MANUAL_BED_     │             │   DASHBOARD     │
│ CALIBRATION     │             │                 │
│ (Active View)   │  ──→        │ (New View)      │
│                 │             │                 │
│ Basic Rig       │             │ Advanced Rig    │
│ ◉ Selected      │             │ ◉ Selected      │
└─────────────────┘             └─────────────────┘

Sidebar:                        Sidebar:
- Manual Calib.                 - Panorama Scan.
- Stereo Calib.                 - Safe Motion
                                - Stereo Calib.
```

### Scenario 2: Switch While Viewing Stereo Calibration

**Current State:**
```
Viewing: Stereo Calibration
Rig: Basic Rig (Manual)
```

**User Action:**
```
Click "Advanced Rig (3-Axis)" button
```

**System Response:**
```
1. Check: Stereo Calibration is compatible with both rigs ✓
2. Keep: View remains on Stereo Calibration (no redirect)
3. Update: Sidebar features only
4. Result: Seamless transition, user stays on same screen
```

**Visual Result:**
```
Before:                          After:
┌──────────────────────┐        ┌──────────────────────┐
│ STEREO CALIBRATION   │        │ STEREO CALIBRATION   │
│ (Active View)        │        │ (Still Active)       │
│                      │   ──→  │                      │
│ Basic Rig (Manual)   │        │ Advanced Rig (3-Axis)│
│ ◉ Selected           │        │ ◉ Selected           │
└──────────────────────┘        └──────────────────────┘

No navigation interrupt!
User continues with calibration
```

---

## Visual Comparison Table

| Aspect | Basic Rig | Advanced Rig |
|--------|-----------|--------------|
| **Rig Selector** | ◉ Basic Rig (Manual) | ◉ Advanced Rig (3-Axis) |
| **Manual Calibration** | 📍 Visible | ✖️ Hidden |
| **Stereo Calibration** | 📍 Visible | 📍 Visible |
| **Panorama Scanner** | ✖️ Hidden | 📍 Visible |
| **Safe Motion** | ✖️ Hidden | 📍 Visible |
| **Menu Items** | 2 features | 3 features |
| **View on Switch** | Redirects to Dashboard | Stays on current (if compatible) |

---

## Button States

### Rig Selector Buttons

#### Manual Height Button (Selected)
```
┌──────────────────────────┐
│ ◉ Manual Height          │  ← Blue highlight
│ bg: industrial-blue      │  ← text-white
│ shadow: blue glow        │  ← shadow-lg
└──────────────────────────┘
```

#### Manual Height Button (Not Selected)
```
┌──────────────────────────┐
│ ○ Manual Height          │  ← Gray button
│ bg: slate-800            │  ← text-slate-300
│ hover: lighter gray      │  ← hover:bg-slate-700
└──────────────────────────┘
```

---

## Features Section Label

### Shows for Manual Height
```
┌────────────────────────────────┐
│ FEATURES                       │  ← appears when Manual Height
│ ├─ Manual Calibration          │     selected
│ └─ Stereo Calibration          │
└────────────────────────────────┘
```

### Shows for 3-Axis Panorama
```
┌────────────────────────────────┐
│ FEATURES                       │  ← appears when 3-Axis
│ ├─ Panorama Scanner            │     selected
│ ├─ Safe Motion Control         │
│ └─ Stereo Calibration          │
└────────────────────────────────┘
```

### How It Works
- "Features" label dynamically appears/disappears
- Based on: `{rigType === RigType.BASIC_RIG && ...}`
- Clean, minimal UI
- No disabled buttons (hidden items use conditional rendering)

---

## Edge Cases Handled

### 1. Panorama View → Switch to Basic Rig
```
Current: PANORAMA_SCANNER view
Action: Select Basic Rig
Result: → Redirect to DASHBOARD
Reason: Panorama not available in Basic rig
```

### 2. Safe Motion View → Switch to Basic Rig  
```
Current: SAFE_MOTION view
Action: Select Basic Rig
Result: → Redirect to DASHBOARD
Reason: Motor control not available in Basic rig
```

### 3. Manual Calibration View → Switch to Advanced Rig
```
Current: MANUAL_BED_CALIBRATION view
Action: Select Advanced Rig (3-Axis)
Result: → Redirect to DASHBOARD
Reason: Single-position calibration not for multi-axis
```

### 4. Stereo Calibration View → Switch Between Any Rigs
```
Current: STEREO_CALIBRATION view
Action: Select ANY rig (switch A→B or B→A)
Result: → STAY on STEREO_CALIBRATION
Reason: Stereo calibration works on both rigs
```

### 5. Dashboard View → Switch Between Any Rigs
```
Current: DASHBOARD view
Action: Select ANY rig (switch A→B or B→A)
Result: → STAY on DASHBOARD
Reason: Dashboard always works everywhere
```

---

## Styling Details

### Rig Configuration Section (Sidebar)
```css
{
  marginTop: 1rem,          /* my-4 */
  paddingTop: 1rem,         /* pt-4 */
  paddingBottom: undefined, /* no pb */
  borderTop: '1px solid',   /* border-t */
  borderColor: '#1e293b',   /* border-slate-800 */
}
```

### Section Label
```css
fontSize: '0.75rem',        /* text-xs */
fontWeight: 600,            /* font-semibold */
color: '#64748b',           /* text-slate-400 */
textTransform: 'uppercase', /* uppercase */
letterSpacing: 0.05em,      /* tracking-wider */
marginBottom: 0.75rem,      /* mb-3 */
```

### Selected Button
```css
backgroundColor: '#1e40af',     /* bg-industrial-blue */
color: '#ffffff',               /* text-white */
boxShadow: '0 20px 25px -5px rgba(30, 64, 175, 0.2)', /* shadow-lg shadow-blue/20 */
borderRadius: '0.5rem',         /* rounded-lg */
padding: '0.5rem 0.75rem',      /* px-3 py-2 */
```

### Unselected Button
```css
backgroundColor: '#1e293b',     /* bg-slate-800 */
color: '#cbd5e1',               /* text-slate-300 */
borderRadius: '0.5rem',         /* rounded-lg */
padding: '0.5rem 0.75rem',      /* px-3 py-2 */
transition: 'all 150ms',        /* transition-colors */
```

---

## Code Structure

### Sidebar JSX Structure
```tsx
<nav className="flex-1">
  {/* Main navigation items */}
  <SidebarItem ... />
  <SidebarItem ... />
  
  {/* RIG TYPE SELECTOR */}
  <div className="my-4 px-2 pt-4 border-t border-slate-800">
    <p className="text-xs font-semibold text-slate-400 uppercase tracking-wider mb-3">
      Rig Configuration
    </p>
    <div className="space-y-2">
      <button onClick={() => setRigType(RigType.BASIC_RIG)} 
              className={rigType === RigType.BASIC_RIG ? 'selected' : 'unselected'}>
        Basic Rig (Manual)
      </button>
      <button onClick={() => setRigType(RigType.ADVANCED_RIG)}
              className={rigType === RigType.ADVANCED_RIG ? 'selected' : 'unselected'}>
        Advanced Rig (3-Axis)
      </button>
    </div>
  </div>
  
  {/* CONDITIONAL FEATURES - BASIC RIG */}
  {rigType === RigType.BASIC_RIG && (
    <>
      <p className="...">Features</p>
      <SidebarItem label="Manual Calibration" ... />
      <SidebarItem label="Stereo Calibration" ... />
    </>
  )}

  {/* CONDITIONAL FEATURES - ADVANCED RIG */}
  {rigType === RigType.ADVANCED_RIG && (
    <>
      <p className="...">Features</p>
      <SidebarItem label="Panorama Scanner" ... />
      <SidebarItem label="Safe Motion Control" ... />
      <SidebarItem label="Stereo Calibration" ... />
    </>
  )}
</nav>
```

---

## User Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    WeldVision X5 Startup                     │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ├──→ rigType = MANUAL_HEIGHT (default)
                 │
                 └──→ Display Sidebar:
                     ├─ Dashboard
                     ├─ Live Scanner
                     ├─ Students
                     ├─ History
                     ├─ RIG SELECTOR
                     │  ├─ ◉ Basic Rig (Manual)
                     │  └─ ○ Advanced Rig (3-Axis)
                     └─ Features:
                        ├─ Manual Calibration
                        └─ Stereo Calibration

              ↓ USER CLICKS "Advanced Rig (3-Axis)"

┌─────────────────────────────────────────────────────────────┐
│              Rig Type Update triggered                        │
└────────────────┬────────────────────────────────────────────┘
                 │
                 ├──→ setRigType(ADVANCED_RIG)
                 │
                 ├──→ Check current view compatibility
                 │    (if MANUAL_CALIBRATION → redirect DASHBOARD)
                 │
                 └──→ Display Sidebar:
                     ├─ Dashboard
                     ├─ Live Scanner
                     ├─ Students
                     ├─ History
                     ├─ RIG SELECTOR
                     │  ├─ ○ Basic Rig (Manual)
                     │  └─ ◉ Advanced Rig (3-Axis)
                     └─ Features:
                        ├─ Panorama Scanner
                        ├─ Safe Motion Control
                        └─ Stereo Calibration
```

---

## Accessibility Features

### Keyboard Navigation
- Tab through rig selector buttons ✅
- Enter/Space to activate buttons ✅
- Focus visible (browser default) ✅

### Screen Readers
- Buttons have descriptive text ✅
- Labels explain "Rig Configuration" ✅
- "Features" section labeled ✅

### Color Contrast
- Blue (industrial-blue) on white: High contrast ✅
- Gray text on slate-800: Sufficient contrast ✅
- Selected state: Clear visual distinction ✅

---

*Generated: December 8, 2025*  
*For: WeldVision X5 Team*  
*Status: ✅ Ready for Deployment*

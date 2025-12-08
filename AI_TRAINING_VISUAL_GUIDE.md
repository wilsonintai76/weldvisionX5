# 📱 AI & Training - Visual Operator's Guide

Step-by-step visual instructions for everyday operations.

---

## Daily Operations Checklist

### Morning - Check System Status

```
┌─────────────────────────────────────┐
│ Step 1: Open Inference Monitor      │
├─────────────────────────────────────┤
│                                     │
│   Sidebar > Click Brain Icon 🧠     │
│                                     │
│   Expected Result:                  │
│   ✓ Green Connected status          │
│   ✓ Last detection shows            │
│   ✓ Statistics updating             │
│                                     │
│   If Red (Disconnected):            │
│   ✗ Restart backend server          │
│   ✗ Check network connection        │
│                                     │
└─────────────────────────────────────┘
```

### During Scanning - Monitor in Real Time

```
┌─────────────────────────────────────┐
│ Keep Inference Monitor Open         │
├─────────────────────────────────────┤
│                                     │
│   Live Scanner Tab        │ Inference Monitor Tab │
│   ┌─────────────────┐     │ ┌──────────────────┐   │
│   │ Scan button     │     │ │ Results updating │   │
│   │ [SCAN NOW]      │────────▶│ confidence: 92% │   │
│   │ (takes photo)   │     │ │ method: hybrid   │   │
│   └─────────────────┘     │ │ time: 87ms       │   │
│                           │ └──────────────────┘   │
│                                     │
│   What to Watch For:                │
│   ✓ Confidence > 85% (high)         │
│   ✓ Time < 200ms (fast)             │
│   ✓ Method shows "hybrid" (good)    │
│                                     │
│   Problem Signs:                    │
│   ✗ Confidence < 60% (uncertain)    │
│   ✗ Time > 500ms (slow)             │
│   ✗ Showing "rule-based" (no ML)    │
│                                     │
└─────────────────────────────────────┘
```

### End of Shift - Review Statistics

```
┌─────────────────────────────────────┐
│ Review Inference Monitor Stats      │
├─────────────────────────────────────┤
│                                     │
│ Question: Is accuracy acceptable?   │
│                                     │
│ ┌──────────────────────────────┐    │
│ │ Statistics:                  │    │
│ │ • Total Scans: 247           │    │
│ │ • Success Rate: 98.4%        │    │
│ │ • Avg Time: 94ms             │    │
│ │                              │    │
│ │ • Good: 189 (76%)            │    │
│ │ • Porosity: 51 (21%)         │    │
│ │ • Undercut: 7 (3%)           │    │
│ └──────────────────────────────┘    │
│                                     │
│ YES ✓  → Keep current model         │
│ NO ✗   → Consider training new      │
│                                     │
└─────────────────────────────────────┘
```

---

## Training a New Model (Step-by-Step)

### Preparation Phase (1-2 hours)

```
┌────────────────────────────────────────────┐
│ Step 1: Collect Training Images            │
├────────────────────────────────────────────┤
│                                            │
│ YOU NEED:                                  │
│ ☐ 100+ good weld photos                    │
│ ☐ 50+ porosity (pits) photos               │
│ ☐ 50+ undercut photos                      │
│ ☐ Clear lighting, consistent background    │
│                                            │
│ HOW TO ORGANIZE:                           │
│ training_data/                             │
│ ├─ good/                                   │
│ │  ├─ weld_001.jpg                         │
│ │  ├─ weld_002.jpg                         │
│ │  └─ ...                                  │
│ ├─ porosity/                               │
│ │  ├─ defect_001.jpg                       │
│ │  └─ ...                                  │
│ └─ undercut/                               │
│    ├─ defect_001.jpg                       │
│    └─ ...                                  │
│                                            │
│ QUALITY CHECKLIST:                         │
│ ☑ All images in focus                      │
│ ☑ Defects clearly visible                  │
│ ☑ Good lighting                            │
│ ☑ Consistent scale                         │
│ ☑ No blurry images                         │
│                                            │
└────────────────────────────────────────────┘
```

### Training Phase (30 min to 3 hours)

```
┌────────────────────────────────────────────┐
│ Step 2: Start Training                     │
├────────────────────────────────────────────┤
│                                            │
│ IN APP:                                    │
│ 1. Click Training Dashboard ✨             │
│                                            │
│ 2. Select Model Type                       │
│    ┌─────────────────────────────────┐    │
│    │ ☐ MobileNetV2 (RECOMMENDED)      │    │
│    │   • Speed: 30-45 min             │    │
│    │   • Accuracy: 75-80%             │    │
│    │   • Size: 18.5 MB                │    │
│    │                                 │    │
│    │ ☐ ResNet50 (BEST)                │    │
│    │   • Speed: 2-3 hours             │    │
│    │   • Accuracy: 90-95%             │    │
│    │   • Size: 104 MB                 │    │
│    └─────────────────────────────────┘    │
│                                            │
│ 3. Leave settings as default               │
│    Epochs: 100  ✓                          │
│    Batch: 32    ✓                          │
│    LR: 0.001    ✓                          │
│                                            │
│ 4. Click [START TRAINING]                  │
│                                            │
│ 5. WAIT - do not close window!             │
│                                            │
└────────────────────────────────────────────┘
```

### Monitoring Phase (variable)

```
┌────────────────────────────────────────────┐
│ Step 3: Watch Progress                     │
├────────────────────────────────────────────┤
│                                            │
│ YOUR WINDOW:                               │
│                                            │
│ ┌──────────────────────────────────────┐   │
│ │ ResNet50_v2                   RUNNING │   │
│ │                                       │   │
│ │ Progress: ████████░░░░░░░░░░  45%    │   │
│ │ Epoch: 45 / 100                     │   │
│ │                                       │   │
│ │ Loss:     0.2145   ↓ GOOD            │   │
│ │ Accuracy: 89.2%    ↑ GOOD            │   │
│ │                                       │   │
│ │ Est. Time: 2h 30m remaining         │   │
│ │                                       │   │
│ │ [Pause] [Download] [View Metrics]    │   │
│ └──────────────────────────────────────┘   │
│                                            │
│ WHAT LOOKS GOOD:                           │
│ ✓ Loss number going DOWN (0.5→0.2)        │
│ ✓ Accuracy going UP (50%→89%)             │
│ ✓ Progress bar moving steadily            │
│                                            │
│ WHAT LOOKS BAD:                            │
│ ✗ Loss stays same or goes UP              │
│ ✗ Accuracy stuck at 50% (random)          │
│ ✗ Progress bar not moving                 │
│ ✗ Training stopped suddenly               │
│                                            │
│ WHAT TO DO IF BAD:                         │
│ → Check data labels are correct            │
│ → Verify images are clear/visible         │
│ → May need more training data             │
│ → Try again with more epochs              │
│                                            │
└────────────────────────────────────────────┘
```

### Completion Phase (5 minutes)

```
┌────────────────────────────────────────────┐
│ Step 4: Complete & Download                │
├────────────────────────────────────────────┤
│                                            │
│ WHEN TRAINING FINISHES:                    │
│                                            │
│ ┌──────────────────────────────────────┐   │
│ │ ResNet50_v2                COMPLETED ✓   │
│ │                                       │   │
│ │ Final Accuracy: 92.1%                 │   │
│ │ Final Loss: 0.0954                    │   │
│ │ Training Time: 2h 47m                 │   │
│ │                                       │   │
│ │ [Download Model] [View Report]        │   │
│ └──────────────────────────────────────┘   │
│                                            │
│ CLICK [Download Model]                     │
│                                            │
│ Browser will download:                     │
│ └─ ResNet50_v2.zip (200+ MB)              │
│    ├─ Model weights                       │
│    ├─ Performance metrics                 │
│    └─ Training history                    │
│                                            │
└────────────────────────────────────────────┘
```

---

## Deploying a Model (5 minutes)

```
┌────────────────────────────────────────────┐
│ Step 5: Deploy New Model                   │
├────────────────────────────────────────────┤
│                                            │
│ 1. Open Model Management 📊                │
│                                            │
│ 2. View All Models:                        │
│                                            │
│    ┌─ Current Model (DEPLOYED) ✓ ──┐      │
│    │ MobileNetV2_v1                  │      │
│    │ Accuracy: 78.5%                 │      │
│    │ Status: DEPLOYED [Delete]       │      │
│    └─────────────────────────────────┘      │
│                                            │
│    ┌─ New Model (NOT DEPLOYED)      ──┐    │
│    │ ResNet50_v2                       │    │
│    │ Accuracy: 92.1%   ← BETTER! ↑    │    │
│    │ Status: IDLE                      │    │
│    │ [Deploy] [Download] [Delete]      │    │
│    └───────────────────────────────────┘    │
│                                            │
│ 3. QUESTION: Is new accuracy better?       │
│    92.1% > 78.5% ✓ YES                     │
│                                            │
│ 4. Click [DEPLOY] on new model             │
│                                            │
│ 5. Confirmation (2-5 seconds):             │
│    ✓ Model deployed successfully!          │
│    ✓ Status changed to "DEPLOYED"          │
│    ✓ Old model now "IDLE"                  │
│                                            │
│ 6. Test new model:                         │
│    • Do 10-20 scans                        │
│    • Watch Inference Monitor              │
│    • Verify accuracy looks good            │
│    • If bad: Re-deploy old model           │
│                                            │
└────────────────────────────────────────────┘
```

---

## Decision Trees

### When to Train New Model?

```
Question: Is current accuracy acceptable?

                    ↓
         ┌──────────┴──────────┐
         │                     │
        YES                    NO
         │                     │
         ↓                     ↓
    Keep current         Should we train?
    model active         
                         ├─ Have 100+ images?
                         │  ├─ YES → Train ✓
                         │  └─ NO  → Collect data
                         │
                         ├─ Have time?
                         │  ├─ YES → Train ✓
                         │  └─ NO  → Wait
                         │
                         └─ Have GPU?
                            ├─ YES → Use ResNet50
                            └─ NO  → Use MobileNetV2
```

### Which Model to Deploy?

```
Question: Which model is better?

Model A: 78% accuracy          Model B: 92% accuracy
(MobileNetV2)                  (ResNet50)
         ↓                             ↓
      Compare:
      Difference = 92% - 78% = 14%

Is difference > 5%?
         ↓
        YES ✓
         ↓
    Deploy Model B
    (re-deploy old if problems)
```

### How Much Data Is Enough?

```
Target Accuracy → Required Images

60-70%  ←──  50 images
↓
70-80%  ←──  100 images  ← MINIMUM START
↓
80-85%  ←──  300 images
↓
85-90%  ←──  500 images  ← RECOMMENDED
↓
90-95%  ←──  1000+ images ← BEST RESULTS
↓
95%+    ←──  2000+ images ← EXCELLENT
```

---

## Troubleshooting Flowchart

### "The accuracy is terrible" (< 60%)

```
Problem: Accuracy stuck at ~60% (random guessing)

                    ↓
         Are labels correct?
         ├─ NO  → Fix labels ✓
         └─ YES → Continue
         
                    ↓
         Are images clear?
         ├─ NO  → Retake photos
         └─ YES → Continue
         
                    ↓
         Do you have enough data?
         ├─ <50 images → Collect more
         ├─ 50-100     → Might be borderline
         └─ 100+       → Continue
         
                    ↓
         Try training with:
         ├─ Lower learning rate
         ├─ More epochs
         ├─ More data
         └─ Different model
         
                    ↓
         Still low? → Check for data quality issues
```

### "Training hasn't started" (No progress)

```
Problem: Training shows 0% progress

                    ↓
         Is your GPU working?
         ├─ Type: nvidia-smi
         ├─ Should show NVIDIA GPU
         └─ If not → Install GPU drivers
         
                    ↓
         Is your data folder correct?
         ├─ Check: /data/training_frames/ exists
         ├─ Check: Has good/ porosity/ undercut/
         └─ If not → Create and populate
         
                    ↓
         Is GPU memory sufficient?
         ├─ Type: nvidia-smi
         ├─ Need 4GB+ free
         ├─ Close Chrome/other apps
         └─ Reduce batch size if needed
         
                    ↓
         Restart training with:
         ├─ Smaller batch size
         ├─ Smaller model (MobileNetV2)
         └─ CPU mode if no GPU

                    ↓
         Check terminal for error messages
         └─ Copy and search online for help
```

---

## Success Checklist

### Before Starting Training
```
☐ Have 100+ labeled images collected
☐ Images organized in correct folders
☐ Image quality is good (clear, well-lit)
☐ Labels are verified correct
☐ Have 1-3 hours free (or GPU to speed up)
☐ Backend server is running
☐ Frontend is accessible
```

### During Training
```
☐ Loss is decreasing (0.5 → 0.2 → 0.05)
☐ Accuracy is increasing (50% → 75% → 90%)
☐ Progress bar moving steadily
☐ No error messages in terminal
☐ GPU memory not exceeded
☐ Don't interrupt training
```

### After Training
```
☐ Final accuracy > 85%
☐ Compare with previous model
☐ Download completed successfully
☐ Test on known-good samples
☐ Only deploy if better than current
☐ Monitor for first 100+ scans
```

---

## Quick Reference Card

Print this out!

```
╔══════════════════════════════════════════════════╗
║         AI & TRAINING QUICK REFERENCE            ║
╚══════════════════════════════════════════════════╝

DAILY CHECK:
✓ Open Inference Monitor (sidebar, 🧠)
✓ See green "Connected" status
✓ Check success rate > 95%

WHEN ACCURACY IS LOW:
1. Collect more training images
2. Verify labels are correct
3. Train new model (Model Training ✨)
4. Compare accuracy (Model Management 📊)
5. Deploy if better

MODEL CHOICE:
• MobileNetV2: Fast (30 min), OK accuracy (75-80%)
• ResNet50: Slow (2-3 hrs), Great accuracy (90-95%)

TRAINING SETTINGS (Keep Default):
Epochs: 100      ← Correct
Batch: 32        ← Correct  
LR: 0.001        ← Correct

SUCCESS SIGNS:
✓ Loss decreasing
✓ Accuracy increasing
✓ Progress bar moving
✓ No errors shown

PROBLEM? 
✗ Check GPU: nvidia-smi
✗ Check data labels
✗ Check image quality
✗ See troubleshooting guide

DEPLOY ONLY IF:
✓ Accuracy > 85%
✓ Better than current by 5%+
✓ Tested on known samples
```

---

## Tips & Tricks

### Speed Up Training
```
✓ Use GPU (nvidia-smi shows GPU)
✓ Close other apps (Chrome, etc.)
✓ Use MobileNetV2 first (faster)
✓ Start with small dataset (test)
✓ Increase batch size (32 → 64)
```

### Improve Accuracy
```
✓ Collect MORE data (100 → 500+)
✓ Use ResNet50 (more powerful)
✓ Train for longer (100 → 200 epochs)
✓ Improve image quality (lighting, focus)
✓ Balance classes (equal good/defect)
```

### Avoid Problems
```
✗ Don't use blurry images
✗ Don't mislabel data
✗ Don't interrupt training
✗ Don't deploy without testing
✗ Don't delete working model
```

---

**Questions? See `AI_AND_TRAINING_GUIDE.md` for complete details.**

**Start now! 🚀**

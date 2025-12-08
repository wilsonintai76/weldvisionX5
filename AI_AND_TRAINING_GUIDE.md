# 🤖 AI & Training In-App Guide

Complete guide to using the integrated AI inference monitoring and model training features in WeldEval X5.

---

## Table of Contents

1. [Overview](#overview)
2. [Inference Monitor](#inference-monitor)
3. [Model Training](#model-training)
4. [Model Management](#model-management)
5. [Workflows](#workflows)
6. [Best Practices](#best-practices)
7. [Troubleshooting](#troubleshooting)
8. [Performance Tips](#performance-tips)

---

## Overview

The WeldEval X5 includes three integrated AI modules:

### 1. **Inference Monitor** 🧠
Real-time monitoring of weld defect detection with live statistics and confidence tracking.

### 2. **Model Training** ✨
Train custom AI models on your weld data using desktop GPU for 2-4x faster training.

### 3. **Model Management** 📊
Manage, deploy, compare, and version all your trained models.

### Hybrid Architecture

```
RDK Camera Input (RGB + Depth)
        ↓
    Hybrid Inference Engine
    ├─ Rule-Based Detection (always available)
    ├─ Machine Learning (if model available)
    └─ Smart Combination (best of both)
        ↓
    Inference Monitor (this app)
    ├─ Display results
    ├─ Track statistics
    └─ Archive history
```

---

## Inference Monitor

### Access
Click **"Inference Monitor"** in the left sidebar (🧠 icon).

### Features

#### 1. **Live Detection Display**
Shows the most recent weld defect detection result:
- **Defect Class**: good / porosity / undercut
- **Confidence Score**: 0-100% certainty
- **Detection Method**: 
  - 🔵 rule-based (fast, always available)
  - 🟢 ml (machine learning model)
  - 🟣 hybrid (both combined)
- **Inference Time**: milliseconds to process
- **Timestamp**: When the detection occurred

#### 2. **Statistics Dashboard**
Live metrics updated every detection:

| Metric | What It Means |
|--------|---------------|
| Total Inferences | All detections processed |
| Average Time | Typical inference latency |
| Success Rate | % of successful detections |
| Detections by Class | Count per defect type |

**Example Stats:**
```
Total Inferences: 1,234
Average Time: 92ms
Success Rate: 98.5%

Detections:
├─ Good: 892 (72.3%)
├─ Porosity: 287 (23.2%)
└─ Undercut: 55 (4.5%)
```

#### 3. **Inference History**
Scrollable list of last 20 detections showing:
- Defect class
- Confidence
- Method used
- Timestamp

#### 4. **Connection Status**
Indicator showing if inference engine is ready:
- 🟢 **Connected**: Engine ready, processing inferences
- 🔴 **Disconnected**: Check backend connection
- 🟡 **Model Loading**: Wait a moment

### How to Use

#### Monitoring Live Scanning
1. Open **Inference Monitor**
2. In another window/tab, run a scan
3. Watch results appear in real-time
4. Check statistics after several scans

#### Understanding Confidence
- **High (>0.85)**: Very confident in detection
- **Medium (0.70-0.85)**: Reasonably confident, possibly hybrid
- **Low (<0.70)**: Uncertain, rule-based fallback used

#### Identifying Detection Method
- **Rule-Based**: Fast, always available, 60-70% accurate
  - Good for: Quick feedback, no GPU required
  - Used when: ML model not available or not confident
  
- **ML (MobileNetV2)**: Smart model, 75-80% accurate
  - Good for: Better accuracy, still fast
  - Used when: Model trained and available
  
- **Hybrid**: Both combined, 80-85% accurate
  - Good for: Best results, most reliable
  - Used when: Confidence is borderline

### Typical Workflow
```
1. Start Live Scanner in another tab
2. Place first weld specimen
3. Click Scan in Scanner view
4. Watch Inference Monitor for result
5. Repeat for multiple specimens
6. Review Statistics section
7. Check if accuracy is acceptable
   ├─ YES → Continue using current model
   └─ NO → Train better model (see Model Training)
```

---

## Model Training

### Access
Click **"Model Training"** in the left sidebar (✨ icon).

### Before You Start

**Requirements:**
- [ ] 100+ labeled weld images (good/porosity/undercut)
- [ ] Image quality: clear, well-lit, consistent background
- [ ] Labels: organized by defect class
- [ ] Time: 30 min - 3 hours depending on model & GPU

**Hardware:**
- **Desktop with GPU**: 2-3 hours for ResNet50
- **Desktop without GPU**: 20+ hours for ResNet50
- **Desktop with GPU**: 30-45 min for MobileNet

### Step-by-Step Guide

#### Step 1: Select Model Type

Choose between two models:

**MobileNetV2 (Recommended for Start)**
```
Size: 18.5 MB
Speed: Fast (100-150ms per scan)
Accuracy: 75-80%
Training: 30-45 min on GPU
Use For: Quick improvements, edge deployment
Deploy To: RDK X5 directly
```

**ResNet50 (For Best Accuracy)**
```
Size: 100+ MB
Speed: Medium (50ms GPU, 200ms CPU)
Accuracy: 90-95%
Training: 2-3 hours on GPU
Use For: Production quality, maximum accuracy
Deploy To: Desktop server or high-end hardware
```

**Selection Guide:**
- First time training? → **MobileNetV2**
- Need best accuracy? → **ResNet50 on GPU**
- Have limited GPU? → **MobileNetV2**
- Production environment? → **ResNet50**

#### Step 2: Configure Training Parameters

```
┌─────────────────────────────────┐
│ Training Configuration          │
├─────────────────────────────────┤
│ Epochs: [100] ═════════         │
│   └─ How many passes over data  │
│                                 │
│ Batch Size: [32] ═════════      │
│   └─ Images per training step   │
│                                 │
│ Learning Rate: [0.001] ═════    │
│   └─ How fast model learns      │
└─────────────────────────────────┘
```

**Parameter Explanations:**

| Parameter | Range | Recommended | What It Does |
|-----------|-------|-------------|--------------|
| **Epochs** | 10-500 | 50-100 | How many times to go through all data |
| **Batch Size** | 8-128 | 32 | How many images to show before updating weights |
| **Learning Rate** | 0.00001-0.1 | 0.001 | How much to adjust weights each step |

**Tuning Tips:**
```
If accuracy is too low:
├─ Increase epochs (50 → 100)
├─ Lower learning rate (0.001 → 0.0005)
└─ Check image quality

If training is too slow:
├─ Decrease batch size (128 → 64)
├─ Increase learning rate (0.001 → 0.002)
└─ Use GPU if available

If overfitting detected:
├─ Increase batch size
├─ Lower learning rate
└─ Add more training data
```

#### Step 3: Start Training

1. **Select Model**: Click MobileNetV2 or ResNet50 card
2. **Set Parameters**: Adjust epochs, batch size, LR
3. **Click "Start Training"** button
4. **Wait** for confirmation message

Expected message:
```
✓ Training started successfully
Job ID: job_1735002345123
Monitor progress below
```

#### Step 4: Monitor Training Progress

**Active Job Display:**
```
┌──────────────────────────────────────┐
│ ResNet50_001 | RUNNING               │
├──────────────────────────────────────┤
│ Progress: ████████░░░░░░░░░░ 45%     │
│                                      │
│ Epoch 45 of 100                      │
│ Loss: 0.2145 (going down ✓)          │
│ Accuracy: 89.2% (improving ✓)        │
│ Estimated Time: 2h 30m               │
│                                      │
│ [Pause] [Download] [View Metrics]    │
└──────────────────────────────────────┘
```

**What to Watch For:**
- **Loss**: Should decrease over time
  - ✓ Good: 0.5 → 0.3 → 0.2 → 0.1
  - ✗ Bad: Stays flat or increases

- **Accuracy**: Should increase over time
  - ✓ Good: 60% → 70% → 80% → 85%
  - ✗ Bad: Stays flat or decreases

- **ETA**: Estimated completion time
  - Updates as training progresses
  - Use to plan your time

#### Step 5: Download Trained Model

When training completes:

```
┌──────────────────────────────────────┐
│ ResNet50_001 | COMPLETED ✓           │
├──────────────────────────────────────┤
│ Final Accuracy: 92.1%                │
│ Final Loss: 0.0954                   │
│ Training Time: 2h 47m                │
│                                      │
│ [Download Model] [View Full Report]  │
└──────────────────────────────────────┘
```

Clicking **"Download Model"** saves:
- Model weights (100+ MB for ResNet50)
- Model metadata
- Training history
- Performance metrics

### Understanding Training Metrics

#### Loss
```
Loss = Error between prediction and actual

Good Loss Curve:
├─ Epoch 1: Loss 0.8
├─ Epoch 50: Loss 0.2
└─ Epoch 100: Loss 0.05
  └─ Getting better ✓

Bad Loss Curve:
├─ Epoch 1: Loss 0.5
├─ Epoch 50: Loss 0.5
└─ Epoch 100: Loss 0.5
  └─ Not learning ✗

What to do if loss is high:
├─ Check image quality
├─ Verify labels are correct
├─ Increase training time
└─ Use more data
```

#### Accuracy
```
Accuracy = % of correct predictions

Good Accuracy Progression:
├─ Epoch 1: 60% (random guessing)
├─ Epoch 50: 75% (learning)
└─ Epoch 100: 90% (expert level)

Accuracy per Class:
├─ Good: 92% (knows good welds)
├─ Porosity: 88% (identifies flaws)
└─ Undercut: 85% (detects undercuts)

Target Accuracy:
├─ Acceptable: 80%+
├─ Good: 85%+
└─ Excellent: 90%+
```

### Common Training Issues

#### Training Stopped Early
**Symptom:** Training stops at epoch 50/100

**Reasons:**
- Early stopping activated (model not improving)
- GPU ran out of memory
- Keyboard interrupt (Ctrl+C)

**Solution:**
- Check if accuracy plateaued (good - model learned)
- Reduce batch size if OOM error
- Restart and increase epochs if interrupted

#### Accuracy Not Improving
**Symptom:** Accuracy stays at 60% (random)

**Reasons:**
- Labels are wrong
- Images are too different from production
- Learning rate is too high
- Not enough data

**Solution:**
```
1. Check 10 random images
   ├─ Verify labels match images
   └─ Ensure image quality is good

2. If labels correct:
   ├─ Decrease learning rate (0.001 → 0.0005)
   ├─ Increase epochs (50 → 200)
   └─ Add more diverse training data

3. If still low:
   └─ Collect 500+ more labeled images
```

#### Very Slow Training
**Symptom:** Training takes 12+ hours

**Reasons:**
- Using CPU instead of GPU
- GPU not properly detected
- Batch size too small

**Solution:**
```bash
# Check if GPU is available
nvidia-smi  # Shows GPU status

# If no GPU:
├─ Install CUDA toolkit
├─ Reinstall PyTorch with GPU support
└─ Use smaller model (MobileNetV2)

# If GPU available but slow:
├─ Increase batch size (32 → 64)
├─ Reduce epochs initially
└─ Update GPU drivers
```

---

## Model Management

### Access
Click **"Model Management"** in the left sidebar (📊 icon).

### View All Models

**Model List Display:**
```
┌──────────────────────────────────────┐
│ mobilenet_v2_001                     │
├──────────────────────────────────────┤
│ v1.0.0 | Edge | 18.5MB               │
│ Accuracy: 78.5% | F1: 0.76           │
│ Created: 2024-01-15                  │
│ Status: DEPLOYED ✓                   │
│                                      │
│ [Download] [Delete]                  │
└──────────────────────────────────────┘

┌──────────────────────────────────────┐
│ resnet50_trained_001                 │
├──────────────────────────────────────┤
│ v1.0.0 | Desktop | 104MB             │
│ Accuracy: 92.1% | F1: 0.91           │
│ Created: 2024-01-14                  │
│ Status: IDLE                         │
│                                      │
│ [Deploy] [Download] [Delete]         │
└──────────────────────────────────────┘
```

### Deploy a Model

**What it means:**
- Model becomes active for inference
- All scans will use this model
- Previous model becomes idle

**How to deploy:**
1. Find the model you trained
2. Click **"Deploy"** button
3. Wait for confirmation (2-5 seconds)
4. See "DEPLOYED" status appear

**Best Practice:**
```
1. Train model offline (dev environment)
2. Test accuracy on sample data
3. Only deploy if accuracy > 85%
4. Monitor Inference Monitor for 50+ scans
5. If accuracy good, keep deployed
6. If not, deploy different model
```

### Compare Models

**When to compare:**
- Deciding which model to deploy
- Tracking improvements over time
- Justifying training time investment

**How to compare:**
1. Check 2 or more models
2. Click **"Show Comparison"**
3. View side-by-side metrics

**Comparison Table:**
```
┌──────────────┬──────────────┬──────────────┐
│ Metric       │ MobileNet    │ ResNet50     │
├──────────────┼──────────────┼──────────────┤
│ Accuracy     │ 78.5%        │ 92.1%        │
│ F1-Score     │ 0.76         │ 0.91         │
│ Size         │ 18.5MB       │ 104MB        │
│ Type         │ Edge         │ Desktop      │
│ Speed        │ 100-150ms    │ 30-50ms (GPU)│
│ Created      │ 2024-01-15   │ 2024-01-14   │
└──────────────┴──────────────┴──────────────┘
```

### Download Models

**Why download:**
- Backup for safety
- Share with colleagues
- Deploy to other systems
- Archive old versions

**What gets downloaded:**
```
model_name.zip (200MB+)
├─ model_weights.onnx
├─ model_metadata.json
│  ├─ accuracy: 0.921
│  ├─ loss: 0.0954
│  ├─ f1_score: 0.91
│  └─ training_time: 2h 47m
└─ training_history.json
   ├─ loss_per_epoch: [...]
   └─ accuracy_per_epoch: [...]
```

### Delete Models

**When to delete:**
- No longer needed
- Taking up storage
- Outdated version
- Poor performance

**Warning:** Cannot be undone!

---

## Workflows

### Workflow 1: Monitor Inference Only

**Goal:** Track current model performance

**Time:** 5 minutes

**Steps:**
1. Open **Inference Monitor**
2. Run 10-20 scans in **Live Scanner**
3. Watch statistics update
4. Review detection accuracy
5. Note any patterns

**Success Criteria:**
- See 100% success rate
- Average time < 200ms
- Accuracy looks acceptable

---

### Workflow 2: Train First Model

**Goal:** Create your first trained model

**Time:** 2-4 hours including data prep

**Steps:**
1. **Prepare Data** (1-2 hours)
   - Take 100+ photos of welds
   - Label as: good, porosity, or undercut
   - Organize into folders

2. **Collect Data** (30 min)
   - Go to Model Training
   - Upload all labeled images
   - Verify upload completed

3. **Start Training** (30 min setup)
   - Select MobileNetV2 (faster)
   - Set to 50 epochs
   - Click Start Training

4. **Monitor** (variable)
   - Watch progress bar
   - Check loss decreases
   - Check accuracy increases

5. **Evaluate** (15 min)
   - Review final metrics
   - Download model
   - Test on known samples

6. **Deploy** (5 min)
   - Go to Model Management
   - Click Deploy
   - Verify "DEPLOYED" status

---

### Workflow 3: Improve Model Accuracy

**Goal:** Increase accuracy from 75% to 90%+

**Time:** 4-8 hours

**Steps:**
1. **Analyze Current Model**
   - Open Inference Monitor
   - Run 50+ scans
   - Note which classes are wrong (if any)

2. **Collect More Data**
   - Focus on poorly detected classes
   - Collect 200+ additional images
   - Ensure diverse lighting/angles

3. **Train with More Data**
   - Select ResNet50 (more powerful)
   - Set to 100 epochs
   - Start training

4. **Compare Results**
   - Download new model
   - Go to Model Management
   - Compare with old model
   - Check accuracy improvement

5. **Deploy if Better**
   - If new accuracy > old + 5%
   - Click Deploy on new model
   - Monitor with 50+ scans

---

### Workflow 4: A/B Testing Models

**Goal:** Compare two models on real data

**Time:** 2 hours per model

**Steps:**
1. **Deploy Model A**
   - Go to Model Management
   - Deploy first model
   - Run 100+ scans
   - Record accuracy

2. **Deploy Model B**
   - Switch to second model
   - Run 100+ scans
   - Record accuracy

3. **Compare**
   - Model A: 87% accuracy
   - Model B: 91% accuracy
   - Winner: Model B

4. **Keep Winner**
   - Re-deploy Model B
   - Delete Model A if no longer needed

---

## Best Practices

### Data Collection

**✓ Do This:**
```
├─ Take photos from multiple angles
├─ Vary lighting conditions
├─ Include defect examples
├─ Include perfect weld examples
├─ Label consistently
├─ Keep organized folders
└─ Document your process
```

**✗ Don't Do This:**
```
├─ Take photos in dim light
├─ Use only good welding examples
├─ Mislabel defects
├─ Mix scales (tiny + huge)
├─ Store randomly named files
└─ Reuse same image multiple times
```

### Training Best Practices

**✓ Quality Checklist:**
```
Before training:
├─ [ ] 100+ images minimum
├─ [ ] Labels verified correct
├─ [ ] Images similar to production
├─ [ ] All classes represented
├─ [ ] GPU available (if large model)
└─ [ ] Backup original data

During training:
├─ [ ] Monitor loss decreasing
├─ [ ] Monitor accuracy increasing
├─ [ ] Watch for early stopping
└─ [ ] Don't interrupt training

After training:
├─ [ ] Review final metrics
├─ [ ] Test on holdout data
├─ [ ] Compare with previous model
└─ [ ] Only deploy if better
```

### Model Deployment Best Practices

**✓ Safe Deployment:**
```
1. Train offline first
2. Test thoroughly
3. Compare with current model
4. Gradually roll out (if possible)
5. Monitor closely first week
6. Keep old model as fallback
7. Document what changed
```

**✗ Risky Deployment:**
```
├─ Deploying without testing
├─ Not monitoring afterwards
├─ Deleting old model immediately
├─ Training only on small dataset
└─ Ignoring low accuracy
```

---

## Troubleshooting

### Common Issues & Solutions

#### Issue: "Connection Failed" in Inference Monitor

**Problem:** Red disconnected status

**Causes:**
- Backend not running
- Port mismatch
- Network issue

**Solutions:**
```bash
# Check backend is running
# Terminal 1: cd backend && python app.py
# Should see: "Running on http://127.0.0.1:5000"

# Check frontend port
# Terminal 2: npm run dev
# Should see: "Local: http://localhost:5173"

# If still failing:
# Clear browser cache: Ctrl+Shift+Del
# Restart both terminals
# Check firewall settings
```

#### Issue: Training Starts but Shows No Progress

**Problem:** Progress stuck at 0%

**Causes:**
- Data loading issue
- GPU memory problem
- Incorrect dataset path

**Solutions:**
```
1. Check GPU available:
   # Run: nvidia-smi
   # Should show NVIDIA GPU

2. Check dataset path:
   # Verify images exist in /data/training_frames
   # Check labels.json is valid JSON

3. Reduce batch size:
   # Go back to config
   # Change batch from 32 → 16
   # Restart training

4. Check console for errors:
   # Terminal 1 (backend)
   # Look for red error messages
   # Copy error and search online
```

#### Issue: Very Low Accuracy (< 60%)

**Problem:** Model not learning, accuracy random

**Causes:**
- Wrong labels
- Poor image quality
- Insufficient data
- Learning rate too high

**Solutions:**
```
Priority 1: Check labels
├─ Review 10 random images
├─ Verify labels match images
└─ Fix any wrong labels

Priority 2: Check image quality
├─ All images clear and visible
├─ Consistent background
├─ Proper lighting
└─ Defects clearly visible

Priority 3: Adjust training
├─ Decrease learning rate
├─ Increase training time (epochs)
├─ Use more training data
└─ Try different model architecture

Priority 4: Collect more data
└─ If still low, collect 200+ more images
```

#### Issue: Out of Memory (OOM) Error

**Problem:** Training crashes with memory error

**Causes:**
- Batch size too large
- Model too large for GPU
- Other apps using GPU

**Solutions:**
```bash
# Check GPU memory:
nvidia-smi

# If less than 4GB free:
├─ Close other apps (Chrome, etc)
├─ Reduce batch size (32 → 16)
└─ Switch to smaller model (MobileNet)

# If still failing:
├─ Restart computer
├─ Update GPU drivers
└─ Use CPU training (slower)
```

#### Issue: Model Download Failed

**Problem:** Download hangs or shows error

**Causes:**
- File too large
- Network interruption
- Browser settings

**Solutions:**
```
1. Try again after waiting 30 sec
2. Try different browser
3. Check available disk space
4. Disable VPN if using
5. Check antivirus isn't blocking
6. Use different download folder
```

---

## Performance Tips

### Speeding Up Training

**On Desktop with GPU:**
```
Baseline (default settings):
├─ MobileNetV2: 45 minutes
├─ ResNet50: 3 hours
└─ Using: Batch 32, 100 epochs

Optimized:
├─ MobileNetV2: 30 minutes (-33%)
├─ ResNet50: 2 hours (-33%)
└─ Using: Batch 64, reduced epochs early

Tips:
├─ Use latest GPU drivers
├─ Close other GPU apps
├─ Ensure good ventilation (GPU stays cool)
├─ Use latest PyTorch version
└─ Use GPU with 6GB+ VRAM
```

**On Desktop without GPU (CPU only):**
```
⚠️ Warning: Much slower!

Realistic times:
├─ MobileNetV2: 4-6 hours
├─ ResNet50: 20+ hours
└─ Not recommended for ResNet50

Optimization:
├─ Use MobileNetV2 only
├─ Reduce epochs (50 instead of 100)
├─ Increase batch size (64 instead of 32)
├─ Use SSD (faster than HDD)
└─ Consider borrowing a GPU computer
```

### Improving Accuracy

**Data Quality (most important):**
```
Rule: Garbage in = Garbage out

Good training data:
├─ 100+ labeled images (minimum)
├─ 500+ for best results
├─ Diverse angles and lighting
├─ All classes well represented
├─ Clear labels
├─ Similar to production data
└─ Good image resolution

Effect on accuracy:
├─ 50 images: 60-70% accuracy
├─ 100 images: 75-80% accuracy
├─ 500 images: 85-90% accuracy
└─ 2000+ images: 90-95% accuracy
```

**Model Selection:**
```
For accuracy goals:

75-80% target:
└─ MobileNetV2 ✓ (fast, good enough)

85-90% target:
└─ ResNet50 ✓ (slow, best accuracy)

95%+ target:
└─ ResNet50 + 1000+ images ✓
```

**Training Optimization:**
```
For better accuracy:

├─ More epochs (50 → 100)
├─ Lower learning rate (0.001 → 0.0005)
├─ Larger batch size (32 → 64)
├─ More training data (100 → 500+)
├─ Better labeled data
├─ Longer training time
└─ Ensemble multiple models
```

### Monitoring Resource Usage

**Check System Performance:**
```
While training:

GPU Usage:
├─ nvidia-smi (in terminal)
├─ Should show 80-90% utilization
├─ Temperature should stay < 85°C
└─ Memory should be mostly used

CPU Usage:
├─ Task Manager (Windows)
├─ Should be 20-50%
└─ Don't need to be maxed out

Memory (RAM):
├─ Should have 4GB+ available
├─ Monitor in Task Manager
└─ If low, close other apps

Disk:
├─ Training writes to disk
├─ Need 5GB+ free space
├─ SSD much faster than HDD
└─ Check disk temperature
```

---

## Quick Reference

### Keyboard Shortcuts
```
(Available soon)
Ctrl+M: Switch to Model Management
Ctrl+T: Switch to Training
Ctrl+I: Switch to Inference Monitor
```

### Common Parameters

**For Quick Testing:**
```
Model: MobileNetV2
Epochs: 50
Batch Size: 32
Learning Rate: 0.001
Expected Time: 30-45 min
Expected Accuracy: 75-80%
```

**For Production Quality:**
```
Model: ResNet50
Epochs: 100
Batch Size: 64 (if GPU has 8GB+)
Learning Rate: 0.001
Expected Time: 2-3 hours
Expected Accuracy: 90-95%
```

**For Maximum Accuracy:**
```
Model: ResNet50
Epochs: 200
Batch Size: 64
Learning Rate: 0.0005
Expected Time: 4-6 hours
Expected Accuracy: 92-96%
```

### Support & Help

**Within the App:**
- Click **"Help & Guide"** button (top right)
- View tooltips (hover over icons)
- Check sidebar help panels

**Online Resources:**
- Backend API: See backend/api/
- Training Code: See backend/models/
- Frontend: See components/

**Reporting Issues:**
```
When reporting a problem, provide:
├─ What you were doing
├─ What went wrong
├─ Error message (if any)
├─ System info (GPU, CPU, RAM)
├─ Training parameters used
└─ Steps to reproduce
```

---

## Summary

### Three Simple Steps to AI Success

**1. Monitor** 🧠
- Use Inference Monitor to track current accuracy
- See real-time detection results
- Identify improvement opportunities

**2. Train** ✨
- Collect labeled weld images
- Select model (MobileNetV2 or ResNet50)
- Start training, monitor progress
- Download when complete

**3. Deploy** 📊
- Compare with existing model
- Deploy if accuracy improved
- Monitor results on production data

### Expected Results Timeline

```
Week 1: Get comfortable with interface
├─ Monitor current inference
├─ Understand metrics
└─ Start collecting data

Week 2: Train first model
├─ Prepare 100+ labeled images
├─ Run first training
├─ Evaluate results
└─ Deploy if good

Week 3-4: Improve accuracy
├─ Collect more diverse data
├─ Train ResNet50 model
├─ Compare multiple models
└─ Fine-tune hyperparameters

Ongoing:
├─ Monitor inference quality
├─ Retrain periodically
├─ Collect edge cases
└─ Continuous improvement
```

### Success Metrics

**You'll know it's working when:**
- ✅ Inference Monitor shows 85%+ accuracy
- ✅ Average inference time < 200ms
- ✅ No failed detections
- ✅ New trained model better than previous
- ✅ System improving over time

---

## Next Steps

1. **Read**: This guide completely
2. **Explore**: Open Inference Monitor and explore
3. **Prepare**: Start collecting labeled training data
4. **Train**: Run your first training job
5. **Deploy**: Deploy your trained model
6. **Monitor**: Track performance in Inference Monitor

**You're ready to start! 🚀**

---

*Last Updated: December 2025*
*WeldEval X5 AI & Training Guide*

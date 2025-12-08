# 🚀 AI & Training Quick Start (5 Minutes)

Get started with AI inference monitoring and model training in 5 minutes.

---

## 1️⃣ Inference Monitor (2 minutes)

### Open It
Click **"Inference Monitor"** 🧠 in the sidebar.

### What You See
```
Last Detection:
├─ Class: Good/Porosity/Undercut
├─ Confidence: 78%
├─ Method: Rule-based/ML/Hybrid
└─ Speed: 92ms

Statistics:
├─ Total Scans: 1,234
├─ Success Rate: 98.5%
└─ Good: 892 | Porosity: 287
```

### What It Means
- **High confidence (>85%)**: Trust this result
- **Low confidence (<70%)**: Result uncertain, use rule-based
- **Average time <150ms**: Fast enough
- **Success rate >95%**: System working well

---

## 2️⃣ Model Training (2 minutes)

### Open It
Click **"Model Training"** ✨ in the sidebar.

### Quick Start
```
1. Choose: MobileNetV2 (fast) or ResNet50 (accurate)
2. Keep default settings
3. Click "Start Training"
4. Watch progress bar
5. Download when done ✓
```

### Pick Your Model
- **MobileNetV2**: 30-45 min, 75-80% accuracy ← START HERE
- **ResNet50**: 2-3 hours, 90-95% accuracy ← BEST ACCURACY

### Default Settings (Perfect for Testing)
```
Epochs: 100 (how many times through data)
Batch Size: 32 (images per step)
Learning Rate: 0.001 (how fast to learn)
```

### Monitor
- **Green** = Training going well
- **Red** = Something's wrong
- **Watch loss decrease** = Model learning ✓
- **Watch accuracy increase** = Getting smarter ✓

---

## 3️⃣ Model Management (1 minute)

### Open It
Click **"Model Management"** 📊 in the sidebar.

### Actions
- **Deploy**: Make it active (others become idle)
- **Download**: Save for backup
- **Compare**: View side-by-side metrics
- **Delete**: Remove if not needed

### Best Practice
```
1. Train a model
2. See if accuracy > 85%
3. Compare with current model
4. Deploy only if better
5. Keep old model as backup
```

---

## Common Questions

**Q: Which model should I train?**
A: Start with **MobileNetV2**. It's fast (30 min) and good enough (75-80% accuracy).

**Q: How long does training take?**
A: MobileNetV2: 30-45 min on GPU. ResNet50: 2-3 hours on GPU. Much longer on CPU.

**Q: Do I need a GPU?**
A: Optional. CPU works but is 10-100x slower. GPU recommended.

**Q: What if accuracy is low?**
A: Get more training data. 50 images = poor. 500 images = good. 2000+ images = excellent.

**Q: When should I deploy a new model?**
A: Only if accuracy is >85% AND better than current model by 5%+.

**Q: Can I switch back to old model?**
A: Yes! Keep old model and re-deploy if needed.

---

## Next Steps

1. ✅ Read this guide (5 min)
2. ✅ Explore Inference Monitor (5 min)
3. ⏭️ Collect 100+ labeled weld images (1-2 hours)
4. ⏭️ Train first model (30 min to 3 hours)
5. ⏭️ Deploy and monitor

**You're ready! Start with Inference Monitor. 🎯**

---

*See `AI_AND_TRAINING_GUIDE.md` for complete documentation.*

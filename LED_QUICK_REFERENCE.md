# LED Control - Quick Reference Card

## ⚡ One-Minute Setup

### No Hardware Needed - Start Now
```bash
npm start                    # Frontend on http://localhost:3001
python backend/app.py        # Backend on http://localhost:5000
```

Navigate to: **Settings → Lighting Control**

---

## 🎮 How to Use LED Control

### In the App
1. Go to **Settings** tab
2. Scroll to **Lighting Control**
3. Use slider to adjust brightness (0-100%)
4. Click preset buttons for quick access
5. Monitor temperature in real-time

### From Command Line
```bash
# Turn on at 80%
curl -X POST http://localhost:5000/api/led/on \
  -H "Content-Type: application/json" \
  -d '{"intensity": 80}'

# Check status
curl http://localhost:5000/api/led/status

# Set to preset
curl -X POST http://localhost:5000/api/led/preset \
  -H "Content-Type: application/json" \
  -d '{"preset": "bright"}'
```

### From Python
```python
from backend.vision.led_control import create_led_controller

led = create_led_controller(mode="pwm")
led.turn_on(intensity=75)
led.set_preset("bright")
led.turn_off()
```

---

## ❓ Do I Need a Light Sensor?

**NO - Not required for basic LED control**

✅ Works without sensor:
- Turn on/off
- Brightness 0-100%
- Presets
- Status monitoring

✅ Sensor useful for:
- Auto-brightness adjustment
- Light feedback
- Diagnostics

**Add sensor later if needed** - System works perfectly without it

---

## 📡 API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/led/status` | GET | Get LED status |
| `/api/led/on` | POST | Turn on (optional intensity) |
| `/api/led/off` | POST | Turn off |
| `/api/led/intensity` | POST | Set intensity 0-100 |
| `/api/led/preset` | POST | Apply preset |
| `/api/led/presets` | GET | Get available presets |
| `/api/led/light-level` | GET | Read light sensor |
| `/api/led/temperature` | POST | Update temperature |

---

## 🔧 Hardware Options

### Option 1: GPIO (RDK X5) - RECOMMENDED
```
GPIO 17 → LED+ (via resistor)
GND     → LED-
```

### Option 2: PWM (Smooth Control)
Same as GPIO, uses PWM for smooth brightness

### Option 3: USB Relay
Alternative control method

### Option 4: Simulator (No Hardware)
Default mode for development - LED responds in software only

---

## 🎯 Brightness Presets

| Preset | Level | Use Case |
|--------|-------|----------|
| **off** | 0% | No light |
| **dim** | 25% | Low light mode |
| **medium** | 50% | General use |
| **bright** | 75% | Standard inspection |
| **max** | 100% | Maximum detail |

---

## 🌡️ Temperature Monitoring

- **Safe**: 0-50°C ✅
- **Warning**: >55°C ⚠️
- **Shutdown**: >60°C 🛑

If LED gets hot:
1. Reduce brightness
2. Wait 15-20 minutes
3. Check ventilation

---

## 📂 Files Reference

| File | Purpose |
|------|---------|
| `backend/vision/led_control.py` | LED controller logic |
| `backend/api/led_routes.py` | Flask API endpoints |
| `components/LEDControl.tsx` | React UI component |
| `App.tsx` | App integration |
| `LED_CONTROL_GUIDE.md` | Full documentation |
| `verify_led_integration.py` | Verification script |

---

## ✅ Verification

Run verification script:
```bash
python verify_led_integration.py
```

Expected output:
```
✅ ALL SYSTEMS READY - LED Control fully integrated!
```

---

## 🚀 Next Steps

1. ✅ Access LED Control in Settings
2. ✅ Test turning on/off
3. ✅ Adjust brightness with slider
4. ✅ Try preset buttons
5. ⏭️ (Optional) Install light sensor if auto-adjustment needed

---

## 📞 Troubleshooting

**LED Not Responding?**
- Check API: `curl http://localhost:5000/api/led/status`
- Check power supply
- Check GPIO wiring (if using hardware)
- System should show "simulator" mode if hardware unavailable

**Light Sensor Not Detected?**
- This is normal if not installed
- LED control works without it
- Sensor is optional

**Temperature Warning?**
- Reduce brightness
- Allow cooling time
- Check ventilation

---

**Everything is ready! Start using LED control now.** 🔆

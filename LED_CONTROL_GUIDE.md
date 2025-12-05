# LED Control Integration Guide

## Overview

WeldMaster AI now includes comprehensive LED ring light control capabilities. You can control brightness and on/off status with or without a light sensor.

## Quick Answer: Do You Need a Light Sensor?

### ❌ NO - Light Sensor is Optional

**You CAN control LED without a light sensor:**
- ✅ Turn LED on/off
- ✅ Set brightness (0-100%)
- ✅ Use presets (dim, medium, bright)
- ✅ Monitor LED status
- ✅ Temperature monitoring

**Light sensor is ONLY useful for:**
- Optional auto-brightness adjustment
- Feedback on ambient lighting conditions
- Diagnostic purposes
- Not required for basic LED control

### ✅ YES - Light Sensor is Recommended for:

If you want **automatic brightness adjustment** based on ambient light, then get a sensor. Otherwise, manual control is sufficient.

---

## Hardware Setup

### Option 1: GPIO-Based Control (RDK X5)

**Required:**
- RDK X5 device
- LED ring light with resistor (current limiting)
- Jumper wires
- GPIO pin (default: GPIO 17)

**Wiring:**
```
RDK X5 GPIO 17  → LED+ (via resistor)
RDK X5 GND      → LED-
```

**Enable in code:**
```python
from backend.vision.led_control import create_led_controller
led = create_led_controller(mode="pwm", gpio_pin=17)
```

### Option 2: USB Relay Control

**Required:**
- USB relay module (e.g., SainSmart 8-channel)
- LED ring light
- USB cable

**Installation:**
```bash
pip install pyusb
```

**Enable in code:**
```python
led = create_led_controller(mode="relay")
```

### Option 3: PWM Control (Brightness)

**Required:**
- Same as GPIO but with PWM-capable pin
- LED driver supporting PWM

**Enable in code:**
```python
led = create_led_controller(mode="pwm", gpio_pin=17)
```

### Option 4: Development/Testing (Default)

**No hardware required** - Software simulator:
```python
led = create_led_controller(mode="simulator")
```

---

## Light Sensor (OPTIONAL)

### Which Sensor to Choose?

**BH1750** (Recommended)
- Cost: ~$3-5
- Range: 0-65,535 lux
- Accuracy: Good
- Library: `adafruit-circuitpython-bh1750`

**TSL2561** (Infrared)
- Cost: ~$5-8
- Range: 0-65,000 lux
- Good for non-visible light
- Library: `adafruit-circuitpython-tsl2561`

**VEML7700**
- Cost: ~$8-10
- Range: 0-120,000 lux
- Very accurate
- Library: `adafruit-circuitpython-veml7700`

### Installation (BH1750 Example)

```bash
pip install adafruit-circuitpython-bh1750
```

**Wiring (I2C):**
```
Sensor VCC    → RDK X5 3.3V
Sensor GND    → RDK X5 GND
Sensor SDA    → RDK X5 GPIO 2 (I2C SDA)
Sensor SCL    → RDK X5 GPIO 3 (I2C SCL)
```

### Usage

```python
# System automatically detects sensor
led = create_led_controller(mode="pwm")

# Get light level
light_lux = led.get_light_level()
print(f"Ambient light: {light_lux} lux")
```

---

## API Endpoints

### 1. Get LED Status

```
GET /api/led/status
```

**Response:**
```json
{
  "is_on": true,
  "current_intensity": 75,
  "target_intensity": 75,
  "mode": "pwm",
  "temperature": 35.2,
  "temperature_warning": false,
  "sensor_available": true,
  "light_level_lux": 450.5
}
```

### 2. Turn LED On

```
POST /api/led/on
Content-Type: application/json

{
  "intensity": 75
}
```

### 3. Turn LED Off

```
POST /api/led/off
```

### 4. Set Intensity (0-100%)

```
POST /api/led/intensity
Content-Type: application/json

{
  "intensity": 50
}
```

### 5. Set Preset

```
POST /api/led/preset
Content-Type: application/json

{
  "preset": "bright"
}
```

**Available presets:**
- `off` - 0%
- `dim` - 25%
- `medium` - 50%
- `bright` - 75%
- `max` - 100%

### 6. Get Presets

```
GET /api/led/presets
```

**Response:**
```json
{
  "off": 0,
  "dim": 25,
  "medium": 50,
  "bright": 75,
  "max": 100
}
```

### 7. Get Light Level (Sensor)

```
GET /api/led/light-level
```

**Response:**
```json
{
  "sensor_available": true,
  "light_level_lux": 450.5
}
```

### 8. Set Temperature

```
POST /api/led/temperature
Content-Type: application/json

{
  "temperature": 45.5
}
```

---

## Frontend Integration

### Basic Usage

```tsx
import { LEDControl } from '@/components/LEDControl';

export default function App() {
  return (
    <div>
      <LEDControl />
    </div>
  );
}
```

### With Callback

```tsx
<LEDControl 
  onStatusChange={(status) => {
    console.log('LED Status:', status);
  }}
/>
```

### Custom Control

```tsx
// Turn on at 80% brightness
const response = await fetch('http://localhost:5000/api/led/on', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ intensity: 80 })
});
const data = await response.json();
```

---

## Python Backend Integration

### Initialize in app.py

```python
from backend.api.led_routes import init_led_controller, get_led_blueprint

app = Flask(__name__)

# Initialize LED controller
init_led_controller(app, mode="pwm")  # or "simulator", "relay", "gpio"

# Add LED routes
app.register_blueprint(get_led_blueprint())
```

### Use in Code

```python
from backend.vision.led_control import create_led_controller

# Create controller
led = create_led_controller(mode="pwm", gpio_pin=17)

# Turn on
led.turn_on(intensity=75)

# Set specific intensity
led.set_intensity(50)

# Use preset
led.set_preset("bright")

# Check status
status = led.get_status()

# Get light level (if sensor available)
light_lux = led.get_light_level()

# Cleanup when done
led.cleanup()
```

---

## Usage Examples

### Example 1: Simple On/Off

```bash
# Turn on at 75%
curl -X POST http://localhost:5000/api/led/on \
  -H "Content-Type: application/json" \
  -d '{"intensity": 75}'

# Turn off
curl -X POST http://localhost:5000/api/led/off
```

### Example 2: Brightness Control

```bash
# Set to 50%
curl -X POST http://localhost:5000/api/led/intensity \
  -H "Content-Type: application/json" \
  -d '{"intensity": 50}'
```

### Example 3: Using Presets

```bash
# Set to "bright" preset
curl -X POST http://localhost:5000/api/led/preset \
  -H "Content-Type: application/json" \
  -d '{"preset": "bright"}'
```

### Example 4: Check Status

```bash
curl http://localhost:5000/api/led/status
```

---

## Configuration

### Brightness Levels

Modify in `backend/vision/led_control.py`:

```python
self.presets = {
    "off": 0,
    "dim": 25,      # Adjust these values
    "medium": 50,
    "bright": 75,
    "max": 100
}
```

### Temperature Safety

```python
self.max_temp = 60.0      # Shutdown temperature
self.warning_temp = 55.0  # Warning temperature
```

### PWM Frequency

```python
self.pwm = GPIO.PWM(self.gpio_pin, 1000)  # 1000 Hz frequency
```

---

## Troubleshooting

### LED Not Responding

1. **Check GPIO connection:**
   ```bash
   python
   >>> import RPi.GPIO as GPIO
   >>> GPIO.setmode(GPIO.BCM)
   >>> GPIO.setup(17, GPIO.OUT)
   >>> GPIO.output(17, GPIO.HIGH)  # Should turn on
   ```

2. **Verify power supply:**
   - Check LED has adequate power
   - Verify resistor is correctly sized
   - Test with multimeter

3. **Check API connectivity:**
   ```bash
   curl http://localhost:5000/api/led/status
   ```

### Light Sensor Not Detected

This is **normal** if you haven't installed a sensor. LED control works without it.

To verify:
```bash
curl http://localhost:5000/api/led/light-level
# Should show: "sensor_available": false
```

### Temperature Warning

If LED gets too hot:
1. Reduce brightness
2. Ensure adequate ventilation
3. Check for short circuits
4. Monitor with: `curl http://localhost:5000/api/led/status`

---

## Performance Specifications

### Brightness Control
- Range: 0-100%
- Precision: 1% steps
- Smooth transitions: Yes (hardware dependent)
- Response time: <200ms

### Safety Features
- Temperature monitoring
- Automatic shutdown at >60°C
- Warning at >55°C
- Soft shutdown (smooth fade)

### Supported Modes
| Mode | Hardware | Brightness | Cost | Status |
|------|----------|-----------|------|--------|
| GPIO | RDK X5 | On/Off only | ~$0 | ✅ Ready |
| PWM | RDK X5 | 0-100% | ~$0 | ✅ Ready |
| Relay | USB | On/Off | ~$10 | ✅ Ready |
| Simulator | None | Simulated | $0 | ✅ Ready |

---

## Best Practices

### Lighting Setup
1. Position LED ring at 45° angle
2. Use diffuser for uniform light
3. Avoid direct reflections
4. Maintain consistent distance from subject

### Brightness Selection
- **Calibration**: 50-75% (bright but not harsh)
- **Scanning**: 75-100% (maximum detail)
- **Display**: 25-50% (reduce eye strain)

### Temperature Management
- Monitor regularly: Check status every 5 minutes during use
- Allow cooling: 15-20 minutes between intensive sessions
- Check ventilation: Ensure LED has airflow

### Power Consumption
- Max brightness: ~5W typical LED ring
- Min brightness: ~0.5W
- Consider with RDK X5 total power budget

---

## Next Steps

1. **Choose control mode** (simulator/GPIO/relay)
2. **Install sensor** (optional, only if needed)
3. **Test with frontend component** - LEDControl component ready in app
4. **Configure brightness presets** as needed
5. **Integrate into scanning workflow** - LED turns on before scans

All set! Start using LED control now! 🔆

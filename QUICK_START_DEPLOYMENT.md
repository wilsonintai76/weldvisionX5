# WeldVision X5: 5-Minute Quick Start Guide

## Prerequisites ✓
- RDK X5 (powered, camera connected)
- Ethernet cable or WiFi
- Desktop/laptop with browser
- SSH access (optional but recommended)

---

## Step 1: Find RDK X5 IP (2 min)

### Option A: Check Router
1. Open router admin page (`192.168.1.1`)
2. Find connected device "RDK-X5-XXXXXX"
3. Note IP (e.g., `192.168.1.100`)

### Option B: Network Scan
```bash
# Linux/Mac
nmap -sn 192.168.1.0/24 | grep -i rdk

# Windows
arp -a | findstr "rdk" (case-insensitive search)
```

### Option C: SSH Direct
```bash
# Try common default IPs
ssh root@192.168.1.100
# Password: root (or empty, just press Enter)
# Run: hostname -I
```

---

## Step 2: Deploy Code (2 min)

### Via Git (Recommended)
```bash
# SSH to RDK X5
ssh root@<RDK_X5_IP>

# Deploy
mkdir -p /opt/weldvision
cd /opt/weldvision
git clone https://github.com/wilsonintai76/weldvisionX5.git .
chmod -R 755 .
```

### Via SCP (If no Git)
```bash
# From your desktop
scp -r ./weldvisionX5 root@<RDK_X5_IP>:/opt/weldvision
```

---

## Step 3: Install & Start (1 min)

```bash
# Still SSH'd into RDK X5

# Install backend
cd /opt/weldvision/backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Install frontend
cd /opt/weldvision
apt-get install -y nodejs npm 2>/dev/null || echo "Node already installed"
npm install
npm run build

# Start services (in background)
cd backend && source venv/bin/activate && python3 app.py &
cd .. && npm run preview -- --host 0.0.0.0 --port 3000 &

echo "✓ Services started"
```

---

## Step 4: Access the Application

### From Browser
```
Frontend:  http://<RDK_X5_IP>:3000
Backend:   http://<RDK_X5_IP>:5000

Example:   http://192.168.1.100:3000
```

### Using mDNS (if enabled)
```
Frontend:  http://weldvision-x5.local:3000
Backend:   http://weldvision-x5.local:5000
```

---

## Step 5: Test It Works

1. **Open browser** → `http://<RDK_X5_IP>:3000`
2. **Add a student** → Dashboard → "Students" → "Add Student"
3. **Test scanner** → "Live Scanner" → Select student → "Capture"
4. **Check results** → Results appear on right panel

---

## Troubleshooting Quick Fix

| Issue | Fix |
|-------|-----|
| Can't find IP | Run `arp -a` or check router | 
| Can't SSH | Try `ssh -v root@<IP>` (verbose) |
| Port 3000/5000 not responding | Check `ufw` firewall: `ufw allow 3000` |
| Camera not found | Verify cables in MIPI connectors |
| Slow performance | Use Ethernet instead of WiFi |

---

## For Production: Setup Auto-Start (Optional)

```bash
# Create systemd services (see full guide for details)
# After setup, services start automatically on RDK X5 reboot
```

---

## Full Documentation

👉 See `COMPLETE_DEPLOYMENT_AND_REMOTE_ACCESS_GUIDE.md` for:
- Systemd service setup
- Firewall configuration
- VPN/SSH tunnel remote access
- Security hardening
- Detailed troubleshooting

---

## Key Ports
- **3000** = Frontend UI
- **5000** = Backend API
- **22** = SSH access

---

**Status:** ✅ Ready to Deploy

*First-time setup usually takes 10-15 minutes total. After that, it's just `http://<IP>:3000` to access.*

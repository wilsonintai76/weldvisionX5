# WeldVision X5: Complete Deployment & Remote Access Guide

**Last Updated:** December 8, 2025  
**Status:** Production Ready  
**Platform:** RDK X5 (Horizon Robotics)

---

## Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Part 1: Initial RDK X5 Setup](#part-1-initial-rdk-x5-setup)
3. [Part 2: Software Deployment](#part-2-software-deployment)
4. [Part 3: Network Configuration](#part-3-network-configuration)
5. [Part 4: Remote Access Setup](#part-4-remote-access-setup)
6. [Part 5: Testing & Verification](#part-5-testing--verification)
7. [Troubleshooting](#troubleshooting)
8. [Quick Reference](#quick-reference)

---

## Hardware Requirements

### RDK X5 Edge Device
- **CPU:** Octa-core ARM v8 (2.0 GHz)
- **Memory:** 4GB LPDDR4X
- **Storage:** 32GB eMMC
- **Network:** Gigabit Ethernet, optional WiFi module
- **OS:** Ubuntu 20.04 LTS (pre-installed)
- **GPU:** Horizon BPU (AI accelerator)

### RDK Stereo Camera
- **Model:** D-Robotics RDK Stereo Camera Module
- **Sensors:** Dual 2MP SC230AI (1920×1080)
- **Interface:** Dual 22PIN MIPI CSI-2
- **Baseline:** 70mm
- **Synchronization:** Hardware synchronized
- **Frame Rate:** 30 FPS

### Connectivity Options
- **Recommended:** Gigabit Ethernet (RJ45 cable)
- **Alternative:** WiFi module (if installed)
- **Desktop Access:** WiFi or Ethernet

### Optional Hardware
- USB keyboard/mouse for initial setup
- HDMI monitor/cable (initial setup only)
- Ethernet switch (for multiple devices)

---

## Part 1: Initial RDK X5 Setup

### Step 1: Unbox & Power On

1. **Unpack the RDK X5:**
   - RDK X5 main board
   - RDK Stereo Camera module
   - Power adapter (recommended: 12V/2A minimum)
   - Ethernet cable

2. **Connect Camera:**
   - Locate dual 22PIN MIPI connectors on RDK X5
   - Insert camera flat cables into dedicated ports
   - Ensure connectors are fully seated
   - **Note:** Both left and right channels should be connected

3. **Power Connections:**
   ```
   Power Source (12V/2A) → RDK X5 DC Jack
   ```

4. **Network Connection:**
   - Plug Ethernet cable into RDK X5 Gigabit port
   - Connect other end to your network router/switch
   - **LED Indicator:** Green light = connected

5. **Power On:**
   - Press power button on RDK X5
   - Wait 30-60 seconds for boot
   - Check for status LEDs

### Step 2: Find RDK X5 IP Address

**Option A: Using Router Admin Panel**
1. Access your router's admin interface (usually `192.168.1.1`)
2. Look for connected devices list
3. Find device named "RDK-X5-XXXXXX"
4. Note its IP address (e.g., `192.168.1.100`)

**Option B: Using Network Scanning (Linux/Mac)**
```bash
# Install nmap if needed
sudo apt-get install nmap

# Scan your network subnet (replace with your network)
nmap -sn 192.168.1.0/24 | grep -i rdk
```

**Option C: From RDK X5 Terminal**
```bash
hostname -I
# Output: 192.168.1.100 (or similar)
```

**Option D: Using IP Discovery Tool**
- Use Angry IP Scanner (Windows/Linux)
- Look for hostname containing "RDK" or "Horizon"

### Step 3: SSH Access to RDK X5

1. **From your desktop, open terminal:**
   ```bash
   ssh root@<RDK_X5_IP>
   # Example: ssh root@192.168.1.100
   ```

2. **Default Credentials:**
   - **Username:** `root`
   - **Password:** `root` (or empty, press Enter)
   - **Note:** Change password immediately in production

3. **First-time Setup Commands:**
   ```bash
   # Update system
   apt-get update && apt-get upgrade -y

   # Install essential tools
   apt-get install -y curl wget git htop net-tools

   # Verify camera is detected
   ls -la /dev/video*
   # Should show: /dev/video0, /dev/video1 (stereo pair)
   ```

### Step 4: Verify ROS2 Installation

```bash
# Check ROS2 TROS installation
source /opt/tros/humble/setup.bash
echo $ROS_DISTRO
# Should output: humble

# List available topics (camera test)
ros2 topic list | grep camera
```

---

## Part 2: Software Deployment

### Step 1: Deploy WeldVision X5 Code

**On RDK X5 (via SSH):**

```bash
# Create deployment directory
mkdir -p /opt/weldvision
cd /opt/weldvision

# Clone repository (or use SCP to copy)
git clone https://github.com/wilsonintai76/weldvisionX5.git .
# OR: scp -r /path/to/local/code root@<RDK_IP>:/opt/weldvision/

# Set permissions
chmod -R 755 .
```

### Step 2: Install Backend Dependencies

```bash
cd /opt/weldvision/backend

# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python packages
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

# Verify key packages installed
python3 -c "import flask, cv2, numpy, sqlalchemy; print('✓ All core packages installed')"
```

### Step 3: Install Frontend Dependencies

```bash
cd /opt/weldvision

# Install Node.js (if not present)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
apt-get install -y nodejs

# Install frontend dependencies
npm install

# Verify installation
npm list vite react
```

### Step 4: Configure Environment

**Create `/opt/weldvision/.env.production` on RDK X5:**

```env
# WeldVision X5 Production Environment
NODE_ENV=production
VITE_API_URL=http://0.0.0.0:5000
REACT_APP_BACKEND_URL=http://0.0.0.0:5000

# Backend Configuration
FLASK_ENV=production
FLASK_DEBUG=0
PYTHONUNBUFFERED=1

# ROS2 Configuration
ROS_DISTRO=humble
ROS_MIDDLEWARE_IMPLEMENTATION=rmw_cyclonedds_cpp

# Database
DATABASE_URL=sqlite:////opt/weldvision/backend/weld_data.db

# Server Configuration
BACKEND_PORT=5000
FRONTEND_PORT=3000
```

### Step 5: Build Frontend for Production

```bash
cd /opt/weldvision

# Build optimized bundle
npm run build

# Verify build output
ls -lah dist/
# Should show: index.html, assets/ directory, etc.
```

### Step 6: Create Systemd Services

**Create `/etc/systemd/system/weldvision-backend.service`:**

```ini
[Unit]
Description=WeldVision X5 Backend API Server
After=network.target
Wants=weldvision-frontend.service

[Service]
Type=simple
User=root
WorkingDirectory=/opt/weldvision/backend
Environment="PATH=/opt/weldvision/backend/venv/bin"
Environment="PYTHONUNBUFFERED=1"
ExecStart=/opt/weldvision/backend/venv/bin/python3 app.py
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Create `/etc/systemd/system/weldvision-frontend.service`:**

```ini
[Unit]
Description=WeldVision X5 Frontend (Vite)
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/opt/weldvision
ExecStart=/usr/bin/npm run preview -- --host 0.0.0.0 --port 3000
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Enable and Start Services:**

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable weldvision-backend.service
sudo systemctl enable weldvision-frontend.service

# Start services
sudo systemctl start weldvision-backend.service
sudo systemctl start weldvision-frontend.service

# Check status
sudo systemctl status weldvision-backend.service
sudo systemctl status weldvision-frontend.service

# View logs
sudo journalctl -u weldvision-backend.service -f
sudo journalctl -u weldvision-frontend.service -f
```

---

## Part 3: Network Configuration

### Step 1: Configure Network Interface

**Check Available Interfaces:**

```bash
ip link show
# Output: eth0 (Ethernet), wlan0 (WiFi if available)

ip addr show
# Shows IP addresses assigned to each interface
```

**For Ethernet (Recommended):**

```bash
# DHCP (Automatic IP) - Usually default
sudo dhclient eth0

# Verify
ip addr show eth0
```

**For Static IP (Optional):**

```bash
# Edit netplan config
sudo nano /etc/netplan/00-installer-config.yaml
```

Add:
```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

Apply:
```bash
sudo netplan apply
ip addr show eth0
```

### Step 2: Configure Firewall

```bash
# Install UFW (if not present)
apt-get install -y ufw

# Enable firewall
ufw enable

# Allow SSH
ufw allow 22/tcp

# Allow WeldVision ports
ufw allow 3000/tcp  # Frontend
ufw allow 5000/tcp  # Backend

# Allow ROS2 ports (optional)
ufw allow 11311/tcp  # ROS Master

# Check status
ufw status
```

### Step 3: Configure Hostname

```bash
# Set hostname
sudo hostnamectl set-hostname weldvision-x5

# Edit hosts file
sudo nano /etc/hosts
# Add: 127.0.0.1 weldvision-x5

# Verify
hostname
```

### Step 4: Enable mDNS (Optional - for .local domain)

```bash
# Install avahi
apt-get install -y avahi-daemon

# Enable mDNS service
systemctl enable avahi-daemon
systemctl start avahi-daemon

# Now accessible as: http://weldvision-x5.local:3000
```

---

## Part 4: Remote Access Setup

### Method 1: Same Network Access (Recommended)

**Prerequisites:**
- RDK X5 and desktop on same WiFi/LAN network
- Know RDK X5 IP address

**From Desktop Browser:**

```
Frontend:  http://<RDK_X5_IP>:3000
Backend:   http://<RDK_X5_IP>:5000

Example (if RDK X5 IP is 192.168.1.100):
Frontend:  http://192.168.1.100:3000
Backend:   http://192.168.1.100:5000
```

**Using mDNS (if enabled):**

```
Frontend:  http://weldvision-x5.local:3000
Backend:   http://weldvision-x5.local:5000
```

### Method 2: Remote Network Access (VPN)

**Setup VPN Server on RDK X5:**

```bash
# Install OpenVPN
apt-get install -y openvpn easy-rsa

# Generate keys
make-cadir ~/openvpn-ca
cd ~/openvpn-ca
./easyrsa init-pki
./easyrsa build-ca
./easyrsa gen-req server nopass
./easyrsa sign-req server server
./easyrsa gen-dh

# Copy keys
cp pki/ca.crt pki/private/server.key pki/issued/server.crt dh.pem /etc/openvpn/server/

# Configure VPN
nano /etc/openvpn/server/server.conf
```

Add configuration:
```
port 1194
proto tcp
dev tun
ca ca.crt
cert server.crt
key server.key
dh dh.pem
ifconfig-pool-persist ipp.txt
push "route 0.0.0.0 0.0.0.0"
```

Enable and start:
```bash
systemctl enable openvpn-server@server.service
systemctl start openvpn-server@server.service
```

### Method 3: SSH Tunnel (Secure Remote Access)

**From Desktop:**

```bash
# Create SSH tunnel
ssh -L 3000:localhost:3000 -L 5000:localhost:5000 root@<RDK_X5_IP>

# In another terminal, access locally
Frontend:  http://localhost:3000
Backend:   http://localhost:5000
```

**Keep tunnel open while using the application.**

### Method 4: Reverse SSH Proxy (For Multiple Desktops)

**On RDK X5:**

```bash
# Install autossh for persistent tunnel
apt-get install -y autossh

# Create tunnel back to desktop
autossh -M 20000 -N -R 3000:localhost:3000 -R 5000:localhost:5000 user@desktop.ip
```

---

## Part 5: Testing & Verification

### Step 1: Backend Health Check

```bash
# From desktop, test API
curl http://<RDK_X5_IP>:5000/api/health

# Expected response:
# {"status": "ok", "timestamp": "..."}
```

### Step 2: Frontend Access

1. Open browser
2. Navigate to `http://<RDK_X5_IP>:3000`
3. Should see WeldVision X5 dashboard
4. Verify all UI elements load

### Step 3: Camera System Check

**Via SSH on RDK X5:**

```bash
# Test stereo camera detection
python3 -c "
from backend.vision.rdk_stereo_camera import RDKStereoCameraHandler
cam = RDKStereoCameraHandler()
if cam.connect():
    print('✓ Stereo camera detected')
    print(f'Resolution: {cam.get_resolution()}')
    print(f'FPS: {cam.get_fps()}')
else:
    print('✗ Camera connection failed')
"
```

### Step 4: Database Verification

```bash
# Check database
sqlite3 /opt/weldvision/backend/weld_data.db ".tables"

# Should show: scan, student, scan_configuration, etc.
```

### Step 5: Network Performance Test

```bash
# From desktop
ping -c 5 <RDK_X5_IP>
# Check latency (should be < 10ms on same network)

# Test bandwidth (requires iperf3 on both sides)
iperf3 -c <RDK_X5_IP>
```

### Step 6: Full System Test

1. **Access frontend:** `http://<RDK_X5_IP>:3000`
2. **Add a student:**
   - Click "Students"
   - Click "Add Student"
   - Fill form, save
3. **Test Live Scanner:**
   - Click "Live Scanner"
   - Select student
   - Click "Capture"
   - Verify result displays
4. **Check History:**
   - Click "Scan History"
   - Verify scan appears in list

---

## Troubleshooting

### Problem: Cannot find RDK X5 on network

**Solutions:**

1. **Check physical connections:**
   ```bash
   # On RDK X5
   ethtool eth0
   # Should show: Link detected: yes
   ```

2. **Verify DHCP:**
   ```bash
   # On RDK X5
   dhclient -v eth0
   ip addr show eth0
   ```

3. **Check firewall on desktop:**
   - Ensure network is not isolated
   - Disable Windows Firewall temporarily (Windows)
   - Check router DHCP range

4. **Use USB cable fallback:**
   - Connect RDK X5 via USB for initial setup
   - Configure network, then switch to Ethernet

### Problem: Cannot access backend API

**Solutions:**

1. **Verify service is running:**
   ```bash
   sudo systemctl status weldvision-backend.service
   ```

2. **Check port availability:**
   ```bash
   sudo netstat -tulpn | grep 5000
   ```

3. **Test locally on RDK X5:**
   ```bash
   curl http://localhost:5000/api/health
   ```

4. **Check firewall:**
   ```bash
   ufw status
   sudo ufw allow 5000/tcp
   ```

### Problem: Camera not detected

**Solutions:**

1. **Verify physical connection:**
   - Reseat camera cables in MIPI connectors
   - Check both left and right channels connected

2. **Check device nodes:**
   ```bash
   ls -la /dev/video*
   # Should show /dev/video0, /dev/video1
   ```

3. **Test with v4l2:**
   ```bash
   apt-get install -y v4l-utils
   v4l2-ctl --list-devices
   ```

4. **Check ROS2 topics:**
   ```bash
   source /opt/tros/humble/setup.bash
   ros2 topic list | grep camera
   ```

### Problem: High latency or slow performance

**Solutions:**

1. **Check network connection:**
   ```bash
   iperf3 -c <RDK_X5_IP>
   # Should see > 100 Mbps for LAN
   ```

2. **Monitor CPU/Memory:**
   ```bash
   htop
   # Check for high CPU usage
   ```

3. **Optimize frontend:**
   - Close unused browser tabs
   - Use Chrome/Chromium (fastest)
   - Disable browser extensions

4. **Check ROS2 middleware:**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   # Faster than default CycloneDDS
   ```

### Problem: Services not starting

**Solutions:**

1. **Check service logs:**
   ```bash
   sudo journalctl -u weldvision-backend.service -n 50
   ```

2. **Verify Python environment:**
   ```bash
   /opt/weldvision/backend/venv/bin/python3 --version
   ```

3. **Test manual startup:**
   ```bash
   cd /opt/weldvision/backend
   source venv/bin/activate
   python3 app.py
   ```

4. **Check Node.js:**
   ```bash
   node --version
   npm --version
   ```

---

## Quick Reference

### Network Access
```
Frontend:     http://<RDK_X5_IP>:3000
Backend:      http://<RDK_X5_IP>:5000
SSH Access:   ssh root@<RDK_X5_IP>
```

### Common Commands

**RDK X5 (via SSH):**
```bash
# Get IP address
hostname -I

# Restart services
sudo systemctl restart weldvision-backend.service
sudo systemctl restart weldvision-frontend.service

# View logs
sudo journalctl -u weldvision-backend.service -f

# Check resources
htop

# Test camera
ls /dev/video*
```

**From Desktop:**
```bash
# SSH to RDK X5
ssh root@<IP>

# Test API
curl http://<IP>:5000/api/health

# Copy files to RDK X5
scp -r /local/path root@<IP>:/remote/path

# Ping test
ping <IP>
```

### Service Management

```bash
# Start services
sudo systemctl start weldvision-backend.service
sudo systemctl start weldvision-frontend.service

# Stop services
sudo systemctl stop weldvision-backend.service
sudo systemctl stop weldvision-frontend.service

# Restart services
sudo systemctl restart weldvision-backend.service
sudo systemctl restart weldvision-frontend.service

# Enable auto-start
sudo systemctl enable weldvision-backend.service
sudo systemctl enable weldvision-frontend.service

# Disable auto-start
sudo systemctl disable weldvision-backend.service
sudo systemctl disable weldvision-frontend.service

# Check status
sudo systemctl status weldvision-*.service
```

### Firewall Rules

```bash
# Enable firewall
ufw enable

# Allow ports
ufw allow 22/tcp    # SSH
ufw allow 3000/tcp  # Frontend
ufw allow 5000/tcp  # Backend
ufw allow 1194/udp  # VPN (if using OpenVPN)

# Reset to defaults
ufw reset
```

### Performance Monitoring

```bash
# Real-time resource usage
htop

# Network statistics
iftop

# Disk usage
df -h

# Memory usage
free -h

# Process monitoring
ps aux | grep python3
ps aux | grep node
```

---

## Security Recommendations

### For Production Deployment:

1. **Change default password:**
   ```bash
   passwd root
   ```

2. **Disable SSH password, use keys:**
   ```bash
   # Generate key on desktop
   ssh-keygen -t ed25519

   # Copy to RDK X5
   ssh-copy-id -i ~/.ssh/id_ed25519.pub root@<RDK_X5_IP>

   # Disable password auth
   sudo nano /etc/ssh/sshd_config
   # Set: PasswordAuthentication no
   sudo systemctl restart ssh
   ```

3. **Use HTTPS for web access:**
   - Install nginx with SSL
   - Use Let's Encrypt certificates
   - Configure reverse proxy

4. **Enable firewall:**
   - Allow only necessary ports
   - Implement rate limiting
   - Log all access

5. **Regular backups:**
   ```bash
   # Backup database
   tar -czf backup_$(date +%Y%m%d).tar.gz /opt/weldvision/backend/weld_data.db
   ```

---

## Maintenance Schedule

### Daily:
- Monitor system logs
- Check service status
- Verify camera functionality

### Weekly:
- Review scan history
- Check storage usage
- Test remote access

### Monthly:
- Update system packages: `apt-get update && apt-get upgrade -y`
- Rotate logs
- Performance analysis

### Quarterly:
- Test disaster recovery
- Update documentation
- Hardware inspection

---

## Additional Resources

- **ROS2 Documentation:** https://docs.ros.org/en/humble/
- **RDK X5 Official Guide:** https://developer.horizon.ai/
- **Flask Documentation:** https://flask.palletsprojects.com/
- **Vite Documentation:** https://vitejs.dev/
- **OpenVPN Setup:** https://openvpn.net/community-downloads/

---

## Support & Troubleshooting

**For issues, check:**

1. Service logs: `sudo journalctl -u weldvision-*.service -n 100`
2. Application logs: `/opt/weldvision/backend/app.log`
3. Network connectivity: `ping <RDK_X5_IP>`
4. API health: `curl http://<RDK_X5_IP>:5000/api/health`

**Contact:**
- Repository: https://github.com/wilsonintai76/weldvisionX5
- Issues: GitHub Issues
- Email: wilsonintai76@example.com

---

## Deployment Checklist

- [ ] RDK X5 hardware verified and powered on
- [ ] Camera module installed and connected
- [ ] Network cable connected (Ethernet preferred)
- [ ] RDK X5 IP address identified
- [ ] SSH access confirmed
- [ ] Code deployed to `/opt/weldvision`
- [ ] Backend dependencies installed
- [ ] Frontend dependencies installed
- [ ] Frontend built for production
- [ ] Systemd services created and enabled
- [ ] Services started successfully
- [ ] Firewall configured correctly
- [ ] Backend API accessible via HTTP
- [ ] Frontend UI loads in browser
- [ ] Camera system verified
- [ ] Database initialized
- [ ] Full system test completed
- [ ] Remote access tested
- [ ] Security hardening applied
- [ ] Documentation reviewed

**Status:** ✅ Ready for Production Deployment

---

*This guide provides everything needed to deploy WeldVision X5 from initial hardware setup through production remote access. Follow each section sequentially for smooth deployment.*

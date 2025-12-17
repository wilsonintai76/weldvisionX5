# WeldMaster AI Evaluator - Deployment Instructions

> Complete deployment guide for development, staging, and production environments

**Document Version:** 1.0  
**Last Updated:** December 17, 2025  
**Target Platform:** RDK X5, Windows (Development), Linux (Production)

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Environment Overview](#environment-overview)
3. [Development Deployment (Windows)](#development-deployment-windows)
4. [Production Deployment (RDK X5)](#production-deployment-rdk-x5)
5. [Database Deployment](#database-deployment)
6. [Frontend Deployment](#frontend-deployment)
7. [System Configuration](#system-configuration)
8. [Security Hardening](#security-hardening)
9. [Monitoring & Logging](#monitoring--logging)
10. [Backup & Recovery](#backup--recovery)
11. [Troubleshooting](#troubleshooting)
12. [Rollback Procedures](#rollback-procedures)

---

## Prerequisites

### Hardware Requirements

#### Development Environment
- **CPU**: x86_64 dual-core or better
- **RAM**: 4GB minimum, 8GB recommended
- **Storage**: 10GB free space
- **OS**: Windows 10/11, macOS 10.15+, Ubuntu 20.04+

#### Production Environment (RDK X5)
- **Board**: Horizon Robotics RDK X5
- **CPU**: ARM Cortex-A55 quad-core @ 1.8GHz
- **RAM**: 2GB LPDDR4
- **Storage**: 16GB eMMC or microSD (32GB recommended)
- **Camera**: RDK Stereo Camera (1920x1080, MIPI or USB)
- **Power**: 12V 2A DC adapter
- **Cooling**: Heatsink or active cooling fan

### Software Requirements

#### All Environments
```
Python 3.9+
pip 21.0+
Git 2.30+
SQLite 3.35+
```

#### Development Only
```
Node.js 16+ (for frontend)
npm 8+
Visual Studio Code (recommended)
```

#### Production Only
```
ROS2 Foxy or Humble
TROS (Together ROS) for RDK X5
systemd (for service management)
nginx (for production frontend)
```

### Network Requirements

- **Development**: Internet access for package installation
- **Production**: 
  - Intranet connectivity for API access
  - Outbound HTTPS (443) for package updates (optional)
  - Inbound HTTP (80/443) for frontend
  - Inbound HTTP (5000) for API

---

## Environment Overview

### Architecture by Environment

```
┌─────────────────────────────────────────────────┐
│             DEVELOPMENT                         │
│  Windows/macOS/Linux Desktop                    │
│  - Vite dev server (hot reload)                 │
│  - Flask debug mode                             │
│  - SQLite with test data                        │
│  - Mock camera feeds                            │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│             STAGING                              │
│  RDK X5 (internal network)                      │
│  - Production build preview                      │
│  - Flask production mode                         │
│  - Real camera (test rig)                        │
│  - Limited user access                           │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│             PRODUCTION                           │
│  RDK X5 (welding lab)                           │
│  - nginx + production build                      │
│  - Flask with gunicorn                           │
│  - Calibrated stereo camera                      │
│  - Full user access                              │
│  - Monitoring & backups                          │
└─────────────────────────────────────────────────┘
```

---

## Development Deployment (Windows)

### Step 1: Clone Repository

```powershell
# Navigate to your workspace
cd "D:\WeldMaster AI Evaluation"

# Clone repository (if not already cloned)
git clone https://github.com/wilsonintai76/weldvisionX5.git
cd weldvisionX5

# Or navigate to existing directory
cd "d:\WeldMaster AI Evaluation\rdk_weld_evaluator"
```

### Step 2: Create Python Virtual Environment

```powershell
# Create virtual environment
python -m venv venv

# Activate virtual environment
.\venv\Scripts\Activate.ps1

# Verify activation
python --version
# Expected: Python 3.9.x or higher
```

**If execution policy error occurs:**

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### Step 3: Install Python Dependencies

```powershell
# Upgrade pip
python -m pip install --upgrade pip

# Install requirements
pip install -r requirements.txt

# Verify installation
pip list | Select-String -Pattern "flask|sqlalchemy|opencv"
```

**Expected packages:**
- Flask 3.0+
- Flask-SQLAlchemy 3.0+
- SQLAlchemy 2.0+
- opencv-python 4.5+
- numpy 1.24+
- pyyaml 6.0+

### Step 4: Initialize Database

```powershell
# Initialize database with demo data
python main.py --mode test

# Expected output:
# ✓ Database initialized
# ✓ Tables created
# ✓ Demo data loaded
```

### Step 5: Start Backend Server

```powershell
# Start Flask API in debug mode
python main.py --mode server --debug true

# Expected output:
# * Running on http://127.0.0.1:5000
# * Debug mode: on
```

**Verify backend:**

```powershell
# Test API endpoint
curl http://localhost:5000/api/classes
# Expected: JSON array of classes
```

### Step 6: Setup Frontend

```powershell
# Open new PowerShell terminal
cd "d:\WeldMaster AI Evaluation\weld-frontend"

# Install Node dependencies
npm install

# Verify installation
npm list --depth=0
```

### Step 7: Start Frontend Development Server

```powershell
# Start Vite dev server
npm run dev

# Expected output:
# VITE v4.5.14 ready in 200 ms
# ➜ Local: http://localhost:3000/
```

**Verify frontend:**

Open browser: `http://localhost:3000`

Expected: Dashboard view with system status

### Step 8: Verify Integration

**Test workflow:**
1. Navigate to Dashboard (/)
2. Click "Student Manager" - verify students load
3. Click "Calibration" - verify calibration UI
4. Click "Scan Weld" - verify scanning interface
5. Click "Results" - verify results display

**Check browser console:**
- No 404 errors for API calls
- API responses return valid JSON
- Mock data displays correctly

---

## Production Deployment (RDK X5)

### Pre-Deployment Checklist

- [ ] RDK X5 flashed with Ubuntu 20.04/22.04
- [ ] Stereo camera connected and tested
- [ ] Network configured (static IP recommended)
- [ ] SSH access verified
- [ ] Calibration checkerboard pattern prepared
- [ ] Production configuration files prepared

### Step 1: Connect to RDK X5

```bash
# SSH into RDK X5
ssh ubuntu@<rdk-ip-address>
# Default password: ubuntu (change immediately)

# Change default password
passwd

# Update system
sudo apt update
sudo apt upgrade -y
```

### Step 2: Install System Dependencies

```bash
# Install Python and development tools
sudo apt install -y \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-opencv \
    git \
    build-essential \
    cmake \
    libopencv-dev \
    libatlas-base-dev \
    libhdf5-dev \
    sqlite3

# Install ROS2 (if not included in RDK OS)
# Follow official RDK X5 documentation for TROS installation
```

### Step 3: Deploy Application Code

**Option A: Git Clone (Recommended)**

```bash
# Clone repository
cd /opt
sudo git clone https://github.com/wilsonintai76/weldvisionX5.git weld_evaluator
sudo chown -R ubuntu:ubuntu /opt/weld_evaluator
cd /opt/weld_evaluator
```

**Option B: SCP Transfer**

```powershell
# From Windows development machine
scp -r "d:\WeldMaster AI Evaluation\rdk_weld_evaluator" ubuntu@<rdk-ip>:/opt/weld_evaluator
```

### Step 4: Setup Python Environment

```bash
cd /opt/weld_evaluator

# Create virtual environment
python3 -m venv venv

# Activate environment
source venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Verify installation
python -c "import flask, sqlalchemy, cv2; print('All imports successful')"
```

### Step 5: Configure Camera

```bash
# List available cameras
ls -l /dev/video*

# Test camera (for USB camera)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
    videoconvert ! videoscale ! \
    video/x-raw,width=640,height=480 ! \
    autovideosink

# For MIPI camera, follow RDK documentation for correct device nodes
```

### Step 6: Perform Stereo Calibration

```bash
# Run calibration wizard
python main.py --mode calibrate

# Follow prompts to capture checkerboard patterns
# Minimum 10-15 images at various angles

# Verify calibration file created
ls -l config/camera_calibration.yaml
cat config/camera_calibration.yaml
```

**Expected calibration output:**

```yaml
camera_matrix_left: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
camera_matrix_right: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
dist_coeffs_left: [k1, k2, p1, p2, k3]
dist_coeffs_right: [k1, k2, p1, p2, k3]
rotation_matrix: [[r11, r12, r13], ...]
translation_vector: [tx, ty, tz]
baseline: 120.0
```

### Step 7: Initialize Database

```bash
# Create production database
python main.py --mode test

# Verify database
sqlite3 demo_weld_evaluator.db ".tables"
# Expected: weld_class, student, evaluation, class_statistics
```

### Step 8: Create Systemd Service

```bash
# Create service file
sudo nano /etc/systemd/system/weld-evaluator.service
```

**Service file content:**

```ini
[Unit]
Description=WeldMaster AI Evaluator Backend
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/opt/weld_evaluator
Environment="PATH=/opt/weld_evaluator/venv/bin"
ExecStart=/opt/weld_evaluator/venv/bin/python main.py --mode server --host 0.0.0.0 --port 5000
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Enable and start service:**

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable weld-evaluator

# Start service
sudo systemctl start weld-evaluator

# Check status
sudo systemctl status weld-evaluator

# View logs
sudo journalctl -u weld-evaluator -f
```

### Step 9: Deploy Frontend Production Build

```bash
# Install Node.js on RDK X5 (if needed)
curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt install -y nodejs

# Or build on development machine and transfer

cd /opt/weld_evaluator/weld-frontend

# Install dependencies
npm install

# Build production bundle
npm run build

# Build output in: dist/
ls -lh dist/
```

### Step 10: Setup Nginx

```bash
# Install nginx
sudo apt install -y nginx

# Create nginx configuration
sudo nano /etc/nginx/sites-available/weld-evaluator
```

**Nginx configuration:**

```nginx
server {
    listen 80;
    server_name _;

    # Frontend static files
    location / {
        root /opt/weld_evaluator/weld-frontend/dist;
        try_files $uri $uri/ /index.html;
        
        # Cache static assets
        location ~* \.(js|css|png|jpg|jpeg|gif|ico|svg|woff|woff2|ttf|eot)$ {
            expires 1y;
            add_header Cache-Control "public, immutable";
        }
    }

    # Backend API proxy
    location /api {
        proxy_pass http://127.0.0.1:5000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        
        # WebSocket support (for video streaming)
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        
        # Timeouts
        proxy_connect_timeout 60s;
        proxy_send_timeout 60s;
        proxy_read_timeout 60s;
    }
}
```

**Enable nginx configuration:**

```bash
# Create symlink
sudo ln -s /etc/nginx/sites-available/weld-evaluator /etc/nginx/sites-enabled/

# Remove default site
sudo rm /etc/nginx/sites-enabled/default

# Test configuration
sudo nginx -t

# Reload nginx
sudo systemctl reload nginx

# Enable nginx on boot
sudo systemctl enable nginx
```

### Step 11: Configure Firewall

```bash
# Allow SSH (if not already)
sudo ufw allow 22/tcp

# Allow HTTP
sudo ufw allow 80/tcp

# Allow HTTPS (if using SSL)
sudo ufw allow 443/tcp

# Enable firewall
sudo ufw enable

# Check status
sudo ufw status verbose
```

### Step 12: Verify Production Deployment

```bash
# Check backend service
curl http://localhost:5000/api/classes
# Expected: JSON response

# Check frontend (from another machine)
curl http://<rdk-ip>/
# Expected: HTML content

# Check full integration
curl http://<rdk-ip>/api/classes
# Expected: JSON response through nginx proxy
```

**Access from browser:**

Navigate to: `http://<rdk-ip>/`

Expected: Production WeldMaster dashboard

---

## Database Deployment

### Development Database

Location: `demo_weld_evaluator.db` (local, not committed to git)

**Initialize:**

```powershell
python main.py --mode test
```

**Seed with test data:**

```python
# Create test_data.py
from src.database_sqlalchemy import DatabaseService

db = DatabaseService()
db.initialize_database()

# Create test class
class_id = db.create_class("Welding 101", "Instructor Smith")

# Create test students
students = [
    ("Alice Johnson", "Advanced"),
    ("Bob Smith", "Intermediate"),
    ("Charlie Brown", "Beginner")
]

for name, level in students:
    db.create_student(name, class_id, level)

print("Test data created successfully")
```

### Production Database

Location: `/opt/weld_evaluator/production.db`

**Create production database:**

```bash
cd /opt/weld_evaluator

# Set environment variable
export DATABASE_PATH="/opt/weld_evaluator/production.db"

# Initialize
python main.py --mode test

# Set permissions
chmod 660 production.db
chown ubuntu:ubuntu production.db
```

**Update configuration:**

Edit `src/api/app.py`:

```python
import os

# Use environment variable or default
db_path = os.environ.get('DATABASE_PATH', 'demo_weld_evaluator.db')
app.config['SQLALCHEMY_DATABASE_URI'] = f'sqlite:///{db_path}'
```

### Database Backup Strategy

**Manual backup:**

```bash
# Create backup directory
mkdir -p /opt/weld_evaluator/backups

# Backup with timestamp
cp production.db "backups/production_$(date +%Y%m%d_%H%M%S).db"
```

**Automated backup (cron):**

```bash
# Edit crontab
crontab -e

# Add daily backup at 2 AM
0 2 * * * /opt/weld_evaluator/scripts/backup_database.sh
```

**Backup script (`scripts/backup_database.sh`):**

```bash
#!/bin/bash
BACKUP_DIR="/opt/weld_evaluator/backups"
DB_PATH="/opt/weld_evaluator/production.db"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BACKUP_FILE="$BACKUP_DIR/production_$TIMESTAMP.db"

# Create backup
cp "$DB_PATH" "$BACKUP_FILE"

# Compress backup
gzip "$BACKUP_FILE"

# Keep only last 30 days
find "$BACKUP_DIR" -name "*.db.gz" -mtime +30 -delete

echo "Backup completed: ${BACKUP_FILE}.gz"
```

**Make executable:**

```bash
chmod +x scripts/backup_database.sh
```

---

## Frontend Deployment

### Development Build

```powershell
cd "d:\WeldMaster AI Evaluation\weld-frontend"
npm run dev
```

**Configuration:** `vite.config.js`

```javascript
export default {
  server: {
    port: 3000,
    proxy: {
      '/api': {
        target: 'http://localhost:5000',
        changeOrigin: true
      }
    }
  }
}
```

### Production Build

```bash
cd /opt/weld_evaluator/weld-frontend

# Build for production
npm run build

# Output directory
ls -lh dist/

# Expected files:
# - index.html (entry point)
# - assets/*.js (bundled JavaScript)
# - assets/*.css (bundled styles)
```

**Build optimizations:**

- Minification enabled
- Tree-shaking enabled
- Code splitting enabled
- Source maps generated (for debugging)

**Environment variables:**

Create `.env.production`:

```ini
VITE_API_BASE_URL=/api
VITE_APP_VERSION=1.0.0
```

---

## System Configuration

### Vision Pipeline Configuration

Edit: `config/vision_config.yaml`

```yaml
stereo:
  baseline: 120.0                # Camera baseline in mm
  focal_length: 480.0            # Focal length in pixels
  image_width: 1920
  image_height: 1080
  disparity_range: 128           # Max disparity search range
  block_size: 15                 # Stereo matching block size

camera:
  exposure: auto                 # auto or manual
  gain: 1.0                      # Sensor gain (1.0-4.0)
  white_balance: auto
  fps: 30                        # Target frame rate

weld_detection:
  color_lower: [150, 100, 100]   # BGR lower bound
  color_upper: [255, 200, 150]   # BGR upper bound
  erosion_kernel: 5
  dilation_kernel: 5
  min_area: 500                  # Minimum contour area (pixels)

measurement:
  min_bead_width: 2.0            # mm
  max_bead_width: 10.0           # mm
  confidence_threshold: 0.7      # Measurement confidence
```

### Rubric Configuration

Edit: `config/rubric_config.yaml`

```yaml
iso_5817_level_b:
  name: "ISO 5817 Level B (Stringent)"
  bead_width_range: [5.0, 8.0]
  height_range: [1.5, 3.0]
  height_uniformity: 0.8
  porosity_limit: 5
  undercut_limit: 3
  spatter_limit: 10
  pass_threshold: 80

iso_5817_level_d:
  name: "ISO 5817 Level D (Moderate)"
  bead_width_range: [4.0, 10.0]
  height_range: [1.0, 4.0]
  height_uniformity: 0.6
  porosity_limit: 10
  undercut_limit: 5
  spatter_limit: 20
  pass_threshold: 60
```

### API Configuration

Edit: `config/api_config.yaml`

```yaml
server:
  host: "0.0.0.0"
  port: 5000
  debug: false

database:
  path: "/opt/weld_evaluator/production.db"
  backup_enabled: true
  backup_interval: 86400  # 24 hours in seconds

cors:
  enabled: true
  origins: ["http://localhost:3000", "http://<rdk-ip>"]

logging:
  level: "INFO"
  file: "/var/log/weld-evaluator/app.log"
  max_size: 10485760  # 10 MB
  backup_count: 5
```

---

## Security Hardening

### Change Default Credentials

```bash
# Change ubuntu user password
passwd

# Disable password authentication (SSH key only)
sudo nano /etc/ssh/sshd_config
# Set: PasswordAuthentication no
sudo systemctl restart sshd
```

### Setup SSH Keys

```powershell
# On Windows development machine
ssh-keygen -t ed25519 -C "your_email@example.com"

# Copy public key to RDK X5
scp ~/.ssh/id_ed25519.pub ubuntu@<rdk-ip>:~/.ssh/authorized_keys
```

### Database Security

```bash
# Set restrictive permissions
chmod 600 /opt/weld_evaluator/production.db

# Ensure only ubuntu user can access
chown ubuntu:ubuntu /opt/weld_evaluator/production.db
```

### API Security (Future Enhancement)

**Add API authentication** (if needed):

```python
# src/api/app.py
from flask import request, abort

API_KEY = os.environ.get('API_KEY', 'your-secret-key')

@app.before_request
def verify_api_key():
    if request.path.startswith('/api'):
        key = request.headers.get('X-API-Key')
        if key != API_KEY:
            abort(401)
```

### HTTPS Setup (Optional)

```bash
# Install certbot
sudo apt install -y certbot python3-certbot-nginx

# Obtain certificate (requires domain name)
sudo certbot --nginx -d yourdomain.com

# Auto-renewal cron job (certbot installs this automatically)
sudo certbot renew --dry-run
```

---

## Monitoring & Logging

### System Monitoring

**CPU and Memory:**

```bash
# Install htop
sudo apt install -y htop

# Monitor resources
htop
```

**Disk Usage:**

```bash
# Check disk space
df -h

# Check directory sizes
du -sh /opt/weld_evaluator/*
```

### Application Logs

**Backend logs:**

```bash
# View service logs
sudo journalctl -u weld-evaluator -f

# Last 100 lines
sudo journalctl -u weld-evaluator -n 100

# Logs since boot
sudo journalctl -u weld-evaluator -b
```

**Nginx logs:**

```bash
# Access log
sudo tail -f /var/log/nginx/access.log

# Error log
sudo tail -f /var/log/nginx/error.log
```

**Application log rotation:**

Create: `/etc/logrotate.d/weld-evaluator`

```
/var/log/weld-evaluator/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 0644 ubuntu ubuntu
}
```

### Performance Monitoring

**Install monitoring tools:**

```bash
sudo apt install -y sysstat

# Enable statistics collection
sudo systemctl enable sysstat
sudo systemctl start sysstat
```

**Monitor metrics:**

```bash
# CPU usage
mpstat 1 10

# Memory usage
vmstat 1 10

# I/O statistics
iostat -x 1 10
```

---

## Backup & Recovery

### Full System Backup

**Backup checklist:**
- [ ] Database file
- [ ] Configuration files (config/*.yaml)
- [ ] Calibration file (config/camera_calibration.yaml)
- [ ] Custom code changes
- [ ] Environment variables

**Automated backup script:**

```bash
#!/bin/bash
# backup_system.sh

BACKUP_ROOT="/mnt/backup"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="$BACKUP_ROOT/weld_evaluator_$TIMESTAMP"

mkdir -p "$BACKUP_DIR"

# Backup database
cp /opt/weld_evaluator/production.db "$BACKUP_DIR/"

# Backup configuration
cp -r /opt/weld_evaluator/config "$BACKUP_DIR/"

# Backup custom code (if any)
tar -czf "$BACKUP_DIR/custom_code.tar.gz" /opt/weld_evaluator/src

# Create manifest
echo "Backup created: $TIMESTAMP" > "$BACKUP_DIR/manifest.txt"
echo "Database: production.db" >> "$BACKUP_DIR/manifest.txt"
echo "Config: config/" >> "$BACKUP_DIR/manifest.txt"

echo "Backup completed: $BACKUP_DIR"
```

### Recovery Procedure

**Restore from backup:**

```bash
# Stop services
sudo systemctl stop weld-evaluator
sudo systemctl stop nginx

# Restore database
cp /mnt/backup/weld_evaluator_YYYYMMDD_HHMMSS/production.db \
   /opt/weld_evaluator/production.db

# Restore configuration
cp -r /mnt/backup/weld_evaluator_YYYYMMDD_HHMMSS/config/* \
      /opt/weld_evaluator/config/

# Set permissions
chown -R ubuntu:ubuntu /opt/weld_evaluator

# Start services
sudo systemctl start weld-evaluator
sudo systemctl start nginx

# Verify
curl http://localhost:5000/api/classes
```

---

## Troubleshooting

### Backend Issues

**Service won't start:**

```bash
# Check service status
sudo systemctl status weld-evaluator

# View detailed logs
sudo journalctl -u weld-evaluator -n 50 --no-pager

# Common issues:
# 1. Python module not found -> pip install -r requirements.txt
# 2. Port already in use -> kill process on port 5000
# 3. Database locked -> check for stale processes
```

**Port already in use:**

```bash
# Find process using port 5000
sudo lsof -i :5000

# Kill process
sudo kill -9 <PID>

# Or change port in config
```

**Database errors:**

```bash
# Check database file
ls -l /opt/weld_evaluator/production.db

# Check permissions
chmod 660 /opt/weld_evaluator/production.db

# Verify database integrity
sqlite3 production.db "PRAGMA integrity_check;"

# Rebuild if corrupted
mv production.db production.db.backup
python main.py --mode test
```

### Frontend Issues

**Blank page after build:**

```bash
# Check nginx error log
sudo tail -f /var/log/nginx/error.log

# Verify dist files exist
ls -lh /opt/weld_evaluator/weld-frontend/dist/

# Check nginx configuration
sudo nginx -t

# Reload nginx
sudo systemctl reload nginx
```

**API calls failing:**

```bash
# Check nginx proxy
curl -v http://localhost/api/classes

# Check backend directly
curl http://localhost:5000/api/classes

# Verify nginx proxy configuration
sudo cat /etc/nginx/sites-enabled/weld-evaluator | grep proxy_pass
```

### Camera Issues

**Camera not detected:**

```bash
# List video devices
ls -l /dev/video*

# Check USB cameras
lsusb | grep -i camera

# Check MIPI cameras (RDK specific)
cat /sys/class/video4linux/*/name

# Test camera
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
```

**Poor image quality:**

```bash
# Adjust camera parameters in vision_config.yaml
# Try different exposure/gain settings
# Check focus and lens cleanliness
```

### Performance Issues

**Slow evaluation:**

```bash
# Check CPU usage
top -bn1 | grep weld

# Check memory
free -h

# Profile Python code
python -m cProfile -s cumulative main.py --mode demo > profile.txt

# Consider:
# - Reduce image resolution
# - Lower disparity range
# - Optimize algorithms
```

**High latency:**

```bash
# Check network latency
ping <rdk-ip>

# Check nginx response time
curl -w "@curl-format.txt" http://<rdk-ip>/api/classes

# curl-format.txt:
# time_total: %{time_total}s
# time_connect: %{time_connect}s
# time_starttransfer: %{time_starttransfer}s
```

---

## Rollback Procedures

### Rollback Backend

```bash
# Stop current version
sudo systemctl stop weld-evaluator

# Checkout previous version
cd /opt/weld_evaluator
git log --oneline -5
git checkout <previous-commit-hash>

# Reinstall dependencies (if changed)
source venv/bin/activate
pip install -r requirements.txt

# Restart service
sudo systemctl start weld-evaluator

# Verify
curl http://localhost:5000/api/classes
```

### Rollback Database

```bash
# Stop backend
sudo systemctl stop weld-evaluator

# Restore from backup
cp backups/production_YYYYMMDD_HHMMSS.db production.db

# Restart backend
sudo systemctl start weld-evaluator
```

### Rollback Frontend

```bash
# Checkout previous version
cd /opt/weld_evaluator/weld-frontend
git checkout <previous-commit-hash>

# Rebuild
npm install
npm run build

# Reload nginx
sudo systemctl reload nginx
```

---

## Deployment Verification Checklist

### Post-Deployment Checks

- [ ] Backend service running: `sudo systemctl status weld-evaluator`
- [ ] Nginx running: `sudo systemctl status nginx`
- [ ] Database accessible: `sqlite3 production.db ".tables"`
- [ ] API responding: `curl http://localhost:5000/api/classes`
- [ ] Frontend loading: `curl http://localhost/`
- [ ] Camera detected: `ls /dev/video*`
- [ ] Calibration file exists: `ls config/camera_calibration.yaml`
- [ ] Logs rotating: `ls -lh /var/log/weld-evaluator/`
- [ ] Backup script working: `./scripts/backup_database.sh`
- [ ] Firewall configured: `sudo ufw status`

### Functional Tests

1. **API Tests:**
   ```bash
   # Get classes
   curl http://localhost:5000/api/classes
   
   # Get students
   curl http://localhost:5000/api/students
   
   # Health check
   curl http://localhost:5000/api/health
   ```

2. **Frontend Tests:**
   - Open dashboard: `http://<rdk-ip>/`
   - Navigate all views
   - Test student creation
   - Test rubric selection
   - Perform mock evaluation

3. **Camera Tests:**
   ```bash
   # Capture test image
   python -c "import cv2; cam = cv2.VideoCapture(0); ret, frame = cam.read(); cv2.imwrite('test.jpg', frame)"
   
   # Verify image
   ls -lh test.jpg
   ```

4. **Database Tests:**
   ```bash
   # Run validation
   python main.py --mode test
   ```

---

## Support & Maintenance

### Regular Maintenance Tasks

**Weekly:**
- Check disk space: `df -h`
- Review error logs: `sudo journalctl -u weld-evaluator --since "1 week ago" | grep ERROR`
- Verify backups: `ls -lh backups/`

**Monthly:**
- Update system packages: `sudo apt update && sudo apt upgrade`
- Rotate old backups: `find backups/ -mtime +30 -delete`
- Review performance metrics

**Quarterly:**
- Update Python dependencies: `pip list --outdated`
- Review security patches
- Test backup restoration procedure

### Getting Help

1. Check documentation:
   - [RDK_X5_DEPLOYMENT_GUIDE.md](RDK_X5_DEPLOYMENT_GUIDE.md)
   - [TECHNICAL_IMPLEMENTATION.md](TECHNICAL_IMPLEMENTATION.md)
   - [README.md](README.md)

2. Check logs:
   ```bash
   sudo journalctl -u weld-evaluator -n 100
   ```

3. GitHub Issues: Create issue with:
   - System information (`uname -a`)
   - Error logs
   - Steps to reproduce
   - Expected vs actual behavior

---

**Document End**

*For technical details, see [TECHNICAL_IMPLEMENTATION.md](TECHNICAL_IMPLEMENTATION.md)*  
*For quick setup, see [QUICK_START.md](QUICK_START.md)*  
*For operations, see [RDK_X5_DEPLOYMENT_GUIDE.md](RDK_X5_DEPLOYMENT_GUIDE.md)*

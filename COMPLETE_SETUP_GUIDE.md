# WeldVision X5 - Complete Setup Guide

**Desktop + RDK X5 Deployment**

---

## Table of Contents

1. [Desktop Setup (Training Workstation)](#desktop-setup)
2. [RDK X5 Setup (Edge Inference Device)](#rdk-x5-setup)
3. [Network Configuration](#network-configuration)
4. [End-to-End Workflow](#end-to-end-workflow)
5. [Troubleshooting](#troubleshooting)

---

# Part 1: Desktop Setup (Training Workstation)

## Overview

The desktop computer hosts:
- **Web UI (React frontend)** - Browser-based interface for control and monitoring
- **Backend API (Flask)** - Executes training, handles file operations, runs Python scripts
- **Model training pipeline** - PyTorch/YOLO training executed by backend
- **Dataset management** - File storage and organization
- **All data storage** - Images, labels, trained models

**How it works:**
1. You open the web UI in your browser (http://localhost:3000)
2. You configure and start training from the UI
3. The UI sends commands to the backend API
4. The backend executes the actual training (GPU computation)
5. Progress updates stream back to the UI in real-time

**Requirements:**
- Windows 10/11 (or Linux/macOS)
- 8GB RAM minimum, 16GB recommended
- NVIDIA GPU recommended (for training)
- 50GB free disk space
- Python 3.9+
- Node.js 16+

---

## Step 1: Install Prerequisites

### Install Python 3.9+

**Download:**
https://www.python.org/downloads/

**Installation:**
1. Run installer
2. ✅ Check "Add Python to PATH"
3. Click "Install Now"
4. Verify:
   ```powershell
   python --version
   # Expected: Python 3.9.x or higher
   ```

### Install Node.js 16+

**Download:**
https://nodejs.org/ (LTS version)

**Installation:**
1. Run installer
2. Accept defaults
3. Verify:
   ```powershell
   node --version
   npm --version
   # Expected: v16.x.x or higher
   ```

### Install Git (Optional)

**Download:**
https://git-scm.com/download/win

Needed only if cloning from GitHub.

---

## Step 2: Extract Application

**Option A: From GitHub**
```powershell
cd D:\
git clone https://github.com/wilsonintai76/weldvisionX5.git "WeldMaster AI Evaluation"
cd "WeldMaster AI Evaluation"
```

**Option B: From ZIP**
1. Extract to `D:\WeldMaster AI Evaluation\`
2. Open PowerShell:
   ```powershell
   cd "D:\WeldMaster AI Evaluation"
   ```

---

## Step 3: Install Dependencies

### Backend Dependencies

```powershell
# Navigate to project
cd "D:\WeldMaster AI Evaluation"

# Install Python packages
pip install -r backend/requirements.txt

# Verify installation
pip list | Select-String -Pattern "flask|opencv"
```

**Expected packages:**
- Flask 3.0+
- opencv-python 4.5+
- numpy 1.24+
- Pillow 10.0+

### Frontend Dependencies

```powershell
# Still in project root
npm install

# This installs React, Vite, and all dependencies
```

---

## Step 4: Configure Network Settings

### Option A: Localhost Only (Single Computer)

**No configuration needed!** Skip to Step 5.

### Option B: Network Access (Multiple Computers)

**Find your IP address:**
```powershell
ipconfig
# Look for "IPv4 Address" under your network adapter
# Example: 10.80.16.151
```

**Update `.env.local` (if it exists):**
```env
VITE_API_URL=http://YOUR_IP_ADDRESS:5000
```

**Or just use localhost - it works fine!**

---

## Step 5: Start the Application

### Method 1: Batch File (Easiest)

**Double-click:**
```
Start-WeldVision.bat
```

This automatically:
1. Starts backend (Flask) on port 5000
2. Starts frontend (Vite) on port 3000
3. Opens browser

### Method 2: Manual Start

**Terminal 1 - Backend:**
```powershell
cd "D:\WeldMaster AI Evaluation\backend"
python app.py
```

**Terminal 2 - Frontend:**
```powershell
cd "D:\WeldMaster AI Evaluation"
npm run dev
```

**Browser:**
```
http://localhost:3000
```

---

## Step 6: Verify Installation

**Open browser to:** `http://localhost:3000`

**Check:**
- ✅ Dashboard loads
- ✅ No error messages in browser console (F12)
- ✅ Backend terminal shows "Running on http://0.0.0.0:5000"
- ✅ Frontend terminal shows "Local: http://localhost:3000"

**Test Navigation:**
1. Click "Dataset Studio" - Should load
2. Click "Model Training" - Should load
3. Click "Model Management" - Should load

**✅ Success!** Your desktop is ready.

### Important: How the System Works

**Everything happens through your web browser:**
- 🌐 **Frontend (Port 3000):** Web UI you see in browser - buttons, forms, displays
- ⚙️ **Backend (Port 5000):** Python server that does the actual work - training, file operations, API calls

**You never need to run Python commands manually!**

When you click "Start Training" in the browser:
1. UI sends HTTP request to backend
2. Backend runs Python training script
3. Progress updates sent back to UI
4. You see real-time progress in browser

**Both terminals must stay open:**
- Terminal 1: Backend (Flask) - Does the heavy lifting
- Terminal 2: Frontend (Vite) - Serves the web UI

---

## Desktop Directory Structure

```
D:\WeldMaster AI Evaluation\
├── backend/
│   ├── app.py              # Flask server
│   ├── requirements.txt    # Python dependencies
│   └── weld_data.db        # SQLite database
├── components/             # React components
├── api/                    # API wrappers
├── index.html             # Main HTML
├── App.tsx                # Main React app
├── package.json           # Node dependencies
├── Start-WeldVision.bat   # Easy launcher
└── README.md
```

---

# Part 2: RDK X5 Setup (Edge Inference Device)

## Overview

The RDK X5 is an edge device for:
- Real-time camera capture
- BPU-accelerated inference
- Weld scanning during live welding

**NOT used for:**
- Training (done on desktop)
- Web UI hosting (desktop only)
- Dataset storage (desktop only)

---

## Step 1: RDK X5 Prerequisites

### Hardware Checklist

- ✅ RDK X5 board (Horizon Sunrise X5)
- ✅ Power supply (12V 2A)
- ✅ MicroSD card (32GB+, Class 10)
- ✅ Ethernet cable OR WiFi adapter
- ✅ RDK Stereo Camera (optional but recommended)
- ✅ USB cable (for serial/debugging)
- ✅ Heatsink or cooling fan

### Network Access

**Option A: Ethernet via Router (Best for Production)**
- Connect RDK X5 to same network as desktop via router
- RDK will get IP via DHCP
- Best for permanent installations
- Allows multiple devices

**Option B: Direct Ethernet P2P (Best for Portability)** ⭐
- Ethernet cable directly between desktop and RDK X5
- No router needed - point-to-point connection
- Gigabit speed (faster than USB)
- Ideal for portable rigs, testing, demos
- See "Direct Ethernet Setup" below

**Option C: USB Direct Connection (Best for Setup/Testing)**
- USB cable directly between desktop and RDK X5
- No router needed - point-to-point connection
- Ideal for initial setup and debugging
- See "USB Connection Setup" below

**Option D: WiFi**
- USB WiFi adapter on RDK X5
- Configure during setup
- Good for wireless deployments

---

## Step 2: Flash RDK X5 Operating System

### Download RDK OS Image

**Official Source:**
https://developer.horizon.cc/rdkx5

**Recommended:** Ubuntu 20.04 for RDK X5

### Flash to MicroSD

**Tool:** Balena Etcher or Win32DiskImager

**Steps:**
1. Insert microSD into computer
2. Open Etcher
3. Select downloaded `.img` file
4. Select microSD drive
5. Click "Flash"
6. Wait ~15 minutes

### First Boot

1. Insert microSD into RDK X5
2. Connect power
3. Wait 2-3 minutes for boot
4. Connect via Ethernet or USB-Serial

---

## Step 3: Connect to RDK X5

### Find RDK IP Address

**Method 1: Router Admin Page**
- Login to router
- Look for device named "rdk-x5" or "horizon"

**Method 2: Network Scan**
```powershell
# On Windows (if nmap installed)
nmap -sn 192.168.1.0/24
```

**Default IP:** Usually 192.168.1.xxx or 10.0.0.xxx

### SSH Access

**Default credentials:**
- Username: `root` or `ubuntu`
- Password: `root` or `ubuntu`

**Connect:**
```powershell
ssh root@RDK_IP_ADDRESS
# Example: ssh root@192.168.1.100
```

**Change password immediately:**
```bash
passwd
# Enter new password
```

---

## Step 3A: Direct Ethernet P2P Connection (Alternative)

### Overview

Connect desktop directly to RDK X5 with single Ethernet cable - no router needed!

**Advantages:**
- ✅ Simple point-to-point connection
- ✅ No router/switch required
- ✅ Gigabit speed (1000 Mbps) - faster than USB
- ✅ Perfect for portable rigs and mobile carts
- ✅ Isolated from network issues
- ✅ Works with standard Ethernet cable (auto-MDIX)

### Hardware Requirements

**You need:**
- Standard Ethernet cable (Cat5e or better)
- RDK X5 with Ethernet port
- Desktop with Ethernet port (or USB-to-Ethernet adapter)

**Note:** Modern network cards support auto-MDIX, so you don't need a crossover cable - regular Ethernet cable works fine!

### Setup Steps

#### 1. Physical Connection

```
Desktop Ethernet Port ←→ Ethernet Cable ←→ RDK X5 Ethernet Port
```

**Connect the cable** between desktop and RDK X5 directly.

#### 2. Configure Desktop (Windows)

**Open Network Settings:**

```powershell
# Option 1: GUI method
# Press Win+R → ncpa.cpl → Enter
```

Or:

1. Settings → Network & Internet
2. Ethernet → Change adapter options
3. Find your Ethernet adapter

**Configure Static IP:**

```
Right-click Ethernet adapter → Properties
→ Internet Protocol Version 4 (TCP/IPv4) → Properties
→ Use the following IP address:

IP Address: 10.0.0.1
Subnet Mask: 255.255.255.0
Default Gateway: (leave blank)
Preferred DNS: (leave blank)

Click OK
```

**Or via PowerShell (Admin):**

```powershell
# Find your Ethernet adapter name
Get-NetAdapter | Where-Object {$_.Status -eq "Up"}

# Set static IP (replace "Ethernet" with your adapter name)
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 10.0.0.1 -PrefixLength 24

# Remove default gateway if asked
Remove-NetRoute -InterfaceAlias "Ethernet" -Confirm:$false -ErrorAction SilentlyContinue
```

#### 3. Configure RDK X5

**First, connect via serial/USB or existing network to configure.**

**Edit netplan configuration:**

```bash
# SSH to RDK (via serial or temporary connection)
ssh root@RDK_TEMP_IP

# Edit network config
sudo nano /etc/netplan/01-netcfg.yaml
```

**Replace with P2P configuration:**

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 10.0.0.2/24
```

**Apply configuration:**

```bash
sudo netplan apply

# Verify
ip addr show eth0
# Should show: inet 10.0.0.2/24
```

#### 4. Test Connection

**From Desktop:**

```powershell
# Test connectivity
ping 10.0.0.2

# Should reply:
# Reply from 10.0.0.2: bytes=32 time<1ms TTL=64
```

**From RDK:**

```bash
ping 10.0.0.1

# Should reply:
# 64 bytes from 10.0.0.1: icmp_seq=1 ttl=128 time=0.5 ms
```

#### 5. SSH and File Transfer

**SSH via P2P Ethernet:**

```powershell
ssh root@10.0.0.2
```

**Transfer files:**

```powershell
# Desktop → RDK
scp model.bin root@10.0.0.2:/opt/weldvision/models/

# RDK → Desktop
scp root@10.0.0.2:/var/log/inference.log D:\logs\
```

### P2P Ethernet IP Scheme

```
Desktop:  10.0.0.1
RDK X5:   10.0.0.2
Subnet:   255.255.255.0 (/24)
Gateway:  None (P2P doesn't need gateway)
```

### Connection Comparison

| Feature | P2P Ethernet | Via Router | USB Direct |
|---------|--------------|------------|------------|
| **Speed** | ~1000 Mbps | ~1000 Mbps | ~480 Mbps |
| **Setup** | Medium | Medium | Fast |
| **Hardware** | Ethernet cable | Router + 2 cables | USB cable |
| **Portability** | ✅ Excellent | ❌ Need router | ✅ Excellent |
| **Cost** | $5 cable | $30+ router | $3 cable |
| **Best For** | Mobile rigs | Fixed installs | Quick setup |

### Internet Access While Connected to RDK

**Important:** P2P Ethernet uses your Ethernet port!

**Scenario 1: Desktop has WiFi + Ethernet**
- ✅ **Best solution!**
- Connect WiFi → Internet
- Connect Ethernet → RDK (P2P)
- Both work simultaneously!

**Scenario 2: Desktop has only one Ethernet port**

**Option A: USB-to-Ethernet Adapter** ⭐ Recommended
```
Desktop:
├── Built-in Ethernet → RDK (P2P: 10.0.0.1)
└── USB Ethernet Adapter → Router (Internet via DHCP)
```

**Or reverse:**
```
Desktop:
├── Built-in Ethernet → Router (Internet)
└── USB Ethernet Adapter → RDK (P2P: 10.0.0.1)
```

**Option B: Use USB Direct Connection**
```
Desktop:
├── Ethernet → Router (Internet)
└── USB → RDK (192.168.7.2)
```

**Option C: Connect both to same router**
```
Router → Desktop (Internet + RDK access)
Router → RDK (Accessible from desktop)
```

**Option D: Internet Connection Sharing (Windows)**
```
Desktop WiFi → Internet
Desktop shares via Ethernet → RDK gets internet too!
```

### Recommended Setups by Use Case

| Use Case | Best Connection | Internet Access |
|----------|----------------|-----------------|
| **Office/Lab** | Router network | ✅ Desktop + RDK both have internet |
| **Mobile Cart (WiFi available)** | WiFi→Internet, Ethernet→RDK | ✅ Desktop has internet |
| **Mobile Cart (no WiFi)** | USB→RDK, Ethernet→Hotspot | ✅ Desktop has internet |
| **Field (no internet)** | P2P Ethernet or USB direct | ❌ Offline (training done beforehand) |

### When to Use P2P Ethernet

**Perfect for:**
- ✅ Mobile welding inspection carts
- ✅ Portable demonstration systems
- ✅ Field deployments without infrastructure
- ✅ High-speed model transfers
- ✅ Testing new models quickly
- ✅ When desktop has WiFi for internet

**Not ideal for:**
- ❌ Desktop with only one Ethernet port (no WiFi)
- ❌ Permanent installations (use router)
- ❌ Multiple devices (router better)
- ❌ Remote monitoring from multiple PCs

### Switching Between Connections

**You can easily switch between P2P and router:**

**On Desktop:**
```powershell
# Switch to P2P
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 10.0.0.1 -PrefixLength 24

# Switch to DHCP (for router)
Set-NetIPInterface -InterfaceAlias "Ethernet" -Dhcp Enabled
```

**On RDK:**
```bash
# P2P config (as shown above)
sudo nano /etc/netplan/01-netcfg.yaml
# Set: addresses: - 10.0.0.2/24

# Or router/DHCP config
# Set: dhcp4: yes

sudo netplan apply
```

---

## Step 3B: USB Direct Connection (Alternative)

### Overview

Connect desktop directly to RDK X5 via USB cable - no router needed!

**Advantages:**
- ✅ Simple point-to-point connection
- ✅ No network infrastructure required
- ✅ Perfect for testing and initial setup
- ✅ Faster transfer speeds than WiFi
- ✅ Isolated from network issues

### USB Connection Methods

#### Method 1: USB-Serial (Console Access)

**Hardware:**
- USB Type-A to Micro-USB or USB-C cable
- Connects to RDK X5 debug/serial port

**On Windows - Install Driver:**
```powershell
# Download and install CP210x or FTDI driver
# (depends on RDK X5 USB-serial chip)
```

**Connect:**
1. Plug USB cable: Desktop ↔ RDK X5 serial port
2. Open Device Manager → Ports (COM & LPT)
3. Note COM port number (e.g., COM3)

**Access via PuTTY:**
```
Connection Type: Serial
Serial Line: COM3
Speed: 115200
```

**Or PowerShell:**
```powershell
# Install PuTTY
winget install PuTTY.PuTTY

# Connect
putty -serial COM3 -sercfg 115200,8,n,1,N
```

#### Method 2: USB-Ethernet (RNDIS/Network)

**Better for file transfers and API access!**

**On RDK X5 - Enable USB Networking:**

```bash
# SSH via serial or existing connection first
ssh root@RDK_IP

# Load USB gadget modules
sudo modprobe g_ether

# Or configure persistently
sudo nano /etc/modules
# Add line: g_ether

# Configure USB network interface
sudo nano /etc/network/interfaces.d/usb0
```

**Add to file:**
```
auto usb0
iface usb0 inet static
    address 192.168.7.2
    netmask 255.255.255.0
```

**Apply:**
```bash
sudo ifup usb0
```

**On Windows Desktop:**

1. **Plug USB cable** (Desktop ↔ RDK X5)

2. **Windows will detect new network adapter:**
   - Open Settings → Network & Internet → Change adapter options
   - Look for "RNDIS/Ethernet Gadget" or "USB Ethernet"

3. **Configure Windows USB network adapter:**
   ```
   Right-click adapter → Properties → IPv4 → Use the following IP:
   IP Address: 192.168.7.1
   Subnet Mask: 255.255.255.0
   Gateway: (leave blank)
   ```

4. **Test connection:**
   ```powershell
   ping 192.168.7.2
   # Should reply from RDK X5
   ```

5. **SSH via USB:**
   ```powershell
   ssh root@192.168.7.2
   ```

### USB vs Ethernet Comparison

| Feature | USB Direct | Ethernet via Router |
|---------|------------|--------------------|
| **Setup Speed** | Fast (no router config) | Medium (need router) |
| **Transfer Speed** | USB 2.0: ~480 Mbps | Gigabit: ~1000 Mbps |
| **Portability** | ✅ Excellent | ❌ Need router |
| **Multiple Devices** | ❌ One-to-one only | ✅ Many devices |
| **Best For** | Testing, setup, demos | Production, permanent |

### Recommended Workflow

**Phase 1: Initial Setup → Use USB**
- Connect via USB for first-time configuration
- Test inference service
- Transfer initial models

**Phase 2: Production → Switch to Ethernet**
- Install in fixed location with network
- Permanent connection
- Remote access from multiple computers

---

## Step 4: Configure RDK X5 Network

### Static IP (Recommended for Production)

**Edit network config:**
```bash
sudo nano /etc/netplan/01-netcfg.yaml
```

**Add static IP:**
```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

**Apply:**
```bash
sudo netplan apply
```

### Verify Connectivity

**From RDK, ping desktop:**
```bash
ping DESKTOP_IP_ADDRESS
# Example: ping 10.80.16.151
```

**From desktop, ping RDK:**
```powershell
ping RDK_IP_ADDRESS
# Example: ping 192.168.1.100
```

---

## Step 5: Install Dependencies on RDK X5

### Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### Install Python and Tools

```bash
# Install Python 3 and pip
sudo apt install -y python3 python3-pip python3-dev

# Install system libraries
sudo apt install -y \
    libopencv-dev \
    python3-opencv \
    git \
    wget \
    curl
```

### Install ROS2 (If Required)

**For RDK X5 with TROS (Together ROS):**
```bash
# Follow official RDK documentation
# https://developer.horizon.cc/tros
```

---

## Step 6: Install Camera Drivers

### RDK Stereo Camera

**Check camera detection:**
```bash
ls /dev/video*
# Expected: /dev/video0, /dev/video1
```

**Test camera:**
```bash
v4l2-ctl --list-devices
```

### Install hobot_dnn (BPU Runtime)

**Official Horizon packages:**
```bash
# Add Horizon repository
echo "deb http://sunrise.horizon.cc/ubuntu-ports focal main" | sudo tee /etc/apt/sources.list.d/horizon.list

# Install hobot_dnn
sudo apt update
sudo apt install -y hobot-dnn
```

---

## Step 7: Deploy Inference Service

### Create Inference Directory

```bash
# Create app directory
sudo mkdir -p /opt/weldvision
cd /opt/weldvision

# Create model directory
sudo mkdir -p models
sudo mkdir -p logs
```

### Transfer Inference Code from Desktop

**On desktop (PowerShell):**
```powershell
# Navigate to backend
cd "D:\WeldMaster AI Evaluation\backend"

# Transfer inference files
scp inference_service.py root@RDK_IP:/opt/weldvision/
scp -r models/ root@RDK_IP:/opt/weldvision/
```

### Install Python Dependencies on RDK

```bash
# On RDK X5
cd /opt/weldvision

# Create requirements file
cat > requirements.txt << EOF
flask==3.0.0
numpy==1.24.0
opencv-python==4.8.0
pillow==10.0.0
EOF

# Install
pip3 install -r requirements.txt
```

---

## Step 8: Create Systemd Service

### Create Service File

```bash
sudo nano /etc/systemd/system/weldvision-inference.service
```

**Add content:**
```ini
[Unit]
Description=WeldVision X5 Inference Service
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/opt/weldvision
ExecStart=/usr/bin/python3 inference_service.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

### Enable and Start Service

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable on boot
sudo systemctl enable weldvision-inference

# Start service
sudo systemctl start weldvision-inference

# Check status
sudo systemctl status weldvision-inference
```

### View Logs

```bash
# Real-time logs
sudo journalctl -u weldvision-inference -f

# Recent logs
sudo journalctl -u weldvision-inference -n 50
```

---

## Step 9: Deploy Trained Model to RDK

### From Desktop After Training

**Train model on desktop first** (via Model Training page)

**After training completes:**

**1. Compile to Horizon format:**
```powershell
# On desktop - use Orchestration Panel
# Click "Start Compile" after training
# Output: D:\models\horizon\model.bin
```

**2. Transfer to RDK:**
```powershell
# On desktop
scp "D:\models\horizon\model.bin" root@RDK_IP:/opt/weldvision/models/
```

**3. Restart inference service:**
```bash
# On RDK X5
sudo systemctl restart weldvision-inference
```

---

# Part 3: Network Configuration

## Network Topology

### Option 1: Network via Router (Production)

```
┌─────────────────────────────────────┐
│   Desktop Workstation               │
│   IP: 10.80.16.151 (DHCP)           │
│   • Web UI (port 3000)              │
│   • Backend API (port 5000)         │
│   • Training Pipeline               │
│   • Dataset Storage                 │
└──────────┬──────────────────────────┘
           │
           │ Ethernet
           │
┌──────────▼──────────────────────────┐
│   Network Router                    │
│   Gateway: 192.168.1.1              │
└──────────┬──────────────────────────┘
           │
           │ Ethernet
           │
┌──────────▼──────────────────────────┐
│   RDK X5 Edge Device                │
│   IP: 192.168.1.100 (DHCP)          │
│   • Camera Capture                  │
│   • BPU Inference                   │
│   • Inference Service (port 8080)   │
└─────────────────────────────────────┘
```

### Option 2: Direct Ethernet P2P (Portable Rig)

```
┌─────────────────────────────────────┐
│   Desktop Workstation               │
│   P2P IP: 10.0.0.1 (Static)         │
│   • Web UI (port 3000)              │
│   • Backend API (port 5000)         │
│   • Training Pipeline               │
│   • Dataset Storage                 │
└──────────┬──────────────────────────┘
           │
           │ Ethernet Cable (Direct)
           │ No Router! Gigabit Speed!
           │
┌──────────▼──────────────────────────┐
│   RDK X5 Edge Device                │
│   P2P IP: 10.0.0.2 (Static)         │
│   • Camera Capture                  │
│   • BPU Inference                   │
│   • Inference Service (port 8080)   │
└─────────────────────────────────────┘
```

### Option 3: Direct USB Connection (Setup/Testing)

```
┌─────────────────────────────────────┐
│   Desktop Workstation               │
│   USB IP: 192.168.7.1               │
│   • Web UI (port 3000)              │
│   • Backend API (port 5000)         │
│   • Training Pipeline               │
│   • Dataset Storage                 │
└──────────┬──────────────────────────┘
           │
           │ USB Cable (Direct)
           │ No Router Needed!
           │
┌──────────▼──────────────────────────┐
│   RDK X5 Edge Device                │
│   USB IP: 192.168.7.2               │
│   • Camera Capture                  │
│   • BPU Inference                   │
│   • Inference Service (port 8080)   │
└─────────────────────────────────────┘
```

## Firewall Configuration

### Desktop (Windows)

**Allow ports in Windows Firewall:**

```powershell
# Frontend
New-NetFirewallRule -DisplayName "WeldVision Frontend" -Direction Inbound -LocalPort 3000 -Protocol TCP -Action Allow

# Backend
New-NetFirewallRule -DisplayName "WeldVision Backend" -Direction Inbound -LocalPort 5000 -Protocol TCP -Action Allow
```

### RDK X5 (Linux)

**Configure UFW firewall:**
```bash
# Enable firewall
sudo ufw enable

# Allow SSH
sudo ufw allow 22/tcp

# Allow inference service
sudo ufw allow 8080/tcp

# Check status
sudo ufw status
```

---

# Part 4: End-to-End Workflow

## Complete Training and Deployment Flow

### Phase 1: Dataset Creation (Desktop)

1. **Open WeldVision on desktop:**
   ```
   http://localhost:3000
   ```

2. **Navigate to Dataset Studio**

3. **Capture images:**
   - Use webcam or transfer from RDK
   - Click "Capture" to save images

4. **Label images:**
   - Draw bounding boxes on defects
   - Assign classes (porosity, undercut, etc.)
   - Save labels

### Phase 2: Model Training (Desktop)

1. **Open web UI in browser:**
   ```
   http://localhost:3000
   ```

2. **Navigate to "Model Training" page**

3. **Configure in Orchestration Panel:**
   - Dataset Path: `D:\data\weldsets\setA`
   - Epochs: `50` (test) or `100+` (production)
   - Click "Sync to Dataset Studio" to use current dataset

4. **Start Training:**
   - Click "Start Training" button in the web UI
   - Backend starts Python training process
   - Monitor progress in real-time (UI shows live updates)
   - Wait for completion (1-4 hours depending on dataset size)

**Note:** Training runs on the backend (Flask/Python), but you control everything from the browser UI. You can close the browser - training continues on the backend.

### Phase 3: Model Compilation (Desktop)

1. **In Orchestration Panel (same web UI):**
   - ONNX Path: `D:\models\yolo.onnx` (auto-filled after training)
   - Output Dir: `D:\models\horizon`

2. **Start Compile:**
   - Click "Start Compile" button in the web UI
   - Backend runs Horizon compilation tools
   - Converts ONNX model to RDK BPU format
   - Output: `model.bin` (optimized for RDK X5)

**Note:** This also runs on the backend. The UI sends the command, backend does the compilation work.

### Phase 4: Deployment (Desktop → RDK)

**The system automatically checks RDK connectivity before deployment!**

1. **In Orchestration Panel (same web UI):**
   - Device Host: `192.168.1.100` (or `10.0.0.2` for P2P, `192.168.7.2` for USB)
   - Device User: `root`
   - Dest Path: `/opt/weldvision/models/model.bin`

2. **Connection Status Indicator:**
   - 🟢 **Green:** RDK is reachable and responding
   - 🔴 **Red:** Cannot connect to RDK
   - 🟡 **Yellow:** Checking connection...

3. **Deploy:**
   - Click "Deploy" button in the web UI
   - Backend tests connection first (SSH ping)
   - If connected: Transfers model file to RDK via SCP
   - If disconnected: Shows error "Cannot reach RDK at 192.168.1.100"
   - Backend sends restart command to RDK service
   - Deployment complete in seconds

**Note:** You do everything from the browser. Backend handles the actual file transfer and RDK communication.

### Phase 5: Inference (RDK)

1. **Start inference from Orchestration Panel (web UI):**
   - Click "Start Inference" button
   - Backend sends start command to RDK
   - RDK camera activates and begins real-time detection

2. **View results in browser:**
   - Results stream back from RDK to backend to your UI
   - Real-time defect detection displayed in browser
   - Live video feed with bounding boxes
   - 30-40 FPS on RDK BPU

**Note:** Everything is controlled from your browser. The RDK runs autonomously, but you monitor and control it via the web UI.

---

# Part 6: Connection Monitoring & Health Checks

## RDK Connection Status

### "Cannot connect to RDK" error in UI

**Check connection:**
```powershell
# Test if RDK is reachable
ping RDK_IP_ADDRESS

# Test SSH
ssh root@RDK_IP_ADDRESS

# If both fail, check:
# 1. Cable is plugged in
# 2. IP address is correct in UI settings
# 3. RDK is powered on
```

**Update RDK IP in UI:**
1. Model Management → Device Settings
2. Update "Device Host" field
3. Click "Save" and "Test Connection"

### Automatic Detection

**The application continuously monitors RDK connectivity:**

```
Desktop Backend → Checks RDK every 30 seconds
                ↓
           Is RDK reachable?
                ↓
         Yes → 🟢 Connected
         No  → 🔴 Disconnected
```

### How It Works

**Backend performs health checks:**

1. **Ping Test:**
   ```python
   # Backend checks if RDK responds
   ping RDK_IP -n 1
   # Success: RDK is online
   ```

2. **SSH Test:**
   ```python
   # Try to establish SSH connection
   ssh -o ConnectTimeout=3 root@RDK_IP "echo connected"
   # Success: RDK is accessible
   ```

3. **Service Test:**
   ```python
   # Check if inference service is running
   curl http://RDK_IP:8080/health
   # Success: Service is ready
   ```

### UI Indicators

**In Model Management / Orchestration Panel:**

```
┌─────────────────────────────────────┐
│ RDK Device Status                   │
├─────────────────────────────────────┤
│ 🟢 Connected to 192.168.1.100       │
│    Inference Service: Running       │
│    Last Seen: 2 seconds ago         │
└─────────────────────────────────────┘
```

**When disconnected:**

```
┌─────────────────────────────────────┐
│ RDK Device Status                   │
├─────────────────────────────────────┤
│ 🔴 Cannot reach 192.168.1.100       │
│    Check network connection         │
│    [Test Connection] [View Logs]    │
└─────────────────────────────────────┘
```

### Manual Connection Test

**From Web UI:**

1. Navigate to **Model Management**
2. Click **"Test RDK Connection"** button
3. System performs:
   - Network ping
   - SSH accessibility check
   - Service health check
4. Results displayed with diagnostics

**From Terminal (Manual):**

```powershell
# Test network connectivity
ping 192.168.1.100

# Test SSH
ssh root@192.168.1.100 "echo OK"

# Test inference service
curl http://192.168.1.100:8080/health
```

### API Endpoints for Connection Status

**Backend provides REST API:**

```javascript
// Check RDK connectivity
GET /api/rdk/status

Response:
{
  "connected": true,
  "ip": "192.168.1.100",
  "inferenceServiceRunning": true,
  "lastSeen": "2024-12-17T10:30:45Z",
  "latencyMs": 12
}
```

```javascript
// Test connection on demand
POST /api/rdk/test-connection
{
  "host": "192.168.1.100",
  "user": "root"
}

Response:
{
  "reachable": true,
  "sshAvailable": true,
  "serviceHealthy": true,
  "message": "RDK is ready"
}
```

### Troubleshooting Connection Issues from UI

**The UI helps diagnose problems:**

1. **Network Issue:**
   ```
   ❌ Ping failed
   → Check Ethernet/USB cable
   → Verify IP address in settings
   ```

2. **SSH Issue:**
   ```
   ❌ SSH connection refused
   → RDK might be booting (wait 2 minutes)
   → Check SSH credentials
   ```

3. **Service Issue:**
   ```
   ❌ Inference service not responding
   → Service might be stopped
   → Click "Restart Service" in UI
   ```

### Automatic Reconnection

**Backend handles reconnection automatically:**

```
Connection Lost → Retry every 30 seconds
                ↓
           Reconnected?
                ↓
         Yes → Resume operations
         No  → Keep trying (show notification)
```

**User sees toast notifications:**
- "🔴 Lost connection to RDK"
- "🟡 Attempting to reconnect..."
- "🟢 Reconnected to RDK!"

### Connection History Log

**View connection events:**

```
[10:30:45] Connected to RDK at 192.168.1.100
[10:35:12] Deployed model: yolo_weld_v3.bin
[10:40:30] Connection lost (timeout)
[10:41:00] Reconnected to RDK
[10:45:15] Started inference job #42
```

Access via: **Model Management → Connection Log**

---

# Part 7: Troubleshooting

## Desktop Issues

### Backend won't start

**Error: "Port 5000 already in use"**

```powershell
# Find process using port
netstat -ano | findstr :5000

# Kill process
taskkill /PID <PID> /F
```

### Frontend won't start

**Error: "Port 3000 already in use"**

```powershell
# Stop other Vite processes
Get-Process node | Stop-Process -Force
```

### Cannot access from other computer

**Check firewall:**
```powershell
# Test from other computer
Test-NetConnection -ComputerName DESKTOP_IP -Port 3000
```

## RDK X5 Issues

### Cannot SSH to RDK

1. Check network cable
2. Verify IP address: `ipconfig` (desktop) / `ip addr` (RDK)
3. Try default IPs: `192.168.1.100`, `10.0.0.1`

### Camera not detected

```bash
# Check devices
ls /dev/video*

# Test camera
v4l2-ctl --device=/dev/video0 --all
```

### Inference service crashes

```bash
# Check logs
sudo journalctl -u weldvision-inference -n 100

# Restart service
sudo systemctl restart weldvision-inference
```

### Model file missing

```bash
# Check model exists
ls -lh /opt/weldvision/models/

# If missing, re-deploy from desktop
```

## P2P Ethernet Issues

### No link light on Ethernet port

**Check:**
```powershell
# On Windows - verify adapter is up
Get-NetAdapter | Where-Object {$_.Name -like "*Ethernet*"}

# Should show: Status = Up
```

```bash
# On RDK - verify interface is up
ip link show eth0

# Should show: state UP
```

### Cannot ping RDK over P2P Ethernet

**Desktop side:**
```powershell
# Check IP configuration
ipconfig

# Should see Ethernet adapter with 10.0.0.1

# Disable Windows Firewall temporarily to test
Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled False

# Test again
ping 10.0.0.2

# Re-enable firewall
Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled True
```

**RDK side:**
```bash
# Verify IP
ip addr show eth0

# Should show: 10.0.0.2/24

# Test from RDK to Desktop
ping 10.0.0.1
```

### "No Internet" warning on Windows

**This is normal for P2P!** Windows expects Ethernet to have internet access.

**To remove warning (optional):**
```powershell
# Mark connection as private
Set-NetConnectionProfile -InterfaceAlias "Ethernet" -NetworkCategory Private
```

### Slow transfer speeds

**Check negotiated speed:**
```powershell
# On Windows
Get-NetAdapter | Select-Object Name, LinkSpeed

# Should show: 1 Gbps
```

```bash
# On RDK
ethtool eth0 | grep Speed

# Should show: Speed: 1000Mb/s
```

**If showing 100 Mbps:**
- Try different cable (Cat5e or better)
- Check for cable damage
- Update network drivers

### Desktop needs internet while connected to RDK

**Solution 1: Add USB Ethernet Adapter** (Best!)

```powershell
# After connecting USB Ethernet adapter
# Configure built-in Ethernet for RDK (P2P)
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 10.0.0.1 -PrefixLength 24

# USB adapter gets internet via DHCP (automatic)
# Verify both connections
Get-NetAdapter | Select-Object Name, Status, LinkSpeed, InterfaceDescription
```

**Solution 2: Use Internet Connection Sharing (ICS)**

```powershell
# Share WiFi to Ethernet (RDK gets internet too!)
# Go to: Network Connections → WiFi → Properties → Sharing
# ✅ Allow other network users to connect
# Select: Ethernet

# This gives RDK internet access via your desktop
```

**Solution 3: Switch to USB connection for RDK**
- Keep Ethernet for internet
- Use USB cable for RDK
- Both work simultaneously

## USB Connection Issues

### USB device not recognized (Windows)

**Install RNDIS driver:**
```powershell
# Download driver from RDK documentation
# Or use Windows built-in RNDIS driver

# Check Device Manager
# Should see "RNDIS/Ethernet Gadget"
```

### Cannot ping RDK over USB

```powershell
# Check USB network adapter IP
ipconfig
# Look for "USB Ethernet" or "RNDIS"

# Should be: 192.168.7.1

# Verify RDK side
ssh root@192.168.7.2
ifconfig usb0
# Should show: 192.168.7.2
```

### USB connection drops frequently

```bash
# On RDK - make persistent
sudo nano /etc/rc.local

# Add before "exit 0":
modprobe g_ether
ifup usb0

exit 0
```

## Network Issues

### Desktop cannot reach RDK

```powershell
# Test connectivity
ping RDK_IP_ADDRESS

# Test port
Test-NetConnection -ComputerName RDK_IP -Port 8080
```

### RDK cannot reach desktop

```bash
# From RDK
ping DESKTOP_IP_ADDRESS

# Test desktop API
curl http://DESKTOP_IP:5000/api/health
```

---

# Quick Reference

## Desktop Commands

```powershell
# Start application
.\Start-WeldVision.bat

# P2P Ethernet connection
ping 10.0.0.2              # Test P2P Ethernet connectivity
ssh root@10.0.0.2          # SSH via P2P Ethernet

# USB connection
ping 192.168.7.2           # Test USB connectivity
ssh root@192.168.7.2       # SSH via USB

# Manual start
cd backend
python app.py          # Terminal 1

cd ..
npm run dev            # Terminal 2

# Access
http://localhost:3000
```

## RDK Commands

```bash
# SSH access (Ethernet via Router)
ssh root@RDK_IP

# SSH access (P2P Ethernet)
ssh root@10.0.0.2

# SSH access (USB)
ssh root@192.168.7.2

# Service management
sudo systemctl status weldvision-inference
sudo systemctl restart weldvision-inference
sudo systemctl stop weldvision-inference

# View logs
sudo journalctl -u weldvision-inference -f

# Check camera
ls /dev/video*
v4l2-ctl --list-devices
```

## File Transfers

```powershell
# Desktop → RDK
scp local_file root@RDK_IP:/remote/path/

# RDK → Desktop
scp root@RDK_IP:/remote/file D:\local\path\
```

---

# Success Checklist

## Desktop Setup ✅

- [ ] Python 3.9+ installed
- [ ] Node.js 16+ installed
- [ ] Dependencies installed (`pip install`, `npm install`)
- [ ] Backend starts without errors
- [ ] Frontend loads at http://localhost:3000
- [ ] Can navigate between pages
- [ ] Dataset Studio accessible
- [ ] Model Training panel visible
- [ ] Desktop has internet access (for downloading models, updates)

## RDK X5 Setup ✅

- [ ] RDK OS flashed and booting
- [ ] Network connectivity (can SSH)
- [ ] Static IP configured
- [ ] Python and dependencies installed
- [ ] Camera detected (`ls /dev/video*`)
- [ ] **RDK shows as connected in UI** 🟢
- [ ] Deployed model to RDK successfully
- [ ] Started inference on RDK
- [ ] Received inference results in UI

## Network Setup ✅

**Choose one connection method:**
- [ ] Router: Desktop and RDK can ping each other
- [ ] P2P Ethernet: Desktop (10.0.0.1) ↔ RDK (10.0.0.2)
- [ ] USB Direct: Desktop (192.168.7.1) ↔ RDK (192.168.7.2)
- [ ] Firewall rules configured (if using router)
- [ ] Can SSH to RDK
- [ ] Desktop still has internet (via WiFi or secondary adapter)
- [ ] Can transfer files via SCP

## End-to-End Test ✅

- [ ] Captured sample images
- [ ] Labeled sample images
- [ ] Trained small model (10 epochs)
- [ ] Compiled model to .bin
- [ ] Deployed model to RDK
- [ ] Started inference on RDK
- [ ] Received inference results

---

**Setup Complete!** 🎉

You now have:
- ✅ Desktop training workstation
- ✅ RDK X5 inference device
- ✅ Complete training → deployment pipeline
- ✅ Real-time defect detection system

**Next Steps:**
1. Capture production dataset (100+ images per class)
2. Train production model (100+ epochs)
3. Deploy and test on real welds
4. Iterate and improve

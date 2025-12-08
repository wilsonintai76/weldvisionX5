# WeldVision X5 - Remote Access Setup Guide

## Overview
Access the WeldVision X5 application from any computer on your network (WiFi or LAN) using the IP address.

## Network Configuration

### Your Machine Details
- **Hostname:** PKS-JKM-WILSON
- **Network IP (IPv4):** `10.80.16.151`
- **Frontend Port:** 3000
- **Backend Port:** 5000

### Current Server Status
Both servers are configured to listen on all network interfaces (`0.0.0.0`):

**Frontend (Vite):**
```
vite.config.ts → server.host = '0.0.0.0'
Accessible at: http://10.80.16.151:3000
```

**Backend (Flask):**
```
app.py → app.run(host='0.0.0.0', port=5000)
Accessible at: http://10.80.16.151:5000
```

---

## Access Methods

### Method 1: Same Machine (Local)
```
Frontend:  http://localhost:3000
Backend:   http://localhost:5000
```

### Method 2: Different Computer on Network
From another computer connected to the same network:

```
Frontend:  http://10.80.16.151:3000
Backend:   http://10.80.16.151:5000
```

### Method 3: IP-Based Configuration (Recommended)
The app is pre-configured to use the IP-based approach in `.env.local`:

```env
VITE_API_URL=http://10.80.16.151:5000
```

This allows the frontend to find the backend regardless of where you access it from.

---

## Step-by-Step Setup

### 1. Verify Network Connectivity
On the remote computer, verify you can reach the server:

**Windows:**
```cmd
ping 10.80.16.151
```

**Mac/Linux:**
```bash
ping 10.80.16.151
```

Expected output: Successful ping responses

### 2. Start the Servers (on Server Machine)

#### Terminal 1 - Frontend:
```bash
cd "d:\WeldMaster AI Evaluation"
npm run dev
```
Output should show:
```
Local:   http://localhost:3000/
Network: http://10.80.16.151:3000/
```

#### Terminal 2 - Backend:
```bash
cd "d:\WeldMaster AI Evaluation\backend"
python app.py
```
Output should show:
```
Running on http://0.0.0.0:5000
Running on http://127.0.0.1:5000
Running on http://10.80.16.151:5000
```

### 3. Access from Remote Computer
Open a browser on the remote computer and navigate to:
```
http://10.80.16.151:3000
```

The app will:
- Load the frontend from port 3000
- Automatically connect to the backend on port 5000
- Use the `.env.local` configuration to find the backend API

---

## Troubleshooting

### Issue: Cannot Reach Server
**Check:**
1. Both servers are running
2. Network connectivity: `ping 10.80.16.151`
3. Firewall isn't blocking ports 3000 and 5000
4. IP address is correct (verify with `ipconfig` on Windows)

### Issue: Frontend Loads but Backend Doesn't Connect
**Check:**
1. Backend is running on port 5000
2. `.env.local` has correct IP: `VITE_API_URL=http://10.80.16.151:5000`
3. CORS is enabled (Flask app has `CORS(app)` configured)
4. Open browser Developer Tools (F12) → Network tab to see API requests

### Issue: API Requests Failing
**Solutions:**
1. Verify backend health:
   ```
   http://10.80.16.151:5000/api/health
   ```
   Should return: `{"status": "healthy"}`

2. Check system diagnostics:
   ```
   http://10.80.16.151:5000/api/system/diagnostics
   ```

---

## Environment Variables

### To Switch Between Local and Network Access

**For Local Access (same machine):**
Edit `.env.local`:
```env
VITE_API_URL=http://localhost:5000
```

**For Network Access:**
Edit `.env.local`:
```env
VITE_API_URL=http://10.80.16.151:5000
```

Then restart the frontend:
```bash
npm run dev
```

---

## Network Topology

```
┌─────────────────────────────────────────────────────┐
│                   Your Network                      │
│                  (WiFi or LAN)                      │
└─────────────────────────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         │                                 │
    ┌────▼─────────────────┐      ┌──────▼──────────┐
    │  Server Machine      │      │ Remote Computer  │
    │ 10.80.16.151        │      │ (Any device)     │
    │                      │      │                  │
    │ Frontend: :3000      │      │ Browser:         │
    │ Backend:  :5000      │◄─────┤ 10.80.16.151:3000│
    │                      │      │                  │
    └──────────────────────┘      └──────────────────┘
```

---

## Security Considerations

**⚠️ Development Only**

The current setup is for **development/testing only**. For production:

1. **Use a reverse proxy** (Nginx, Apache)
2. **Enable HTTPS/SSL** certificates
3. **Add authentication** (JWT tokens, username/password)
4. **Implement rate limiting** to prevent abuse
5. **Use environment-specific configs** (dev, staging, production)
6. **Firewall rules** to restrict access to trusted IPs only

---

## API Health Checks

### Check Backend Status
```bash
curl http://10.80.16.151:5000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-08T08:45:07.423000"
}
```

### Check System Diagnostics
```bash
curl http://10.80.16.151:5000/api/system/diagnostics
```

---

## Quick Reference

| Access Type | Frontend URL | Backend URL | Use Case |
|-------------|-------------|-------------|----------|
| **Local** | `http://localhost:3000` | `http://localhost:5000` | Testing on server machine |
| **Network** | `http://10.80.16.151:3000` | `http://10.80.16.151:5000` | Remote access from LAN/WiFi |
| **Docker** | Requires port mapping | Requires port mapping | Containerized deployment |

---

## Default Ports

- **Frontend (Vite):** 3000
- **Backend (Flask):** 5000
- **Database:** SQLite (local file: `weld_data.db`)

To change ports, edit:
- Frontend: `vite.config.ts` → `server.port`
- Backend: `backend/app.py` → `app.run(port=XXXX)`

Then update `.env.local` accordingly.

---

## Next Steps

1. ✅ Servers are already configured for network access
2. ✅ `.env.local` is set up with your network IP
3. 🔄 Start the servers (see Step-by-Step Setup)
4. 🔄 Access from remote computer using `http://10.80.16.151:3000`

For questions or issues, check the troubleshooting section or review server logs for detailed error messages.

# Network Connectivity Setup - WeldVision X5

## System Information

**Server Machine:**
- Hostname: `PKS-JKM-WILSON`
- Network IP: `10.80.16.151`
- OS: Windows 11
- Python: 3.14.0

**Server Ports:**
- Frontend: `3000` (Vite dev server)
- Backend: `5000` (Flask API)

---

## Pre-Configured for Remote Access

✅ **Frontend (Vite)** - Already listening on all network interfaces
```typescript
// vite.config.ts
server: {
  port: 3000,
  host: '0.0.0.0'  // Listens on all interfaces
}
```

✅ **Backend (Flask)** - Already listening on all network interfaces
```python
# backend/app.py
app.run(host='0.0.0.0', port=5000)  # Listens on all interfaces
```

✅ **Frontend API Configuration** - `.env.local` set up for network access
```env
VITE_API_URL=http://10.80.16.151:5000
```

---

## How to Access the Application

### From the Server Machine (Local)
```
http://localhost:3000
```

### From Any Computer on the Network
```
http://10.80.16.151:3000
```

**Or using hostname:**
```
http://PKS-JKM-WILSON.local:3000
```

---

## Starting the Application

### Terminal 1 - Frontend Server
```bash
cd "d:\WeldMaster AI Evaluation"
npm run dev
```

Expected output:
```
VITE v6.4.1  ready in 599 ms
  Local:   http://localhost:3000/
  Network: http://10.80.16.151:3000/
```

### Terminal 2 - Backend Server
```bash
cd "d:\WeldMaster AI Evaluation\backend"
python app.py
```

Expected output:
```
Running on all addresses (0.0.0.0)
Running on http://127.0.0.1:5000
Running on http://10.80.16.151:5000
```

---

## Testing Remote Connection

### From the Remote Computer:

**1. Test Network Connectivity:**
```bash
# Windows
ping 10.80.16.151

# Mac/Linux
ping 10.80.16.151
```

Expected: Successful ping responses

**2. Test Backend API:**
```bash
# Windows
curl http://10.80.16.151:5000/api/health

# Mac/Linux
curl http://10.80.16.151:5000/api/health
```

Expected response:
```json
{"status": "healthy", "timestamp": "..."}
```

**3. Open Browser:**
```
http://10.80.16.151:3000
```

---

## Network Architecture

```
┌──────────────────────────────────────────────────────┐
│            WiFi / LAN Network (192.168.x.x)          │
└──────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        │          ┌──────▼──────┐          │
        │          │  Server PC  │          │
        │          │ 10.80.16.151│          │
        │          │             │          │
        │    ┌─────▼──┐      ┌──▼────┐     │
        │    │ Vite   │      │ Flask │     │
        │    │ :3000  │      │ :5000 │     │
        │    └─────┬──┘      └──┬────┘     │
        │          │             │         │
        └──────────┼─────────────┼─────────┘
                   │             │
        ┌──────────▼──────┬──────▼──────────┐
        │                 │                 │
    ┌───▼────┐       ┌────▼─────┐    ┌────▼─────┐
    │Computer│       │Computer  │    │  Mobile  │
    │  #1    │       │   #2     │    │ Device   │
    │ Browser│       │ Browser  │    │ Browser  │
    └────────┘       └──────────┘    └──────────┘

All access: http://10.80.16.151:3000
```

---

## Environment Configuration

### Current Setup in `.env.local`
```env
GEMINI_API_KEY=PLACEHOLDER_API_KEY
VITE_API_URL=http://10.80.16.151:5000
```

**This tells the frontend where to find the backend API when you access the app from any network location.**

### To Switch Between Access Methods

**For Local Development (same machine):**
```env
VITE_API_URL=http://localhost:5000
```

**For Network Access (from other computers):**
```env
VITE_API_URL=http://10.80.16.151:5000
```

Then restart Vite:
```bash
npm run dev
```

---

## API Endpoints Available

| Endpoint | URL | Purpose |
|----------|-----|---------|
| Health Check | `http://10.80.16.151:5000/api/health` | Test backend connectivity |
| Diagnostics | `http://10.80.16.151:5000/api/system/diagnostics` | System status |
| LED Status | `http://10.80.16.151:5000/api/led/status` | LED control status |
| Students | `http://10.80.16.151:5000/api/students` | Student database |
| Scans | `http://10.80.16.151:5000/api/scans` | Scan history |
| Calibration | `http://10.80.16.151:5000/api/calibration` | Camera calibration |

---

## Troubleshooting Remote Access

### Problem: Cannot Connect to Server

**Check 1: Network Connectivity**
```bash
ping 10.80.16.151
```
- If fails: Check if both computers are on same network
- If fails: Verify network cables/WiFi connection

**Check 2: Servers Running**
- Verify frontend is running: `npm run dev` output visible
- Verify backend is running: `python app.py` output visible

**Check 3: Firewall**
- Windows Firewall may block ports 3000 and 5000
- Allow both ports in Windows Firewall settings:
  1. Settings → Privacy & Security → Windows Defender Firewall
  2. Allow app through firewall
  3. Check both "Private" and "Public" for Node.js and Python

**Check 4: IP Address**
- Verify IP is correct: Run `ipconfig` on server
- Look for IPv4 Address under network adapter
- Update `.env.local` if IP changed

### Problem: Frontend Loads but API Not Working

**Solution 1: Verify Backend**
```bash
curl http://10.80.16.151:5000/api/health
```
- Should return JSON with status

**Solution 2: Check CORS**
- Backend has CORS enabled: `CORS(app)`
- Frontend requests should include credentials

**Solution 3: Check Console**
- Open browser Developer Tools (F12)
- Go to Console tab
- Look for error messages
- Check Network tab for failed API calls

### Problem: Getting "localhost Not Reachable"

**Cause:** Browser trying to use localhost instead of network IP

**Solution 1:** Clear browser cache
```
Ctrl + Shift + Delete → Clear all
```

**Solution 2:** Verify `.env.local`
```env
VITE_API_URL=http://10.80.16.151:5000
```

**Solution 3:** Restart Vite
```bash
npm run dev
```

---

## Monitoring Connections

### Check Active Connections on Server

**Windows - Port 3000 (Frontend):**
```bash
netstat -ano | findstr :3000
```

**Windows - Port 5000 (Backend):**
```bash
netstat -ano | findstr :5000
```

Expected output shows process listening on the port.

---

## Performance Considerations

**Local Network (LAN/WiFi):** Expected < 50ms latency
- Fast camera streaming
- Real-time data sync

**Same Machine:** Expected < 5ms latency
- Best for development

**Remote Access (WiFi):** May see 50-200ms latency
- Acceptable for UI operations
- May affect video streaming

---

## Security Notes (Development Only)

⚠️ **Current setup is NOT PRODUCTION READY**

For production deployment:
1. Enable HTTPS/SSL encryption
2. Add authentication (JWT tokens)
3. Use reverse proxy (Nginx)
4. Implement rate limiting
5. Restrict to specific IP ranges
6. Use environment-specific configs

---

## Quick Commands Reference

| Task | Command |
|------|---------|
| Get server IP | `ipconfig` |
| Test connectivity | `ping 10.80.16.151` |
| Test backend | `curl http://10.80.16.151:5000/api/health` |
| Start frontend | `npm run dev` |
| Start backend | `python backend/app.py` |
| Stop servers | `Ctrl+C` in each terminal |

---

## Files Modified for Remote Access

1. **`vite.config.ts`** - Already configured with `host: '0.0.0.0'`
2. **`backend/app.py`** - Already configured with `host='0.0.0.0'`
3. **`.env.local`** - Updated with `VITE_API_URL=http://10.80.16.151:5000`
4. **`services/apiService.ts`** - Uses `VITE_API_URL` environment variable

---

## Next Steps

1. ✅ Servers configured for network access
2. ✅ `.env.local` set up with correct IP
3. Start both servers (Frontend & Backend)
4. Access from remote computer at `http://10.80.16.151:3000`
5. Monitor server logs for any errors

---

## Support

For detailed guides, see:
- `REMOTE_ACCESS_GUIDE.md` - Comprehensive setup guide
- `REMOTE_ACCESS_QUICK_START.md` - Quick reference
- Server logs for debugging


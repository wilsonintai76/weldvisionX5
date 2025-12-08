# 🌐 WeldVision X5 - Remote Access Quick Start

## Your Server IP Address
```
10.80.16.151
```

## Access the App from Remote Computer

### Option 1: Direct IP Access (Recommended)
Open browser and go to:
```
http://10.80.16.151:3000
```

### Option 2: Using Hostname
```
http://PKS-JKM-WILSON.local:3000
```

---

## Make Sure Server is Running

### Terminal 1 - Start Frontend:
```bash
cd "d:\WeldMaster AI Evaluation"
npm run dev
```

### Terminal 2 - Start Backend:
```bash
cd "d:\WeldMaster AI Evaluation\backend"
python app.py
```

Look for these messages:
- Frontend: `Running on http://10.80.16.151:3000`
- Backend: `Running on http://10.80.16.151:5000`

---

## Test Connection from Remote Computer

### Windows Command Prompt:
```cmd
ping 10.80.16.151
```

### Then Visit:
```
http://10.80.16.151:3000
```

---

## Frontend Configuration
The `.env.local` file is already set up for remote access:

```env
VITE_API_URL=http://10.80.16.151:5000
```

This tells the frontend where to find the backend API.

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Cannot reach server" | Check if both servers are running |
| "Connection refused" | Verify IP with `ipconfig` on Windows |
| "Firewall blocked" | Allow ports 3000 and 5000 in Windows Firewall |
| "API not responding" | Restart backend: `python app.py` |
| "Page won't load" | Clear browser cache (Ctrl+F5) |

---

## API Endpoints (for testing)

**Health Check:**
```
http://10.80.16.151:5000/api/health
```

**System Status:**
```
http://10.80.16.151:5000/api/system/diagnostics
```

**LED Control:**
```
http://10.80.16.151:5000/api/led/status
```

---

## Need Help?
1. Check `REMOTE_ACCESS_GUIDE.md` for detailed setup
2. Review server logs for error messages
3. Verify network connectivity with `ping`
4. Check firewall settings allowing ports 3000 & 5000

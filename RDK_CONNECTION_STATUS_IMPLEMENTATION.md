# RDK Connection Status - Implementation Guide

## Current Status: NOT YET IMPLEMENTED ❌

The connection status indicator described in the COMPLETE_SETUP_GUIDE.md is a **recommended feature** but is not currently implemented in the codebase.

---

## Where It Should Be Added

### Frontend Location: `components/OrchestrationPanel.tsx`

The RDK connection indicator should appear in the **Deployment** section, which is currently at **line 256**.

**Current code (line 256-288):**
```tsx
<div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
  <h4 className="text-white font-medium mb-2">Deployment</h4>
  <div className="grid gap-2">
    <label>
      <span>Device Host</span>
      <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" 
             value={deviceHost} 
             onChange={(e) => setDeviceHost(e.target.value)} />
    </label>
    {/* ... more fields ... */}
  </div>
</div>
```

**Recommended addition:**
```tsx
<div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
  <div className="flex items-center justify-between mb-2">
    <h4 className="text-white font-medium">Deployment</h4>
    {/* ADD CONNECTION STATUS HERE */}
    <RDKConnectionBadge host={deviceHost} />
  </div>
  {/* ... rest of deployment section ... */}
</div>
```

---

## Implementation Steps

### Step 1: Create API Endpoint (Backend)

**File:** `backend/app.py`

**Add new endpoint:**
```python
@app.route('/api/rdk/test-connection', methods=['POST'])
def test_rdk_connection():
    """Test if RDK device is reachable"""
    data = request.json
    host = data.get('host', '')
    user = data.get('user', 'root')
    
    if not host:
        return jsonify({'error': 'Host is required'}), 400
    
    # Test 1: Ping
    ping_result = os.system(f'ping -n 1 -w 1000 {host} >nul 2>&1')
    reachable = (ping_result == 0)
    
    # Test 2: SSH (if ping succeeds)
    ssh_available = False
    if reachable:
        try:
            ssh_test = subprocess.run(
                ['ssh', '-o', 'ConnectTimeout=3', '-o', 'StrictHostKeyChecking=no', 
                 f'{user}@{host}', 'echo OK'],
                capture_output=True,
                timeout=5
            )
            ssh_available = (ssh_test.returncode == 0)
        except:
            ssh_available = False
    
    # Test 3: Service health check
    service_healthy = False
    if ssh_available:
        try:
            import requests
            response = requests.get(f'http://{host}:8080/health', timeout=3)
            service_healthy = (response.status_code == 200)
        except:
            service_healthy = False
    
    return jsonify({
        'reachable': reachable,
        'sshAvailable': ssh_available,
        'serviceHealthy': service_healthy,
        'connected': reachable and ssh_available,
        'message': 'RDK is ready' if (reachable and ssh_available) else 'Cannot connect to RDK'
    })


@app.route('/api/rdk/status', methods=['GET'])
def get_rdk_status():
    """Get current RDK connection status (from cache/state)"""
    # This would use cached status from periodic checks
    # For now, return a simple status
    return jsonify({
        'connected': False,
        'lastCheck': None,
        'message': 'Status check not implemented'
    })
```

### Step 2: Create API Wrapper (Frontend)

**File:** `api/rdk.ts` (NEW FILE)

```typescript
export async function testRDKConnection(host: string, user: string = 'root') {
  const response = await fetch('/api/rdk/test-connection', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ host, user })
  });
  
  if (!response.ok) {
    throw new Error('Connection test failed');
  }
  
  return await response.json();
}

export async function getRDKStatus() {
  const response = await fetch('/api/rdk/status');
  if (!response.ok) {
    throw new Error('Failed to get RDK status');
  }
  return await response.json();
}
```

### Step 3: Create Connection Badge Component

**File:** `components/RDKConnectionBadge.tsx` (NEW FILE)

```typescript
import React, { useEffect, useState } from 'react';
import { testRDKConnection } from '../api/rdk';

interface Props {
  host: string;
  user?: string;
  autoRefresh?: boolean;
}

export default function RDKConnectionBadge({ host, user = 'root', autoRefresh = true }: Props) {
  const [status, setStatus] = useState<'checking' | 'connected' | 'disconnected'>('checking');
  const [lastCheck, setLastCheck] = useState<Date | null>(null);
  const [testing, setTesting] = useState(false);

  const checkConnection = async () => {
    if (!host || testing) return;
    
    setTesting(true);
    setStatus('checking');
    
    try {
      const result = await testRDKConnection(host, user);
      setStatus(result.connected ? 'connected' : 'disconnected');
      setLastCheck(new Date());
    } catch (error) {
      setStatus('disconnected');
      setLastCheck(new Date());
    } finally {
      setTesting(false);
    }
  };

  // Auto-check on mount and when host changes
  useEffect(() => {
    checkConnection();
  }, [host, user]);

  // Auto-refresh every 30 seconds
  useEffect(() => {
    if (!autoRefresh) return;
    
    const interval = setInterval(() => {
      checkConnection();
    }, 30000); // 30 seconds
    
    return () => clearInterval(interval);
  }, [host, user, autoRefresh]);

  const getStatusConfig = () => {
    switch (status) {
      case 'connected':
        return {
          icon: '🟢',
          text: 'Connected',
          className: 'bg-green-900/30 text-green-400 border-green-700',
          title: `Connected to ${host}`
        };
      case 'disconnected':
        return {
          icon: '🔴',
          text: 'Disconnected',
          className: 'bg-red-900/30 text-red-400 border-red-700',
          title: `Cannot reach ${host}`
        };
      case 'checking':
        return {
          icon: '🟡',
          text: 'Checking...',
          className: 'bg-yellow-900/30 text-yellow-400 border-yellow-700',
          title: 'Testing connection...'
        };
    }
  };

  const config = getStatusConfig();

  return (
    <div className="flex items-center gap-2">
      <div 
        className={`flex items-center gap-1.5 px-2 py-1 rounded border text-xs ${config.className}`}
        title={config.title}
      >
        <span>{config.icon}</span>
        <span className="font-medium">{config.text}</span>
      </div>
      
      <button
        onClick={checkConnection}
        disabled={testing}
        className="text-xs px-2 py-1 rounded bg-slate-700 text-slate-300 hover:bg-slate-600 disabled:opacity-50"
        title="Test connection now"
      >
        Test
      </button>
      
      {lastCheck && (
        <span className="text-xs text-slate-500" title={lastCheck.toLocaleString()}>
          {Math.round((Date.now() - lastCheck.getTime()) / 1000)}s ago
        </span>
      )}
    </div>
  );
}
```

### Step 4: Update OrchestrationPanel

**File:** `components/OrchestrationPanel.tsx`

**Add import at top:**
```typescript
import RDKConnectionBadge from './RDKConnectionBadge';
```

**Replace line 256-258:**
```typescript
// BEFORE:
<div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
  <h4 className="text-white font-medium mb-2">Deployment</h4>

// AFTER:
<div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
  <div className="flex items-center justify-between mb-2">
    <h4 className="text-white font-medium">Deployment</h4>
    <RDKConnectionBadge host={deviceHost} user={deviceUser} />
  </div>
```

---

## Visual Preview

**What it will look like:**

```
┌─────────────────────────────────────────────────────┐
│ Deployment                    🟢 Connected    [Test]│
│                               2s ago                │
├─────────────────────────────────────────────────────┤
│ Device Host                                         │
│ [192.168.1.100                                    ] │
│                                                     │
│ Device User                                         │
│ [root                                             ] │
│                                                     │
│ [Deploy Model]                                      │
└─────────────────────────────────────────────────────┘
```

**Status variations:**
- 🟢 **Connected** - RDK is reachable and ready
- 🔴 **Disconnected** - Cannot reach RDK
- 🟡 **Checking...** - Currently testing connection

---

## Current Workaround (Manual Testing)

**Until this feature is implemented, users can manually test RDK connectivity:**

### From Windows PowerShell:

```powershell
# Test if RDK is reachable
ping 192.168.1.100

# Test SSH access
ssh root@192.168.1.100 "echo OK"

# Test inference service
curl http://192.168.1.100:8080/health
```

### From the Application:

Currently, the only way to know if RDK is reachable is to **try deploying**:
- Click "Deploy Model" button
- If it fails: RDK is not reachable
- If it succeeds: RDK is connected

---

## Implementation Priority

**Recommended priority:** **MEDIUM**

**Pros of implementing:**
- ✅ Better user experience
- ✅ Prevents deployment errors
- ✅ Real-time feedback
- ✅ Saves time troubleshooting

**Cons of not implementing:**
- ⚠️ Users won't know if RDK is connected until they try to deploy
- ⚠️ Deployment failures are harder to diagnose
- ⚠️ More manual testing required

**Effort:** ~2-4 hours to implement all steps

---

## Alternative: Simple Implementation

If the full implementation is too complex, add a **simple manual test button:**

**Quick implementation in OrchestrationPanel.tsx:**

```typescript
// Add state
const [rdkStatus, setRdkStatus] = useState<string>('unknown');

// Add test function
const testRDKConnection = async () => {
  setRdkStatus('testing');
  try {
    const response = await fetch('/api/rdk/test-connection', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ host: deviceHost, user: deviceUser })
    });
    const result = await response.json();
    setRdkStatus(result.connected ? 'connected' : 'disconnected');
    setMessage(result.message);
  } catch (error) {
    setRdkStatus('disconnected');
    setError('Cannot connect to RDK');
  }
};

// Add button in Deployment section
<button 
  className="mt-2 px-3 py-1.5 rounded bg-slate-700 text-white"
  onClick={testRDKConnection}
>
  {rdkStatus === 'testing' ? '🟡 Testing...' : 
   rdkStatus === 'connected' ? '🟢 Connected' :
   rdkStatus === 'disconnected' ? '🔴 Disconnected' : 
   'Test RDK Connection'}
</button>
```

This gives users a manual way to check connectivity without auto-refresh.

---

## Summary

**Current state:** The RDK connection indicator is described in documentation but **not implemented** in the code.

**Where to add it:** `components/OrchestrationPanel.tsx` in the Deployment section.

**What's needed:**
1. Backend API endpoint for connection testing
2. Frontend API wrapper
3. React component for status badge
4. Integration into OrchestrationPanel

**For now:** Users can manually test with `ping` and `ssh` commands.

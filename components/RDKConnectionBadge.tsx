import React, { useEffect, useState } from 'react';
import { testRDKConnection } from '../api/rdk';

interface Props {
  host: string;
  user?: string;
  autoRefresh?: boolean;
  refreshInterval?: number;
}

export default function RDKConnectionBadge({ 
  host, 
  user = 'root', 
  autoRefresh = true,
  refreshInterval = 30000 // 30 seconds
}: Props) {
  const [status, setStatus] = useState<'checking' | 'connected' | 'disconnected' | 'error'>('checking');
  const [lastCheck, setLastCheck] = useState<Date | null>(null);
  const [testing, setTesting] = useState(false);
  const [message, setMessage] = useState<string>('');
  const [detailedStatus, setDetailedStatus] = useState<{
    reachable: boolean;
    sshAvailable: boolean;
    serviceHealthy: boolean;
  } | null>(null);

  const checkConnection = async () => {
    if (!host || testing) return;
    
    setTesting(true);
    setStatus('checking');
    
    try {
      const result = await testRDKConnection(host, user);
      
      setStatus(result.connected ? 'connected' : 'disconnected');
      setMessage(result.message);
      setDetailedStatus({
        reachable: result.reachable,
        sshAvailable: result.sshAvailable,
        serviceHealthy: result.serviceHealthy
      });
      setLastCheck(new Date());
    } catch (error: any) {
      setStatus('error');
      setMessage(error?.message || 'Connection test failed');
      setDetailedStatus(null);
      setLastCheck(new Date());
    } finally {
      setTesting(false);
    }
  };

  // Auto-check on mount and when host/user changes
  useEffect(() => {
    if (host) {
      checkConnection();
    }
  }, [host, user]);

  // Auto-refresh at specified interval
  useEffect(() => {
    if (!autoRefresh || !host) return;
    
    const interval = setInterval(() => {
      checkConnection();
    }, refreshInterval);
    
    return () => clearInterval(interval);
  }, [host, user, autoRefresh, refreshInterval]);

  const getStatusConfig = () => {
    switch (status) {
      case 'connected':
        return {
          icon: '🟢',
          text: 'Connected',
          bgColor: 'bg-green-900/30',
          textColor: 'text-green-400',
          borderColor: 'border-green-700',
          title: `Connected to ${host}\n${message}`
        };
      case 'disconnected':
        return {
          icon: '🔴',
          text: 'Disconnected',
          bgColor: 'bg-red-900/30',
          textColor: 'text-red-400',
          borderColor: 'border-red-700',
          title: message || `Cannot reach ${host}`
        };
      case 'error':
        return {
          icon: '⚠️',
          text: 'Error',
          bgColor: 'bg-yellow-900/30',
          textColor: 'text-yellow-400',
          borderColor: 'border-yellow-700',
          title: message || 'Connection test error'
        };
      case 'checking':
        return {
          icon: '🟡',
          text: 'Checking...',
          bgColor: 'bg-blue-900/30',
          textColor: 'text-blue-400',
          borderColor: 'border-blue-700',
          title: 'Testing connection...'
        };
    }
  };

  const config = getStatusConfig();

  // Calculate time since last check
  const getTimeSinceLastCheck = () => {
    if (!lastCheck) return null;
    const seconds = Math.round((Date.now() - lastCheck.getTime()) / 1000);
    if (seconds < 60) return `${seconds}s ago`;
    const minutes = Math.round(seconds / 60);
    if (minutes < 60) return `${minutes}m ago`;
    const hours = Math.round(minutes / 60);
    return `${hours}h ago`;
  };

  return (
    <div className="flex items-center gap-2">
      {/* Status Badge */}
      <div 
        className={`flex items-center gap-1.5 px-2 py-1 rounded border text-xs font-medium ${config.bgColor} ${config.textColor} ${config.borderColor}`}
        title={config.title}
      >
        <span>{config.icon}</span>
        <span>{config.text}</span>
      </div>
      
      {/* Test Button */}
      <button
        onClick={checkConnection}
        disabled={testing}
        className="text-xs px-2 py-1 rounded bg-slate-700 text-slate-300 hover:bg-slate-600 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
        title="Test connection now"
      >
        {testing ? 'Testing...' : 'Test'}
      </button>
      
      {/* Time Since Last Check */}
      {lastCheck && (
        <span className="text-xs text-slate-500" title={lastCheck.toLocaleString()}>
          {getTimeSinceLastCheck()}
        </span>
      )}
      
      {/* Detailed Status Tooltip (hover) */}
      {detailedStatus && status === 'connected' && (
        <div className="relative group">
          <button className="text-xs text-slate-500 hover:text-slate-300">ℹ️</button>
          <div className="absolute left-0 top-full mt-1 hidden group-hover:block z-10 bg-slate-800 border border-slate-700 rounded p-2 text-xs whitespace-nowrap shadow-lg">
            <div className="flex items-center gap-2 mb-1">
              <span className={detailedStatus.reachable ? 'text-green-400' : 'text-red-400'}>●</span>
              <span className="text-slate-300">Network: {detailedStatus.reachable ? 'Reachable' : 'Unreachable'}</span>
            </div>
            <div className="flex items-center gap-2 mb-1">
              <span className={detailedStatus.sshAvailable ? 'text-green-400' : 'text-red-400'}>●</span>
              <span className="text-slate-300">SSH: {detailedStatus.sshAvailable ? 'Available' : 'Not available'}</span>
            </div>
            <div className="flex items-center gap-2">
              <span className={detailedStatus.serviceHealthy ? 'text-green-400' : 'text-yellow-400'}>●</span>
              <span className="text-slate-300">Service: {detailedStatus.serviceHealthy ? 'Healthy' : 'Not running'}</span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

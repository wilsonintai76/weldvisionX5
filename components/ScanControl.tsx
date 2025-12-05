/**
 * WeldVision X5 Scan Control Component
 * 
 * Frontend React component for managing automated scanning operations
 */

import React, { useState, useEffect, useCallback } from 'react';
import './ScanControl.css';

interface ScanConfig {
  student_id?: number;
  workpiece_type: string;
  workpiece_material: string;
  operator_notes: string;
  grid_x_min: number;
  grid_x_max: number;
  grid_y_min: number;
  grid_y_max: number;
  configuration_id?: number;
}

interface ScanStatus {
  scan_id: string;
  status: string;
  completed_points: number;
  total_points: number;
  quality_score?: number;
}

interface HardwareStatus {
  printer: {
    connected: boolean;
    state: string;
  };
  camera: {
    ready: boolean;
    stats: any;
  };
  orchestrator: {
    initialized: boolean;
    active_scans: number;
  };
}

const ScanControl: React.FC = () => {
  // State
  const [scanId, setScanId] = useState<string | null>(null);
  const [isScanning, setIsScanning] = useState(false);
  const [isPaused, setIsPaused] = useState(false);
  const [scanStatus, setScanStatus] = useState<ScanStatus | null>(null);
  const [hardwareStatus, setHardwareStatus] = useState<HardwareStatus | null>(null);
  
  // Form state
  const [config, setConfig] = useState<ScanConfig>({
    workpiece_type: 'Weld Joint',
    workpiece_material: 'Steel',
    operator_notes: '',
    grid_x_min: 0,
    grid_x_max: 100,
    grid_y_min: 0,
    grid_y_max: 100,
  });

  // Fetch hardware status
  useEffect(() => {
    const fetchHardwareStatus = async () => {
      try {
        const response = await fetch('/api/scan/hardware-status');
        if (response.ok) {
          const data = await response.json();
          setHardwareStatus(data);
        }
      } catch (error) {
        console.error('Failed to fetch hardware status:', error);
      }
    };

    fetchHardwareStatus();
    const interval = setInterval(fetchHardwareStatus, 5000);
    return () => clearInterval(interval);
  }, []);

  // Poll scan status
  useEffect(() => {
    if (!scanId || (!isScanning && !isPaused)) {
      return;
    }

    const pollStatus = async () => {
      try {
        const response = await fetch(`/api/scan/status/${scanId}`);
        if (response.ok) {
          const data = await response.json();
          setScanStatus(data);
          
          // Update scanning status
          if (data.status === 'completed' || data.status === 'failed') {
            setIsScanning(false);
            setIsPaused(false);
          }
        }
      } catch (error) {
        console.error('Failed to fetch scan status:', error);
      }
    };

    const interval = setInterval(pollStatus, 1000);
    return () => clearInterval(interval);
  }, [scanId, isScanning, isPaused]);

  // Handle start scan
  const handleStartScan = useCallback(async () => {
    try {
      const response = await fetch('/api/scan/start', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(config),
      });

      if (response.ok) {
        const data = await response.json();
        setScanId(data.scan_id);
        setIsScanning(true);
        setIsPaused(false);
      } else {
        alert('Failed to start scan');
      }
    } catch (error) {
      console.error('Failed to start scan:', error);
      alert('Error starting scan');
    }
  }, [config]);

  // Handle pause scan
  const handlePauseScan = useCallback(async () => {
    if (!scanId) return;

    try {
      const response = await fetch(`/api/scan/pause/${scanId}`, {
        method: 'POST',
      });

      if (response.ok) {
        setIsPaused(true);
        setIsScanning(false);
      }
    } catch (error) {
      console.error('Failed to pause scan:', error);
    }
  }, [scanId]);

  // Handle resume scan
  const handleResumeScan = useCallback(async () => {
    if (!scanId) return;

    try {
      const response = await fetch(`/api/scan/resume/${scanId}`, {
        method: 'POST',
      });

      if (response.ok) {
        setIsScanning(true);
        setIsPaused(false);
      }
    } catch (error) {
      console.error('Failed to resume scan:', error);
    }
  }, [scanId]);

  // Handle stop scan
  const handleStopScan = useCallback(async () => {
    if (!scanId) return;

    try {
      const response = await fetch(`/api/scan/stop/${scanId}`, {
        method: 'POST',
      });

      if (response.ok) {
        setIsScanning(false);
        setIsPaused(false);
        setScanStatus(null);
      }
    } catch (error) {
      console.error('Failed to stop scan:', error);
    }
  }, [scanId]);

  // Calculate progress percentage
  const progressPercent = scanStatus
    ? (scanStatus.completed_points / Math.max(scanStatus.total_points, 1)) * 100
    : 0;

  return (
    <div className="scan-control">
      <h2>WeldVision X5 Scan Control</h2>

      {/* Hardware Status */}
      {hardwareStatus && (
        <div className="hardware-status">
          <h3>Hardware Status</h3>
          <div className="status-grid">
            <div className={`status-item ${hardwareStatus.printer.connected ? 'connected' : 'disconnected'}`}>
              <span className="status-label">Printer:</span>
              <span className="status-value">{hardwareStatus.printer.state}</span>
            </div>
            <div className={`status-item ${hardwareStatus.camera.ready ? 'ready' : 'not-ready'}`}>
              <span className="status-label">Camera:</span>
              <span className="status-value">{hardwareStatus.camera.ready ? 'Ready' : 'Not Ready'}</span>
            </div>
            <div className={`status-item ${hardwareStatus.orchestrator.initialized ? 'ready' : 'not-ready'}`}>
              <span className="status-label">Orchestrator:</span>
              <span className="status-value">
                {hardwareStatus.orchestrator.initialized ? 'Ready' : 'Not Ready'}
              </span>
            </div>
          </div>
        </div>
      )}

      {/* Scan Configuration */}
      {!isScanning && !isPaused && (
        <div className="scan-config">
          <h3>Scan Configuration</h3>
          
          <div className="form-group">
            <label>Workpiece Type:</label>
            <input
              type="text"
              value={config.workpiece_type}
              onChange={(e) => setConfig({ ...config, workpiece_type: e.target.value })}
              placeholder="e.g., Weld Joint"
            />
          </div>

          <div className="form-group">
            <label>Material:</label>
            <input
              type="text"
              value={config.workpiece_material}
              onChange={(e) => setConfig({ ...config, workpiece_material: e.target.value })}
              placeholder="e.g., Steel"
            />
          </div>

          <div className="form-group">
            <label>Operator Notes:</label>
            <textarea
              value={config.operator_notes}
              onChange={(e) => setConfig({ ...config, operator_notes: e.target.value })}
              placeholder="Any additional notes"
              rows={3}
            />
          </div>

          <div className="form-row">
            <div className="form-group">
              <label>Grid X Range (mm):</label>
              <div className="range-inputs">
                <input
                  type="number"
                  value={config.grid_x_min}
                  onChange={(e) => setConfig({ ...config, grid_x_min: parseFloat(e.target.value) })}
                  placeholder="Min"
                />
                <span>to</span>
                <input
                  type="number"
                  value={config.grid_x_max}
                  onChange={(e) => setConfig({ ...config, grid_x_max: parseFloat(e.target.value) })}
                  placeholder="Max"
                />
              </div>
            </div>

            <div className="form-group">
              <label>Grid Y Range (mm):</label>
              <div className="range-inputs">
                <input
                  type="number"
                  value={config.grid_y_min}
                  onChange={(e) => setConfig({ ...config, grid_y_min: parseFloat(e.target.value) })}
                  placeholder="Min"
                />
                <span>to</span>
                <input
                  type="number"
                  value={config.grid_y_max}
                  onChange={(e) => setConfig({ ...config, grid_y_max: parseFloat(e.target.value) })}
                  placeholder="Max"
                />
              </div>
            </div>
          </div>

          <button 
            className="btn btn-primary"
            onClick={handleStartScan}
            disabled={!hardwareStatus?.printer.connected || !hardwareStatus?.camera.ready}
          >
            Start Scan
          </button>
        </div>
      )}

      {/* Scan Progress */}
      {(isScanning || isPaused || scanStatus) && (
        <div className="scan-progress">
          <h3>Scan Progress</h3>
          
          <div className="scan-info">
            <p><strong>Scan ID:</strong> {scanId}</p>
            <p><strong>Status:</strong> <span className={`status-badge ${scanStatus?.status}`}>
              {scanStatus?.status?.toUpperCase() || 'IN PROGRESS'}
            </span></p>
          </div>

          <div className="progress-container">
            <div 
              className="progress-bar"
              style={{ '--progress': `${progressPercent}%` } as React.CSSProperties}
            >
              <div className="progress-fill" />
            </div>
            <p className="progress-text">
              {scanStatus?.completed_points || 0} / {scanStatus?.total_points || 0} points
              ({progressPercent.toFixed(1)}%)
            </p>
          </div>

          {scanStatus?.quality_score !== undefined && (
            <div className="quality-score">
              <p><strong>Quality Score:</strong> {scanStatus.quality_score.toFixed(1)} / 100</p>
            </div>
          )}

          <div className="scan-controls">
            {isScanning && (
              <button className="btn btn-warning" onClick={handlePauseScan}>
                Pause Scan
              </button>
            )}
            
            {isPaused && (
              <button className="btn btn-success" onClick={handleResumeScan}>
                Resume Scan
              </button>
            )}
            
            <button className="btn btn-danger" onClick={handleStopScan}>
              Stop Scan
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ScanControl;

import React, { useState, useEffect } from 'react';
import './StereoCameraCalibration.css';

interface CalibrationStatus {
  state: string;
  progress: number;
  message: string;
  captures_completed: number;
  total_captures: number;
  calibration_running?: boolean;
}

interface CalibrationResult {
  status: string;
  baseline: number;
  reprojection_error: number;
  left_intrinsics: number[][];
  left_distortion: number[];
  right_intrinsics: number[][];
  right_distortion: number[];
  rotation_matrix: number[][];
  translation_vector: number[];
  total_captures: number;
  successful_captures: number;
  capture_success_rate: number;
}

interface CalibrationConfig {
  total_captures: number;
  heights: number[];
  tilt_angles: number[];
  z_sweep_range: [number, number];
  pattern_board_size: [number, number];
  pattern_square_size: number;
}

const StereoCameraCalibration: React.FC = () => {
  const [status, setStatus] = useState<CalibrationStatus | null>(null);
  const [result, setResult] = useState<CalibrationResult | null>(null);
  const [config, setConfig] = useState<CalibrationConfig | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [isRunning, setIsRunning] = useState(false);
  const [showConfig, setShowConfig] = useState(false);
  const [configForm, setConfigForm] = useState<Partial<CalibrationConfig>>({});
  const [error, setError] = useState<string | null>(null);

  // Fetch configuration
  useEffect(() => {
    fetchConfig();
  }, []);

  // Poll status during calibration
  useEffect(() => {
    if (!isRunning) return;

    const interval = setInterval(() => {
      fetchStatus();
    }, 1000);

    return () => clearInterval(interval);
  }, [isRunning]);

  const fetchConfig = async () => {
    try {
      const response = await fetch('/api/calibration/stereo/auto/config');
      if (response.ok) {
        const data: CalibrationConfig = await response.json();
        setConfig(data);
        setConfigForm(data);
      }
    } catch (err) {
      console.error('Failed to fetch config:', err);
    }
  };

  const fetchStatus = async () => {
    try {
      const response = await fetch('/api/calibration/stereo/auto/status');
      if (response.ok) {
        const data: CalibrationStatus = await response.json();
        setStatus(data);
        setIsRunning(data.calibration_running || false);

        // Auto-fetch results when complete
        if (data.state === 'complete') {
          fetchResults();
        }
      }
    } catch (err) {
      console.error('Failed to fetch status:', err);
    }
  };

  const fetchResults = async () => {
    try {
      const response = await fetch('/api/calibration/stereo/auto/results');
      if (response.ok) {
        const data: CalibrationResult = await response.json();
        setResult(data);
        setIsRunning(false);
      } else if (response.status === 202) {
        // Not ready yet
        setTimeout(fetchResults, 2000);
      }
    } catch (err) {
      console.error('Failed to fetch results:', err);
    }
  };

  const startCalibration = async () => {
    try {
      setError(null);
      setIsLoading(true);
      setResult(null);

      const response = await fetch('/api/calibration/stereo/auto/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      });

      if (response.ok || response.status === 202) {
        setIsRunning(true);
        setStatus({
          state: 'starting',
          progress: 0,
          message: 'Initializing stereo calibration...',
          captures_completed: 0,
          total_captures: 0,
          calibration_running: true
        });
        // Start polling
        setTimeout(fetchStatus, 500);
      } else {
        const data = await response.json();
        setError(data.error || 'Failed to start calibration');
      }
    } catch (err) {
      setError((err as Error).message);
    } finally {
      setIsLoading(false);
    }
  };

  const stopCalibration = async () => {
    try {
      const response = await fetch('/api/calibration/stereo/auto/stop', {
        method: 'POST'
      });

      if (response.ok) {
        setIsRunning(false);
        setStatus(prev => prev ? { ...prev, calibration_running: false } : null);
      }
    } catch (err) {
      console.error('Failed to stop calibration:', err);
    }
  };

  const saveCalibration = async () => {
    try {
      setError(null);
      const response = await fetch('/api/calibration/stereo/auto/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          filename: `stereo_calibration_${new Date().toISOString().split('T')[0]}.json`
        })
      });

      if (response.ok) {
        const data = await response.json();
        alert(data.message);
      } else {
        const data = await response.json();
        setError(data.error || 'Failed to save calibration');
      }
    } catch (err) {
      setError((err as Error).message);
    }
  };

  const updateConfig = async () => {
    try {
      setError(null);
      const response = await fetch('/api/calibration/stereo/auto/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(configForm)
      });

      if (response.ok) {
        await fetchConfig();
        setShowConfig(false);
        alert('Configuration updated');
      } else {
        const data = await response.json();
        setError(data.error || 'Failed to update config');
      }
    } catch (err) {
      setError((err as Error).message);
    }
  };

  return (
    <div className="stereo-calibration-container">
      <h1>Automatic Stereo Camera Calibration</h1>
      <p className="subtitle">Multi-view calibration using triple Z-axis positioning</p>

      {error && <div className="error-message">{error}</div>}

      {/* Control Panel */}
      <div className="control-panel">
        <button
          className="btn btn-primary btn-large"
          onClick={startCalibration}
          disabled={isRunning || isLoading}
        >
          {isRunning ? 'Calibration Running...' : 'Start Calibration'}
        </button>

        {isRunning && (
          <button
            className="btn btn-danger"
            onClick={stopCalibration}
          >
            Stop Calibration
          </button>
        )}

        <button
          className="btn btn-secondary"
          onClick={() => setShowConfig(!showConfig)}
        >
          {showConfig ? 'Hide Config' : 'Configure'}
        </button>

        {result && (
          <button
            className="btn btn-success"
            onClick={saveCalibration}
          >
            Save Results
          </button>
        )}
      </div>

      {/* Configuration Panel */}
      {showConfig && (
        <div className="config-panel">
          <h3>Calibration Configuration</h3>
          <div className="config-grid">
            <div className="config-field">
              <label>Heights (mm):</label>
              <input
                type="text"
                value={configForm.heights?.join(', ') || ''}
                onChange={(e) => setConfigForm({
                  ...configForm,
                  heights: e.target.value.split(',').map(h => parseFloat(h.trim()))
                })}
                placeholder="e.g., 200, 225, 250"
              />
            </div>
            <div className="config-field">
              <label>Tilt Angles (°):</label>
              <input
                type="text"
                value={configForm.tilt_angles?.join(', ') || ''}
                onChange={(e) => setConfigForm({
                  ...configForm,
                  tilt_angles: e.target.value.split(',').map(a => parseFloat(a.trim()))
                })}
                placeholder="e.g., -5, 0, 5"
              />
            </div>
            <div className="config-field">
              <label>Pattern Board Size:</label>
              <input
                type="text"
                value={`${configForm.pattern_board_size?.[0] || 9} x ${configForm.pattern_board_size?.[1] || 6}`}
                disabled
              />
            </div>
            <div className="config-field">
              <label>Square Size (mm):</label>
              <input
                type="number"
                value={configForm.pattern_square_size || 30}
                onChange={(e) => setConfigForm({
                  ...configForm,
                  pattern_square_size: parseFloat(e.target.value)
                })}
              />
            </div>
          </div>
          <button className="btn btn-primary" onClick={updateConfig}>
            Update Configuration
          </button>
        </div>
      )}

      {/* Status Panel */}
      {status && (
        <div className="status-panel">
          <h3>Calibration Status</h3>
          <div className="status-grid">
            <div className="status-item">
              <label>State:</label>
              <span className={`status-value state-${status.state}`}>
                {status.state.toUpperCase()}
              </span>
            </div>
            <div className="status-item">
              <label>Progress:</label>
              <span className="status-value">{status.progress.toFixed(1)}%</span>
            </div>
            <div className="status-item">
              <label>Captures:</label>
              <span className="status-value">
                {status.captures_completed} / {status.total_captures}
              </span>
            </div>
            <div className="status-item">
              <label>Message:</label>
              <span className="status-value message">{status.message}</span>
            </div>
          </div>

          {/* Progress Bar */}
          <div className="progress-bar-container">
            <div
              className="progress-bar-fill"
              style={{ width: `${status.progress}%` }}
            />
          </div>
        </div>
      )}

      {/* Results Panel */}
      {result && (
        <div className="results-panel">
          <h3>Calibration Results</h3>

          {/* Overall Metrics */}
          <div className="results-section">
            <h4>Overall Metrics</h4>
            <div className="metrics-grid">
              <div className="metric">
                <label>Baseline:</label>
                <span>{result.baseline.toFixed(2)} mm</span>
              </div>
              <div className="metric">
                <label>Reprojection Error:</label>
                <span>{result.reprojection_error.toFixed(4)} px</span>
              </div>
              <div className="metric">
                <label>Total Captures:</label>
                <span>{result.total_captures}</span>
              </div>
              <div className="metric">
                <label>Successful Captures:</label>
                <span>{result.successful_captures}</span>
              </div>
              <div className="metric">
                <label>Success Rate:</label>
                <span>{result.capture_success_rate.toFixed(1)}%</span>
              </div>
            </div>
          </div>

          {/* Camera Intrinsics */}
          <div className="results-section">
            <h4>Camera Intrinsics</h4>
            <div className="intrinsics-grid">
              <div className="intrinsic-camera">
                <h5>Left Camera</h5>
                <div className="matrix-display">
                  <div className="matrix-row">
                    <span>{result.left_intrinsics[0][0].toFixed(2)}</span>
                    <span>{result.left_intrinsics[0][1].toFixed(2)}</span>
                    <span>{result.left_intrinsics[0][2].toFixed(2)}</span>
                  </div>
                  <div className="matrix-row">
                    <span>{result.left_intrinsics[1][0].toFixed(2)}</span>
                    <span>{result.left_intrinsics[1][1].toFixed(2)}</span>
                    <span>{result.left_intrinsics[1][2].toFixed(2)}</span>
                  </div>
                  <div className="matrix-row">
                    <span>{result.left_intrinsics[2][0].toFixed(2)}</span>
                    <span>{result.left_intrinsics[2][1].toFixed(2)}</span>
                    <span>{result.left_intrinsics[2][2].toFixed(2)}</span>
                  </div>
                </div>
                <div className="distortion-display">
                  <span>Distortion: {result.left_distortion.map(d => d.toFixed(4)).join(', ')}</span>
                </div>
              </div>

              <div className="intrinsic-camera">
                <h5>Right Camera</h5>
                <div className="matrix-display">
                  <div className="matrix-row">
                    <span>{result.right_intrinsics[0][0].toFixed(2)}</span>
                    <span>{result.right_intrinsics[0][1].toFixed(2)}</span>
                    <span>{result.right_intrinsics[0][2].toFixed(2)}</span>
                  </div>
                  <div className="matrix-row">
                    <span>{result.right_intrinsics[1][0].toFixed(2)}</span>
                    <span>{result.right_intrinsics[1][1].toFixed(2)}</span>
                    <span>{result.right_intrinsics[1][2].toFixed(2)}</span>
                  </div>
                  <div className="matrix-row">
                    <span>{result.right_intrinsics[2][0].toFixed(2)}</span>
                    <span>{result.right_intrinsics[2][1].toFixed(2)}</span>
                    <span>{result.right_intrinsics[2][2].toFixed(2)}</span>
                  </div>
                </div>
                <div className="distortion-display">
                  <span>Distortion: {result.right_distortion.map(d => d.toFixed(4)).join(', ')}</span>
                </div>
              </div>
            </div>
          </div>

          {/* Stereo Geometry */}
          <div className="results-section">
            <h4>Stereo Geometry</h4>
            <div className="geometry-grid">
              <div className="geometry-item">
                <h5>Rotation Matrix</h5>
                <div className="matrix-display">
                  {result.rotation_matrix.map((row, i) => (
                    <div key={i} className="matrix-row">
                      {row.map((val, j) => (
                        <span key={j}>{val.toFixed(4)}</span>
                      ))}
                    </div>
                  ))}
                </div>
              </div>

              <div className="geometry-item">
                <h5>Translation Vector</h5>
                <div className="vector-display">
                  {result.translation_vector.map((val, i) => (
                    <div key={i} className="vector-row">
                      <span>T{i}:</span>
                      <span>{val.toFixed(4)}</span>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Empty State */}
      {!status && !result && (
        <div className="empty-state">
          <p>Click "Start Calibration" to begin automatic stereo camera calibration</p>
          <p className="hint">
            The system will position the camera at 15-20 different viewpoints using the triple Z-axis,
            capture calibration patterns, and compute stereo parameters automatically.
          </p>
        </div>
      )}
    </div>
  );
};

export default StereoCameraCalibration;

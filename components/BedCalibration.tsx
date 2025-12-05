/**
 * Bed Tilt Calibration Component with Triple Z-Axis Support
 * 
 * Handles camera calibration and bed leveling using independent Z motors
 */

import React, { useState, useEffect } from 'react';
import './BedCalibration.css';

interface PlaneEquation {
  a: number;
  b: number;
  c: number;
}

interface TiltAngles {
  tilt_x_degrees: number;
  tilt_y_degrees: number;
}

interface ZOffsets {
  z1_mm: number;
  z2_mm: number;
  z3_mm: number;
}

interface CalibrationState {
  isCalibrating: boolean;
  tiltData: {
    plane_equation?: PlaneEquation;
    tilt_angles?: TiltAngles;
    z_offsets?: ZOffsets;
    residual_mm?: number;
  } | null;
  gridSize: number;
  referenceHeight: number;
}

const BedCalibration: React.FC = () => {
  const [state, setState] = useState<CalibrationState>({
    isCalibrating: false,
    tiltData: null,
    gridSize: 3,
    referenceHeight: 10.0,
  });

  const [currentTilt, setCurrentTilt] = useState<any>(null);
  const [appliedOffsets, setAppliedOffsets] = useState<ZOffsets | null>(null);
  const [calibrationMessage, setCalibrationMessage] = useState<string>('');

  // Fetch current calibration on mount
  useEffect(() => {
    const fetchCurrentTilt = async () => {
      try {
        const response = await fetch('/api/calibration/bed-tilt/current');
        if (response.ok) {
          const data = await response.json();
          setCurrentTilt(data);
          setAppliedOffsets(data.z_offsets);
        }
      } catch (error) {
        console.error('Failed to fetch current tilt:', error);
      }
    };

    fetchCurrentTilt();
  }, []);

  // Handle auto calibration
  const handleAutoCalibrate = async () => {
    try {
      setState(prev => ({ ...prev, isCalibrating: true }));
      setCalibrationMessage('Starting auto bed calibration...');

      const response = await fetch('/api/calibration/bed-tilt/calibrate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          grid_rows: state.gridSize,
          grid_cols: state.gridSize,
          reference_height: state.referenceHeight,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setState(prev => ({
          ...prev,
          tiltData: data,
          isCalibrating: false,
        }));
        setCurrentTilt(data);
        setAppliedOffsets(data.z_offsets);
        setCalibrationMessage('✓ Calibration complete!');
      } else {
        setCalibrationMessage('✗ Calibration failed');
        setState(prev => ({ ...prev, isCalibrating: false }));
      }
    } catch (error) {
      console.error('Calibration error:', error);
      setCalibrationMessage('Error during calibration');
      setState(prev => ({ ...prev, isCalibrating: false }));
    }
  };

  // Handle apply offsets
  const handleApplyOffsets = async () => {
    if (!state.tiltData?.z_offsets) {
      alert('No offsets to apply');
      return;
    }

    try {
      const response = await fetch('/api/calibration/bed-tilt/apply-triple-z', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(state.tiltData.z_offsets),
      });

      if (response.ok) {
        setAppliedOffsets(state.tiltData.z_offsets);
        setCalibrationMessage('✓ Offsets applied to triple Z motors');
      } else {
        setCalibrationMessage('✗ Failed to apply offsets');
      }
    } catch (error) {
      console.error('Apply offsets error:', error);
      setCalibrationMessage('Error applying offsets');
    }
  };

  return (
    <div className="bed-calibration">
      <h2>Bed Tilt Calibration (Triple Z-Axis)</h2>

      {/* Configuration Panel */}
      <div className="calibration-config">
        <h3>Configuration</h3>
        
        <div className="config-grid">
          <div className="config-item">
            <label>Grid Size (N×N):</label>
            <div className="input-group">
              <button onClick={() => setState(prev => ({
                ...prev,
                gridSize: Math.max(2, prev.gridSize - 1)
              }))}>−</button>
              <input
                type="number"
                value={state.gridSize}
                onChange={(e) => setState(prev => ({
                  ...prev,
                  gridSize: parseInt(e.target.value) || 3
                }))}
                min="2"
                max="5"
              />
              <button onClick={() => setState(prev => ({
                ...prev,
                gridSize: Math.min(5, prev.gridSize + 1)
              }))}>+</button>
            </div>
            <span className="info">Total points: {state.gridSize * state.gridSize}</span>
          </div>

          <div className="config-item">
            <label>Reference Height (mm):</label>
            <input
              type="number"
              value={state.referenceHeight}
              onChange={(e) => setState(prev => ({
                ...prev,
                referenceHeight: parseFloat(e.target.value) || 10.0
              }))}
              step="0.5"
              min="1"
              max="50"
            />
            <span className="info">Above workpiece</span>
          </div>
        </div>

        <button 
          className="btn btn-primary btn-large"
          onClick={handleAutoCalibrate}
          disabled={state.isCalibrating}
        >
          {state.isCalibrating ? 'Calibrating...' : 'Start Auto Calibration'}
        </button>

        {calibrationMessage && (
          <div className={`message ${calibrationMessage.includes('✓') ? 'success' : 'error'}`}>
            {calibrationMessage}
          </div>
        )}
      </div>

      {/* Calibration Results */}
      {state.tiltData && (
        <div className="calibration-results">
          <h3>Calibration Results</h3>

          {/* Plane Equation */}
          <div className="result-section">
            <h4>Bed Plane Equation</h4>
            <div className="equation">
              z = {state.tiltData.plane_equation?.a.toFixed(6)} × x + 
                  {state.tiltData.plane_equation?.b.toFixed(6)} × y + 
                  {state.tiltData.plane_equation?.c.toFixed(2)}
            </div>
          </div>

          {/* Tilt Angles */}
          {state.tiltData.tilt_angles && (
            <div className="result-section">
              <h4>Tilt Angles</h4>
              <div className="tilt-angles">
                <div className="angle-item">
                  <span className="label">X-axis tilt:</span>
                  <span className="value">{state.tiltData.tilt_angles.tilt_x_degrees.toFixed(2)}°</span>
                </div>
                <div className="angle-item">
                  <span className="label">Y-axis tilt:</span>
                  <span className="value">{state.tiltData.tilt_angles.tilt_y_degrees.toFixed(2)}°</span>
                </div>
              </div>
            </div>
          )}

          {/* Z Offsets */}
          {state.tiltData.z_offsets && (
            <div className="result-section">
              <h4>Triple Z-Motor Offsets</h4>
              <div className="z-motors">
                <div className="motor-box motor-1">
                  <span className="motor-label">Motor 1 (Front-Left)</span>
                  <span className="motor-value">{state.tiltData.z_offsets.z1_mm.toFixed(2)} mm</span>
                </div>
                <div className="motor-box motor-2">
                  <span className="motor-label">Motor 2 (Front-Right)</span>
                  <span className="motor-value">{state.tiltData.z_offsets.z2_mm.toFixed(2)} mm</span>
                </div>
                <div className="motor-box motor-3">
                  <span className="motor-label">Motor 3 (Back-Left)</span>
                  <span className="motor-value">{state.tiltData.z_offsets.z3_mm.toFixed(2)} mm</span>
                </div>
              </div>

              <button 
                className="btn btn-success btn-large"
                onClick={handleApplyOffsets}
              >
                Apply Z Motor Offsets
              </button>
            </div>
          )}

          {/* Accuracy */}
          {state.tiltData.residual_mm !== undefined && (
            <div className="result-section">
              <h4>Fit Accuracy</h4>
              <div className="accuracy">
                <span>Residual Error: </span>
                <span className={`error-value ${state.tiltData.residual_mm < 0.5 ? 'good' : 'warning'}`}>
                  {state.tiltData.residual_mm.toFixed(4)} mm
                </span>
              </div>
            </div>
          )}
        </div>
      )}

      {/* Current Calibration Status */}
      {currentTilt && (
        <div className="current-status">
          <h3>Current Calibration Status</h3>
          
          <div className="status-info">
            <div className="status-item">
              <span className="label">Plane Equation:</span>
              <span className="value">
                z = {currentTilt.plane_equation?.a.toFixed(6)} × x + 
                    {currentTilt.plane_equation?.b.toFixed(6)} × y + 
                    {currentTilt.plane_equation?.c.toFixed(2)}
              </span>
            </div>

            <div className="status-item">
              <span className="label">Tilt X:</span>
              <span className="value">{currentTilt.tilt_angles?.tilt_x_degrees.toFixed(2)}°</span>
            </div>

            <div className="status-item">
              <span className="label">Tilt Y:</span>
              <span className="value">{currentTilt.tilt_angles?.tilt_y_degrees.toFixed(2)}°</span>
            </div>

            <div className="status-item">
              <span className="label">Measured Points:</span>
              <span className="value">{currentTilt.measured_points}</span>
            </div>
          </div>

          {appliedOffsets && (
            <div className="applied-offsets">
              <h4>Currently Applied Z Motor Offsets</h4>
              <div className="offset-display">
                <div className="offset-item">
                  <span>Z1:</span>
                  <span className="offset-value">{appliedOffsets.z1_mm.toFixed(2)} mm</span>
                </div>
                <div className="offset-item">
                  <span>Z2:</span>
                  <span className="offset-value">{appliedOffsets.z2_mm.toFixed(2)} mm</span>
                </div>
                <div className="offset-item">
                  <span>Z3:</span>
                  <span className="offset-value">{appliedOffsets.z3_mm.toFixed(2)} mm</span>
                </div>
              </div>
            </div>
          )}
        </div>
      )}

      {/* Info Section */}
      <div className="info-section">
        <h4>How It Works</h4>
        <ul>
          <li><strong>Probe Grid:</strong> System probes {state.gridSize * state.gridSize} points across the bed</li>
          <li><strong>Plane Fitting:</strong> Calculates bed tilt plane using least squares</li>
          <li><strong>Z Motor Offsets:</strong> Computes independent Z motor positions to level the bed</li>
          <li><strong>RDK Stereo Camera:</strong> Enables precise depth calibration on tilted surfaces</li>
        </ul>
      </div>
    </div>
  );
};

export default BedCalibration;

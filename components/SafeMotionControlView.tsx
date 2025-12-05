import React, { useState, useEffect } from 'react';
import {
  Play,
  Square,
  Home,
  AlertTriangle,
  CheckCircle2,
  Cpu,
  Zap,
  Activity,
  Settings as SettingsIcon,
  TrendingUp,
  Lock,
  AlertCircle,
  Radio,
} from 'lucide-react';

interface MotionState {
  position_x_mm: number;
  position_y_mm: number;
  position_z_mm: number;
  homed_x: boolean;
  homed_y: boolean;
  homed_z: boolean;
  virtual_zero_offset_z_mm: number;
}

interface PanoramaConfig {
  start_x_mm: number;
  end_x_mm: number;
  step_mm: number;
  maintain_height_mm: number;
}

type MotionStatus = 'idle' | 'homing' | 'moving' | 'scanning' | 'error';

const SAFE_MOTION_SOFT_LIMIT_MM = 90;
const VIRTUAL_ZERO_OFFSET_MM = 100; // Z=0 = 100mm above bed (safe parking)

const SafeMotionControlView: React.FC = () => {
  // State management
  const [status, setStatus] = useState<MotionStatus>('idle');
  const [isConnected, setIsConnected] = useState(true);
  const [motionState, setMotionState] = useState<MotionState>({
    position_x_mm: 0,
    position_y_mm: 0,
    position_z_mm: 0,
    homed_x: false,
    homed_y: false,
    homed_z: false,
    virtual_zero_offset_z_mm: VIRTUAL_ZERO_OFFSET_MM,
  });

  const [targetHeight, setTargetHeight] = useState(50);
  const [showPanoramaConfig, setShowPanoramaConfig] = useState(false);
  const [panoramaConfig, setPanoramaConfig] = useState<PanoramaConfig>({
    start_x_mm: 0,
    end_x_mm: 100,
    step_mm: 5,
    maintain_height_mm: 50,
  });
  const [errorMessage, setErrorMessage] = useState('');
  const [lockMotors, setLockMotors] = useState(true);
  const [vibrationSettleTime, setVibrationSettleTime] = useState(2);

  // Check if rig is fully homed
  const isFullyHomed = motionState.homed_x && motionState.homed_y && motionState.homed_z;

  // Check if height is safe
  const isHeightSafe = motionState.position_z_mm <= SAFE_MOTION_SOFT_LIMIT_MM;

  // Handle safe homing (X/Y first, then Z)
  const handleSafeHome = async () => {
    setErrorMessage('');
    setStatus('homing');

    try {
      // Mock API call - replace with real backend
      console.log('Starting safe homing sequence...');
      
      // Simulate homing stages
      await new Promise((resolve) => setTimeout(resolve, 1500));
      setMotionState((prev) => ({
        ...prev,
        homed_x: true,
        homed_y: true,
        position_x_mm: 0,
        position_y_mm: 0,
      }));

      await new Promise((resolve) => setTimeout(resolve, 1500));
      setMotionState((prev) => ({
        ...prev,
        homed_z: true,
        position_z_mm: 0,
      }));

      setStatus('idle');
    } catch (error) {
      setStatus('error');
      setErrorMessage(`Homing failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  // Park toolhead (safe position)
  const handleParkToolhead = async () => {
    setErrorMessage('');
    setStatus('moving');

    try {
      console.log('Parking toolhead to safe position (Z=0)...');
      await new Promise((resolve) => setTimeout(resolve, 1500));

      setMotionState((prev) => ({
        ...prev,
        position_z_mm: 0,
      }));

      setStatus('idle');
    } catch (error) {
      setStatus('error');
      setErrorMessage(`Park failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  // Move to inspection height
  const handleMoveToHeight = async () => {
    if (targetHeight > SAFE_MOTION_SOFT_LIMIT_MM) {
      setErrorMessage(
        `Target height ${targetHeight}mm exceeds soft limit ${SAFE_MOTION_SOFT_LIMIT_MM}mm`
      );
      return;
    }

    if (!isFullyHomed) {
      setErrorMessage('Rig must be homed before moving to inspection height');
      return;
    }

    setErrorMessage('');
    setStatus('moving');

    try {
      console.log(`Moving to inspection height: ${targetHeight}mm`);
      await new Promise((resolve) => setTimeout(resolve, 1000));

      setMotionState((prev) => ({
        ...prev,
        position_z_mm: targetHeight,
      }));

      if (lockMotors) {
        console.log('Locking motors for vibration control...');
        await new Promise((resolve) => setTimeout(resolve, 500));
      }

      setStatus('idle');
    } catch (error) {
      setStatus('error');
      setErrorMessage(
        `Move failed: ${error instanceof Error ? error.message : 'Unknown error'}`
      );
    }
  };

  // Emergency stop
  const handleEmergencyStop = () => {
    console.log('EMERGENCY STOP TRIGGERED');
    setStatus('idle');
    setLockMotors(true);
    setErrorMessage('Emergency stop activated. All motors locked.');
  };

  // Start panorama scan at current height
  const handleStartPanoramaScan = async () => {
    if (!isFullyHomed) {
      setErrorMessage('Rig must be homed before scanning');
      return;
    }

    setErrorMessage('');
    setStatus('scanning');

    try {
      console.log('Starting panorama scan at height:', motionState.position_z_mm);
      
      // Simulate scanning
      await new Promise((resolve) => setTimeout(resolve, 5000));

      setStatus('idle');
      console.log('Panorama scan complete');
    } catch (error) {
      setStatus('error');
      setErrorMessage(`Scan failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  return (
    <div className="space-y-6">
      {/* Status Section */}
      <div className="grid grid-cols-1 md:grid-cols-5 gap-4">
        {/* Connection Status */}
        <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Connection</span>
            {isConnected ? (
              <CheckCircle2 className="w-4 h-4 text-industrial-success" />
            ) : (
              <AlertTriangle className="w-4 h-4 text-industrial-danger" />
            )}
          </div>
          <p className="text-lg font-semibold text-white">
            {isConnected ? 'Connected' : 'Disconnected'}
          </p>
        </div>

        {/* Rig Status */}
        <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Rig Status</span>
            {isFullyHomed ? (
              <CheckCircle2 className="w-4 h-4 text-industrial-success" />
            ) : (
              <AlertCircle className="w-4 h-4 text-industrial-warning" />
            )}
          </div>
          <p className="text-lg font-semibold text-white">
            {isFullyHomed ? 'Homed' : 'Not Homed'}
          </p>
        </div>

        {/* Current Status */}
        <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Status</span>
            <Cpu className="w-4 h-4 text-industrial-blue" />
          </div>
          <p className="text-lg font-semibold text-white capitalize">{status}</p>
        </div>

        {/* Height Safety */}
        <div className={`rounded-lg p-4 border ${
          isHeightSafe
            ? 'bg-industrial-success/10 border-industrial-success/30'
            : 'bg-industrial-danger/10 border-industrial-danger/30'
        }`}>
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Height Safe</span>
            {isHeightSafe ? (
              <CheckCircle2 className="w-4 h-4 text-industrial-success" />
            ) : (
              <AlertTriangle className="w-4 h-4 text-industrial-danger" />
            )}
          </div>
          <p className={`text-lg font-semibold ${
            isHeightSafe ? 'text-industrial-success' : 'text-industrial-danger'
          }`}>
            {isHeightSafe ? 'Safe' : 'EXCEEDS LIMIT'}
          </p>
        </div>

        {/* Motor Lock Status */}
        <div className={`rounded-lg p-4 border ${
          lockMotors
            ? 'bg-industrial-success/10 border-industrial-success/30'
            : 'bg-slate-700 border-slate-600'
        }`}>
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Motors</span>
            <Lock className="w-4 h-4 text-industrial-success" />
          </div>
          <p className={`text-lg font-semibold ${
            lockMotors ? 'text-industrial-success' : 'text-slate-300'
          }`}>
            {lockMotors ? 'Locked' : 'Free'}
          </p>
        </div>
      </div>

      {/* Main Control Area */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Motion Controls */}
        <div className="lg:col-span-2 bg-slate-800 border border-slate-700 rounded-lg p-6">
          <h3 className="text-lg font-semibold text-white mb-6 flex items-center">
            <Cpu className="w-5 h-5 mr-2 text-industrial-blue" />
            Motion Control
          </h3>

          {/* Error Message */}
          {errorMessage && (
            <div className="mb-4 p-3 bg-industrial-danger/10 border border-industrial-danger/30 rounded-lg flex items-start space-x-3">
              <AlertTriangle className="w-5 h-5 text-industrial-danger flex-shrink-0 mt-0.5" />
              <p className="text-sm text-industrial-danger">{errorMessage}</p>
            </div>
          )}

          {/* Position Display */}
          <div className="bg-slate-900 rounded-lg p-4 mb-6">
            <h4 className="text-sm font-semibold text-white mb-3">Current Position</h4>
            <div className="grid grid-cols-3 gap-4 text-sm">
              <div>
                <p className="text-slate-400">X Position</p>
                <p className="text-xl font-semibold text-white">{motionState.position_x_mm} mm</p>
                <p className="text-xs text-slate-500 mt-1">
                  {motionState.homed_x ? '✓ Homed' : 'Not Homed'}
                </p>
              </div>
              <div>
                <p className="text-slate-400">Y Position</p>
                <p className="text-xl font-semibold text-white">{motionState.position_y_mm} mm</p>
                <p className="text-xs text-slate-500 mt-1">
                  {motionState.homed_y ? '✓ Homed' : 'Not Homed'}
                </p>
              </div>
              <div>
                <p className="text-slate-400">Z Position (Virtual)</p>
                <p className="text-xl font-semibold text-white">{motionState.position_z_mm} mm</p>
                <p className="text-xs text-slate-500 mt-1">
                  {motionState.homed_z ? '✓ Homed' : 'Not Homed'}
                </p>
              </div>
            </div>
            <div className="mt-4 pt-4 border-t border-slate-700">
              <p className="text-xs text-slate-400 mb-2">Virtual Zero Concept:</p>
              <p className="text-sm text-slate-300">
                Z=0 = Safe parking (camera 100mm above bed)
                <br />
                Z=0 to {SAFE_MOTION_SOFT_LIMIT_MM}mm = Inspection range
              </p>
            </div>
          </div>

          {/* Homing Controls */}
          <div className="mb-6">
            <h4 className="text-sm font-semibold text-white mb-3">Homing Sequence</h4>
            <div className="grid grid-cols-2 gap-3">
              <button
                onClick={handleSafeHome}
                disabled={status !== 'idle'}
                className="flex items-center justify-center space-x-2 px-4 py-3 bg-industrial-blue hover:bg-industrial-blue/90 disabled:bg-slate-600 disabled:cursor-not-allowed text-white rounded-lg font-medium transition-colors"
              >
                <Home className="w-4 h-4" />
                <span>Safe Home (X/Y→Z)</span>
              </button>
              <button
                onClick={handleParkToolhead}
                disabled={!isFullyHomed || status !== 'idle'}
                className="flex items-center justify-center space-x-2 px-4 py-3 bg-industrial-success hover:bg-industrial-success/90 disabled:bg-slate-600 disabled:cursor-not-allowed text-white rounded-lg font-medium transition-colors"
              >
                <Zap className="w-4 h-4" />
                <span>Park (Z=0)</span>
              </button>
            </div>
          </div>

          {/* Height Control */}
          <div className="mb-6">
            <h4 className="text-sm font-semibold text-white mb-3">Inspection Height Control</h4>
            <div className="bg-slate-900 rounded-lg p-4">
              <div className="mb-4">
                <div className="flex items-center justify-between mb-2">
                  <label className="text-sm text-slate-300">Target Height</label>
                  <span className={`font-semibold ${
                    targetHeight <= SAFE_MOTION_SOFT_LIMIT_MM
                      ? 'text-industrial-success'
                      : 'text-industrial-danger'
                  }`}>
                    {targetHeight} mm
                  </span>
                </div>
                <input
                  type="range"
                  min="0"
                  max={SAFE_MOTION_SOFT_LIMIT_MM}
                  value={targetHeight}
                  onChange={(e) => setTargetHeight(parseFloat(e.target.value))}
                  className="w-full"
                />
                <div className="mt-2 text-xs text-slate-400 flex justify-between">
                  <span>Park (0mm)</span>
                  <span>Soft Limit ({SAFE_MOTION_SOFT_LIMIT_MM}mm)</span>
                </div>
              </div>

              <button
                onClick={handleMoveToHeight}
                disabled={!isFullyHomed || status !== 'idle'}
                className="w-full px-4 py-2 bg-industrial-blue hover:bg-industrial-blue/90 disabled:bg-slate-600 disabled:cursor-not-allowed text-white rounded font-medium transition-colors"
              >
                Move to Height
              </button>
            </div>
          </div>

          {/* Safety Options */}
          <div className="mb-6 bg-slate-900 rounded-lg p-4">
            <h4 className="text-sm font-semibold text-white mb-3">Safety Options</h4>
            <div className="space-y-3">
              <label className="flex items-center space-x-3 cursor-pointer">
                <input
                  type="checkbox"
                  checked={lockMotors}
                  onChange={(e) => setLockMotors(e.target.checked)}
                  disabled={status !== 'idle'}
                  className="w-4 h-4 rounded bg-slate-700 border-slate-600"
                />
                <span className="text-sm text-slate-300">Lock motors after movement (reduces vibration)</span>
              </label>
              <div>
                <label className="text-sm text-slate-300 block mb-1">
                  Vibration Settle Time: {vibrationSettleTime}s
                </label>
                <input
                  type="range"
                  min="0"
                  max="5"
                  step="0.5"
                  value={vibrationSettleTime}
                  onChange={(e) => setVibrationSettleTime(parseFloat(e.target.value))}
                  className="w-full"
                />
              </div>
            </div>
          </div>

          {/* Emergency Stop */}
          <button
            onClick={handleEmergencyStop}
            className="w-full px-6 py-3 bg-industrial-danger hover:bg-industrial-danger/90 text-white rounded-lg font-semibold transition-colors flex items-center justify-center space-x-2"
          >
            <AlertTriangle className="w-5 h-5" />
            <span>EMERGENCY STOP</span>
          </button>
        </div>

        {/* Info Panel */}
        <div className="space-y-4">
          {/* Virtual Zero Info */}
          <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
            <h4 className="text-sm font-semibold text-white mb-3 flex items-center">
              <Radio className="w-4 h-4 mr-2 text-industrial-blue" />
              Virtual Zero Concept
            </h4>
            <div className="space-y-2 text-sm text-slate-300">
              <div className="bg-slate-900 p-2 rounded">
                <p className="font-mono font-semibold text-white">Z=0 = Safe Parking</p>
                <p className="text-xs text-slate-400">Camera 100mm above bed</p>
              </div>
              <div className="bg-slate-900 p-2 rounded">
                <p className="font-mono font-semibold text-white">Z=+1 to +{SAFE_MOTION_SOFT_LIMIT_MM}mm</p>
                <p className="text-xs text-slate-400">Inspection range (descending)</p>
              </div>
              <p className="text-xs text-slate-400 pt-2">
                This offset prevents accidental lens crashes while maintaining intuitive motion control.
              </p>
            </div>
          </div>

          {/* Homing Status */}
          <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
            <h4 className="text-sm font-semibold text-white mb-3">Homing Status</h4>
            <div className="space-y-2 text-sm">
              <div className="flex items-center justify-between">
                <span className="text-slate-400">X-Axis</span>
                <span className={motionState.homed_x ? 'text-industrial-success' : 'text-industrial-warning'}>
                  {motionState.homed_x ? '✓ Homed' : 'Not Homed'}
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-slate-400">Y-Axis</span>
                <span className={motionState.homed_y ? 'text-industrial-success' : 'text-industrial-warning'}>
                  {motionState.homed_y ? '✓ Homed' : 'Not Homed'}
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-slate-400">Z-Axis</span>
                <span className={motionState.homed_z ? 'text-industrial-success' : 'text-industrial-warning'}>
                  {motionState.homed_z ? '✓ Homed' : 'Not Homed'}
                </span>
              </div>
            </div>
          </div>

          {/* Soft Limits */}
          <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
            <h4 className="text-sm font-semibold text-white mb-3 flex items-center">
              <AlertCircle className="w-4 h-4 mr-2 text-industrial-warning" />
              Soft Limits
            </h4>
            <div className="space-y-2 text-sm">
              <div>
                <p className="text-slate-400">Z Maximum</p>
                <p className="text-white font-semibold">{SAFE_MOTION_SOFT_LIMIT_MM} mm</p>
              </div>
              <div className="bg-slate-900 p-2 rounded mt-2">
                <p className="text-xs text-slate-400">
                  Heights above {SAFE_MOTION_SOFT_LIMIT_MM}mm are blocked to prevent lens damage.
                </p>
              </div>
            </div>
          </div>

          {/* Panorama Scanner Panel */}
          <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
            <h4 className="text-sm font-semibold text-white mb-3 flex items-center">
              <Activity className="w-4 h-4 mr-2 text-industrial-blue" />
              Panorama at Current Height
            </h4>
            <button
              onClick={handleStartPanoramaScan}
              disabled={!isFullyHomed || status !== 'idle'}
              className="w-full px-4 py-2 bg-industrial-blue hover:bg-industrial-blue/90 disabled:bg-slate-600 disabled:cursor-not-allowed text-white rounded font-medium transition-colors mb-2"
            >
              Start Scan
            </button>
            <p className="text-xs text-slate-400">
              Scans from {panoramaConfig.start_x_mm}mm to {panoramaConfig.end_x_mm}mm at height {motionState.position_z_mm}mm
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SafeMotionControlView;

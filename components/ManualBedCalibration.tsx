import React, { useState, useRef, useEffect } from 'react';
import {
  AlertTriangle,
  CheckCircle2,
  Camera,
  Zap,
  Save,
  RotateCcw,
  Activity,
  ArrowUp,
  ArrowDown,
  RotateCw,
  HelpCircle,
} from 'lucide-react';

interface CalibrationResult {
  matrix: number[][];
  distCoeffs: number[];
  error: number;
  timestamp: string;
}

type CalibrationStatus = 'idle' | 'capturing' | 'processing' | 'complete' | 'error';

const ManualBedCalibration: React.FC = () => {
  // Camera and UI state
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isCameraActive, setIsCameraActive] = useState(false);
  const [cameraError, setCameraError] = useState<string | null>(null);

  // Calibration state
  const [status, setStatus] = useState<CalibrationStatus>('idle');
  const [progress, setProgress] = useState(0);
  const [result, setResult] = useState<CalibrationResult | null>(null);
  const [errorMessage, setErrorMessage] = useState<string | null>(null);

  // Manual adjustment guides
  const [showTilting, setShowTilting] = useState(true);
  const [estimatedTilt, setEstimatedTilt] = useState(0);
  const [estimatedHeight, setEstimatedHeight] = useState(0);

  // Initialize camera
  useEffect(() => {
    let stream: MediaStream | null = null;

    const initCamera = async () => {
      try {
        stream = await navigator.mediaDevices.getUserMedia({
          video: {
            width: { ideal: 1920 },
            height: { ideal: 1080 },
            facingMode: 'environment',
          },
        });

        if (videoRef.current) {
          videoRef.current.srcObject = stream;
          setIsCameraActive(true);
          setCameraError(null);
        }
      } catch (err) {
        setCameraError(
          `Camera access failed: ${err instanceof Error ? err.message : 'Unknown error'}`
        );
        setIsCameraActive(false);
      }
    };

    initCamera();

    return () => {
      if (stream) {
        stream.getTracks().forEach((track) => track.stop());
      }
    };
  }, []);

  // Capture single frame from video
  const captureFrame = () => {
    if (!videoRef.current || !canvasRef.current || !isCameraActive) {
      setErrorMessage('Camera not ready. Please wait and try again.');
      return;
    }

    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;

    canvasRef.current.width = videoRef.current.videoWidth;
    canvasRef.current.height = videoRef.current.videoHeight;
    ctx.drawImage(videoRef.current, 0, 0);

    return canvasRef.current.toDataURL('image/png');
  };

  // Handle capture and calibrate
  const handleCaptureAndCalibrate = async () => {
    setErrorMessage(null);
    setStatus('capturing');

    try {
      // Capture frame
      const frameData = captureFrame();
      if (!frameData) {
        throw new Error('Failed to capture frame');
      }

      setProgress(30);

      // Simulate sending to backend for processing
      setStatus('processing');
      setProgress(60);

      // Mock calibration result
      await new Promise((resolve) => setTimeout(resolve, 2000));

      const mockResult: CalibrationResult = {
        matrix: [
          [1200.5, 0, 960],
          [0, 1200.5, 540],
          [0, 0, 1],
        ],
        distCoeffs: [-0.2, 0.05, 0, 0, 0],
        error: 0.45,
        timestamp: new Date().toISOString(),
      };

      setResult(mockResult);
      setStatus('complete');
      setProgress(100);
    } catch (e: any) {
      setStatus('error');
      setErrorMessage(e.message || 'Calibration failed');
      setProgress(0);
    }
  };

  // Handle save
  const handleSave = async () => {
    if (!result) return;

    try {
      setProgress(0);
      setStatus('processing');

      // Mock save to backend
      await new Promise((resolve) => setTimeout(resolve, 1500));

      setStatus('complete');
      setProgress(100);
      alert('Calibration saved successfully!');
    } catch (e: any) {
      setStatus('error');
      setErrorMessage(`Save failed: ${e.message}`);
    }
  };

  // Reset calibration
  const handleReset = () => {
    setStatus('idle');
    setProgress(0);
    setResult(null);
    setErrorMessage(null);
    setEstimatedTilt(0);
    setEstimatedHeight(0);
  };

  return (
    <div className="space-y-6">
      {/* System Type Badge */}
      <div className="bg-blue-900/30 border border-blue-700 rounded-lg p-4">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="text-lg font-semibold text-white">Manual Bed Calibration</h3>
            <p className="text-sm text-slate-400 mt-1">
              Fixed bed - Height and tilt adjusted by hand (no motors)
            </p>
          </div>
          <div className="text-right">
            <div className="text-2xl font-bold text-blue-400">Hand Adjusted</div>
            <p className="text-xs text-slate-500">Single Capture</p>
          </div>
        </div>
      </div>

      {/* Main Layout */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Camera Feed Section */}
        <div className="lg:col-span-2 space-y-4">
          {/* Video Stream */}
          <div className="bg-slate-900 border-2 border-slate-700 rounded-lg overflow-hidden">
            <video
              ref={videoRef}
              autoPlay
              playsInline
              className="w-full aspect-video bg-black"
            />
            <div className="hidden">
              <canvas ref={canvasRef} />
            </div>

            {/* Camera Status Overlay */}
            {!isCameraActive && (
              <div className="absolute inset-0 bg-black/70 flex items-center justify-center">
                <div className="text-center">
                  <Camera className="w-16 h-16 text-slate-500 mx-auto mb-4" />
                  <p className="text-slate-300">
                    {cameraError || 'Initializing camera...'}
                  </p>
                </div>
              </div>
            )}
          </div>

          {/* Camera Status */}
          <div className="flex items-center space-x-2">
            {isCameraActive ? (
              <>
                <CheckCircle2 className="w-5 h-5 text-industrial-success" />
                <span className="text-sm text-slate-300">Camera Ready</span>
              </>
            ) : (
              <>
                <AlertTriangle className="w-5 h-5 text-industrial-warning" />
                <span className="text-sm text-slate-300">Camera Initializing...</span>
              </>
            )}
          </div>

          {/* Error Message */}
          {errorMessage && (
            <div className="bg-industrial-danger/10 border border-industrial-danger/30 rounded-lg p-3 flex items-start space-x-3">
              <AlertTriangle className="w-5 h-5 text-industrial-danger flex-shrink-0 mt-0.5" />
              <p className="text-sm text-industrial-danger">{errorMessage}</p>
            </div>
          )}

          {/* Progress Bar */}
          {(status === 'capturing' || status === 'processing') && (
            <div>
              <div className="flex items-center justify-between mb-2">
                <span className="text-sm text-slate-300">Processing</span>
                <span className="text-sm font-semibold text-industrial-blue">{progress}%</span>
              </div>
              <div className="w-full bg-slate-700 h-2 rounded-full overflow-hidden">
                <div
                  className="bg-industrial-blue h-full transition-all duration-300"
                  style={{ width: `${progress}%` }}
                ></div>
              </div>
            </div>
          )}

          {/* Calibration Results */}
          {result && (
            <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
              <h4 className="font-semibold text-white mb-3 flex items-center">
                <CheckCircle2 className="w-5 h-5 mr-2 text-industrial-success" />
                Calibration Complete
              </h4>
              <div className="space-y-2 text-sm">
                <div className="flex justify-between">
                  <span className="text-slate-400">Reprojection Error:</span>
                  <span className="text-white font-mono">{result.error.toFixed(2)} px</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-400">Focal Length (fx):</span>
                  <span className="text-white font-mono">
                    {result.matrix[0][0].toFixed(1)}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-400">Principal Point (cx, cy):</span>
                  <span className="text-white font-mono">
                    ({result.matrix[0][2].toFixed(0)}, {result.matrix[1][2].toFixed(0)})
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-400">Distortion k1:</span>
                  <span className="text-white font-mono">{result.distCoeffs[0].toFixed(3)}</span>
                </div>
              </div>
            </div>
          )}
        </div>

        {/* Control Panel */}
        <div className="space-y-4">
          {/* Instructions */}
          <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
            <h4 className="font-semibold text-white mb-3 flex items-center">
              <HelpCircle className="w-5 h-5 mr-2 text-industrial-blue" />
              Setup Instructions
            </h4>
            <div className="space-y-3 text-sm text-slate-300">
              <div>
                <p className="font-medium text-white mb-1">1. Position Calibration Card</p>
                <p>Place checkerboard pattern flat on your fixed bed</p>
              </div>
              <div>
                <p className="font-medium text-white mb-1">2. Adjust Height (by hand)</p>
                <p>
                  Manually move camera up/down until checkerboard fills ~50% of frame
                </p>
              </div>
              <div>
                <p className="font-medium text-white mb-1">3. Adjust Tilt (by hand)</p>
                <p>Manually tilt camera so pattern is at slight angle (15-30°)</p>
              </div>
              <div>
                <p className="font-medium text-white mb-1">4. Capture & Calibrate</p>
                <p>Click button below to capture and process</p>
              </div>
            </div>
          </div>

          {/* Manual Adjustment Guide */}
          {showTilting && (
            <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
              <h4 className="font-semibold text-white mb-3 flex items-center">
                <Activity className="w-5 h-5 mr-2 text-industrial-warning" />
                Manual Hand Adjustments
              </h4>
              <p className="text-xs text-slate-400 mb-4">
                Use your hands to adjust the camera height and tilt. No motors - manual adjustment only.
              </p>
              <div className="space-y-4">
                {/* Height Adjustment */}
                <div className="bg-slate-700 rounded p-3">
                  <div className="flex items-center justify-between mb-3">
                    <span className="text-sm font-semibold text-white">Height (Hand Adjust)</span>
                    <span className="text-sm font-mono text-industrial-blue">
                      {estimatedHeight.toFixed(1)} mm
                    </span>
                  </div>
                  <div className="flex items-center space-x-2">
                    <button
                      onClick={() => setEstimatedHeight((h) => Math.max(0, h - 5))}
                      className="flex-1 px-3 py-2 bg-slate-600 hover:bg-slate-500 rounded text-white text-sm flex items-center justify-center space-x-2 transition-colors"
                      disabled
                    >
                      <ArrowDown className="w-4 h-4" />
                      <span>Lower (↓)</span>
                    </button>
                    <button
                      onClick={() => setEstimatedHeight((h) => h + 5)}
                      className="flex-1 px-3 py-2 bg-slate-600 hover:bg-slate-500 rounded text-white text-sm flex items-center justify-center space-x-2 transition-colors"
                      disabled
                    >
                      <ArrowUp className="w-4 h-4" />
                      <span>Raise (↑)</span>
                    </button>
                  </div>
                  <p className="text-xs text-slate-300 mt-2">👆 Manually adjust camera height until card fills 50% of frame</p>
                </div>

                {/* Tilt Adjustment */}
                <div className="bg-slate-700 rounded p-3">
                  <div className="flex items-center justify-between mb-3">
                    <span className="text-sm font-semibold text-white">Tilt Angle (Hand Adjust)</span>
                    <span className="text-sm font-mono text-industrial-blue">{estimatedTilt}°</span>
                  </div>
                  <div className="flex items-center space-x-2">
                    <button
                      onClick={() => setEstimatedTilt((t) => Math.max(-45, t - 5))}
                      className="flex-1 px-3 py-2 bg-slate-600 hover:bg-slate-500 rounded text-white text-sm flex items-center justify-center space-x-2 transition-colors"
                      disabled
                    >
                      <RotateCw className="w-4 h-4" />
                      <span>Tilt Left (↙)</span>
                    </button>
                    <button
                      onClick={() => setEstimatedTilt((t) => Math.min(45, t + 5))}
                      className="flex-1 px-3 py-2 bg-slate-600 hover:bg-slate-500 rounded text-white text-sm flex items-center justify-center space-x-2 transition-colors"
                      disabled
                    >
                      <RotateCcw className="w-4 h-4" />
                      <span>Tilt Right (↘)</span>
                    </button>
                  </div>
                  <p className="text-xs text-slate-300 mt-2">👆 Manually tilt camera until card is at slight angle (15-30°)</p>
                </div>
              </div>
            </div>
          )}

          {/* Action Buttons */}
          <div className="space-y-2">
            {status === 'idle' || status === 'error' ? (
              <>
                <button
                  onClick={handleCaptureAndCalibrate}
                  disabled={!isCameraActive}
                  className="w-full flex items-center justify-center space-x-2 px-4 py-3 bg-industrial-success hover:bg-industrial-success/90 disabled:bg-slate-600 disabled:cursor-not-allowed text-white rounded-lg font-semibold transition-colors"
                >
                  <Camera className="w-5 h-5" />
                  <span>Capture & Calibrate</span>
                </button>
                <button
                  onClick={() => setShowTilting(!showTilting)}
                  className="w-full px-4 py-2 bg-slate-700 hover:bg-slate-600 text-white rounded-lg text-sm font-medium transition-colors"
                >
                  {showTilting ? 'Hide' : 'Show'} Adjustment Guide
                </button>
              </>
            ) : status === 'complete' ? (
              <>
                <button
                  onClick={handleSave}
                  className="w-full flex items-center justify-center space-x-2 px-4 py-3 bg-industrial-blue hover:bg-industrial-blue/90 text-white rounded-lg font-semibold transition-colors"
                >
                  <Save className="w-5 h-5" />
                  <span>Save Calibration</span>
                </button>
                <button
                  onClick={handleReset}
                  className="w-full flex items-center justify-center space-x-2 px-4 py-2 bg-slate-700 hover:bg-slate-600 text-white rounded-lg font-medium transition-colors"
                >
                  <RotateCcw className="w-5 h-5" />
                  <span>New Calibration</span>
                </button>
              </>
            ) : (
              <button
                disabled
                className="w-full px-4 py-3 bg-slate-600 text-slate-300 rounded-lg font-semibold cursor-not-allowed"
              >
                {status === 'capturing' ? 'Capturing...' : 'Processing...'}
              </button>
            )}
          </div>

          {/* Comparison Info */}
          <div className="bg-slate-900 border border-slate-700 rounded-lg p-3">
            <p className="text-xs text-slate-400 mb-2 font-semibold">SYSTEM COMPARISON</p>
            <div className="space-y-2 text-xs text-slate-400">
              <div>
                <p className="text-slate-300 font-medium mb-1">vs. Auto Calibration</p>
                <ul className="list-disc list-inside space-y-1">
                  <li>Single capture vs. 15-20 captures</li>
                  <li>Manual positioning vs. automatic</li>
                  <li>Quick (1 min) vs. thorough (5-10 min)</li>
                  <li>Less accurate but good for quick setup</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ManualBedCalibration;

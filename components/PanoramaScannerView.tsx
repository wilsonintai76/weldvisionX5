import React, { useState, useEffect } from 'react';
import {
  Play,
  Square,
  Save,
  RefreshCw,
  AlertTriangle,
  CheckCircle2,
  Clock,
  Zap,
  Image,
  Settings as SettingsIcon,
  Activity,
  FileDown,
} from 'lucide-react';

interface PanoramaScanConfig {
  start_x_mm: number;
  end_x_mm: number;
  step_mm: number;
  scan_speed_mm_s: number;
}

interface ScanFrame {
  frame_num: number;
  position_x_mm: number;
  timestamp: string;
  image_path?: string;
}

type ScanStatus = 'idle' | 'scanning' | 'processing' | 'complete' | 'error';

const PanoramaScannerView: React.FC = () => {
  // State management
  const [status, setStatus] = useState<ScanStatus>('idle');
  const [progress, setProgress] = useState(0);
  const [isConnected, setIsConnected] = useState(true);
  const [scanConfig, setScanConfig] = useState<PanoramaScanConfig>({
    start_x_mm: 0,
    end_x_mm: 100,
    step_mm: 5,
    scan_speed_mm_s: 20,
  });
  const [editingConfig, setEditingConfig] = useState<PanoramaScanConfig>(scanConfig);
  const [showConfigEditor, setShowConfigEditor] = useState(false);
  const [frames, setFrames] = useState<ScanFrame[]>([]);
  const [panoramaResult, setPanoramaResult] = useState<string | null>(null);
  const [elapsedTime, setElapsedTime] = useState(0);
  const [estimatedTotal, setEstimatedTotal] = useState(0);
  const [errorMessage, setErrorMessage] = useState('');

  // Calculate estimated scan time
  useEffect(() => {
    const distance = Math.abs(editingConfig.end_x_mm - editingConfig.start_x_mm);
    const numFrames = Math.ceil(distance / editingConfig.step_mm) + 1;
    const estimatedSeconds = (distance / editingConfig.scan_speed_mm_s) + (numFrames * 0.5); // 0.5s per capture
    setEstimatedTotal(Math.round(estimatedSeconds));
  }, [editingConfig]);

  // Timer effect
  useEffect(() => {
    if (status === 'scanning' || status === 'processing') {
      const interval = setInterval(() => {
        setElapsedTime((prev) => prev + 1);
      }, 1000);
      return () => clearInterval(interval);
    }
  }, [status]);

  // Polling effect for scan progress (mock implementation)
  useEffect(() => {
    if (status === 'scanning') {
      const pollInterval = setInterval(async () => {
        // In real implementation, this would call the backend API
        setProgress((prev) => {
          if (prev >= 90) return prev;
          return prev + Math.random() * 15;
        });
        
        // Simulate frame capture
        if (Math.random() > 0.7) {
          setFrames((prev) => [
            ...prev,
            {
              frame_num: prev.length + 1,
              position_x_mm: scanConfig.start_x_mm + (prev.length * scanConfig.step_mm),
              timestamp: new Date().toISOString(),
            },
          ]);
        }
      }, 1000);
      return () => clearInterval(pollInterval);
    }
  }, [status, scanConfig]);

  // Start panorama scan
  const handleStartScan = async () => {
    setErrorMessage('');
    setFrames([]);
    setPanoramaResult(null);
    setProgress(0);
    setElapsedTime(0);
    setStatus('scanning');
    setScanConfig(editingConfig);

    try {
      // Mock API call - replace with real implementation
      await new Promise((resolve) => setTimeout(resolve, 100));
      
      // Simulate scanning for demo purposes
      let mockProgress = 0;
      const scanInterval = setInterval(() => {
        mockProgress += Math.random() * 12;
        if (mockProgress >= 100) {
          clearInterval(scanInterval);
          setProgress(100);
          setStatus('processing');
          
          // Simulate processing
          setTimeout(() => {
            setStatus('complete');
            setPanoramaResult('/placeholder-panorama.jpg');
          }, 2000);
        } else {
          setProgress(mockProgress);
        }
      }, 500);
    } catch (error) {
      setStatus('error');
      setErrorMessage(`Scan failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  // Stop current scan
  const handleStopScan = () => {
    setStatus('idle');
    setProgress(0);
    setElapsedTime(0);
    setFrames([]);
  };

  // Save panorama result
  const handleSavePanorama = async () => {
    try {
      // Mock save functionality
      console.log('Saving panorama with', frames.length, 'frames');
      alert(`Panorama saved! Stitched ${frames.length} frames into panoramic image.`);
    } catch (error) {
      setErrorMessage(`Save failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  // Export panorama
  const handleExportPanorama = async () => {
    try {
      console.log('Exporting panorama...');
      alert('Panorama exported to Downloads/panorama_scan.png');
    } catch (error) {
      setErrorMessage(`Export failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  };

  // Format time display
  const formatTime = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, '0')}`;
  };

  return (
    <div className="space-y-6">
      {/* Status Section */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
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

        {/* Scan Status */}
        <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Status</span>
            <Zap className="w-4 h-4 text-industrial-blue" />
          </div>
          <p className="text-lg font-semibold text-white capitalize">{status}</p>
        </div>

        {/* Elapsed Time */}
        <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Elapsed</span>
            <Clock className="w-4 h-4 text-industrial-blue" />
          </div>
          <p className="text-lg font-semibold text-white">{formatTime(elapsedTime)}</p>
        </div>

        {/* Frames Captured */}
        <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm text-slate-400">Frames</span>
            <Image className="w-4 h-4 text-industrial-blue" />
          </div>
          <p className="text-lg font-semibold text-white">{frames.length}</p>
        </div>
      </div>

      {/* Main Scan Area */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Scan Controls */}
        <div className="lg:col-span-2 bg-slate-800 border border-slate-700 rounded-lg p-6">
          <h3 className="text-lg font-semibold text-white mb-6 flex items-center">
            <Zap className="w-5 h-5 mr-2 text-industrial-blue" />
            Panorama Scan Control
          </h3>

          {/* Error Message */}
          {errorMessage && (
            <div className="mb-4 p-3 bg-industrial-danger/10 border border-industrial-danger/30 rounded-lg flex items-start space-x-3">
              <AlertTriangle className="w-5 h-5 text-industrial-danger flex-shrink-0 mt-0.5" />
              <p className="text-sm text-industrial-danger">{errorMessage}</p>
            </div>
          )}

          {/* Progress Bar */}
          {(status === 'scanning' || status === 'processing') && (
            <div className="mb-6">
              <div className="flex items-center justify-between mb-2">
                <span className="text-sm text-slate-300">Progress</span>
                <span className="text-sm font-semibold text-industrial-blue">
                  {Math.round(progress)}%
                </span>
              </div>
              <div className="w-full bg-slate-700 rounded-full h-3 overflow-hidden">
                <div
                  className="bg-gradient-to-r from-industrial-blue to-industrial-success h-full rounded-full transition-all duration-300"
                  style={{ width: `${progress}%` }}
                ></div>
              </div>
              <div className="mt-2 text-xs text-slate-400">
                {status === 'scanning' && `Scanning... ${formatTime(elapsedTime)} / ~${formatTime(estimatedTotal)}`}
                {status === 'processing' && `Processing panorama...`}
              </div>
            </div>
          )}

          {/* Scan Configuration Display */}
          {!showConfigEditor && (
            <div className="bg-slate-900 rounded-lg p-4 mb-6">
              <div className="grid grid-cols-2 gap-4 text-sm">
                <div>
                  <p className="text-slate-400">Start Position</p>
                  <p className="font-semibold text-white">{scanConfig.start_x_mm} mm</p>
                </div>
                <div>
                  <p className="text-slate-400">End Position</p>
                  <p className="font-semibold text-white">{scanConfig.end_x_mm} mm</p>
                </div>
                <div>
                  <p className="text-slate-400">Step Size</p>
                  <p className="font-semibold text-white">{scanConfig.step_mm} mm</p>
                </div>
                <div>
                  <p className="text-slate-400">Scan Speed</p>
                  <p className="font-semibold text-white">{scanConfig.scan_speed_mm_s} mm/s</p>
                </div>
              </div>
            </div>
          )}

          {/* Scan Configuration Editor */}
          {showConfigEditor && (
            <div className="bg-slate-900 rounded-lg p-4 mb-6 space-y-4">
              <div>
                <label className="block text-sm text-slate-300 mb-2">Start Position (mm)</label>
                <input
                  type="number"
                  value={editingConfig.start_x_mm}
                  onChange={(e) =>
                    setEditingConfig({ ...editingConfig, start_x_mm: parseFloat(e.target.value) })
                  }
                  className="w-full px-3 py-2 bg-slate-800 border border-slate-600 rounded text-white placeholder-slate-500 focus:border-industrial-blue focus:outline-none"
                  min="-500"
                  max="500"
                />
              </div>
              <div>
                <label className="block text-sm text-slate-300 mb-2">End Position (mm)</label>
                <input
                  type="number"
                  value={editingConfig.end_x_mm}
                  onChange={(e) =>
                    setEditingConfig({ ...editingConfig, end_x_mm: parseFloat(e.target.value) })
                  }
                  className="w-full px-3 py-2 bg-slate-800 border border-slate-600 rounded text-white placeholder-slate-500 focus:border-industrial-blue focus:outline-none"
                  min="-500"
                  max="500"
                />
              </div>
              <div>
                <label className="block text-sm text-slate-300 mb-2">Step Size (mm)</label>
                <input
                  type="number"
                  value={editingConfig.step_mm}
                  onChange={(e) =>
                    setEditingConfig({ ...editingConfig, step_mm: parseFloat(e.target.value) })
                  }
                  className="w-full px-3 py-2 bg-slate-800 border border-slate-600 rounded text-white placeholder-slate-500 focus:border-industrial-blue focus:outline-none"
                  min="0.1"
                  max="50"
                  step="0.1"
                />
              </div>
              <div>
                <label className="block text-sm text-slate-300 mb-2">Scan Speed (mm/s)</label>
                <input
                  type="number"
                  value={editingConfig.scan_speed_mm_s}
                  onChange={(e) =>
                    setEditingConfig({
                      ...editingConfig,
                      scan_speed_mm_s: parseFloat(e.target.value),
                    })
                  }
                  className="w-full px-3 py-2 bg-slate-800 border border-slate-600 rounded text-white placeholder-slate-500 focus:border-industrial-blue focus:outline-none"
                  min="1"
                  max="100"
                />
              </div>
              <div className="flex space-x-2">
                <button
                  onClick={() => setShowConfigEditor(false)}
                  className="flex-1 px-4 py-2 bg-industrial-blue text-white rounded font-medium hover:bg-industrial-blue/90 transition-colors"
                >
                  Done
                </button>
                <button
                  onClick={() => setEditingConfig(scanConfig)}
                  className="flex-1 px-4 py-2 bg-slate-700 text-white rounded font-medium hover:bg-slate-600 transition-colors"
                >
                  Cancel
                </button>
              </div>
            </div>
          )}

          {/* Action Buttons */}
          <div className="flex flex-wrap gap-3">
            {status === 'idle' || status === 'complete' || status === 'error' ? (
              <>
                <button
                  onClick={handleStartScan}
                  disabled={!isConnected || status === 'error'}
                  className="flex items-center space-x-2 px-6 py-3 bg-industrial-success hover:bg-industrial-success/90 disabled:bg-slate-600 disabled:cursor-not-allowed text-white rounded-lg font-medium transition-colors"
                >
                  <Play className="w-4 h-4" />
                  <span>Start Scan</span>
                </button>
                <button
                  onClick={() => setShowConfigEditor(!showConfigEditor)}
                  className="flex items-center space-x-2 px-6 py-3 bg-slate-700 hover:bg-slate-600 text-white rounded-lg font-medium transition-colors"
                >
                  <SettingsIcon className="w-4 h-4" />
                  <span>{showConfigEditor ? 'Hide' : 'Configure'}</span>
                </button>
              </>
            ) : (
              <button
                onClick={handleStopScan}
                className="flex items-center space-x-2 px-6 py-3 bg-industrial-danger hover:bg-industrial-danger/90 text-white rounded-lg font-medium transition-colors"
              >
                <Square className="w-4 h-4" />
                <span>Stop Scan</span>
              </button>
            )}
            {status === 'complete' && (
              <>
                <button
                  onClick={handleSavePanorama}
                  className="flex items-center space-x-2 px-6 py-3 bg-industrial-blue hover:bg-industrial-blue/90 text-white rounded-lg font-medium transition-colors"
                >
                  <Save className="w-4 h-4" />
                  <span>Save</span>
                </button>
                <button
                  onClick={handleExportPanorama}
                  className="flex items-center space-x-2 px-6 py-3 bg-slate-700 hover:bg-slate-600 text-white rounded-lg font-medium transition-colors"
                >
                  <FileDown className="w-4 h-4" />
                  <span>Export</span>
                </button>
              </>
            )}
          </div>
        </div>

        {/* Info Panel */}
        <div className="space-y-4">
          {/* Scan Parameters Summary */}
          <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
            <h4 className="text-sm font-semibold text-white mb-3 flex items-center">
              <Activity className="w-4 h-4 mr-2 text-industrial-blue" />
              Scan Parameters
            </h4>
            <div className="space-y-2 text-sm">
              <div className="flex justify-between">
                <span className="text-slate-400">Distance:</span>
                <span className="text-white font-medium">
                  {Math.abs(editingConfig.end_x_mm - editingConfig.start_x_mm)} mm
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Est. Frames:</span>
                <span className="text-white font-medium">
                  {Math.ceil(Math.abs(editingConfig.end_x_mm - editingConfig.start_x_mm) / editingConfig.step_mm) + 1}
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Est. Time:</span>
                <span className="text-white font-medium">{formatTime(estimatedTotal)}</span>
              </div>
            </div>
          </div>

          {/* Captured Frames Preview */}
          <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
            <h4 className="text-sm font-semibold text-white mb-3">Captured Frames</h4>
            {frames.length > 0 ? (
              <div className="space-y-2 max-h-48 overflow-y-auto">
                {frames.map((frame) => (
                  <div key={frame.frame_num} className="bg-slate-900 p-2 rounded text-xs">
                    <div className="flex justify-between">
                      <span className="text-slate-300">Frame {frame.frame_num}</span>
                      <span className="text-slate-400">{frame.position_x_mm} mm</span>
                    </div>
                  </div>
                ))}
              </div>
            ) : (
              <p className="text-sm text-slate-400">No frames captured yet</p>
            )}
          </div>

          {/* Panorama Result Preview */}
          {panoramaResult && (
            <div className="bg-slate-800 border border-slate-700 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-white mb-3">Panorama Result</h4>
              <div className="bg-slate-900 rounded aspect-video flex items-center justify-center border border-slate-700">
                <div className="text-center">
                  <Image className="w-8 h-8 text-slate-500 mx-auto mb-2" />
                  <p className="text-xs text-slate-400">Panorama Preview</p>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default PanoramaScannerView;

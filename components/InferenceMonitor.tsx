import React, { useEffect, useState } from 'react';
import { Activity, AlertOctagon, CheckCircle2, RefreshCw, Wifi, WifiOff } from 'lucide-react';

interface InferenceResult {
  class: 'good' | 'porosity' | 'undercut' | 'spatter' | 'unknown';
  confidence: number;
  source: 'rule-based' | 'ml-model' | 'ensemble';
  ml_score?: number;
  rule_score?: number;
  timestamp: string;
}

interface InferenceMonitorProps {
  latestResult: InferenceResult | null;
  isRunning: boolean;
  onStart: () => void;
  onStop: () => void;
  desktopConnected?: boolean;
  onToggleDesktop?: (enable: boolean) => void;
}

export const InferenceMonitor: React.FC<InferenceMonitorProps> = ({
  latestResult,
  isRunning,
  onStart,
  onStop,
  desktopConnected = false,
  onToggleDesktop
}) => {
  const [cpuUsage, setCpuUsage] = useState(15);

  // Simulate CPU usage monitoring
  useEffect(() => {
    const interval = setInterval(() => {
      setCpuUsage(Math.random() * 10 + 10); // 10-20%
    }, 2000);
    return () => clearInterval(interval);
  }, []);

  const getConfidenceColor = (confidence: number) => {
    if (confidence > 0.8) return 'text-green-400';
    if (confidence > 0.6) return 'text-yellow-400';
    return 'text-red-400';
  };

  const getClassIcon = (weldClass: string) => {
    switch (weldClass) {
      case 'good':
        return <CheckCircle2 className="w-5 h-5 text-green-400" />;
      case 'porosity':
      case 'undercut':
      case 'spatter':
        return <AlertOctagon className="w-5 h-5 text-red-400" />;
      default:
        return <Activity className="w-5 h-5 text-gray-400" />;
    }
  };

  return (
    <div className="bg-slate-900 rounded-xl border border-slate-800 p-6 space-y-6">
      <div className="flex items-center justify-between">
        <h2 className="text-xl font-bold text-white">Hybrid Inference (RDK)</h2>
        <div className="flex items-center gap-2">
          {desktopConnected ? (
            <div className="flex items-center gap-2 px-3 py-1 bg-green-900/50 border border-green-700 rounded-lg">
              <Wifi className="w-4 h-4 text-green-400" />
              <span className="text-xs font-medium text-green-300">Desktop Connected</span>
            </div>
          ) : (
            <div className="flex items-center gap-2 px-3 py-1 bg-slate-800 border border-slate-700 rounded-lg">
              <WifiOff className="w-4 h-4 text-slate-400" />
              <span className="text-xs font-medium text-slate-400">Standalone</span>
            </div>
          )}
        </div>
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-2 gap-3">
        <button
          onClick={onStart}
          disabled={isRunning}
          className="flex items-center justify-center gap-2 bg-industrial-blue hover:bg-industrial-blue/80 disabled:opacity-50 disabled:cursor-not-allowed text-white py-2 rounded-lg font-medium transition-colors"
        >
          <Activity className="w-4 h-4" />
          Start Inference
        </button>
        <button
          onClick={onStop}
          disabled={!isRunning}
          className="flex items-center justify-center gap-2 bg-red-600 hover:bg-red-700 disabled:opacity-50 disabled:cursor-not-allowed text-white py-2 rounded-lg font-medium transition-colors"
        >
          <RefreshCw className="w-4 h-4" />
          Stop Inference
        </button>
      </div>

      {/* Desktop Connection Toggle */}
      <div className="border border-slate-700 rounded-lg p-4 bg-slate-800">
        <div className="flex items-center justify-between mb-3">
          <h3 className="text-sm font-semibold text-slate-300">Desktop Refinement</h3>
          <button
            onClick={() => onToggleDesktop?.(!desktopConnected)}
            className={`px-3 py-1 rounded-lg text-xs font-medium transition-colors ${
              desktopConnected
                ? 'bg-green-900/50 text-green-300 border border-green-700'
                : 'bg-slate-700 text-slate-300 border border-slate-600'
            }`}
          >
            {desktopConnected ? 'Enabled' : 'Disabled'}
          </button>
        </div>
        <p className="text-xs text-slate-400">
          {desktopConnected
            ? 'Uncertain predictions will be sent to desktop for refinement'
            : 'Running in standalone mode (75-80% accuracy)'}
        </p>
      </div>

      {/* Status Indicator */}
      <div
        className={`flex items-center gap-3 p-4 rounded-lg border transition-colors ${
          isRunning
            ? 'bg-green-900/20 border-green-700'
            : 'bg-slate-800 border-slate-700'
        }`}
      >
        <Activity
          className={`w-5 h-5 ${isRunning ? 'text-green-400 animate-pulse' : 'text-slate-400'}`}
        />
        <div>
          <p className="text-sm font-medium text-slate-200">
            {isRunning ? 'Inference Running...' : 'Inference Idle'}
          </p>
          <p className="text-xs text-slate-400 mt-1">
            {isRunning ? 'Processing frames in real-time' : 'Ready to start scanning'}
          </p>
        </div>
      </div>

      {/* Latest Result */}
      {latestResult && (
        <div className="border border-slate-700 rounded-lg p-4 bg-slate-800 space-y-3">
          <h3 className="text-sm font-semibold text-slate-300">Latest Prediction</h3>

          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-sm text-slate-400">Class:</span>
              <div className="flex items-center gap-2">
                {getClassIcon(latestResult.class)}
                <span className="text-white font-medium capitalize">{latestResult.class}</span>
              </div>
            </div>

            <div className="flex items-center justify-between">
              <span className="text-sm text-slate-400">Confidence:</span>
              <span className={`font-medium text-sm ${getConfidenceColor(latestResult.confidence)}`}>
                {(latestResult.confidence * 100).toFixed(1)}%
              </span>
            </div>

            <div className="flex items-center justify-between">
              <span className="text-sm text-slate-400">Source:</span>
              <span className="text-blue-400 font-medium text-sm">{latestResult.source}</span>
            </div>

            <div className="flex items-center justify-between">
              <span className="text-sm text-slate-400">Timestamp:</span>
              <span className="text-slate-300 text-xs">
                {new Date(latestResult.timestamp).toLocaleTimeString()}
              </span>
            </div>
          </div>

          {/* Score Breakdown */}
          {latestResult.ml_score !== undefined && (
            <div className="mt-4 pt-4 border-t border-slate-700">
              <h4 className="text-xs font-semibold text-slate-300 mb-2">Score Breakdown</h4>
              <div className="space-y-2 text-xs text-slate-400">
                <div className="flex justify-between">
                  <span>ML Model Score:</span>
                  <span className="text-blue-400">{(latestResult.ml_score * 100).toFixed(1)}%</span>
                </div>
                <div className="flex justify-between">
                  <span>Rule-Based Score:</span>
                  <span className="text-purple-400">
                    {((latestResult.rule_score || 0) * 100).toFixed(1)}%
                  </span>
                </div>
              </div>
            </div>
          )}
        </div>
      )}

      {/* CPU Usage Monitor */}
      <div className="border border-slate-700 rounded-lg p-4 bg-slate-800">
        <div className="flex items-center justify-between mb-2">
          <h3 className="text-sm font-semibold text-slate-300">RDK Resources</h3>
          <span className={`text-sm font-medium ${cpuUsage > 25 ? 'text-red-400' : 'text-green-400'}`}>
            {cpuUsage.toFixed(1)}%
          </span>
        </div>

        {/* CPU Usage Bar */}
        <div className="w-full bg-slate-700 rounded-full h-2 overflow-hidden mb-3">
          <div
            className={`h-full transition-all ${
              cpuUsage > 25
                ? 'bg-red-600'
                : cpuUsage > 15
                  ? 'bg-yellow-600'
                  : 'bg-green-600'
            }`}
            style={{ width: `${Math.min(cpuUsage, 100)}%` }}
          ></div>
        </div>

        <div className="grid grid-cols-3 gap-2 text-xs">
          <div className="bg-slate-700 rounded p-2">
            <p className="text-slate-400">Memory</p>
            <p className="text-white font-medium">~200MB</p>
          </div>
          <div className="bg-slate-700 rounded p-2">
            <p className="text-slate-400">FPS</p>
            <p className="text-white font-medium">~10</p>
          </div>
          <div className="bg-slate-700 rounded p-2">
            <p className="text-slate-400">Latency</p>
            <p className="text-white font-medium">~100ms</p>
          </div>
        </div>
      </div>

      {/* Performance Metrics */}
      <div className="grid grid-cols-2 gap-3 text-xs text-slate-400">
        <div className="bg-slate-800 rounded p-3 border border-slate-700">
          <p className="text-slate-500 mb-1">Inference Time</p>
          <p className="text-white font-medium">100-150ms</p>
        </div>
        <div className="bg-slate-800 rounded p-3 border border-slate-700">
          <p className="text-slate-500 mb-1">Model Size</p>
          <p className="text-white font-medium">18 MB</p>
        </div>
      </div>
    </div>
  );
};

export default InferenceMonitor;

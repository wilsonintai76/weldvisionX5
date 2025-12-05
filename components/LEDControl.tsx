import React, { useState, useEffect } from 'react';
import { Lightbulb, AlertCircle, Check, Thermometer, Zap } from 'lucide-react';

interface LEDStatus {
  is_on: boolean;
  current_intensity: number;
  target_intensity: number;
  mode: string;
  temperature: number;
  temperature_warning: boolean;
  sensor_available: boolean;
  light_level_lux: number | null;
}

interface LEDControlProps {
  onStatusChange?: (status: LEDStatus) => void;
}

export const LEDControl: React.FC<LEDControlProps> = ({ onStatusChange }) => {
  const [status, setStatus] = useState<LEDStatus | null>(null);
  const [intensity, setIntensity] = useState(75);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [presets] = useState({
    off: 0,
    dim: 25,
    medium: 50,
    bright: 75,
    max: 100
  });

  // Fetch LED status on mount and periodically
  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 2000);
    return () => clearInterval(interval);
  }, []);

  // Notify parent of status changes
  useEffect(() => {
    if (status && onStatusChange) {
      onStatusChange(status);
    }
  }, [status, onStatusChange]);

  const fetchStatus = async () => {
    try {
      const response = await fetch('http://localhost:5000/api/led/status');
      if (response.ok) {
        const data = await response.json();
        setStatus(data.data);
        setIntensity(data.data.current_intensity);
        setError(null);
      }
    } catch (err) {
      setError('Failed to fetch LED status');
    }
  };

  const handleTurnOn = async (preset?: number) => {
    setLoading(true);
    try {
      const intensityValue = preset ?? intensity;
      const response = await fetch('http://localhost:5000/api/led/on', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ intensity: intensityValue })
      });
      
      if (response.ok) {
        const data = await response.json();
        setStatus(data.data);
        setIntensity(data.data.current_intensity);
        setError(null);
      } else {
        setError('Failed to turn on LED');
      }
    } catch (err) {
      setError('Failed to turn on LED');
    } finally {
      setLoading(false);
    }
  };

  const handleTurnOff = async () => {
    setLoading(true);
    try {
      const response = await fetch('http://localhost:5000/api/led/off', {
        method: 'POST'
      });
      
      if (response.ok) {
        const data = await response.json();
        setStatus(data.data);
        setIntensity(0);
        setError(null);
      } else {
        setError('Failed to turn off LED');
      }
    } catch (err) {
      setError('Failed to turn off LED');
    } finally {
      setLoading(false);
    }
  };

  const handleIntensityChange = async (newIntensity: number) => {
    setIntensity(newIntensity);
    setLoading(true);
    try {
      const response = await fetch('http://localhost:5000/api/led/intensity', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ intensity: newIntensity })
      });
      
      if (response.ok) {
        const data = await response.json();
        setStatus(data.data);
        setError(null);
      } else {
        setError('Failed to set intensity');
      }
    } catch (err) {
      setError('Failed to set intensity');
    } finally {
      setLoading(false);
    }
  };

  const handlePreset = async (presetName: string) => {
    setLoading(true);
    try {
      const response = await fetch('http://localhost:5000/api/led/preset', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preset: presetName })
      });
      
      if (response.ok) {
        const data = await response.json();
        setStatus(data.data);
        setIntensity(data.data.current_intensity);
        setError(null);
      } else {
        setError(`Failed to set preset ${presetName}`);
      }
    } catch (err) {
      setError(`Failed to set preset ${presetName}`);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="bg-slate-800 rounded-2xl p-6 border border-slate-700 shadow-xl">
      {/* Header */}
      <div className="flex items-center justify-between mb-6">
        <div className="flex items-center gap-3">
          <Lightbulb className="w-6 h-6 text-industrial-orange" />
          <h3 className="text-lg font-semibold text-white">LED Ring Light Control</h3>
        </div>
        {status?.is_on ? (
          <div className="flex items-center gap-2 px-3 py-1 rounded-full bg-industrial-orange/20 border border-industrial-orange/50">
            <div className="w-2 h-2 rounded-full bg-industrial-orange animate-pulse"></div>
            <span className="text-sm font-medium text-industrial-orange">Active</span>
          </div>
        ) : (
          <div className="flex items-center gap-2 px-3 py-1 rounded-full bg-slate-700 border border-slate-600">
            <div className="w-2 h-2 rounded-full bg-slate-500"></div>
            <span className="text-sm font-medium text-slate-400">Off</span>
          </div>
        )}
      </div>

      {/* Error Message */}
      {error && (
        <div className="bg-red-900/20 border border-red-700/50 rounded-lg p-3 mb-4 flex gap-2">
          <AlertCircle className="w-5 h-5 text-red-600 flex-shrink-0" />
          <span className="text-sm text-red-300">{error}</span>
        </div>
      )}

      {/* Status Information */}
      {status && (
        <div className="grid grid-cols-2 gap-3 mb-6">
          <div className="bg-slate-700 rounded p-3">
            <p className="text-xs text-slate-400 mb-1">Current Intensity</p>
            <p className="text-2xl font-bold text-white">{status.current_intensity}%</p>
          </div>
          <div className="bg-slate-700 rounded p-3">
            <div className="flex items-center gap-2 mb-1">
              <Thermometer className="w-4 h-4 text-slate-400" />
              <p className="text-xs text-slate-400">Temperature</p>
            </div>
            <p className={`text-2xl font-bold ${
              status.temperature_warning ? 'text-yellow-400' : 'text-white'
            }`}>
              {status.temperature.toFixed(1)}°C
            </p>
          </div>
          {status.sensor_available && status.light_level_lux !== null && (
            <div className="col-span-2 bg-slate-700 rounded p-3">
              <div className="flex items-center gap-2 mb-1">
                <Zap className="w-4 h-4 text-slate-400" />
                <p className="text-xs text-slate-400">Ambient Light Level</p>
              </div>
              <p className="text-lg font-bold text-white">{status.light_level_lux.toFixed(0)} lux</p>
            </div>
          )}
          {status.mode !== 'simulator' && (
            <div className="col-span-2 bg-slate-700 rounded p-2 px-3">
              <p className="text-xs text-slate-400">Mode: <span className="text-slate-200 font-medium capitalize">{status.mode}</span></p>
            </div>
          )}
        </div>
      )}

      {/* Intensity Slider */}
      <div className="mb-6">
        <label className="block text-sm font-medium text-slate-300 mb-3">
          Brightness Control
        </label>
        <input
          type="range"
          min="0"
          max="100"
          value={intensity}
          onChange={(e) => handleIntensityChange(parseInt(e.target.value))}
          disabled={loading}
          title="LED brightness slider from 0 to 100 percent"
          className="w-full h-2 bg-slate-700 rounded-lg appearance-none cursor-pointer accent-industrial-blue"
        />
        <div className="flex justify-between text-xs text-slate-400 mt-2">
          <span>Off</span>
          <span>Min</span>
          <span>Max</span>
        </div>
      </div>

      {/* Preset Buttons */}
      <div className="mb-6">
        <label className="block text-sm font-medium text-slate-300 mb-3">
          Brightness Presets
        </label>
        <div className="grid grid-cols-5 gap-2">
          {Object.entries(presets).map(([name, value]) => (
            <button
              key={name}
              onClick={() => handlePreset(name)}
              disabled={loading}
              className={`py-2 px-2 rounded-lg text-sm font-medium transition-colors ${
                status?.current_intensity === value
                  ? 'bg-industrial-blue text-white shadow-lg shadow-industrial-blue/20'
                  : 'bg-slate-700 text-slate-300 hover:bg-slate-600'
              } disabled:opacity-50 disabled:cursor-not-allowed`}
            >
              <span className="capitalize hidden sm:inline">{name}</span>
              <span className="sm:hidden">{value}%</span>
            </button>
          ))}
        </div>
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-2 gap-3">
        <button
          onClick={() => handleTurnOn()}
          disabled={loading || status?.is_on}
          className="flex items-center justify-center gap-2 py-3 px-4 bg-industrial-orange hover:bg-industrial-orange/90 text-white font-medium rounded-lg transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
        >
          <Lightbulb className="w-5 h-5" />
          <span>Turn On</span>
        </button>
        <button
          onClick={handleTurnOff}
          disabled={loading || !status?.is_on}
          className="flex items-center justify-center gap-2 py-3 px-4 bg-slate-700 hover:bg-slate-600 text-slate-300 font-medium rounded-lg transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
        >
          <Lightbulb className="w-5 h-5" />
          <span>Turn Off</span>
        </button>
      </div>

      {/* Info Box */}
      <div className="mt-6 bg-blue-900/20 border border-blue-700/50 rounded-lg p-4">
        <p className="text-sm text-blue-300">
          <span className="font-semibold">💡 Tip:</span> Use presets for common brightness levels, or adjust the slider for fine control.
          {!status?.sensor_available && ' Light sensor not detected - brightness is software-controlled only.'}
        </p>
      </div>
    </div>
  );
};

export default LEDControl;

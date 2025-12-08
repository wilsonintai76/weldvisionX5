import React, { useState, useEffect } from 'react';
import { Play, Pause, Download, Trash2, RefreshCw, TrendingUp } from 'lucide-react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

interface TrainingJob {
  id: string;
  model: string;
  status: 'running' | 'completed' | 'failed' | 'cancelled';
  epoch: number;
  total_epochs: number;
  loss: number;
  accuracy: number;
  progress: number;
  started_at: string;
  completed_at?: string;
}

interface TrainingDashboardProps {
  onStartTraining?: (model: string, config: any) => void;
  onCancelTraining?: (jobId: string) => void;
}

export const TrainingDashboard: React.FC<TrainingDashboardProps> = ({
  onStartTraining,
  onCancelTraining
}) => {
  const [activeJobs, setActiveJobs] = useState<TrainingJob[]>([]);
  const [selectedModel, setSelectedModel] = useState('mobilenet');
  const [trainingConfig, setTrainingConfig] = useState({
    epochs: 100,
    batch_size: 32,
    learning_rate: 0.001
  });
  const [isStarting, setIsStarting] = useState(false);
  const [trainingHistory, setTrainingHistory] = useState<any[]>([]);

  const handleStartTraining = async () => {
    setIsStarting(true);
    try {
      onStartTraining?.(selectedModel, trainingConfig);
      
      // Simulate job creation
      const newJob: TrainingJob = {
        id: `job_${Date.now()}`,
        model: selectedModel,
        status: 'running',
        epoch: 0,
        total_epochs: trainingConfig.epochs,
        loss: 0,
        accuracy: 0,
        progress: 0,
        started_at: new Date().toISOString()
      };
      setActiveJobs([...activeJobs, newJob]);
    } catch (error) {
      console.error('Failed to start training:', error);
    } finally {
      setIsStarting(false);
    }
  };

  const handleCancelJob = (jobId: string) => {
    onCancelTraining?.(jobId);
    setActiveJobs(activeJobs.map(job =>
      job.id === jobId ? { ...job, status: 'cancelled' as const } : job
    ));
  };

  return (
    <div className="space-y-6">
      {/* Training Configuration */}
      <div className="bg-slate-900 rounded-xl border border-slate-800 p-6">
        <h2 className="text-xl font-bold text-white mb-6">Model Training</h2>

        <div className="space-y-4">
          {/* Model Selection */}
          <div>
            <label className="block text-sm font-medium text-slate-300 mb-2">Select Model</label>
            <div className="grid grid-cols-2 gap-3">
              {['mobilenet', 'resnet50'].map(model => (
                <button
                  key={model}
                  onClick={() => setSelectedModel(model)}
                  className={`p-3 rounded-lg border transition-colors ${
                    selectedModel === model
                      ? 'bg-industrial-blue border-industrial-blue text-white'
                      : 'bg-slate-800 border-slate-700 text-slate-300 hover:border-slate-600'
                  }`}
                >
                  <p className="font-medium capitalize">{model}</p>
                  <p className="text-xs mt-1">
                    {model === 'mobilenet' ? '18MB - Edge (75-80%)' : '100MB - Desktop (90-95%)'}
                  </p>
                </button>
              ))}
            </div>
          </div>

          {/* Training Parameters */}
          <div className="grid grid-cols-3 gap-3">
            <div>
              <label className="block text-sm font-medium text-slate-300 mb-2">Epochs</label>
              <input
                type="number"
                value={trainingConfig.epochs}
                onChange={e =>
                  setTrainingConfig({ ...trainingConfig, epochs: parseInt(e.target.value) })
                }
                min="10"
                max="500"
                className="w-full bg-slate-800 border border-slate-700 text-white rounded-lg px-3 py-2 text-sm focus:outline-none focus:border-industrial-blue"
              />
            </div>
            <div>
              <label className="block text-sm font-medium text-slate-300 mb-2">Batch Size</label>
              <input
                type="number"
                value={trainingConfig.batch_size}
                onChange={e =>
                  setTrainingConfig({ ...trainingConfig, batch_size: parseInt(e.target.value) })
                }
                min="8"
                max="128"
                className="w-full bg-slate-800 border border-slate-700 text-white rounded-lg px-3 py-2 text-sm focus:outline-none focus:border-industrial-blue"
              />
            </div>
            <div>
              <label className="block text-sm font-medium text-slate-300 mb-2">Learning Rate</label>
              <input
                type="number"
                value={trainingConfig.learning_rate}
                onChange={e =>
                  setTrainingConfig({
                    ...trainingConfig,
                    learning_rate: parseFloat(e.target.value)
                  })
                }
                step="0.0001"
                min="0.00001"
                max="0.1"
                className="w-full bg-slate-800 border border-slate-700 text-white rounded-lg px-3 py-2 text-sm focus:outline-none focus:border-industrial-blue"
              />
            </div>
          </div>

          {/* Start Button */}
          <button
            onClick={handleStartTraining}
            disabled={isStarting || activeJobs.some(j => j.status === 'running')}
            className="w-full flex items-center justify-center gap-2 bg-industrial-blue hover:bg-industrial-blue/80 disabled:opacity-50 disabled:cursor-not-allowed text-white py-3 rounded-lg font-medium transition-colors"
          >
            <Play className="w-4 h-4" />
            Start Training
          </button>
        </div>
      </div>

      {/* Active Training Jobs */}
      {activeJobs.length > 0 && (
        <div className="space-y-3">
          {activeJobs.map(job => (
            <div key={job.id} className="bg-slate-900 rounded-xl border border-slate-800 p-6">
              <div className="flex items-start justify-between mb-4">
                <div>
                  <h3 className="text-lg font-semibold text-white capitalize">{job.model}</h3>
                  <p className="text-sm text-slate-400 mt-1">
                    Job ID: {job.id.slice(0, 20)}...
                  </p>
                </div>
                <div className="flex items-center gap-2">
                  <span className={`px-3 py-1 rounded-lg text-xs font-medium ${
                    job.status === 'running'
                      ? 'bg-blue-900/50 text-blue-300'
                      : job.status === 'completed'
                        ? 'bg-green-900/50 text-green-300'
                        : 'bg-red-900/50 text-red-300'
                  }`}>
                    {job.status.toUpperCase()}
                  </span>
                  {job.status === 'running' && (
                    <button
                      onClick={() => handleCancelJob(job.id)}
                      className="p-2 hover:bg-slate-800 rounded-lg transition-colors"
                    >
                      <Pause className="w-4 h-4 text-slate-400" />
                    </button>
                  )}
                </div>
              </div>

              {/* Progress Bar */}
              <div className="mb-4">
                <div className="flex justify-between text-sm mb-2">
                  <span className="text-slate-400">Progress</span>
                  <span className="text-white font-medium">{job.progress}%</span>
                </div>
                <div className="w-full bg-slate-800 rounded-full h-2 overflow-hidden">
                  <div
                    className="h-full bg-industrial-blue transition-all"
                    style={{ width: `${job.progress}%` }}
                  ></div>
                </div>
              </div>

              {/* Training Stats */}
              <div className="grid grid-cols-4 gap-2 text-sm mb-4">
                <div className="bg-slate-800 rounded p-2">
                  <p className="text-slate-400 text-xs">Epoch</p>
                  <p className="text-white font-medium">
                    {job.epoch}/{job.total_epochs}
                  </p>
                </div>
                <div className="bg-slate-800 rounded p-2">
                  <p className="text-slate-400 text-xs">Loss</p>
                  <p className="text-white font-medium">{job.loss.toFixed(4)}</p>
                </div>
                <div className="bg-slate-800 rounded p-2">
                  <p className="text-slate-400 text-xs">Accuracy</p>
                  <p className="text-white font-medium">{(job.accuracy * 100).toFixed(1)}%</p>
                </div>
                <div className="bg-slate-800 rounded p-2">
                  <p className="text-slate-400 text-xs">ETA</p>
                  <p className="text-white font-medium">~2h 30m</p>
                </div>
              </div>

              {/* Action Buttons */}
              {job.status === 'completed' && (
                <div className="flex gap-2">
                  <button className="flex-1 flex items-center justify-center gap-2 bg-industrial-blue hover:bg-industrial-blue/80 text-white py-2 rounded-lg text-sm font-medium transition-colors">
                    <Download className="w-4 h-4" />
                    Download Model
                  </button>
                  <button className="flex-1 flex items-center justify-center gap-2 bg-slate-800 hover:bg-slate-700 text-white py-2 rounded-lg text-sm font-medium transition-colors">
                    <TrendingUp className="w-4 h-4" />
                    View Metrics
                  </button>
                </div>
              )}
            </div>
          ))}
        </div>
      )}

      {/* Training History Chart */}
      {trainingHistory.length > 0 && (
        <div className="bg-slate-900 rounded-xl border border-slate-800 p-6">
          <h3 className="text-lg font-semibold text-white mb-4">Training Progress</h3>
          <ResponsiveContainer width="100%" height={300}>
            <LineChart data={trainingHistory}>
              <CartesianGrid strokeDasharray="3 3" stroke="#334155" />
              <XAxis dataKey="epoch" stroke="#94a3b8" />
              <YAxis stroke="#94a3b8" />
              <Tooltip
                contentStyle={{
                  backgroundColor: '#1e293b',
                  border: '1px solid #475569',
                  borderRadius: '0.5rem'
                }}
              />
              <Line
                type="monotone"
                dataKey="loss"
                stroke="#ef4444"
                name="Loss"
                strokeWidth={2}
                dot={false}
              />
              <Line
                type="monotone"
                dataKey="accuracy"
                stroke="#10b981"
                name="Accuracy"
                strokeWidth={2}
                dot={false}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      )}

      {/* Info Box */}
      <div className="bg-blue-900/20 border border-blue-700/50 rounded-xl p-4">
        <p className="text-sm text-blue-300">
          <strong>Training times:</strong> MobileNet ~1 hour, ResNet50 ~4 hours on GPU. Training runs
          in background and doesn't affect RDK inference.
        </p>
      </div>
    </div>
  );
};

export default TrainingDashboard;

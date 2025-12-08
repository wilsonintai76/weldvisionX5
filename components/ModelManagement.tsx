import React, { useState, useEffect } from 'react';
import { Download, Trash2, Send, BarChart3, Copy, Check } from 'lucide-react';

interface Model {
  id: string;
  name: string;
  type: 'mobilenet' | 'resnet50';
  version: string;
  accuracy: number;
  f1_score: number;
  size: string;
  created_at: string;
  deployed: boolean;
}

interface ModelComparisonProps {
  models: Model[];
}

export const ModelManagement: React.FC = () => {
  const [models, setModels] = useState<Model[]>([
    {
      id: '1',
      name: 'mobilenet_v2_001',
      type: 'mobilenet',
      version: '1.0.0',
      accuracy: 0.78,
      f1_score: 0.75,
      size: '18.5 MB',
      created_at: '2024-01-15',
      deployed: true
    },
    {
      id: '2',
      name: 'resnet50_trained_001',
      type: 'resnet50',
      version: '1.0.0',
      accuracy: 0.92,
      f1_score: 0.91,
      size: '104.2 MB',
      created_at: '2024-01-14',
      deployed: false
    }
  ]);

  const [selectedModels, setSelectedModels] = useState<string[]>([]);
  const [showComparison, setShowComparison] = useState(false);
  const [copiedId, setCopiedId] = useState<string | null>(null);

  const handleSelectModel = (id: string) => {
    setSelectedModels(prev =>
      prev.includes(id) ? prev.filter(m => m !== id) : [...prev, id]
    );
  };

  const handleCopyId = (id: string) => {
    navigator.clipboard.writeText(id);
    setCopiedId(id);
    setTimeout(() => setCopiedId(null), 2000);
  };

  const handleDeploy = async (modelId: string) => {
    setModels(models.map(m => (m.id === modelId ? { ...m, deployed: true } : m)));
  };

  const handleDelete = async (modelId: string) => {
    setModels(models.filter(m => m.id !== modelId));
  };

  const handleDownload = (modelId: string) => {
    const model = models.find(m => m.id === modelId);
    if (model) {
      // Simulate download
      console.log(`Downloading ${model.name}...`);
    }
  };

  return (
    <div className="space-y-6">
      {/* Model List */}
      <div className="space-y-3">
        {models.map(model => (
          <div key={model.id} className="bg-slate-900 rounded-xl border border-slate-800 p-4">
            <div className="flex items-start justify-between mb-4">
              <div className="flex-1">
                <div className="flex items-center gap-2 mb-2">
                  <input
                    type="checkbox"
                    checked={selectedModels.includes(model.id)}
                    onChange={() => handleSelectModel(model.id)}
                    className="w-4 h-4 rounded border-slate-600 bg-slate-800 cursor-pointer"
                    title="Select model"
                  />
                  <h3 className="text-lg font-semibold text-white">{model.name}</h3>
                  <span className={`px-2 py-1 rounded text-xs font-medium ${
                    model.deployed
                      ? 'bg-green-900/50 text-green-300'
                      : 'bg-slate-800 text-slate-400'
                  }`}>
                    {model.deployed ? 'DEPLOYED' : 'IDLE'}
                  </span>
                </div>
                <div className="flex items-center gap-3 text-sm text-slate-400">
                  <span>v{model.version}</span>
                  <span>•</span>
                  <span>{model.type.toUpperCase()}</span>
                  <span>•</span>
                  <span>{model.size}</span>
                  <span>•</span>
                  <span>{model.created_at}</span>
                </div>
              </div>
            </div>

            {/* Model Performance Metrics */}
            <div className="grid grid-cols-2 gap-3 mb-4">
              <div className="bg-slate-800 rounded p-3">
                <p className="text-xs text-slate-400 mb-1">Accuracy</p>
                <p className="text-xl font-bold text-white">{(model.accuracy * 100).toFixed(1)}%</p>
              </div>
              <div className="bg-slate-800 rounded p-3">
                <p className="text-xs text-slate-400 mb-1">F1-Score</p>
                <p className="text-xl font-bold text-white">{(model.f1_score * 100).toFixed(1)}%</p>
              </div>
            </div>

            {/* Model Type Badge */}
            <div className="mb-4">
              <p className="text-xs text-slate-400 mb-2">Type</p>
              <div className="flex gap-2">
                {model.type === 'mobilenet' ? (
                  <div className="bg-orange-900/30 border border-orange-700/50 rounded-lg px-3 py-2 text-sm text-orange-300">
                    <p className="font-medium">Edge Model (MobileNetV2)</p>
                    <p className="text-xs mt-1 text-orange-400">~100-150ms inference, 18.5MB</p>
                  </div>
                ) : (
                  <div className="bg-blue-900/30 border border-blue-700/50 rounded-lg px-3 py-2 text-sm text-blue-300">
                    <p className="font-medium">Desktop Model (ResNet50)</p>
                    <p className="text-xs mt-1 text-blue-400">~50ms inference (GPU), 104MB</p>
                  </div>
                )}
              </div>
            </div>

            {/* Model ID and Copy Button */}
            <div className="flex items-center gap-2 mb-4 bg-slate-800 rounded p-2">
              <code className="flex-1 text-xs text-slate-300 font-mono truncate">{model.id}</code>
              <button
                onClick={() => handleCopyId(model.id)}
                title="Copy model ID"
                className="p-1 hover:bg-slate-700 rounded transition-colors"
              >
                {copiedId === model.id ? (
                  <Check className="w-4 h-4 text-green-400" />
                ) : (
                  <Copy className="w-4 h-4 text-slate-400" />
                )}
              </button>
            </div>

            {/* Action Buttons */}
            <div className="flex gap-2">
              {!model.deployed && (
                <button
                  onClick={() => handleDeploy(model.id)}
                  className="flex-1 flex items-center justify-center gap-2 bg-industrial-blue hover:bg-industrial-blue/80 text-white py-2 rounded-lg text-sm font-medium transition-colors"
                >
                  <Send className="w-4 h-4" />
                  Deploy
                </button>
              )}
              <button
                onClick={() => handleDownload(model.id)}
                className="flex-1 flex items-center justify-center gap-2 bg-slate-800 hover:bg-slate-700 text-white py-2 rounded-lg text-sm font-medium transition-colors"
                title="Download model"
              >
                <Download className="w-4 h-4" />
                Download
              </button>
              <button
                onClick={() => handleDelete(model.id)}
                className="flex items-center justify-center gap-2 bg-red-900/30 hover:bg-red-900/50 text-red-400 py-2 px-3 rounded-lg text-sm font-medium transition-colors"
                title="Delete model"
              >
                <Trash2 className="w-4 h-4" />
              </button>
            </div>
          </div>
        ))}
      </div>

      {/* Comparison Section */}
      {selectedModels.length >= 2 && (
        <div className="bg-slate-900 rounded-xl border border-slate-800 p-6">
          <div className="flex items-center justify-between mb-4">
            <h2 className="text-xl font-bold text-white">Model Comparison</h2>
            <button
              onClick={() => setShowComparison(!showComparison)}
              className="flex items-center gap-2 px-4 py-2 bg-industrial-blue hover:bg-industrial-blue/80 text-white rounded-lg text-sm font-medium transition-colors"
            >
              <BarChart3 className="w-4 h-4" />
              {showComparison ? 'Hide' : 'Show'} Comparison
            </button>
          </div>

          {showComparison && (
            <div className="overflow-x-auto">
              <table className="w-full text-sm">
                <thead>
                  <tr className="border-b border-slate-700">
                    <th className="text-left py-3 px-4 text-slate-400 font-medium">Metric</th>
                    {selectedModels.map(id => {
                      const model = models.find(m => m.id === id);
                      return (
                        <th
                          key={id}
                          className="text-left py-3 px-4 text-slate-300 font-medium"
                        >
                          {model?.name}
                        </th>
                      );
                    })}
                  </tr>
                </thead>
                <tbody>
                  <tr className="border-b border-slate-800 hover:bg-slate-800/50">
                    <td className="py-3 px-4 text-slate-400">Accuracy</td>
                    {selectedModels.map(id => {
                      const model = models.find(m => m.id === id);
                      return (
                        <td key={id} className="py-3 px-4 text-white font-medium">
                          {(model?.accuracy ?? 0) * 100}%
                        </td>
                      );
                    })}
                  </tr>
                  <tr className="border-b border-slate-800 hover:bg-slate-800/50">
                    <td className="py-3 px-4 text-slate-400">F1-Score</td>
                    {selectedModels.map(id => {
                      const model = models.find(m => m.id === id);
                      return (
                        <td key={id} className="py-3 px-4 text-white font-medium">
                          {(model?.f1_score ?? 0) * 100}%
                        </td>
                      );
                    })}
                  </tr>
                  <tr className="border-b border-slate-800 hover:bg-slate-800/50">
                    <td className="py-3 px-4 text-slate-400">Size</td>
                    {selectedModels.map(id => {
                      const model = models.find(m => m.id === id);
                      return (
                        <td key={id} className="py-3 px-4 text-white font-medium">
                          {model?.size}
                        </td>
                      );
                    })}
                  </tr>
                  <tr className="border-b border-slate-800 hover:bg-slate-800/50">
                    <td className="py-3 px-4 text-slate-400">Type</td>
                    {selectedModels.map(id => {
                      const model = models.find(m => m.id === id);
                      return (
                        <td key={id} className="py-3 px-4 text-white font-medium">
                          {model?.type === 'mobilenet' ? 'Edge' : 'Desktop'}
                        </td>
                      );
                    })}
                  </tr>
                  <tr className="hover:bg-slate-800/50">
                    <td className="py-3 px-4 text-slate-400">Created</td>
                    {selectedModels.map(id => {
                      const model = models.find(m => m.id === id);
                      return (
                        <td key={id} className="py-3 px-4 text-white font-medium">
                          {model?.created_at}
                        </td>
                      );
                    })}
                  </tr>
                </tbody>
              </table>
            </div>
          )}
        </div>
      )}

      {/* Info Box */}
      <div className="bg-blue-900/20 border border-blue-700/50 rounded-xl p-4">
        <p className="text-sm text-blue-300">
          <strong>Model management:</strong> Download models for backup/sharing. Deploy to activate
          for inference. Delete unused models to free storage.
        </p>
      </div>
    </div>
  );
};

export default ModelManagement;

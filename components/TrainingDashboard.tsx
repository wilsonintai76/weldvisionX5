import React from 'react';
import { Sparkles, ArrowDown, Database, Zap, Upload } from 'lucide-react';

export const TrainingDashboard: React.FC = () => {
  return (
    <div className="space-y-6">
      {/* Overview Card */}
      <div className="bg-slate-900 rounded-xl border border-slate-800 p-6">
        <div className="flex items-start gap-4">
          <div className="bg-industrial-blue/20 p-3 rounded-lg">
            <Sparkles className="w-6 h-6 text-industrial-blue" />
          </div>
          <div className="flex-1">
            <h2 className="text-xl font-bold text-white mb-2">Model Training Workflow</h2>
            <p className="text-slate-400 text-sm mb-4">
              Complete end-to-end training pipeline: from dataset preparation to device deployment.
              Use the Orchestration Panel below to manage the entire workflow.
            </p>
            <div className="grid grid-cols-1 md:grid-cols-4 gap-3">
              <div className="bg-slate-800 p-3 rounded-lg">
                <div className="flex items-center gap-2 mb-1">
                  <Database className="w-4 h-4 text-industrial-blue" />
                  <span className="text-sm font-medium text-white">1. Dataset</span>
                </div>
                <p className="text-xs text-slate-400">Prepare images & labels</p>
              </div>
              <div className="bg-slate-800 p-3 rounded-lg">
                <div className="flex items-center gap-2 mb-1">
                  <Sparkles className="w-4 h-4 text-industrial-blue" />
                  <span className="text-sm font-medium text-white">2. Train</span>
                </div>
                <p className="text-xs text-slate-400">YOLOv8 on desktop GPU</p>
              </div>
              <div className="bg-slate-800 p-3 rounded-lg">
                <div className="flex items-center gap-2 mb-1">
                  <Zap className="w-4 h-4 text-industrial-blue" />
                  <span className="text-sm font-medium text-white">3. Compile</span>
                </div>
                <p className="text-xs text-slate-400">ONNX to Horizon BIN</p>
              </div>
              <div className="bg-slate-800 p-3 rounded-lg">
                <div className="flex items-center gap-2 mb-1">
                  <Upload className="w-4 h-4 text-industrial-blue" />
                  <span className="text-sm font-medium text-white">4. Deploy</span>
                </div>
                <p className="text-xs text-slate-400">Push to RDK-X5 device</p>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Quick Guide */}
      <div className="bg-slate-900 rounded-xl border border-slate-800 p-6">
        <h3 className="text-lg font-semibold text-white mb-4">Quick Start Guide</h3>
        <div className="space-y-3">
          <div className="flex gap-3">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue/20 flex items-center justify-center text-industrial-blue text-sm font-bold">1</div>
            <div>
              <p className="text-white font-medium text-sm">Prepare Your Dataset</p>
              <p className="text-slate-400 text-xs mt-1">
                Use Dataset Studio to capture images and create YOLO labels. Ensure you have at least 100+ images per class.
              </p>
            </div>
          </div>
          <div className="flex gap-3">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue/20 flex items-center justify-center text-industrial-blue text-sm font-bold">2</div>
            <div>
              <p className="text-white font-medium text-sm">Configure Training Settings</p>
              <p className="text-slate-400 text-xs mt-1">
                In the Orchestration Panel below, set your dataset path and number of epochs (10-50 for testing, 100+ for production).
              </p>
            </div>
          </div>
          <div className="flex gap-3">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue/20 flex items-center justify-center text-industrial-blue text-sm font-bold">3</div>
            <div>
              <p className="text-white font-medium text-sm">Start Training</p>
              <p className="text-slate-400 text-xs mt-1">
                Click "Start Training" in the Orchestration Panel. Monitor progress and logs in real-time.
              </p>
            </div>
          </div>
          <div className="flex gap-3">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue/20 flex items-center justify-center text-industrial-blue text-sm font-bold">4</div>
            <div>
              <p className="text-white font-medium text-sm">Compile & Deploy</p>
              <p className="text-slate-400 text-xs mt-1">
                After training completes, compile the ONNX model to Horizon format, then deploy to your RDK-X5 device.
              </p>
            </div>
          </div>
        </div>
      </div>

      {/* Info Box */}
      <div className="bg-blue-900/20 border border-blue-700/50 rounded-xl p-4">
        <div className="flex items-start gap-3">
          <ArrowDown className="w-5 h-5 text-blue-300 flex-shrink-0 mt-0.5" />
          <div>
            <p className="text-sm text-blue-300 font-medium mb-1">Use the Orchestration Panel below</p>
            <p className="text-sm text-blue-300/80">
              All training, compilation, deployment, and inference controls are available in the panel below. 
              Training runs in the background on your desktop GPU and doesn't affect RDK inference.
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default TrainingDashboard;

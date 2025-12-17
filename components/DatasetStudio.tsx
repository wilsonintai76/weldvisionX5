import React from 'react';
import DatasetCapture from './DatasetCapture';
import DatasetLabeler from './DatasetLabeler';
import { listDatasetFiles } from '../api/jobs';
import { Settings } from 'lucide-react';

type TabType = 'capture' | 'labeler' | 'settings';

export default function DatasetStudio() {
  const initial = React.useMemo(() => {
    try {
      const studio = localStorage.getItem('studioDatasetDir');
      if (studio) return studio;
      const orch = JSON.parse(localStorage.getItem('orchestrationSettings') || '{}');
      if (orch && orch.datasetPath) return orch.datasetPath;
    } catch {}
    return 'D:/data/weldsets/setA';
  }, []);

  const [studioDir, setStudioDir] = React.useState<string>(initial);
  const [imgCount, setImgCount] = React.useState<number>(0);
  const [missingCount, setMissingCount] = React.useState<number>(0);
  const [toast, setToast] = React.useState<string>('');
  const [activeTab, setActiveTab] = React.useState<TabType>('capture');
  const [autoSync, setAutoSync] = React.useState<boolean>(() => {
    try {
      return JSON.parse(localStorage.getItem('studioAutoSync') || 'true');
    } catch { return true; }
  });

  const showToast = React.useCallback((msg: string) => {
    setToast(msg);
    window.clearTimeout((window as any).__studio_toast_timer);
    (window as any).__studio_toast_timer = window.setTimeout(() => setToast(''), 2500);
  }, []);

  const refreshCounts = React.useCallback(async () => {
    try {
      const resp = await listDatasetFiles(studioDir);
      setImgCount((resp.images || []).length);
      setMissingCount((resp.labels_missing || []).length);
    } catch {}
  }, [studioDir]);

  React.useEffect(() => { refreshCounts(); }, [refreshCounts]);

  // Auto-sync dataset path to training settings when enabled
  const syncTrainingPath = React.useCallback((path: string, silent = false) => {
    try {
      const saved = JSON.parse(localStorage.getItem('orchestrationSettings') || '{}');
      const next = { ...saved, datasetPath: path };
      localStorage.setItem('orchestrationSettings', JSON.stringify(next));
      if (!silent) showToast('Synced dataset path to Training');
    } catch {}
  }, [showToast]);

  const debounceRef = React.useRef<number | null>(null);
  React.useEffect(() => {
    if (!autoSync) return;
    if (debounceRef.current) window.clearTimeout(debounceRef.current);
    debounceRef.current = window.setTimeout(() => syncTrainingPath(studioDir, true), 300) as unknown as number;
    return () => {
      if (debounceRef.current) window.clearTimeout(debounceRef.current);
    };
  }, [studioDir, autoSync, syncTrainingPath]);

  return (
    <div>
      {/* Top bar with stats */}
      <div className="mb-4 bg-slate-800 border border-slate-700 rounded p-4 flex items-center justify-between">
        <div className="flex items-center gap-6">
          <span className="text-sm text-slate-300">
            Images: <span className="font-mono text-slate-100">{imgCount}</span>
          </span>
          <span className="text-sm text-slate-300">
            Missing labels: <span className="font-mono text-amber-300">{missingCount}</span>
          </span>
          <span className="text-sm text-slate-400">
            {studioDir}
          </span>
        </div>
        <button 
          className="px-3 py-1.5 rounded bg-slate-700 text-white hover:bg-slate-600" 
          title="Refresh Counts" 
          onClick={refreshCounts}
        >
          Refresh
        </button>
      </div>

      {/* Tab Navigation */}
      <div className="mb-4 flex gap-2 border-b border-slate-700">
        <button
          className={`px-4 py-2 font-medium transition-colors ${
            activeTab === 'capture'
              ? 'text-industrial-blue border-b-2 border-industrial-blue'
              : 'text-slate-400 hover:text-slate-200'
          }`}
          onClick={() => setActiveTab('capture')}
        >
          📸 Data Capture
        </button>
        <button
          className={`px-4 py-2 font-medium transition-colors ${
            activeTab === 'labeler'
              ? 'text-industrial-blue border-b-2 border-industrial-blue'
              : 'text-slate-400 hover:text-slate-200'
          }`}
          onClick={() => setActiveTab('labeler')}
        >
          🏷️ Data Labeler
        </button>
        <button
          className={`px-4 py-2 font-medium transition-colors flex items-center gap-2 ${
            activeTab === 'settings'
              ? 'text-industrial-blue border-b-2 border-industrial-blue'
              : 'text-slate-400 hover:text-slate-200'
          }`}
          onClick={() => setActiveTab('settings')}
        >
          <Settings size={16} />
          Settings
        </button>
      </div>

      {toast && (
        <div className="fixed right-4 bottom-4 z-50 bg-slate-800/95 text-white px-4 py-2 rounded shadow-lg border border-slate-700" role="status" aria-live="polite">
          {toast}
        </div>
      )}

      {/* Tab Content */}
      <div className="mt-6">
        {activeTab === 'capture' && (
          <DatasetCapture datasetDir={studioDir} onDatasetDirChange={setStudioDir} onUploadComplete={refreshCounts} />
        )}
        
        {activeTab === 'labeler' && (
          <DatasetLabeler datasetDir={studioDir} onDatasetDirChange={setStudioDir} />
        )}
        
        {activeTab === 'settings' && (
          <div className="bg-slate-800 border border-slate-700 rounded p-6 max-w-3xl">
            <h2 className="text-xl font-semibold text-slate-100 mb-4">Dataset Settings</h2>
            
            <div className="space-y-6">
              <div>
                <label className="block text-slate-300 text-sm font-medium mb-2">Dataset Directory</label>
                <input 
                  className="w-full bg-slate-900 border border-slate-700 rounded px-3 py-2 text-slate-200" 
                  value={studioDir} 
                  onChange={e => { 
                    setStudioDir(e.target.value); 
                    try { localStorage.setItem('studioDatasetDir', e.target.value); } catch {} 
                  }} 
                  placeholder="e.g. D:/data/weldsets/setA" 
                  title="Dataset Directory" 
                />
                <p className="text-slate-400 text-xs mt-2">
                  This directory should contain <code className="bg-slate-900 px-1 rounded">images/</code> and <code className="bg-slate-900 px-1 rounded">labels/</code> subdirectories.
                </p>
              </div>

              <div className="border-t border-slate-700 pt-4">
                <label className="flex items-center gap-3 text-slate-300">
                  <input 
                    type="checkbox" 
                    checked={autoSync} 
                    onChange={(e) => {
                      const next = e.currentTarget.checked; 
                      setAutoSync(next);
                      try { localStorage.setItem('studioAutoSync', JSON.stringify(next)); } catch {}
                      showToast(next ? 'Auto-sync enabled' : 'Auto-sync disabled');
                    }}
                    className="w-4 h-4"
                  />
                  <div>
                    <span className="font-medium">Auto-sync to Training</span>
                    <p className="text-slate-400 text-xs mt-1">
                      Automatically update the Training module's dataset path when changed here
                    </p>
                  </div>
                </label>
              </div>

              <div className="border-t border-slate-700 pt-4">
                <button 
                  className="px-4 py-2 rounded bg-industrial-blue text-white hover:bg-blue-600" 
                  onClick={() => syncTrainingPath(studioDir)}
                >
                  Sync to Training Now
                </button>
                <p className="text-slate-400 text-xs mt-2">
                  Manually sync the current dataset path to the Training module
                </p>
              </div>

              <div className="border-t border-slate-700 pt-4">
                <h3 className="text-slate-200 font-medium mb-2">Quick Actions</h3>
                <div className="flex gap-2">
                  <button 
                    className="px-3 py-1.5 rounded bg-slate-700 text-white hover:bg-slate-600" 
                    onClick={refreshCounts}
                  >
                    Refresh Counts
                  </button>
                  <button 
                    className="px-3 py-1.5 rounded bg-slate-700 text-white hover:bg-slate-600" 
                    onClick={() => {
                      const evt = new KeyboardEvent('keydown', { key: 'ArrowRight' });
                      window.dispatchEvent(evt);
                      setActiveTab('labeler');
                    }}
                  >
                    Next Unlabeled
                  </button>
                  <button 
                    className="px-3 py-1.5 rounded bg-slate-700 text-white hover:bg-slate-600" 
                    onClick={() => { 
                      try { window.open('/rdk_weld_evaluator/AI_MODEL_TRAINING_GUIDE.md', '_blank'); } catch {} 
                    }}
                  >
                    Open Guide
                  </button>
                </div>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

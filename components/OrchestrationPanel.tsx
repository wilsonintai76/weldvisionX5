import React, { useEffect, useState } from "react";
import "../styles/progress.css";
import { startTrain, trainStatus, startCompile, compileStatus, deployModel, startInference, stopInference, cancelTrain, cancelCompile } from "../api/jobs";
import RDKConnectionBadge from "./RDKConnectionBadge";

export default function OrchestrationPanel() {
  const [trainJob, setTrainJob] = useState<string>("");
  const [compileJob, setCompileJob] = useState<string>("");
  const [logs, setLogs] = useState<string>("");
  const [message, setMessage] = useState<string>("");
  const [error, setError] = useState<string>("");
  const [trainState, setTrainState] = useState<string>("");
  const [compileState, setCompileState] = useState<string>("");
  const [trainProgress, setTrainProgress] = useState<number>(0);
  const [trainEpoch, setTrainEpoch] = useState<number>(0);
  const [trainTotalEpochs, setTrainTotalEpochs] = useState<number>(0);
  const [compileProgress, setCompileProgress] = useState<number>(0);
  const [compileStage, setCompileStage] = useState<string>("");
  const [datasetPath, setDatasetPath] = useState<string>("D:\\data\\weldsets\\setA");
  const [epochs, setEpochs] = useState<number>(10);
  const [onnxPath, setOnnxPath] = useState<string>("D:\\models\\yolo.onnx");
  const [outputDir, setOutputDir] = useState<string>("D:\\models\\horizon");
  const [deviceHost, setDeviceHost] = useState<string>("rdk-x5.local");
  const [deviceUser, setDeviceUser] = useState<string>("root");
  const [destPath, setDestPath] = useState<string>("/opt/models/model.bin");

  // Load persisted values
  useEffect(() => {
    try {
      const saved = JSON.parse(localStorage.getItem("orchestrationSettings") || "{}");
      if (saved.datasetPath) setDatasetPath(saved.datasetPath);
      if (saved.epochs) setEpochs(saved.epochs);
      if (saved.onnxPath) setOnnxPath(saved.onnxPath);
      if (saved.outputDir) setOutputDir(saved.outputDir);
      if (saved.deviceHost) setDeviceHost(saved.deviceHost);
      if (saved.deviceUser) setDeviceUser(saved.deviceUser);
      if (saved.destPath) setDestPath(saved.destPath);
    } catch {}
  }, []);

  // Persist on change
  useEffect(() => {
    const settings = { datasetPath, epochs, onnxPath, outputDir, deviceHost, deviceUser, destPath };
    try { localStorage.setItem("orchestrationSettings", JSON.stringify(settings)); } catch {}
  }, [datasetPath, epochs, onnxPath, outputDir, deviceHost, deviceUser, destPath]);

  // Listen for external updates (e.g., Dataset Studio sync)
  useEffect(() => {
    const handler = (e: StorageEvent) => {
      if (e.key === 'orchestrationSettings' && e.newValue) {
        try {
          const saved = JSON.parse(e.newValue);
          if (saved.datasetPath !== undefined) setDatasetPath(saved.datasetPath);
          if (saved.epochs !== undefined) setEpochs(saved.epochs);
          if (saved.onnxPath !== undefined) setOnnxPath(saved.onnxPath);
          if (saved.outputDir !== undefined) setOutputDir(saved.outputDir);
          if (saved.deviceHost !== undefined) setDeviceHost(saved.deviceHost);
          if (saved.deviceUser !== undefined) setDeviceUser(saved.deviceUser);
          if (saved.destPath !== undefined) setDestPath(saved.destPath);
        } catch {}
      }
    };
    window.addEventListener('storage', handler);
    return () => window.removeEventListener('storage', handler);
  }, []);

  // Auto-refresh job status
  useEffect(() => {
    const interval = setInterval(async () => {
      try {
        if (trainJob) {
          const s = await trainStatus(trainJob);
          if (s.state) setTrainState(String(s.state));
          if (typeof s.progress === 'number') setTrainProgress(s.progress);
          if (typeof s.epoch === 'number') setTrainEpoch(s.epoch);
          if (typeof s.total_epochs === 'number') setTrainTotalEpochs(s.total_epochs);
          if (s.logs) setLogs((s.logs || []).join("\n"));
        }
        if (compileJob) {
          const s = await compileStatus(compileJob);
          if (s.state) setCompileState(String(s.state));
          if (typeof s.progress === 'number') setCompileProgress(s.progress);
          if (typeof s.stage === 'string') setCompileStage(s.stage);
          if (s.logs) setLogs((s.logs || []).join("\n"));
        }
      } catch {}
    }, 2000);
    return () => clearInterval(interval);
  }, [trainJob, compileJob]);

  const badge = (state?: string) => {
    const s = (state || '').toLowerCase();
    const cls = s === 'running' ? 'bg-industrial-blue/20 text-industrial-blue' : s === 'completed' ? 'bg-industrial-success/20 text-industrial-success' : s === 'failed' ? 'bg-industrial-danger/20 text-industrial-danger' : 'bg-slate-700 text-slate-300';
    const label = s ? s.toUpperCase() : 'IDLE';
    return <span className={`text-xs px-2 py-0.5 rounded ${cls}`}>{label}</span>;
  };

  return (
    <div className="p-4">
      <h3 className="text-white text-lg font-semibold mb-3">Orchestration Panel</h3>

      <div className="grid gap-3 grid-cols-1 md:grid-cols-2">
        <div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
          <div className="flex items-center justify-between mb-2">
            <h4 className="text-white font-medium">Training</h4>
            <div className="flex items-center gap-2">
              {badge(trainState || (trainJob ? 'running' : undefined))}
              {(trainState || trainJob) && (
                <span className="text-xs text-slate-300">
                  {Math.round(trainProgress) || 0}%{trainTotalEpochs > 0 ? ` (${trainEpoch}/${trainTotalEpochs})` : ''}
                </span>
              )}
            </div>
          </div>
          <div className="grid gap-2">
            <label>
              <span>Dataset Path</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={datasetPath} onChange={(e) => setDatasetPath(e.target.value)} />
            </label>
            <label>
              <span>Epochs</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" type="number" min={1} max={200} value={epochs} onChange={(e) => setEpochs(Number(e.target.value))} />
            </label>
          </div>
          <button className="mt-2 px-3 py-1.5 rounded bg-industrial-blue text-white hover:opacity-90" onClick={async () => {
            setError(""); setMessage("");
            if (!datasetPath) { setError("Dataset path is required."); return; }
            if (epochs < 1) { setError("Epochs must be >= 1."); return; }
            try {
              const r = await startTrain(datasetPath, epochs);
              setTrainJob(r.job_id);
              setTrainState('running');
              setMessage(`Training started: ${r.job_id}`);
            } catch (e: any) {
              setError(e?.message || "Failed to start training");
            }
          }}>Start Training</button>
          <button className="mt-2 ml-2 px-3 py-1.5 rounded bg-slate-700 text-white disabled:opacity-50" disabled={!trainJob} onClick={async () => {
            const s = await trainStatus(trainJob);
            if (s.state) setTrainState(String(s.state));
            if (typeof s.progress === 'number') setTrainProgress(s.progress);
            if (typeof s.epoch === 'number') setTrainEpoch(s.epoch);
            if (typeof s.total_epochs === 'number') setTrainTotalEpochs(s.total_epochs);
            setLogs((s.logs || []).join("\n"));
          }}>Check Training Status</button>
          <button className="mt-2 ml-2 px-3 py-1.5 rounded bg-industrial-danger text-white disabled:opacity-50" disabled={!trainJob} onClick={async () => {
            try {
              await cancelTrain(trainJob);
              setTrainState('cancelled');
              setMessage('Training cancelled');
            } catch (e: any) {
              setError(e?.message || 'Cancel failed');
            }
          }}>Cancel Training</button>
          {(trainState === 'running' || trainProgress > 0) && (
            <div className="mt-2">
              <div className="flex items-center justify-between text-xs text-slate-400 mb-1">
                <span>Progress</span>
                <span>{Math.round(trainProgress)}%</span>
              </div>
              <div className="w-full bg-slate-700 h-2 rounded">
                {(() => {
                  const pct = Math.max(0, Math.min(100, Math.round(trainProgress / 5) * 5));
                  const cls = `w-pct-${pct}`;
                  return (<div className={`bg-industrial-blue h-2 rounded ${cls}`}></div>);
                })()}
              </div>
              {trainTotalEpochs > 0 && (
                <div className="text-xs text-slate-400 mt-1">Epoch {trainEpoch} / {trainTotalEpochs}</div>
              )}
            </div>
          )}
        </div>

        <div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
          <div className="flex items-center justify-between mb-2">
            <h4 className="text-white font-medium">Compilation</h4>
            <div className="flex items-center gap-2">
              {badge(compileState || (compileJob ? 'running' : undefined))}
              {(compileState || compileJob) && (
                <span className="text-xs text-slate-300">{Math.round(compileProgress) || 0}%</span>
              )}
            </div>
          </div>
          <div className="grid gap-2">
            <label>
              <span>ONNX Path</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={onnxPath} onChange={(e) => setOnnxPath(e.target.value)} />
            </label>
            <label>
              <span>Output Dir</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={outputDir} onChange={(e) => setOutputDir(e.target.value)} />
            </label>
          </div>
          <button className="mt-2 px-3 py-1.5 rounded bg-industrial-blue text-white hover:opacity-90" onClick={async () => {
            setError(""); setMessage("");
            if (!onnxPath) { setError("ONNX path is required."); return; }
            if (!outputDir) { setError("Output directory is required."); return; }
            try {
              const r = await startCompile(onnxPath, outputDir);
              setCompileJob(r.job_id);
              setCompileState('running');
              setMessage(`Compilation started: ${r.job_id}`);
            } catch (e: any) {
              setError(e?.message || "Failed to start compilation");
            }
          }}>Start Compile</button>
          <button className="mt-2 ml-2 px-3 py-1.5 rounded bg-slate-700 text-white disabled:opacity-50" disabled={!compileJob} onClick={async () => {
            const s = await compileStatus(compileJob);
            if (s.state) setCompileState(String(s.state));
            if (typeof s.progress === 'number') setCompileProgress(s.progress);
            if (typeof s.stage === 'string') setCompileStage(s.stage);
            setLogs((s.logs || []).join("\n"));
          }}>Check Compile Status</button>
          <button className="mt-2 ml-2 px-3 py-1.5 rounded bg-industrial-danger text-white disabled:opacity-50" disabled={!compileJob} onClick={async () => {
            try {
              await cancelCompile(compileJob);
              setCompileState('cancelled');
              setMessage('Compilation cancelled');
            } catch (e: any) {
              setError(e?.message || 'Cancel failed');
            }
          }}>Cancel Compile</button>
          {(compileState === 'running' || compileProgress > 0) && (
            <div className="mt-2">
              <div className="flex items-center justify-between text-xs text-slate-400 mb-1">
                <span>Progress</span>
                <span>{Math.round(compileProgress)}%</span>
              </div>
              <div className="w-full bg-slate-700 h-2 rounded">
                {(() => {
                  const pct = Math.max(0, Math.min(100, Math.round(compileProgress / 5) * 5));
                  const cls = `w-pct-${pct}`;
                  return (<div className={`bg-industrial-blue h-2 rounded ${cls}`}></div>);
                })()}
              </div>
                {compileStage && (
                  <div className="text-xs text-slate-400 mt-1">Stage: {compileStage}</div>
                )}
              {/* Stage Timeline */}
              <div className="mt-2 flex items-center gap-2">
                {['quantize','map','pack'].map((st, idx) => {
                  const active = (compileStage || '').toLowerCase() === st;
                  const done = ['quantize','map','pack'].indexOf((compileStage || '').toLowerCase()) > idx;
                  const base = 'flex items-center gap-2';
                  return (
                    <div key={st} className="flex items-center gap-2">
                      <div className={`px-2 py-0.5 rounded text-xs border ${active ? 'bg-industrial-blue/20 text-industrial-blue border-industrial-blue' : done ? 'bg-industrial-success/20 text-industrial-success border-industrial-success' : 'bg-slate-700 text-slate-300 border-slate-600'}`}>{st.toUpperCase()}</div>
                      {idx < 2 && <div className={`w-6 h-0.5 ${done ? 'bg-industrial-success' : 'bg-slate-600'}`}></div>}
                    </div>
                  );
                })}
              </div>
            </div>
          )}
        </div>

        <div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
          <div className="flex items-center justify-between mb-2">
            <h4 className="text-white font-medium">Deployment</h4>
            <RDKConnectionBadge host={deviceHost} user={deviceUser} />
          </div>
          <div className="grid gap-2">
            <label>
              <span>Device Host</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={deviceHost} onChange={(e) => setDeviceHost(e.target.value)} />
            </label>
            <label>
              <span>Device User</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={deviceUser} onChange={(e) => setDeviceUser(e.target.value)} />
            </label>
            <label>
              <span>Model .bin Path</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-400" value={outputDir + "\\\\model.bin"} readOnly />
            </label>
            <label>
              <span>Destination Path</span>
              <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={destPath} onChange={(e) => setDestPath(e.target.value)} />
            </label>
          </div>
          <button className="mt-2 px-3 py-1.5 rounded bg-industrial-blue text-white hover:opacity-90" onClick={async () => {
            setError(""); setMessage("");
            if (!deviceHost || !deviceUser) { setError("Device host and user are required."); return; }
            try {
              await deployModel(deviceHost, deviceUser, `${outputDir}\\\\model.bin`, destPath);
              setMessage("Deployment complete");
            } catch (e: any) {
              setError(e?.message || "Deployment failed");
            }
          }}>Deploy Model</button>
        </div>

        <div className="border border-slate-700 rounded-lg p-3 bg-slate-800">
          <h4 className="text-white font-medium mb-2">Inference</h4>
          <button className="px-3 py-1.5 rounded bg-industrial-blue text-white hover:opacity-90" onClick={async () => {
            setError(""); setMessage("");
            try {
              await startInference(deviceHost, deviceUser, `${outputDir}\\model.bin`, undefined);
              setMessage("Inference started");
            } catch (e: any) {
              setError(e?.message || "Failed to start inference");
            }
          }}>Start Inference</button>
          <button className="ml-2 px-3 py-1.5 rounded bg-slate-700 text-white" title="Stop Inference" onClick={async () => {
            setError(""); setMessage("");
            const ok = window.confirm(`Stop inference on ${deviceHost} as ${deviceUser}?`);
            if (!ok) return;
            try {
              await stopInference(deviceHost, deviceUser);
              setMessage("Inference stopped");
            } catch (e: any) {
              setError(e?.message || "Failed to stop inference");
            }
          }}>Stop Inference</button>
        </div>
      </div>

      {message && (<div className="mt-3 p-2 bg-green-900 text-green-100 border border-green-800 rounded">{message}</div>)}
      {error && (<div className="mt-3 p-2 bg-red-900 text-red-100 border border-red-800 rounded">{error}</div>)}
      <pre className="mt-3 whitespace-pre-wrap bg-slate-900 text-green-400 p-3 rounded border border-slate-700">{logs}</pre>
    </div>
  );
}

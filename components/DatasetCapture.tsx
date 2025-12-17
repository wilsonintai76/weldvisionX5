import React, { useEffect, useRef, useState } from 'react';
import { uploadDatasetImage, listDatasetFiles, deleteDatasetImage } from '../api/jobs';

interface Props {
  datasetDir?: string;
  onDatasetDirChange?: (dir: string) => void;
  onUploadComplete?: () => void;
}

export default function DatasetCapture({ datasetDir: propDatasetDir, onDatasetDirChange, onUploadComplete }: Props) {
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [datasetDirState, setDatasetDirState] = useState('D:/data/weldsets/setA');
  const [refreshTrigger, setRefreshTrigger] = useState(0);
  const [nextIndex, setNextIndex] = useState(1);
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [images, setImages] = useState<string[]>([]);
  const [labelsMissing, setLabelsMissing] = useState<string[]>([]);
  const [uploading, setUploading] = useState(false);

  const datasetDir = propDatasetDir ?? datasetDirState;
  const setDatasetDir = (val: string) => {
    if (onDatasetDirChange) onDatasetDirChange(val);
    else setDatasetDirState(val);
  };

  useEffect(() => {
    (async () => {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ video: true });
        if (videoRef.current) {
          videoRef.current.srcObject = stream as any;
          await videoRef.current.play();
        }
      } catch (e) {
        setError('Camera access failed. You can still paste images.');
      }
    })();
    return () => {
      const tracks = (videoRef.current?.srcObject as MediaStream | undefined)?.getTracks();
      tracks?.forEach(t => t.stop());
    };
  }, []);

  useEffect(() => {
    let cancelled = false;
    async function scan() {
      try {
        const resp = await listDatasetFiles(datasetDir, 'images', 'labels');
        if (cancelled) return;
        if (resp.error) {
          console.error('Failed to list dataset files:', resp.error);
          return;
        }
        const imgs: string[] = resp.images || [];
        setImages(imgs);
        setLabelsMissing(resp.labels_missing || []);
        // derive next index from highest img_XXXX.jpg
        const nums = imgs.map(n => {
          const m = n.match(/img_(\d+)\.(jpg|jpeg|png)/i);
          return m ? parseInt(m[1], 10) : 0;
        }).filter(n=>!isNaN(n));
        const next = (nums.length ? Math.max(...nums) : 0) + 1;
        setNextIndex(next);
      } catch (e) {
        console.error('Error scanning dataset:', e);
      }
    }
    if (datasetDir) scan();
    return () => { cancelled = true; };
  }, [datasetDir, refreshTrigger]);

  const capture = async () => {
    try {
      setError(''); setMessage('');
      const w = 640, h = 480;
      const canvas = canvasRef.current!;
      canvas.width = w; canvas.height = h;
      const ctx = canvas.getContext('2d')!;
      if (videoRef.current) {
        ctx.drawImage(videoRef.current, 0, 0, w, h);
      }
      const dataUrl = canvas.toDataURL('image/jpeg', 0.92);
      const imgName = `img_${String(nextIndex).padStart(4, '0')}.jpg`;
      const savePath = `${datasetDir}/images/${imgName}`.replace(/\\/g,'/');
      const resp = await uploadDatasetImage(dataUrl, savePath, null, []);
      if (resp.error) throw new Error(resp.error);
      setMessage(`Saved ${resp.imagePath}`);
      setNextIndex(n => n + 1);
      setRefreshTrigger(prev => prev + 1);
      if (onUploadComplete) onUploadComplete();
      
      // Notify other components via storage event
      try {
        localStorage.setItem('datasetRefresh', Date.now().toString());
      } catch {}
    } catch (e: any) {
      setError(e?.message || 'Capture failed');
    }
  };

  const handleFileUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const files = event.target.files;
    if (!files || files.length === 0) return;

    setUploading(true);
    setError('');
    setMessage('');

    try {
      let uploaded = 0;
      let currentIndex = nextIndex;

      for (let i = 0; i < files.length; i++) {
        const file = files[i];
        
        // Convert file to data URL
        const dataUrl = await new Promise<string>((resolve, reject) => {
          const reader = new FileReader();
          reader.onload = () => resolve(reader.result as string);
          reader.onerror = reject;
          reader.readAsDataURL(file);
        });

        const imgName = `img_${String(currentIndex).padStart(4, '0')}.jpg`;
        const savePath = `${datasetDir}/images/${imgName}`.replace(/\\/g,'/');

        const resp = await uploadDatasetImage(dataUrl, savePath, null, []);
        if (resp.error) throw new Error(resp.error);

        uploaded++;
        currentIndex++;
      }

      setMessage(`Successfully uploaded ${uploaded} image(s)`);
      setNextIndex(currentIndex);
      
      // Clear the file input
      if (fileInputRef.current) {
        fileInputRef.current.value = '';
      }
      
      // Trigger re-scan and notify parent
      setRefreshTrigger(prev => prev + 1);
      if (onUploadComplete) onUploadComplete();
      
      // Notify other components via storage event
      try {
        localStorage.setItem('datasetRefresh', Date.now().toString());
      } catch {}
    } catch (e: any) {
      setError(e?.message || 'Upload failed');
    } finally {
      setUploading(false);
    }
  };

  return (
    <div className="p-4">
      <h3 className="text-white text-lg font-semibold mb-3">Dataset Capture</h3>
      <div className="grid md:grid-cols-2 gap-4">
        <div className="bg-slate-800 border border-slate-700 rounded p-3">
          <div className="mb-3">
            <div className="text-sm text-slate-400 mb-2">📷 Live Camera (RDK Stereo Camera or Webcam)</div>
            <video ref={videoRef} className="w-full h-auto rounded bg-slate-900" />
            <canvas ref={canvasRef} className="hidden" />
          </div>
          
          <div className="mt-3">
            <label className="block text-slate-300 text-sm mb-1">Dataset Directory</label>
            <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={datasetDir} onChange={e=>setDatasetDir(e.target.value)} placeholder="e.g. D:/data/weldsets/setA" title="Dataset Directory" />
            
            <div className="mt-3 flex gap-2">
              <button 
                className="flex-1 px-3 py-2 rounded bg-industrial-blue text-white hover:bg-sky-400 transition-colors font-medium"
                onClick={capture}
                title="Capture from live camera"
              >
                📸 Capture from Camera
              </button>
            </div>
            
            <div className="mt-3 border-t border-slate-700 pt-3">
              <div className="text-sm text-slate-400 mb-2">📁 Upload Existing Images</div>
              <input
                ref={fileInputRef}
                type="file"
                accept="image/*"
                multiple
                onChange={handleFileUpload}
                className="hidden"
                id="file-upload"
              />
              <label
                htmlFor="file-upload"
                className={`block w-full px-3 py-2 rounded border-2 border-dashed text-center cursor-pointer transition-colors ${
                  uploading 
                    ? 'bg-slate-700 border-slate-600 text-slate-500 cursor-not-allowed' 
                    : 'bg-slate-900 border-slate-600 text-slate-300 hover:border-industrial-blue hover:bg-slate-800'
                }`}
              >
                {uploading ? '⏳ Uploading...' : '📤 Click to Upload Images'}
              </label>
              <p className="text-xs text-slate-500 mt-1">
                Supports: JPG, PNG | Multiple files allowed
              </p>
            </div>

            {/* Status Cards */}
            <div className="mt-3 grid grid-cols-3 gap-2">
              <div className="bg-slate-900 border border-slate-700 rounded p-3">
                <div className="text-slate-400 text-xs mb-1">Total Images</div>
                <div className="text-2xl font-bold text-slate-100">{images.length}</div>
              </div>
              <div className="bg-slate-900 border border-amber-700 rounded p-3">
                <div className="text-slate-400 text-xs mb-1">Unlabeled</div>
                <div className="text-2xl font-bold text-amber-300">{labelsMissing.length}</div>
              </div>
              <div className="bg-slate-900 border border-green-700 rounded p-3">
                <div className="text-slate-400 text-xs mb-1">Labeled</div>
                <div className="text-2xl font-bold text-green-400">{images.length - labelsMissing.length}</div>
              </div>
            </div>
            
            {/* Image list with delete buttons */}
            {images.length > 0 && (
              <div className="mt-4">
                <div className="text-slate-300 text-sm font-semibold mb-2 flex items-center justify-between">
                  <span>📦 Dataset Images ({images.length})</span>
                  <span className="text-xs text-slate-500">Click to manage</span>
                </div>
                <div className="max-h-80 overflow-auto border border-slate-700 rounded bg-slate-900">
                  {images.map((img) => {
                    const hasLabel = !labelsMissing.some(m => m.replace('.txt', '') === img.replace(/\.(jpg|jpeg|png)$/i, ''));
                    return (
                      <div key={img} className={`px-3 py-2 hover:bg-slate-800 flex justify-between items-center border-b border-slate-800 last:border-b-0 ${hasLabel ? 'bg-green-900/10' : 'bg-amber-900/10'}`}>
                        <div className="flex items-center gap-3 flex-1">
                          <div className={`w-2 h-2 rounded-full ${hasLabel ? 'bg-green-400' : 'bg-amber-400'}`} />
                          <span className="text-slate-200 text-sm font-mono">{img}</span>
                          {hasLabel && <span className="text-xs px-2 py-0.5 rounded-full bg-green-700/30 text-green-300 border border-green-700">Labeled</span>}
                          {!hasLabel && <span className="text-xs px-2 py-0.5 rounded-full bg-amber-700/30 text-amber-300 border border-amber-700">Needs Label</span>}
                        </div>
                        <div className="flex items-center gap-2">
                          <button
                            className="px-3 py-1 rounded bg-red-700 hover:bg-red-600 text-white text-xs font-medium transition-colors"
                            onClick={async () => {
                              if (!confirm(`Delete ${img}?`)) return;
                              try {
                                setError('');
                                const imgPath = `${datasetDir}/images/${img}`.replace(/\\\\/g, '/');
                                const lblPath = `${datasetDir}/labels/${img.replace(/\\.(jpg|jpeg|png)$/i, '.txt')}`.replace(/\\\\/g, '/');
                                const resp = await deleteDatasetImage(imgPath, lblPath);
                                if (resp.error) throw new Error(resp.error);
                                setMessage(`Deleted ${img}`);
                                setRefreshTrigger(prev => prev + 1);
                                if (onUploadComplete) onUploadComplete();
                                try {
                                  localStorage.setItem('datasetRefresh', Date.now().toString());
                                } catch {}
                              } catch (e: any) {
                                setError(e?.message || 'Delete failed');
                              }
                            }}
                          >
                            Delete
                          </button>
                        </div>
                      </div>
                    );
                  })}
                </div>
              </div>
            )}
          </div>
        </div>
        
        <div>
          {message && <div className="p-3 bg-green-900 text-green-100 border border-green-800 rounded mb-3">{message}</div>}
          {error && <div className="p-3 bg-red-900 text-red-100 border border-red-800 rounded mb-3">{error}</div>}
          
          <div className="bg-slate-800 border border-slate-700 rounded p-4">
            <h4 className="text-white font-medium mb-3">💡 Capture Options</h4>
            <div className="space-y-3 text-sm">
              <div className="bg-slate-900 rounded p-3">
                <div className="text-industrial-blue font-medium mb-1">🎥 Live Camera Capture</div>
                <p className="text-slate-400">
                  For real-time welding inspection using RDK Stereo Camera connected to RDK X5, or webcam for testing.
                </p>
              </div>
              
              <div className="bg-slate-900 rounded p-3">
                <div className="text-green-400 font-medium mb-1">📁 Upload Existing Images</div>
                <p className="text-slate-400">
                  Import images from files, previous scans, or other sources to build training dataset.
                </p>
              </div>
            </div>
          </div>

          <div className="mt-4 bg-slate-900 border border-slate-700 rounded p-3">
            <p className="text-slate-400 text-xs">
              <strong>Storage:</strong> Images → <code className="text-slate-300">{datasetDir}/images</code><br/>
              Labels → <code className="text-slate-300">{datasetDir}/labels</code>
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}

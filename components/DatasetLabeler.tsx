import React, { useEffect, useRef, useState } from 'react';
import { uploadDatasetImage, generateDatasetYaml, listDatasetFiles, deleteDatasetImage, readLabelFile } from '../api/jobs';

interface Box { x: number; y: number; w: number; h: number; class_id: number; }

interface Props {
  datasetDir?: string;
  onDatasetDirChange?: (dir: string) => void;
}

export default function DatasetLabeler({ datasetDir: propDatasetDir, onDatasetDirChange }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [imageSrc, setImageSrc] = useState<string>('');
  const [boxes, setBoxes] = useState<Box[]>([]);
  const [current, setCurrent] = useState<Box | null>(null);
  const [datasetDirState, setDatasetDirState] = useState('D:/data/weldsets/setA');
  const [imageName, setImageName] = useState('img_0001.jpg');
  const [classId, setClassId] = useState(0);
  const [classNames, setClassNames] = useState<string[]>(['good','porosity','undercut','spatter']);
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [images, setImages] = useState<string[]>([]);
  const [labelsMissing, setLabelsMissing] = useState<string[]>([]);
  const [scanInfo, setScanInfo] = useState<{imageDir?: string; labelDir?: string}>({});
  const [cursor, setCursor] = useState<number>(0);
  const [showHelp, setShowHelp] = useState<boolean>(false);
  const [refreshTrigger, setRefreshTrigger] = useState(0);
  const [labeledImagesInfo, setLabeledImagesInfo] = useState<Array<{name: string, classes: string[]}>>([]);

  const datasetDir = propDatasetDir ?? datasetDirState;
  const setDatasetDir = (val: string) => {
    if (onDatasetDirChange) onDatasetDirChange(val);
    else setDatasetDirState(val);
  };

  // Helper function to convert file path to HTTP URL
  const getImageUrl = (imageDir: string, imageName: string) => {
    const fullPath = `${imageDir.replace(/\\/g, '/')}/${imageName}`;
    const url = `/api/dataset/image/${encodeURIComponent(fullPath)}`;
    console.log('Loading image:', { imageDir, imageName, fullPath, url });
    return url;
  };

  useEffect(() => {
    if (!imageSrc) {
      console.log('No image source set');
      return;
    }
    console.log('Loading image from:', imageSrc);
    const img = new Image();
    img.onload = () => {
      console.log('Image loaded successfully:', img.width, 'x', img.height);
      const canvas = canvasRef.current;
      if (!canvas) {
        console.error('Canvas not found');
        return;
      }
      
      // Set canvas to actual image size (not scaled)
      canvas.width = img.width;
      canvas.height = img.height;
      
      const ctx = canvas.getContext('2d');
      if (ctx) {
        // Clear canvas first
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        // Draw white background to see if canvas is working
        ctx.fillStyle = 'white';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        // Draw image
        ctx.drawImage(img, 0, 0);
        console.log('Image drawn to canvas');
        
        // Load existing labels if available
        loadExistingLabels();
      }
    };
    img.onerror = (e) => {
      console.error('Failed to load image:', imageSrc, e);
    };
    img.src = imageSrc;
  }, [imageSrc]);

  const loadExistingLabels = async () => {
    try {
      if (!imageName || !scanInfo.labelDir) return;
      const labelFileName = imageName.replace(/\.(jpg|jpeg|png)$/i, '.txt');
      const labelPath = `${scanInfo.labelDir}/${labelFileName}`.replace(/\\/g, '/');
      const resp = await readLabelFile(labelPath);
      if (resp.ok && resp.boxes && resp.boxes.length > 0) {
        const canvas = canvasRef.current;
        if (!canvas) return;
        // Convert normalized YOLO format back to pixel coordinates
        const loadedBoxes = resp.boxes.map((b: any) => ({
          x: (b.x - b.w / 2) * canvas.width,
          y: (b.y - b.h / 2) * canvas.height,
          w: b.w * canvas.width,
          h: b.h * canvas.height,
          class_id: b.class_id
        }));
        setBoxes(loadedBoxes);
        console.log('Loaded existing labels:', loadedBoxes);
      } else {
        setBoxes([]);
      }
    } catch (e) {
      console.log('No existing labels or error loading:', e);
      setBoxes([]);
    }
  };

  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      if (e.key === '?') {
        e.preventDefault();
        setShowHelp(s => !s);
        return;
      }
      if (e.key === 'ArrowLeft') {
        e.preventDefault();
        if (labelsMissing.length > 0) {
          const prev = Math.max(0, cursor - 1);
          setCursor(prev);
          const stem = labelsMissing[prev].replace('.txt','');
          const pick = images.find(f => f.startsWith(stem));
          if (pick && scanInfo.imageDir) {
            setImageName(pick);
            setImageSrc(getImageUrl(scanInfo.imageDir, pick));
          }
        }
      } else if (e.key === 'ArrowRight' || e.key.toLowerCase() === 'n') {
        e.preventDefault();
        if (labelsMissing.length > 0) {
          const next = Math.min(labelsMissing.length - 1, cursor + 1);
          setCursor(next);
          const stem = labelsMissing[next].replace('.txt','');
          const pick = images.find(f => f.startsWith(stem));
          if (pick && scanInfo.imageDir) {
            setImageName(pick);
            setImageSrc(getImageUrl(scanInfo.imageDir, pick));
          }
        }
      } else if (e.key.toLowerCase() === 's') {
        e.preventDefault();
        // trigger save
        (async () => { await saveLabels(); })();
      }
    };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [cursor, labelsMissing, images, scanInfo.imageDir, imageSrc]);

  useEffect(() => {
    let cancelled = false;
    async function scan() {
      setError('');
      try {
        const resp = await listDatasetFiles(datasetDir, 'images', 'labels');
        console.log('Dataset scan response:', resp);
        if (!resp.ok) throw new Error(resp.error || 'Scan failed');
        if (cancelled) return;
        setImages(resp.images || []);
        const missing = resp.labels_missing || [];
        setLabelsMissing(missing);
        setScanInfo({ imageDir: resp.image_dir, labelDir: resp.label_dir });
        
        console.log('Scan complete:', { 
          images: resp.images, 
          missing, 
          imageDir: resp.image_dir 
        });
        
        // Check if current image still exists in the list
        const currentImageExists = (resp.images || []).includes(imageName);
        
        // Auto-select image: prioritize missing labels, otherwise load first image
        if (missing.length > 0) {
          const idx = Math.min(cursor, missing.length - 1);
          const stem = (missing[idx] || '').replace('.txt','');
          const pick = (resp.images || []).find((f: string) => f.startsWith(stem));
          if (pick && resp.image_dir) {
            console.log('Loading missing label image:', pick);
            setImageName(pick);
            setImageSrc(getImageUrl(resp.image_dir, pick));
          }
        } else if ((resp.images || []).length > 0 && resp.image_dir) {
          // If current image was deleted, load first image
          if (!currentImageExists) {
            const firstImage = resp.images[0];
            console.log('Loading first image:', firstImage);
            setImageName(firstImage);
            setImageSrc(getImageUrl(resp.image_dir, firstImage));
          }
        } else {
          // No images left - clear display
          console.log('No images available - clearing display');
          setImageName('');
          setImageSrc('');
          setBoxes([]);
        }
        
        // Load label information for labeled images
        const labeledImages: Array<{name: string, classes: string[]}> = [];
        const labelDir = resp.label_dir || `${datasetDir}/labels`;
        for (const img of (resp.images || [])) {
          const hasLabel = !missing.some(m => m.replace('.txt', '') === img.replace(/\.(jpg|jpeg|png)$/i, ''));
          if (hasLabel) {
            const labelFileName = img.replace(/\.(jpg|jpeg|png)$/i, '.txt');
            const labelPath = `${labelDir}/${labelFileName}`.replace(/\\/g, '/');
            try {
              const labelResp = await readLabelFile(labelPath);
              if (labelResp.ok && labelResp.boxes && labelResp.boxes.length > 0) {
                const uniqueClasses = [...new Set(labelResp.boxes.map((b: any) => b.class_id))];
                const classNamesList = uniqueClasses.map(id => classNames[id] || `class_${id}`);
                labeledImages.push({ name: img, classes: classNamesList });
              }
            } catch (e) {
              console.error(`Failed to read label for ${img}:`, e);
            }
          }
        }
        setLabeledImagesInfo(labeledImages);
      } catch (e: any) {
        setError(e?.message || 'Scan failed');
      }
    }
    if (datasetDir) scan();
    return () => { cancelled = true; };
  }, [datasetDir, refreshTrigger]);

  // Listen for storage events to refresh when files are uploaded/deleted
  useEffect(() => {
    const handleStorageChange = () => {
      // Trigger refresh by incrementing refreshTrigger
      setRefreshTrigger(prev => prev + 1);
    };
    
    // Listen for storage events from other tabs/windows
    window.addEventListener('storage', (e: StorageEvent) => {
      if (e.key === 'datasetRefresh') handleStorageChange();
    });
    
    // Also listen for custom events from same window
    const intervalId = setInterval(() => {
      try {
        const timestamp = localStorage.getItem('datasetRefresh');
        if (timestamp && parseInt(timestamp) > Date.now() - 1000) {
          handleStorageChange();
        }
      } catch {}
    }, 500);
    
    return () => {
      clearInterval(intervalId);
    };
  }, []);

  const onMouseDown = (e: React.MouseEvent) => {
    const rect = (e.target as HTMLCanvasElement).getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    setCurrent({ x, y, w: 0, h: 0, class_id: classId });
  };
  const onMouseMove = (e: React.MouseEvent) => {
    if (!current) return;
    const canvas = canvasRef.current!;
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const newBox = { ...current, w: x - current.x, h: y - current.y };
    drawAll(newBox);
  };
  const onMouseUp = () => {
    if (!current) return;
    const final = { ...current };
    setBoxes(prev => [...prev, final]);
    setCurrent(null);
    drawAll();
  };

  const drawAll = (temp?: Box) => {
    const canvas = canvasRef.current!;
    const ctx = canvas.getContext('2d')!;
    const img = new Image();
    img.src = imageSrc;
    img.onload = () => {
      ctx.clearRect(0,0,canvas.width,canvas.height);
      ctx.fillStyle = 'white';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(img, 0, 0);
      [...boxes, ...(temp ? [temp] : [])].forEach(b => {
        ctx.strokeStyle = '#0ea5e9';
        ctx.lineWidth = 3;
        ctx.strokeRect(b.x, b.y, b.w, b.h);
        // Draw class label
        const className = classNames[b.class_id] || `class_${b.class_id}`;
        ctx.fillStyle = '#0ea5e9';
        ctx.fillRect(b.x, b.y - 20, ctx.measureText(className).width + 8, 20);
        ctx.fillStyle = 'white';
        ctx.font = '12px monospace';
        ctx.fillText(className, b.x + 4, b.y - 6);
      });
    };
  };

  // Re-draw when boxes change
  useEffect(() => {
    if (imageSrc && boxes.length > 0) {
      drawAll();
    }
  }, [boxes]);

  const saveLabels = async () => {
    try {
      setError(''); setMessage('');
      
      if (boxes.length === 0) {
        setError('Please draw at least one bounding box before saving');
        return;
      }
      
      if (!imageName || !scanInfo.imageDir) {
        setError('Image not loaded. Please select an image first.');
        return;
      }
      
      const canvas = canvasRef.current!;
      const dataUrl = canvas.toDataURL('image/jpeg', 0.92);
      const imgName = imageName;
      const labelName = imgName.replace(/\.(jpg|jpeg|png)$/i, '.txt');
      const savePath = `${scanInfo.imageDir}/${imgName}`.replace(/\\/g,'/');
      const labelPath = `${scanInfo.labelDir}/${labelName}`.replace(/\\/g,'/');
      
      // Convert to YOLO normalized bbox
      const labels = boxes.map(b => {
        const x = (b.x + b.w/2) / canvas.width;
        const y = (b.y + b.h/2) / canvas.height;
        const w = Math.abs(b.w) / canvas.width;
        const h = Math.abs(b.h) / canvas.height;
        return { class_id: b.class_id, bbox: [x,y,w,h] };
      });
      
      const resp = await uploadDatasetImage(dataUrl, savePath, labelPath, labels);
      if (resp.error) throw new Error(resp.error);
      
      setMessage(`✓ Saved ${labels.length} label(s) for ${imgName}`);
      setBoxes([]);
      
      // Trigger full refresh
      setRefreshTrigger(prev => prev + 1);
      
      // Notify other components
      try {
        localStorage.setItem('datasetRefresh', Date.now().toString());
      } catch {}
    } catch (e: any) {
      setError(e?.message || 'Save failed');
    }
  };

  const renderPreviewLines = () => {
    const canvas = canvasRef.current;
    if (!canvas || canvas.width === 0 || canvas.height === 0) return [] as string[];
    return boxes.map(b => {
      const x = (b.x + Math.abs(b.w)/2) / canvas.width;
      const y = (b.y + Math.abs(b.h)/2) / canvas.height;
      const w = Math.abs(b.w) / canvas.width;
      const h = Math.abs(b.h) / canvas.height;
      return `${b.class_id} ${x.toFixed(6)} ${y.toFixed(6)} ${w.toFixed(6)} ${h.toFixed(6)}`;
    });
  };

  return (
    <>
    <div className="p-4">
      <div className="flex items-center justify-between mb-3">
        <h3 className="text-white text-lg font-semibold">Dataset Labeler</h3>
        <div className="flex items-center gap-2">
          <button className="px-2 py-1 rounded bg-slate-700 text-white" title="Labeling Best Practices" onClick={()=>{
            try {
              window.open('/rdk_weld_evaluator/AI_MODEL_TRAINING_GUIDE.md','_blank');
            } catch {}
          }}>Open Guide</button>
          <button className="px-2 py-1 rounded bg-slate-700 text-white" title="Show help (?)" onClick={()=>setShowHelp(true)}>?</button>
        </div>
      </div>
      <div className="grid md:grid-cols-2 gap-4">
        <div className="bg-slate-800 border border-slate-700 rounded p-3">
          <div className="mb-2">
            <label className="block text-slate-300 text-sm mb-1">Image URL or Data URI</label>
            <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={imageSrc} onChange={e=>setImageSrc(e.target.value)} placeholder="Paste image URL or data URI" />
          </div>
          <div className="border border-slate-600 rounded p-2 bg-white">
            <canvas ref={canvasRef} className="max-w-full h-auto" onMouseDown={onMouseDown} onMouseMove={onMouseMove} onMouseUp={onMouseUp} />
          </div>
        </div>
        <div className="bg-slate-800 border border-slate-700 rounded p-3">
          <div className="mb-3">
            <label className="block text-slate-300 text-sm mb-1">Image Name</label>
            <div className="flex items-center gap-2">
              <input className="flex-1 bg-slate-900 border border-slate-700 rounded px-3 py-2 text-slate-200 font-mono text-sm" readOnly value={imageName} title="Image Name" />
              <div className="text-xs text-slate-400 px-2 py-1 bg-slate-900 rounded border border-slate-700">Read-only</div>
            </div>
            <p className="text-xs text-slate-500 mt-1">💡 To delete images, use the Data Capture tab</p>
          </div>
          <label className="block text-slate-300 text-sm mt-2 mb-1">Class</label>
          <select className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={classId} onChange={e=>setClassId(Number(e.target.value))} title="Class">
            {classNames.map((n, idx) => (
              <option key={n} value={idx}>{idx} - {n}</option>
            ))}
          </select>
          
          <div className="flex gap-2 mt-3">
            <button className="flex-1 px-3 py-1.5 rounded bg-industrial-blue text-white hover:bg-blue-600" title="Save (S)" onClick={saveLabels}>Save Image + Labels</button>
            <button className="px-3 py-1.5 rounded bg-slate-700 text-white hover:bg-slate-600" title="Clear all boxes" onClick={() => { setBoxes([]); drawAll(); }}>Clear Boxes</button>
          </div>
          
          {boxes.length === 0 && (
            <button 
              className="w-full mt-2 px-3 py-1.5 rounded bg-red-700 text-white hover:bg-red-600" 
              title="Delete label file to re-classify"
              onClick={async () => {
                if (!confirm(`Delete label for ${imageName}? You can then re-classify it.`)) return;
                try {
                  setError('');
                  const lblPath = `${scanInfo.labelDir}/${imageName.replace(/\.(jpg|jpeg|png)$/i, '.txt')}`.replace(/\\/g, '/');
                  const resp = await deleteDatasetImage('', lblPath);
                  if (resp.error) throw new Error(resp.error);
                  setMessage('Label deleted - ready to re-classify');
                  setRefreshTrigger(prev => prev + 1);
                  try {
                    localStorage.setItem('datasetRefresh', Date.now().toString());
                  } catch {}
                } catch (e: any) {
                  setError(e?.message || 'Delete failed');
                }
              }}
            >
              🗑️ Delete Label (Re-classify)
            </button>
          )}
          <button className="mt-2 ml-2 px-3 py-1.5 rounded bg-slate-700 text-white disabled:opacity-50 disabled:cursor-not-allowed" title="Jump to previous unlabeled image" disabled={cursor<=0 || labelsMissing.length===0} onClick={()=>{
            if (labelsMissing.length > 0) {
              const prev = Math.max(0, cursor - 1);
              setCursor(prev);
              const stem = labelsMissing[prev].replace('.txt','');
              const pick = images.find(f => f.startsWith(stem));
              if (pick && scanInfo.imageDir) {
                setImageName(pick);
                setImageSrc(getImageUrl(scanInfo.imageDir, pick));
              }
            }
          }}>Previous (←)</button>
          <button className="mt-2 ml-2 px-3 py-1.5 rounded bg-slate-700 text-white disabled:opacity-50 disabled:cursor-not-allowed" title="Skip to next unlabeled image" disabled={labelsMissing.length===0 || cursor>=labelsMissing.length-1} onClick={()=>{
            if (labelsMissing.length > 0) {
              const next = Math.min(labelsMissing.length - 1, cursor + 1);
              setCursor(next);
              const stem = labelsMissing[next].replace('.txt','');
              const pick = images.find(f => f.startsWith(stem));
              if (pick && scanInfo.imageDir) {
                setImageName(pick);
                setImageSrc(getImageUrl(scanInfo.imageDir, pick));
              }
            }
          }}>Skip (→/N)</button>
          <span className="ml-3 text-slate-300 text-sm" title="Current unlabeled index">
            {labelsMissing.length > 0 ? `${Math.min(cursor+1, labelsMissing.length)} of ${labelsMissing.length}` : '0 of 0'}
          </span>
          <span className="ml-3 text-slate-500 text-xs" title="Shortcuts help">
            Shortcuts: Save (S), Previous (←), Skip (→/N)
          </span>
          <button className="mt-2 ml-2 px-2 py-1 rounded bg-slate-700 text-white" title="Show help (?)" onClick={()=>setShowHelp(true)}>?</button>
          <div className="mt-3">
            <label className="block text-slate-300 text-sm mb-1">Label Format Preview</label>
            <div className="bg-slate-900 border border-slate-700 rounded p-2 text-slate-200 text-xs font-mono max-h-32 overflow-auto">
              {renderPreviewLines().length === 0 ? (
                <div className="text-slate-500">No boxes yet. Draw to preview.</div>
              ) : (
                renderPreviewLines().map((line, idx) => (
                  <div key={idx}>{line}</div>
                ))
              )}
            </div>
          </div>
          <div className="mt-3 text-slate-300 text-sm">
            <div className="flex justify-between mb-3">
              <div>Total: <span className="text-slate-100 font-mono font-bold">{images.length}</span></div>
              <div>Labeled: <span className="text-green-400 font-mono font-bold">{images.length - labelsMissing.length}</span></div>
              <div>Unlabeled: <span className="text-amber-400 font-mono font-bold">{labelsMissing.length}</span></div>
            </div>
            
            {/* Unified Image List */}
            {images.length > 0 && (
              <div>
                <div className="font-semibold mb-2 text-slate-200 flex items-center justify-between">
                  <span>📁 Dataset Images ({images.length})</span>
                  <span className="text-xs text-slate-500">Click to load & label</span>
                </div>
                <div className="max-h-96 overflow-auto border border-slate-700 rounded bg-slate-900">
                  {images.map((img) => {
                    const hasLabel = !labelsMissing.some(m => m.replace('.txt', '') === img.replace(/\.(jpg|jpeg|png)$/i, ''));
                    const labelInfo = labeledImagesInfo.find(info => info.name === img);
                    const isSelected = imageName === img;
                    
                    return (
                      <div
                        key={img}
                        className={`px-3 py-2.5 hover:bg-slate-800 cursor-pointer border-b border-slate-800 last:border-b-0 transition-colors ${
                          isSelected ? 'bg-slate-700 border-l-4 border-l-blue-400' : ''
                        }`}
                        onClick={() => {
                          if (scanInfo.imageDir) {
                            setImageName(img);
                            setImageSrc(getImageUrl(scanInfo.imageDir, img));
                          }
                        }}
                      >
                        <div className="flex items-center justify-between gap-3">
                          <div className="flex items-center gap-3 flex-1 min-w-0">
                            <div className={`w-3 h-3 rounded-full flex-shrink-0 ${
                              hasLabel ? 'bg-green-400' : 'bg-amber-400 animate-pulse'
                            }`} />
                            <span className="text-slate-200 text-sm font-mono truncate">{img}</span>
                          </div>
                          <div className="flex items-center gap-2 flex-shrink-0">
                            {hasLabel ? (
                              <>
                                {labelInfo && labelInfo.classes.length > 0 ? (
                                  <div className="flex gap-1 flex-wrap">
                                    {labelInfo.classes.map((cls, idx) => (
                                      <span key={idx} className="text-xs px-2 py-0.5 rounded-full bg-green-700 text-white font-medium">
                                        {cls}
                                      </span>
                                    ))}
                                  </div>
                                ) : (
                                  <span className="text-xs px-2 py-1 rounded-full bg-green-700/30 text-green-300 border border-green-700 font-medium">
                                    ✓ Labeled
                                  </span>
                                )}
                              </>
                            ) : (
                              <span className="text-xs px-2 py-1 rounded-full bg-amber-700/30 text-amber-300 border border-amber-700 font-medium">
                                ⚠ Needs Label
                              </span>
                            )}
                          </div>
                        </div>
                      </div>
                    );
                  })}
                </div>
              </div>
            )}
            
            <div className="mt-4">
              <label className="block text-slate-300 text-sm mb-1">Class Names (comma-separated)</label>
            <input className="w-full bg-slate-900 border border-slate-700 rounded px-2 py-1 text-slate-200" value={classNames.join(',')} onChange={e=>setClassNames(e.target.value.split(',').map(s=>s.trim()).filter(Boolean))} placeholder="good,porosity,undercut,spatter" title="Class Names" />
            <button className="mt-2 px-3 py-1.5 rounded bg-slate-700 text-white" onClick={async ()=>{
              try {
                const resp = await generateDatasetYaml(datasetDir, classNames);
                if (resp.error) throw new Error(resp.error);
                setMessage(`Generated dataset.yaml at ${resp.yamlPath}`);
              } catch (e: any) {
                setError(e?.message || 'YAML generation failed');
              }
            }}>Generate dataset.yaml</button>
            </div>
            
            {message && <div className="mt-2 p-2 bg-green-900 text-green-100 border border-green-800 rounded">{message}</div>}
            {error && <div className="mt-2 p-2 bg-red-900 text-red-100 border border-red-800 rounded">{error}</div>}
            <p className="text-slate-400 text-sm mt-2">Labels saved in YOLO format: <code>class x y w h</code> normalized to 0..1.</p>
          </div>
        </div>
      </div>
    </div>
    {showHelp && (
      <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-50" onClick={()=>setShowHelp(false)}>
        <div className="bg-slate-800 border border-slate-700 rounded p-4 w-full max-w-md" onClick={e=>e.stopPropagation()}>
          <div className="flex justify-between items-center mb-2">
            <h4 className="text-white text-lg font-semibold">Labeler Shortcuts</h4>
            <button className="px-2 py-1 rounded bg-slate-700 text-white" onClick={()=>setShowHelp(false)}>Close</button>
          </div>
          <ul className="text-slate-200 text-sm space-y-1">
            <li><span className="font-mono">S</span>: Save Image + Labels</li>
            <li><span className="font-mono">←</span>: Previous unlabeled</li>
            <li><span className="font-mono">→</span> or <span className="font-mono">N</span>: Next/Skip unlabeled</li>
            <li><span className="font-mono">?</span>: Toggle this help</li>
            <li>Draw box: click-drag on the image canvas</li>
            <li>Class: choose from the dropdown before drawing</li>
          </ul>
          <hr className="my-3 border-slate-700" />
          <h5 className="text-white text-sm font-semibold mb-1">Labeling Do’s</h5>
          <ul className="text-slate-200 text-sm list-disc ml-5 space-y-1">
            <li>Include the full object in the box; avoid cut-offs.</li>
            <li>Annotate every visible instance of the target class.</li>
            <li>Keep class selection consistent across similar cases.</li>
            <li>Use tight boxes with minimal background.</li>
            <li>Review saved label text for expected format (class x y w h).</li>
          </ul>
          <h5 className="text-white text-sm font-semibold mb-1 mt-3">Labeling Don’ts</h5>
          <ul className="text-slate-200 text-sm list-disc ml-5 space-y-1">
            <li>Don’t overlap boxes unnecessarily for a single object.</li>
            <li>Don’t mix up classes; avoid ambiguous labeling.</li>
            <li>Don’t skip small objects if they’re relevant.</li>
            <li>Don’t include large background margins inside boxes.</li>
          </ul>
          <div className="mt-3">
            <button className="px-2 py-1 rounded bg-slate-700 text-white" onClick={()=>{
              try { window.open('/AI_TRAINING_QUICK_START.md','_blank'); } catch {}
            }}>Open Quick Checklist</button>
          </div>
        </div>
      </div>
    )}
    </>
  );
}

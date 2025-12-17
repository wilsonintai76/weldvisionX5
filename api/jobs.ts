export async function startTrain(datasetPath: string, epochs = 10) {
  const res = await fetch("/api/train/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ datasetPath, epochs }),
  });
  return res.json(); // { job_id }
}

export async function trainStatus(jobId: string) {
  const res = await fetch(`/api/train/status/${jobId}`);
  return res.json(); // { state, logs }
}

export async function startCompile(onnxPath: string, outputDir: string) {
  const res = await fetch("/api/compile/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ onnxPath, outputDir }),
  });
  return res.json();
}

export async function compileStatus(jobId: string) {
  const res = await fetch(`/api/compile/status/${jobId}`);
  return res.json();
}

export async function cancelTrain(jobId: string) {
  const res = await fetch(`/api/train/cancel/${jobId}`, { method: "POST" });
  return res.json();
}

export async function cancelCompile(jobId: string) {
  const res = await fetch(`/api/compile/cancel/${jobId}`, { method: "POST" });
  return res.json();
}

export async function deployModel(deviceHost: string, deviceUser: string, modelBin: string, destPath: string) {
  const res = await fetch("/api/models/deploy", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ deviceHost, deviceUser, modelBin, destPath }),
  });
  return res.json();
}

export async function startInference(deviceHost: string, deviceUser: string, modelBin?: string, scriptPath?: string) {
  const res = await fetch("/api/inference/start", { 
    method: "POST", 
    headers: { "Content-Type": "application/json" }, 
    body: JSON.stringify({ rdkIp: deviceHost, user: deviceUser, modelBin, scriptPath }) 
  });
  return res.json();
}

export async function stopInference(deviceHost: string, deviceUser: string) {
  const res = await fetch("/api/inference/stop", { 
    method: "POST", 
    headers: { "Content-Type": "application/json" }, 
    body: JSON.stringify({ rdkIp: deviceHost, user: deviceUser }) 
  });
  return res.json();
}

export async function uploadDatasetImage(imageBase64: string, savePath: string, labelPath?: string, labels?: Array<{class_id: number; bbox: number[];}>) {
  const res = await fetch("/api/dataset/upload", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ imageBase64, savePath, labelPath, labels })
  });
  if (!res.ok) {
    const text = await res.text();
    try {
      const errorData = JSON.parse(text);
      return { error: errorData.error || `HTTP ${res.status}` };
    } catch {
      return { error: `HTTP ${res.status}: ${text || 'Unknown error'}` };
    }
  }
  return res.json();
}

export async function generateDatasetYaml(baseDir: string, names: string[], trainRel = 'images', valRel = 'images') {
  const res = await fetch("/api/dataset/generate_yaml", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ baseDir, names, trainRel, valRel })
  });
  if (!res.ok) {
    const text = await res.text();
    try {
      const errorData = JSON.parse(text);
      return { error: errorData.error || `HTTP ${res.status}` };
    } catch {
      return { error: `HTTP ${res.status}: ${text || 'Unknown error'}` };
    }
  }
  return res.json();
}

export async function listDatasetFiles(basePath: string, imageSubdir = 'images/train', labelSubdir = 'labels/train') {
  const res = await fetch('/api/dataset/list', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ base_path: basePath, image_subdir: imageSubdir, label_subdir: labelSubdir })
  });
  if (!res.ok) {
    const text = await res.text();
    try {
      const errorData = JSON.parse(text);
      return { error: errorData.error || `HTTP ${res.status}`, images: [], labels: [], labels_missing: [] };
    } catch {
      return { error: `HTTP ${res.status}: ${text || 'Unknown error'}`, images: [], labels: [], labels_missing: [] };
    }
  }
  return res.json();
}

export async function deleteDatasetImage(imagePath: string, labelPath?: string) {
  const res = await fetch('/api/dataset/delete', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ imagePath, labelPath })
  });
  if (!res.ok) {
    const text = await res.text();
    try {
      const errorData = JSON.parse(text);
      return { error: errorData.error || `HTTP ${res.status}` };
    } catch {
      return { error: `HTTP ${res.status}: ${text || 'Unknown error'}` };
    }
  }
  return res.json();
}

export async function readLabelFile(labelPath: string) {
  const res = await fetch('/api/dataset/label/read', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ labelPath })
  });
  if (!res.ok) {
    const text = await res.text();
    try {
      const errorData = JSON.parse(text);
      return { error: errorData.error || `HTTP ${res.status}` };
    } catch {
      return { error: `HTTP ${res.status}: ${text || 'Unknown error'}` };
    }
  }
  return res.json();
}

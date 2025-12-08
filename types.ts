export interface Student {
  id: number;
  name: string;
  student_id: string; // unique
  class_name: string;
  level: 'Novice' | 'Intermediate' | 'Advanced';
}

export interface WeldingMetrics {
  width_val: number; // Target 8mm
  height_val: number; // Target 2mm
  uniformity_score: number; // Derived from std dev
  porosity_count: number;
  spatter_count: number;
  undercut_detected: boolean;
}

export interface ScanResult {
  id: number;
  student_id: number;
  timestamp: string;
  total_score: number;
  metrics: WeldingMetrics;
  defects_json: string[]; // e.g., ["Porosity", "Undercut"]
  image_path: string;
  status: 'Pass' | 'Fail';
}

export enum ViewState {
  DASHBOARD = 'DASHBOARD',
  STUDENTS = 'STUDENTS',
  SCANNER = 'SCANNER',
  HISTORY = 'HISTORY',
  CALIBRATION = 'CALIBRATION',
  BED_TILT_CALIBRATION = 'BED_TILT_CALIBRATION',
  MANUAL_BED_CALIBRATION = 'MANUAL_BED_CALIBRATION',
  STEREO_CALIBRATION = 'STEREO_CALIBRATION',
  PANORAMA_SCANNER = 'PANORAMA_SCANNER',
  SAFE_MOTION = 'SAFE_MOTION',
  INFERENCE_MONITOR = 'INFERENCE_MONITOR',
  TRAINING_DASHBOARD = 'TRAINING_DASHBOARD',
  MODEL_MANAGEMENT = 'MODEL_MANAGEMENT',
  SETTINGS = 'SETTINGS',
}

export enum RigType {
  BASIC_RIG = 'BASIC_RIG',
  ADVANCED_RIG = 'ADVANCED_RIG',
}

export interface RubricConfig {
  name?: string;
  targetWidth: number;
  widthTolerance: number;
  targetHeight: number;
  heightTolerance: number;
  maxPorosity: number;
  maxSpatter: number;
}

// Training and Model Interfaces
export interface TrainingConfig {
  epochs: number;
  batch_size: number;
  learning_rate: number;
  validation_split?: number;
  early_stopping_patience?: number;
}

export interface TrainingJob {
  id: string;
  job_id: string;
  model_name: string;
  status: 'running' | 'completed' | 'failed' | 'cancelled';
  progress: number;
  epoch: number;
  total_epochs: number;
  loss: number;
  accuracy: number;
  created_at: string;
  started_at?: string;
  completed_at?: string;
}

export interface ModelMetadata {
  id: string;
  model_name: string;
  version: string;
  accuracy: number;
  loss: number;
  f1_score: number;
  created_at: string;
  updated_at: string;
  description?: string;
  tags?: string[];
  model_path: string;
  deployed: boolean;
}

export interface InferenceResult {
  class: string; // 'good', 'porosity', 'undercut'
  confidence: number;
  method: 'rule-based' | 'ml' | 'hybrid';
  timestamp: string;
  inference_time?: number; // milliseconds
}

export interface InferenceStats {
  total_inferences: number;
  avg_inference_time: number;
  success_rate: number;
  detections_by_class: {
    [key: string]: number;
  };
  last_inference?: InferenceResult;
}
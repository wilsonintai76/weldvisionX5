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
  SETTINGS = 'SETTINGS',
}

export enum RigType {
  MANUAL_HEIGHT = 'MANUAL_HEIGHT',
  THREE_AXIS_PANORAMA = 'THREE_AXIS_PANORAMA',
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
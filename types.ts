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
  SETTINGS = 'SETTINGS',
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
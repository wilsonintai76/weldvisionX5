import { Student, ScanResult, WeldingMetrics, RubricConfig } from '../types';
import { MOCK_STUDENTS, DEFAULT_RUBRIC, WELD_RESULT_PLACEHOLDER } from '../constants';

// Simulating database storage in memory for the session
let students = [...MOCK_STUDENTS];
let scans: ScanResult[] = [];
let activeRubric: RubricConfig = { ...DEFAULT_RUBRIC };

const delay = (ms: number) => new Promise(resolve => setTimeout(resolve, ms));

export const fetchStudents = async (): Promise<Student[]> => {
  await delay(500);
  return students;
};

export const addStudent = async (student: Omit<Student, 'id'>): Promise<Student> => {
  await delay(500);
  const newStudent = { ...student, id: Math.floor(Math.random() * 10000) };
  students.push(newStudent);
  return newStudent;
};

export const updateStudent = async (id: number, updates: Partial<Student>): Promise<Student> => {
  await delay(500);
  const index = students.findIndex(s => s.id === id);
  if (index === -1) throw new Error("Student not found");
  
  students[index] = { ...students[index], ...updates };
  return students[index];
};

export const deleteStudent = async (id: number): Promise<void> => {
  await delay(500);
  students = students.filter(s => s.id !== id);
};

export const fetchHistory = async (): Promise<ScanResult[]> => {
  await delay(600);
  return scans.sort((a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime());
};

export const getRubric = async (): Promise<RubricConfig> => {
  await delay(300);
  return activeRubric;
};

export const saveRubric = async (config: RubricConfig): Promise<RubricConfig> => {
  await delay(500);
  activeRubric = config;
  return activeRubric;
};

export const triggerCalibration = async (): Promise<{ matrix: number[][], distCoeffs: number[], error: number }> => {
  await delay(3000); // Simulate capturing multiple frames and processing
  
  // Simulate random hardware failure (20% chance)
  if (Math.random() > 0.8) {
    throw new Error("Camera synchronization timeout. Ensure stereo pair is connected.");
  }

  return {
    matrix: [
      [1024.5, 0, 640.0],
      [0, 1024.5, 360.0],
      [0, 0, 1]
    ],
    distCoeffs: [-0.102, 0.045, -0.001, 0.002, 0],
    error: 0.23
  };
};

export const saveCalibration = async (data: { matrix: number[][], distCoeffs: number[] }): Promise<void> => {
  await delay(800);
  
  // Simulate random write failure (10% chance)
  if (Math.random() > 0.9) {
    throw new Error("Write permission denied: /config/calibration.yaml");
  }

  console.log("Calibration data persisted to config.yaml", data);
};

// This function simulates the "Vision Pipeline" described in the prompt
export const triggerScan = async (studentId: number): Promise<ScanResult> => {
  await delay(2500); // Simulate processing time on RDK X5 BPU

  // Generate semi-random metrics to simulate real-world data
  // Biased slightly towards "Good" results but with variance
  const isGoodScan = Math.random() > 0.3; 
  
  const widthVal = isGoodScan 
    ? activeRubric.targetWidth + (Math.random() * 1 - 0.5) 
    : activeRubric.targetWidth + (Math.random() * 4 - 2);     

  const heightVal = isGoodScan
    ? activeRubric.targetHeight + (Math.random() * 0.4 - 0.2)
    : activeRubric.targetHeight + (Math.random() * 2 - 1);       

  const porosityCount = isGoodScan ? 0 : Math.floor(Math.random() * 3);
  const spatterCount = isGoodScan ? Math.floor(Math.random() * 2) : Math.floor(Math.random() * 8);
  const undercutDetected = !isGoodScan && Math.random() > 0.5;

  const metrics: WeldingMetrics = {
    width_val: parseFloat(widthVal.toFixed(2)),
    height_val: parseFloat(heightVal.toFixed(2)),
    uniformity_score: isGoodScan ? 0.95 : 0.7, // Lower is strictly worse for std dev, but here we model score
    porosity_count: porosityCount,
    spatter_count: spatterCount,
    undercut_detected: undercutDetected,
  };

  const defects = [];
  if (porosityCount > activeRubric.maxPorosity) defects.push(`Porosity (${porosityCount})`);
  if (spatterCount > activeRubric.maxSpatter) defects.push(`Excessive Spatter (${spatterCount})`);
  if (undercutDetected) defects.push("Undercut");
  
  if (widthVal < activeRubric.targetWidth - activeRubric.widthTolerance || 
      widthVal > activeRubric.targetWidth + activeRubric.widthTolerance) {
      defects.push("Width Out of Spec");
  }
  
  if (heightVal < activeRubric.targetHeight - activeRubric.heightTolerance || 
      heightVal > activeRubric.targetHeight + activeRubric.heightTolerance) {
      defects.push("Height Out of Spec");
  }

  // Calculate simple score
  let score = 100;
  score -= defects.length * 15; // Adjusted penalty
  if (score < 0) score = 0;

  const result: ScanResult = {
    id: Math.floor(Math.random() * 100000),
    student_id: studentId,
    timestamp: new Date().toISOString(),
    total_score: score,
    metrics: metrics,
    defects_json: defects,
    image_path: WELD_RESULT_PLACEHOLDER,
    status: defects.length === 0 ? 'Pass' : 'Fail'
  };

  scans.push(result);
  return result;
};
import { Student, ScanResult, WeldingMetrics, RubricConfig } from '../types';

// API Base URL - can be configured via environment variable
const API_BASE_URL = (import.meta as any).env?.VITE_API_URL || 'http://localhost:5000';

// Helper function for API calls
const apiCall = async <T,>(
  endpoint: string,
  method: 'GET' | 'POST' | 'PUT' | 'DELETE' = 'GET',
  data?: Record<string, any>
): Promise<T> => {
  const url = `${API_BASE_URL}${endpoint}`;
  const options: RequestInit = {
    method,
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include',
  };

  if (data) {
    options.body = JSON.stringify(data);
  }

  try {
    const response = await fetch(url, options);
    
    if (!response.ok) {
      const error = await response.json().catch(() => ({ error: response.statusText }));
      throw new Error(error.error || `HTTP ${response.status}: ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error(`API Error [${method} ${endpoint}]:`, error);
    throw error;
  }
};

/**
 * Fetch all students from backend
 */
export const fetchStudents = async (): Promise<Student[]> => {
  return apiCall<Student[]>('/api/students', 'GET');
};

/**
 * Add a new student
 */
export const addStudent = async (student: Omit<Student, 'id'>): Promise<Student> => {
  const response = await apiCall<{ id: number }>('/api/students', 'POST', student);
  return { ...student, id: response.id };
};

/**
 * Update a student
 */
export const updateStudent = async (id: number, updates: Partial<Student>): Promise<Student> => {
  return apiCall<Student>(`/api/students/${id}`, 'PUT', updates);
};

/**
 * Delete a student
 */
export const deleteStudent = async (id: number): Promise<void> => {
  await apiCall(`/api/students/${id}`, 'DELETE');
};

/**
 * Fetch scan history
 */
export const fetchHistory = async (): Promise<ScanResult[]> => {
  return apiCall<ScanResult[]>('/api/scans', 'GET');
};

/**
 * Trigger a scan for a student
 */
export const triggerScan = async (studentId: number, rubric: RubricConfig): Promise<ScanResult> => {
  return apiCall<ScanResult>('/api/scan', 'POST', {
    student_id: studentId,
    rubric,
  });
};

/**
 * Get current rubric configuration
 */
export const getRubric = async (): Promise<RubricConfig> => {
  return apiCall<RubricConfig>('/api/rubric', 'GET');
};

/**
 * Save rubric configuration
 */
export const saveRubric = async (config: RubricConfig): Promise<RubricConfig> => {
  return apiCall<RubricConfig>('/api/rubric', 'POST', config);
};

/**
 * Trigger camera calibration
 */
export const triggerCalibration = async (): Promise<{
  matrix: number[][];
  distCoeffs: number[];
  error: number;
}> => {
  return apiCall('/api/calibrate', 'POST');
};

/**
 * Save calibration data
 */
export const saveCalibration = async (data: {
  matrix: number[][];
  distCoeffs: number[];
}): Promise<void> => {
  await apiCall('/api/calibrate/save', 'POST', data);
};

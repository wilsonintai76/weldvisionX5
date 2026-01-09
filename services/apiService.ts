import { Student, ScanResult, WeldingMetrics, RubricConfig, Class } from '../types';

// API Base URL - can be configured via environment variable
// - In dev, prefer relative URLs so Vite can proxy `/api` to Django.
// - In production/Electron, set `VITE_API_URL` (e.g. http://localhost:8000).
const API_BASE_URL = ((import.meta as any).env?.VITE_API_URL as string | undefined)?.replace(/\/+$/, '') || '';
const DEFAULT_RDK_HOST = 'rdk-x5.local';
const DEFAULT_RDK_PORT = 5001;

const getRdkBaseUrl = (): string => {
  try {
    const saved = JSON.parse(localStorage.getItem('orchestrationSettings') || '{}');
    const host = (saved?.deviceHost as string | undefined) || DEFAULT_RDK_HOST;
    const port = Number(saved?.streamPort || DEFAULT_RDK_PORT);
    return `http://${host}:${port}`;
  } catch {
    return `http://${DEFAULT_RDK_HOST}:${DEFAULT_RDK_PORT}`;
  }
};

// RDK Helper
const rdkCall = async (endpoint: string, method: 'GET' | 'POST' = 'GET') => {
  try {
    const response = await fetch(`${getRdkBaseUrl()}${endpoint}`, { method });
    if (!response.ok) throw new Error('RDK Error');
    return await response.json();
  } catch (e) {
    console.error("RDK Connection Failed", e);
    throw e;
  }
};

export const startRDKScan = () => rdkCall('/start', 'POST');
export const stopRDKScan = () => rdkCall('/stop', 'POST');
export const getRDKStatus = () => rdkCall('/status', 'GET');

// Helper function for API calls
const apiCall = async <T,>(
  endpoint: string,
  method: 'GET' | 'POST' | 'PUT' | 'DELETE' = 'GET',
  data?: Record<string, any>
): Promise<T> => {
  const url = `${API_BASE_URL}${endpoint.startsWith('/') ? endpoint : `/${endpoint}`}`;
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
  return apiCall<Student[]>('/api/students/', 'GET');
};

/**
 * Add a new student
 */
export const addStudent = async (student: Omit<Student, 'id'>): Promise<Student> => {
  return apiCall<Student>('/api/students/', 'POST', student);
};

/**
 * Update a student
 */
export const updateStudent = async (id: number, updates: Partial<Student>): Promise<Student> => {
  return apiCall<Student>(`/api/students/${id}/`, 'PUT', updates);
};

/**
 * Delete a student
 */
export const deleteStudent = async (id: number): Promise<void> => {
  await apiCall(`/api/students/${id}/`, 'DELETE');
};

/**
 * Fetch all classes
 */
export const fetchClasses = async (): Promise<Class[]> => {
  return apiCall<Class[]>('/api/classes/', 'GET');
};

/**
 * Add a new class
 */
export const addClass = async (cls: Omit<Class, 'id' | 'created_at' | 'updated_at'>): Promise<Class> => {
  return apiCall<Class>('/api/classes/', 'POST', cls);
};

/**
 * Update a class
 */
export const updateClass = async (id: number, updates: Partial<Class>): Promise<Class> => {
  return apiCall<Class>(`/api/classes/${id}/`, 'PUT', updates);
};

/**
 * Delete a class
 */
export const deleteClass = async (id: number): Promise<void> => {
  await apiCall(`/api/classes/${id}/`, 'DELETE');
};

/**
 * Fetch scan history
 */
export const fetchHistory = async (): Promise<ScanResult[]> => {
  return apiCall<ScanResult[]>('/api/scans/', 'GET');
};

/**
 * Trigger a scan for a student
 */
export const triggerScan = async (studentId: number, rubric: RubricConfig): Promise<ScanResult> => {
  return apiCall<ScanResult>('/api/scan/', 'POST', {
    student_id: studentId,
    rubric,
  });
};

/**
 * Get current rubric configuration
 */
export const getRubric = async (): Promise<RubricConfig> => {
  return apiCall<RubricConfig>('/api/rubric/', 'GET');
};

/**
 * Save rubric configuration
 */
export const saveRubric = async (config: RubricConfig): Promise<RubricConfig> => {
  return apiCall<RubricConfig>('/api/rubric/', 'POST', config);
};

/**
 * Trigger camera calibration
 */
export const triggerCalibration = async (): Promise<{
  matrix: number[][];
  distCoeffs: number[];
  error: number;
}> => {
  return apiCall('/api/calibrate/', 'POST');
};

/**
 * Save calibration data
 */
export const saveCalibration = async (data: {
  matrix: number[][];
  distCoeffs: number[];
}): Promise<void> => {
  await apiCall('/api/calibrate/save/', 'POST', data);
};

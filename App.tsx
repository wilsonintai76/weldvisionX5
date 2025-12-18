import React, { useState, useEffect, useCallback, useRef } from 'react';
import { 
  LayoutDashboard, 
  ScanLine, 
  Users, 
  History, 
  Camera, 
  Activity, 
  Zap, 
  CheckCircle2, 
  AlertOctagon,
  ChevronRight,
  Plus,
  XCircle,
  Pencil,
  Trash2,
  AlertTriangle,
  Grid3x3,
  RefreshCw,
  Save,
  RotateCcw,
  Settings,
  Sliders,
  Ruler,
  Sparkles,
  Search,
  Calendar,
  ArrowUpDown,
  ChevronLeft,
  Filter,
  HelpCircle,
  Brain,
  BarChart3,
  Database,
  Network
} from 'lucide-react';
import { 
  BarChart, 
  Bar, 
  XAxis, 
  YAxis, 
  CartesianGrid, 
  Tooltip, 
  ResponsiveContainer, 
  LineChart, 
  Line 
} from 'recharts';

import { Student, ScanResult, ViewState, WeldingMetrics, RubricConfig, RigType } from './types';
import { RUBRIC_PRESETS, CAMERA_FEED_PLACEHOLDER } from './constants';
import { fetchStudents, fetchHistory, triggerScan, addStudent, updateStudent, deleteStudent, triggerCalibration, saveCalibration, getRubric, saveRubric } from './services/apiService';
import { MetricCard } from './components/MetricCard';
import UserGuide from './components/UserGuide';
import StereoCameraCalibration from './components/StereoCameraCalibration';
import BedCalibration from './components/BedCalibration';
import ManualBedCalibration from './components/ManualBedCalibration';
import InferenceMonitor from './components/InferenceMonitor';
import TrainingDashboard from './components/TrainingDashboard';
import ModelManagement from './components/ModelManagement';
import OrchestrationPanel from './components/OrchestrationPanel';
import DatasetCapture from './components/DatasetCapture';
import DatasetLabeler from './components/DatasetLabeler';
import DatasetStudio from './components/DatasetStudio';
import './styles/progress.css';
import { listDatasetFiles } from './api/jobs';

// --- Sub-Components ---

const SidebarItem = ({ 
  icon: Icon, 
  label, 
  active, 
  onClick 
}: { 
  icon: React.ElementType, 
  label: string, 
  active: boolean, 
  onClick: () => void 
}) => (
  <button 
    onClick={onClick}
    className={`flex items-center w-full p-3 mb-2 rounded-lg transition-colors ${
      active 
        ? 'bg-industrial-blue text-white shadow-lg shadow-industrial-blue/20' 
        : 'text-slate-400 hover:bg-slate-800 hover:text-white'
    }`}
  >
    <Icon className="w-5 h-5 mr-3" />
    <span className="font-medium">{label}</span>
  </button>
);

const EmptyState = ({ message }: { message: string }) => (
  <div className="flex flex-col items-center justify-center h-64 text-slate-500 border-2 border-dashed border-slate-800 rounded-xl">
    <ScanLine className="w-12 h-12 mb-4 opacity-50" />
    <p>{message}</p>
  </div>
);

// --- Main Views ---

const DashboardView = ({ 
  scans, 
  students, 
  onNavigate 
}: { 
  scans: ScanResult[], 
  students: Student[], 
  onNavigate: (view: ViewState) => void 
}) => {
  const avgScore = scans.length > 0 
    ? Math.round(scans.reduce((acc, s) => acc + s.total_score, 0) / scans.length) 
    : 0;
  
  const passRate = scans.length > 0 
    ? Math.round((scans.filter(s => s.status === 'Pass').length / scans.length) * 100)
    : 0;

  const recentScans = scans.slice(0, 5);

  return (
    <div className="space-y-6">
      <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        <div className="bg-slate-800 p-6 rounded-2xl border border-slate-700 shadow-xl">
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-slate-400 font-medium">Class Average</h3>
            <Activity className="text-industrial-blue w-5 h-5" />
          </div>
          <p className="text-4xl font-bold text-white">{avgScore}</p>
          <p className="text-sm text-slate-500 mt-2">Based on {scans.length} scans</p>
        </div>
        <div className="bg-slate-800 p-6 rounded-2xl border border-slate-700 shadow-xl">
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-slate-400 font-medium">Pass Rate</h3>
            <CheckCircle2 className="text-industrial-success w-5 h-5" />
          </div>
          <p className="text-4xl font-bold text-white">{passRate}%</p>
          <p className="text-sm text-slate-500 mt-2">{scans.filter(s => s.status === 'Pass').length} passed / {scans.length} total</p>
        </div>
        <div className="bg-slate-800 p-6 rounded-2xl border border-slate-700 shadow-xl cursor-pointer hover:bg-slate-750 transition-colors" onClick={() => onNavigate(ViewState.STUDENTS)}>
          <div className="flex items-center justify-between mb-4">
            <h3 className="text-slate-400 font-medium">Active Students</h3>
            <Users className="text-industrial-orange w-5 h-5" />
          </div>
          <p className="text-4xl font-bold text-white">{students.length}</p>
          <div className="flex items-center mt-2 text-industrial-blue text-sm">
            <span>Manage Class</span>
            <ChevronRight className="w-4 h-4 ml-1" />
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        <div className="lg:col-span-2 bg-slate-800 p-6 rounded-2xl border border-slate-700">
          <h3 className="text-lg font-semibold text-white mb-6">Performance Trend</h3>
          <div className="h-64">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={recentScans.reverse()}>
                <CartesianGrid strokeDasharray="3 3" stroke="#334155" />
                <XAxis dataKey="id" stroke="#94a3b8" tickFormatter={(v) => `#${v}`} />
                <YAxis stroke="#94a3b8" domain={[0, 100]} />
                <Tooltip 
                  contentStyle={{ backgroundColor: '#1e293b', borderColor: '#334155', color: '#fff' }}
                />
                <Line type="monotone" dataKey="total_score" stroke="#0ea5e9" strokeWidth={3} activeDot={{ r: 8 }} />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>

        <div className="bg-slate-800 p-6 rounded-2xl border border-slate-700">
          <h3 className="text-lg font-semibold text-white mb-4">Recent Scans</h3>
          <div className="space-y-3">
            {recentScans.length === 0 && <EmptyState message="No scans recorded yet." />}
            {recentScans.map(scan => (
              <div key={scan.id} className="flex items-center justify-between p-3 bg-slate-900 rounded-lg border border-slate-700">
                <div className="flex items-center">
                  <div className={`w-2 h-10 rounded-full mr-3 ${scan.status === 'Pass' ? 'bg-industrial-success' : 'bg-industrial-danger'}`} />
                  <div>
                    <p className="font-medium text-white">Student #{scan.student_id}</p>
                    <p className="text-xs text-slate-400">{new Date(scan.timestamp).toLocaleTimeString()}</p>
                  </div>
                </div>
                <div className="text-right">
                  <span className="block font-bold text-white text-lg">{scan.total_score}</span>
                  <span className={`text-xs px-2 py-0.5 rounded ${
                    scan.status === 'Pass' ? 'bg-industrial-success/20 text-industrial-success' : 'bg-industrial-danger/20 text-industrial-danger'
                  }`}>
                    {scan.status.toUpperCase()}
                  </span>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

const HistoryView = ({ 
  scans, 
  students 
}: { 
  scans: ScanResult[], 
  students: Student[] 
}) => {
  const [searchTerm, setSearchTerm] = useState('');
  const [dateRange, setDateRange] = useState<{start: string, end: string}>({ start: '', end: '' });
  const [currentPage, setCurrentPage] = useState(1);
  const [sortConfig, setSortConfig] = useState<{ key: string, direction: 'asc' | 'desc' }>({ key: 'timestamp', direction: 'desc' });
  const itemsPerPage = 10;

  // Helper to get student info
  const getStudent = (id: number) => students.find(s => s.id === id);

  // Filter Logic
  const filteredScans = scans.filter(scan => {
    const student = getStudent(scan.student_id);
    const matchesSearch = !searchTerm || 
      (student && (
        student.name.toLowerCase().includes(searchTerm.toLowerCase()) || 
        student.student_id.toLowerCase().includes(searchTerm.toLowerCase())
      ));
    
    const scanDate = new Date(scan.timestamp);
    const matchesStart = !dateRange.start || scanDate >= new Date(dateRange.start);
    const matchesEnd = !dateRange.end || scanDate <= new Date(new Date(dateRange.end).setHours(23, 59, 59));

    return matchesSearch && matchesStart && matchesEnd;
  });

  // Sort Logic
  const sortedScans = [...filteredScans].sort((a, b) => {
    if (sortConfig.key === 'student_name') {
      const nameA = getStudent(a.student_id)?.name || '';
      const nameB = getStudent(b.student_id)?.name || '';
      return sortConfig.direction === 'asc' 
        ? nameA.localeCompare(nameB) 
        : nameB.localeCompare(nameA);
    }
    
    // Generic sort for keys present in ScanResult
    const valA = a[sortConfig.key as keyof ScanResult];
    const valB = b[sortConfig.key as keyof ScanResult];

    if (valA < valB) return sortConfig.direction === 'asc' ? -1 : 1;
    if (valA > valB) return sortConfig.direction === 'asc' ? 1 : -1;
    return 0;
  });

  // Pagination Logic
  const totalPages = Math.ceil(sortedScans.length / itemsPerPage);
  const paginatedScans = sortedScans.slice(
    (currentPage - 1) * itemsPerPage,
    currentPage * itemsPerPage
  );

  const handleSort = (key: string) => {
    setSortConfig(current => ({
      key,
      direction: current.key === key && current.direction === 'desc' ? 'asc' : 'desc'
    }));
  };

  return (
    <div className="space-y-6">
      {/* Filters Toolbar */}
      <div className="bg-slate-800 p-4 rounded-xl border border-slate-700 flex flex-col md:flex-row gap-4 justify-between items-center">
        <div className="relative flex-1 w-full">
           <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-slate-500" />
           <input 
             type="text" 
             placeholder="Search by Student Name or ID..." 
             value={searchTerm}
             onChange={(e) => setSearchTerm(e.target.value)}
             className="w-full bg-slate-900 border border-slate-600 rounded-lg pl-10 pr-4 py-2 text-white focus:ring-2 focus:ring-industrial-blue outline-none"
           />
        </div>
        <div className="flex gap-4 w-full md:w-auto">
          <div className="flex items-center gap-2 bg-slate-900 border border-slate-600 rounded-lg px-3 py-2">
             <Calendar className="w-4 h-4 text-slate-500" />
             <input 
               type="date" 
               value={dateRange.start}
               onChange={(e) => setDateRange({...dateRange, start: e.target.value})}
               className="bg-transparent text-white text-sm outline-none w-32 [&::-webkit-calendar-picker-indicator]:invert"
               title="Start Date"
               placeholder="YYYY-MM-DD"
             />
             <span className="text-slate-500">-</span>
             <input 
               type="date" 
               value={dateRange.end}
               onChange={(e) => setDateRange({...dateRange, end: e.target.value})}
               className="bg-transparent text-white text-sm outline-none w-32 [&::-webkit-calendar-picker-indicator]:invert"
               title="End Date"
               placeholder="YYYY-MM-DD"
             />
          </div>
          <button 
             onClick={() => { setSearchTerm(''); setDateRange({start: '', end: ''}); }}
             className="p-2 text-slate-400 hover:text-white hover:bg-slate-700 rounded-lg transition-colors"
             title="Clear Filters"
          >
             <Filter className="w-5 h-5" />
          </button>
        </div>
      </div>

      {/* Data Table */}
      <div className="bg-slate-800 rounded-2xl border border-slate-700 overflow-hidden shadow-xl">
        <table className="w-full text-left">
          <thead className="bg-slate-900 border-b border-slate-700">
            <tr>
               <th className="p-4 font-semibold text-slate-400 cursor-pointer hover:text-white" onClick={() => handleSort('id')}>
                 <div className="flex items-center gap-2">Scan ID <ArrowUpDown className="w-3 h-3"/></div>
               </th>
               <th className="p-4 font-semibold text-slate-400 cursor-pointer hover:text-white" onClick={() => handleSort('timestamp')}>
                 <div className="flex items-center gap-2">Timestamp <ArrowUpDown className="w-3 h-3"/></div>
               </th>
               <th className="p-4 font-semibold text-slate-400 cursor-pointer hover:text-white" onClick={() => handleSort('student_name')}>
                 <div className="flex items-center gap-2">Student <ArrowUpDown className="w-3 h-3"/></div>
               </th>
               <th className="p-4 font-semibold text-slate-400 cursor-pointer hover:text-white" onClick={() => handleSort('total_score')}>
                 <div className="flex items-center gap-2">Score <ArrowUpDown className="w-3 h-3"/></div>
               </th>
               <th className="p-4 font-semibold text-slate-400 text-right">Metrics Summary</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-700">
            {paginatedScans.length === 0 ? (
               <tr>
                 <td colSpan={5} className="p-8 text-center text-slate-500">
                   No records found matching your filters.
                 </td>
               </tr>
            ) : (
               paginatedScans.map(scan => {
                 const student = getStudent(scan.student_id);
                 return (
                   <tr key={scan.id} className="hover:bg-slate-750 transition-colors">
                     <td className="p-4 font-mono text-slate-500">#{scan.id}</td>
                     <td className="p-4 text-white">
                        <div>{new Date(scan.timestamp).toLocaleDateString()}</div>
                        <div className="text-xs text-slate-500">{new Date(scan.timestamp).toLocaleTimeString()}</div>
                     </td>
                     <td className="p-4">
                        <div className="font-medium text-white">{student?.name || 'Unknown'}</div>
                        <div className="text-xs text-slate-400">{student?.student_id || 'N/A'}</div>
                     </td>
                     <td className="p-4">
                        <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium border ${
                          scan.status === 'Pass' 
                            ? 'bg-industrial-success/10 text-industrial-success border-industrial-success/20' 
                            : 'bg-industrial-danger/10 text-industrial-danger border-industrial-danger/20'
                        }`}>
                          {scan.status} ({scan.total_score}%)
                        </span>
                     </td>
                     <td className="p-4 text-right">
                        <div className="flex justify-end gap-3 text-xs text-slate-400">
                           <span title="Bead Width">W: <span className="text-slate-200">{scan.metrics.width_val}mm</span></span>
                           <span title="Height">H: <span className="text-slate-200">{scan.metrics.height_val}mm</span></span>
                           {scan.defects_json.length > 0 ? (
                              <span className="text-industrial-danger" title="Defects">{scan.defects_json.length} Defect(s)</span>
                           ) : (
                              <span className="text-industrial-success">Clean</span>
                           )}
                        </div>
                     </td>
                   </tr>
                 );
               })
            )}
          </tbody>
        </table>
        
        {/* Pagination Controls */}
        <div className="bg-slate-900 p-4 border-t border-slate-700 flex justify-between items-center">
           <div className="text-sm text-slate-500">
             Showing <span className="font-medium text-white">{(currentPage - 1) * itemsPerPage + 1}</span> to <span className="font-medium text-white">{Math.min(currentPage * itemsPerPage, filteredScans.length)}</span> of <span className="font-medium text-white">{filteredScans.length}</span> results
           </div>
           <div className="flex gap-2">
              <button 
                disabled={currentPage === 1}
                onClick={() => setCurrentPage(c => Math.max(1, c - 1))}
                className="p-2 rounded-lg bg-slate-800 border border-slate-700 text-white hover:bg-slate-700 disabled:opacity-50 disabled:cursor-not-allowed"
                title="Previous Page"
                aria-label="Previous Page"
              >
                <ChevronLeft className="w-4 h-4" />
              </button>
              <button 
                disabled={currentPage === totalPages || totalPages === 0}
                onClick={() => setCurrentPage(c => Math.min(totalPages, c + 1))}
                className="p-2 rounded-lg bg-slate-800 border border-slate-700 text-white hover:bg-slate-700 disabled:opacity-50 disabled:cursor-not-allowed"
                title="Next Page"
                aria-label="Next Page"
              >
                <ChevronRight className="w-4 h-4" />
              </button>
           </div>
        </div>
      </div>
    </div>
  );
};

const CalibrationView = () => {
  const [status, setStatus] = useState<'idle' | 'running' | 'complete' | 'error'>('idle');
  const [progress, setProgress] = useState(0);
  const [result, setResult] = useState<{ matrix: number[][], distCoeffs: number[], error: number } | null>(null);
  const [isSaving, setIsSaving] = useState(false);
  const [isSaved, setIsSaved] = useState(false);
  const [errorMessage, setErrorMessage] = useState<string | null>(null);
  const [isMatrixEditing, setIsMatrixEditing] = useState(false);
  
  const videoRef = useRef<HTMLVideoElement>(null);
  const [isCameraActive, setIsCameraActive] = useState(false);

  useEffect(() => {
    let stream: MediaStream | null = null;
    
    const initCamera = async () => {
      try {
        stream = await navigator.mediaDevices.getUserMedia({ 
          video: {
            width: { ideal: 1920 },
            height: { ideal: 1080 }
          }
        });
        
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
          setIsCameraActive(true);
        }
      } catch (err) {
        console.warn("Camera access failed, falling back to placeholder", err);
        setIsCameraActive(false);
      }
    };

    initCamera();

    return () => {
      if (stream) {
        stream.getTracks().forEach(track => track.stop());
      }
    };
  }, []);

  const handleStartCalibration = async () => {
    setStatus('running');
    setErrorMessage(null);
    setProgress(0);
    setResult(null);
    setIsSaved(false);
    setIsMatrixEditing(false);
    
    // Simulate progress bar
    const interval = setInterval(() => {
      setProgress(p => Math.min(p + 5, 95));
    }, 150);

    try {
      const data = await triggerCalibration();
      setResult(data);
      setStatus('complete');
      setProgress(100);
    } catch (e: any) {
      console.error(e);
      setStatus('error');
      setErrorMessage(e.message || "Unknown error during calibration sequence.");
    } finally {
      clearInterval(interval);
    }
  };

  const handleSave = async () => {
    if(!result) return;
    setIsSaving(true);
    setErrorMessage(null);
    try {
      await saveCalibration(result);
      setIsSaved(true);
      setIsMatrixEditing(false);
    } catch (e: any) {
      console.error(e);
      setErrorMessage(e.message || "Failed to save configuration parameters.");
    } finally {
      setIsSaving(false);
    }
  };

  const handleReset = () => {
    setStatus('idle');
    setResult(null);
    setIsSaved(false);
    setProgress(0);
    setErrorMessage(null);
    setIsMatrixEditing(false);
  };

  const handleMatrixChange = (rIndex: number, cIndex: number, valStr: string) => {
    if (!result) return;
    const newVal = parseFloat(valStr);
    
    // Create deep copy of matrix
    const newMatrix = result.matrix.map(row => [...row]);
    // Allow empty string or partial typing, defaulting to 0 only for matrix math, but input holds value
    newMatrix[rIndex][cIndex] = isNaN(newVal) ? 0 : newVal;
    
    setResult({ ...result, matrix: newMatrix });
  };

  const getCalibrationStepLabel = (p: number) => {
    if (p < 30) return "Detecting Feature Points";
    if (p < 60) return "Calculating Matrix";
    if (p < 90) return "Verifying Homography";
    return "Finalizing Parameters";
  };

  return (
    <div className="grid grid-cols-1 lg:grid-cols-2 gap-8 h-full">
      {/* Feed Area */}
      <div className="space-y-4">
        <div className={`relative aspect-video bg-black rounded-2xl overflow-hidden border-2 shadow-2xl transition-colors ${
          status === 'error' ? 'border-industrial-danger' : 'border-slate-700'
        }`}>
          {isCameraActive ? (
            <video 
              ref={videoRef} 
              autoPlay 
              muted 
              playsInline 
              className="w-full h-full object-cover" 
            />
          ) : (
            <img 
              src={CAMERA_FEED_PLACEHOLDER} 
              alt="Calibration Feed" 
              className="w-full h-full object-cover opacity-60"
            />
          )}
          
          {/* Instructions Overlay */}
          <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
            {status === 'idle' && (
              <div className="text-center p-6 bg-slate-900/80 rounded-xl backdrop-blur-sm border border-slate-600">
                <Grid3x3 className="w-12 h-12 text-industrial-warning mx-auto mb-3" />
                <h3 className="text-xl font-bold text-white">Stereo Calibration Required</h3>
                <p className="text-slate-300 mt-2 max-w-sm">
                  Please hold a standard 9x6 Chessboard pattern visible in both cameras.
                </p>
              </div>
            )}
            
            {status === 'running' && (
              <div className="w-full h-full relative">
                {/* Simulated Detection Grids */}
                <div className="absolute top-[20%] left-[25%] w-[50%] h-[60%] border-2 border-industrial-success/50 bg-industrial-success/5 animate-pulse grid grid-cols-6 grid-rows-4">
                   {[...Array(24)].map((_, i) => <div key={i} className="border-[0.5px] border-industrial-success/20"></div>)}
                </div>
                <div className="absolute bottom-4 left-0 right-0 text-center">
                   <span className="bg-slate-900/80 text-industrial-success px-3 py-1 rounded-full font-mono text-xs border border-industrial-success/30">
                     PATTERN DETECTED: 54 POINTS
                   </span>
                </div>
              </div>
            )}

            {status === 'error' && (
               <div className="text-center p-6 bg-slate-900/90 rounded-xl backdrop-blur-sm border border-industrial-danger">
                  <AlertTriangle className="w-12 h-12 text-industrial-danger mx-auto mb-3" />
                  <h3 className="text-xl font-bold text-white">Calibration Error</h3>
                  <p className="text-slate-300 mt-2 max-w-sm text-sm">
                    Sensor connection unstable or pattern not detected.
                  </p>
               </div>
            )}
          </div>
        </div>
        
        <div className="bg-slate-800 p-4 rounded-xl border border-slate-700">
          <div className="flex items-center justify-between text-sm text-slate-400 mb-2">
            <span>Sensor Status</span>
            <span className={`flex items-center ${status === 'error' ? 'text-industrial-danger' : 'text-industrial-success'}`}>
              {status === 'error' ? <XCircle className="w-3 h-3 mr-1"/> : <CheckCircle2 className="w-3 h-3 mr-1"/>}
              {status === 'error' ? 'Error' : isCameraActive ? 'Active (Live)' : 'Simulated (Mock)'}
            </span>
          </div>
          <div className="flex items-center justify-between text-sm text-slate-400">
            <span>Resolution</span>
            <span className="text-white font-mono">1920x1080 (Dual)</span>
          </div>
        </div>
      </div>

      {/* Controls Area */}
      <div className="bg-slate-800 rounded-2xl border border-slate-700 p-8 flex flex-col">
        <h2 className="text-2xl font-bold text-white mb-2">Calibration Tool</h2>
        <p className="text-slate-400 mb-8">
          Run this utility to update intrinsic and extrinsic parameters for the stereo pair.
        </p>

        {/* Running State */}
        {status === 'running' && (
          <div className="mb-8 space-y-2">
            <div className="flex justify-between text-sm font-medium text-slate-300">
              <span>Capturing Frames...</span>
              <span>{progress}%</span>
            </div>
            <div className="w-full bg-slate-700 rounded-full h-2.5">
              {(() => {
                const pct = Math.max(0, Math.min(100, Math.round(progress / 5) * 5));
                const cls = `w-pct-${pct}`;
                return (
                  <div className={`bg-industrial-blue h-2.5 rounded-full transition-all duration-300 ${cls}`}></div>
                );
              })()}
            </div>
            <div className="flex items-center justify-between text-xs text-slate-500 font-mono mt-2">
               <span className="flex items-center">
                 <div className="w-1.5 h-1.5 bg-industrial-blue rounded-full mr-2 animate-ping"></div>
                 {getCalibrationStepLabel(progress)}...
               </span>
            </div>
          </div>
        )}

        {/* Error State */}
        {status === 'error' && (
          <div className="mb-8 p-6 bg-industrial-danger/10 rounded-xl border border-industrial-danger/50 animate-in fade-in slide-in-from-bottom-2">
             <div className="flex items-start text-industrial-danger mb-4">
                <AlertTriangle className="w-6 h-6 mr-3 mt-0.5 flex-shrink-0" />
                <div>
                  <h4 className="font-bold text-lg">Calibration Failed</h4>
                  <p className="text-slate-300 mt-1 text-sm leading-relaxed">{errorMessage}</p>
                </div>
             </div>
             <p className="text-xs text-slate-500 mb-4">
                Troubleshooting: Check MIPI/USB connections and ensure lighting is adequate for pattern detection.
             </p>
          </div>
        )}

        {/* Complete State */}
        {status === 'complete' && result && (
          <div className="mb-8 p-6 bg-slate-900/50 rounded-xl border border-slate-700 animate-in fade-in slide-in-from-bottom-2">
            <div className="flex items-center justify-between mb-6">
               <div className="flex items-center text-industrial-success">
                  <CheckCircle2 className="w-5 h-5 mr-2" />
                  <span className="font-bold">Calibration Successful</span>
               </div>
               <span className="text-xs text-slate-500 font-mono">RMS Error: {result.error}</span>
            </div>
            
            <div className="space-y-6">
              <div>
                <div className="flex items-center justify-between mb-2">
                  <h4 className="text-xs uppercase text-slate-500 tracking-wider">Camera Matrix</h4>
                  <button 
                    onClick={() => setIsMatrixEditing(!isMatrixEditing)}
                    className={`p-1.5 rounded-lg transition-colors flex items-center space-x-2 text-xs font-medium ${isMatrixEditing ? 'bg-industrial-blue text-white' : 'bg-slate-800 text-slate-400 hover:text-white hover:bg-slate-700'}`}
                    title={isMatrixEditing ? "Done Editing" : "Manual Override"}
                  >
                     {isMatrixEditing ? (
                       <>
                         <CheckCircle2 className="w-3 h-3" />
                         <span>Done</span>
                       </>
                     ) : (
                       <>
                         <Pencil className="w-3 h-3" />
                         <span>Edit</span>
                       </>
                     )}
                  </button>
                </div>
                <div className="flex items-center justify-center bg-slate-950 p-4 rounded-lg border border-slate-800">
                   {/* Matrix Visual */}
                   <div className="relative flex">
                      <div className="w-2 border-l-2 border-t-2 border-b-2 border-slate-500 rounded-l-lg"></div>
                      <div className="grid grid-cols-3 gap-x-2 gap-y-2 font-mono text-sm px-4 py-1 text-industrial-blue">
                        {result.matrix.map((row, rI) => 
                          row.map((val, cI) => (
                            <div key={`${rI}-${cI}`} className="flex items-center justify-end">
                              {isMatrixEditing ? (
                                <input 
                                  type="number"
                                  value={val}
                                  onChange={(e) => handleMatrixChange(rI, cI, e.target.value)}
                                  className="w-20 bg-slate-900 border border-slate-700 rounded px-2 py-1 text-xs text-right focus:border-industrial-blue focus:ring-1 focus:ring-industrial-blue outline-none text-white font-mono"
                                  step="0.1"
                                  title="Calibration Matrix Value"
                                  placeholder="0.0"
                                />
                              ) : (
                                <span className="px-2 py-1">{val.toFixed(1)}</span>
                              )}
                            </div>
                          ))
                        )}
                      </div>
                      <div className="w-2 border-r-2 border-t-2 border-b-2 border-slate-500 rounded-r-lg"></div>
                   </div>
                </div>
              </div>

              <div>
                <h4 className="text-xs uppercase text-slate-500 mb-2 tracking-wider">Distortion Coefficients</h4>
                <div className="font-mono text-xs text-slate-300 bg-slate-950 p-3 rounded border border-slate-800 break-all">
                  k = [{result.distCoeffs.join(', ')}]
                </div>
              </div>
            </div>
          </div>
        )}

        <div className="mt-auto space-y-3">
          {status === 'complete' ? (
            <div className="space-y-3">
              <div className="flex space-x-3">
                <button 
                  onClick={handleReset}
                  className="flex-1 py-3 rounded-xl font-bold bg-slate-700 hover:bg-slate-600 text-white flex items-center justify-center transition-all"
                >
                  <RotateCcw className="w-5 h-5 mr-2" />
                  Recalibrate
                </button>
                <button 
                  onClick={handleSave}
                  disabled={isSaving || isSaved}
                  className={`flex-[2] py-3 rounded-xl font-bold flex items-center justify-center transition-all ${
                    isSaved 
                      ? 'bg-industrial-success text-white' 
                      : 'bg-industrial-blue hover:bg-sky-400 text-white shadow-lg shadow-industrial-blue/20'
                  }`}
                >
                  {isSaving ? (
                    <RefreshCw className="w-5 h-5 mr-2 animate-spin" />
                  ) : isSaved ? (
                    <CheckCircle2 className="w-5 h-5 mr-2" />
                  ) : (
                    <Save className="w-5 h-5 mr-2" />
                  )}
                  {isSaving ? 'Saving...' : isSaved ? 'Configuration Saved' : 'Save Parameters'}
                </button>
              </div>
              {/* Save Error Message Inline */}
              {errorMessage && (
                 <div className="text-center p-2 rounded bg-industrial-danger/10 text-industrial-danger text-sm flex items-center justify-center animate-in fade-in">
                    <AlertTriangle className="w-4 h-4 mr-2" />
                    {errorMessage}
                 </div>
              )}
            </div>
          ) : status === 'error' ? (
            <div className="flex space-x-3">
               <button 
                onClick={handleReset}
                className="flex-1 py-3 rounded-xl font-bold bg-slate-700 hover:bg-slate-600 text-white flex items-center justify-center transition-all"
              >
                Cancel
              </button>
              <button 
                onClick={handleStartCalibration}
                className="flex-[2] py-3 rounded-xl font-bold bg-industrial-blue hover:bg-sky-400 text-white shadow-lg shadow-industrial-blue/20 flex items-center justify-center transition-all"
              >
                <RefreshCw className="w-5 h-5 mr-2" />
                Retry Calibration
              </button>
            </div>
          ) : (
            <button 
              onClick={handleStartCalibration}
              disabled={status === 'running'}
              className={`w-full py-4 rounded-xl font-bold uppercase tracking-wider flex items-center justify-center transition-all ${
                status === 'running' 
                  ? 'bg-slate-700 text-slate-500 cursor-not-allowed'
                  : 'bg-industrial-blue hover:bg-sky-400 text-white hover:scale-[1.02] shadow-lg shadow-industrial-blue/20'
              }`}
            >
              {status === 'running' ? (
                <RefreshCw className="w-5 h-5 mr-2 animate-spin" />
              ) : (
                <Cpu className="w-5 h-5 mr-2" />
              )}
              {status === 'running' ? 'Calibrating...' : 'Start Calibration Sequence'}
            </button>
          )}
        </div>
      </div>
    </div>
  );
};

const SettingsView = ({ 
  currentRubric, 
  onSaveRubric 
}: { 
  currentRubric: RubricConfig, 
  onSaveRubric: (r: RubricConfig) => Promise<void> 
}) => {
  const [rubric, setRubric] = useState<RubricConfig>(currentRubric);
  const [selectedPreset, setSelectedPreset] = useState<string>(
    Object.keys(RUBRIC_PRESETS).find(key => RUBRIC_PRESETS[key].name === currentRubric.name) || 'Custom'
  );
  const [isSaving, setIsSaving] = useState(false);
  const [isDirty, setIsDirty] = useState(false);
  
  // RDK Connection Settings
  const [rdkHost, setRdkHost] = useState<string>('rdk-x5.local');
  const [rdkUser, setRdkUser] = useState<string>('root');
  const [rdkDestPath, setRdkDestPath] = useState<string>('/opt/weldvision/models/model.bin');
  const [rdkSettingsDirty, setRdkSettingsDirty] = useState(false);
  
  // Load RDK settings from localStorage
  useEffect(() => {
    try {
      const saved = JSON.parse(localStorage.getItem("orchestrationSettings") || "{}");
      if (saved.deviceHost) setRdkHost(saved.deviceHost);
      if (saved.deviceUser) setRdkUser(saved.deviceUser);
      if (saved.destPath) setRdkDestPath(saved.destPath);
    } catch {}
  }, []);

  // Sync state when prop updates externally
  useEffect(() => {
    setRubric(currentRubric);
    const presetKey = Object.keys(RUBRIC_PRESETS).find(key => RUBRIC_PRESETS[key].name === currentRubric.name);
    setSelectedPreset(presetKey || 'Custom');
  }, [currentRubric]);

  const handlePresetChange = (presetKey: string) => {
    setSelectedPreset(presetKey);
    if (presetKey !== 'Custom' && RUBRIC_PRESETS[presetKey]) {
      setRubric({ ...RUBRIC_PRESETS[presetKey] });
      setIsDirty(true);
    }
  };

  const handleChange = (field: keyof RubricConfig, value: string | number) => {
    setRubric(prev => ({ ...prev, [field]: Number(value), name: 'Custom Configuration' }));
    setSelectedPreset('Custom');
    setIsDirty(true);
  };

  const handleSave = async () => {
    setIsSaving(true);
    try {
      await onSaveRubric(rubric);
      setIsDirty(false);
    } catch (e) {
      console.error(e);
    } finally {
      setIsSaving(false);
    }
  };
  
  const handleSaveRdkSettings = () => {
    try {
      const settings = JSON.parse(localStorage.getItem("orchestrationSettings") || "{}");
      settings.deviceHost = rdkHost;
      settings.deviceUser = rdkUser;
      settings.destPath = rdkDestPath;
      localStorage.setItem("orchestrationSettings", JSON.stringify(settings));
      setRdkSettingsDirty(false);
      // Notify other tabs/components
      window.dispatchEvent(new StorageEvent('storage', {
        key: 'orchestrationSettings',
        newValue: JSON.stringify(settings)
      }));
    } catch (e) {
      console.error("Failed to save RDK settings:", e);
    }
  };

  return (
    <div className="max-w-4xl mx-auto space-y-6">
      {/* RDK Connection Settings */}
      <div className="bg-slate-800 rounded-2xl border border-slate-700 p-8 shadow-2xl">
        <div className="flex items-center justify-between mb-6">
          <div>
            <h2 className="text-2xl font-bold text-white flex items-center">
              <Network className="w-6 h-6 mr-3 text-industrial-blue" />
              RDK X5 Connection
            </h2>
            <p className="text-slate-400 mt-1">Configure connection settings for your RDK X5 edge device.</p>
          </div>
          <div className="flex items-center space-x-3">
            <button 
              onClick={handleSaveRdkSettings}
              disabled={!rdkSettingsDirty}
              className="px-6 py-2 rounded-lg bg-industrial-blue hover:bg-sky-400 text-white font-bold shadow-lg shadow-industrial-blue/20 flex items-center disabled:opacity-50 disabled:cursor-not-allowed transition-all"
            >
              <Save className="w-4 h-4 mr-2"/>
              Save Connection
            </button>
          </div>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          <div>
            <label className="block text-sm font-medium text-slate-400 mb-2">
              Device Host (IP or Hostname)
            </label>
            <input 
              type="text"
              value={rdkHost}
              onChange={(e) => { setRdkHost(e.target.value); setRdkSettingsDirty(true); }}
              className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none font-mono"
              placeholder="e.g., 192.168.1.100 or 10.0.0.2"
            />
            <p className="text-xs text-slate-500 mt-2">
              Common options: 
              <button className="text-industrial-blue hover:underline ml-1" onClick={() => { setRdkHost('192.168.1.100'); setRdkSettingsDirty(true); }}>Router</button>,
              <button className="text-industrial-blue hover:underline ml-1" onClick={() => { setRdkHost('10.0.0.2'); setRdkSettingsDirty(true); }}>P2P</button>,
              <button className="text-industrial-blue hover:underline ml-1" onClick={() => { setRdkHost('192.168.7.2'); setRdkSettingsDirty(true); }}>USB</button>
            </p>
          </div>

          <div>
            <label className="block text-sm font-medium text-slate-400 mb-2">
              SSH Username
            </label>
            <input 
              type="text"
              value={rdkUser}
              onChange={(e) => { setRdkUser(e.target.value); setRdkSettingsDirty(true); }}
              className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none font-mono"
              placeholder="e.g., root"
            />
          </div>

          <div className="md:col-span-2">
            <label className="block text-sm font-medium text-slate-400 mb-2">
              Model Destination Path on RDK
            </label>
            <input 
              type="text"
              value={rdkDestPath}
              onChange={(e) => { setRdkDestPath(e.target.value); setRdkSettingsDirty(true); }}
              className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none font-mono"
              placeholder="/opt/weldvision/models/model.bin"
            />
          </div>
        </div>

        {/* Connection Status */}
        <div className="mt-6 pt-6 border-t border-slate-700">
          <div className="flex items-center justify-between">
            <div className="text-sm text-slate-400">
              Connection status will be shown when deploying models
            </div>
            <div className="text-xs text-slate-500">
              See Model Training → Orchestration Panel for live connection status
            </div>
          </div>
        </div>
      </div>

      {/* Grading Rubric Configuration */}
      <div className="bg-slate-800 rounded-2xl border border-slate-700 p-8 shadow-2xl">
        <div className="flex items-center justify-between mb-8">
          <div>
            <h2 className="text-2xl font-bold text-white flex items-center">
              <Sliders className="w-6 h-6 mr-3 text-industrial-blue" />
              Grading Rubric Configuration
            </h2>
            <p className="text-slate-400 mt-1">Adjust the acceptance criteria for the automated evaluation system.</p>
          </div>
          <div className="flex items-center space-x-3">
             <button 
               onClick={() => {
                 setRubric(currentRubric);
                 setIsDirty(false);
               }}
               disabled={!isDirty}
               className="px-4 py-2 rounded-lg text-slate-400 hover:text-white disabled:opacity-50 transition-colors"
             >
               Discard
             </button>
             <button 
               onClick={handleSave}
               disabled={!isDirty || isSaving}
               className="px-6 py-2 rounded-lg bg-industrial-blue hover:bg-sky-400 text-white font-bold shadow-lg shadow-industrial-blue/20 flex items-center disabled:opacity-50 disabled:cursor-not-allowed transition-all"
             >
               {isSaving ? <RefreshCw className="w-4 h-4 mr-2 animate-spin"/> : <Save className="w-4 h-4 mr-2"/>}
               Apply Changes
             </button>
          </div>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
           {/* Sidebar / Preset Selector */}
           <div className="md:col-span-1 space-y-6 border-r border-slate-700 pr-8">
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-2">Rubric Preset</label>
                <div className="space-y-2">
                  {Object.keys(RUBRIC_PRESETS).map(key => (
                    <button
                      key={key}
                      onClick={() => handlePresetChange(key)}
                      className={`w-full text-left px-4 py-3 rounded-xl border transition-all ${
                        selectedPreset === key 
                          ? 'bg-industrial-blue/10 border-industrial-blue text-white ring-1 ring-industrial-blue' 
                          : 'bg-slate-900 border-slate-700 text-slate-400 hover:border-slate-500'
                      }`}
                    >
                      <div className="font-semibold">{key}</div>
                      <div className="text-xs opacity-70 mt-1">{RUBRIC_PRESETS[key].name}</div>
                    </button>
                  ))}
                  <button
                      onClick={() => handlePresetChange('Custom')}
                      className={`w-full text-left px-4 py-3 rounded-xl border transition-all ${
                        selectedPreset === 'Custom' 
                          ? 'bg-industrial-orange/10 border-industrial-orange text-white ring-1 ring-industrial-orange' 
                          : 'bg-slate-900 border-slate-700 text-slate-400 hover:border-slate-500'
                      }`}
                    >
                      <div className="font-semibold">Custom</div>
                      <div className="text-xs opacity-70 mt-1">Manual Parameters</div>
                    </button>
                </div>
              </div>
           </div>

           {/* Form Area */}
           <div className="md:col-span-2 space-y-8">
             {/* Width Section */}
             <div>
                <h3 className="text-lg font-semibold text-white mb-4 flex items-center">
                  <Ruler className="w-5 h-5 mr-2 text-slate-400" />
                  Bead Width
                </h3>
                <div className="grid grid-cols-2 gap-6">
                  <div>
                    <label className="block text-sm font-medium text-slate-400 mb-2">Target (mm)</label>
                    <input 
                      type="number"
                      step="0.1"
                      value={rubric.targetWidth}
                      onChange={(e) => handleChange('targetWidth', e.target.value)}
                      className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none"
                      title="Target Bead Width (mm)"
                      placeholder="e.g. 8.0"
                    />
                  </div>
                  <div>
                    <label className="block text-sm font-medium text-slate-400 mb-2">Tolerance (±mm)</label>
                    <input 
                      type="number"
                      step="0.1"
                      value={rubric.widthTolerance}
                      onChange={(e) => handleChange('widthTolerance', e.target.value)}
                      className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none"
                      title="Width Tolerance (±mm)"
                      placeholder="e.g. 1.0"
                    />
                  </div>
                </div>
                <div className="mt-2 text-xs text-slate-500 bg-slate-900/50 p-2 rounded">
                   Acceptable Range: <span className="text-white font-mono">{(rubric.targetWidth - rubric.widthTolerance).toFixed(1)}mm</span> to <span className="text-white font-mono">{(rubric.targetWidth + rubric.widthTolerance).toFixed(1)}mm</span>
                </div>
             </div>

             {/* Height Section */}
             <div>
                <h3 className="text-lg font-semibold text-white mb-4 flex items-center">
                  <ScanLine className="w-5 h-5 mr-2 text-slate-400" />
                  Reinforcement Height
                </h3>
                <div className="grid grid-cols-2 gap-6">
                  <div>
                    <label className="block text-sm font-medium text-slate-400 mb-2">Target (mm)</label>
                    <input 
                      type="number"
                      step="0.1"
                      value={rubric.targetHeight}
                      onChange={(e) => handleChange('targetHeight', e.target.value)}
                      className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none"
                      title="Target Reinforcement Height (mm)"
                      placeholder="e.g. 2.0"
                    />
                  </div>
                  <div>
                    <label className="block text-sm font-medium text-slate-400 mb-2">Tolerance (±mm)</label>
                    <input 
                      type="number"
                      step="0.1"
                      value={rubric.heightTolerance}
                      onChange={(e) => handleChange('heightTolerance', e.target.value)}
                      className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none"
                      title="Height Tolerance (±mm)"
                      placeholder="e.g. 0.5"
                    />
                  </div>
                </div>
             </div>

             {/* Defects Section */}
             <div>
               <h3 className="text-lg font-semibold text-white mb-4 flex items-center">
                  <AlertOctagon className="w-5 h-5 mr-2 text-slate-400" />
                  Defect Sensitivity
                </h3>
                <div className="space-y-4">
                  <div>
                     <label className="block text-sm font-medium text-slate-400 mb-2">Max Allowed Porosity Count</label>
                     <div className="flex items-center space-x-4">
                        <input 
                          type="range" 
                          min="0" 
                          max="10" 
                          step="1"
                          value={rubric.maxPorosity}
                          onChange={(e) => handleChange('maxPorosity', e.target.value)}
                          className="flex-1 h-2 bg-slate-700 rounded-lg appearance-none cursor-pointer accent-industrial-blue"
                          title="Max Allowed Porosity"
                        />
                        <span className="w-12 text-center bg-slate-900 py-1 rounded border border-slate-600 font-mono">
                          {rubric.maxPorosity}
                        </span>
                     </div>
                  </div>
                  <div>
                     <label className="block text-sm font-medium text-slate-400 mb-2">Max Allowed Spatter Count</label>
                     <div className="flex items-center space-x-4">
                        <input 
                          type="range" 
                          min="0" 
                          max="20" 
                          step="1"
                          value={rubric.maxSpatter || 0}
                          onChange={(e) => handleChange('maxSpatter', e.target.value)}
                          className="flex-1 h-2 bg-slate-700 rounded-lg appearance-none cursor-pointer accent-industrial-orange"
                          title="Max Allowed Spatter"
                        />
                        <span className="w-12 text-center bg-slate-900 py-1 rounded border border-slate-600 font-mono">
                          {rubric.maxSpatter || 0}
                        </span>
                     </div>
                     <p className="text-xs text-slate-500 mt-2">
                       Spatter tolerance is higher for training but strictly 0 for critical components.
                     </p>
                  </div>
                </div>
             </div>
           </div>
        </div>
      </div>
    </div>
  );
};

const ScannerView = ({ 
  students, 
  rubric,
  onScanComplete 
}: { 
  students: Student[], 
  rubric: RubricConfig,
  onScanComplete: (res: ScanResult) => void 
}) => {
  const [selectedStudentId, setSelectedStudentId] = useState<number | ''>('');
  const [isScanning, setIsScanning] = useState(false);
  const [result, setResult] = useState<ScanResult | null>(null);

  const handleScan = async () => {
    if (!selectedStudentId) return;
    setIsScanning(true);
    setResult(null);
    try {
      const res = await triggerScan(Number(selectedStudentId), rubric);
      setResult(res);
      onScanComplete(res);
    } catch (e) {
      console.error(e);
    } finally {
      setIsScanning(false);
    }
  };

  return (
    <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 h-full">
      {/* Left Col: Controls & Live Feed */}
      <div className="lg:col-span-2 space-y-4">
        <div className="flex gap-4">
           <select 
            className="flex-1 bg-slate-800 border border-slate-600 text-white p-3 rounded-xl focus:ring-2 focus:ring-industrial-blue outline-none"
            title="Select Student"
            value={selectedStudentId}
            onChange={(e) => setSelectedStudentId(Number(e.target.value) || '')}
          >
            <option value="">-- Select Student --</option>
            {students.map(s => (
              <option key={s.id} value={s.id}>{s.name} ({s.student_id})</option>
            ))}
          </select>
          <button
            disabled={!selectedStudentId || isScanning}
            onClick={handleScan}
            className={`px-8 py-3 rounded-xl font-bold uppercase tracking-wider shadow-lg flex items-center transition-all ${
              !selectedStudentId || isScanning
                ? 'bg-slate-700 text-slate-500 cursor-not-allowed'
                : 'bg-industrial-blue hover:bg-sky-400 text-white hover:scale-105 active:scale-95'
            }`}
          >
            {isScanning ? (
              <>
                <Zap className="w-5 h-5 mr-2 animate-pulse" /> Scanning...
              </>
            ) : (
              <>
                <Camera className="w-5 h-5 mr-2" /> Capture
              </>
            )}
          </button>
        </div>

        <div className="relative aspect-video bg-black rounded-2xl overflow-hidden border-2 border-slate-700 shadow-2xl">
          {/* ROI Overlay */}
          <div className="absolute inset-0 pointer-events-none z-10 opacity-30">
             <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[80%] h-[20%] border-2 border-dashed border-industrial-warning"></div>
             <div className="absolute bottom-4 right-4 text-industrial-warning font-mono text-xs">STEREO_CAM_ACTIVE</div>
          </div>
          
          {isScanning ? (
            <div className="absolute inset-0 flex flex-col items-center justify-center bg-slate-900 z-20">
              <div className="w-16 h-16 border-4 border-industrial-blue border-t-transparent rounded-full animate-spin mb-4"></div>
              <div className="text-industrial-blue font-mono animate-pulse">PROCESSING BPU...</div>
              <div className="text-slate-500 text-xs mt-2 font-mono">Running hobot_dnn.py</div>
            </div>
          ) : (
            <img 
              src={result ? result.image_path : CAMERA_FEED_PLACEHOLDER} 
              alt="Live Feed" 
              className="w-full h-full object-cover opacity-80"
            />
          )}
        </div>
      </div>

      {/* Right Col: Real-time Results */}
      <div className="bg-slate-800 rounded-2xl border border-slate-700 p-6 flex flex-col h-full overflow-y-auto">
        <h2 className="text-xl font-bold text-white mb-6 flex items-center">
          <ScanLine className="w-5 h-5 mr-2 text-industrial-blue" />
          Analysis Results
        </h2>

        {!result ? (
          <EmptyState message="Ready to scan. Select a student and capture." />
        ) : (
          <div className="space-y-6 animate-in fade-in slide-in-from-bottom-4 duration-500">
            {/* Total Score */}
            <div className="text-center p-4 bg-slate-900 rounded-xl border border-slate-700">
              <span className="text-slate-400 text-sm uppercase tracking-wider">Overall Grade</span>
              <div className={`text-5xl font-black mt-2 ${
                result.status === 'Pass' ? 'text-industrial-success' : 'text-industrial-danger'
              }`}>
                {result.total_score}
              </div>
              <div className={`inline-block px-3 py-1 rounded-full text-xs font-bold mt-2 ${
                 result.status === 'Pass' ? 'bg-industrial-success/20 text-industrial-success' : 'bg-industrial-danger/20 text-industrial-danger'
              }`}>
                {result.status.toUpperCase()}
              </div>
            </div>

            {/* Metrics Grid */}
            <div className="space-y-4">
              <MetricCard 
                label="Bead Width" 
                value={result.metrics.width_val} 
                unit="mm" 
                status={
                  Math.abs(result.metrics.width_val - rubric.targetWidth) <= rubric.widthTolerance ? 'success' : 'error'
                }
                subtext={`Target: ${rubric.targetWidth}mm ±${rubric.widthTolerance}`}
              />
              <MetricCard 
                label="Reinforcement Height" 
                value={result.metrics.height_val} 
                unit="mm" 
                status={
                  Math.abs(result.metrics.height_val - rubric.targetHeight) <= rubric.heightTolerance ? 'success' : 'error'
                }
                subtext={`Target: ${rubric.targetHeight}mm ±${rubric.heightTolerance}`}
              />
               <MetricCard 
                label="Spatter Count" 
                value={result.metrics.spatter_count} 
                unit="" 
                status={
                  result.metrics.spatter_count <= (rubric.maxSpatter || 0) ? 'success' : 'warning'
                }
                subtext={`Max Allowed: ${rubric.maxSpatter || 0}`}
              />
              <MetricCard 
                label="Defects Found" 
                value={result.defects_json.length} 
                unit=""
                status={result.defects_json.length === 0 ? 'success' : 'error'}
                subtext={result.defects_json.length > 0 ? result.defects_json.join(', ') : 'None detected'}
              />
            </div>
            
            <div className="p-4 bg-slate-900 rounded-xl">
               <h4 className="text-xs text-slate-500 uppercase mb-2">Bead Uniformity Profile</h4>
               {/* Tiny chart to visualize uniformity */}
               <div className="h-20 w-full">
                  <ResponsiveContainer>
                    <BarChart data={[
                        {v: result.metrics.width_val * 0.9}, 
                        {v: result.metrics.width_val}, 
                        {v: result.metrics.width_val * 1.1},
                        {v: result.metrics.width_val},
                        {v: result.metrics.width_val * 0.95}
                        ]}>
                        <Bar dataKey="v" fill={result.metrics.uniformity_score > 0.8 ? '#22c55e' : '#f97316'} radius={[2,2,0,0]} />
                    </BarChart>
                  </ResponsiveContainer>
               </div>
            </div>

          </div>
        )}
      </div>
    </div>
  );
};

const StudentsView = ({ 
  students, 
  onAddStudent,
  onUpdateStudent,
  onDeleteStudent
}: { 
  students: Student[], 
  onAddStudent: (student: Omit<Student, 'id'>) => Promise<void>,
  onUpdateStudent: (id: number, student: Partial<Student>) => Promise<void>,
  onDeleteStudent: (id: number) => Promise<void>
}) => {
  const [isFormOpen, setIsFormOpen] = useState(false);
  const [editingId, setEditingId] = useState<number | null>(null);
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const [studentToDelete, setStudentToDelete] = useState<Student | null>(null);
  
  const [formData, setFormData] = useState({
    name: '',
    student_id: '',
    class_name: '',
    level: 'Novice' as Student['level']
  });
  
  const [errors, setErrors] = useState<{name?: string, student_id?: string, class_name?: string}>({});
  const [isSubmitting, setIsSubmitting] = useState(false);

  const resetForm = () => {
    setFormData({ name: '', student_id: '', class_name: '', level: 'Novice' });
    setErrors({});
    setEditingId(null);
    setIsFormOpen(false);
  };

  const handleEditClick = (student: Student, e: React.MouseEvent) => {
    e.stopPropagation(); // Prevent row selection when clicking edit
    setFormData({
      name: student.name,
      student_id: student.student_id,
      class_name: student.class_name,
      level: student.level
    });
    setErrors({});
    setEditingId(student.id);
    setIsFormOpen(true);
  };

  const handleDeleteClick = (student: Student, e: React.MouseEvent) => {
    e.stopPropagation(); // Prevent row selection when clicking delete
    setStudentToDelete(student);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    // Validation
    const newErrors: {name?: string, student_id?: string, class_name?: string} = {};
    
    if (!formData.name.trim()) newErrors.name = "Full Name is required";
    if (!formData.student_id.trim()) newErrors.student_id = "Student ID is required";
    if (!formData.class_name.trim()) newErrors.class_name = "Class Name is required";
    
    const duplicate = students.find(s => 
      s.student_id.trim().toLowerCase() === formData.student_id.trim().toLowerCase() && 
      s.id !== editingId
    );
    if (duplicate) {
      newErrors.student_id = "Student ID must be unique";
    }

    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      return;
    }

    setIsSubmitting(true);
    setErrors({});
    try {
      if (editingId) {
        await onUpdateStudent(editingId, formData);
      } else {
        await onAddStudent(formData);
      }
      resetForm();
    } catch (error) {
      console.error("Error saving student:", error);
    } finally {
      setIsSubmitting(false);
    }
  };

  const confirmDelete = async () => {
    if (!studentToDelete) return;
    try {
      await onDeleteStudent(studentToDelete.id);
      setStudentToDelete(null);
      if (selectedId === studentToDelete.id) setSelectedId(null);
    } catch (e) {
      console.error(e);
    }
  };

  return (
    <div className="space-y-6 relative">
      <div className="flex justify-between items-center">
        <h2 className="text-2xl font-bold text-white">Student Roster</h2>
        <button 
          onClick={() => {
            if (isFormOpen) {
              resetForm();
            } else {
              setIsFormOpen(true);
            }
          }}
          className={`flex items-center px-4 py-2 rounded-lg transition-colors ${
            isFormOpen 
              ? 'bg-slate-700 text-slate-300 hover:bg-slate-600' 
              : 'bg-industrial-blue hover:bg-sky-400 text-white'
          }`}
        >
          {isFormOpen ? (
            <>
              <XCircle className="w-4 h-4 mr-2" />
              Cancel
            </>
          ) : (
            <>
              <Plus className="w-4 h-4 mr-2" />
              Add Student
            </>
          )}
        </button>
      </div>

      {isFormOpen && (
        <div className="bg-slate-800 rounded-2xl border border-slate-700 p-6 animate-in fade-in slide-in-from-top-4 duration-300">
          <h3 className="text-lg font-semibold text-white mb-6">
            {editingId ? 'Edit Student Details' : 'Register New Student'}
          </h3>
          <form onSubmit={handleSubmit} className="space-y-4">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-1">Full Name</label>
                <input 
                  type="text" 
                  value={formData.name}
                  onChange={e => setFormData({...formData, name: e.target.value})}
                  className={`w-full bg-slate-900 border rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none transition-all placeholder:text-slate-600 ${errors.name ? 'border-industrial-danger' : 'border-slate-600'}`}
                  placeholder="e.g. Alex Worker"
                />
                {errors.name && <p className="text-industrial-danger text-xs mt-1">{errors.name}</p>}
              </div>
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-1">Student ID</label>
                <input 
                  type="text" 
                  value={formData.student_id}
                  onChange={e => setFormData({...formData, student_id: e.target.value})}
                  className={`w-full bg-slate-900 border rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none transition-all placeholder:text-slate-600 ${errors.student_id ? 'border-industrial-danger' : 'border-slate-600'}`}
                  placeholder="e.g. S-2024-001"
                />
                {errors.student_id && <p className="text-industrial-danger text-xs mt-1">{errors.student_id}</p>}
              </div>
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-1">Class</label>
                <input 
                  type="text" 
                  value={formData.class_name}
                  onChange={e => setFormData({...formData, class_name: e.target.value})}
                  className={`w-full bg-slate-900 border rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none transition-all placeholder:text-slate-600 ${errors.class_name ? 'border-industrial-danger' : 'border-slate-600'}`}
                  placeholder="e.g. Welding Basics 101"
                />
                {errors.class_name && <p className="text-industrial-danger text-xs mt-1">{errors.class_name}</p>}
              </div>
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-1">Skill Level</label>
                <select 
                  title="Skill Level"
                  value={formData.level}
                  onChange={e => setFormData({...formData, level: e.target.value as any})}
                  className="w-full bg-slate-900 border border-slate-600 rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none transition-all appearance-none"
                >
                  <option value="Novice">Novice</option>
                  <option value="Intermediate">Intermediate</option>
                  <option value="Advanced">Advanced</option>
                </select>
              </div>
            </div>
            <div className="flex justify-end pt-4">
              <button 
                type="submit" 
                disabled={isSubmitting}
                className="bg-industrial-success hover:bg-green-600 text-white px-6 py-2.5 rounded-lg font-bold transition-all disabled:opacity-50 disabled:cursor-not-allowed flex items-center shadow-lg hover:shadow-green-900/20"
              >
                {isSubmitting ? (
                  <>
                    <Activity className="w-4 h-4 mr-2 animate-spin" />
                    Saving...
                  </>
                ) : (
                  <>
                    <CheckCircle2 className="w-4 h-4 mr-2" />
                    {editingId ? 'Update Student' : 'Save Student'}
                  </>
                )}
              </button>
            </div>
          </form>
        </div>
      )}
      
      <div className="bg-slate-800 rounded-2xl border border-slate-700 overflow-hidden">
        <table className="w-full text-left">
          <thead className="bg-slate-900 border-b border-slate-700">
            <tr>
              <th className="p-4 font-semibold text-slate-400">ID</th>
              <th className="p-4 font-semibold text-slate-400">Name</th>
              <th className="p-4 font-semibold text-slate-400">Class</th>
              <th className="p-4 font-semibold text-slate-400">Level</th>
              <th className="p-4 font-semibold text-slate-400">Actions</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-slate-700">
            {students.map(student => (
              <tr 
                key={student.id} 
                onClick={() => setSelectedId(selectedId === student.id ? null : student.id)}
                className={`transition-all duration-200 cursor-pointer ${
                  editingId === student.id || selectedId === student.id
                    ? 'bg-industrial-blue/10 border-l-2 border-industrial-blue' 
                    : 'hover:bg-slate-800 border-l-2 border-transparent'
                }`}
              >
                <td className="p-4 text-white font-mono">{student.student_id}</td>
                <td className="p-4 text-white font-medium">{student.name}</td>
                <td className="p-4 text-slate-300">{student.class_name}</td>
                <td className="p-4">
                  <span className={`px-2 py-1 rounded text-xs font-bold ${
                    student.level === 'Advanced' ? 'bg-purple-500/20 text-purple-400' :
                    student.level === 'Intermediate' ? 'bg-blue-500/20 text-blue-400' :
                    'bg-slate-500/20 text-slate-400'
                  }`}>
                    {student.level}
                  </span>
                </td>
                <td className="p-4">
                  <button 
                    onClick={(e) => handleEditClick(student, e)}
                    className="text-slate-400 hover:text-white mr-3 transition-colors"
                    title="Edit Student"
                  >
                    <Pencil className="w-4 h-4" />
                  </button>
                  <button 
                    onClick={(e) => handleDeleteClick(student, e)}
                    className="text-industrial-danger hover:text-red-400 transition-colors"
                    title="Delete Student"
                  >
                    <Trash2 className="w-4 h-4" />
                  </button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {/* Delete Confirmation Modal */}
      {studentToDelete && (
        <div className="fixed inset-0 bg-black/70 flex items-center justify-center z-50 p-4">
          <div className="bg-slate-800 rounded-2xl border border-slate-700 max-w-md w-full p-6 shadow-2xl animate-in zoom-in-95 duration-200">
            <div className="flex items-center mb-4 text-industrial-danger">
              <AlertTriangle className="w-8 h-8 mr-3" />
              <h3 className="text-xl font-bold">Confirm Deletion</h3>
            </div>
            <p className="text-slate-300 mb-6">
              Are you sure you want to remove <span className="font-bold text-white">{studentToDelete.name}</span>? 
              This action cannot be undone and will remove all associated scan history.
            </p>
            <div className="flex justify-end space-x-3">
              <button 
                onClick={() => setStudentToDelete(null)}
                className="px-4 py-2 rounded-lg bg-slate-700 text-white hover:bg-slate-600 transition-colors font-medium"
              >
                Cancel
              </button>
              <button 
                onClick={confirmDelete}
                className="px-4 py-2 rounded-lg bg-industrial-danger text-white hover:bg-red-600 transition-colors font-medium"
              >
                Delete Student
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

// --- App Shell ---

const App: React.FC = () => {
  const [view, setView] = useState<ViewState>(ViewState.DASHBOARD);
  const [students, setStudents] = useState<Student[]>([]);
  const [scans, setScans] = useState<ScanResult[]>([]);
  const [rubric, setRubric] = useState<RubricConfig>(RUBRIC_PRESETS['Standard']);
  const [loading, setLoading] = useState(true);
  const [guideOpen, setGuideOpen] = useState(false);
  
  // Inference Monitor State
  const [inferenceRunning, setInferenceRunning] = useState(false);
  const [latestInferenceResult, setLatestInferenceResult] = useState<any>(null);
  const [desktopConnected, setDesktopConnected] = useState(false);

  // Initial Data Load
  useEffect(() => {
    const init = async () => {
      setLoading(true);
      try {
        const [loadedStudents, loadedHistory, loadedRubric] = await Promise.all([
          fetchStudents(),
          fetchHistory(),
          getRubric()
        ]);
        setStudents(loadedStudents);
        setScans(loadedHistory);
        setRubric(loadedRubric);
      } catch (e) {
        console.error("Failed to load initial data", e);
      } finally {
        setLoading(false);
      }
    };
    init();
  }, []);

  const handleScanComplete = useCallback((newScan: ScanResult) => {
    setScans(prev => [newScan, ...prev]);
  }, []);

  const handleAddStudent = async (studentData: Omit<Student, 'id'>) => {
    try {
      const newStudent = await addStudent(studentData);
      setStudents(prev => [...prev, newStudent]);
    } catch (e) {
      console.error("Failed to add student", e);
    }
  };

  const handleUpdateStudent = async (id: number, updates: Partial<Student>) => {
    try {
      const updated = await updateStudent(id, updates);
      setStudents(prev => prev.map(s => s.id === id ? updated : s));
    } catch (e) {
      console.error("Failed to update student", e);
    }
  };

  const handleDeleteStudent = async (id: number) => {
    try {
      await deleteStudent(id);
      setStudents(prev => prev.filter(s => s.id !== id));
    } catch (e) {
      console.error("Failed to delete student", e);
    }
  };

  const handleSaveRubric = async (newRubric: RubricConfig) => {
    try {
      const saved = await saveRubric(newRubric);
      setRubric(saved);
    } catch (e) {
      console.error("Failed to save rubric", e);
    }
  };

  // Inference Monitor Handlers
  const handleStartInference = async () => {
    try {
      setInferenceRunning(true);
      // Use Orchestration Panel settings from localStorage
      const settings = JSON.parse(localStorage.getItem('orchestrationSettings') || '{}');
      const deviceHost = settings.deviceHost || 'rdk-x5.local';
      const deviceUser = settings.deviceUser || 'root';
      const modelBin = settings.outputDir ? `${settings.outputDir}\\model.bin` : '';
      
      const result = await fetch('/api/inference/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ rdkIp: deviceHost, user: deviceUser, modelBin })
      });
      
      const data = await result.json();
      
      if (result.status === 501) {
        // Not implemented - show helpful error
        alert(`Inference Not Available\n\n${data.message}\n\nDetails: ${data.details}`);
        setInferenceRunning(false);
      } else if (!result.ok) {
        throw new Error(data.error || 'Failed to start inference');
      }
    } catch (error: any) {
      console.error('Failed to start inference:', error);
      alert(`Failed to start inference: ${error.message}`);
      setInferenceRunning(false);
    }
  };

  const handleStopInference = async () => {
    try {
      const settings = JSON.parse(localStorage.getItem('orchestrationSettings') || '{}');
      const deviceHost = settings.deviceHost || 'rdk-x5.local';
      const deviceUser = settings.deviceUser || 'root';
      
      const result = await fetch('/api/inference/stop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ rdkIp: deviceHost, user: deviceUser })
      });
      
      if (!result.ok) {
        const data = await result.json();
        throw new Error(data.error || 'Failed to stop inference');
      }
      
      setInferenceRunning(false);
      setLatestInferenceResult(null);
    } catch (error: any) {
      console.error('Failed to stop inference:', error);
      alert(`Failed to stop inference: ${error.message}`);
    }
  };

  const handleToggleDesktop = (enable: boolean) => {
    setDesktopConnected(enable);
    console.log('Desktop refinement:', enable ? 'enabled' : 'disabled');
  };

  if (loading) {
    return (
      <div className="h-screen w-screen bg-slate-950 flex items-center justify-center">
        <div className="flex flex-col items-center">
           <Activity className="w-12 h-12 text-industrial-blue animate-bounce mb-4" />
           <h2 className="text-white text-xl font-bold">Initializing RDK X5 System...</h2>
           <p className="text-slate-500 mt-2">Connecting to ROS2 Nodes</p>
        </div>
      </div>
    );
  }

  return (
    <div className="flex h-screen bg-slate-950 text-slate-100 font-sans selection:bg-industrial-blue selection:text-white">
      {/* Sidebar */}
      <aside className="w-64 bg-slate-900 border-r border-slate-800 flex flex-col p-4">
        <div className="mb-8 px-2 flex items-center">
          <div className="w-10 h-10 bg-gradient-to-br from-industrial-blue to-blue-700 rounded-lg flex items-center justify-center mr-3 shadow-lg">
            <Zap className="text-white w-6 h-6" />
          </div>
          <div>
            <h1 className="font-bold text-lg leading-tight">WeldEval <span className="text-industrial-blue">X5</span></h1>
            <p className="text-xs text-slate-500">Horizon RDK Powered</p>
          </div>
        </div>

        <nav className="flex-1">
          <SidebarItem 
            icon={LayoutDashboard} 
            label="Dashboard" 
            active={view === ViewState.DASHBOARD} 
            onClick={() => setView(ViewState.DASHBOARD)} 
          />
          <SidebarItem 
            icon={Camera} 
            label="Live Scanner" 
            active={view === ViewState.SCANNER} 
            onClick={() => setView(ViewState.SCANNER)} 
          />
          <SidebarItem 
            icon={Users} 
            label="Students" 
            active={view === ViewState.STUDENTS} 
            onClick={() => setView(ViewState.STUDENTS)} 
          />
          <SidebarItem 
            icon={History} 
            label="Scan History" 
            active={view === ViewState.HISTORY} 
            onClick={() => setView(ViewState.HISTORY)} 
          />

          {/* Features */}
          <p className="text-xs font-semibold text-slate-400 uppercase tracking-wider px-2 mt-4 mb-3">Features</p>
          <SidebarItem 
            icon={Settings} 
            label="Manual Calibration" 
            active={view === ViewState.MANUAL_BED_CALIBRATION} 
            onClick={() => setView(ViewState.MANUAL_BED_CALIBRATION)} 
          />

          {/* Common Settings */}
          <div className="mt-4 pt-4 border-t border-slate-800">
            <p className="text-xs font-semibold text-slate-400 uppercase tracking-wider px-2 mb-3">AI & Training</p>
            <SidebarItem 
              icon={Brain} 
              label="Inference Monitor" 
              active={view === ViewState.INFERENCE_MONITOR} 
              onClick={() => setView(ViewState.INFERENCE_MONITOR)} 
            />
            <SidebarItem 
              icon={Sparkles} 
              label="Model Training" 
              active={view === ViewState.TRAINING_DASHBOARD} 
              onClick={() => setView(ViewState.TRAINING_DASHBOARD)} 
            />
            <SidebarItem 
              icon={Database} 
              label="Dataset Studio" 
              active={view === ViewState.DATASET_STUDIO} 
              onClick={() => setView(ViewState.DATASET_STUDIO)} 
            />
            <SidebarItem 
              icon={BarChart3} 
              label="Model Management" 
              active={view === ViewState.MODEL_MANAGEMENT} 
              onClick={() => setView(ViewState.MODEL_MANAGEMENT)} 
            />
          </div>

          {/* Settings */}
          <div className="mt-4 pt-4 border-t border-slate-800">
            <SidebarItem 
              icon={Settings} 
              label="Settings" 
              active={view === ViewState.SETTINGS} 
              onClick={() => setView(ViewState.SETTINGS)} 
            />
          </div>
        </nav>

        <button
            onClick={() => setGuideOpen(true)}
            className="flex items-center w-full p-3 mb-2 rounded-lg text-slate-400 hover:bg-slate-800 hover:text-white transition-colors font-medium"
            title="Open help and user guide"
            aria-label="Help and documentation"
          >
            <HelpCircle className="w-5 h-5 mr-3" />
            <span className="font-medium">Help & Guide</span>
          </button>

        <div className="mt-auto p-4 bg-slate-800 rounded-xl border border-slate-700">
           <div className="flex items-center justify-between text-xs text-slate-400 mb-2">
             <span>CPU Load</span>
             <span>12%</span>
           </div>
           <div className="w-full bg-slate-700 h-1.5 rounded-full mb-3">
             <div className="bg-industrial-success h-1.5 rounded-full w-[12%]"></div>
           </div>
           <div className="flex items-center justify-between text-xs text-slate-400 mb-2">
             <span>BPU Load</span>
             <span>0%</span>
           </div>
           <div className="w-full bg-slate-700 h-1.5 rounded-full">
             <div className="bg-industrial-blue h-1.5 rounded-full w-[0%]"></div>
           </div>
        </div>
      </aside>

      {/* Main Content */}
      <main className="flex-1 overflow-auto p-8 relative">
        <header className="flex justify-between items-center mb-8">
          <div>
            <h2 className="text-2xl font-bold text-white">
              {view === ViewState.DASHBOARD && 'Overview'}
              {view === ViewState.SCANNER && 'Evaluation Station'}
              {view === ViewState.STUDENTS && 'Class Management'}
              {view === ViewState.HISTORY && 'Scan Archives'}
              {view === ViewState.MANUAL_BED_CALIBRATION && 'Manual Calibration'}
              {view === ViewState.STEREO_CALIBRATION && 'Stereo Camera Calibration'}
              {view === ViewState.INFERENCE_MONITOR && 'Inference Monitor'}
              {view === ViewState.TRAINING_DASHBOARD && 'Model Training'}
              {view === ViewState.DATASET_STUDIO && 'Dataset Studio'}
              {view === ViewState.MODEL_MANAGEMENT && 'Model Management'}
              {view === ViewState.SETTINGS && 'System Configuration'}
            </h2>
            <p className="text-slate-400 text-sm mt-1">
              {view === ViewState.SCANNER 
                ? 'Place workpiece in ROI and stabilize before capturing.' 
                : view === ViewState.INFERENCE_MONITOR
                ? 'Real-time inference monitoring and statistics.'
                : view === ViewState.TRAINING_DASHBOARD
                ? 'Train models on desktop GPU for improved accuracy.'
                : view === ViewState.DATASET_STUDIO
                ? 'Capture and annotate images in one workspace.'
                : view === ViewState.MODEL_MANAGEMENT
                ? 'Manage, deploy, and compare trained models.'
                : 'System Status: Nominal | Camera: Connected'}
            </p>
          </div>
          <div className="flex items-center space-x-4">
             <button 
               onClick={() => setGuideOpen(true)}
               className="flex items-center space-x-2 px-4 py-2 rounded-lg bg-slate-800 hover:bg-slate-700 border border-slate-700 transition-colors text-slate-300 hover:text-white"
               title="Open user guide and help documentation"
               aria-label="Open help guide"
             >
               <HelpCircle className="w-5 h-5" />
               <span className="text-sm font-medium hidden sm:inline">Help</span>
             </button>
             <div className="flex items-center text-sm text-slate-400 bg-slate-900 px-3 py-1.5 rounded-full border border-slate-700">
               <div className="w-2 h-2 rounded-full bg-industrial-success mr-2 animate-pulse"></div>
               ROS2 Active
             </div>
             <div className="w-10 h-10 rounded-full bg-slate-800 border border-slate-700 flex items-center justify-center">
               <Users className="w-5 h-5 text-slate-400" />
             </div>
          </div>
        </header>

        {view === ViewState.DASHBOARD && (
          <DashboardView scans={scans} students={students} onNavigate={setView} />
        )}
        {view === ViewState.SCANNER && (
          <ScannerView students={students} rubric={rubric} onScanComplete={handleScanComplete} />
        )}
        {view === ViewState.STUDENTS && (
          <StudentsView 
            students={students} 
            onAddStudent={handleAddStudent} 
            onUpdateStudent={handleUpdateStudent}
            onDeleteStudent={handleDeleteStudent}
          />
        )}
        {view === ViewState.HISTORY && (
          <HistoryView scans={scans} students={students} />
        )}
        {view === ViewState.MANUAL_BED_CALIBRATION && (
          <ManualBedCalibration />
        )}
        {view === ViewState.STEREO_CALIBRATION && (
          <StereoCameraCalibration />
        )}
        {view === ViewState.INFERENCE_MONITOR && (
          <InferenceMonitor 
            latestResult={latestInferenceResult}
            isRunning={inferenceRunning}
            onStart={handleStartInference}
            onStop={handleStopInference}
            desktopConnected={desktopConnected}
            onToggleDesktop={handleToggleDesktop}
          />
        )}
        {view === ViewState.TRAINING_DASHBOARD && (
          <>
            <TrainingDashboard />
            <div className="mt-8">
              <OrchestrationPanel />
            </div>
          </>
        )}
        {view === ViewState.DATASET_STUDIO && <DatasetStudio />}
        {view === ViewState.MODEL_MANAGEMENT && (
          <>
            <ModelManagement />
          </>
        )}
        {view === ViewState.SETTINGS && (
          <SettingsView currentRubric={rubric} onSaveRubric={handleSaveRubric} />
        )}
      </main>

      {/* User Guide Modal */}
      <UserGuide isOpen={guideOpen} onClose={() => setGuideOpen(false)} />
    </div>
  );
};

export default App;
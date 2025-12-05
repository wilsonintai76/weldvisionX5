import React, { useState } from 'react';
import {
  X,
  Zap,
  Camera,
  Ruler,
  AlertCircle,
  CheckCircle2,
  HardDrive,
  BookOpen,
  Cable,
  Lightbulb,
  Users,
  Settings,
} from 'lucide-react';

interface GuideProps {
  isOpen: boolean;
  onClose: () => void;
}

type GuideSection = 'welcome' | 'hardware' | 'calibration' | 'scanning' | 'troubleshooting' | 'tips';

const UserGuide: React.FC<GuideProps> = ({ isOpen, onClose }) => {
  const [activeSection, setActiveSection] = useState<GuideSection>('welcome');

  const sections: Record<GuideSection, { title: string; icon: React.ElementType }> = {
    welcome: { title: 'Welcome', icon: BookOpen },
    hardware: { title: 'Hardware Setup', icon: Cable },
    calibration: { title: 'Camera Calibration', icon: Ruler },
    scanning: { title: 'Running Scans', icon: Camera },
    troubleshooting: { title: 'Troubleshooting', icon: AlertCircle },
    tips: { title: 'Best Practices', icon: Lightbulb },
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black/70 flex items-center justify-center z-50 p-4">
      <div className="bg-slate-900 rounded-2xl max-w-4xl w-full max-h-[90vh] flex flex-col border border-slate-700 shadow-2xl">
        {/* Header */}
        <div className="flex items-center justify-between p-6 border-b border-slate-700 bg-gradient-to-r from-slate-900 to-slate-800">
          <div className="flex items-center gap-3">
            <BookOpen className="w-6 h-6 text-industrial-blue" />
            <h1 className="text-2xl font-bold text-white">WeldMaster AI User Guide</h1>
          </div>
          <button
            onClick={onClose}
            className="p-2 hover:bg-slate-700 rounded-lg transition-colors"
            title="Close user guide"
            aria-label="Close user guide"
          >
            <X className="w-6 h-6 text-slate-400" />
          </button>
        </div>

        <div className="flex flex-1 overflow-hidden">
          {/* Navigation Sidebar */}
          <div className="w-64 bg-slate-800 border-r border-slate-700 overflow-y-auto">
            {Object.entries(sections).map(([key, { title, icon: Icon }]) => (
              <button
                key={key}
                onClick={() => setActiveSection(key as GuideSection)}
                className={`w-full flex items-center gap-3 px-4 py-4 text-left transition-colors border-l-4 ${
                  activeSection === key
                    ? 'bg-slate-700 border-industrial-blue text-white'
                    : 'border-transparent text-slate-400 hover:bg-slate-700 hover:text-white'
                }`}
              >
                <Icon className="w-5 h-5 flex-shrink-0" />
                <span className="font-medium">{title}</span>
              </button>
            ))}
          </div>

          {/* Content Area */}
          <div className="flex-1 overflow-y-auto p-8">
            {activeSection === 'welcome' && <WelcomeSection />}
            {activeSection === 'hardware' && <HardwareSection />}
            {activeSection === 'calibration' && <CalibrationSection />}
            {activeSection === 'scanning' && <ScanningSection />}
            {activeSection === 'troubleshooting' && <TroubleshootingSection />}
            {activeSection === 'tips' && <TipsSection />}
          </div>
        </div>
      </div>
    </div>
  );
};

const WelcomeSection: React.FC = () => (
  <div className="space-y-6">
    <div className="bg-gradient-to-r from-industrial-blue/10 to-industrial-orange/10 border border-industrial-blue/20 rounded-lg p-6">
      <h2 className="text-2xl font-bold text-white mb-4">Welcome to WeldMaster AI</h2>
      <p className="text-slate-300 mb-4">
        An automated visual inspection system for evaluating student welding workpieces on the Horizon Robotics RDK X5 platform.
      </p>
      <div className="grid grid-cols-2 gap-4 mt-4">
        <div className="flex items-start gap-3">
          <Camera className="w-5 h-5 text-industrial-blue mt-1 flex-shrink-0" />
          <div>
            <h4 className="font-semibold text-white">Real-time Inspection</h4>
            <p className="text-sm text-slate-400">Analyze welds instantly</p>
          </div>
        </div>
        <div className="flex items-start gap-3">
          <Ruler className="w-5 h-5 text-industrial-blue mt-1 flex-shrink-0" />
          <div>
            <h4 className="font-semibold text-white">Precise Measurement</h4>
            <p className="text-sm text-slate-400">Depth and width detection</p>
          </div>
        </div>
        <div className="flex items-start gap-3">
          <CheckCircle2 className="w-5 h-5 text-industrial-success mt-1 flex-shrink-0" />
          <div>
            <h4 className="font-semibold text-white">Standards Compliance</h4>
            <p className="text-sm text-slate-400">ISO 5817 rubric evaluation</p>
          </div>
        </div>
        <div className="flex items-start gap-3">
          <HardDrive className="w-5 h-5 text-industrial-blue mt-1 flex-shrink-0" />
          <div>
            <h4 className="font-semibold text-white">Student Tracking</h4>
            <p className="text-sm text-slate-400">Complete history & analytics</p>
          </div>
        </div>
      </div>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Quick Start Steps</h3>
      <ol className="space-y-3">
        <li className="flex gap-3">
          <span className="flex items-center justify-center w-6 h-6 rounded-full bg-industrial-blue text-white text-sm font-bold flex-shrink-0">1</span>
          <span className="text-slate-300">Set up and connect hardware (Camera & RDK X5)</span>
        </li>
        <li className="flex gap-3">
          <span className="flex items-center justify-center w-6 h-6 rounded-full bg-industrial-blue text-white text-sm font-bold flex-shrink-0">2</span>
          <span className="text-slate-300">Calibrate the camera for accurate measurements</span>
        </li>
        <li className="flex gap-3">
          <span className="flex items-center justify-center w-6 h-6 rounded-full bg-industrial-blue text-white text-sm font-bold flex-shrink-0">3</span>
          <span className="text-slate-300">Create student profiles and select evaluation rubric</span>
        </li>
        <li className="flex gap-3">
          <span className="flex items-center justify-center w-6 h-6 rounded-full bg-industrial-blue text-white text-sm font-bold flex-shrink-0">4</span>
          <span className="text-slate-300">Position weld under camera and scan</span>
        </li>
        <li className="flex gap-3">
          <span className="flex items-center justify-center w-6 h-6 rounded-full bg-industrial-blue text-white text-sm font-bold flex-shrink-0">5</span>
          <span className="text-slate-300">View results and track student progress</span>
        </li>
      </ol>
    </div>
  </div>
);

const HardwareSection: React.FC = () => (
  <div className="space-y-6">
    <div>
      <h2 className="text-2xl font-bold text-white mb-4">Hardware Setup</h2>
      <p className="text-slate-300 mb-4">
        WeldMaster AI requires the Horizon Robotics RDK X5 platform with an RDK Stereo Camera module.
      </p>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4 flex items-center gap-2">
        <Cable className="w-5 h-5 text-industrial-blue" />
        Connection Requirements
      </h3>
      <div className="space-y-4">
        <div className="bg-slate-700 rounded p-4">
          <h4 className="font-semibold text-white mb-2">RDK X5 Edge Device</h4>
          <ul className="text-slate-300 space-y-1 text-sm">
            <li>• 8GB+ RAM</li>
            <li>• 64GB+ Storage (SSD recommended)</li>
            <li>• ROS2 Framework installed</li>
            <li>• Network connectivity (Ethernet or WiFi)</li>
          </ul>
        </div>
        <div className="bg-slate-700 rounded p-4">
          <h4 className="font-semibold text-white mb-2">RDK Stereo Camera</h4>
          <ul className="text-slate-300 space-y-1 text-sm">
            <li>• OV5647 RGB sensor (5MP)</li>
            <li>• Paired stereo depth sensor</li>
            <li>• CSI/MIPI interface</li>
            <li>• Connected via dedicated camera connector</li>
          </ul>
        </div>
      </div>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Setup Instructions</h3>
      <div className="space-y-4">
        <div className="flex gap-4">
          <div className="flex-shrink-0 w-8 h-8 rounded-full bg-industrial-blue flex items-center justify-center text-white font-bold">1</div>
          <div>
            <h4 className="font-semibold text-white mb-1">Power On RDK X5</h4>
            <p className="text-slate-400 text-sm">Connect power supply and ensure device boots successfully</p>
          </div>
        </div>
        <div className="flex gap-4">
          <div className="flex-shrink-0 w-8 h-8 rounded-full bg-industrial-blue flex items-center justify-center text-white font-bold">2</div>
          <div>
            <h4 className="font-semibold text-white mb-1">Connect Stereo Camera</h4>
            <p className="text-slate-400 text-sm">Insert camera module into CSI connector (power off device first)</p>
          </div>
        </div>
        <div className="flex gap-4">
          <div className="flex-shrink-0 w-8 h-8 rounded-full bg-industrial-blue flex items-center justify-center text-white font-bold">3</div>
          <div>
            <h4 className="font-semibold text-white mb-1">Enable Camera in ROS2</h4>
            <p className="text-slate-400 text-sm">Run: ros2 launch horizon_camera camera.launch.py</p>
          </div>
        </div>
        <div className="flex gap-4">
          <div className="flex-shrink-0 w-8 h-8 rounded-full bg-industrial-blue flex items-center justify-center text-white font-bold">4</div>
          <div>
            <h4 className="font-semibold text-white mb-1">Verify Camera Topics</h4>
            <p className="text-slate-400 text-sm">Run: ros2 topic list (should show /image_raw and /depth_raw)</p>
          </div>
        </div>
        <div className="flex gap-4">
          <div className="flex-shrink-0 w-8 h-8 rounded-full bg-industrial-blue flex items-center justify-center text-white font-bold">5</div>
          <div>
            <h4 className="font-semibold text-white mb-1">Setup Lighting</h4>
            <p className="text-slate-400 text-sm">Position LED ring light to avoid shadows (uniform 45° angle recommended)</p>
          </div>
        </div>
      </div>
    </div>

    <div className="bg-yellow-900/20 border border-yellow-700/50 rounded-lg p-4">
      <div className="flex gap-3">
        <AlertCircle className="w-5 h-5 text-yellow-600 flex-shrink-0 mt-0.5" />
        <div>
          <h4 className="font-semibold text-yellow-200 mb-1">Important Safety Notes</h4>
          <ul className="text-yellow-100 text-sm space-y-1">
            <li>• Never connect/disconnect camera while device is powered on</li>
            <li>• Ensure proper ventilation around RDK X5 device</li>
            <li>• Keep away from water and moisture</li>
            <li>• Allow device to cool between intensive scanning sessions</li>
          </ul>
        </div>
      </div>
    </div>
  </div>
);

const CalibrationSection: React.FC = () => (
  <div className="space-y-6">
    <div>
      <h2 className="text-2xl font-bold text-white mb-4">Camera Calibration</h2>
      <p className="text-slate-300 mb-4">
        Calibration is essential for accurate bead width and height measurements. Perform calibration before first use and after any mechanical changes.
      </p>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Why Calibrate?</h3>
      <div className="grid grid-cols-2 gap-4">
        <div className="bg-slate-700 rounded p-3">
          <p className="text-slate-300 text-sm">
            <span className="font-semibold">Pixel-to-MM Conversion</span>: Maps camera pixels to real-world millimeters
          </p>
        </div>
        <div className="bg-slate-700 rounded p-3">
          <p className="text-slate-300 text-sm">
            <span className="font-semibold">Distortion Correction</span>: Removes lens barrel/pincushion distortion
          </p>
        </div>
        <div className="bg-slate-700 rounded p-3">
          <p className="text-slate-300 text-sm">
            <span className="font-semibold">Depth Accuracy</span>: Enables stereo depth map generation
          </p>
        </div>
        <div className="bg-slate-700 rounded p-3">
          <p className="text-slate-300 text-sm">
            <span className="font-semibold">Measurement Precision</span>: ±0.1mm accuracy for ISO compliance
          </p>
        </div>
      </div>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Calibration Procedure</h3>
      <div className="space-y-4">
        <div className="bg-slate-700 rounded p-4">
          <h4 className="font-semibold text-white mb-2">What You Need</h4>
          <ul className="text-slate-300 text-sm space-y-1">
            <li>• Checkerboard calibration pattern (9x6 squares, 30mm each)</li>
            <li>• Printed or displayed on monitor</li>
            <li>• Stable stand to hold pattern at various angles</li>
          </ul>
        </div>

        <div className="space-y-3">
          <h4 className="font-semibold text-white">Steps</h4>
          <div className="flex gap-4">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">1</div>
            <div>
              <p className="text-slate-300 text-sm"><span className="font-semibold">Open Calibration Tool</span>: Navigate to Settings → Calibration</p>
            </div>
          </div>
          <div className="flex gap-4">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">2</div>
            <div>
              <p className="text-slate-300 text-sm"><span className="font-semibold">Position Pattern</span>: Hold checkerboard in front of camera</p>
            </div>
          </div>
          <div className="flex gap-4">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">3</div>
            <div>
              <p className="text-slate-300 text-sm"><span className="font-semibold">Capture Multiple Angles</span>: Take 20+ images from different positions</p>
            </div>
          </div>
          <div className="flex gap-4">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">4</div>
            <div>
              <p className="text-slate-300 text-sm"><span className="font-semibold">Include Variations</span>: Tilt, rotate, and move pattern around frame</p>
            </div>
          </div>
          <div className="flex gap-4">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">5</div>
            <div>
              <p className="text-slate-300 text-sm"><span className="font-semibold">Run Calibration</span>: Click "Start Calibration" (takes 30-60 seconds)</p>
            </div>
          </div>
          <div className="flex gap-4">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">6</div>
            <div>
              <p className="text-slate-300 text-sm"><span className="font-semibold">Review Results</span>: Check RMS error (target: &lt;0.5mm)</p>
            </div>
          </div>
          <div className="flex gap-4">
            <div className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">7</div>
            <div>
              <p className="text-slate-300 text-sm"><span className="font-semibold">Save Configuration</span>: Click "Save" to persist calibration</p>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div className="bg-green-900/20 border border-green-700/50 rounded-lg p-4">
      <div className="flex gap-3">
        <CheckCircle2 className="w-5 h-5 text-green-600 flex-shrink-0 mt-0.5" />
        <div>
          <h4 className="font-semibold text-green-200 mb-1">Success Indicators</h4>
          <ul className="text-green-100 text-sm space-y-1">
            <li>• RMS Error &lt; 0.5mm</li>
            <li>• All checkerboard corners detected in images</li>
            <li>• Calibration file saved successfully</li>
            <li>• "Ready for Operation" status shown on Health Check</li>
          </ul>
        </div>
      </div>
    </div>
  </div>
);

const ScanningSection: React.FC = () => (
  <div className="space-y-6">
    <div>
      <h2 className="text-2xl font-bold text-white mb-4">Running Scans</h2>
      <p className="text-slate-300 mb-4">
        Complete guide to setting up and executing welding workpiece evaluations.
      </p>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Pre-Scan Checklist</h3>
      <div className="space-y-3">
        <label className="flex items-center gap-3 p-3 bg-slate-700 rounded cursor-pointer hover:bg-slate-600 transition-colors">
          <input type="checkbox" className="w-4 h-4" defaultChecked disabled />
          <span className="text-slate-300">Calibration completed and saved</span>
        </label>
        <label className="flex items-center gap-3 p-3 bg-slate-700 rounded cursor-pointer hover:bg-slate-600 transition-colors">
          <input type="checkbox" className="w-4 h-4" defaultChecked disabled />
          <span className="text-slate-300">Lighting is uniform and shadows minimized</span>
        </label>
        <label className="flex items-center gap-3 p-3 bg-slate-700 rounded cursor-pointer hover:bg-slate-600 transition-colors">
          <input type="checkbox" className="w-4 h-4" defaultChecked disabled />
          <span className="text-slate-300">Student is selected</span>
        </label>
        <label className="flex items-center gap-3 p-3 bg-slate-700 rounded cursor-pointer hover:bg-slate-600 transition-colors">
          <input type="checkbox" className="w-4 h-4" defaultChecked disabled />
          <span className="text-slate-300">Evaluation rubric is configured</span>
        </label>
        <label className="flex items-center gap-3 p-3 bg-slate-700 rounded cursor-pointer hover:bg-slate-600 transition-colors">
          <input type="checkbox" className="w-4 h-4" defaultChecked disabled />
          <span className="text-slate-300">Welding sample positioned in frame center</span>
        </label>
      </div>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Positioning Guidelines</h3>
      <div className="grid grid-cols-2 gap-4">
        <div className="bg-slate-700 rounded p-4">
          <h4 className="font-semibold text-white mb-2">✓ Correct Position</h4>
          <ul className="text-slate-300 text-sm space-y-1">
            <li>• Weld centered in frame</li>
            <li>• Bead fully visible</li>
            <li>• Angle: 0° (perpendicular)</li>
            <li>• Distance: 10-20cm from camera</li>
            <li>• Focus: Sharp and clear</li>
          </ul>
        </div>
        <div className="bg-slate-700 rounded p-4">
          <h4 className="font-semibold text-white mb-2">✗ Avoid</h4>
          <ul className="text-slate-300 text-sm space-y-1">
            <li>• Weld at edge of frame</li>
            <li>• Angled or tilted views</li>
            <li>• Shadows or reflections</li>
            <li>• Too close or too far</li>
            <li>• Out of focus images</li>
          </ul>
        </div>
      </div>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Scan Execution Steps</h3>
      <div className="space-y-3">
        <div className="flex gap-4">
          <span className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">1</span>
          <div>
            <p className="text-slate-300"><span className="font-semibold">Navigate to Scanner</span></p>
            <p className="text-slate-400 text-sm">Click "Scanner" in left sidebar</p>
          </div>
        </div>
        <div className="flex gap-4">
          <span className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">2</span>
          <div>
            <p className="text-slate-300"><span className="font-semibold">Select Student</span></p>
            <p className="text-slate-400 text-sm">Choose from dropdown list</p>
          </div>
        </div>
        <div className="flex gap-4">
          <span className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">3</span>
          <div>
            <p className="text-slate-300"><span className="font-semibold">View Live Feed</span></p>
            <p className="text-slate-400 text-sm">Camera feed displays in real-time</p>
          </div>
        </div>
        <div className="flex gap-4">
          <span className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">4</span>
          <div>
            <p className="text-slate-300"><span className="font-semibold">Position Weld</span></p>
            <p className="text-slate-400 text-sm">Move weld sample into optimal position</p>
          </div>
        </div>
        <div className="flex gap-4">
          <span className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">5</span>
          <div>
            <p className="text-slate-300"><span className="font-semibold">Start Scan</span></p>
            <p className="text-slate-400 text-sm">Click "Scan Weld" button</p>
          </div>
        </div>
        <div className="flex gap-4">
          <span className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">6</span>
          <div>
            <p className="text-slate-300"><span className="font-semibold">Wait for Processing</span></p>
            <p className="text-slate-400 text-sm">Vision analysis runs on RDK X5 (~3 seconds)</p>
          </div>
        </div>
        <div className="flex gap-4">
          <span className="flex-shrink-0 w-6 h-6 rounded-full bg-industrial-blue flex items-center justify-center text-white text-sm font-bold">7</span>
          <div>
            <p className="text-slate-300"><span className="font-semibold">Review Results</span></p>
            <p className="text-slate-400 text-sm">Results saved to History automatically</p>
          </div>
        </div>
      </div>
    </div>

    <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
      <h3 className="text-lg font-semibold text-white mb-4">Understanding Results</h3>
      <div className="space-y-3">
        <div className="bg-slate-700 rounded p-3">
          <p className="font-semibold text-white text-sm">Width (mm)</p>
          <p className="text-slate-400 text-xs">Horizontal bead dimension. Target: 8mm ±1mm</p>
        </div>
        <div className="bg-slate-700 rounded p-3">
          <p className="font-semibold text-white text-sm">Height (mm)</p>
          <p className="text-slate-400 text-xs">Vertical bead elevation. Target: 2mm ±0.5mm</p>
        </div>
        <div className="bg-slate-700 rounded p-3">
          <p className="font-semibold text-white text-sm">Uniformity Score</p>
          <p className="text-slate-400 text-xs">Surface consistency 0.0-1.0 (higher is better)</p>
        </div>
        <div className="bg-slate-700 rounded p-3">
          <p className="font-semibold text-white text-sm">Porosity Count</p>
          <p className="text-slate-400 text-xs">Number of void defects detected</p>
        </div>
        <div className="bg-slate-700 rounded p-3">
          <p className="font-semibold text-white text-sm">Spatter Count</p>
          <p className="text-slate-400 text-xs">Number of unwanted metal particles</p>
        </div>
      </div>
    </div>
  </div>
);

const TroubleshootingSection: React.FC = () => (
  <div className="space-y-6">
    <div>
      <h2 className="text-2xl font-bold text-white mb-4">Troubleshooting</h2>
      <p className="text-slate-300 mb-4">
        Common issues and solutions for WeldMaster AI operation.
      </p>
    </div>

    <div className="space-y-4">
      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3">Camera Not Detected</h3>
        <div className="space-y-2">
          <p className="text-slate-300"><span className="font-semibold">Symptom:</span> "Camera: NOT CONNECTED" on Health Check</p>
          <div className="bg-slate-700 rounded p-3 mt-2">
            <p className="text-slate-200 font-semibold mb-2">Solutions:</p>
            <ul className="text-slate-300 text-sm space-y-1">
              <li>1. Check physical camera connection (CSI cable)</li>
              <li>2. Verify ROS2 is running: ros2 topic list</li>
              <li>3. Restart camera node: ros2 launch horizon_camera camera.launch.py</li>
              <li>4. Check RDK X5 kernel logs for hardware errors</li>
            </ul>
          </div>
        </div>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3">Blurry Camera Feed</h3>
        <div className="space-y-2">
          <p className="text-slate-300"><span className="font-semibold">Symptom:</span> Scanned images are out of focus</p>
          <div className="bg-slate-700 rounded p-3 mt-2">
            <p className="text-slate-200 font-semibold mb-2">Solutions:</p>
            <ul className="text-slate-300 text-sm space-y-1">
              <li>1. Adjust camera focus (mechanical adjustment on lens)</li>
              <li>2. Verify optimal distance (10-20cm)</li>
              <li>3. Wait for auto-focus to stabilize</li>
              <li>4. Check lens for dirt or scratches</li>
            </ul>
          </div>
        </div>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3">Inaccurate Measurements</h3>
        <div className="space-y-2">
          <p className="text-slate-300"><span className="font-semibold">Symptom:</span> Width/height readings seem off</p>
          <div className="bg-slate-700 rounded p-3 mt-2">
            <p className="text-slate-200 font-semibold mb-2">Solutions:</p>
            <ul className="text-slate-300 text-sm space-y-1">
              <li>1. Re-run calibration with new checkerboard images</li>
              <li>2. Verify checkerboard dimensions (30mm squares)</li>
              <li>3. Ensure adequate lighting (no shadows)</li>
              <li>4. Check if camera was physically moved</li>
            </ul>
          </div>
        </div>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3">High Defect Detection</h3>
        <div className="space-y-2">
          <p className="text-slate-300"><span className="font-semibold">Symptom:</span> All scans show excessive defects</p>
          <div className="bg-slate-700 rounded p-3 mt-2">
            <p className="text-slate-200 font-semibold mb-2">Solutions:</p>
            <ul className="text-slate-300 text-sm space-y-1">
              <li>1. Improve lighting uniformity</li>
              <li>2. Clean camera lens</li>
              <li>3. Reduce background noise/reflections</li>
              <li>4. Verify algorithm thresholds in Settings</li>
            </ul>
          </div>
        </div>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3">Slow Processing</h3>
        <div className="space-y-2">
          <p className="text-slate-300"><span className="font-semibold">Symptom:</span> Scans take longer than 5 seconds</p>
          <div className="bg-slate-700 rounded p-3 mt-2">
            <p className="text-slate-200 font-semibold mb-2">Solutions:</p>
            <ul className="text-slate-300 text-sm space-y-1">
              <li>1. Check RDK X5 CPU usage (should be &lt;80%)</li>
              <li>2. Close other applications</li>
              <li>3. Ensure adequate cooling</li>
              <li>4. Check network connectivity to backend</li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  </div>
);

const TipsSection: React.FC = () => (
  <div className="space-y-6">
    <div>
      <h2 className="text-2xl font-bold text-white mb-4">Best Practices</h2>
      <p className="text-slate-300 mb-4">
        Tips and recommendations for optimal WeldMaster AI usage and accurate results.
      </p>
    </div>

    <div className="grid grid-cols-1 gap-4">
      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Lightbulb className="w-5 h-5 text-industrial-orange" />
          Lighting Optimization
        </h3>
        <ul className="text-slate-300 space-y-2 text-sm">
          <li>• Use LED ring light with consistent color temperature (5000K)</li>
          <li>• Position light at 45° angle to reduce reflections</li>
          <li>• Avoid direct overhead lighting (causes harsh shadows)</li>
          <li>• Diffuse light with frosted plexiglass if needed</li>
          <li>• Keep consistent brightness across all scans</li>
        </ul>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Camera className="w-5 h-5 text-industrial-blue" />
          Camera Maintenance
        </h3>
        <ul className="text-slate-300 space-y-2 text-sm">
          <li>• Clean lens with soft, dry cloth weekly</li>
          <li>• Check CSI cable connections monthly</li>
          <li>• Allow device to cool for 15min between scanning sessions</li>
          <li>• Store camera in dust-free environment</li>
          <li>• Recalibrate after physical relocation</li>
        </ul>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Ruler className="w-5 h-5 text-industrial-success" />
          Measurement Accuracy
        </h3>
        <ul className="text-slate-300 space-y-2 text-sm">
          <li>• Calibrate once per week for consistent accuracy</li>
          <li>• Use 9x6 checkerboard pattern at multiple angles</li>
          <li>• Target RMS error &lt; 0.5mm</li>
          <li>• Perform test scans of known reference samples</li>
          <li>• Document calibration date in laboratory log</li>
        </ul>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Users className="w-5 h-5 text-industrial-orange" />
          Student Management
        </h3>
        <ul className="text-slate-300 space-y-2 text-sm">
          <li>• Create profiles at class start to track progress</li>
          <li>• Assign appropriate skill level (Novice/Intermediate/Advanced)</li>
          <li>• Use consistent rubric for each class</li>
          <li>• Review historical scans to identify patterns</li>
          <li>• Export results periodically for records</li>
        </ul>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Settings className="w-5 h-5 text-industrial-blue" />
          Rubric Selection
        </h3>
        <ul className="text-slate-300 space-y-2 text-sm">
          <li>• <span className="font-semibold">Novice Training:</span> Loose standards for learning phase</li>
          <li>• <span className="font-semibold">Standard (ISO-5817-C):</span> Industry standard defect levels</li>
          <li>• <span className="font-semibold">Strict (ISO-5817-B):</span> High-precision requirements</li>
          <li>• Adjust rubric as students progress through course</li>
          <li>• Document rubric used with each evaluation</li>
        </ul>
      </div>

      <div className="bg-slate-800 rounded-lg p-6 border border-slate-700">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Zap className="w-5 h-5 text-industrial-success" />
          System Performance
        </h3>
        <ul className="text-slate-300 space-y-2 text-sm">
          <li>• Monitor backend logs: tail -f backend/weld_evaluator.log</li>
          <li>• Check system health: curl /api/health</li>
          <li>• Keep ROS2 packages updated</li>
          <li>• Reboot RDK X5 weekly to clear memory</li>
          <li>• Archive scan results monthly to free storage</li>
        </ul>
      </div>
    </div>

    <div className="bg-gradient-to-r from-industrial-blue/10 to-industrial-success/10 border border-industrial-blue/20 rounded-lg p-6">
      <h3 className="text-lg font-semibold text-white mb-4">Quality Assurance Tips</h3>
      <div className="space-y-3">
        <div className="flex gap-3">
          <CheckCircle2 className="w-5 h-5 text-industrial-success flex-shrink-0 mt-0.5" />
          <p className="text-slate-300"><span className="font-semibold">Daily:</span> Check system health status</p>
        </div>
        <div className="flex gap-3">
          <CheckCircle2 className="w-5 h-5 text-industrial-success flex-shrink-0 mt-0.5" />
          <p className="text-slate-300"><span className="font-semibold">Weekly:</span> Recalibrate and clean optics</p>
        </div>
        <div className="flex gap-3">
          <CheckCircle2 className="w-5 h-5 text-industrial-success flex-shrink-0 mt-0.5" />
          <p className="text-slate-300"><span className="font-semibold">Monthly:</span> Validate against reference samples</p>
        </div>
        <div className="flex gap-3">
          <CheckCircle2 className="w-5 h-5 text-industrial-success flex-shrink-0 mt-0.5" />
          <p className="text-slate-300"><span className="font-semibold">Quarterly:</span> Review system logs for anomalies</p>
        </div>
      </div>
    </div>
  </div>
);

export default UserGuide;

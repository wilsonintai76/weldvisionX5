import React from 'react';
import { CheckCircle, AlertTriangle, XCircle } from 'lucide-react';

interface MetricCardProps {
  label: string;
  value: string | number;
  unit?: string;
  status: 'success' | 'warning' | 'error' | 'neutral';
  subtext?: string;
}

export const MetricCard: React.FC<MetricCardProps> = ({ label, value, unit, status, subtext }) => {
  const getColors = () => {
    switch (status) {
      case 'success': return 'border-industrial-success/50 bg-industrial-success/10 text-industrial-success';
      case 'warning': return 'border-industrial-warning/50 bg-industrial-warning/10 text-industrial-warning';
      case 'error': return 'border-industrial-danger/50 bg-industrial-danger/10 text-industrial-danger';
      default: return 'border-slate-700 bg-slate-800 text-slate-400';
    }
  };

  const getIcon = () => {
    switch (status) {
      case 'success': return <CheckCircle className="w-5 h-5" />;
      case 'warning': return <AlertTriangle className="w-5 h-5" />;
      case 'error': return <XCircle className="w-5 h-5" />;
      default: return null;
    }
  };

  return (
    <div className={`p-4 rounded-xl border ${getColors()} flex flex-col justify-between shadow-sm transition-all duration-200`}>
      <div className="flex justify-between items-start mb-2">
        <span className="text-sm font-semibold uppercase tracking-wider opacity-80">{label}</span>
        {getIcon()}
      </div>
      <div>
        <div className="text-2xl font-bold">
          {value} <span className="text-sm font-normal opacity-70">{unit}</span>
        </div>
        {subtext && <div className="text-xs mt-1 opacity-70">{subtext}</div>}
      </div>
    </div>
  );
};
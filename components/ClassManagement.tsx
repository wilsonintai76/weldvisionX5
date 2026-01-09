import React, { useState } from 'react';
import { Plus, Pencil, Trash2, XCircle, CheckCircle2, Activity, GraduationCap, Users } from 'lucide-react';
import { Class } from '../types';

interface ClassManagementProps {
  classes: Class[];
  onAddClass: (cls: Omit<Class, 'id' | 'created_at' | 'updated_at'>) => Promise<void>;
  onUpdateClass: (id: number, cls: Partial<Class>) => Promise<void>;
  onDeleteClass: (id: number) => Promise<void>;
}

export const ClassManagement = ({
  classes,
  onAddClass,
  onUpdateClass,
  onDeleteClass
}: ClassManagementProps) => {
  const [isFormOpen, setIsFormOpen] = useState(false);
  const [editingId, setEditingId] = useState<number | null>(null);
  const [classToDelete, setClassToDelete] = useState<Class | null>(null);

  const [formData, setFormData] = useState({
    name: '',
    lecturer: ''
  });

  const [errors, setErrors] = useState<{ name?: string, lecturer?: string }>({});
  const [isSubmitting, setIsSubmitting] = useState(false);

  const resetForm = () => {
    setFormData({ name: '', lecturer: '' });
    setErrors({});
    setEditingId(null);
    setIsFormOpen(false);
  };

  const handleEditClick = (cls: Class, e: React.MouseEvent) => {
    e.stopPropagation();
    setFormData({
      name: cls.name,
      lecturer: cls.lecturer
    });
    setErrors({});
    setEditingId(cls.id);
    setIsFormOpen(true);
  };

  const handleDeleteClick = (cls: Class, e: React.MouseEvent) => {
    e.stopPropagation();
    setClassToDelete(cls);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Validation
    const newErrors: { name?: string, lecturer?: string } = {};

    if (!formData.name.trim()) newErrors.name = "Class Name is required";
    if (!formData.lecturer.trim()) newErrors.lecturer = "Lecturer Name is required";

    const duplicate = classes.find(c =>
      c.name.trim().toLowerCase() === formData.name.trim().toLowerCase() &&
      c.id !== editingId
    );
    if (duplicate) {
      newErrors.name = "Class Name must be unique";
    }

    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      return;
    }

    setIsSubmitting(true);
    setErrors({});
    try {
      if (editingId) {
        await onUpdateClass(editingId, formData);
      } else {
        await onAddClass(formData);
      }
      resetForm();
    } catch (error) {
      console.error("Error saving class:", error);
    } finally {
      setIsSubmitting(false);
    }
  };

  const confirmDelete = async () => {
    if (!classToDelete) return;
    try {
      await onDeleteClass(classToDelete.id);
      setClassToDelete(null);
    } catch (e) {
      console.error(e);
    }
  };

  return (
    <div className="space-y-6 relative">
      <div className="flex justify-between items-center">
        <div>
          <h2 className="text-2xl font-bold text-white flex items-center">
            <GraduationCap className="w-7 h-7 mr-3 text-industrial-blue" />
            Class Management
          </h2>
          <p className="text-slate-400 mt-1">Manage classes and assign lecturers</p>
        </div>
        <button
          onClick={() => {
            if (isFormOpen) {
              resetForm();
            } else {
              setIsFormOpen(true);
            }
          }}
          className={`flex items-center px-4 py-2 rounded-lg transition-colors ${isFormOpen
            ? 'bg-slate-700 text-slate-300 hover:bg-slate-600'
            : 'bg-industrial-blue hover:bg-sky-400 text-white shadow-lg'
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
              Add Class
            </>
          )}
        </button>
      </div>

      {isFormOpen && (
        <div className="bg-slate-800 rounded-2xl border border-slate-700 p-6 animate-in fade-in slide-in-from-top-4 duration-300">
          <h3 className="text-lg font-semibold text-white mb-6">
            {editingId ? 'Edit Class Details' : 'Create New Class'}
          </h3>
          <form onSubmit={handleSubmit} className="space-y-4">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-1">Class Name</label>
                <input
                  type="text"
                  value={formData.name}
                  onChange={e => setFormData({ ...formData, name: e.target.value })}
                  className={`w-full bg-slate-900 border rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none transition-all placeholder:text-slate-600 ${errors.name ? 'border-industrial-danger' : 'border-slate-600'
                    }`}
                  placeholder="e.g. Welding Fundamentals 101"
                />
                {errors.name && <p className="text-industrial-danger text-xs mt-1">{errors.name}</p>}
              </div>
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-1">Lecturer Name</label>
                <input
                  type="text"
                  value={formData.lecturer}
                  onChange={e => setFormData({ ...formData, lecturer: e.target.value })}
                  className={`w-full bg-slate-900 border rounded-lg px-4 py-2.5 text-white focus:ring-2 focus:ring-industrial-blue outline-none transition-all placeholder:text-slate-600 ${errors.lecturer ? 'border-industrial-danger' : 'border-slate-600'
                    }`}
                  placeholder="e.g. Dr. John Smith"
                />
                {errors.lecturer && <p className="text-industrial-danger text-xs mt-1">{errors.lecturer}</p>}
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
                    {editingId ? 'Update Class' : 'Create Class'}
                  </>
                )}
              </button>
            </div>
          </form>
        </div>
      )}

      <div className="bg-slate-800 rounded-2xl border border-slate-700 overflow-hidden shadow-xl">
        {classes.length === 0 ? (
          <div className="p-12 text-center">
            <GraduationCap className="w-16 h-16 text-slate-600 mx-auto mb-4" />
            <p className="text-slate-400 text-lg">No classes created yet</p>
            <p className="text-slate-500 text-sm mt-2">Click "Add Class" to create your first class</p>
          </div>
        ) : (
          <table className="w-full text-left">
            <thead className="bg-slate-900 border-b border-slate-700">
              <tr>
                <th className="p-4 font-semibold text-slate-400">Class Name</th>
                <th className="p-4 font-semibold text-slate-400">Lecturer</th>
                <th className="p-4 font-semibold text-slate-400">Students</th>
                <th className="p-4 font-semibold text-slate-400">Actions</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-slate-700">
              {classes.map(cls => (
                <tr
                  key={cls.id}
                  className="hover:bg-slate-750 transition-colors"
                >
                  <td className="p-4">
                    <div className="flex items-center">
                      <GraduationCap className="w-5 h-5 text-industrial-blue mr-3" />
                      <span className="text-white font-medium">{cls.name}</span>
                    </div>
                  </td>
                  <td className="p-4 text-slate-300">{cls.lecturer}</td>
                  <td className="p-4">
                    <div className="flex items-center text-slate-400">
                      <Users className="w-4 h-4 mr-2" />
                      <span>{cls.student_count || 0} enrolled</span>
                    </div>
                  </td>
                  <td className="p-4">
                    <div className="flex items-center gap-2">
                      <button
                        onClick={(e) => handleEditClick(cls, e)}
                        className="p-2 text-slate-400 hover:text-white hover:bg-slate-700 rounded-lg transition-colors"
                        title="Edit Class"
                      >
                        <Pencil className="w-4 h-4" />
                      </button>
                      <button
                        onClick={(e) => handleDeleteClick(cls, e)}
                        className="p-2 text-slate-400 hover:text-industrial-danger hover:bg-red-950 rounded-lg transition-colors"
                        title="Delete Class"
                      >
                        <Trash2 className="w-4 h-4" />
                      </button>
                    </div>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>

      {/* Delete Confirmation Modal */}
      {classToDelete && (
        <div className="fixed inset-0 bg-black/70 flex items-center justify-center z-50 p-4 animate-in fade-in duration-200">
          <div className="bg-slate-800 rounded-2xl border border-slate-700 p-8 max-w-md w-full animate-in zoom-in-95 duration-200 shadow-2xl">
            <div className="flex items-center mb-6">
              <div className="w-12 h-12 bg-industrial-danger/20 rounded-full flex items-center justify-center mr-4">
                <Trash2 className="w-6 h-6 text-industrial-danger" />
              </div>
              <div>
                <h3 className="text-xl font-bold text-white">Delete Class?</h3>
                <p className="text-slate-400 text-sm mt-1">This action cannot be undone</p>
              </div>
            </div>
            <div className="bg-slate-900 p-4 rounded-lg border border-slate-700 mb-6">
              <p className="text-white font-medium">{classToDelete.name}</p>
              <p className="text-slate-400 text-sm">Lecturer: {classToDelete.lecturer}</p>
              {classToDelete.student_count && classToDelete.student_count > 0 && (
                <p className="text-industrial-warning text-sm mt-2">
                  ⚠️ {classToDelete.student_count} student(s) are enrolled in this class
                </p>
              )}
            </div>
            <div className="flex gap-3">
              <button
                onClick={() => setClassToDelete(null)}
                className="flex-1 px-4 py-2.5 bg-slate-700 hover:bg-slate-600 text-white rounded-lg font-medium transition-colors"
              >
                Cancel
              </button>
              <button
                onClick={confirmDelete}
                className="flex-1 px-4 py-2.5 bg-industrial-danger hover:bg-red-600 text-white rounded-lg font-bold transition-colors flex items-center justify-center"
              >
                <Trash2 className="w-4 h-4 mr-2" />
                Delete Class
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

from django.contrib import admin
from .models import Class, Student, Scan, TrainingJob, Rubric, CalibrationData


@admin.register(Class)
class ClassAdmin(admin.ModelAdmin):
    list_display = ['name', 'lecturer', 'student_count', 'created_at']
    search_fields = ['name', 'lecturer']
    
    def student_count(self, obj):
        return obj.students.count()
    student_count.short_description = 'Students'


@admin.register(Student)
class StudentAdmin(admin.ModelAdmin):
    list_display = ['name', 'student_id', 'class_enrolled', 'level', 'created_at']
    list_filter = ['class_enrolled', 'level']
    search_fields = ['name', 'student_id']


@admin.register(Scan)
class ScanAdmin(admin.ModelAdmin):
    list_display = ['id', 'student', 'timestamp', 'score', 'status']
    list_filter = ['status', 'timestamp']
    search_fields = ['student__name', 'student__student_id']


@admin.register(TrainingJob)
class TrainingJobAdmin(admin.ModelAdmin):
    list_display = ['id', 'status', 'progress', 'created_at']
    list_filter = ['status']


@admin.register(Rubric)
class RubricAdmin(admin.ModelAdmin):
    list_display = ['id', 'name', 'targetWidth', 'targetHeight', 'updated_at']


@admin.register(CalibrationData)
class CalibrationDataAdmin(admin.ModelAdmin):
    list_display = ['id', 'error', 'created_at']

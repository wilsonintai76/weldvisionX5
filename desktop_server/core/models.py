from django.db import models


class Class(models.Model):
    """Class/Course entity with lecturer information"""
    name = models.CharField(max_length=100, unique=True)
    lecturer = models.CharField(max_length=100)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        verbose_name_plural = "Classes"
        ordering = ['name']

    def __str__(self):
        return f"{self.name} - {self.lecturer}"


class Student(models.Model):
    name = models.CharField(max_length=100)
    student_id = models.CharField(max_length=50, unique=True)
    class_enrolled = models.ForeignKey(Class, on_delete=models.SET_NULL, related_name='students', null=True, blank=True)
    level = models.CharField(max_length=20, default='Novice')
    created_at = models.DateTimeField(auto_now_add=True)

    def __str__(self):
        return f"{self.name} ({self.student_id})"

class Scan(models.Model):
    student = models.ForeignKey(Student, on_delete=models.CASCADE, related_name='scans')
    timestamp = models.DateTimeField(auto_now_add=True)
    image = models.ImageField(upload_to='scans/%Y/%m/%d/', blank=True, null=True)
    score = models.FloatField()
    
    # Metrics stored as JSON
    width_val = models.FloatField()
    height_val = models.FloatField()
    uniformity_score = models.FloatField()
    porosity_count = models.IntegerField()
    spatter_count = models.IntegerField()
    undercut_detected = models.BooleanField(default=False)
    
    defects_json = models.JSONField(default=list)
    status = models.CharField(max_length=20) # Pass/Fail

    def __str__(self):
        return f"Scan {self.id} - {self.student.name} - {self.status}"

class TrainingJob(models.Model):
    STATUS_CHOICES = [
        ('PENDING', 'Pending'),
        ('RUNNING', 'Running'),
        ('COMPLETED', 'Completed'),
        ('FAILED', 'Failed'),
    ]
    
    created_at = models.DateTimeField(auto_now_add=True)
    status = models.CharField(max_length=20, choices=STATUS_CHOICES, default='PENDING')
    progress = models.IntegerField(default=0)
    result_message = models.TextField(blank=True)
    
    def __str__(self):
        return f"Job {self.id} - {self.status}"


class Rubric(models.Model):
    name = models.CharField(max_length=100, blank=True)
    targetWidth = models.FloatField(default=8.0)
    widthTolerance = models.FloatField(default=1.0)
    targetHeight = models.FloatField(default=2.0)
    heightTolerance = models.FloatField(default=0.5)
    maxPorosity = models.IntegerField(default=0)
    maxSpatter = models.IntegerField(default=2)
    updated_at = models.DateTimeField(auto_now=True)


class CalibrationData(models.Model):
    matrix = models.JSONField(default=list)
    distCoeffs = models.JSONField(default=list)
    error = models.FloatField(default=0.0)
    created_at = models.DateTimeField(auto_now_add=True)

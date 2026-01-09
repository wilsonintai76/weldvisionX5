from rest_framework import serializers
from .models import Student, Scan, TrainingJob, Rubric, CalibrationData, Class


class ClassSerializer(serializers.ModelSerializer):
    """Serializer for Class model with lecturer information"""
    student_count = serializers.SerializerMethodField()

    class Meta:
        model = Class
        fields = ['id', 'name', 'lecturer', 'created_at', 'updated_at', 'student_count']
        read_only_fields = ['created_at', 'updated_at']

    def get_student_count(self, obj):
        """Return the number of students enrolled in this class"""
        return obj.students.count()


class StudentSerializer(serializers.ModelSerializer):
    class_name = serializers.CharField(source='class_enrolled.name', read_only=True)
    class_id = serializers.PrimaryKeyRelatedField(
        queryset=Class.objects.all(), 
        source='class_enrolled',
        write_only=True
    )
    
    class Meta:
        model = Student
        fields = ['id', 'name', 'student_id', 'class_enrolled', 'class_name', 'class_id', 'level', 'created_at']
        read_only_fields = ['created_at']

class ScanSerializer(serializers.ModelSerializer):
    # Accept either:
    # - RDK: a string student identifier (matches Student.student_id)
    # - Frontend: a numeric primary key (Student.id)
    student_id = serializers.CharField(write_only=True, required=False)

    class Meta:
        model = Scan
        fields = '__all__'
        read_only_fields = ['student']

    def create(self, validated_data):
        incoming_student_id = validated_data.pop('student_id', None)
        student = None

        if incoming_student_id is not None:
            # Prefer matching the external/student identifier (RDK-friendly).
            student = Student.objects.filter(student_id=str(incoming_student_id)).first()

            # If not found and numeric-looking, treat as PK (frontend-friendly).
            if student is None and str(incoming_student_id).isdigit():
                student = Student.objects.filter(pk=int(incoming_student_id)).first()

        if student is None:
            # Auto-create only if we were given a student identifier; otherwise reject.
            if incoming_student_id is None:
                raise serializers.ValidationError({'student_id': 'student_id is required'})

            # Get or create a default "Unassigned" class for auto-created students
            default_class, _ = Class.objects.get_or_create(
                name="Unassigned",
                defaults={'lecturer': 'N/A'}
            )

            student = Student.objects.create(
                student_id=str(incoming_student_id),
                name="Unknown Student",
                class_enrolled=default_class,
            )

        scan = Scan.objects.create(student=student, **validated_data)
        return scan

    def to_representation(self, instance):
        # Shape responses to match the frontend's ScanResult interface.
        request = self.context.get('request')
        image_path = ''
        if instance.image:
            try:
                image_path = request.build_absolute_uri(instance.image.url) if request else instance.image.url
            except Exception:
                image_path = ''

        return {
            'id': instance.pk,
            'student_id': instance.student_id if hasattr(instance, 'student_id') else instance.student.pk,
            'timestamp': instance.timestamp.isoformat() if instance.timestamp else None,
            'total_score': instance.score,
            'metrics': {
                'width_val': instance.width_val,
                'height_val': instance.height_val,
                'uniformity_score': instance.uniformity_score,
                'porosity_count': instance.porosity_count,
                'spatter_count': instance.spatter_count,
                'undercut_detected': instance.undercut_detected,
            },
            'defects_json': instance.defects_json,
            'image_path': image_path,
            'status': instance.status,
        }

class TrainingJobSerializer(serializers.ModelSerializer):
    class Meta:
        model = TrainingJob
        fields = '__all__'


class RubricSerializer(serializers.ModelSerializer):
    class Meta:
        model = Rubric
        fields = '__all__'


class CalibrationDataSerializer(serializers.ModelSerializer):
    class Meta:
        model = CalibrationData
        fields = '__all__'

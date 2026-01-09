import base64
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError
import json
from rest_framework import viewsets, status
from rest_framework.response import Response
from rest_framework.decorators import action
from rest_framework.decorators import api_view
from .models import Student, Scan, TrainingJob, Class
from .models import Rubric, CalibrationData
from .serializers import (
    StudentSerializer,
    ScanSerializer,
    TrainingJobSerializer,
    RubricSerializer,
    CalibrationDataSerializer,
    ClassSerializer,
)
from .tasks import start_training_task
from django.core.files.base import ContentFile


class ClassViewSet(viewsets.ModelViewSet):
    """ViewSet for Class management with CRUD operations"""
    queryset = Class.objects.all()
    serializer_class = ClassSerializer


class StudentViewSet(viewsets.ModelViewSet):
    queryset = Student.objects.all()
    serializer_class = StudentSerializer

class ScanViewSet(viewsets.ModelViewSet):
    queryset = Scan.objects.all()
    serializer_class = ScanSerializer

    def create(self, request, *args, **kwargs):
        # RDK X5 sends data here
        return super().create(request, *args, **kwargs)

class TrainingJobViewSet(viewsets.ReadOnlyModelViewSet):
    queryset = TrainingJob.objects.all()
    serializer_class = TrainingJobSerializer

    @action(detail=False, methods=['post'])
    def start(self, request):
        # Start a background task via Celery
        job = TrainingJob.objects.create(status='PENDING')
        start_training_task.delay(job.id)
        return Response(TrainingJobSerializer(job).data, status=status.HTTP_201_CREATED)

from django.shortcuts import render

def live_monitor(request):
    """
    Serves the HTML shell. The browser will fetch the video 
    directly from the RDK's IP via Side-Channel.
    """
    return render(request, 'live_monitor.html')


def _get_active_rubric() -> Rubric:
    rubric = Rubric.objects.order_by('-updated_at').first()
    if rubric:
        return rubric
    return Rubric.objects.create()


@api_view(['GET', 'POST'])
def rubric_view(request):
    if request.method == 'GET':
        rubric = _get_active_rubric()
        return Response(RubricSerializer(rubric).data)

    rubric = _get_active_rubric()
    serializer = RubricSerializer(instance=rubric, data=request.data, partial=True)
    serializer.is_valid(raise_exception=True)
    serializer.save()
    return Response(serializer.data)


@api_view(['POST'])
def scan_trigger(request):
    """Create a Scan record from JSON.

    Expected (minimal):
      - student_id: number (Student.id) OR string (Student.student_id)
      - metrics: { width_val, height_val, uniformity_score, porosity_count, spatter_count, undercut_detected }
      - defects_json?: string[]
      - total_score?: number
      - status?: 'Pass'|'Fail'
      - image_base64?: base64-encoded bytes (optional)
      - image_filename?: string (optional)
    """

    student_id = request.data.get('student_id')
    metrics = request.data.get('metrics') or {}
    defects_json = request.data.get('defects_json') or []
    total_score = request.data.get('total_score')
    status_value = request.data.get('status')

    if student_id is None:
        return Response({'error': 'student_id is required'}, status=status.HTTP_400_BAD_REQUEST)

    # Defaults to keep the endpoint resilient.
    width_val = float(metrics.get('width_val', 0.0))
    height_val = float(metrics.get('height_val', 0.0))
    uniformity_score = float(metrics.get('uniformity_score', 0.0))
    porosity_count = int(metrics.get('porosity_count', 0))
    spatter_count = int(metrics.get('spatter_count', 0))
    undercut_detected = bool(metrics.get('undercut_detected', False))

    # If rubric was provided, optionally derive defects/status.
    rubric_payload = request.data.get('rubric') or None
    if rubric_payload and isinstance(rubric_payload, dict) and not defects_json:
        try:
            if porosity_count > int(rubric_payload.get('maxPorosity', 0)):
                defects_json.append(f"Porosity ({porosity_count})")
            if spatter_count > int(rubric_payload.get('maxSpatter', 0)):
                defects_json.append(f"Excessive Spatter ({spatter_count})")
            if undercut_detected:
                defects_json.append('Undercut')
        except Exception:
            pass

    if status_value not in ('Pass', 'Fail'):
        status_value = 'Pass' if len(defects_json) == 0 else 'Fail'

    if total_score is None:
        # Minimal scoring: penalize defects.
        total_score = max(0, 100 - (len(defects_json) * 15))

    # Build Scan model payload compatible with ScanSerializer.
    scan_payload = {
        'student_id': student_id,
        'score': float(total_score),
        'width_val': width_val,
        'height_val': height_val,
        'uniformity_score': uniformity_score,
        'porosity_count': porosity_count,
        'spatter_count': spatter_count,
        'undercut_detected': undercut_detected,
        'defects_json': defects_json,
        'status': status_value,
    }

    # Optional image
    image_b64 = request.data.get('image_base64')
    image_filename = request.data.get('image_filename') or 'scan.jpg'
    if image_b64:
        try:
            if isinstance(image_b64, str) and ',' in image_b64:
                image_b64 = image_b64.split(',', 1)[1]
            image_bytes = base64.b64decode(image_b64)
            scan_payload['image'] = ContentFile(image_bytes, name=image_filename)
        except Exception:
            # Don't fail the whole scan if the image payload is malformed.
            pass

    serializer = ScanSerializer(data=scan_payload, context={'request': request})
    serializer.is_valid(raise_exception=True)
    scan = serializer.save()

    # Re-serialize to return ScanResult-shaped response.
    return Response(ScanSerializer(scan, context={'request': request}).data, status=status.HTTP_201_CREATED)


@api_view(['POST'])
def calibrate_trigger(request):
    """Return latest calibration data (or a default payload)."""
    latest = CalibrationData.objects.order_by('-created_at').first()
    if latest:
        return Response(CalibrationDataSerializer(latest).data)

    # Default structure expected by the frontend.
    return Response({
        'matrix': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        'distCoeffs': [0, 0, 0, 0, 0],
        'error': 0.0,
    })


@api_view(['POST'])
def calibrate_save(request):
    serializer = CalibrationDataSerializer(data={
        'matrix': request.data.get('matrix') or [],
        'distCoeffs': request.data.get('distCoeffs') or [],
        'error': float(request.data.get('error') or 0.0),
    })
    serializer.is_valid(raise_exception=True)
    serializer.save()
    return Response({}, status=status.HTTP_204_NO_CONTENT)


@api_view(['GET'])
def health(request):
    return Response({'ok': True})


def _safe_resolve_dir(path_value: str) -> Path:
    # This is a desktop-local app; allow absolute paths, but normalize them.
    resolved = Path(path_value).expanduser().resolve()
    return resolved


@api_view(['POST'])
def dataset_list(request):
    base_path = request.data.get('base_path')
    image_subdir = request.data.get('image_subdir', 'images/train')
    label_subdir = request.data.get('label_subdir', 'labels/train')

    if not base_path:
        return Response({'error': 'base_path is required', 'images': [], 'labels': [], 'labels_missing': []}, status=400)

    base_dir = _safe_resolve_dir(str(base_path))
    images_dir = (base_dir / str(image_subdir)).resolve()
    labels_dir = (base_dir / str(label_subdir)).resolve()

    if not images_dir.exists():
        return Response({'error': f'Images directory not found: {images_dir}', 'images': [], 'labels': [], 'labels_missing': []}, status=404)

    image_paths: list[str] = []
    for ext in ('*.jpg', '*.jpeg', '*.png', '*.bmp', '*.webp'):
        image_paths.extend([str(p) for p in images_dir.glob(ext)])
    image_paths = sorted(set(image_paths))

    label_paths: list[str] = []
    if labels_dir.exists():
        label_paths = sorted([str(p) for p in labels_dir.glob('*.txt')])

    image_stems = {Path(p).stem for p in image_paths}
    label_stems = {Path(p).stem for p in label_paths}
    labels_missing = sorted([stem for stem in image_stems if stem not in label_stems])

    return Response({
        'images': image_paths,
        'labels': label_paths,
        'labels_missing': labels_missing,
    })


@api_view(['POST'])
def rdk_test_connection(request):
    host = request.data.get('host')
    user = request.data.get('user', 'root')
    timestamp = request.data.get('timestamp')

    if not host:
        return Response({'message': 'host is required'}, status=400)

    # Side-channel strategy: check the RDK's tiny video/control server health.
    # We treat a successful /status HTTP request as "reachable" + "serviceHealthy".
    url = f'http://{host}:5001/status'
    reachable = False
    service_healthy = False
    message = ''

    try:
        with urlopen(url, timeout=2) as resp:
            reachable = True
            if resp.status == 200:
                service_healthy = True
                try:
                    payload = json.loads(resp.read().decode('utf-8'))
                    message = f"RDK service responded: {payload.get('status', 'ok')}"
                except Exception:
                    message = 'RDK service responded'
            else:
                message = f'RDK status HTTP {resp.status}'
    except URLError as e:
        message = f'RDK not reachable on side-channel: {e}'
    except Exception as e:
        message = f'RDK connection check failed: {e}'

    connected = reachable and service_healthy
    return Response({
        'reachable': reachable,
        'sshAvailable': False,
        'serviceHealthy': service_healthy,
        'connected': connected,
        'status': 'connected' if connected else ('unreachable' if not reachable else 'unknown'),
        'message': message or ('Connected' if connected else 'Not connected'),
        'host': host,
        'timestamp': timestamp or '',
        'user': user,
    })


@api_view(['GET'])
def rdk_status(request):
    return Response({
        'statusCheckAvailable': True,
        'message': 'ok',
        'timestamp': '',
    })

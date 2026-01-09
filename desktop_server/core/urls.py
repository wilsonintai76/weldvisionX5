from django.urls import path, include
from rest_framework.routers import DefaultRouter
from .views import (
    ClassViewSet,
    StudentViewSet,
    ScanViewSet,
    TrainingJobViewSet,
    live_monitor,
    scan_trigger,
    rubric_view,
    calibrate_trigger,
    calibrate_save,
    health,
    dataset_list,
    rdk_test_connection,
    rdk_status,
)

router = DefaultRouter()
router.register(r'classes', ClassViewSet)
router.register(r'students', StudentViewSet)
router.register(r'scans', ScanViewSet)
router.register(r'training', TrainingJobViewSet)

urlpatterns = [
    path('health', health, name='health'),
    path('health/', health, name='health_slash'),
    path('live/', live_monitor, name='live_monitor'),
    path('scan/', scan_trigger, name='scan_trigger'),
    path('rubric/', rubric_view, name='rubric_view'),
    path('calibrate/', calibrate_trigger, name='calibrate_trigger'),
    path('calibrate/save/', calibrate_save, name='calibrate_save'),
    path('dataset/list', dataset_list, name='dataset_list'),
    path('dataset/list/', dataset_list, name='dataset_list_slash'),
    path('rdk/test-connection', rdk_test_connection, name='rdk_test_connection'),
    path('rdk/test-connection/', rdk_test_connection, name='rdk_test_connection_slash'),
    path('rdk/status', rdk_status, name='rdk_status'),
    path('rdk/status/', rdk_status, name='rdk_status_slash'),
    path('', include(router.urls)),
]

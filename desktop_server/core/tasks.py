from celery import shared_task
from .models import TrainingJob
import time

@shared_task
def start_training_task(job_id):
    """
    Simulates a heavy GPU training job.
    In production, this would launch PyTorch processes.
    """
    try:
        job = TrainingJob.objects.get(id=job_id)
        job.status = 'RUNNING'
        job.save()

        # Simulate progress
        for i in range(10, 101, 10):
            time.sleep(1) # Fake work
            job.progress = i
            job.save()

        job.status = 'COMPLETED'
        job.result_message = "Model training finished successfully. Accuracy: 94.5%"
        job.save()
        
    except TrainingJob.DoesNotExist:
        pass
    except Exception as e:
        job = TrainingJob.objects.get(id=job_id)
        job.status = 'FAILED'
        job.result_message = str(e)
        job.save()

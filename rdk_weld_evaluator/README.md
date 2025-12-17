# WeldMaster AI Evaluator

Frontend-orchestrated hybrid Edge AI pipeline for weld inspection on RDK X5.

## Orchestration Overview
- Frontend triggers training, compilation, deployment, and inference via APIs.
- Backend executes heavy jobs (GPU training, Horizon OE Docker compilation).
- RDK X5 agent runs BPU inference (`hobot_dnn`) for 30–40 FPS.

## Quick Start

### Backend (Flask)
- Ensure the Flask app registers `src/api/job_routes.py` blueprint at `/api`.
- Endpoints include:
  - `POST /api/train/start`, `GET /api/train/status/:jobId`
  - `POST /api/compile/start`, `GET /api/compile/status/:jobId`
  - `POST /api/models/deploy`
  - `POST /api/inference/start|stop`

### RDK Agent
- Copy `rdk_agent/bpu_inference.py` to RDK X5 and run:

```bash
python3 /opt/weld_evaluator/bpu_inference.py --model /opt/weld_evaluator/models/weld_quantized.bin
```

### Frontend
- Add panels for Datasets, Training, Compilation, Deployment, Inference.
- Call the `/api/*` endpoints to orchestrate jobs and display logs/metrics.

## Documentation
- See `DEPLOYMENT_INSTRUCTIONS.md` for deployment steps.
- See `AI_MODEL_TRAINING_GUIDE.md` for the full training/compilation pipeline.
- See `DEPLOYMENT_INDEX.md` for a quick map of all docs.

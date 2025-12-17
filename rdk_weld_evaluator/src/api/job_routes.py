from flask import Blueprint, request, jsonify
import uuid
import subprocess
import threading
import os

jobs_bp = Blueprint('jobs', __name__, url_prefix='/api')

# In-memory job registry (replace with persistent storage if needed)
JOB_REGISTRY = {}


def _spawn_job(cmd_list, job_id):
    JOB_REGISTRY[job_id] = {
        'state': 'running',
        'logs': [],
        'result': None,
        'proc': None,
        'cancelled': False,
    }
    try:
        proc = subprocess.Popen(
            cmd_list,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        JOB_REGISTRY[job_id]['proc'] = proc
        for line in proc.stdout:
            JOB_REGISTRY[job_id]['logs'].append(line.rstrip())
            if JOB_REGISTRY[job_id].get('cancelled'):
                break
        if JOB_REGISTRY[job_id].get('cancelled') and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:
                pass
        rc = proc.wait()
        if JOB_REGISTRY[job_id].get('cancelled'):
            JOB_REGISTRY[job_id]['state'] = 'cancelled'
        else:
            JOB_REGISTRY[job_id]['state'] = 'succeeded' if rc == 0 else 'failed'
        JOB_REGISTRY[job_id]['result'] = {'returncode': rc}
    except Exception as e:
        JOB_REGISTRY[job_id]['state'] = 'failed'
        JOB_REGISTRY[job_id]['logs'].append(f'ERROR: {e}')
        JOB_REGISTRY[job_id]['result'] = {'error': str(e)}


@jobs_bp.route('/train/start', methods=['POST'])
def start_train():
    payload = request.get_json(force=True)
    dataset = payload.get('dataset', 'weld_dataset.yaml')
    epochs = int(payload.get('epochs', 50))
    imgsz = int(payload.get('imgsz', 640))
    project = payload.get('project', 'weld_models')
    name = payload.get('name', 'weld_v1')
    device = str(payload.get('device', 0))  # GPU index or 'cpu'

    job_id = str(uuid.uuid4())
    cmd = [
        'python', '-m', 'ultralytics', 'train',
        f'data={dataset}', f'epochs={epochs}', f'imgsz={imgsz}',
        f'project={project}', f'name={name}', f'device={device}',
        'model=yolov8n.pt'
    ]
    threading.Thread(target=_spawn_job, args=(cmd, job_id), daemon=True).start()
    return jsonify({'jobId': job_id}), 202


@jobs_bp.route('/train/status/<job_id>', methods=['GET'])
def train_status(job_id):
    job = JOB_REGISTRY.get(job_id)
    if not job:
        return jsonify({'error': 'job not found'}), 404
    # Return only last 200 lines to limit payload
    logs = job['logs'][-200:]
    return jsonify({'state': job['state'], 'logs': logs, 'result': job['result']}), 200


@jobs_bp.route('/train/cancel/<job_id>', methods=['POST'])
def train_cancel(job_id):
    job = JOB_REGISTRY.get(job_id)
    if not job:
        return jsonify({'error': 'job not found'}), 404
    job['cancelled'] = True
    proc = job.get('proc')
    if proc and proc.poll() is None:
        try:
            proc.terminate()
        except Exception as e:
            job['logs'].append(f'ERROR terminating: {e}')
    job['state'] = 'cancelled'
    job['logs'].append('Job cancelled by user')
    return jsonify({'ok': True, 'state': job['state']}), 200


@jobs_bp.route('/compile/start', methods=['POST'])
def compile_start():
    payload = request.get_json(force=True)
    onnx_path = payload.get('onnxPath', 'weld_v1.onnx')
    config_yaml = payload.get('configYaml', 'weld_config.yaml')
    workspace = payload.get('workspace', os.getcwd())
    docker_image = payload.get('dockerImage', 'horizon/open_explorer:v2.0')

    job_id = str(uuid.uuid4())
    cmd = [
        'docker', 'run', '-it', '--rm',
        '-v', f'{workspace}:/workspace',
        docker_image,
        'bash', '-lc', f'cd /workspace && hb_mapper makertbin --config {config_yaml} --model-type onnx'
    ]
    threading.Thread(target=_spawn_job, args=(cmd, job_id), daemon=True).start()
    return jsonify({'jobId': job_id}), 202


@jobs_bp.route('/compile/status/<job_id>', methods=['GET'])
def compile_status(job_id):
    job = JOB_REGISTRY.get(job_id)
    if not job:
        return jsonify({'error': 'job not found'}), 404
    logs = job['logs'][-200:]
    return jsonify({'state': job['state'], 'logs': logs, 'result': job['result']}), 200


@jobs_bp.route('/compile/cancel/<job_id>', methods=['POST'])
def compile_cancel(job_id):
    job = JOB_REGISTRY.get(job_id)
    if not job:
        return jsonify({'error': 'job not found'}), 404
    job['cancelled'] = True
    proc = job.get('proc')
    if proc and proc.poll() is None:
        try:
            proc.terminate()
        except Exception as e:
            job['logs'].append(f'ERROR terminating: {e}')
    job['state'] = 'cancelled'
    job['logs'].append('Compilation cancelled by user')
    return jsonify({'ok': True, 'state': job['state']}), 200


@jobs_bp.route('/models/deploy', methods=['POST'])
def models_deploy():
    payload = request.get_json(force=True)
    rdk_ip = payload.get('rdkIp')
    bin_path = payload.get('binPath', 'weld_quantized.bin')
    target_path = payload.get('targetPath', '/opt/weld_evaluator/models/weld_quantized.bin')
    user = payload.get('user', 'ubuntu')

    if not rdk_ip:
        return jsonify({'error': 'rdkIp required'}), 400

    job_id = str(uuid.uuid4())
    cmd = [
        'scp', bin_path, f'{user}@{rdk_ip}:{target_path}'
    ]
    threading.Thread(target=_spawn_job, args=(cmd, job_id), daemon=True).start()
    return jsonify({'jobId': job_id}), 202


@jobs_bp.route('/inference/start', methods=['POST'])
def inference_start():
    payload = request.get_json(force=True)
    rdk_ip = payload.get('rdkIp')
    user = payload.get('user', 'ubuntu')
    model_bin = payload.get('modelBin', '/opt/weld_evaluator/models/weld_quantized.bin')
    script_path = payload.get('scriptPath', '/opt/weld_evaluator/bpu_inference.py')

    if not rdk_ip:
        return jsonify({'error': 'rdkIp required'}), 400

    job_id = str(uuid.uuid4())
    cmd = [
        'ssh', f'{user}@{rdk_ip}',
        f'nohup python3 {script_path} --model {model_bin} > /tmp/bpu_infer.log 2>&1 &'
    ]
    threading.Thread(target=_spawn_job, args=(cmd, job_id), daemon=True).start()
    return jsonify({'jobId': job_id}), 202


@jobs_bp.route('/inference/stop', methods=['POST'])
def inference_stop():
    payload = request.get_json(force=True)
    rdk_ip = payload.get('rdkIp')
    user = payload.get('user', 'ubuntu')

    if not rdk_ip:
        return jsonify({'error': 'rdkIp required'}), 400

    job_id = str(uuid.uuid4())
    cmd = [
        'ssh', f'{user}@{rdk_ip}',
        "pkill -f 'bpu_inference.py'"
    ]
    threading.Thread(target=_spawn_job, args=(cmd, job_id), daemon=True).start()
    return jsonify({'jobId': job_id}), 202

#!/usr/bin/env python3
import cv2
import time
import argparse
import requests
import json
import base64
import numpy as np
import threading
from flask import Flask, Response
from flask_cors import CORS
from pathlib import Path

# Import our vision logic
from vision.evaluator import WeldEvaluator

# Mock Calibrator for standalone RDK
class MockCalibrator:
    pass

# Global for video streaming and control
output_frame = None
lock = threading.Lock()
is_scanning = False

# Flask App for Side-Channel Streaming AND Control
app = Flask(__name__)
CORS(app)

@app.route('/start', methods=['POST'])
def start_scan():
    global is_scanning
    is_scanning = True
    print("🚀 API Command: START SCAN")
    return {"status": "scanning"}

@app.route('/stop', methods=['POST'])
def stop_scan():
    global is_scanning
    is_scanning = False
    print("🛑 API Command: STOP SCAN")
    return {"status": "idle"}

@app.route('/status', methods=['GET'])
def get_status():
    global is_scanning
    return {"status": "scanning" if is_scanning else "idle"}

def generate_frames():
    global output_frame, lock
    while True:
        with lock:
            if output_frame is None:
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
               bytearray(encodedImage) + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_stream_server(port):
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True, use_reloader=False)

def main():
    global output_frame, lock, is_scanning
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--desktop_url', type=str, default='http://localhost:8000', help='Django Server URL')
    parser.add_argument('--camera', type=str, default='0', help='Camera ID or Path (0 for webcam)')
    parser.add_argument('--student_id', type=str, default='guest', help='Student ID')
    parser.add_argument('--stream_port', type=int, default=5001, help='Port for MJPEG Stream')
    args = parser.parse_args()

    # Start Streaming Thread
    t = threading.Thread(target=start_stream_server, args=(args.stream_port,))
    t.daemon = True
    t.start()

    # Initialize Vision System
    print("Initializing Vision System for Butt Joint Evaluation...")
    calibrator = MockCalibrator()
    evaluator = WeldEvaluator(calibrator)
    
    # Butt Joint Configuration/Rubric
    rubric = {
        'targetWidth': 8.0,      # mm
        'widthTolerance': 1.5,   # mm
        'targetHeight': 2.0,     # mm
        'heightTolerance': 0.8,  # mm
        'maxPorosity': 0,        # count
        'maxSpatter': 2          # count
    }

    # Open Camera
    try:
        cap = cv2.VideoCapture(int(args.camera))
    except:
        cap = cv2.VideoCapture(args.camera)
        
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")

    print(f"RDK Scan Agent Started. Sending to {args.desktop_url}")
    print(f"🎥 Side-Channel Video Feed available at: http://<RDK_IP>:{args.stream_port}/video_feed")
    print(f"🕹️ Control API available at: http://<RDK_IP>:{args.stream_port}/start | /stop")

    while True:
        # 1. Capture Frame (Always capture for Live View)
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to read frame (or end of stream). Retrying...")
            time.sleep(1)
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "No Camera Signal", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Draw Overlay (Always, but show status)
        display_frame = frame.copy()
        
        if is_scanning:
            # Mock depth
            depth = np.ones((frame.shape[0], frame.shape[1]), dtype=np.float32) * 10.0

            # 2. Process Scan (Only if Scanning)
            # print("Processing frame for Butt Joint Weld...")
            results = evaluator.process_scan(frame, depth, rubric)
            
            cv2.putText(display_frame, f"SCANNING - Score: {results['score']}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 3. Upload to Django (throttled)
            # Here we might want to throttle upload, but process every frame? 
            # For simplicity, let's process/upload every loop if scanning, but add a sleep.
            
            ok, buf = cv2.imencode('.jpg', frame)
            files = {'image': ('scan_capture.jpg', buf.tobytes(), 'image/jpeg')}
            data = {
                'student_id': args.student_id,
                'score': results['score'],
                'width_val': results['metrics']['width_val'],
                'height_val': results['metrics']['height_val'],
                'uniformity_score': results['metrics']['uniformity_score'],
                'porosity_count': results['metrics']['porosity_count'],
                'spatter_count': results['metrics']['spatter_count'],
                'undercut_detected': results['metrics']['undercut_detected'],
                'defects_json': json.dumps(results['defects']),
                'status': results['status']
            }

            try:
                url = f"{args.desktop_url.rstrip('/')}/api/scans/"
                requests.post(url, data=data, files=files, timeout=5)
            except Exception as e:
                print(f"❌ Connection Error during upload: {e}")
            
            time.sleep(1) # Upload 1 FPS max
            
        else:
            cv2.putText(display_frame, "IDLE - Waiting for Command", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            time.sleep(0.03) # 30 FPS preview

        # Update Stream Frame
            output_frame = display_frame.copy()

if __name__ == "__main__":
    main()

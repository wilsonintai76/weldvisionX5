#!/usr/bin/env python3
import cv2
import time
import argparse
import requests
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--desktop_url', type=str, required=True, help='http://DESKTOP_IP:5000')
parser.add_argument('--dataset_dir', type=str, default='/opt/weld_dataset/setA', help='Destination dataset base dir on desktop')
parser.add_argument('--interval', type=float, default=1.0, help='Capture interval seconds')
parser.add_argument('--camera', type=str, default='/dev/video0', help='V4L2 device path')
args = parser.parse_args()

cap = cv2.VideoCapture(args.camera)
if not cap.isOpened():
    raise RuntimeError(f'Cannot open camera {args.camera}')

idx = 1
print('Starting capture loop...')
while True:
    ok, frame = cap.read()
    if not ok:
        print('Frame read failed, retrying...')
        time.sleep(args.interval)
        continue
    # Encode JPEG
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if not ok:
        print('JPEG encode failed')
        time.sleep(args.interval)
        continue
    img_name = f'img_{str(idx).zfill(4)}.jpg'
    label_name = img_name.replace('.jpg', '.txt')
    save_path = str(Path(args.dataset_dir) / 'images' / img_name)
    label_path = str(Path(args.dataset_dir) / 'labels' / label_name)

    files = {'image': ('image.jpg', buf.tobytes(), 'image/jpeg')}
    data = {
        'savePath': save_path.replace('\\','/'),
        'labelPath': label_path.replace('\\','/'),
        # optional: 'labels': json.dumps([...])
    }
    try:
        resp = requests.post(f'{args.desktop_url}/api/dataset/upload_multipart', files=files, data=data, timeout=5)
        if resp.ok:
            print(f'Saved {save_path}')
            idx += 1
        else:
            print('Upload failed:', resp.status_code, resp.text)
    except Exception as e:
        print('Upload error:', e)
    time.sleep(args.interval)

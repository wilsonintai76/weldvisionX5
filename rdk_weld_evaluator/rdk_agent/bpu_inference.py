import argparse
import cv2
import numpy as np
import time
try:
    from hobot_dnn import pyeasy_dnn as dnn
except Exception:
    dnn = None


def main(model_path):
    if dnn is None:
        print('hobot_dnn not available on this host')
        return 1
    models = dnn.load(model_path)
    cap = cv2.VideoCapture(0)
    fps_n = 0
    fps_t = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        img = cv2.resize(frame, (640, 640))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[None, ...]
        t0 = time.time()
        _ = dnn.forward(models, img)
        infer_ms = (time.time() - t0) * 1000
        fps_n += 1
        if fps_n % 30 == 0:
            fps = 30 / (time.time() - fps_t)
            fps_t = time.time()
            print(f'FPS: {fps:.1f}, inference: {infer_ms:.1f} ms')
    cap.release()
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    args = parser.parse_args()
    raise SystemExit(main(args.model))

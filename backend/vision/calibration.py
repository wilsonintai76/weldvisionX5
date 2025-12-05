import cv2
import numpy as np
import yaml
import os

class Calibrator:
    def __init__(self, config_path='config/calibration.yaml'):
        self.config_path = config_path
        self.matrix = None
        self.dist_coeffs = None
        # Create config dir if not exists
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        self.load_config()

    def load_config(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as f:
                    data = yaml.safe_load(f)
                    self.matrix = np.array(data['camera_matrix'])
                    self.dist_coeffs = np.array(data['dist_coeffs'])
            except Exception as e:
                print(f"Failed to load calibration: {e}")

    def save_config(self, matrix, dist, error):
        data = {
            'camera_matrix': matrix.tolist(),
            'dist_coeffs': dist.tolist(),
            'rms_error': error
        }
        with open(self.config_path, 'w') as f:
            yaml.dump(data, f)
        self.matrix = matrix
        self.dist_coeffs = dist

    def run_calibration(self, frames, pattern_size=(9, 6)):
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        
        objpoints = []
        imgpoints = []
        img_size = None

        for frame in frames:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if img_size is None:
                img_size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)

        if not objpoints:
            raise ValueError("No checkerboard patterns detected in frames")

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
        return ret, mtx, dist
import cv2
import numpy as np

class WeldEvaluator:
    def __init__(self, calibration=None):
        self.calibration = calibration

    def measure_bead_width(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, 0.0
        
        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        width_px = min(rect[1]) 
        
        # Simple px to mm conversion (would typically use depth/calibration)
        mm_per_px = 0.05 
        width_mm = width_px * mm_per_px
        
        # Uniformity estimation
        area = cv2.contourArea(c)
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        uniformity = area / hull_area if hull_area > 0 else 0
        
        return width_mm, uniformity

    def measure_height(self, mask, depth_map):
        if depth_map is None:
            return 0.0
        
        # Mask out the bead
        bead_depth = cv2.bitwise_and(depth_map, depth_map, mask=mask)
        # Simple logic: height is diff between bead peak and surrounding base
        if np.count_nonzero(bead_depth) == 0:
            return 0.0
            
        # Filter 0s
        valid_depths = bead_depth[bead_depth > 0]
        min_dist = np.min(valid_depths) # Closest point = Peak
        
        # Sample base metal (inverted mask)
        base_mask = cv2.bitwise_not(mask)
        base_depth = cv2.bitwise_and(depth_map, depth_map, mask=base_mask)
        valid_base = base_depth[base_depth > 0]
        
        if len(valid_base) == 0:
            return 0.0
            
        base_avg = np.mean(valid_base)
        
        # Height = Base - Peak
        height_mm = max(0.0, base_avg - min_dist)
        return height_mm

    def detect_porosity(self, roi_gray):
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 5
        params.maxArea = 50
        params.filterByCircularity = True
        params.minCircularity = 0.5
        
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(roi_gray)
        return len(keypoints)

    def detect_spatter(self, image, bead_mask):
        """
        Detects spatter by looking for small disconnected blobs in the base metal area.
        """
        # 1. Invert mask to focus on base metal (ignore the main weld bead)
        base_metal_mask = cv2.bitwise_not(bead_mask)
        
        # 2. Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 3. Thresholding
        # Use adaptive thresholding to detect high-contrast irregularities (spatter) on the base metal
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY_INV, 11, 2)
        
        # 4. Mask out the bead so we only see thresholded spots on base metal
        spatter_candidates = cv2.bitwise_and(thresh, thresh, mask=base_metal_mask)
        
        # 5. Find contours of these spots
        contours, _ = cv2.findContours(spatter_candidates, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        spatter_count = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 6. Filter by area to distinguish actual spatter from sensor noise (<3) or large structures (>60)
            if 3 < area < 60: 
                spatter_count += 1
                
        return spatter_count

    def process_scan(self, image, depth, rubric):
        # ROI Segmentation (Color based for demo)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_val = np.array([0, 0, 50])
        upper_val = np.array([180, 50, 200])
        mask = cv2.inRange(hsv, lower_val, upper_val)
        
        # Clean mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        width, uniformity = self.measure_bead_width(mask)
        height = self.measure_height(mask, depth)
        
        roi_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        roi_gray = cv2.bitwise_and(roi_gray, roi_gray, mask=mask)
        
        porosity = self.detect_porosity(roi_gray)
        spatter = self.detect_spatter(image, mask)
        
        undercut = False 
        
        # Grading against Rubric
        defects = []
        if porosity > rubric.get('maxPorosity', 0): defects.append(f"Porosity ({porosity})")
        if spatter > rubric.get('maxSpatter', 2): defects.append(f"Spatter ({spatter})")
        
        t_width = rubric.get('targetWidth', 8.0)
        tol_width = rubric.get('widthTolerance', 1.0)
        if abs(width - t_width) > tol_width: defects.append("Width Error")
        
        t_height = rubric.get('targetHeight', 2.0)
        tol_height = rubric.get('heightTolerance', 0.5)
        if abs(height - t_height) > tol_height: defects.append("Height Error")
        
        score = max(0, 100 - len(defects) * 20)
        status = "Pass" if not defects else "Fail"

        return {
            "metrics": {
                "width_val": round(width, 2),
                "height_val": round(height, 2),
                "uniformity_score": round(uniformity, 2),
                "porosity_count": porosity,
                "spatter_count": spatter,
                "undercut_detected": undercut
            },
            "defects": defects,
            "score": score,
            "status": status
        }
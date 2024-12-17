import cv2
import numpy as np
import os
import glob

def calibrate_camera_fisheye(camera_folder):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    objp = np.zeros((1, 6*9, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob(os.path.join(camera_folder, '*.jpg'))
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
        
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
        else:
            print(f"Chessboard not found in {fname}")
            os.remove(fname)  # Optionally, remove this line if you want to keep images for review.

    if objpoints and imgpoints:
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW)
        ret, mtx, dist, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints, imgpoints, gray.shape[::-1], K, D, [], [], flags, criteria)
        
        np.savez(os.path.join(camera_folder, 'calibration_data_fisheye'), mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
        return mtx, dist, rvecs, tvecs
    else:
        print("No valid images available for calibration.")
        return None, None, None, None

if __name__ == "__main__":
    for i in range(1, 6):
        folder = f'Camera_{i}'
        print(f"Calibrating {folder}")
        calibrate_camera_fisheye(folder)
        print(f"Calibration completed for {folder}")
    print("All cameras calibrated.")

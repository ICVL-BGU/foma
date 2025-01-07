""" 
This script calibrates the cameras using the images captured from the cameras.
The script assumes that the images are stored in the respective camera directories.
The script uses OpenCV to calibrate the cameras and save the calibration parameters.
The calibration parameters are saved in the respective camera directories.
The script assumes that the chessboard pattern is 6x9.
"""

import cv2
import glob
import numpy as np
import os
import pickle

camera_dirs = [f"G:/My Drive/Alex/School/MSc/Thesis/FOV/CodeBackUp/etc/Camera_{i}" for i in range(1,6)]

CV_CALIB_USE_INTRINSIC_GUESS = 1
CV_CALIB_ZERO_TANGENT_DIST = 8
CV_CALIB_RATIONAL_MODEL = 16384

# prepare object points: (0, 0, 0), (1, 0, 0), ...
objp = np.zeros((6*9, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)

for camera_dir in camera_dirs:
    objps = []
    imgps = []

    for img in glob.glob(camera_dir + '/*.jpg'):
        im = cv2.imread(img)
        gray_im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        # find chessboard corners
        pattern_found, corners = cv2.findChessboardCorners(gray_im, (6, 9))
        if pattern_found:
            objps.append(objp)
            imgps.append(corners)
            # refine corner locations
            corners = cv2.cornerSubPix(gray_im, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    cv2.destroyAllWindows()

    # initialize matrix initial coefficient for intrinsic_guess is true
    matrix_init = np.zeros((3, 3), np.float32)
    matrix_init[0][0] = 1328.89  # f_x
    matrix_init[0][2] = 1280.0   # c_x
    matrix_init[1][1] = 1493.33  # f_y
    matrix_init[1][2] = 960.0    # c_y
    matrix_init[2][2] = 1.0
    dist_init = np.zeros((1, 4), np.float32)

    # Perform camera calibration
    pattern_found, K, D, rvecs, tvecs = cv2.calibrateCamera(objps, imgps, gray_im.shape[::-1], matrix_init, dist_init, flags=CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_RATIONAL_MODEL)
    
    # Save calibration parameters
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (gray_im.shape[1], gray_im.shape[0]), 1, (gray_im.shape[1], gray_im.shape[0]))
    calib_params = {
        'K': K,
        'D': D,
        'new_camera_mtx': (new_camera_mtx, roi)
    }

    with open(os.path.join(camera_dir, 'calib_params.pkl'), 'wb') as f:
        pickle.dump(calib_params, f)

    print(f"Calibration parameters for {camera_dir} saved.")

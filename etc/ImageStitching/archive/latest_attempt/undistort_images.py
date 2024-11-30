import cv2
import glob
import numpy as np
import os
import pickle
import matplotlib.pyplot as plt

camera_dirs = [f"Camera_{i}" for i in range(1,6)]

def undistort_image(image_path, calib_params):
    img = cv2.imread(image_path)
    K = calib_params['K']
    D = calib_params['D']
    new_camera_mtx, roi = calib_params['new_camera_mtx']
    h, w = img.shape[:2]

    # Undistort the image
    undistorted_img = cv2.undistort(img, K, D, None, new_camera_mtx)
    
    # Crop the image based on the ROI (Region of Interest)
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y + h, x:x + w]
    
    return img, undistorted_img

# Load calibration parameters and undistort images for each camera
for camera_dir in camera_dirs:
    with open(os.path.join(camera_dir, 'calib_params_multi.pkl'), 'rb') as f:
        calib_params = pickle.load(f)

    # Load an image to undistort
    img_path = glob.glob(camera_dir + '/*.jpg')[0]
    original_img, undistorted_img = undistort_image(img_path, calib_params)

    # Display the original and undistorted images side by side using Matplotlib
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.title(f'Original Image - {os.path.basename(camera_dir)}')
    plt.imshow(cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB))
    plt.axis('off')

    plt.subplot(1, 2, 2)
    plt.title(f'Undistorted Image - {os.path.basename(camera_dir)}')
    plt.imshow(cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB))
    plt.axis('off')

    plt.show()

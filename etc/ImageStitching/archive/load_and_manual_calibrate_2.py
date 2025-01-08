import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

def load_calibration_data(camera_folder):
    try:
        data = np.load(os.path.join(camera_folder, 'calibration_data_fisheye.npz'))
        return data['mtx'], data['dist']
    except FileNotFoundError:
        print(f"Calibration data not found for {camera_folder}.")
        return None, None

def undistort_image(img, mtx, dist):
    if mtx is None or dist is None:
        return img
    h, w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, (w, h), cv2.CV_16SC2)
    dst = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return dst

def process_and_display_images():
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))  # Setup a 2x3 plot grid
    axs = axs.flatten()
    camera_positions = [2, 5, 4, 0, 3]  # Position indices in a 2x3 flattened array
    rotations = [0, 90, 0, 90, 0]  # Degrees of rotation for cam2 and cam4

    for i in range(1, 6):
        folder = f'Camera_{i}'
        mtx, dist = load_calibration_data(folder)
        if mtx is None or dist is None:
            continue  # Skip this camera if calibration data is not available
        
        img_path = os.path.join('ManualCalibrationImages', f'Cam{i}.jpg')
        img = cv2.imread(img_path)
        if img is None:
            print(f"No image found in {img_path}")
            continue
        
        print(f"Processing {img_path}, Shape: {img.shape}")
        
        img = undistort_image(img, mtx, dist)
        print(f"Undistorted shape: {img.shape}")
        
        if i in [2, 4]:  # Rotate images from cam2 and cam4
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        ax = axs[camera_positions[i-1]]
        ax.imshow(img_rgb)
        ax.axis('off')
        ax.set_title(f'Camera {i}')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    process_and_display_images()

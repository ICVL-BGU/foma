import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

def load_calibration_data(camera_folder):
    # Load the saved calibration data from the file
    data = np.load(os.path.join(camera_folder, 'calibration_data.npz'))
    return data['mtx'], data['dist']

def undistort_image(img, mtx, dist):
    # Apply undistortion to the image using the camera matrix and distortion coefficients
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # Crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def process_and_display_images():
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))  # Setup a 2x3 plot grid
    axs = axs.flatten()
    camera_positions = [2, 5, 4, 0, 3]  # Position indices in a 2x3 flattened array
    rotations = [0, 90, 0, 90, 0]  # Degrees of rotation for cam2 and cam4

    for i in range(1, 6):
        folder = f'Camera_{i}'
        mtx, dist = load_calibration_data(folder)
        img_path = os.path.join('ManualCalibrationImages', f'Cam{i}.jpg')  # Adjust filename as needed
        img = cv2.imread(img_path)
        print(f"Processing {img_path}",f"Shape: {img.shape}")
        if img is None:
            print(f"No image found in {img_path}")
            continue
        
        # Undistort the image
        img = undistort_image(img, mtx, dist)
        print(f"Undistorted shape: {img.shape}")
        
        # Rotate images from cam2 and cam4 by 90 degrees clockwise
        if i == 2 or i == 4:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        
        # Convert image to RGB for plotting
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Display the image in the specified grid position
        ax = axs[camera_positions[i-1]]
        ax.imshow(img_rgb)
        ax.axis('off')
        ax.set_title(f'Camera {i}')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    process_and_display_images()

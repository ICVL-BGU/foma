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

def stitch_images(images):
    # Create a stitcher object
    stitcher = cv2.Stitcher.create()
    # Stitch the images
    status, stitched = stitcher.stitch(images)
    if status == cv2.Stitcher_OK:
        return stitched
    else:
        print("Stitching failed: ", status)
        return None

def process_and_stitch_images():
    images = []
    for i in range(1, 6):
        folder = f'Camera_{i}'
        mtx, dist = load_calibration_data(folder)
        img_path = os.path.join('ManualCalibrationImages', f'Cam{i}.jpg')  # Adjust filename as needed
        img = cv2.imread(img_path)
        if img is None:
            print(f"No image found in {img_path}")
            continue
        
        # Undistort the image
        img = undistort_image(img, mtx, dist)
        
        # Rotate images from cam2 and cam4 by 90 degrees clockwise
        if i == 2 or i == 4:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        
        images.append(img)
    
    # Stitch the images
    stitched_image = stitch_images(images)
    
    if stitched_image is not None:
        # Convert stitched image to RGB for plotting
        stitched_image_rgb = cv2.cvtColor(stitched_image, cv2.COLOR_BGR2RGB)
        
        # Display the stitched image
        plt.figure(figsize=(20, 10))
        plt.imshow(stitched_image_rgb)
        plt.axis('off')
        plt.title('Stitched Image')
        plt.show()
    else:
        print("Image stitching was not successful.")

if __name__ == "__main__":
    process_and_stitch_images()

""" 
This script captures live feed from multiple cameras and allows the user to take screenshots by clicking on the feed. 
The screenshots are saved in the respective camera directories. 
The script can be used to capture images for manual calibration. 
Press 'q' to quit the program and 'n' to switch between cameras. 
The script uses OpenCV to capture and display the camera feed. 
The script assumes that the cameras are connected via RTSP.
"""

import cv2
import os
import time

def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Take a screenshot on left mouse button click
        current_camera_index, frame = param
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        folder_name = f'Camera_{current_camera_index+1}'
        filename = os.path.join(folder_name, f"{timestamp}.jpg")
        cv2.imwrite(filename, frame)
        print(f"Screenshot saved: {filename}")

def capture_and_display_cameras():
    camera_count = 5  # Number of cameras
    current_camera_index = 0
    cameras = []

    # Initialize all cameras and create directories for screenshots
    for i in range(camera_count):
        cam = cv2.VideoCapture(f'rtsp://admin:icvl2023@1.1.2.10{i+1}:554')
        if not cam.isOpened():
            print(f"Cannot open camera {i+1}")
        else:
            cameras.append(cam)
            os.makedirs(f'Camera_{i+1}', exist_ok=True)

    # Set up window for display
    cv2.namedWindow("Live Camera Feed", cv2.WINDOW_NORMAL)

    # Set mouse callback with the current camera index and frame as parameters
    cv2.setMouseCallback("Live Camera Feed", mouse_click, (current_camera_index, None))

    while True:
        # Get current camera based on index
        if len(cameras) > 0:
            cam = cameras[current_camera_index]

            ret, frame = cam.read()
            if not ret:
                print(f"Failed to grab frame from camera {current_camera_index+1}")
                continue
            cv2.imshow("Live Camera Feed", frame)

            # Update the frame in the callback parameter for screenshot purposes
            cv2.setMouseCallback("Live Camera Feed", mouse_click, (current_camera_index, frame))

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('n'):
                # Move to the next camera
                current_camera_index = (current_camera_index + 1) % len(cameras)

    # Clean up
    for cam in cameras:
        cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_and_display_cameras()

import cv2
import numpy as np
import os
import glob

def calibrate_camera(camera_folder):
    # Criteria for termination of the corner refinement process
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Prepare object points, assuming the chessboard is 9x6
    objp = np.zeros((6*9, 3), np.float32)
    objp[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob(os.path.join(camera_folder, '*.jpg'))
    
    for fname in images:
        print(fname)
        img = cv2.imread(fname)
        # print(img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
        
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            
            # Draw and display the corners
            # cv2.drawChessboardCorners(img, (9,6), corners2, ret)
            # cv2.imshow('img', img)
            # cv2.waitKey(500)
        else:
            # If no chessboard found, delete the image
            os.remove(fname)
            print(f"Deleted {fname} as chessboard could not be found.")

    # cv2.destroyAllWindows()
    
    # Calibration
    if objpoints and imgpoints:  # Check if there were any valid images
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        
        # Save matrices to disk
        np.savez(os.path.join(camera_folder, 'calibration_data'), mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
        return mtx, dist, rvecs, tvecs
    else:
        print("No valid images available for calibration.")
        return None, None, None, None

if __name__ == "__main__":
    for i in range(1, 6):
        folder = f'Camera_{i}'
        print(f"Calibrating {folder}")
        calibrate_camera(folder)
        print(f"Calibration completed for {folder}")
    print("All cameras calibrated.")

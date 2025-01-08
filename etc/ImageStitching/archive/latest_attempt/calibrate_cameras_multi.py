import cv2
import glob
import numpy as np
import os
import pickle
import argparse

# Constants for calibration flags
CV_CALIB_USE_INTRINSIC_GUESS = 1
CV_CALIB_ZERO_TANGENT_DIST = 8
CV_CALIB_RATIONAL_MODEL = 16384

# Prepare object points: (0, 0, 0), (1, 0, 0), ...
objp = np.zeros((6 * 9, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)

def find_chessboards(gray_im, pattern_size=(6, 9)):
    """ Find all chessboards in an image and return their corners and object points. """
    chessboards = []
    search_region = gray_im.copy()
    max_chessboards = 5  # Maximum number of chessboards to detect to avoid infinite loops
    chessboard_idx = 0
    while True:
        pattern_found, corners = cv2.findChessboardCorners(search_region, pattern_size)
        if pattern_found:
            chessboard_idx += 1
            print(f"Chessboard {chessboard_idx} found.")
            refined_corners = cv2.cornerSubPix(gray_im, corners, (11, 11), (-1, -1), 
                                               criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            chessboards.append((objp, refined_corners))
            # Mask out the detected chessboard area to find another one in the remaining image
            mask = np.zeros_like(search_region)
            cv2.fillConvexPoly(mask, np.int32(refined_corners), 255)
            search_region = cv2.bitwise_and(search_region, search_region, mask=cv2.bitwise_not(mask))
        else:
            break

    return chessboards

def parse_camera_ranges(camera_range_str, max_camera_num):
    """ Parse the camera range string and return a list of camera indices. """
    camera_indices = []
    if camera_range_str:
        ranges = camera_range_str.split(',')
        for r in ranges:
            if '-' in r:
                start, end = map(int, r.split('-'))
                camera_indices.extend(range(start, end + 1))
            else:
                camera_indices.append(int(r))
    else:
        camera_indices = list(range(1, max_camera_num + 1))
    return camera_indices

def main(base_dir, camera_range_str, fx, fy, cx, cy, output_dir):
    subdirectories = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    max_camera_num = max([int(d) for d in subdirectories if d.isdigit()], default=0)

    camera_indices = parse_camera_ranges(camera_range_str, max_camera_num)
    camera_dirs = [os.path.join(base_dir, f"{i}") for i in camera_indices]

    for camera_idx, camera_dir in enumerate(camera_dirs):
        objps = []
        imgps = []
    
        for image_idx, img_path in enumerate(glob.glob(camera_dir + '/*.jpg')):
            print(f"Processing image {image_idx + 1} of camera {camera_idx+1}...")
            im = cv2.imread(img_path)
            gray_im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

            chessboards = find_chessboards(gray_im)
            for objp, corners in chessboards:
                objps.append(objp)
                imgps.append(corners)

        cv2.destroyAllWindows()

        # Initialize matrix initial coefficient for intrinsic_guess is true
        matrix_init = np.zeros((3, 3), np.float32)
        matrix_init[0][0] = fx  # f_x
        matrix_init[0][2] = cx  # c_x
        matrix_init[1][1] = fy  # f_y
        matrix_init[1][2] = cy  # c_y
        matrix_init[2][2] = 1.0
        dist_init = np.zeros((1, 4), np.float32)

        # Perform camera calibration
        pattern_found, K, D, rvecs, tvecs = cv2.calibrateCamera(objps, imgps, gray_im.shape[::-1], matrix_init, dist_init, 
                                                                flags=CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_RATIONAL_MODEL)

        # Save calibration parameters
        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (gray_im.shape[1], gray_im.shape[0]), 1, (gray_im.shape[1], gray_im.shape[0]))
        calib_params = {
            'K': K,
            'D': D,
            'new_camera_mtx': (new_camera_mtx, roi)
        }

        camera_id = os.path.basename(camera_dir)
        output_file = os.path.join(output_dir, f'calib_params_{camera_id}.pkl')
        with open(output_file, 'wb') as f:
            pickle.dump(calib_params, f)

        print(f"Calibration parameters for camera {camera_id} saved to {output_file}.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Camera calibration using chessboard images.")
    parser.add_argument('-d','--base_dir', type=str, required=True, help="Base directory containing subdirectories with camera images.")
    parser.add_argument('-r','--camera_range', type=str, default="", help="Comma-separated list of camera indices or ranges (e.g., '1-5,7-8,11-25'). Default is all cameras.")
    parser.add_argument('--fx', type=float, default=1328.89, help="Initial guess for focal length in x direction (f_x).")
    parser.add_argument('--fy', type=float, default=1493.33, help="Initial guess for focal length in y direction (f_y).")
    parser.add_argument('--cx', type=float, default=1280.0, help="Initial guess for principal point in x direction (c_x).")
    parser.add_argument('--cy', type=float, default=960.0, help="Initial guess for principal point in y direction (c_y).")
    parser.add_argument('-o','--output_dir', type=str, default=os.getcwd(), help="Directory to save the calibration parameters. Default is the current working directory.")
    
    args = parser.parse_args()
    main(args.base_dir, args.camera_range, args.fx, args.fy, args.cx, args.cy, args.output_dir)

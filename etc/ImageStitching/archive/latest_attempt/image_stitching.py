import cv2
import os
import pickle
from funcs import *
from chessboardmatcher import ChessboardMatcher

def compute_point_sets(images):
    point_sets = {}
    # runs once because of a bug
    matcher = ChessboardMatcher(images[0], images[0])
    matcher.plot_corners()

    for i in range(len(images)-1):
        for j in range(i+1, len(images)):
            matcher = ChessboardMatcher(images[i], images[j])
            matcher.plot_corners()
            if matcher.selected_corners1 and matcher.selected_corners2:
                point_sets[(i, j)] = (matcher.selected_corners1, matcher.selected_corners2)
    return point_sets

def compute_and_save_homographies():
    images = []
    homographies = {}
    for i in range(1, 6):
        folder = f'Camera_{i}'
        K, D, new_camera_mtx = load_calibration_data(folder)
        img_path = os.path.join('ManualCalibrationImages', f'Cam{i}.jpg')
        img = cv2.imread(img_path)
        if img is None:
            print(f"No image found in {img_path}")
            continue
        img = undistort_image(img, K, D, new_camera_mtx)
        images.append(img)

    # Pairwise matching with the base image (Camera 3)
    base_idx = 2  # Camera 3
    
    matcher = ChessboardMatcher(images[base_idx], images[0])
    matcher.plot_corners()
    for i in range(len(images)):
        if i == base_idx:
            continue
        print(f"Comparing Camera {i+1} with base Camera 3")  # Debug print statement
        matcher = ChessboardMatcher(images[base_idx], images[i])
        matcher.plot_corners()

        if matcher.selected_corners1 and matcher.selected_corners2:
            try:
                H, _ = cv2.findHomography(np.array(matcher.selected_corners2), np.array(matcher.selected_corners1), cv2.RANSAC, 5.0)
                homographies[(base_idx, i)] = H
                print(f"Successfully found homography between base image and image {i}")
            except cv2.error as e:
                print(f"Error finding homography between base image and image {i}: {e}")
        else:
            print(f"No matching corners found between base image and image {i}")

    # Save homographies to disk
    with open('homographies_ransac.pkl', 'wb') as f:
        pickle.dump(homographies, f)

    print("Homographies have been saved to disk.")

def main():
    path = 'ManualCalibrationImages'
    base_idx = 2
    images = load_images(path)
    calib_params = [load_calibration_data(os.path.join(f'Camera_{i}','calib_params_multi.pkl')) for i in range(1,len(images)+1)]
    undistorted_images = undistort_images(images, calib_params)
    point_sets = compute_point_sets(undistorted_images)
    with open('points_pairs.pkl', 'wb') as f:
        pickle.dump(point_sets, f)
    # with open('points_pairs.pkl', 'rb') as f:
    #     point_sets = pickle.load(f)
    # for idxs, sets in point_sets.items():
    #     print(idxs)
    #     print(sets)
    # print(point_sets)
    homographies = compute_homographies(point_sets, base_idx)
    # homographies = optimize_homographies(homographies, point_sets, base_idx)
    with open('homographies.pkl', 'wb') as f:
        pickle.dump(homographies, f)
    print(homographies)
    

if __name__ == "__main__":
    main()

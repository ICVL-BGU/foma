import os
import pickle
import cv2
import numpy as np
import matplotlib.pyplot as plt

def load_images(folder_path):
    images = []
    for img_path in os.listdir(folder_path):
        if img_path.endswith('.jpg'):
            img = cv2.imread(os.path.join(folder_path, img_path))
            if img is not None:
                images.append(img)
    return images

def load_calibration_data(calibration_path):
    with open(calibration_path, 'rb') as f:
        calib_params = pickle.load(f)
    return calib_params# calib_params['K'], calib_params['D'], calib_params['new_camera_mtx']

def undistort_image(img, calib_params):
    K = calib_params['K']
    D = calib_params['D']
    new_camera_mtx = calib_params['new_camera_mtx']
    new_camera_mtx, _ = new_camera_mtx
    undistorted_img = cv2.undistort(img, K, D, None, new_camera_mtx)
    return undistorted_img

def undistort_images(images, calib_params):
    undistorted_images = []
    for i, img in enumerate(images):
        undistorted_img = undistort_image(img, calib_params[i])
        undistorted_images.append(undistorted_img)
    return undistorted_images

def compute_homographies(point_sets, base_idx):
    '''Compute homographies between corresponding point sets.
    Can also be used as initial homography for later refinement.'''
    homographies = []    
    for idxs, sets in point_sets.items():
        # print(idxs)
        if base_idx not in idxs:
            continue
        # print(sets[0][0], sets[1][0])
        if len(sets[0]) != len(sets[1]):
            raise ValueError("Number of points in point sets must be equal.")
        if idxs[0] == base_idx:
            dstPoints, srcPoints = sets
        else:
            srcPoints, dstPoints = sets
        srcPoints, dstPoints = np.array(srcPoints), np.array(dstPoints)
        homographies.append(cv2.findHomography(srcPoints, dstPoints, 0)[0])
    return homographies

def optimize_homographies(homographies, point_sets, base_idx):
    # TODO
    '''Refine homographies using Levenberg-Marquardt optimization.'''
    return homographies

def stitch_images(images, homographies, base_idx):
    # Use the image from camera 3 as the base image
    base_img = images[base_idx]
    h, w, _ = base_img.shape

    # Determine the size of the resulting stitched image
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2)
    all_corners = []
    all_corners.extend(corners)
    for i in range(len(images)):
        if i == 2:
            continue
        if (2, i) in homographies:
            H = homographies[(2, i)]
            warped_corners = cv2.perspectiveTransform(corners, H)
            all_corners.extend(warped_corners)

    all_corners = np.array(all_corners)
    [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel())
    [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel())
    t = [-xmin, -ymin]
    H_translate = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)

    # Create the final stitched image
    stitched_img = np.zeros((ymax - ymin, xmax - xmin, 3), dtype=np.float32)
    weight_map = np.zeros((ymax - ymin, xmax - xmin), dtype=np.float32)

    # Add base image
    warped_base_img = cv2.warpPerspective(base_img, H_translate, (xmax - xmin, ymax - ymin)).astype(np.float32)
    mask_base = (warped_base_img > 0).astype(np.float32).max(axis=2)
    stitched_img += warped_base_img * mask_base[..., None]
    weight_map += mask_base

    # Add other images
    for i in range(len(images)):
        if i == 2:
            continue
        if (2, i) in homographies:
            H = H_translate @ homographies[(2, i)]
            warped_img = cv2.warpPerspective(images[i], H, (xmax - xmin, ymax - ymin)).astype(np.float32)
            mask = (warped_img > 0).astype(np.float32).max(axis=2)
            stitched_img += warped_img * mask[..., None]
            weight_map += mask

    # Normalize the stitched image
    weight_map[weight_map == 0] = 1  # Avoid division by zero
    stitched_img /= weight_map[..., None]

    # Convert to uint8
    stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)

    # Display the final stitched image
    stitched_image_rgb = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2RGB)

    cv2.imwrite("stitched_simple_RANSAC.jpg", stitched_image_rgb)
    plt.figure(figsize=(20, 10))
    plt.imshow(stitched_image_rgb)
    plt.axis('off')
    plt.title('Final Stitched Image')
    plt.show()
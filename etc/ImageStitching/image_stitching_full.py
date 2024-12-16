import argparse
from itertools import combinations
import logging
import cv2
import glob
import numpy as np
import os
import pickle
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import largestinteriorrectangle as lir
import time

# Constants for calibration flags
CV_CALIB_USE_INTRINSIC_GUESS = 1
CV_CALIB_ZERO_TANGENT_DIST = 8
CV_CALIB_RATIONAL_MODEL = 16384

# Prepare object points: (0, 0, 0), (1, 0, 0), ...
objp = np.zeros((6 * 9, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)

class ChessboardMatcher:
    def __init__(self, img1, img2, pattern_size=(9, 6)):
        self.img1 = img1
        self.img2 = img2
        self.pattern_size = pattern_size
        self.corners1 = self.detect_multiple_chessboards(img1)
        self.corners2 = self.detect_multiple_chessboards(img2)
        logging.info(f"Found {len(self.corners1)} chessboards in image 1 and {len(self.corners2)} chessboards in image 2.")
        self.selected_corners1 = []
        self.selected_corners2 = []
        self.matched_pairs = set()
        self.matched_corners1 = set()
        self.matched_corners2 = set()
        self.current_pair = (0, 0)
        
    def detect_multiple_chessboards(self, img):
        img = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Chessboard Detection', gray)
        # cv2.waitKey(0)
        corners_list = []
        found = True
        while found:
            found, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            # logging.info(f"Found chessboard: {found}")
            if found:
                cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                corners_list.append(corners)
                cv2.drawChessboardCorners(gray, self.pattern_size, corners, found)
        return corners_list

    def plot_corners(self):
        while self.current_pair[0] < len(self.corners1) and self.current_pair[1] < len(self.corners2):
            if (self.current_pair[0] in self.matched_corners1 or 
                self.current_pair[1] in self.matched_corners2 or 
                (self.current_pair[0], self.current_pair[1]) in self.matched_pairs):
                if self.current_pair[1] + 1 < len(self.corners2):
                    self.current_pair = (self.current_pair[0], self.current_pair[1] + 1)
                elif self.current_pair[0] + 1 < len(self.corners1):
                    self.current_pair = (self.current_pair[0] + 1, 0)
                else:
                    plt.close()
                    return
            else:
                break

        if self.current_pair[0] >= len(self.corners1) or self.current_pair[1] >= len(self.corners2):
            plt.close()
            return

        img1_with_corners = self.img1.copy()
        img2_with_corners = self.img2.copy()

        cv2.drawChessboardCorners(img1_with_corners, self.pattern_size, self.corners1[self.current_pair[0]], True)
        cv2.drawChessboardCorners(img2_with_corners, self.pattern_size, self.corners2[self.current_pair[1]], True)

        fig, axs = plt.subplots(1, 2, figsize=(15, 5))

        # Maximize the plot window
        mng = plt.get_current_fig_manager()
        if mng is not None:
            mng.full_screen_toggle()

        axs[0].imshow(cv2.cvtColor(img1_with_corners, cv2.COLOR_BGR2RGB))
        axs[0].set_title('Image 1')
        axs[1].imshow(cv2.cvtColor(img2_with_corners, cv2.COLOR_BGR2RGB))
        axs[1].set_title('Image 2')

        ax_accept = plt.axes((0.65, 0.05, 0.1, 0.075))
        ax_reject = plt.axes((0.76, 0.05, 0.1, 0.075))
        ax_skip = plt.axes((0.87, 0.05, 0.1, 0.075))
        self.btn_accept = Button(ax_accept, 'Accept')
        self.btn_reject = Button(ax_reject, 'Reject')
        self.btn_skip = Button(ax_skip, 'Skip')

        self.btn_accept.on_clicked(self.accept_corners)
        self.btn_reject.on_clicked(self.reject_corners)
        self.btn_skip.on_clicked(self.skip_corners)

        plt.show()

    def accept_corners(self, event):
        self.selected_corners1.extend(self.corners1[self.current_pair[0]])
        self.selected_corners2.extend(self.corners2[self.current_pair[1]])
        self.matched_pairs.add((self.current_pair[0], self.current_pair[1]))
        self.matched_corners1.add(self.current_pair[0])
        self.matched_corners2.add(self.current_pair[1])
        self.next_pair()

    def reject_corners(self, event):
        self.next_pair()

    def skip_corners(self, event):
        self.matched_corners1.add(self.current_pair[0])
        self.next_pair()

    def next_pair(self):
        if self.current_pair[1] + 1 < len(self.corners2):
            self.current_pair = (self.current_pair[0], self.current_pair[1] + 1)
        elif self.current_pair[0] + 1 < len(self.corners1):
            self.current_pair = (self.current_pair[0] + 1, 0)
        else:
            plt.close()
            return
        plt.close()
        self.plot_corners()

def find_chessboards(gray_im, pattern_size=(6, 9)):
    """ Find all chessboards in an image and return their corners and object points. """
    chessboards = []
    search_region = gray_im.copy()

    while True:
        pattern_found, corners = cv2.findChessboardCorners(search_region, pattern_size, flags = cv2.CALIB_CB_FAST_CHECK)
        if pattern_found:
            refined_corners = cv2.cornerSubPix(gray_im, corners, (11, 11), (-1, -1), 
                                               criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) 
            chessboards.append((objp, refined_corners))
            mask = np.zeros_like(search_region)
            cv2.fillConvexPoly(mask, np.int32(refined_corners.reshape(-1, 2)), (255, 255, 255))
            search_region = cv2.bitwise_and(search_region, search_region, mask=cv2.bitwise_not(mask))
        else:
            break

    return chessboards

def parse_ranges(camera_range_str, max_camera_num):
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

def calibrate_cameras(base_dir, calib_range, fx, fy, cx, cy, resize_factor):
    cameras = os.path.join(base_dir, 'cameras')
    subdirectories = [d for d in os.listdir(cameras)]
    max_camera_num = max([int(d) for d in subdirectories if d.isdigit()], default=0)

    camera_indices = parse_ranges(calib_range, max_camera_num)
    camera_dirs = [os.path.join(cameras, f"{i}") for i in camera_indices]

    calib_params_path = os.path.join(base_dir, "calib_params.pkl")
    if os.path.exists(calib_params_path):
        with open(calib_params_path, "rb") as f:
            calib_params = pickle.load(f)
    else:
        calib_params = {}

    for camera_dir in camera_dirs:
        objps = []
        imgps = []
        logging.info("Processing camera calibration parameters for %s", os.path.basename(camera_dir))

        for i, img_path in enumerate(glob.glob(camera_dir + '/*.jpg')):
            logging.info("Processing image %d/%d: %s", i+1, len(glob.glob(camera_dir + '/*.jpg')) ,img_path)
            im = cv2.imread(img_path)
            resized_im = cv2.resize(im, (0, 0), fx=resize_factor, fy=resize_factor)
            gray_im = cv2.cvtColor(resized_im, cv2.COLOR_BGR2GRAY)

            chessboards = find_chessboards(gray_im)
            for j, (objp, corners) in enumerate(chessboards):
                logging.info("Processing chessboard %d/%d", j+1, len(chessboards))
                objps.append(objp)
                imgps.append(corners)

        cv2.destroyAllWindows()

        # Initialize matrix initial coefficient for intrinsic_guess is true
        matrix_init = np.zeros((3, 3), np.float32)
        matrix_init[0][0] = fx * resize_factor  # f_x
        matrix_init[0][2] = cx * resize_factor  # c_x
        matrix_init[1][1] = fy * resize_factor  # f_y
        matrix_init[1][2] = cy * resize_factor  # c_y
        matrix_init[2][2] = 1.0
        dist_init = np.zeros((1, 4), np.float32)

        # Perform camera calibration
        _, K, D, _, _ = cv2.calibrateCamera(objps, imgps, gray_im.shape[::-1], matrix_init, dist_init, 
                                            flags=CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_RATIONAL_MODEL)

        # Save calibration parameters
        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (gray_im.shape[1], gray_im.shape[0]), 1, (gray_im.shape[1], gray_im.shape[0]))
        camera_id = os.path.basename(camera_dir)
        calib_params[int(camera_id)] = {
            'K': K,
            'D': D,
            'mtx': new_camera_mtx,
            'roi': roi
        }

        logging.info(f"Calibration parameters for camera {camera_id} processed.")

        # Save all calibration parameters to a single file
        logging.info(f"Saving calibration parameters to {calib_params_path} for camera {camera_id}.")
        with open(calib_params_path, 'wb') as f:
            pickle.dump(calib_params, f)

    logging.info(f"All calibration parameters saved to {calib_params_path}.")
    return calib_params

def match_images(base_dir, match_range, calib_params, resize_factor):
    images_path = os.path.join(base_dir, 'final_images')
    max_image_num = len(os.listdir(images_path))

    images_indices = parse_ranges(match_range, max_image_num)
    # images = {i: cv2.imread(os.path.join(images_path, f"{i}.jpg")) for i in images_indices}

    images = {i: cv2.resize(cv2.imread(os.path.join(images_path, f"{i}.jpg")), (0, 0), fx=resize_factor, fy=resize_factor) for i in images_indices}

    matched_points_path = os.path.join(base_dir, "matched_points.pkl")
    if os.path.exists(matched_points_path):
        with open(matched_points_path, "rb") as f:
            matched_points = pickle.load(f)
    else:
        matched_points = {}

    for i in images_indices:
        K, D, mtx = calib_params[i]['K'], calib_params[i]['D'], calib_params[i]['mtx']
        images[i] = cv2.undistort(images[i], K, D, None, mtx)
    
    index = list(images.keys())[0]
    matcher = ChessboardMatcher(images[index], images[index])
    matcher.plot_corners()

    for i, j in combinations(images_indices, 2):
        if i == j:
            continue
        logging.info(f"Comparing Camera {i} with Camera {j}")
        matcher = ChessboardMatcher(images[i], images[j])
        matcher.plot_corners()

        if matcher.selected_corners1 and matcher.selected_corners2:
            matched_points[(i, j)] = (matcher.selected_corners1, matcher.selected_corners2)
            logging.info(f"Successfully matched points between {i} and image {j}")
        else:
            logging.warning(f"No matching corners found between {i} and image {j}")

    # Save matched points to disk
    with open(matched_points_path, 'wb') as f:
        pickle.dump(matched_points, f)

    logging.info(f"Matched points have been saved to {matched_points_path}.")

    return matched_points

def compute_homographies(base_dir, homographies_range, calib_params, matched_points, base_idx):
    images_path = os.path.join(base_dir, 'final_images')
    max_image_num = len(os.listdir(images_path))

    images_indices = parse_ranges(homographies_range, max_image_num)
    images = {i: cv2.imread(os.path.join(images_path, f"{i}.jpg")) for i in images_indices}

    homographies_path = os.path.join(base_dir, "homographies.pkl")
    if os.path.exists(homographies_path):
        with open(homographies_path, "rb") as f:
            homographies = pickle.load(f)
    else:
        homographies = {}

    for i in images_indices:
        K, D, mtx = calib_params[i]['K'], calib_params[i]['D'], calib_params[i]['mtx']
        images[i] = cv2.undistort(images[i], K, D, None, mtx)

    # Compute homographies
    for idx in images_indices:
        if idx == base_idx:
            continue
        base_points, other_points = matched_points[(base_idx, idx)] if base_idx < idx else matched_points[(idx, base_idx)][::-1]
        try:
            H, _ = cv2.findHomography(np.array(other_points), np.array(base_points), 0)  # cv2.RANSAC, 5.0
            homographies[(base_idx, idx)] = H
            logging.info(f"Successfully found homography between base image and image {idx}")
        except cv2.error as e:
            logging.error(f"Error finding homography between base image and image {idx}: {e}")

    # Save homographies to disk
    with open(homographies_path, 'wb') as f:
        pickle.dump(homographies, f)

    logging.info(f"Homographies have been saved to {homographies_path}.")

    return homographies

# Feather blending with gain compensation

# def compute_gain_compensations(images, base_idx, homographies):
#     num_images = len(images)
#     gains = np.ones(num_images)

#     for i in range(num_images):
#         if i == base_idx:
#             continue
#         if (base_idx, i) in homographies:
#             H = homographies[(base_idx, i)]
#             warped_base = cv2.warpPerspective(images[base_idx], H, (images[i].shape[1], images[i].shape[0]))
#             mask_base = cv2.warpPerspective(np.ones_like(images[base_idx], dtype=np.uint8) * 255, H, (images[i].shape[1], images[i].shape[0]))
#             overlap = (mask_base > 0) & (images[i] > 0)
#             if np.sum(overlap) > 0:
#                 diff = np.mean(warped_base[overlap] / (images[i][overlap] + 1e-8))
#                 gains[i] = diff
#             else:
#                 gains[i] = 1.0  # If no overlap, use default gain

#     avg_gain = np.mean(gains)
#     gains = gains / avg_gain

#     return gains

# def feather_blending(images, masks, output_shape):
#     blender = cv2.detail_FeatherBlender()
#     blender.setSharpness(1.0 / 50.0)  # Adjust sharpness for better blending
#     blender.prepare((0, 0, output_shape[1], output_shape[0]))
#     for img, mask in zip(images, masks):
#         blender.feed(cv2.UMat(img.astype(np.int16)), cv2.UMat(mask), (0, 0))
#     result, result_mask = blender.blend(None, None)
#     return result.get() if isinstance(result, cv2.UMat) else result

# def stitch_images(base_dir, calib_params, homographies, base_idx, num_bands=5, sigma=1):
#     images_path = Path(base_dir) / 'final_images'
#     images_indices = [int(i.stem) for i in images_path.glob('*') if i.suffix in {'.jpg', '.png', '.bmp', '.jpeg'}]
#     images = {i: cv2.imread(str(images_path / f"{i}.jpg")) for i in images_indices}

#     for i in images_indices:
#         K, D, mtx = calib_params[i]['K'], calib_params[i]['D'], calib_params[i]['mtx']
#         images[i] = cv2.undistort(images[i], K, D, None, mtx)
    
#     base_img = images[base_idx]
#     h, w = base_img.shape[:2]

#     corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2)
#     all_corners = []
#     all_corners.extend(corners)
#     for i in images_indices:
#         if i == base_idx:
#             continue
#         if (base_idx, i) in homographies:
#             H = homographies[(base_idx, i)]
#             warped_corners = cv2.perspectiveTransform(corners, H)
#             all_corners.extend(warped_corners)

#     all_corners = np.array(all_corners)
#     [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel())
#     [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel())
#     t = [-xmin, -ymin]
#     H_translate = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)

#     output_width = xmax - xmin
#     output_height = ymax - ymin

#     warped_images = []
#     masks = []
#     for i in images_indices:
#         if i == base_idx:
#             H = H_translate
#         else:
#             H = H_translate @ homographies[(base_idx, i)]
#         warped_image = cv2.warpPerspective(images[i], H, (output_width, output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
#         mask = (warped_image > 0).astype(np.uint8) * 255
#         masks.append(mask[:, :, 0])
#         warped_images.append(warped_image)

#     # Compute and apply gain compensations
#     gains = compute_gain_compensations(images, base_idx, homographies)
#     for i in range(len(warped_images)):
#         warped_images[i] = cv2.normalize((warped_images[i].astype(np.float32) * gains[i]), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

#     # Blend images with feather blending
#     result = feather_blending(warped_images, masks, (output_height, output_width))

#     stitched_img = cv2.normalize(result, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

#     stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)

#     stitched_image_rgb = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2RGB)

#     plt.figure(figsize=(20, 10))
#     plt.imshow(stitched_image_rgb)
#     plt.axis('off')
#     plt.title('Final Stitched Image')
#     plt.show()

#     stitched_image_path = os.path.join(base_dir, 'stitched_image.jpg')
#     cv2.imwrite(stitched_image_path, stitched_img)

#     return stitched_image_rgb




# Stitching the ROI of the images instead of the entire image, also dropping the base image

def feather_blending(images, masks, output_shape):
    blender = cv2.detail_FeatherBlender()
    blender.setSharpness(1.0 / 50.0)  # Adjust sharpness for better blending
    blender.prepare((0, 0, output_shape[1], output_shape[0]))
    for img, mask in zip(images, masks):
        blender.feed(cv2.UMat(img.astype(np.int16)), cv2.UMat(mask), (0, 0))
    result, result_mask = blender.blend(None, None)
    return result.get() if isinstance(result, cv2.UMat) else result

def find_and_save_largest_inscribed_rectangle(mask, image, filepath):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)

    # Fill the contour to ensure no internal black pixels
    cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

    mask_bool = mask.astype(bool)  # Convert mask to boolean array
    largest_rect = lir.lir(mask_bool, largest_contour[:, 0, :])

    with open(filepath, 'wb') as f:
        pickle.dump(largest_rect, f)

    x, y, w, h = largest_rect
    cropped_image = image[y:y+h, x:x+w]
    return cropped_image

def stitch_images(base_dir, calib_params, homographies, base_idx, resize_factor, num_bands=5, sigma=1):
    if resize_factor != 1.0:
        # logging.warning("Resizing images to %.2f", resize_factor)
        S = np.array([[resize_factor, 0, 0],
              [0, resize_factor, 0],
              [0, 0, 1]])
        for h in homographies:
            # logging.info(f"Resizing homography ({h[0]}, {h[1]})")
            # logging.info(f"Resizing homography ({homographies[h]})")
            homographies[h] = S @ homographies[h]# @ S
            # logging.info(f"New homography: {homographies[h]}")
    
    images_path = os.path.join(base_dir, 'final_images')
    images_files = [f for f in os.listdir(images_path) if f.lower().endswith(('.jpg', '.png', '.bmp', '.jpeg'))]
    images_indices = [int(os.path.splitext(f)[0]) for f in images_files]
    # images = {i: cv2.resize(cv2.imread(os.path.join(images_path, f"{i}.jpg")), (0, 0), fx=resize_factor, fy=resize_factor) for i in images_indices}
    images = {i: cv2.imread(os.path.join(images_path, f"{i}.jpg")) for i in images_indices}

    # Step 1: Undistort Images
    for i in images_indices:
        K, D, mtx = calib_params[i]['K'], calib_params[i]['D'], calib_params[i]['mtx']
        images[i] = cv2.undistort(images[i], K, D, None, mtx)

    base_img = images[base_idx]
    h, w = base_img.shape[:2]

    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2) # * resize_factor

    all_corners = []
    all_corners.extend(corners)
    for i in images_indices:
        if i == base_idx:
            continue
        if (base_idx, i) in homographies:
            H = homographies[(base_idx, i)]
            warped_corners = cv2.perspectiveTransform(corners, H)
            # logging.info(f"Warped corners for image {i}: {warped_corners}")
            all_corners.extend(warped_corners)

    all_corners = np.array(all_corners)
    [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel())
    [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel())
    t = [-xmin, -ymin]
    H_translate = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)

    output_width = xmax - xmin
    output_height = ymax - ymin

    warped_images = []
    masks = []
    for i in images_indices:
        if i == base_idx:
            continue
        H = H_translate @ homographies[(base_idx, i)]
        warped_image = cv2.warpPerspective(images[i], H, (output_width, output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        # Create a mask based on the ROI
        roi = calib_params[i]['roi']
        x, y, w, h = roi
        mask = np.zeros((images[i].shape[0], images[i].shape[1]), dtype=np.uint8)
        mask[y:y+h, x:x+w] = 255

        # Warp the mask using the same homography
        warped_mask = cv2.warpPerspective(mask, H, (output_width, output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        # Apply the mask to the warped image
        masked_warped_image = cv2.bitwise_and(warped_image, warped_image, mask=warped_mask)

        masks.append(warped_mask)
        warped_images.append(masked_warped_image)

    # Step 3: Blend images with feather blending
    result = feather_blending(warped_images, masks, (output_height, output_width))
    stitched_img = cv2.normalize(result, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)
    stitched_image_rgb = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2RGB)

    # Step 4: Find and crop the largest inscribed rectangle
    mask = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
    
    lir_filepath = 'lir.pkl'
    if os.path.exists(lir_filepath):
        logging.info("Loading largest inscribed rectangle.")
        with open(lir_filepath, 'rb') as f:
            largest_rect = pickle.load(f)
        x, y, w, h = largest_rect
        cropped_image = stitched_image_rgb[y:y+h, x:x+w]
    else:
        logging.info("Largest inscribed rectangle not found. Computing and saving.")
        cropped_image = find_and_save_largest_inscribed_rectangle(mask, stitched_image_rgb, lir_filepath)

    stitched_image_path = os.path.join(base_dir, 'stitched_image.jpg')
    cv2.imwrite(stitched_image_path, stitched_img)

    cropped_image_path = os.path.join(base_dir, 'cropped_image.jpg')
    cv2.imwrite(cropped_image_path, cropped_image)

    return stitched_image_rgb, cropped_image




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Camera calibration and image stitching using chessboard images.")

    # Working directory containing everything
    parser.add_argument('-d', '--dir', type=str, required=True, help="Base directory")
    parser.add_argument('-b', '--base_image', type=int, default=3, help="Base image index (default: 3)")

    # Stage 1: Camera calibration
    parser.add_argument('-c', '--calibrate', action='store_true', help="Perform camera calibration.")
    parser.add_argument('-cr', '--calib_range', type=str, help="Comma-separated list of camera indices or ranges (e.g., '1-5,7-8,11-25').")
    parser.add_argument('-rf', '--resize_factor', type=float, default=1.0, help="Resize factor for the images.")
    parser.add_argument('--fx', type=float, default=1328.89, help="Initial guess for focal length in x direction (f_x).")
    parser.add_argument('--fy', type=float, default=1493.33, help="Initial guess for focal length in y direction (f_y).")
    parser.add_argument('--cx', type=float, default=1280.0, help="Initial guess for principal point in x direction (c_x).")
    parser.add_argument('--cy', type=float, default=960.0, help="Initial guess for principal point in y direction (c_y).")

    # Stage 2: Match chessboards between images
    parser.add_argument('-m', '--match', action='store_true', help="Perform chessboard matching between images.")
    parser.add_argument('-mr', '--match_range', type=str, help="Comma-separated list of camera indices or ranges (e.g., '1-5,7-8,11-25').")
    
    # Stage 3 Compute homographies
    parser.add_argument('-ch', '--compute_homographies', action='store_true', help="Compute homographies using the calibration parameters.")
    parser.add_argument('-hr', '--homographies_range', type=str, help="Comma-separated list of camera indices or ranges (e.g., '1-5,7-8,11-25').")
    
    # Stage 4: Stitch images
    parser.add_argument('-s', '--stitch', action='store_true', help="Stitch images using the computed homographies.")
    parser.add_argument('-l', '--log_level', type=str, default='INFO', help="Set the logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL).")

    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(level=getattr(logging, args.log_level.upper()), format='%(asctime)s - %(levelname)s - %(message)s')

    logging.info("Script started.")


    if args.calibrate:
        logging.info("Starting calibration.")
        calib_params = calibrate_cameras(args.dir, args.calib_range, args.fx, args.fy, args.cx, args.cy, args.resize_factor)

    if args.match:
        if not args.calibrate:
            logging.info("Loading calibration parameters.")
            calib_params = pickle.load(open(os.path.join(args.dir, "calib_params.pkl"), "rb"))
        logging.info("Matching chessboards between images.")
        matched_points = match_images(args.dir, args.match_range, calib_params, args.resize_factor)

    if args.compute_homographies:
        if not args.calibrate:
            logging.info("Loading calibration parameters.")
            calib_params = pickle.load(open(os.path.join(args.dir, "calib_params.pkl"), "rb"))
        if not args.match:
            logging.info("Loading matched points.")
            matched_points = pickle.load(open(os.path.join(args.dir, "matched_points.pkl"), "rb"))
        logging.info("Computing homographies.")
        homographies = compute_homographies(args.dir, args.homographies_range, calib_params, matched_points, args.base_image)

    if args.stitch:
        if not args.calibrate:
            logging.info("Loading calibration parameters.")
            calib_params = pickle.load(open(os.path.join(args.dir, "calib_params.pkl"), "rb"))
        if not args.compute_homographies:
            logging.info("Loading homographies.")
            homographies = pickle.load(open(os.path.join(args.dir, "homographies.pkl"), "rb"))
        logging.info("Stitching images.")
        stitch_images(args.dir, calib_params, homographies, args.base_image, args.resize_factor)

    logging.info("Script finished.")

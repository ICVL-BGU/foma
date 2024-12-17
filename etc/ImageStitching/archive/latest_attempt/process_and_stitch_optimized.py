import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import pickle
from scipy.optimize import minimize

class ChessboardMatcher:
    def __init__(self, img1, img2, pattern_size=(9, 6)):
        self.img1 = img1
        self.img2 = img2
        self.pattern_size = pattern_size
        self.corners1 = self.detect_multiple_chessboards(img1)
        self.corners2 = self.detect_multiple_chessboards(img2)
        self.selected_corners1 = []
        self.selected_corners2 = []
        self.matched_pairs = set()
        self.matched_corners1 = set()
        self.matched_corners2 = set()
        self.current_pair = (0, 0)
        
    def detect_multiple_chessboards(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners_list = []
        found = True
        while found:
            found, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
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
        mng.full_screen_toggle()

        axs[0].imshow(cv2.cvtColor(img1_with_corners, cv2.COLOR_BGR2RGB))
        axs[0].set_title('Image 1')
        axs[1].imshow(cv2.cvtColor(img2_with_corners, cv2.COLOR_BGR2RGB))
        axs[1].set_title('Image 2')

        ax_accept = plt.axes([0.65, 0.05, 0.1, 0.075])
        ax_reject = plt.axes([0.76, 0.05, 0.1, 0.075])
        ax_skip = plt.axes([0.87, 0.05, 0.1, 0.075])
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

def load_calibration_data(camera_folder):
    with open(os.path.join(camera_folder, 'calib_params.pkl'), 'rb') as f:
        calib_params = pickle.load(f)
    return calib_params['K'], calib_params['D'], calib_params['new_camera_mtx']

def undistort_image(img, K, D, new_camera_mtx):
    new_camera_mtx, roi = new_camera_mtx
    undistorted_img = cv2.undistort(img, K, D, None, new_camera_mtx)
    return undistorted_img

def project_points(H, points):
    points_h = np.hstack([points, np.ones((points.shape[0], 1))])
    projected_points_h = np.dot(H, points_h.T).T
    projected_points = projected_points_h[:, :2] / projected_points_h[:, 2:]
    return projected_points

def compute_initial_homographies(images, base_idx, points_pairs):
    initial_homographies = []
    for i in range(len(images)):
        if i == base_idx:
            initial_homographies.append(np.eye(3).ravel())  # Identity matrix for base image
        else:
            pts_base, pts_i = points_pairs[(min(base_idx, i), max(base_idx, i))]
            if i > base_idx:
                H, _ = cv2.findHomography(pts_i, pts_base, 0)
            else:
                H, _ = cv2.findHomography(pts_base, pts_i, 0)
            initial_homographies.append(H.ravel())
    initial_params = np.hstack(initial_homographies)
    return initial_params

def optimize_homographies(points_pairs, base_idx, num_images, initial_params):
    result = minimize(residuals, initial_params, args=(points_pairs, base_idx, num_images), method='L-BFGS-B', options={'maxfun': 15000})
    
    optimized_homographies = []
    for i in range(num_images):
        if i == base_idx:
            optimized_homographies.append(np.eye(3))
        else:
            start_idx = (i - 1) * 9 if i > base_idx else i * 9
            H = result.x[start_idx : start_idx + 9].reshape(3, 3)
            optimized_homographies.append(H)

    return optimized_homographies


def residuals(params, points_pairs, base_idx, num_images):
    homographies = []
    for i in range(num_images):
        if i == base_idx:
            homographies.append(np.eye(3))
        else:
            start_idx = (i - 1) * 9 if i > base_idx else i * 9
            homographies.append(params[start_idx : start_idx + 9].reshape(3, 3))

    res = []
    for (i, j), (pts_i, pts_j) in points_pairs.items():
        H_i = homographies[i]
        H_j = homographies[j]
        proj_i = project_points(H_i, pts_i)
        proj_j = project_points(H_j, pts_j)
        res.append((proj_i - proj_j).ravel())

    res = np.concatenate(res)
    return np.sum(res ** 2)


def find_and_save_points():
    images = []
    points_pairs = {}
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

    ChessboardMatcher(images[0], images[0]).plot_corners()

    for i in range(len(images)):
        for j in range(i + 1, len(images)):
            print(f"Comparing Camera {i+1} with Camera {j+1}")
            matcher = ChessboardMatcher(images[i], images[j])
            matcher.plot_corners()
            if matcher.selected_corners1 and matcher.selected_corners2:
                points_pairs[(i, j)] = (np.array(matcher.selected_corners1).reshape(-1, 2), 
                                        np.array(matcher.selected_corners2).reshape(-1, 2))
            else:
                print(f"No matching corners found between Camera {i+1} and Camera {j+1}")

    with open('points_pairs.pkl', 'wb') as f:
        pickle.dump(points_pairs, f)

    print("Points pairs have been saved to disk.")

def read_points_and_optimize():
    with open('points_pairs.pkl', 'rb') as f:
        points_pairs = pickle.load(f)
    
    if not points_pairs:
        print("No points pairs found in the loaded data.")
        return
    print("Points pairs loaded successfully.")

    base_idx = 2  # Camera 3 as the base image
    num_images = 5
    
    images = []
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
    
    if len(images) != num_images:
        print(f"Expected {num_images} images but loaded {len(images)}.")
        return
    
    initial_params = compute_initial_homographies(images, base_idx, points_pairs)
    print("Initial homographies computed.")
    
    optimized_homographies = optimize_homographies(points_pairs, base_idx, num_images, initial_params)
    
    for idx, H in enumerate(optimized_homographies):
        if H.shape != (3, 3):
            print(f"Invalid homography shape for image {idx}: {H.shape}")
            return
        print(f"Optimized homography for image {idx}: {H}")
    
    with open('optimized_homographies.pkl', 'wb') as f:
        pickle.dump(optimized_homographies, f)
    
    print("Optimized homographies have been saved to disk.")

def stitch_images_using_homographies(image_files):
    with open('optimized_homographies.pkl', 'rb') as f:
        optimized_homographies = pickle.load(f)

    images = []
    for img_path in image_files:
        folder = f'Camera_{img_path[-5]}'  # Extract camera folder name
        K, D, new_camera_mtx = load_calibration_data(folder)
        img = cv2.imread(img_path)
        if img is None:
            print(f"No image found in {img_path}")
            continue
        img = undistort_image(img, K, D, new_camera_mtx)
        images.append(img)

    base_img = images[2]
    h, w = base_img.shape[:2]

    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2)
    all_corners = []
    all_corners.extend(corners)
    for i in range(len(images)):
        if i == 2:
            continue
        H = optimized_homographies[i]
        if H.shape != (3, 3):
            print(f"Invalid homography shape for image {i}: {H.shape}")
            continue
        warped_corners = cv2.perspectiveTransform(corners, H)
        all_corners.extend(warped_corners)

    all_corners = np.array(all_corners)
    [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel())
    [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel())
    t = [-xmin, -ymin]
    H_translate = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)

    stitched_img = np.zeros((ymax - ymin, xmax - xmin, 3), dtype=np.float32)
    weight_map = np.zeros((ymax - ymin, xmax - xmin), dtype=np.float32)

    warped_base_img = cv2.warpPerspective(base_img, H_translate, (xmax - xmin, ymax - ymin)).astype(np.float32)
    mask_base = (warped_base_img > 0).astype(np.float32).max(axis=2)
    stitched_img += warped_base_img * mask_base[..., None]
    weight_map += mask_base

    for i in range(len(images)):
        if i == 2:
            continue
        if i >= len(optimized_homographies) or optimized_homographies[i].shape != (3, 3):
            print(f"Invalid homography shape for image {i}: {optimized_homographies[i].shape if i < len(optimized_homographies) else 'not found'}")
            continue
        H = H_translate @ optimized_homographies[i]
        warped_img = cv2.warpPerspective(images[i], H, (xmax - xmin, ymax - ymin)).astype(np.float32)
        mask = (warped_img > 0).astype(np.float32).max(axis=2)
        stitched_img += warped_img * mask[..., None]
        weight_map += mask

    weight_map[weight_map == 0] = 1
    stitched_img /= weight_map[..., None]
    stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)

    stitched_image_rgb = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2RGB)

    cv2.imwrite("stitched_optimized.jpg", stitched_image_rgb)
    plt.figure(figsize=(20, 10))
    plt.imshow(stitched_image_rgb)
    plt.axis('off')
    plt.title('Final Stitched Image')
    plt.show()

if __name__ == "__main__":
    # compute_and_save_homographies()
    # find_and_save_points()
    read_points_and_optimize()
    # Replace with the paths to the images you want to stitch
    image_files = [os.path.join('ManualCalibrationImages', f'Cam{i}.jpg') for i in range(1, 6)]
    stitch_images_using_homographies(image_files)

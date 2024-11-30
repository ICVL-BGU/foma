import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import pickle

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
    with open(os.path.join(camera_folder, 'calib_params_multi.pkl'), 'rb') as f:
        calib_params = pickle.load(f)
    return calib_params['K'], calib_params['D'], calib_params['new_camera_mtx']

def undistort_image(img, K, D, new_camera_mtx):
    new_camera_mtx, roi = new_camera_mtx
    undistorted_img = cv2.undistort(img, K, D, None, new_camera_mtx)
    return undistorted_img

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
                H, _ = cv2.findHomography(np.array(matcher.selected_corners2), np.array(matcher.selected_corners1), 0)# cv2.RANSAC, 5.0)
                homographies[(base_idx, i)] = H
                print(f"Successfully found homography between base image and image {i}")
            except cv2.error as e:
                print(f"Error finding homography between base image and image {i}: {e}")
        else:
            print(f"No matching corners found between base image and image {i}")

    # Save homographies to disk
    with open('homographies.pkl', 'wb') as f:
        pickle.dump(homographies, f)

    print("Homographies have been saved to disk.")

def stitch_images_using_homographies(image_files):
    with open('homographies.pkl', 'rb') as f:
        homographies = pickle.load(f)

    # formatting cause I'm dumb:
    homographies = {(2, i if i<2 else i+1): homographies[i] for i in range(len(homographies))}

    # with open('homographies.pkl', 'rb') as f:
    #     homographies = pickle.load(f)

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

    # Use the image from camera 3 as the base image
    base_img = images[2]
    h, w = base_img.shape[:2]

    # Determine the size of the resulting stitched image
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2)
    all_corners = []
    all_corners.extend(corners)
    for i in range(len(images)):
        if i == 2:
            continue
        if (2, i) in homographies:
            H = homographies[(2, i)]
            print(H,H.shape)
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

    # cv2.imwrite("stitched_simple_RANSAC.jpg", stitched_img)#stitched_image_rgb)
    plt.figure(figsize=(20, 10))
    plt.imshow(stitched_image_rgb)
    plt.axis('off')
    plt.title('Final Stitched Image')
    plt.show()

if __name__ == "__main__":
    # compute_and_save_homographies()
    # Replace with the paths to the images you want to stitch
    image_files = [os.path.join('ManualCalibrationImages', f'Cam{i}.jpg') for i in range(1, 6)]
    stitch_images_using_homographies(image_files)

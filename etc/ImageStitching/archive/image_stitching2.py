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
        self.corners1 = self.detect_chessboard_corners(img1)
        self.corners2 = self.detect_chessboard_corners(img2)
        self.selected_corners = []

    def detect_chessboard_corners(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
        if ret:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        return corners if ret else []

    def plot_corners(self):
        img1_with_corners = self.img1.copy()
        img2_with_corners = self.img2.copy()

        if len(self.corners1) > 0:
            cv2.drawChessboardCorners(img1_with_corners, self.pattern_size, self.corners1, True)
        if len(self.corners2) > 0:
            cv2.drawChessboardCorners(img2_with_corners, self.pattern_size, self.corners2, True)

        fig, axs = plt.subplots(1, 2, figsize=(15, 5))

        # Maximize the plot window
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()

        axs[0].imshow(cv2.cvtColor(img1_with_corners, cv2.COLOR_BGR2RGB))
        axs[0].set_title('Image 1')
        axs[1].imshow(cv2.cvtColor(img2_with_corners, cv2.COLOR_BGR2RGB))
        axs[1].set_title('Image 2')

        ax_accept = plt.axes([0.7, 0.05, 0.1, 0.075])
        ax_reject = plt.axes([0.81, 0.05, 0.1, 0.075])
        self.btn_accept = Button(ax_accept, 'Accept')
        self.btn_reject = Button(ax_reject, 'Reject')

        self.btn_accept.on_clicked(self.accept_corners)
        self.btn_reject.on_clicked(self.reject_corners)

        plt.show()

    def accept_corners(self, event):
        self.selected_corners.append((self.corners1, self.corners2))
        self.next_corners()

    def reject_corners(self, event):
        self.next_corners()

    def next_corners(self):
        plt.close()

def load_calibration_data(camera_folder):
    data = np.load(os.path.join(camera_folder, 'calibration_data.npz'))
    return data['mtx'], data['dist']

def undistort_image(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def compute_and_save_homographies():
    images = []
    homographies = {}
    for i in range(1, 6):
        folder = f'Camera_{i}'
        mtx, dist = load_calibration_data(folder)
        img_path = os.path.join('ManualCalibrationImages', f'Cam{i}.jpg')
        img = cv2.imread(img_path)
        if img is None:
            print(f"No image found in {img_path}")
            continue
        img = undistort_image(img, mtx, dist)
        if i == 2 or i == 4:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        images.append(img)

    all_selected_corners = []
    # Pairwise matching for all combinations
    for i in range(len(images)):
        for j in range(i + 1, len(images)):
            matcher = ChessboardMatcher(images[i], images[j])
            matcher.plot_corners()
            all_selected_corners.extend(matcher.selected_corners)
            if matcher.selected_corners:
                (corners1, corners2) = matcher.selected_corners[0]
                H, _ = cv2.findHomography(corners2, corners1, cv2.RANSAC, 5.0)
                homographies[(i, j)] = H
                H_inv, _ = cv2.findHomography(corners1, corners2, cv2.RANSAC, 5.0)
                homographies[(j, i)] = H_inv

    # Save homographies to disk
    with open('homographies.pkl', 'wb') as f:
        pickle.dump(homographies, f)

    print("Homographies have been saved to disk.")

def stitch_images_using_homographies(image_files):
    with open('homographies.pkl', 'rb') as f:
        homographies = pickle.load(f)

    images = []
    for img_path in image_files:
        img = cv2.imread(img_path)
        if img is None:
            print(f"No image found in {img_path}")
            continue
        images.append(img)

    base_img = images[0]
    h, w = base_img.shape[:2]

    for i in range(1, len(images)):
        H = homographies[(0, i)]
        warped_img = cv2.warpPerspective(images[i], H, (w * len(images), h))
        base_img = cv2.addWeighted(base_img, 0.5, warped_img, 0.5, 0)

    # Display the stitched image
    stitched_image_rgb = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
    plt.figure(figsize=(20, 10))
    plt.imshow(stitched_image_rgb)
    plt.axis('off')
    plt.title('Stitched Image')
    plt.show()

if __name__ == "__main__":
    compute_and_save_homographies()
    # Replace with the paths to the images you want to stitch
    image_files = [os.path.join('ManualCalibrationImages', f'Cam{i}.jpg') for i in range(1, 6)]
    stitch_images_using_homographies(image_files)

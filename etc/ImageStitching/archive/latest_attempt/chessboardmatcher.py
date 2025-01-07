import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

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
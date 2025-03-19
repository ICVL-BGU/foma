import cv2
import numpy as np

# Load the image
img = cv2.imread("G:\My Drive\Alex\School\BGU\Research\Projects\FOMA\Code\FOMA\FOMA\output\lidar_detection.png")
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to reduce noise
blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

# Perform edge detection (Canny edge detector)
edges = cv2.Canny(blurred_img, 30, 50, apertureSize=3)

# Apply Hough Line Transform to detect straight lines
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10)

# Create a copy of the image to draw the lines on
img_copy = img.copy()

# If lines are detected, draw them
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)

# Now we need to find two lines that form an "X"
# A simple heuristic could be to check for perpendicular lines, forming the X
if lines is not None:
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            x1, y1, x2, y2 = lines[i][0]
            x3, y3, x4, y4 = lines[j][0]
            
            # Calculate angle between the two lines
            angle1 = np.arctan2(y2 - y1, x2 - x1)
            angle2 = np.arctan2(y4 - y3, x4 - x3)
            
            # Check if the two lines are perpendicular (around 90 degrees)
            angle_diff = np.abs(angle1 - angle2)
            if np.pi / 4 < angle_diff < 3 * np.pi / 4:  # angles close to 90 degrees
                # Draw the two lines that form the X shape
                cv2.line(img_copy, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.line(img_copy, (x3, y3), (x4, y4), (255, 0, 0), 2)

# Display the image with detected X shape
cv2.imshow('X Detection', img_copy)
cv2.waitKey(0)
cv2.destroyAllWindows()

import cv2
import numpy as np

# Load the image
image_path = r"G:\My Drive\Alex\School\BGU\Research\Projects\FOMA\Code\FOMA\FOMA\output\lidar_detection.png"  # Replace with your image path
image = cv2.imread(image_path)

# Convert the image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Updated HSV bounds for olive green
lower_olive = np.array([30, 100, 150])  
upper_olive = np.array([80, 255, 255])  

# Threshold the image to get only olive green colors
mask = cv2.inRange(hsv, lower_olive, upper_olive)

# Find contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Process detected contours
if contours:
    for i, contour in enumerate(contours):
        x, y, w, h = cv2.boundingRect(contour)

        # Create a mask for the specific contour (only keeping the detected shape)
        contour_mask = np.zeros_like(mask)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        # Extract HSV values only inside the detected contour
        contour_hsv_values = hsv[contour_mask == 255]
        avg_hsv = np.mean(contour_hsv_values, axis=0)  # Compute the average HSV only inside the contour

        # Print details of the detected area
        print(f"Candidate {i+1}:")
        print(f"  - Coordinates: (x={x}, y={y}, width={w}, height={h})")
        print(f"  - Average HSV (inside contour) = {avg_hsv}")

        # Draw a bounding box around the detected region
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red rectangle

        # Add text annotation for coordinates
        text = f"({x}, {y})"
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

# Show the result
cv2.imshow("Detected Olive Green Areas", image)
cv2.imshow("Olive Green Mask", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

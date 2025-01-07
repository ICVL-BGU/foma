import cv2
import numpy as np

# Load the image
image = cv2.imread('img.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Create a binary mask where the stitching lines are white
_, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

# Invert the mask
mask_inv = cv2.bitwise_not(mask)

# Blur the edges of the mask to create a feather effect
mask_inv = cv2.GaussianBlur(mask_inv, (21, 21), 0)

# Blend the original image with the blurred mask
blended = cv2.bitwise_and(image, image, mask=mask_inv)

# Save the result
cv2.imwrite('img_fixed.jpg', blended)

import cv2

# Load the original image and the mask
image = cv2.imread('img.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
_, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

# Find the contours of the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Get the bounding box of the largest contour
x, y, w, h = cv2.boundingRect(contours[0])

# Create a mask for seamless cloning
center = (x + w // 2, y + h // 2)
seamless_clone = cv2.seamlessClone(image, image, mask, center, cv2.NORMAL_CLONE)

# Save the result
cv2.imwrite('clone.jpg', seamless_clone)

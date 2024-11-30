import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the images
image_with_vignette = cv2.imread(r'G:\My Drive\Alex\School\MSc\Thesis\FOV\Code\ImageStitching\final_images\3.jpg', cv2.IMREAD_COLOR)
white_screen_image = cv2.imread(r'G:\My Drive\Alex\School\MSc\Thesis\FOV\Code\ImageStitching\white_images\3.jpg', cv2.IMREAD_COLOR)

# Convert to grayscale
gray_vignette = cv2.cvtColor(image_with_vignette, cv2.COLOR_BGR2GRAY)
gray_white_screen = cv2.cvtColor(white_screen_image, cv2.COLOR_BGR2GRAY)

# Normalize the white screen image to create the vignette mask
vignette_mask = gray_white_screen.astype(np.float32) / gray_white_screen.max()

# Ensure no division by zero
vignette_mask[vignette_mask == 0] = 1

# Apply vignette correction by dividing the original image by the vignette mask
corrected_image = cv2.divide(gray_vignette.astype(np.float32), vignette_mask)

# Normalize the corrected image to range [0, 255] and convert to uint8
corrected_image = cv2.normalize(corrected_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

# Display the images using matplotlib
fig, ax = plt.subplots(1, 3, figsize=(18, 6))

ax[0].imshow(gray_vignette, cmap='gray')
ax[0].set_title('Original Image with Vignette')
ax[0].axis('off')

ax[1].imshow(vignette_mask, cmap='gray')
ax[1].set_title('Vignette Mask')
ax[1].axis('off')

ax[2].imshow(corrected_image, cmap='gray')
ax[2].set_title('Corrected Image')
ax[2].axis('off')

plt.show()
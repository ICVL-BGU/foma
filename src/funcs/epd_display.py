import numpy as np
import requests
from PIL import Image

# ESP32 server URL

url = 'http://{}/'
chunk_size = 1024  # Chunk size for sending the image data

# GDEPD color codes
color_map = {
    (0, 0, 0): 0x00,  # Black
    (255, 255, 255): 0x01,  # White
    (0, 255, 0): 0x06,  # Green
    (0, 0, 255): 0x05,  # Blue
    (255, 0, 0): 0x03,  # Red
    (255, 255, 0): 0x02,  # Yellow
    (127, 127, 127): 0x07  # Clean
}

def color_to_gdepd_code(color):
    distances = {k: np.linalg.norm(np.array(k) - np.array(color)) for k in color_map.keys()}
    closest_color = min(distances, key=distances.get)
    return color_map[closest_color]

def find_closest_palette_color(pixel, palette):
    if len(pixel) == 4:
        pixel = pixel[:3]
    palette = np.array(palette)
    distances = np.sqrt(np.sum((palette - pixel) ** 2, axis=1))
    return palette[np.argmin(distances)]

def floyd_steinberg_dithering(image_np, palette):
    h, w, _ = image_np.shape

    for y in range(h):
        for x in range(w):
            old_pixel = image_np[y, x].copy()
            new_pixel = find_closest_palette_color(old_pixel, palette)
            image_np[y, x] = new_pixel
            quant_error = old_pixel - new_pixel

            if x + 1 < w:
                image_np[y, x + 1] += quant_error * 7 / 16
            if x - 1 >= 0 and y + 1 < h:
                image_np[y + 1, x - 1] += quant_error * 3 / 16
            if y + 1 < h:
                image_np[y + 1, x] += quant_error * 5 / 16
            if x + 1 < w and y + 1 < h:
                image_np[y + 1, x + 1] += quant_error * 1 / 16

    return np.clip(image_np, 0, 255).astype(np.uint8)

def process_image(image_path, width=800, height=480):
    # Open the image using PIL and convert to RGB
    image = Image.open(image_path).convert('RGB')

    # Calculate aspect ratio and new dimensions
    w, h = image.size
    aspect_ratio = w / h

    if (width / height) > aspect_ratio:
        new_height = height
        new_width = int(height * aspect_ratio)
    else:
        new_width = width
        new_height = int(width / aspect_ratio)

    # Resize the image using PIL
    resized_image = image.resize((new_width, new_height), Image.Resampling.LANCZOS)

    # Convert the resized image to a NumPy array
    resized_image_np = np.array(resized_image, dtype=float)

    # Dither the image
    palette = [(0, 0, 0), (255, 255, 255), (0, 255, 0), (0, 0, 255), (255, 0, 0), (255, 255, 0)]
    dithered_image_np = floyd_steinberg_dithering(resized_image_np, palette)

    # Create a white canvas and place the dithered image on it
    canvas = np.ones((height, width, 3), dtype=np.uint8) * 255
    x_offset = (width - new_width) // 2
    y_offset = (height - new_height) // 2
    canvas[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = dithered_image_np

    # Convert the canvas to a color-coded image
    color_coded_image = np.zeros((height, width), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            color = tuple(canvas[y, x])
            gdepd_code = color_to_gdepd_code(color)  # Assumes this function is defined
            color_coded_image[y, x] = gdepd_code

    return color_coded_image

# Note: The function color_to_gdepd_code(color) must be defined elsewhere in your code.



def encode_image(image_array):
    encoded_image = []
    current_color = image_array[0][0]
    count = 0

    for y in range(image_array.shape[0]):
        for x in range(image_array.shape[1]):
            if image_array[y][x] == current_color:
                count += 1
                if count == 31:
                    encoded_image.append((count << 3) | current_color)
                    count = 0
            else:
                if count > 0:
                    encoded_image.append((count << 3) | current_color)
                current_color = image_array[y][x]
                count = 1

    if count > 0:
        encoded_image.append((count << 3) | current_color)

    return encoded_image

def send_image(encoded_image, url):
    image_bytes = bytes(encoded_image)
    print(f"Encoded image size: {len(image_bytes)} bytes")

    while True:
        size_response = requests.post(url + 'setSize', data=str(len(image_bytes)))
        print(f"Set length response: {size_response.text}")
        if size_response.text == 'Image size received':
            break

    while True:
        chunk_response = requests.post(url + 'setChunk', data=str(chunk_size))
        print(f"Set chunk response: {chunk_response.text}")
        if chunk_response.text == 'Chunk size received':
            break

    i = 0
    while i < len(image_bytes):
        chunk = image_bytes[i:i+chunk_size]
        response = requests.post(url + 'upload', data=chunk, timeout=10)
        print(f"Chunk {i//chunk_size + 1}: {response.text}")
        if response.text == 'Chunk received':
            i += chunk_size

def touch(addr):
    ans = requests.get(url.format(addr))
    if ans and "EPD" in ans.text:
        return True
    return False

def display_image(img, addr):
    url.format(addr)
    processed_image = process_image(img)
    encoded_image = encode_image(processed_image)
    send_image(encoded_image, url.format(addr))

import numpy as np

def image_to_world(x_img, y_img, image_width=1024, image_height=768, 
                    focal_length=1.16, camera_pos=(2.5, 2.5, 2.15), 
                    ground_height=0.95, sensor_width=5.5, sensor_height=4):
    """
    Maps an (x, y) coordinate in the image to a real-world (x, y, z) coordinate,
    setting the camera center as the origin.
    
    :param x_img: X coordinate in the image (pixels)
    :param y_img: Y coordinate in the image (pixels)
    :param image_width: Width of the image (pixels)
    :param image_height: Height of the image (pixels)
    :param focal_length: Focal length of the camera (mm)
    :param camera_pos: (x, y, z) position of the camera in world coordinates
    :param ground_height: Known height of the detected point in world coordinates
    :param sensor_width: Width of the camera sensor (mm)
    :param sensor_height: Height of the camera sensor (mm)
    :return: (x_world, y_world, ground_height) real-world coordinates
    """
    # Convert focal length from mm to meters
    focal_length_m = focal_length / 1000  # Convert mm to meters

    sensor_width = sensor_width / 1000
    sensor_height = sensor_height / 1000
    
    # Extract camera parameters
    cam_x, cam_y, cam_z = camera_pos
    
    # Define image center as origin
    origin_x = image_width / 2
    origin_y = image_height / 2
    
    # Compute pixel offsets from the image center
    px = (x_img - origin_x) * (sensor_width / image_width)  # Convert pixels to sensor units (mm)
    py = (y_img - origin_y) * (sensor_height / image_height)  # Convert pixels to sensor units (mm)
    
    # Scale factor to map image coordinates to world coordinates
    scale = (cam_z - ground_height) / focal_length_m  # Use focal length in meters
    
    # Convert sensor distances to real-world distances
    x_world = cam_x + px * scale
    y_world = cam_y + py * scale
    
    return x_world, y_world, ground_height

# 1.18 = cam_x + (x_img - origin_x) * (sensor_width / image_width) * (cam_z - ground_height) / focal_length_m 
# 1.18 = 2.5 + (276 - 512) * (sensor_width / 1024) * (2.15 - 0.95) / 1.16 / 1000 
# 1.32 = 236 * 1.2 / 1.16 / 1000 / 1024 * sensor_width

# 0.86 = cam_y + (y_img - origin_y) * (sensor_height / image_height) * (cam_z - ground_height) / focal_length_m 
# 0.86 = 2.5 + (80 - 384) * (sensor_height / 768) * (2.15 - 0.95) / 1.16 / 1000  
# 1.64 = 304 * 1.2 / 1.16 / 1000 / 768 * sensor_height

# Example usage:
image_x, image_y = 276, 80  # Example pixel (center of the image)
world_coords = image_to_world(image_x, image_y)
print("World coordinates:", world_coords)

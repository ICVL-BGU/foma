#!/usr/bin/env python3

import ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from fov.srv import Coordinate, CoordinateRequest, CoordinateResponse
import cv2
import pickle
import os
import numpy as np
import time
import largestinteriorrectangle as lir

class ImageStitchingNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_stitcher', anonymous=False)

        # Define the camera topics based on a fixed naming convention
        self.sub_topic_format = "cameras/ceiling_camera_{}/image"
    
        # Create a dictionary to store the latest images from each camera
        self.images = {i:None for i in range(1, 6) }
        # Subscribe to each camera's image topic
        self.bridge = CvBridge()
        for i in range(1, 6):
            rospy.Subscriber(self.sub_topic_format.format(i), Image, self.image_callback, callback_args=i)

        # Load calibration parameters
        self.parameters_set = False
        self.coordinates = {}
        self.masks = [None]*5
 
        # Publisher for the stitched image
        self.stitched_image_pub = rospy.Publisher("image_stitcher/image", Image, queue_size=10)
        rospy.Service('image_stitcher/coordinates', Coordinate, self.set_coordinates)

        self.iteration = 0
        self.avg = 0

        rospy.on_shutdown(self.__on_shutdown)

    def set_coordinates(self, data: CoordinateRequest):
        # rospy.loginfo("Got here")
        self.coordinates[data.idx] = {'x.min': data.xmin, 'y.min': data.ymin, 'x.max': data.xmax, 'y.max': data.ymax}
        # rospy.loginfo(f"Coordinates for camera {data.idx} received.")
        # rospy.loginfo(f"Coordinates: {self.coordinates[data.idx]}")
        flattened_mask = np.frombuffer(data.mask.data, dtype=np.uint8)
        self.masks[data.idx - 1] = np.array(flattened_mask, dtype = np.uint8).reshape(data.mask.layout.dim[0].size, data.mask.layout.dim[1].size)
        if len(self.coordinates) == 5:
            self.calculate_parameters()
        return CoordinateResponse()

    def image_callback(self, msg, index):
        try:
            self.images[index] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(f"Error converting image from topic {self.sub_topic_format.format(index)}: {e}")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Check if all cameras have provided images
            if self.parameters_set:
                try:
                    pass
                    # Stitch images
                    # t0 = time.time()
                    stitched_img = self.stitch_images()
                    # rospy.loginfo(f"Stitching took {time.time()-t0}")
                    if stitched_img is not None:
                        # Publish the stitched image
                        # x, y, w, h = self.lir
                        self.image = stitched_img #[y:y+h, x:x+w]
                        
                        stitched_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
                        self.stitched_image_pub.publish(stitched_msg)
                        # rospy.loginfo("Stitched image published.")
                        # output_dir = "/home/icvl/FOMA/src/fov/output"
                        # if not os.path.exists(output_dir):
                        #     os.makedirs(output_dir)
                        
                        # output_path = os.path.join(output_dir, "stitched_image.jpg")
                        # cv2.imwrite(output_path, self.image)
                except Exception as e:
                    rospy.logerr(f"Error stitching images: {e}")

            rate.sleep()

    def calculate_parameters(self):
        self.H_translates = {}

        # Calculate the overall translation to align all images
        min_x = min(self.coordinates[i]['x.min'] for i in range(1, 6))
        min_y = min(self.coordinates[i]['y.min'] for i in range(1, 6))
        max_x = max(self.coordinates[i]['x.max'] for i in range(1, 6))
        max_y = max(self.coordinates[i]['y.max'] for i in range(1, 6))

        # rospy.loginfo(f"Min x: {min_x}, Min y: {min_y}, Max x: {max_x}, Max y: {max_y}")

        self.output_width = max_x - min_x
        self.output_height = max_y - min_y

        # rospy.loginfo(f"Output width: {self.output_width}, Output height: {self.output_height}")

        # Adjust the translation matrices to account for the overall translation
        overall_translation = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]], dtype=np.float32)
        # rospy.loginfo(f"Overall translation: {overall_translation}")
        output_dir = "/home/icvl/FOMA/src/fov/output/masks"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        for i in range(1, 6):
            x_min, y_min = self.coordinates[i]['x.min'], self.coordinates[i]['y.min']
            t = [x_min, y_min]
            self.H_translates[i] = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)
            self.H_translates[i] = overall_translation @ self.H_translates[i]
            self.masks[i-1] = cv2.warpPerspective(self.masks[i-1], self.H_translates[i], (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

            # Write each mask to disk
            mask_path = os.path.join(output_dir, f"mask_{i}.png")
            cv2.imwrite(mask_path, self.masks[i-1])

        # self.blender = cv2.detail_FeatherBlender()
        # self.blender.setSharpness(1.0 / 50.0)  # Adjust sharpness for better blending
        # self.blender.prepare((0, 0, self.output_height, self.output_width))

        self.calc_lir()
        x, y, w, h = self.lir
        for i in range(5):
            self.masks[i] = self.masks[i][y:y+h, x:x+w]
        self.parameters_set = True

    def calc_lir(self):
        combined_mask = np.zeros((self.output_height, self.output_width), dtype=np.uint8)
        for mask in self.masks:
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        mask_bool = combined_mask.astype(bool)  # Convert mask to boolean array
        self.lir = lir.lir(mask_bool)
        # rospy.loginfo(f"LIR: {self.lir}")

    def feather_blending(self, images):
        blender = cv2.detail_FeatherBlender(1.0 / 50.0)        
        _, _, w, h = self.lir
        blender.prepare((0,0, w, h))
        for img, mask in zip(images, self.masks):
            blender.feed(cv2.UMat(img.astype(np.int16)), cv2.UMat(mask), (0, 0))
        result, _ = blender.blend(None, None)
        return result.get() if isinstance(result, cv2.UMat) else result

    def stitch_images(self, num_bands=5, sigma=1):
        x, y, w, h = self.lir
        translated_images = []
        for i in range(1, 6):
            translated_image = cv2.warpPerspective(self.images[i], self.H_translates[i], (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
            translated_image = translated_image[y:y+h, x:x+w]
            translated_images.append(translated_image)

        # t0=time.time()
        # stitched_img = np.zeros((h, w, 3), dtype=np.float32)
        # mask_sum = np.zeros((h, w), dtype=np.float32)

        # for img, mask in zip(translated_images, self.masks):
        #     mask_float = mask.astype(np.float32) / 255.0
        #     for c in range(3):  # For each color channel
        #         stitched_img[:, :, c] += img[:, :, c] * mask_float
        #     mask_sum += mask_float

        # # Avoid division by zero
        # mask_sum[mask_sum == 0] = 1
        # for c in range(3):
        #     stitched_img[:, :, c] /= mask_sum

        # stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)
        result = self.feather_blending(translated_images)
        # # t1 = time.time()
        # # rospy.loginfo(f"Blending: {t1-t0}")
        stitched_img = cv2.normalize(result, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # stitched_img = cv2.normalize(stitched_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)
        # self.iteration += 1
        # self.avg = self.avg*(self.iteration-1)/self.iteration+(t1-t0)/self.iteration
        return stitched_img #cv2.cvtColor(stitched_img, cv2.COLOR_BGR2RGB)

        # Save the images before translation
        # for i, img in self.images.items():
        #     before_translation_path = os.path.join(output_dir, f"image_{i}_before_translation.jpg")
        #     cv2.imwrite(before_translation_path, img)
            # rospy.loginfo(f"Image {i} before translation saved to {before_translation_path}")

        # Save the images after translation
        # for i, img in enumerate(translated_images, start=1):
        #     after_translation_path = os.path.join(output_dir, f"image_{i}_after_translation.jpg")
        #     cv2.imwrite(after_translation_path, img)
            # rospy.loginfo(f"Image {i} after translation saved to {after_translation_path}")

    def __on_shutdown(self):
        rospy.loginfo("Shutting down Image Stitching Node.")
        rospy.loginfo(f"Average time for blending: {self.avg}")


if __name__ == "__main__":
    node = ImageStitchingNode()
    node.run()

#!/usr/bin/env python3

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
        self.changed = {i:False for i in range(1, 6)}
        # Subscribe to each camera's image topic
        self.bridge = CvBridge()
        for i in range(1, 6):
            rospy.Subscriber(self.sub_topic_format.format(i), Image, self.image_callback, callback_args=i)

        # Load calibration parameters
        param_dir = r'/home/icvl/fov_ws/src/fov/src/etc'

        self.calib_params = pickle.load(open(os.path.join(param_dir, "calib_params.pkl"), "rb"))
        self.homographies = pickle.load(open(os.path.join(param_dir, "homographies.pkl"), "rb"))
        self.lir = pickle.load(open(os.path.join(param_dir, "lir.pkl"), "rb"))

        self.base_idx = 3
        self.parameters_set = False
        self.coordinates = {}
        self.masks = [None]*5
 
        # Publisher for the stitched image
        self.stitched_image_pub = rospy.Publisher("image_stitcher/image", Image, queue_size=10)
        rospy.Service('image_stitcher/coordinates', Coordinate, self.set_coordinates)

        rospy.on_shutdown(self.__on_shutdown)

    def set_coordinates(self, data: CoordinateRequest):
        # rospy.loginfo("Got here")
        self.coordinates[data.idx] = {'x.min': data.xmin, 'y.min': data.ymin, 'x.max': data.xmax, 'y.max': data.ymax}
        flattened_mask = np.frombuffer(data.mask.data, dtype=np.uint8)
        self.masks[data.idx - 1] = np.array(flattened_mask, dtype = np.uint8).reshape(data.mask.layout.dim[0].size, data.mask.layout.dim[1].size)
        if len(self.coordinates) == 5:
            self.calculate_parameters()
        return CoordinateResponse()

    def image_callback(self, msg, index):
        try:
            # Convert the ROS Image message to OpenCV format

            # t0=time.time()
            self.images[index] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # rospy.loginfo(f"Converting: {time.time()-t0}")
            self.changed[index] = True

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
                    stitched_img = self.stitch_images()
                    if stitched_img is not None:
                        # Publish the stitched image
                        x, y, w, h = self.lir
                        self.image = stitched_img[y:y+h, x:x+w]
                        stitched_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
                        self.stitched_image_pub.publish(stitched_msg)
                            
                        # output_dir = "/home/icvl/fov_ws/src/fov/output"
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

        self.output_width = max_x - min_x
        self.output_height = max_y - min_y

        # Adjust the translation matrices to account for the overall translation
        overall_translation = np.array([[1, 0, -min_x], [0, 1, -min_y], [0, 0, 1]], dtype=np.float32)
        for i in range(1, 6):
            x_min, y_min = self.coordinates[i]['x.min'], self.coordinates[i]['y.min']
            x_max, y_max = self.coordinates[i]['x.max'], self.coordinates[i]['y.max']
            t = [x_min, y_min]
            self.H_translates[i] = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)
            self.H_translates[i] = overall_translation @ self.H_translates[i]
            self.masks[i-1] = cv2.warpPerspective(self.masks[i-1], self.H_translates[i], (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        self.calc_lir()

        self.parameters_set = True

    def calc_lir(self):
        stitched_img = self.stitch_images()
        
        mask = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)

        # Fill the contour to ensure no internal black pixels
        cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

        mask_bool = mask.astype(bool)  # Convert mask to boolean array
        self.lir = lir.lir(mask_bool, largest_contour[:, 0, :])

    def feather_blending(self, images, masks, output_shape):
        blender = cv2.detail_FeatherBlender()
        blender.setSharpness(1.0 / 50.0)  # Adjust sharpness for better blending
        blender.prepare((0, 0, output_shape[1], output_shape[0]))
        for img, mask in zip(images, masks):
            blender.feed(cv2.UMat(img.astype(np.int16)), cv2.UMat(mask), (0, 0))
        result, result_mask = blender.blend(None, None)
        return result.get() if isinstance(result, cv2.UMat) else result

    def stitch_images(self, num_bands=5, sigma=1):
        translated_images = []
        for i in range(1, 6):
            translated_image = cv2.warpPerspective(self.images[i], self.H_translates[i], (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
            translated_images.append(translated_image)

        t0=time.time()
        result = self.feather_blending(translated_images, self.masks, (self.output_height, self.output_width))
        rospy.loginfo(f"Blending: {time.time()-t0}")
        stitched_img = cv2.normalize(result, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)
        return cv2.cvtColor(stitched_img, cv2.COLOR_BGR2RGB)

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


if __name__ == "__main__":
    node = ImageStitchingNode()
    node.run()

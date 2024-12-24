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

class ImageStitchingNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_stitching_node', anonymous=False)

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
 
        # Publisher for the stitched image
        self.stitched_image_pub = rospy.Publisher("image_stitcher/image", Image, queue_size=10)
        rospy.Service('image_stitcher/coordinates', Coordinate, self.set_coordinates)

        rospy.on_shutdown(self.__on_shutdown)

    def set_coordinates(self, data: CoordinateRequest):
        self.coordinates[data.idx] = {'x.min': data.xmin, 'y.min': data.ymin, 'x.max': data.xmax, 'y.max': data.ymax}
        if len(self.coordinates) == 5:
            self.calculate_parameters()

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
            if any(self.changed.values()) and all(img is not None for img in self.images.values()):
                try:
                    # Stitch images
                    self.stitch_images()
                    rospy.loginfo(f"Shape:{self.image.shape}")
                    # self.image = self.images[1]
                    if self.image is not None:
                        # Publish the stitched image
                        stitched_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
                        self.stitched_image_pub.publish(stitched_msg)
                except Exception as e:
                    rospy.logerr(f"Error stitching images: {e}")

            rate.sleep()

    def calculate_parameters(self):
        # h, w = self.images[self.base_idx].shape[:2]

        # corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2)
        # all_corners = []
        # all_corners.extend(corners)
        # for i in range(1,6):
        #     if i == self.base_idx:
        #         continue
        #     if (self.base_idx, i) in self.homographies:
        #         H = self.homographies[(self.base_idx, i)]
        #         warped_corners = cv2.perspectiveTransform(corners, H)
        #         all_corners.extend(warped_corners)
        all_corners = []
        for i in range(1,6):
            all_corners.extend([self.coordinates[i]['x.min'], self.coordinates[i]['y.min'], self.coordinates[i]['x.max'], self.coordinates[i]['y.max']])
        all_corners = np.array(all_corners).reshape(-1, 2)

        [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel())
        [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel())
        t = [-xmin, -ymin]
        self.H_translate = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)

        self.output_width = xmax - xmin
        self.output_height = ymax - ymin

        # self.masks = []
        # self.final_homographies={}
        # for i in range(1,6):
        #     if i == self.base_idx:
        #         continue
            # self.final_homographies[i] = self.H_translate @ self.homographies[(self.base_idx, i)]
            # warped_image = cv2.warpPerspective(self.images[i], H, (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

            # Create a mask based on the ROI
            # roi = self.calib_params[i]['roi']
            # x, y, w, h = roi
            # mask = np.zeros((self.images[i].shape[0], self.images[i].shape[1]), dtype=np.uint8)
            # mask[y:y+h, x:x+w] = 255

            # Warp the mask using the same homography
            # warped_mask = cv2.warpPerspective(mask, self.final_homographies[i], (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

            # Apply the mask to the warped image

            # self.masks.append(warped_mask)

        self.parameters_set = True

    def feather_blending(self, images, masks, output_shape):
        blender = cv2.detail_FeatherBlender()
        blender.setSharpness(1.0 / 50.0)  # Adjust sharpness for better blending
        blender.prepare((0, 0, output_shape[1], output_shape[0]))
        for img, mask in zip(images, masks):
            blender.feed(cv2.UMat(img.astype(np.int16)), cv2.UMat(mask), (0, 0))
        result, result_mask = blender.blend(None, None)
        return result.get() if isinstance(result, cv2.UMat) else result

    def stitch_images(self, num_bands=5, sigma=1):
        # Step 1: Undistort Images
        # for i in range(1,6):
        #     K, D, mtx = self.calib_params[i]['K'], self.calib_params[i]['D'], self.calib_params[i]['mtx']
        #     self.images[i] = cv2.undistort(self.images[i], K, D, None, mtx)

        # if not self.parameters_set:
        #     self.calculate_parameters()

        # warped_images = []
        # # masks = []
        # for i in range(1,6):
        #     if i == self.base_idx:
        #         continue
        #     # H = self.H_translate @ self.homographies[(self.base_idx, i)]
        #     # warped_image = cv2.warpPerspective(self.images[i], self.final_homographies[i], (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        #     # Create a mask based on the ROI
        #     roi = self.calib_params[i]['roi']
        #     x, y, w, h = roi
        #     # mask = np.zeros((self.images[i].shape[0], self.images[i].shape[1]), dtype=np.uint8)
        #     # mask[y:y+h, x:x+w] = 255

        #     # Warp the mask using the same homography
        #     # warped_mask = cv2.warpPerspective(mask, H, (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        #     # Apply the mask to the warped image
        #     # masked_warped_image = cv2.bitwise_and(warped_image, warped_image, mask=self.masks[i-1] if i<self.base_idx else self.masks[i-2])

        #     # masks.append(warped_mask)
        #     warped_images.append(masked_warped_image)

        # Step 3: Blend images with feather blending
        translated_images = []
        for i in range(1,6):
            translated_image = self.H_translate @ self.images[i]
            translated_images.append(translated_image)

        t0=time.time()
        result = self.feather_blending(translated_images, self.masks, (self.output_height, self.output_width))
        rospy.loginfo(f"Blending: {time.time()-t0}")
        stitched_img = cv2.normalize(result, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        stitched_img = np.clip(stitched_img, 0, 255).astype(np.uint8)
        stitched_image_rgb = cv2.cvtColor(stitched_img, cv2.COLOR_BGR2RGB)

        x, y, w, h = self.lir
        self.image = stitched_image_rgb[y:y+h, x:x+w]

    def __on_shutdown(self):
        rospy.loginfo("Shutting down Image Stitching Node.")


if __name__ == "__main__":
    node = ImageStitchingNode()
    node.run()

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pickle
import os
import numpy as np

class ImageStitchingNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_stitching_node', anonymous=False)

        # Define the camera topics based on a fixed naming convention
        self.sub_topic_format = "cameras/ceiling_camera_{}/image"
    
        self.image = None
        self.base_idx = 3
    
        param_dir = r'/home/icvl/fov_ws/src/fov/src/etc'
        self.calib_params = pickle.load(open(os.path.join(param_dir, "calib_params.pkl"), "rb"))
        self.homographies = pickle.load(open(os.path.join(param_dir, "homographies.pkl"), "rb"))
        self.lir = pickle.load(open(os.path.join(param_dir, "lir.pkl"), "rb"))
        # self.width, self.height = None, None
        # Create a dictionary to store the latest images from each camera
        self.raw_images = {i:None for i in range(1, 6)}
        self.images = {i:None for i in range(1, 6)}
        self.received = {i:False for i in range(1, 6)}
        self.processed = {i:False for i in range(1, 6)}
        self.parameters_set = False
        # Subscribe to each camera's image topic
        self.bridge = CvBridge()
        for i in range(1, 6):
            rospy.Subscriber(self.sub_topic_format.format(i), Image, self.image_callback, callback_args=i)

        # Publisher for the stitched image
        self.stitched_image_pub = rospy.Publisher("image_stitcher/image", Image, queue_size=10)

        rospy.on_shutdown(self.__on_shutdown)

    def image_callback(self, msg, index):
        try:
            # rospy.loginfo("Received image")
            # Convert the ROS Image message to OpenCV format
            self.raw_images[index] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.raw_images[index] is not None:
                self.received[index] = True
                if self.parameters_set:
                    self.process_image(index)
                    self.processed[index] = True
                    self.received[index] = False
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image from topic {self.sub_topic_format.format(index)}: {e}")

    def run(self):
        rate = rospy.Rate(12)  # 10 Hz
        while not rospy.is_shutdown():
            # Check if all cameras have provided images
            if not self.parameters_set and all(self.received.values()):
                self.__calculate_parameters()
                rospy.loginfo("Calculated parameters.")
            if any(self.processed.values()):
                try:
                    # Stitch images
                    self.stitch_images()
                    if self.image is not None:
                        # Publish the stitched image
                        stitched_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
                        self.stitched_image_pub.publish(stitched_msg)
                        rospy.loginfo("Sending stitched image.")
                except Exception as e:
                    rospy.logerr(f"Error stitching images: {e}")

            rate.sleep()

    def __calculate_parameters(self):
        h, w = self.raw_images[self.base_idx].shape[:2]
        corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2)
        all_corners = []
        all_corners.extend(corners)
        for i in range(1,6):
            if i == self.base_idx:
                continue
            if (self.base_idx, i) in self.homographies:
                warped_corners = cv2.perspectiveTransform(corners, self.homographies[(self.base_idx, i)])
                all_corners.extend(warped_corners)

        all_corners = np.array(all_corners)
        [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel())
        [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel())
        t = [-xmin, -ymin]

        # Translate matrix and image dimensions
        self.translate = np.array([[1, 0, t[0]], [0, 1, t[1]], [0, 0, 1]], dtype=np.float32)
        self.width = xmax - xmin
        self.height = ymax - ymin
        self.final_homographies = {i:None for i in range(1, 6)}

        self.masks = {i:None for i in range(1, 6)}
        # Calculate masks
        for i in range(1,6):
            if i == self.base_idx:
                continue
            self.final_homographies[i] = self.translate @ self.homographies[(self.base_idx, i)]
            # Create a mask based on the ROI
            roi = self.calib_params[i]['roi']
            x, y, w, h = roi
            mask = np.zeros((self.raw_images[i].shape[0], self.raw_images[i].shape[1]), dtype=np.uint8)
            mask[y:y+h, x:x+w] = 255
            # Warp the mask using the same homography
            self.masks[i] = cv2.warpPerspective(mask, self.final_homographies[i], (self.width, self.height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        self.blender = cv2.detail_FeatherBlender()
        self.blender.setSharpness(1.0 / 50.0)  # Adjust sharpness for better blending
        self.blender.prepare((0, 0, self.height, self.width))
        self.parameters_set = True

    def process_image(self, index):
        K, D, mtx = self.calib_params[index]['K'], self.calib_params[index]['D'], self.calib_params[index]['mtx']
        undistorted_image = cv2.undistort(self.raw_images[index], K, D, None, mtx)
        if index == self.base_idx:
            self.images[index] = undistorted_image
        else:
            warped_image = cv2.warpPerspective(undistorted_image, self.final_homographies[index], (self.width, self.height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
            masked_warped_image = cv2.bitwise_and(warped_image, warped_image, mask=self.masks[index])
            self.images[index] = masked_warped_image
        rospy.loginfo(f"Processed Image {index}")

    # def feather_blending(self, images, masks, output_shape):
    #     blender = cv2.detail_FeatherBlender()
    #     blender.setSharpness(1.0 / 50.0)  # Adjust sharpness for better blending
    #     blender.prepare((0, 0, output_shape[1], output_shape[0]))
    #     for img, mask in zip(images, masks):
    #         blender.feed(cv2.UMat(img.astype(np.int16)), cv2.UMat(mask), (0, 0))
    #     result, result_mask = blender.blend(None, None)
    #     return result.get() if isinstance(result, cv2.UMat) else result

    def stitch_images(self, num_bands=5, sigma=1):
        # rospy.loginfo("Stitching Images")
        if not all(img is not None for img in self.raw_images.values()):
            return
        for i in range(1, 6):
            if self.processed[i]:
                self.blender.feed(cv2.UMat(self.images[i].astype(np.int16)), cv2.UMat(self.masks[i]), (0, 0))
                self.processed[i] = False

        result, _ = self.blender.blend(None, None)
        result = result.get() if isinstance(result, cv2.UMat) else result
        rospy.logwarn(f"Blending returned: {result}")
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

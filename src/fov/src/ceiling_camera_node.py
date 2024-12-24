#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
from fov.srv import Coordinate, CoordinateRequest, CoordinateResponse

class CeilingCameraNode(AbstractNode):
    def __init__(self):
        # Initialize the node with a name
        super().__init__(f'ceiling_camera_node_{camera_id}', f'Ceiling Camera Node {camera_id}')

        # Retrieve camera-specific parameters
        self.camera_id = rospy.get_param("~camera_id", 0)
        self.camera_ip = rospy.get_param("~camera_ip", "rtsp://admin:icvl2023@1.1.2.101:554")

        # Initialize the camera
        self.camera = cv2.VideoCapture(self.camera_ip)

        # Set up the image publisher
        self.image_pub = rospy.Publisher(f'ceiling_camera_{self.camera_id}/image', Image, queue_size=10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Load calibration parameters
        self.base_idx = 3
        self.resize_factor = 0.5
        self.calib_params = pickle.load(open(os.path.join(param_dir, "calib_params.pkl"), "rb"))[self.camera_id]

        # Load homography and calculate the scaling matrix
        scaling_mtx = np.array([[self.resize_factor, 0, 0], [0, self.resize_factor, 0], [0, 0, 1]])
        if self.camera_id == self.base_idx:
            self.homography = scaling_mtx
        else:            
            homography = pickle.load(open(os.path.join(param_dir, "homographies.pkl"), "rb"))[(self.base_idx, i)]
            self.homography = scaling_mtx @ homography

        self.coordinate_set = rospy.ServiceProxy('image_stitcher/coordinates', Light)

        # Shutdown behavior
        rospy.on_shutdown(self.__on_shutdown)

    def calc_params(self):
        # Calculate the output image size and save coordinates for stitching node
        _, img = self.camera.read()
        h, w = img.shape[:2]
        corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype='float32').reshape(-1, 1, 2)
        corners = cv2.perspectiveTransform(corners, self.homography)
        self.coordinates = {}
        self.coordinates['xmin'], self.coordinates['ymin'] = np.int32(all_corners.min(axis=0).ravel())
        self.coordinates['xmax'], self.coordinates['ymax'] = np.int32(all_corners.max(axis=0).ravel())
        
        self.output_width = self.coordinates['xmax'] - self.coordinates['xmin']
        self.output_height = self.coordinates['ymax'] - self.coordinates['ymin']

        x, y, w, h = self.calib_params[i]['roi']
        mask = np.zeros(img.shape, dtype=np.uint8)
        mask[y:y+h, x:x+w] = 255

        self.mask = cv2.warpPerspective(mask, self.homography, (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        rospy.wait_for_service('image_stitcher/coordinates')
        self.coordinate_set(CoordinateRequest(idx=self.camera_id, xmin=self.coordinates['xmin'], ymin=self.coordinates['ymin'], xmax=self.coordinates['xmax'], ymax=self.coordinates['ymax']))

    def run(self):
        while not rospy.is_shutdown():
            try:
                ret, img = self.camera.read()
                if ret:
                    # Publish the image
                    img = cv2.undistort(img, self.calib_params['K'], self.calib_params['D'], None, self.calib_params['mtx'])
                    warped_image = cv2.warpPerspective(img, self.homography, (self.output_width, self.output_height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
                    masked_warped_image = cv2.bitwise_and(warped_image, warped_image, mask=self.mask)
                    img_msg = self.bridge.cv2_to_imgmsg(warped_image, "bgr8")
                    self.image_pub.publish(img_msg)
                else:
                    rospy.logwarn(f"Camera {self.camera_id}: No image captured.")
            except CvBridgeError as e:
                rospy.logerr(f"Camera {self.camera_id}: CV Bridge Error - {e}")

            rospy.sleep(0.05)

    def get_coordinates(self):
        return self.coordinates

    def __on_shutdown(self):
        rospy.loginfo(f"Ceiling Camera {self.camera_id}: Releasing camera.")
        self.camera.release()


if __name__ == "__main__":
    # Retrieve the camera ID from parameters
    camera_id = rospy.get_param("~camera_id", 0)

    # Initialize the node with a unique name based on the camera ID
    rospy.init_node(f'ceiling_camera_node_{camera_id}', anonymous=False)

    rospy.loginfo(f"Ceiling Camera Node {camera_id}: Node started.")
    node = CeilingCameraNode()
    node.run()
    rospy.spin()

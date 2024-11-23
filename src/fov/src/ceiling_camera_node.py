#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError

class CeilingCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('ceiling_cameras', 'Ceiling cameras')
        self.stitched_image_pub = rospy.Publisher('ceiling_cameras/stitched_image', Image, queue_size=10)
        self.cameras = [None]*5
        for i in range(5):
            self.cameras[i] = cv2.VideoCapture('rtsp://admin:icvl2023@1.1.2.10{}:554'.format(i+1))
        
        self.img = None
        self.bridge = CvBridge()
        rospy.on_shutdown(self.__on_shutdown)

    def run(self):
        while not rospy.is_shutdown():
            if True: #self._system_on:
                try:
                    acc_ret = True
                    images = []
                    for cam in self.cameras:
                        ret, img = cam.read()
                        acc_ret &= ret
                        images.append(img)
                    # ret, img = self.cameras[0].read()
                    images.append(img)
                    if ret:
                        self.img = self.stitch_imgs(images)
                        img_msg = self.bridge.cv2_to_imgmsg(self.img)
                        # rospy.loginfo("publishing")
                        self.stitched_image_pub.publish(img_msg)

                except CvBridgeError as e:
                    print(e)

                rospy.sleep(0.05)

    def __on_shutdown(self):
        for i in range(5):
            rospy.logwarn("CeilingCameraNode: Releasing camera {}.".format(i+1))
            self.cameras[i].release()
        

    def stitch_imgs(self, images):
        # TODO
        return images[0]

    

if __name__ == "__main__":
    rospy.init_node('ceiling_camera_node')
    rospy.loginfo("Ceiling Camera Node: node created.")
    ceiling_cam_handler = CeilingCameraNode()
    ceiling_cam_handler.run()
    rospy.spin()
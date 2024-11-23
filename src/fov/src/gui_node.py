#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError

class GUINode(AbstractNode):
    def __init__(self):
        super().__init__('gui', 'GUI')
        self.image_sub = rospy.Subscriber('web_camera/image', Image, self.read_image)
        self.fish_dir_sub = rospy.Subscriber('fish_detection/direction', UInt16, self.update_direction)
        self.stitched_image_pub = rospy.Subscriber('ceiling_cameras/stitched_image', Image, self.update_direction)
        self.feed = rospy.ServiceProxy('fish_feeder/system_toggle', Trigger)
        # Other subscribers for other nodes...
        self.img = None
        self.bridge = CvBridge()

    def run(self):
        while not rospy.is_shutdown():
            if True:#self._system_on:
                if not self.img is None:
                    #rospy.logwarn(self.img)
                    cv2.imshow("img", self.img)
                    if cv2.waitKey(1) == ord('q'):
                        break
            rospy.sleep(0.005)
        #cap.release()
        cv2.destroyAllWindows()

    def read_image(self, img_msg: Image):
        #rospy.logwarn("reading img")
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
            print(e)


    def update_direction(self, direction_msg):
        # Update direction display on GUI
        pass

if __name__ == "__main__":
    rospy.init_node('gui_node')
    rospy.loginfo("GUI Node: node created.")
    gui = GUINode()
    rospy.loginfo("GUI Node: starting run.")
    gui.run()

    rospy.spin()
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from abstract_node import AbstractNode
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class FishDetectionNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_detection', 'Fish detection')
        self.image_sub = rospy.Subscriber('web_camera/image', Image, self.read_image)
        self.fish_dir_pub = rospy.Publisher('fish_detection/direction', UInt16, queue_size=10)  # Assuming direction is a string, adjust as necessary
        self.direction = None
        self.img = None
        self.bridge = CvBridge()

    def run(self):
        while not rospy.is_shutdown():
            
            if self._system_on:
                if self.img:
                    self.find_dir()

    def read_image(self, img_msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
            print(e)

    def find_dir(self):

        frame = self.img

        if frame is None: return True
            
        # Every color except white
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.higher_hsv)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        fgmask = self.fgbg.apply(result)
        
        _, th1 = cv2.threshold(result, 30, 255 , cv2.THRESH_BINARY)
        gray = 255 -  cv2.cvtColor(th1, cv2.COLOR_BGR2GRAY)

        # Find outer contours
        image, cnts, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        areaSUM, CCy , CCx , maxA , CxMax , CyMax ,maxC ,c = 0 , 0 ,0 ,0, 0 ,0,0,None

        for c in cnts:
            
            M = cv2.moments(c)
            if  M['m00'] > 500 :

                cx = int(M['m10']/M['m00']) # x c.m
                cy = int(M['m01']/M['m00']) # y c.m
                areaSUM  = areaSUM + M['m00'] # sum over Ai
                CCx = CCx  + cx* M['m00'] # sum over x c.m
                CCy = CCy + cy*M['m00']# sum over y c.m

                if maxA < M['m00']: maxA  ,CxMax , CyMax , self.cmax = M['m00'] , cx ,cy ,c

        if areaSUM > 0 :CCx , CCy  = CCx / areaSUM ,CCy / areaSUM
        Xc = tuple(np.array([CCx, CCy], dtype= int).reshape(1, -1)[0])
        Xcmax = tuple(np.array([CxMax, CyMax], dtype= int).reshape(1, -1)[0])
        #cv2.circle(frame, Xcmax, 10, (0, 0, 255), -1)
        
        extBot = 0 
        if  maxA >= 0.9 * areaSUM: #and areaSUM > 1200 and areaSUM<2000:
            
            if not not np.shape(c) and not not np.shape(self.cmax):
                # determine the most extreme points along the contour
                argmaxdist = np.argmax(np.matmul(np.array(np.power((self.cmax - Xc),2) ), [1,1]))
                self.extBot2 =  tuple(self.cmax[argmaxdist,0])
                cv2.circle(frame, self.extBot2, 10, (0, 255, 255), -1)
                cv2.arrowedLine(frame, self.extBot2, Xc, (0, 255, 0), 2, tipLength=0.2)
                self.setV(Xc, self.extBot2)

        cv2.drawContours(frame, cnts, -1, (255 ,0, 0), 2)
        cv2.imshow('arrow', frame)

        cv2.imwrite("~/PoseRecNew/pics/frame%d.jpg" % self.count, frame)   # save frame as JPEG file
        self.count +=1      

        key = cv2.waitKey(30)
        if key == 'q' or key == 27: return True

        return False


if __name__ == "__main__":
    rospy.init_node('fish_detection_node')
    fish_det = FishDetectionNode()
    fish_det.run()
    rospy.spin()
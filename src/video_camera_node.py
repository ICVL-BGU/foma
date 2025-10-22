#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from abstract_node import AbstractNode
import numpy as np
import cv2
from picamera2 import Picamera2
from etc.settings import *
from concurrent.futures import TimeoutError
import faulthandler, sys, signal, traceback

faulthandler.enable(sys.stderr, all_threads=True)

def on_sigill(signum, frame):
    traceback.print_stack(frame)
    sys.exit(1)

signal.signal(signal.SIGILL, on_sigill)

class VideoCameraNode(AbstractNode):
    def __init__(self):
        super().__init__('fish_camera', 'Fish camera')

        self.pub = rospy.Publisher('fish_camera/image', CompressedImage, queue_size=1)

        self.cam = Picamera2()

        # STILL configuration with a continuous lores stream for preview/publishing.
        # Keep your hi-res square still; drive the ROS topic from lores.
        cfg = self.cam.create_still_configuration(
            main={"size": (2464, 2464), "format": "BGR888"},     # full-res stills when needed
            lores={"size": (640, 640), "format": "BGR888"},      # continuous frames for streaming
            buffer_count=6
        )
        self.cam.configure(cfg)
        self.cam.start()
        try:
            self.cam.wait_for_ready()
        except Exception:
            rospy.sleep(0.2)

        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        self.msg = CompressedImage()
        self.msg.format = "jpeg"

        rospy.on_shutdown(self._on_shutdown)

    def run(self):
        rate = rospy.Rate(FOMA_CAMERA_FPS)
        while not rospy.is_shutdown():
            try:
                # Non-blocking read from the *lores* stream so the loop never deadlocks.
                job = self.cam.capture_array(name="lores", wait=False)
                frame = job.get_result(timeout=1.0)  # RGB888 (H,W,3)

                # Rotate 270° (k=3) to match your previous behavior.
                frame = np.rot90(frame, k=3)

                ok, buf = cv2.imencode(".jpg", frame, self._jpeg_params)
                if not ok:
                    self.logwarn("JPEG encode failed; skipping frame.")
                    rate.sleep()
                    continue

                self.msg.header.stamp = rospy.Time.now()
                self.msg.data = buf.tobytes()
                self.pub.publish(self.msg)

            except TimeoutError:
                self.logwarn_throttle(5.0, "Camera frame timeout (lores).")
            except Exception as e:
                self.logerr(f"Error capturing/publishing frame: {e}")

            rate.sleep()

    def _on_shutdown(self):
        self.loginfo("Shutting down camera")
        try:
            self.cam.stop()
        except Exception:
            pass
        try:
            self.cam.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node('video_camera_node')
    VideoCameraNode().run()

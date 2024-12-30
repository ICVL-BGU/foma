#!/usr/bin/env python3

# General imports
import sys
import rospy
import cv2

# PyQt5 imports
from PyQt5.QtCore import Qt, QTimer, QEvent
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QGridLayout,
    QFrame,
    QSizePolicy,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QDialog,
    )

# ROS imports
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge, CvBridgeError

# Custom ROS messages
from fov.srv import Light
     

        

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Rospy configs
        self.__init_subscriptions_and_services()
        self.bridge = CvBridge()

        self.setWindowTitle("Trial control")
        self.drag_start = self.pos()

        self.__init_widgets()
        self.__init_layouts()

        # main_layout = QGridLayout()
        # main_layout.addWidget(self.__TL_widget,0,0)
        # main_layout.addWidget(self.__TR_widget,0,1)
        # main_layout.addWidget(self.__BL_widget,1,0)
        # main_layout.addWidget(self.__BR_widget,1,1)
        # main_layout.setSpacing(0)
        # main_layout.setContentsMargins(0,0,0,0)
        # main_layout.setRowStretch(0, 3)  # Top row (camera sections)
        # main_layout.setRowStretch(1, 1)  # Bottom row (controls)
        # main_layout.setColumnStretch(0, 1)  # Left column
        # main_layout.setColumnStretch(1, 1)  # Right column


    def __init_layouts(self):
        # Top-Left (Fish Camera)
        self.__TL_layout = QVBoxLayout()
        self.__TL_layout.addWidget(self.__fish_image_label, alignment=Qt.AlignCenter)
        self.__TL_layout.addWidget(self.__fish_image)#, alignment=Qt.AlignCenter)
        
        self.__TL_widget = QFrame()
        self.__TL_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__TL_widget.setLineWidth(2)
        self.__TL_widget.setLayout(self.__TL_layout)

        # Top-Right (Room Camera)
        self.__TR_layout = QVBoxLayout()
        self.__TR_layout.addWidget(self.__room_image_label, alignment=Qt.AlignCenter)
        self.__TR_layout.addWidget(self.__room_image)#, alignment=Qt.AlignCenter)
        # self.__TR_layout.addStretch()

        self.__TR_widget = QFrame()
        self.__TR_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__TR_widget.setLineWidth(2)
        self.__TR_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.__TR_widget.setLayout(self.__TR_layout)

        # Bottom-Left (Settings)
        self.__BL_layout = QGridLayout()
        self.__BL_layout.addWidget(self.__lights_label, 0, 0, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__lights_slider, 1, 0, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__feed_label, 0, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__feed_button, 1, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__manual_control_label, 0, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__manual_control_button, 1, 2, alignment=Qt.AlignCenter)
        
        self.__BL_widget = QFrame()
        self.__BL_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__BL_widget.setLineWidth(2)
        self.__BL_widget.setLayout(self.__BL_layout)

        # Bottom-Right (Control Buttons)
        self.__BR_layout = QGridLayout()
        self.__BR_layout.addWidget(self.__start_button, 0, 0, alignment=Qt.AlignCenter)
        self.__BR_layout.addWidget(self.__stop_button, 0, 1, alignment=Qt.AlignCenter)
        self.__BR_layout.addWidget(self.__reset_button, 1, 0, alignment=Qt.AlignCenter)
        self.__BR_layout.addWidget(self.__close_button, 1, 1, alignment=Qt.AlignCenter)

        self.__BR_widget = QFrame()
        self.__BR_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__BR_widget.setLineWidth(2)
        self.__BR_widget.setLayout(self.__BR_layout)

        # Main Layout
        main_layout = QGridLayout()
        main_layout.addWidget(self.__TL_widget, 0, 0)
        main_layout.addWidget(self.__TR_widget, 0, 1)
        main_layout.addWidget(self.__BL_widget, 1, 0)
        main_layout.addWidget(self.__BR_widget, 1, 1)

        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setRowStretch(0, 5)
        main_layout.setRowStretch(1, 1)
        main_layout.setColumnStretch(0, 1)
        main_layout.setColumnStretch(1, 1)

        widget = QWidget()
        widget.setLayout(main_layout)
        self.setCentralWidget(widget)


        # widget = QWidget()
        # widget.setLayout(main_layout)
        # screen = QDesktopWidget().screenGeometry()
        # self.setFixedSize(screen.width(), screen.height())
        # self.setWindowFlags(Qt.CustomizeWindowHint)
        # self.showMaximized()
        # self.setCentralWidget(widget)

    def __init_widgets(self):
        # Start button init
        self.__start_button = QPushButton()
        self.__start_button.setText("Start")
        self.__start_button.setDisabled(False)
        self.__start_button.setMaximumHeight(50)
        self.__start_button.clicked.connect(lambda:self.__set_buttons_state((True,False,True,True)))

        # Stop button init
        self.__stop_button = QPushButton()
        self.__stop_button.setText("Stop")
        self.__stop_button.setDisabled(True)
        self.__stop_button.setMaximumHeight(50)
        self.__stop_button.clicked.connect(lambda:self.__set_buttons_state((True,True,False,False)))

        # Reset button init
        self.__reset_button = QPushButton()
        self.__reset_button.setText("Reset")
        self.__reset_button.setDisabled(True)
        self.__reset_button.setMaximumHeight(50)
        self.__reset_button.clicked.connect(lambda:self.__set_buttons_state((False,True,True,False)))

        # Close button init
        self.__close_button = QPushButton()
        self.__close_button.setText("Close")
        self.__close_button.setDisabled(False)
        self.__close_button.setMaximumHeight(50)
        self.__close_button.clicked.connect(self.__shutdown)

        # Feed button init
        self.__feed_button = QPushButton()
        self.__feed_button.setText("Feed")
        # self.__feed_button.setDisabled(True)
        self.__feed_button.clicked.connect(lambda:self.feed)

        # Feed button label init
        self.__feed_label = QLabel("Manual Feed")
        font = self.__feed_label.font()
        font.setPointSize(13)
        self.__feed_label.setFont(font)
        self.__feed_label.setAlignment(Qt.AlignHCenter)

        # Lights slider init
        self.__lights_slider = QSlider(Qt.Horizontal)
        self.__lights_slider.setMinimum(0)
        self.__lights_slider.setMaximum(2)
        self.__lights_slider.setTickPosition(QSlider.TicksAbove|QSlider.TicksBelow)
        self.__lights_slider.setPageStep(1)
        self.__lights_slider.setMaximumHeight(50)
        self.__lights_slider.valueChanged.connect(lambda val:self.dim_lights(val/self.__lights_slider.maximum()))

        # Lights slider label init
        self.__lights_label = QLabel("Lights dimming")
        font = self.__lights_label.font()
        font.setPointSize(13)
        self.__lights_label.setFont(font)
        self.__lights_label.setAlignment(Qt.AlignHCenter)

        # Manual Control button init
        self.__manual_control_button = QPushButton()
        self.__manual_control_button.setText("Manual Control")
        self.__manual_control_button.setDisabled(False)
        self.__manual_control_button.setMaximumHeight(50)
        self.__manual_control_button.clicked.connect(self.__init_manual_control_window)

        # Manual Control label init
        self.__manual_control_label = QLabel("Manual Control")
        font = self.__manual_control_label.font()
        font.setPointSize(13)
        self.__manual_control_label.setFont(font)
        self.__manual_control_label.setAlignment(Qt.AlignHCenter)

        # Fish image init
        self.__fish_image = QLabel() #TODO : add resize+update
        self.__fish_image.setScaledContents(True)
        self.__fish_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        

        # Fish image label init
        self.__fish_image_label = QLabel("Fish Camera")
        font = self.__fish_image_label.font()
        font.setPointSize(15)
        self.__fish_image_label.setFont(font)
        self.__fish_image_label.setAlignment(Qt.AlignHCenter)
        self.__fish_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Room image init
        self.__room_image = QLabel() #TODO : add resize+update
        self.__room_image.setScaledContents(True)
        self.__room_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Room image label init
        self.__room_image_label = QLabel("Room Camera")
        font = self.__room_image_label.font()
        font.setPointSize(15)
        self.__room_image_label.setFont(font)
        self.__room_image_label.setAlignment(Qt.AlignHCenter)
        self.__room_image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
    def __set_buttons_state(self, state:tuple):
        self.__start_button.setDisabled(state[0])
        self.__stop_button.setDisabled(state[1])
        self.__reset_button.setDisabled(state[2])
        self.__close_button.setDisabled(state[3])
        # Add feed button
        # Add lights slider?

    def __init_subscriptions_and_services(self):
        self.fish_image_sub = rospy.Subscriber('fish_camera/image', Image, self.read_fish_image)
        self.fish_dir_sub = rospy.Subscriber('fish_detection/direction', UInt16, self.update_direction)
        self.stitched_image_sub = rospy.Subscriber('ceiling_cameras/stitched_image', Image, self.update_room_image)
        self.feed = rospy.ServiceProxy('fish_feeder/feed', Trigger)
        self.dim_lights = rospy.ServiceProxy('light_dimmer/change', Light)
        self.fish_camera_control = rospy.ServiceProxy('fish_camera/system_toggle', SetBool)
        self.ceiling_cameras_control = rospy.ServiceProxy('ceiling_cameras/system_toggle', SetBool)
        self.lidar_control = rospy.ServiceProxy('lidar/system_toggle', SetBool)
        self.fish_detection_control = rospy.ServiceProxy('fish_detection/system_toggle', SetBool)
        self.light_dimmer_control = rospy.ServiceProxy('light_dimmer/system_toggle', SetBool)
        self.motor_control = rospy.ServiceProxy('motor_control/system_toggle', SetBool)
        self.manual_control_cmd = rospy.Publisher('/gui/manual_control_cmd', Twist, queue_size=10)

    def __systems_toggle(self, state: bool):
        rospy.wait_for_service('fish_camera/system_toggle')
        self.fish_camera_control(state)
        rospy.wait_for_service('ceiling_cameras/system_toggle')
        self.ceiling_cameras_control(state)
        rospy.wait_for_service('lidar/system_toggle')
        self.lidar_control(state)
        rospy.wait_for_service('fish_detection/system_toggle')
        self.fish_detection_control(state)
        rospy.wait_for_service('light_dimmer/system_toggle')
        self.light_dimmer_control(state)
        rospy.wait_for_service('motor_control/system_toggle')
        self.motor_control(state)

    def __init_manual_control_window(self):
        self.__manual_control_window = QDialog(self)
        self.__manual_control_window.setWindowTitle("Manual Robot Control")
        self.__manual_control_window.setFixedSize(300, 300)
        self.__manual_control_window.setWindowModality(Qt.ApplicationModal)

        def on_close(event):
            self.__velocity = Twist()
            self.manual_control_cmd.publish(self.__velocity)

            # Stop the timer
            if self.__timer:
                self.__timer.stop()
                self.__timer = None

            # Clean up the widget reference
            self.__manual_control_window = None

            # Accept the close event
            event.accept()

        def on_key_press(event):
            """
            Handle key press events for robot control.
            """
            key = event.key()
            event_type = True if event.type() == QEvent.KeyPress else False
            if key == Qt.Key_W:  # Forward
                self.__update_velocity("forward", event_type)
            elif key == Qt.Key_S:  # Backward
                self.__update_velocity("backward", event_type)
            elif key == Qt.Key_A:  # Left
                self.__update_velocity("left", event_type)
            elif key == Qt.Key_D:  # Right
                self.__update_velocity("right", event_type)
            elif key == Qt.Key_Q:  # Counterclockwise rotation
                self.__update_velocity("ccw", event_type)
            elif key == Qt.Key_E:  # Clockwise rotation
                self.__update_velocity("cw", event_type)
            else:
                event.ignore()

        self.__manual_control_window.closeEvent = on_close
        self.__manual_control_window.keyPressEvent = on_key_press
        self.__manual_control_window.keyReleaseEvent = on_key_press

        control_layout = QGridLayout()

        # Direction buttons
        self.__velocity = Twist()
        forward_button = QPushButton("↑")
        backward_button = QPushButton("↓")
        left_button = QPushButton("←")
        right_button = QPushButton("→")
        cw_button = QPushButton("↻")
        ccw_button = QPushButton("↺")

        control_layout.addWidget(forward_button, 0, 1)
        control_layout.addWidget(backward_button, 2, 1)
        control_layout.addWidget(left_button, 1, 0)
        control_layout.addWidget(right_button, 1, 2)
        control_layout.addWidget(cw_button, 1, 3)
        control_layout.addWidget(ccw_button, 1, 4)

        forward_button.pressed.connect(lambda: self.__update_velocity("forward", True))
        forward_button.released.connect(lambda: self.__update_velocity("forward", False))
        backward_button.pressed.connect(lambda: self.__update_velocity("backward", True))
        backward_button.released.connect(lambda: self.__update_velocity("backward", False))
        left_button.pressed.connect(lambda: self.__update_velocity("left", True))
        left_button.released.connect(lambda: self.__update_velocity("left", False))
        right_button.pressed.connect(lambda: self.__update_velocity("right", True))
        right_button.released.connect(lambda: self.__update_velocity("right", False))
        cw_button.pressed.connect(lambda: self.__update_velocity("cw", True))
        cw_button.released.connect(lambda: self.__update_velocity("cw", False))
        ccw_button.pressed.connect(lambda: self.__update_velocity("ccw", True))
        ccw_button.released.connect(lambda: self.__update_velocity("ccw", False))

        self.__manual_control_window.setLayout(control_layout)
        
        self.__timer = QTimer(self)
        self.__timer.timeout.connect(lambda: self.manual_control_cmd.publish(self.__velocity))
        self.__timer.start(100)  # Call every 100ms

        self.__manual_control_window.show()


    def __update_velocity(self, direction, is_pressed):
        """
        Update robot velocity based on button presses.
        """
        linear_speed = 0.5
        angular_speed = 1.0

        if direction == "forward":
            self.__velocity.linear.x = linear_speed if is_pressed else 0
        elif direction == "backward":
            self.__velocity.linear.x = -linear_speed if is_pressed else 0
        elif direction == "left":
            self.__velocity.linear.y = linear_speed if is_pressed else 0
        elif direction == "right":
            self.__velocity.linear.y = -linear_speed if is_pressed else 0
        elif direction == "cw":
            self.__velocity.angular.z = -angular_speed if is_pressed else 0
        elif direction == "ccw":
            self.__velocity.angular.z = angular_speed if is_pressed else 0

        # Publish the velocity to the robot
        self.manual_control_cmd.publish(self.__velocity)
        

    def read_fish_image(self, img_msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.TL_layout.update(img)
        except CvBridgeError as e:
            rospy.logwarn(e)

    def update_direction(self, dir:UInt16):
        pass

    def update_room_image(self, img_msg: Image):
        """
        Callback to update the room camera image on the GUI.
        """
        try:
            # Convert the ROS Image message to a numpy array
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            
            # Check the shape to ensure it's compatible with OpenCV's BGR format
            if img.ndim == 3 and img.shape[2] == 3:
                # Assume the image is BGR and proceed
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                rospy.logwarn("Unexpected image format, expected 3-channel image.")
                return
            
            # Convert to QImage
            height, width, channel = img.shape
            bytes_per_line = 3 * width
            q_image = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Scale the image to fit the QLabel
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaled(
                self.__room_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # Update the QLabel with the scaled QPixmap
            self.__room_image.setPixmap(scaled_pixmap)
            
        except CvBridgeError as e:
            rospy.logwarn(f"Error converting image message: {e}")
        except Exception as e:
            rospy.logwarn(f"Unexpected error in update_room_image: {e}")


    def resizeEvent(self, event):
        if self.__room_image.pixmap():
            scaled_pixmap = self.__room_image.pixmap().scaled(
                self.__room_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.__room_image.setPixmap(scaled_pixmap)
        if self.__fish_image.pixmap():
            scaled_pixmap = self.__fish_image.pixmap().scaled(
                self.__fish_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.__fish_image.setPixmap(scaled_pixmap)
        super(MainWindow, self).resizeEvent(event)

        

    def showEvent(self, event):
        self.showMaximized()

    def mousePressEvent(self, event):
        # Store the initial mouse position
        self.drag_start = event.globalPos()

    def mouseMoveEvent(self, event):
        # Calculate the movement distance
        move_distance = event.globalPos() - self.drag_start
        
        # If the movement is too large, reset the drag start
        if move_distance.manhattanLength() > 10:
            self.drag_start = event.globalPos()
        else:
            event.ignore()

    def __shutdown(self):
        QApplication.quit()
        rospy.signal_shutdown("Closing GUI")


if __name__ == "__main__":
    rospy.init_node('gui_node')
    rospy.loginfo("GUI Node: node created.")
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    app.exec()

    rospy.spin()
#!/usr/bin/env python3

# General imports
import sys
import rospy
import cv2
import numpy as np
import csv
import os
import datetime

# PyQt5 imports
from PyQt5.QtCore import Qt, QTimer, QEvent
from PyQt5.QtGui import QPixmap, QImage, QDoubleValidator
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
    QCheckBox,
    QLineEdit,
    QGroupBox,
    QRadioButton,
    )

# ROS imports
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge, CvBridgeError

# Custom ROS messages
from fov.srv import Light
from fov.msg import FishState, FomaLocation      

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Rospy configs
        self.__init_subscriptions_and_services()
        self.bridge = CvBridge()

        self.setWindowTitle("Trial control")
        self.drag_start = self.pos()
        self.closeEvent = self.__on_close_click

        self.__init_widgets()
        self.__init_layouts()
        self.__init_attributes()
        self.__init_timers()

    def __init_timers(self):
        self.__image_timer = QTimer(self)
        self.__image_timer.timeout.connect(self.__update_right_display)
        self.__image_timer.start(30)

        self.__services_timer = QTimer(self)
        self.__services_timer.timeout.connect(self.__update_services)
        self.__services_timer.start(1000)

    def __init_attributes(self):
        self.__img_location = None
        self.__world_location = None
        self.__velocity = Twist()
        self.__room_image = None
        self.__room_map = None
        self.__room_video_writer = None
        self.__foma_video_writer = None
        self.__location_file = None
        self.__location_csv_writer = None
        self.__current_timestamp = None
        self.__manual_control_window = None
        self.__feeding_load_window = None
        self.__image_timer = None
        self.__services_timer = None
        self.__manual_speed = 0.5
        self.__output_folder = '/home/icvl/FOMA/output'
        self.__feed = None
        self.__dim_lights = None
        self.__motor_mode_control = None

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
        self.__TR_layout.addWidget(self.__top_right_image)#, alignment=Qt.AlignCenter)
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
        # self.__BL_layout.addWidget(self.__manual_control_label, 0, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__manual_control_button, 0, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__feed_loading_button, 1, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__direction_group, 0, 3, 2, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__room_display_group, 0, 4, 2, 1, alignment=Qt.AlignCenter)
        
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

    def __init_widgets(self):
        # Start button init
        self.__start_button = QPushButton()
        self.__start_button.setText("Start")
        self.__start_button.setDisabled(False)
        self.__start_button.setMaximumHeight(50)
        self.__start_button.clicked.connect(self.__on_start_click)

        # Stop button init
        self.__stop_button = QPushButton()
        self.__stop_button.setText("Stop")
        self.__stop_button.setDisabled(True)
        self.__stop_button.setMaximumHeight(50)
        self.__stop_button.clicked.connect(self.__on_stop_click)

        # Reset button init
        self.__reset_button = QPushButton()
        self.__reset_button.setText("Reset")
        self.__reset_button.setDisabled(True)
        self.__reset_button.setMaximumHeight(50)
        self.__reset_button.clicked.connect(self.__on_reset_click)

        # Close button init
        self.__close_button = QPushButton()
        self.__close_button.setText("Close")
        self.__close_button.setDisabled(False)
        self.__close_button.setMaximumHeight(50)
        self.__close_button.clicked.connect(self.__on_close_click)

        # Feed button init
        self.__feed_button = QPushButton()
        self.__feed_button.setText("Feed")
        self.__feed_button.clicked.connect(lambda:self.__feed())
        self.__feed_button.setDisabled(True)

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
        self.__lights_slider.valueChanged.connect(lambda val:self.__dim_lights(255*val/self.__lights_slider.maximum()))
        self.__lights_slider.setDisabled(True)

        # Lights slider label init
        self.__lights_label = QLabel("Lights dimming")
        font = self.__lights_label.font()
        font.setPointSize(13)
        self.__lights_label.setFont(font)
        self.__lights_label.setAlignment(Qt.AlignHCenter)

        # Manual Control button init
        self.__manual_control_button = QPushButton()
        self.__manual_control_button.setText("Manual Control")
        self.__manual_control_button.setMaximumHeight(50)
        self.__manual_control_button.clicked.connect(self.__init_manual_control_window)
        self.__manual_control_button.setDisabled(True)

        # Manual Control label init
        self.__manual_control_label = QLabel("Manual Control")
        font = self.__manual_control_label.font()
        font.setPointSize(13)
        self.__manual_control_label.setFont(font)
        self.__manual_control_label.setAlignment(Qt.AlignHCenter)

        self.__feed_loading_button = QPushButton()
        self.__feed_loading_button.setText("Load Feeder")
        self.__feed_loading_button.setMaximumHeight(50)
        self.__feed_loading_button.clicked.connect(self.__init_feeding_load_window)
        self.__feed_loading_button.setDisabled(True)

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
        self.__top_right_image = QLabel() #TODO : add resize+update
        self.__top_right_image.setScaledContents(True)
        self.__top_right_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Room image label init
        self.__room_image_label = QLabel("Room Camera")
        font = self.__room_image_label.font()
        font.setPointSize(15)
        self.__room_image_label.setFont(font)
        self.__room_image_label.setAlignment(Qt.AlignHCenter)
        self.__room_image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Add radio buttons for toggling direction display
        self.__show_direction_rb = QRadioButton("Yes")
        self.__hide_direction_rb = QRadioButton("No")
        self.__show_direction_rb.setChecked(True)

        # Create and add radio buttons to layout
        direction_layout = QGridLayout()
        direction_layout.addWidget(self.__show_direction_rb, 0, 0, alignment=Qt.AlignCenter)
        direction_layout.addWidget(self.__hide_direction_rb, 1, 0, alignment=Qt.AlignCenter)
        
        # Create a group box for the radio buttons and set the layout
        self.__direction_group = QGroupBox("Display Direction")
        self.__direction_group.setLayout(direction_layout)

        # Add radio buttons for toggling direction display
        self.__room_camera_display_rb = QRadioButton("Camera")
        self.__map_display_rb = QRadioButton("Map")
        self.__room_camera_display_rb.setChecked(True)

        # Create and add radio buttons to layout
        room_display_layout = QGridLayout()
        room_display_layout.addWidget(self.__room_camera_display_rb, 0, 0, alignment=Qt.AlignCenter)
        room_display_layout.addWidget(self.__map_display_rb, 1, 0, alignment=Qt.AlignCenter)
        
        # Create a group box for the radio buttons and set the layout
        self.__room_display_group = QGroupBox("Top-Right Display")
        self.__room_display_group.setLayout(room_display_layout)

    def __init_subscriptions_and_services(self):
        self.fish_image_sub = rospy.Subscriber('fish_camera/image', Image, self.__update_fish_image)
        self.fish_dir_sub = rospy.Subscriber('fish_detection/direction', FishState, self.__update_fish_state)
        self.stitched_image_pub = rospy.Subscriber('ceiling_camera/image', Image, self.__update_room_image)
        # self.stitched_image_pub = rospy.Subscriber('image_stitcher/image', Image, self.update_room_image)
        self.location_sub = rospy.Subscriber('localization/location', FomaLocation, self.__update_foma_location)
        self.manual_control_cmd = rospy.Publisher('gui/manual_control', Twist, queue_size=10)

    def __init_manual_control_window(self):
        rospy.wait_for_service('motor_control/motor_mode_control')
        self.__motor_mode_control(True)
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
            self.__motor_mode_control(False)

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
        # self.__velocity = Twist()
        forward_button = QPushButton("↑")
        backward_button = QPushButton("↓")
        left_button = QPushButton("←")
        right_button = QPushButton("→")
        cw_button = QPushButton("↻")
        ccw_button = QPushButton("↺")
        bypass_lidar_label = QLabel("Bypass LIDAR")
        bypass_lidar_checkbox = QCheckBox()
        speed_control_label = QLabel("Speed Control")
        speed_control_textbox = QLineEdit()
        speed_control_button = QPushButton("Set")
        
        speed_control_textbox.setPlaceholderText("0.5")
        speed_control_textbox.setValidator(QDoubleValidator(0.0, 1.0, 2))
        def set_speed():
            try:
                speed = float(speed_control_textbox.text())
                self.loginfo(f"Speed set to: {speed}")
                self.__manual_speed = speed
            except ValueError:
                self.logwarn("Invalid speed value")

        control_layout.addWidget(forward_button, 0, 1, 1, 2)
        control_layout.addWidget(backward_button, 2, 1, 1, 2)
        control_layout.addWidget(left_button, 1, 0)
        control_layout.addWidget(right_button, 1, 3)
        control_layout.addWidget(cw_button, 1, 1)
        control_layout.addWidget(ccw_button, 1, 2)
        control_layout.addWidget(bypass_lidar_label, 3, 0, 1, 2)
        control_layout.addWidget(bypass_lidar_checkbox, 3, 2, 1, 2)
        control_layout.addWidget(speed_control_label, 4, 0, 1, 2)
        control_layout.addWidget(speed_control_textbox, 4, 2)
        control_layout.addWidget(speed_control_button, 4, 3)


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
        speed_control_button.clicked.connect(set_speed)
        def toggle_lidar_bypass(state):
            try:
                rospy.wait_for_service('motor_control/bypass_lidar')
                lidar_bypass_service = rospy.ServiceProxy('motor_control/bypass_lidar', SetBool)
                response = lidar_bypass_service(state)
                if response.success:
                    self.loginfo(f"LIDAR bypass set to: {state}")
                else:
                    self.logwarn(f"Failed to set LIDAR bypass to: {state}")
            except rospy.ServiceException as e:
                self.logwarn(f"Service call failed: {e}")

        bypass_lidar_checkbox.stateChanged.connect(lambda state: toggle_lidar_bypass(state == Qt.Checked))

        self.__manual_control_window.setLayout(control_layout)
        
        self.__timer = QTimer(self)
        self.__timer.timeout.connect(lambda: self.manual_control_cmd.publish(self.__velocity))
        self.__timer.start(100)  # Call every 100ms

        self.__manual_control_window.show()

    def __init_feeding_load_window(self):
        rospy.wait_for_service('fish_feeder/feed')
        self.__feeding_load_window = QDialog(self)
        self.__feeding_load_window.setWindowTitle("Load feeder")
        self.__feeding_load_window.setFixedSize(300, 300)
        self.__feeding_load_window.setWindowModality(Qt.ApplicationModal)
        step = -1

        def empty_feeder():
            for _ in range(30):
                self.__feed()
                rospy.sleep(0.1)

        def on_key_press(event):
            """
            Handle key press events for robot control.
            """
            key = event.key()
            if key == Qt.Key_Return and step!=-1:  # Enter
                next_step()
            else:
                event.ignore()
        
        def next_step():
            nonlocal step
            if step in range(4):
                for _ in range(6):
                    self.__feed()
                step += 1
                step_label.setText(f"Step: {step}/5")
            elif step == 4:
                for _ in range(4):
                    self.__feed()
                step =-1
                empty_button.setDisabled(False)
                step_label.setText("Finished Loading")
                enter_label.setVisible(False)

        self.__feeding_load_window.keyPressEvent = on_key_press

        control_layout = QGridLayout()

        empty_button = QPushButton("Empty Feeder")
        start_load_button = QPushButton("Start Load")
        step_label = QLabel()
        enter_label = QLabel("Press Enter to continue")
        step_label.setVisible(False)
        enter_label.setVisible(False)

        def on_start_click():
            nonlocal step
            empty_button.setDisabled(True)
            step_label.setText("Step: 0/5")
            step_label.setVisible(True)
            enter_label.setVisible(True)
            step = 0
            self.__feeding_load_window.setFocusPolicy(Qt.StrongFocus)
            self.__feeding_load_window.setFocus()
        
        control_layout.addWidget(empty_button, 0, 0)
        control_layout.addWidget(start_load_button, 1, 0)
        control_layout.addWidget(step_label, 2, 0)
        control_layout.addWidget(enter_label, 3, 0)

        empty_button.pressed.connect(empty_feeder)
        start_load_button.pressed.connect(on_start_click)

        self.__feeding_load_window.setLayout(control_layout)
        self.__feeding_load_window.show()

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
        
    def __update_buttons_state(self, state: tuple):
        self.__start_button.setDisabled(state[0])
        self.__stop_button.setDisabled(state[1])
        self.__reset_button.setDisabled(state[2])
        self.__close_button.setDisabled(state[3])
        # Add feed button
        # Add lights slider?

    def __update_velocity(self, direction: str, is_pressed: bool):
        """
        Update robot velocity based on button presses.
        """

        if direction == "forward":
            self.__velocity.linear.y = self.__manual_speed if is_pressed else 0
        elif direction == "left":
            self.__velocity.linear.x = -self.__manual_speed if is_pressed else 0
        elif direction == "backward":
            self.__velocity.linear.y = -self.__manual_speed if is_pressed else 0
        elif direction == "right":
            self.__velocity.linear.x = self.__manual_speed if is_pressed else 0
        elif direction == "cw":
            self.__velocity.angular.z = self.__manual_speed if is_pressed else 0
        elif direction == "ccw":
            self.__velocity.angular.z = -self.__manual_speed if is_pressed else 0

        # Publish the velocity to the robot
        self.manual_control_cmd.publish(self.__velocity)
        # self.loginfo(f"Velocity: {self.__velocity}")     

    def __update_fish_image(self, img_msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg)
            # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            q_image = QImage(img.data, img.shape[1], img.shape[0], QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.__fish_image.setPixmap(pixmap)
        except CvBridgeError as e:
            self.logwarn(e)

    def __update_fish_state(self, state: FishState):
        """
        Update the GUI image based on the FishState message.

        Args:
            state (FishState): The state containing the direction and position of the fish.
        """
        # Check if direction display is enabled before doing any work
        if not self.show_direction_rb.isChecked():
            self.loginfo("Direction display is disabled. Skipping updates.")
            return

        # Ensure the fish image exists
        if self.__fish_image.pixmap() is None:
            self.logwarn("No image available to update")
            return

        # Convert QLabel's pixmap to a numpy array
        pixmap = self.__fish_image.pixmap()
        if pixmap is None:
            self.logwarn("No pixmap available")
            return
        image = pixmap.toImage()

        # Convert QImage to numpy array
        width = image.width()
        height = image.height()
        ptr = image.bits()
        ptr.setsize(image.byteCount())
        image_array = np.array(ptr).reshape(height, width, 4)  # Assume ARGB32 format

        # Convert to BGR for OpenCV processing
        frame = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)

        # Draw the position on the frame
        position = (state.x, state.y)
        cv2.circle(frame, position, 5, (0, 0, 255), -1)  # Red circle

        # Annotate the direction near the position
        direction_text = f"Dir: {state.direction}"
        cv2.putText(frame, direction_text, (position[0] + 10, position[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Optionally draw an arrow to indicate direction
        end_point = (position[0] + int(30 * np.cos(np.radians(state.direction))),
                    position[1] - int(30 * np.sin(np.radians(state.direction))))
        cv2.arrowedLine(frame, position, end_point, (0, 255, 0), 2, tipLength=0.3)

        # Convert back to QImage
        image_with_annotations = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        qimage = QImage(image_with_annotations.data, image_with_annotations.shape[1],
                        image_with_annotations.shape[0], QImage.Format_ARGB32)

        # Resize the image to fit the QLabel
        resized_pixmap = QPixmap.fromImage(qimage).scaled(
            self.__fish_image.width(),
            self.__fish_image.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )

        # Update the QLabel with the new image
        self.__fish_image.setPixmap(resized_pixmap)

    def __update_room_image(self, img_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8") # was passthrough
            self.__room_image = frame.copy() # might not be correct
            if self.__img_location:
                center_x = self.__img_location.x
                center_y = self.__img_location.y
                cv2.circle(self.__room_image, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
            if self.__room_video_writer:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.__room_video_writer.write(frame)
            # self.update_room_image() 
        except CvBridgeError as e:
            self.logwarn(f"Error converting image message: {e}")
        except Exception as e:
            self.logwarn(f"Unexpected error in update_room_image: {e}")

    def __update_foma_location(self, location: FomaLocation):
        self.__img_location = location.image
        self.__world_location = location.world
        timestamp = rospy.get_time()  # ROS time in seconds
        self.__location_csv_writer.writerow([timestamp, self.__world_location.x, self.__world_location.y, self.__img_location.x, self.__img_location.y])
        self.__location_file.flush()  # Ensure data is written to the file

        if self.__room_map is not None:
            # Ensure coordinates stay within bounds
            x = np.clip(self.__world_location.x, 0, 500 - 1)
            y = np.clip(self.__world_location.y, 0, 500 - 1)
            cv2.circle(self.__room_map, (x, y), 5, (0, 255, 0), -1)

    def __update_right_display(self):
        """
        Callback to update the room camera image on the GUI only ATM.
        """
        data = None
        if self.__room_camera_display_rb.isChecked() and self.__room_image is not None:
            height, width, channel = self.__room_image.shape
            bytes_per_line = 3 * width
            data = self.__room_image.data

        elif self.__map_display_rb.isChecked() and self.__room_map is not None:
            height, width, channel = self.__room_map.shape
            bytes_per_line = 3 * width
            map = self.__room_map.copy()
            # Ensure coordinates stay within bounds
            x = np.clip(self.__world_location.x, 0, 500 - 1)
            y = np.clip(self.__world_location.y, 0, 500 - 1)
            cv2.circle(map, (x, y), 5, (0, 0, 255), -1)
            data = map.data

        if data is not None:
            q_image = QImage(data, width, height, bytes_per_line, QImage.Format_RGB888)             
            # Scale the image to fit the QLabel
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaled(
                self.__top_right_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # Update the QLabel with the scaled QPixmap
            self.__top_right_image.setPixmap(scaled_pixmap)

    def __update_services(self):
        def is_service_available(service_name):
            try:
                rospy.wait_for_service(service_name, timeout=1)  # Timeout of 1 second
                return True
            except rospy.ROSException:
                return False

        if not (self.__feed is None) ^ is_service_available('fish_feeder/feed'):
            if self.__feed:
                self.logerr("Feeder service unavailable - disabling buttons")
                self.__feed_button.setDisabled(True)
                self.__feed_loading_button.setDisabled(True)
                self.__feed = None
            else:
                self.loginfo("Feeder service available - enabling buttons")
                self.__feed_button.setDisabled(False)
                self.__feed = rospy.ServiceProxy('fish_feeder/feed', Trigger)

        if not (self.__dim_lights is None) ^ is_service_available('fish_feeder/feed'):
            if self.__dim_lights:
                self.logerr("Light dimming service unavailable - disabling buttons")
                self.__lights_slider.setDisabled(True)
                self.__dim_lights = None
            else:
                self.loginfo("Light dimming service available - enabling buttons")
                self.__lights_slider.setDisabled(False)
                self.__dim_lights = rospy.ServiceProxy('light_dimmer/change', Light)

        if not (self.__motor_mode_control is None) ^ is_service_available('fish_feeder/feed'):
            if self.__motor_mode_control:
                self.logerr("Manual control service unavailable - disabling buttons")
                self.__manual_control_button.setDisabled(True)
                self.__motor_mode_control = None
            else:
                self.loginfo("Manual control service available - enabling buttons")
                self.__manual_control_button.setDisabled(False)
                self.__motor_mode_control = rospy.ServiceProxy('motor_control/motor_mode_control', SetBool)
        
        # self.fish_camera_control = rospy.ServiceProxy('fish_camera/system_toggle', SetBool)
        # self.ceiling_cameras_control = rospy.ServiceProxy('ceiling_cameras/system_toggle', SetBool)
        # self.lidar_control = rospy.ServiceProxy('lidar/system_toggle', SetBool)
        # self.fish_detection_control = rospy.ServiceProxy('fish_detection/system_toggle', SetBool)
        # self.light_dimmer_control = rospy.ServiceProxy('light_dimmer/system_toggle', SetBool)
        # self.motor_control = rospy.ServiceProxy('motor_control/system_toggle', SetBool)

    def __update_gui(self):
        self.__update_right_display()
        self.__update_services()

    def __on_start_click(self):
        self.__update_buttons_state((True,False,True,True))
   
        # 1. FOMA Location
        # Generate file name with current timestamp (YYYYMMDDHHMM)
        self.__current_timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M")
        location_filename = os.path.join(self.__output_folder, f"foma_location_{self.__current_timestamp}.csv")

        # Open CSV file in append mode and create writer
        self.__location_file = open(location_filename, 'a', newline='')
        self.__location_csv_writer = csv.writer(self.__location_file)

        # Write header if the file is new
        if os.stat(location_filename).st_size == 0:
            self.__location_csv_writer.writerow(["time", "x_w", "y_w", "x_i", "y_i"])
            self.__location_file.flush()  # Ensure the header is written immediately

        # 2. Room Video
        room_video_filename = os.path.join(self.__output_folder, f"room_video_{self.__current_timestamp}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # MP4 format
        room_frame_width = 1024  # Adjust based on your camera resolution
        room_frame_height = 768
        room_fps = 10  # Default FPS (adjust based on the camera FPS)
        self.__room_video_writer = cv2.VideoWriter(room_video_filename, fourcc, room_fps, (room_frame_width, room_frame_height))
        # if self.__room_video_writer.isOpened():
        #     self.loginfo(f"Room video writer opened with filename: {room_video_filename}")


        # 3. FOMA Video
        foma_video_filename = os.path.join(self.__output_folder, f"foma_video_{self.__current_timestamp}.mp4")
        # fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # MP4 format
        foma_frame_width = 1024  # Adjust based on your camera resolution
        foma_frame_height = 768
        foma_fps = 10  # Default FPS (adjust based on the camera FPS)
        self.__foma_video_writer = cv2.VideoWriter(foma_video_filename, fourcc, foma_fps, (foma_frame_width, foma_frame_height))

        # 4. Room Map
        # Create a white image representing the room
        self.__room_map = np.ones((500, 500, 3), dtype=np.uint8) * 255

    def __on_stop_click(self):
        self.__update_buttons_state((True,True,False,False))
        self.__close_file_writers()

    def __on_reset_click(self):
        self.__update_buttons_state((False,True,True,False))
        self.__close_file_writers()

    def __on_close_click(self, event):
        self.__close_file_writers()
        QApplication.quit()
        rospy.signal_shutdown("Closing GUI")

    def __close_file_writers(self):
        if self.__location_file:
            self.__location_file.close()
            self.__location_file = None
            self.__location_csv_writer = None
        if self.__room_video_writer:
            self.__room_video_writer.release()
            self.__room_video_writer = None
        if self.__foma_video_writer:
            self.__foma_video_writer.release()
            self.__foma_video_writer = None
        if self.__room_map is not None:
            cv2.imwrite(f"room_map_{self.__current_timestamp}.png", self.__room_map)

    def __update_image(self, img_msg: Image, destination: QLabel):
        """
        Callback to update the camera image on the GUI.
        """
        try:
            # Convert the ROS Image message to a numpy array
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            
            # Check the shape to ensure it's compatible with OpenCV's BGR format
            if img.ndim == 3 and img.shape[2] == 3:
                # Assume the image is BGR and proceed
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                self.logwarn("Unexpected image format, expected 3-channel image.")
                return
            
            # Convert to QImage
            height, width, channel = img.shape
            bytes_per_line = 3 * width
            q_image = QImage(img.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Scale the image to fit the QLabel
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaled(
                destination.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # Update the QLabel with the scaled QPixmap
            destination.setPixmap(scaled_pixmap)
            
        except CvBridgeError as e:
            self.logwarn(f"Error converting image message: {e}")
        except Exception as e:
            self.logwarn(f"Unexpected error in update_image: {e}")

    def resizeEvent(self, event):
        if self.__top_right_image.pixmap():
            scaled_pixmap = self.__top_right_image.pixmap().scaled(
                self.__top_right_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.__top_right_image.setPixmap(scaled_pixmap)
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

    def logerr(self, msg):
        rospy.logerr(f"GUI Node: {msg}")

    def logwarn(self, msg):
        rospy.logwarn(f"GUI Node: {msg}")

    def loginfo(self, msg):
        rospy.logerr(f"GUI Node: {msg}")


if __name__ == "__main__":
    rospy.init_node('gui_node')
    rospy.loginfo("GUI Node: Node created.")
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    app.exec()

    rospy.spin()
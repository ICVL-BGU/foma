#!/usr/bin/env python3

# General imports
import sys
import rospy
import cv2
import numpy as np
import csv
import os
import datetime
import math

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
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int16MultiArray
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge, CvBridgeError

# Custom ROS messages
from foma.srv import Light, Check
from foma.msg import FomaLocation

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
        self.__image_timer.timeout.connect(self.__update_gui)
        self.__image_timer.start(20)

        self.__services_timer = QTimer(self)
        self.__services_timer.timeout.connect(self.__update_services)
        self.__services_timer.start(1000)

        self.__writers_timer = QTimer(self)
        self.__writers_timer.timeout.connect(self.__write_files)

    def __init_attributes(self):
        # Images and locations
        self.__foma_img_location = None
        self.__foma_world_location = None
        self.__room_image = None
        self.__room_map = None
        self.__fish_state = None
        self.__fish_image = None

        # Writer files and writers
        self.__output_folder = '~/trial_output'
        self.__trial_timestamp = None
        self.__room_video_writer = None
        self.__foma_video_writer = None
        self.__foma_location_file = None
        self.__foma_location_csv_writer = None
        self.__fish_location_file = None
        self.__fish_location_csv_writer = None

        # Windows
        self.__manual_control_window = None
        self.__feeding_load_window = None

        # Timers
        self.__image_timer = None
        self.__services_timer = None
        self.__writers_timer = None

        # Motor Control
        self.__linear_velocity = Twist()
        self.__angular_velocity = Float32()
        # self.__speed = 1
        self.__direction_epsilon = 20
        self.__blocked_directions = None

        # Services
        self.__motor_control_system_check = None
        self.__dim_lights = None
        self.__feed = None
        self.__bypass_lidar = None

        # Trial Control
        self.__ongoing_trial = False 

    def __init_layouts(self):
        # Top-Left (Fish Camera)
        self.__TL_layout = QVBoxLayout()
        self.__TL_layout.addWidget(self.__fish_image_label, alignment=Qt.AlignCenter)
        self.__TL_layout.addWidget(self.__left_image_frame)#, alignment=Qt.AlignCenter)
        
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
        self.__lights_slider.setMaximum(1)
        self.__lights_slider.setTickPosition(QSlider.TicksAbove|QSlider.TicksBelow)
        self.__lights_slider.setPageStep(1)
        self.__lights_slider.setMaximumHeight(50)
        self.__lights_slider.valueChanged.connect(lambda val:self.__dim_lights(int(255*val/self.__lights_slider.maximum())))
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
        self.__left_image_frame = QLabel() #TODO : add resize+update
        self.__left_image_frame.setScaledContents(True)
        self.__left_image_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        

        # Fish image label init
        self.__fish_image_label = QLabel("Fish Camera")
        font = self.__fish_image_label.font()
        font.setPointSize(15)
        self.__fish_image_label.setFont(font)
        self.__fish_image_label.setAlignment(Qt.AlignHCenter)
        self.__left_image_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

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
        rospy.Subscriber('fish_camera/image', Image, self.__update_fish_image)
        rospy.Subscriber('fish_detection/state', Twist, self.__update_fish_state)
        rospy.Subscriber('ceiling_camera/image', Image, self.__update_room_image)
        rospy.Subscriber('localization/location', FomaLocation, self.__update_foma_location)
        rospy.Subscriber('motor_control/blocked', Int16MultiArray, self.__update_blocked_directions)
        self.__motor_control_twist = rospy.Publisher('motor_control/twist', Twist, queue_size=10)
        self.__motor_control_dir = rospy.Publisher('motor_control/angle', Float32, queue_size=10)
        self.__motor_control_vector = rospy.Publisher('motor_control/vector', Vector3, queue_size=10)
        self.__motor_control_rotate = rospy.Publisher('motor_control/rotate', Float32, queue_size=10)
        self.__motor_set_speed = rospy.Publisher('motor_control/set_speed', Float32, queue_size=10)
        # self.__motor_control_system_check = rospy.ServiceProxy('motor_control/system_check', Check)

    def __init_manual_control_window(self):
        self.__motor_set_speed.publish(Float32(0.5))
        self.__bypass_lidar(False)

        self.__manual_control_window = QDialog(self)
        self.__manual_control_window.setWindowTitle("Manual Robot Control")
        self.__manual_control_window.setFixedSize(300, 300)
        self.__manual_control_window.setWindowModality(Qt.ApplicationModal)

        def on_close(event):
            self.__motor_control_twist.publish(Twist())
            self.__motor_set_speed.publish(Float32(1.0))
            self.__bypass_lidar(False)

            # Stop the timer
            # if self.__timer:
            #     self.__timer.stop()
            #     self.__timer = None

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
                self.__update_velocity(0, event_type)
            elif key == Qt.Key_S:  # Backward
                self.__update_velocity(180, event_type)
            elif key == Qt.Key_A:  # Left
                self.__update_velocity(90, event_type)
            elif key == Qt.Key_D:  # Right
                self.__update_velocity(270, event_type)
            elif key == Qt.Key_Q:  # Counterclockwise rotation
                self.__update_velocity(-2, event_type)
            elif key == Qt.Key_E:  # Clockwise rotation
                self.__update_velocity(-1, event_type)
            else:
                event.ignore()

        self.__manual_control_window.closeEvent = on_close
        self.__manual_control_window.keyPressEvent = on_key_press
        self.__manual_control_window.keyReleaseEvent = on_key_press

        control_layout = QGridLayout()

        # Direction buttons
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
                self.__motor_set_speed.publish(Float32(speed))
                # self.__speed = speed
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


        forward_button.pressed.connect(lambda: self.__update_velocity(0, True))
        forward_button.released.connect(lambda: self.__update_velocity(0, False))
        backward_button.pressed.connect(lambda: self.__update_velocity(180, True))
        backward_button.released.connect(lambda: self.__update_velocity(180, False))
        left_button.pressed.connect(lambda: self.__update_velocity(90, True))
        left_button.released.connect(lambda: self.__update_velocity(90, False))
        right_button.pressed.connect(lambda: self.__update_velocity(270, True))
        right_button.released.connect(lambda: self.__update_velocity(270, False))
        cw_button.pressed.connect(lambda: self.__update_velocity(-1, True))
        cw_button.released.connect(lambda: self.__update_velocity(-1, False))
        ccw_button.pressed.connect(lambda: self.__update_velocity(-2, True))
        ccw_button.released.connect(lambda: self.__update_velocity(-2, False))
        speed_control_button.clicked.connect(set_speed)

        bypass_lidar_checkbox.stateChanged.connect(lambda state: self.__bypass_lidar(state == Qt.Checked))

        self.__manual_control_window.setLayout(control_layout)
        
        # self.__timer = QTimer(self)
        # self.__timer.timeout.connect(lambda: self.__motor_control_twist.publish(self.__velocity))
        # self.__timer.start(50)  # Call every 100ms

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

    def __update_velocity(self, direction: int, is_pressed: bool):
        """
        Update robot velocity based on button presses.
        """
        if direction >= 0:
            # self.__linear_velocity = Twist()
            if direction == 0: # "forward"
                self.__linear_velocity.linear.y = 1 if is_pressed else 0
            elif direction == 90: # "left"
                self.__linear_velocity.linear.x = -1 if is_pressed else 0
            elif direction == 180: # "backward"
                self.__linear_velocity.linear.y = -1 if is_pressed else 0
            elif direction == 270: # "right"
                self.__linear_velocity.linear.x = 1 if is_pressed else 0

            # Publish the velocity to the robot
            self.__motor_control_twist.publish(self.__linear_velocity)  
        else:
            # self.__angular_velocity = Float32()
            if direction == -1: # "cw"
                self.__angular_velocity.data = 1 if is_pressed else 0
            elif direction == -2: # "ccw":
                self.__angular_velocity.data = -1 if is_pressed else 0
            # Publish the velocity to the robot
            self.__motor_control_rotate.publish(self.__angular_velocity)

    def __update_fish_image(self, img_msg: Image):
        try:
            self.__fish_image = self.bridge.imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
            self.logwarn(e)

    def __update_fish_state(self, state: Twist):
        if state.angular == Vector3(0, 0, 0):
            self.__fish_state = None
        else:
            self.__fish_state = state
        if self.__ongoing_trial and self.__fish_image is not None:
            # Get fish location
            fish_x = int(state.linear.x)
            fish_y = int(state.linear.y)

            # Calculate the vector from the image center to the fish location
            height, width, _ = self.__fish_image.shape
            center_x = width // 2
            center_y = height // 2
            vector_x = fish_x - center_x
            vector_y = fish_y - center_y

            if vector_x == 0 and vector_y == 0:
                self.__motor_control_vector.publish(Vector3(0, 0, 0))

            # Calculate the direction angle of the fish
            direction_x = state.angular.x
            direction_y = state.angular.y
            direction_angle = math.degrees(math.atan2(direction_y, direction_x))

            # Calculate the angle of the vector from the center to the fish
            center_to_fish_angle = math.degrees(math.atan2(vector_y, vector_x))

            # Check if the direction matches the vector angle within epsilon
            if abs(direction_angle - center_to_fish_angle) <= self.__direction_epsilon:
                self.__motor_control_dir.publish(direction_angle)
                self.loginfo(f"Direction match: {direction_angle}° (expected {center_to_fish_angle}°)")
            else:
                self.__motor_control_vector.publish(Vector3(0, 0, 0))

    def __update_room_image(self, img_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8") # was passthrough
            self.__room_image = frame.copy() # might not be correct
            if self.__foma_img_location:
                center_x = self.__foma_img_location.x
                center_y = self.__foma_img_location.y
                cv2.circle(self.__room_image, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
        except CvBridgeError as e:
            self.logwarn(f"Error converting image message: {e}")
        except Exception as e:
            self.logwarn(f"Unexpected error in update_room_image: {e}")

    def __update_foma_location(self, location: FomaLocation):
        self.__foma_img_location = location.image
        self.__foma_world_location = location.world
        
        if self.__room_map is not None:
            # Ensure coordinates stay within bounds
            x = np.clip(self.__foma_world_location.x, 0, 500 - 1)
            y = np.clip(self.__foma_world_location.y, 0, 500 - 1)
            cv2.circle(self.__room_map, (x, y), 5, (0, 255, 0), -1)

    def __update_blocked_directions(self, blocked: Int16MultiArray):
        """
        Update the blocked directions based on the received data.
        """
        self.__blocked_directions = blocked.data
        # self.loginfo(f"Blocked directions: {self.__blocked_directions}")
        
    def __update_left_display(self):
        """
        Callback to update the fish camera image on the GUI.
        """
        if self.__fish_image is None:
            return

        if self.__show_direction_rb.isChecked() and self.__fish_state is not None or self.__blocked_directions is not None:
            frame = self.__fish_image.copy()
        else:
            frame = self.__fish_image

        if self.__blocked_directions is not None:
            height, width, _ = frame.shape
            for angle in self.__blocked_directions:
                # Calculate the endpoint of the line based on the angle
                radians = math.radians(angle)
                x_end = int(width / 2 + (width / 2) * math.cos(radians))
                y_end = int(height / 2 - (height / 2) * math.sin(radians))
                # Draw the red line on the border
                cv2.line(frame, (width // 2, height // 2), (x_end, y_end), (0, 0, 255), 2)

        # 3) draw direction overlay if requested and we have a Twist state
        if self.__show_direction_rb.isChecked() and self.__fish_state is not None:
            # extract position
            px = int(self.__fish_state.linear.x)
            py = int(self.__fish_state.linear.y)

            # extract direction vector
            dx = self.__fish_state.angular.x
            dy = self.__fish_state.angular.y

            # compute heading angle in degrees
            angle = math.degrees(math.atan2(dy, dx))

            # draw a red circle at (px,py)
            cv2.circle(frame, (px, py), 5, (0, 0, 255), -1)

            # draw the angle text
            cv2.putText(frame,
                        f"Dir: {angle:.1f}°",
                        (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA)

            # draw an arrow showing the heading
            length = 30
            ex = px + int(length * math.cos(math.radians(angle)))
            ey = py - int(length * math.sin(math.radians(angle)))
            cv2.arrowedLine(frame,
                            (px, py),
                            (ex, ey),
                            (0, 255, 0),
                            2,
                            tipLength=0.3)

        # 4) convert BGR → RGB and wrap in QImage
        h, w, ch = frame.shape
        # rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # 5) scale & display
        pix = QPixmap.fromImage(qimg).scaled(
            self.__left_image_frame.width(),
            self.__left_image_frame.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.__left_image_frame.setPixmap(pix)

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
            x = np.clip(self.__foma_world_location.x, 0, 500 - 1)
            y = np.clip(self.__foma_world_location.y, 0, 500 - 1)
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
                self.__feed_loading_button.setDisabled(False)
                self.__feed = rospy.ServiceProxy('fish_feeder/feed', Trigger)

        if not (self.__dim_lights is None) ^ is_service_available('light_dimmer/change'):
            if self.__dim_lights:
                self.logerr("Light dimming service unavailable - disabling buttons")
                self.__lights_slider.setDisabled(True)
                self.__dim_lights = None
            else:
                self.loginfo("Light dimming service available - enabling buttons")
                self.__lights_slider.setDisabled(False)
                self.__dim_lights = rospy.ServiceProxy('light_dimmer/change', Light)

        if not (self.__motor_control_system_check is None) ^ is_service_available('motor_control/system_check'):
            if self.__motor_control_system_check:
                self.logerr("Manual control service unavailable - disabling buttons")
                self.__manual_control_button.setDisabled(True)
                self.__motor_control_system_check = None
            else:
                self.loginfo("Manual control service available - enabling buttons")
                self.__manual_control_button.setDisabled(False)
                self.__motor_control_system_check = rospy.ServiceProxy('motor_control/system_check', Check)

        if not (self.__bypass_lidar is None) ^ is_service_available('motor_control/bypass_lidar'):
            if self.__bypass_lidar:
                self.logerr("Manual control service unavailable - disabling buttons")
                self.__manual_control_button.setDisabled(True)
                self.__bypass_lidar = None
            else:
                self.loginfo("Manual control service available - enabling buttons")
                self.__manual_control_button.setDisabled(False)
                self.__bypass_lidar = rospy.ServiceProxy('motor_control/bypass_lidar', SetBool)

        # if not (self.__motor_set_speed is None) ^ is_service_available('motor_control/set_speed'):
        #     if self.__motor_set_speed:
        #         self.logerr("Manual control service unavailable - disabling buttons")
        #         self.__manual_control_button.setDisabled(True)
        #         self.__motor_set_speed = None
        #     else:
        #         self.loginfo("Manual control service available - enabling buttons")
        #         self.__manual_control_button.setDisabled(False)
        #         self.__motor_set_speed = rospy.ServiceProxy('motor_control/system_check', Check)
        
        # self.fish_camera_control = rospy.ServiceProxy('fish_camera/system_toggle', SetBool)
        # self.ceiling_cameras_control = rospy.ServiceProxy('ceiling_cameras/system_toggle', SetBool)
        # self.lidar_control = rospy.ServiceProxy('lidar/system_toggle', SetBool)
        # self.fish_detection_control = rospy.ServiceProxy('fish_detection/system_toggle', SetBool)
        # self.light_dimmer_control = rospy.ServiceProxy('light_dimmer/system_toggle', SetBool)
        # self.motor_control = rospy.ServiceProxy('motor_control/system_toggle', SetBool)

    def __update_gui(self):
        self.__update_left_display()
        self.__update_right_display()

    def __on_start_click(self):
        self.__update_buttons_state((True,False,True,True))

        self.__ongoing_trial = True
        
        self.__trial_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
   
        # Ensure the output directory exists
        self.__trial_output_folder = os.path.join(os.path.expanduser(self.__output_folder), self.__trial_timestamp)
        if not os.path.exists(self.__trial_output_folder):
            self.loginfo(f"Creating output folder {self.__trial_output_folder}")
            os.makedirs(self.__trial_output_folder)

        # 1. FOMA Location
        # Generate file name with current timestamp (YYYYMMDDHHMM)

        foma_location_filename = os.path.join(self.__trial_output_folder, f"foma_location.csv")

        # Open CSV file in append mode and create writer
        self.__foma_location_file = open(foma_location_filename, 'a', newline='')
        self.__foma_location_csv_writer = csv.writer(self.__foma_location_file)

        # Write header if the file is new
        if os.stat(foma_location_filename).st_size == 0:
            self.__foma_location_csv_writer.writerow(["time", "x_w", "y_w", "x_i", "y_i"])
            self.__foma_location_file.flush()  # Ensure the header is written immediately

        # Fish Location
        fish_location_filename = os.path.join(self.__trial_output_folder, f"fish_location.csv")

        # Open CSV file in append mode and create writer
        self.__fish_location_file = open(fish_location_filename, 'a', newline='')
        self.__fish_location_csv_writer = csv.writer(self.__fish_location_file)

        # Write header if the file is new
        if os.stat(fish_location_filename).st_size == 0:
            self.__fish_location_csv_writer.writerow(["time", "x", "y", "angle"])
            self.__fish_location_file.flush()  # Ensure the header is written immediately

        # 2. Room Video
        room_video_filename = os.path.join(self.__trial_output_folder, f"room_video.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # MP4 format
        room_frame_width = 1024  # Adjust based on your camera resolution
        room_frame_height = 768
        room_fps = 10  # Default FPS (adjust based on the camera FPS)
        self.__room_video_writer = cv2.VideoWriter(room_video_filename, fourcc, room_fps, (room_frame_width, room_frame_height))
        # if self.__room_video_writer.isOpened():
        #     self.loginfo(f"Room video writer opened with filename: {room_video_filename}")


        # 3. FOMA Video
        foma_video_filename = os.path.join(self.__trial_output_folder, f"foma_video.mp4")
        # fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # MP4 format
        foma_frame_width = 960  # Adjust based on your camera resolution
        foma_frame_height = 720
        foma_fps = 10  # Default FPS (adjust based on the camera FPS)
        self.__foma_video_writer = cv2.VideoWriter(foma_video_filename, fourcc, foma_fps, (foma_frame_width, foma_frame_height))

        # 4. Room Map
        # Create a white image representing the room
        self.__room_map = np.ones((500, 500, 3), dtype=np.uint8) * 255

        self.__writers_timer.start(25)

    def __on_stop_click(self):
        self.__update_buttons_state((True,True,False,False))
        self.__ongoing_trial = False
        self.__motor_control_vector.publish(Vector3(0, 0, 0))
        self.__writers_timer.stop()
        self.__close_file_writers()

    def __on_reset_click(self):
        self.__update_buttons_state((False,True,True,False))
        self.__close_file_writers()

    def __on_close_click(self, event):
        self.__close_file_writers()
        QApplication.quit()
        rospy.signal_shutdown("Closing GUI")

    def __close_file_writers(self):
        if self.__foma_location_file:
            self.__foma_location_file.close()
            self.__foma_location_file = None
            self.__foma_location_csv_writer = None
        if self.__room_video_writer:
            self.__room_video_writer.release()
            self.__room_video_writer = None
        if self.__foma_video_writer:
            self.__foma_video_writer.release()
            self.__foma_video_writer = None
        if self.__room_map is not None:
            cv2.imwrite(os.path.join(self.__trial_output_folder, f"room_map_{self.__trial_timestamp}.png"), self.__room_map)

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

    def __write_files(self):
        timestamp = rospy.get_time()  # ROS time in seconds

        if self.__foma_img_location is not None and self.__foma_world_location is not None:
            self.__foma_location_csv_writer.writerow([timestamp, self.__foma_world_location.x, self.__foma_world_location.y, self.__foma_img_location.x, self.__foma_img_location.y])
            self.__foma_location_file.flush()  # Ensure data is written to the file

        if self.__fish_state is not None:
            self.__fish_location_csv_writer.writerow([
                timestamp,
                self.__fish_state.linear.x,
                self.__fish_state.linear.y,
                math.degrees(math.atan2(self.__fish_state.angular.y, self.__fish_state.angular.x))
            ])
            self.__fish_location_file.flush()  # Ensure data is written to the file

        if self.__room_video_writer and self.__room_image is not None:
            frame = cv2.cvtColor(self.__room_image, cv2.COLOR_RGB2BGR)
            self.__room_video_writer.write(frame)

        if self.__foma_video_writer and self.__fish_image is not None:
            frame = cv2.cvtColor(self.__fish_image, cv2.COLOR_RGB2BGR)
            self.__foma_video_writer.write(frame)

    def resizeEvent(self, event):
        if self.__top_right_image.pixmap():
            scaled_pixmap = self.__top_right_image.pixmap().scaled(
                self.__top_right_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.__top_right_image.setPixmap(scaled_pixmap)
        if self.__left_image_frame.pixmap():
            scaled_pixmap = self.__left_image_frame.pixmap().scaled(
                self.__left_image_frame.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.__left_image_frame.setPixmap(scaled_pixmap)
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
        rospy.loginfo(f"GUI Node: {msg}")


if __name__ == "__main__":
    rospy.init_node('gui_node')
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    app.exec()

    rospy.spin()
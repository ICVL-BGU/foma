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
import threading, time

# PyQt5 imports
from PyQt5.QtCore import Qt, QTimer, QEvent, pyqtSignal
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
    QInputDialog,
    )

# ROS imports
from geometry_msgs.msg import Twist, Vector3, TwistStamped
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, Int16MultiArray
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge, CvBridgeError

# Custom ROS messages
from foma.srv import Light, Check, Write
from foma.msg import FomaLocation

class MainWindow(QMainWindow):
    fish_frame_ready = pyqtSignal(np.ndarray)
    room_frame_ready = pyqtSignal(np.ndarray)
    services_updated = pyqtSignal(dict)

    def __init__(self):
        super(MainWindow, self).__init__()

        # Rospy configs
        self.__init_subscriptions_and_services()
        self.bridge = CvBridge()

        self.setWindowTitle("Trial control")
        self.drag_start = self.pos()
        self.closeEvent = self.__on_close_click
        
        self.__init_attributes()
        self.__init_widgets()
        self.__init_layouts()
        self.__init_signals()
        self.__init_service_checker()
        self.__init_timers()

    def __init_signals(self):
        self.fish_frame_ready.connect(self.__update_left_display)
        self.room_frame_ready.connect(self.__update_right_display)
        self.services_updated.connect(self.__update_services)

    def __init_attributes(self):
        # Images and locations
        self.__foma_img_location = None
        self.__foma_world_location = None
        self.__room_image = None
        self.__room_map = None
        self.__fish_state = None
        self.__fish_image = None

        # Camera frame dimensions
        self.__room_frame_shape = (2560, 2560)
        self.__map_frame_shape = (1000, 1000)

        # Windows
        self.__manual_control_window = None
        self.__feeding_load_window = None

        # Timers
        self.__image_timer = None
        self.__services_timer = None
        # self.__writers_timer = None

        # Motor Control
        self.__linear_velocity = Twist()
        self.__angular_velocity = Float32()
        self.__foma_speed = Twist()
        self.__direction_epsilon = 45
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
        self.__BL_layout.addWidget(self.__fish_direction_group, 0, 3, 2, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__foma_direction_group, 0, 4, 2, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__room_display_group, 0, 5, 2, 1, alignment=Qt.AlignCenter)
        
        self.__BL_widget = QFrame()
        self.__BL_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__BL_widget.setLineWidth(2)
        self.__BL_widget.setLayout(self.__BL_layout)

        # Bottom-Right (Control Buttons)
        self.__BR_layout = QGridLayout()
        self.__BR_layout.addWidget(self.__start_button, 0, 0, alignment=Qt.AlignCenter)
        self.__BR_layout.addWidget(self.__pause_button, 0, 1, alignment=Qt.AlignCenter)
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
        self.__pause_button = QPushButton()
        self.__pause_button.setText("Pause")
        self.__pause_button.setDisabled(True)
        self.__pause_button.setMaximumHeight(50)
        self.__pause_button.clicked.connect(self.__on_pause_click)

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
        self.__lights_slider.setDisabled(True)
        self.__lights_slider.setValue(1)

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
        self.__show_fish_direction_rb = QRadioButton("Yes")
        self.__hide_fish_direction_rb = QRadioButton("No")
        self.__show_fish_direction_rb.setChecked(True)

        # Create and add radio buttons to layout
        fish_direction_layout = QGridLayout()
        fish_direction_layout.addWidget(self.__show_fish_direction_rb, 0, 0, alignment=Qt.AlignCenter)
        fish_direction_layout.addWidget(self.__hide_fish_direction_rb, 1, 0, alignment=Qt.AlignCenter)
        
        # Create a group box for the radio buttons and set the layout
        self.__fish_direction_group = QGroupBox("Display Fish Direction")
        self.__fish_direction_group.setLayout(fish_direction_layout)

        # Add radio buttons for toggling direction display
        self.__show_foma_direction_rb = QRadioButton("Yes")
        self.__hide_foma_direction_rb = QRadioButton("No")
        self.__show_foma_direction_rb.setChecked(True)

        # Create and add radio buttons to layout
        foma_direction_layout = QGridLayout()
        foma_direction_layout.addWidget(self.__show_foma_direction_rb, 0, 0, alignment=Qt.AlignCenter)
        foma_direction_layout.addWidget(self.__hide_foma_direction_rb, 1, 0, alignment=Qt.AlignCenter)
        
        # Create a group box for the radio buttons and set the layout
        self.__foma_direction_group = QGroupBox("Display FOMA Direction")
        self.__foma_direction_group.setLayout(foma_direction_layout)

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
        rospy.Subscriber('fish_camera/image', CompressedImage, self.__update_fish_image)
        rospy.Subscriber('fish_detection/state', TwistStamped, self.__update_fish_state)
        rospy.Subscriber('ceiling_camera/image', Image, self.__update_room_image)
        rospy.Subscriber('localization/location', FomaLocation, self.__update_foma_location)
        rospy.Subscriber('motor_control/blocked', Int16MultiArray, self.__update_blocked_directions)
        rospy.Subscriber('motor_control/speed', Twist, self.__update_foma_speed)
        self.__motor_control_twist = rospy.Publisher('motor_control/twist', Twist, queue_size=10)
        self.__motor_control_dir = rospy.Publisher('motor_control/angle', Float32, queue_size=10)
        self.__motor_control_vector = rospy.Publisher('motor_control/vector', Vector3, queue_size=10)
        self.__motor_control_rotate = rospy.Publisher('motor_control/rotate', Float32, queue_size=10)
        self.__motor_set_speed = rospy.Publisher('motor_control/set_speed', Float32, queue_size=10)
        self.__writer_control = rospy.ServiceProxy('writer_node/write', Write)
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
            key = event.key()
            # Map numpad keys and +/-
            key_to_angle = {
                Qt.Key_8: 0,
                Qt.Key_2: 180,
                Qt.Key_4: 90,
                Qt.Key_6: 270,
                Qt.Key_7: 45,
                Qt.Key_9: 315,
                Qt.Key_1: 135,
                Qt.Key_3: 225,
                Qt.Key_Plus: -2,  # ccw
                Qt.Key_Minus: -1  # cw
            }
            if key in key_to_angle:
                if event.type() == QEvent.KeyPress:
                    angle = key_to_angle[key]
                    self.__update_velocity(angle, True)
                    event.accept()
                elif event.type() == QEvent.KeyRelease:
                    self.__update_velocity(0, False)
                    event.accept()
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
        forward_left_button = QPushButton("↖")
        forward_right_button = QPushButton("↗")
        backward_left_button = QPushButton("↙")
        backward_right_button = QPushButton("↘")
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
        
        control_layout.addWidget(forward_left_button, 0, 0)
        control_layout.addWidget(forward_button, 0, 1, 1, 2)
        control_layout.addWidget(forward_right_button, 0, 3)
        control_layout.addWidget(left_button, 1, 0)
        control_layout.addWidget(cw_button, 1, 1)
        control_layout.addWidget(ccw_button, 1, 2)
        control_layout.addWidget(right_button, 1, 3)
        control_layout.addWidget(backward_left_button, 2, 0)
        control_layout.addWidget(backward_button, 2, 1, 1, 2)
        control_layout.addWidget(backward_right_button, 2, 3)
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
        forward_left_button.pressed.connect(lambda: self.__update_velocity(45, True))
        forward_left_button.released.connect(lambda: self.__update_velocity(45, False))
        forward_right_button.pressed.connect(lambda: self.__update_velocity(315, True))
        forward_right_button.released.connect(lambda: self.__update_velocity(315, False))
        backward_left_button.pressed.connect(lambda: self.__update_velocity(135, True))
        backward_left_button.released.connect(lambda: self.__update_velocity(135, False))
        backward_right_button.pressed.connect(lambda: self.__update_velocity(225, True))
        backward_right_button.released.connect(lambda: self.__update_velocity(225, False))
        cw_button.pressed.connect(lambda: self.__update_velocity(-1, True))
        cw_button.released.connect(lambda: self.__update_velocity(-1, False))
        ccw_button.pressed.connect(lambda: self.__update_velocity(-2, True))
        ccw_button.released.connect(lambda: self.__update_velocity(-2, False))
        speed_control_button.clicked.connect(set_speed)

        bypass_lidar_checkbox.stateChanged.connect(lambda state: self.__bypass_lidar(state == Qt.Checked))

        self.__manual_control_window.setLayout(control_layout)
        
        self.__velocity_timer = QTimer(self)
        self.__velocity_timer.timeout.connect(self.__publish_velocity)
        self.__velocity_timer.start(50)  # Call every 100ms

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

    def __init_service_checker(self):
        def checker_loop():
            # list of tuples: (attr_name, ros_service_name, srv_type)
            services = [
                ('__feed',                    'fish_feeder/feed',        Trigger),
                ('__dim_lights',              'light_dimmer/change',     Light),
                ('__motor_control_system_check','motor_control/system_check', Check),
                ('__bypass_lidar',            'motor_control/bypass_lidar', SetBool),
            ]
            while not rospy.is_shutdown():
                status = {}
                for attr, name, srv_type in services:
                    try:
                        # quick timeout so we don’t block for a full second
                        rospy.wait_for_service(name, timeout=0.2)
                        status[attr] = rospy.ServiceProxy(name, srv_type)
                    except Exception:
                        status[attr] = None
                # emit back to the GUI
                self.services_updated.emit(status)
                time.sleep(1)

        t = threading.Thread(target=checker_loop, daemon=True)
        t.start()

    def __update_velocity(self, direction: int, is_pressed: bool):
        """
        Update robot velocity based on button presses.
        """
        if direction >= 0:
            # self.__linear_velocity = Twist()
            # Convert direction (degrees) to radians for vector calculation
            radians = math.radians(direction)
            if is_pressed:
                self.__linear_velocity.linear.x = -math.sin(radians)
                self.__linear_velocity.linear.y = math.cos(radians)
            else:
                self.__linear_velocity.linear.x = 0
                self.__linear_velocity.linear.y = 0
  
        else:
            # self.__angular_velocity = Float32()
            if direction == -1: # "cw"
                self.__angular_velocity.data = 1 if is_pressed else 0
            elif direction == -2: # "ccw":
                self.__angular_velocity.data = -1 if is_pressed else 0
            # Publish the velocity to the robot
            self.__motor_control_rotate.publish(self.__angular_velocity)

    def __publish_velocity(self):
        self.__motor_control_twist.publish(self.__linear_velocity)

    def __update_fish_image(self, img_msg: CompressedImage):
        try:
            self.__fish_image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
            self.fish_frame_ready.emit(self.__fish_image.copy())
        except CvBridgeError as e:
            self.logwarn(e)

    def __update_fish_state(self, state_msg: TwistStamped):
        state = state_msg.twist
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
            # 1. compute the pixel‐offset with y inverted
            vector_x = fish_x - center_x
            vector_y = center_y - fish_y   # <— flip the sign here!

            # 2. get the two angles
            center_to_fish_angle = math.degrees(math.atan2(vector_y, vector_x))

            direction_angle = math.degrees(
                math.atan2(state.angular.y,   # note: angular.y is the ‘up/down’ component
                        state.angular.x)   #        angular.x is the ‘left/right’ component
            )

            # 3. normalize difference into [–180, +180]
            def shortest_angle_diff(a, b):
                """Return the signed smallest difference a–b in degrees."""
                d = (a - b + 180) % 360 - 180
                return d

            diff = shortest_angle_diff(direction_angle, center_to_fish_angle)

            if abs(diff) <= self.__direction_epsilon:
                self.__motor_control_dir.publish(direction_angle)
            else:
                self.__motor_control_vector.publish(Vector3(0, 0, 0))

    def __update_room_image(self, img_msg: Image):
        try:
            self.__room_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8") # was passthrough
            frame = self.__room_image.copy() # might not be correct
            if self.__foma_img_location:
                center_x = self.__foma_img_location.x
                center_y = self.__foma_img_location.y
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
            self.room_frame_ready.emit(frame)
        except CvBridgeError as e:
            self.logwarn(f"Error converting image message: {e}")
        except Exception as e:
            self.logwarn(f"Unexpected error in update_room_image: {e}")

    def __update_foma_location(self, location: FomaLocation):
        # Convert normalized locations (0..1) to pixel coordinates
        self.__foma_img_location = Vector3(
            location.image.x * self.__room_frame_shape[1],
            location.image.y * self.__room_frame_shape[0],
            0
        )
        self.__foma_world_location = Vector3(
            location.world.x * self.__map_frame_shape[0],
            location.world.y * self.__map_frame_shape[1],
            0
        )
        
        if self.__room_map is not None:
            # Ensure coordinates stay within bounds
            x = np.clip(self.__foma_world_location.x, 0, self.__room_frame_shape[1] - 1).astype(int)
            y = np.clip(self.__foma_world_location.y, 0, self.__room_frame_shape[0] - 1).astype(int)
            cv2.circle(self.__room_map, (x, y), 5, (0, 255, 0), -1)

    def __update_foma_speed(self, speed: Twist):
        self.__foma_speed = speed

    def __update_blocked_directions(self, blocked: Int16MultiArray):
        """
        Update the blocked directions based on the received data.
        """
        self.__blocked_directions = blocked.data
        # self.loginfo(f"Blocked directions: {self.__blocked_directions}")

    def __update_left_display(self, frame: np.ndarray):
        # 1) nothing to do if no frame
        if frame is None:
            return

        h, w, ch = frame.shape
        
        if self.__blocked_directions is not None:
            for angle in self.__blocked_directions:
                cx, cy = w // 2, h // 2
                delta_deg = 0.5
                # define the two boundary angles
                angles = (angle - delta_deg, angle + delta_deg)
                pts = []

                for ang in angles:
                    # remap so that 0° = up, CCW positive
                    math_ang = math.radians(ang + 90)
                    dx = math.cos(math_ang)
                    dy = -math.sin(math_ang)

                    # collect positive t‘s for intersection with each border
                    ts = []

                    # vertical borders x=0 and x=width
                    if abs(dx) > 1e-6:
                        t1 = (0 - cx) / dx
                        t2 = (w - cx) / dx
                        if t1 > 0: ts.append(t1)
                        if t2 > 0: ts.append(t2)

                    # horizontal borders y=0 and y=height
                    if abs(dy) > 1e-6:
                        t3 = (0 - cy) / dy
                        t4 = (h - cy) / dy
                        if t3 > 0: ts.append(t3)
                        if t4 > 0: ts.append(t4)

                    if not ts:
                        # ray is parallel to all borders?
                        continue

                    t_min = min(ts)
                    x_edge = int(cx + t_min * dx)
                    y_edge = int(cy + t_min * dy)
                    pts.append((x_edge, y_edge))

                # draw either the segment on the border or a dot if degenerate
                if len(pts) == 2:
                    cv2.line(frame, pts[0], pts[1], (255, 0, 0), 5)
                elif len(pts) == 1:
                    cv2.circle(frame, pts[0], 3, (255, 0, 0), -1)

        # 3) draw direction overlay if requested and we have a Twist state
        if self.__show_fish_direction_rb.isChecked() and self.__fish_state is not None:
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

        # Draw FOMA direction overlay if requested and we have a Twist state
        if self.__show_foma_direction_rb.isChecked() and self.__foma_speed is not None:
            # Extract linear and angular speeds
            linear_speed = math.sqrt(
                self.__foma_speed.linear.x**2 + self.__foma_speed.linear.y**2
            )
            angular_speed = self.__foma_speed.angular.z

            center_x, center_y = w // 2, h // 2

            # Draw linear speed arrow if linear part is not 0
            if linear_speed > 0:
            # Calculate direction angle
                angle = math.degrees(math.atan2(self.__foma_speed.linear.y, self.__foma_speed.linear.x))

                # Calculate arrow endpoint
                arrow_length = 30  # Length of the arrow
                end_x = int(center_x + arrow_length * math.cos(math.radians(angle)))
                end_y = int(center_y - arrow_length * math.sin(math.radians(angle)))

                # Draw the arrow
                cv2.arrowedLine(frame, (center_x, center_y), (end_x, end_y), (0, 255, 255), 2, tipLength=0.3)

                # Draw the speed text
                cv2.putText(frame,
                        f"{linear_speed:.2f}",
                        (end_x + 10, end_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA)

            # Draw angular speed arrow if angular part is not 0
            if angular_speed != 0:
            # Define rotation arrow parameters
                radius = 30
                start_angle = 0
                end_angle = 270 if angular_speed > 0 else -270
                color = (255, 255, 0)

                # Draw the rotation arrow
                cv2.ellipse(frame, (center_x, center_y), (radius, radius), 0, start_angle, end_angle, color, 2)

                # Draw the angular speed text
                cv2.putText(frame,
                        f"{angular_speed:.2f}",
                        (center_x + radius + 10, center_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA)
                
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

    def __update_right_display(self, frame: np.ndarray):
        """
        Callback to update the room camera image on the GUI only ATM.
        """
        if self.__room_camera_display_rb.isChecked() and frame is not None:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            frame = frame.data

        elif self.__map_display_rb.isChecked() and self.__room_map is not None:
            height, width, channel = self.__room_map.shape
            bytes_per_line = 3 * width
            map_frame = self.__room_map.copy()
            # Ensure coordinates stay within bounds
            if self.__foma_world_location is not None:
                x = np.clip(self.__foma_world_location.x, 0, self.__room_frame_shape[1] - 1).astype(int)
                y = np.clip(self.__foma_world_location.y, 0, self.__room_frame_shape[0] - 1).astype(int)
                cv2.circle(map_frame, (x, y), 5, (0, 0, 255), -1)
            frame = map_frame.data

        if frame is not None:
            q_image = QImage(frame, width, height, bytes_per_line, QImage.Format_RGB888)             
            # Scale the image to fit the QLabel
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaled(
                self.__top_right_image.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            # Update the QLabel with the scaled QPixmap
            self.__top_right_image.setPixmap(scaled_pixmap)

    def __update_services(self, status: dict):
        """
        status is a dict mapping attribute names to either a ServiceProxy or None:
          {
            '__feed':            <Trigger proxy> or None,
            '__dim_lights':      <Light proxy>   or None,
            '__motor_control_system_check': <Check proxy> or None,
            '__bypass_lidar':    <SetBool proxy> or None,
          }
        """
        # 1) fish_feeder/feed
        feed_proxy = status.get('__feed')
        # became available?
        if self.__feed is None and feed_proxy is not None:
            self.loginfo("Feeder service available - enabling buttons")
            self.__feed_button.setDisabled(False)
            self.__feed_loading_button.setDisabled(False)
            self.__feed = feed_proxy
        # went away?
        elif self.__feed is not None and feed_proxy is None:
            self.logerr("Feeder service unavailable - disabling buttons")
            self.__feed_button.setDisabled(True)
            self.__feed_loading_button.setDisabled(True)
            self.__feed = None

        # 2) light_dimmer/change
        lights_proxy = status.get('__dim_lights')
        if self.__dim_lights is None and lights_proxy is not None:
            self.loginfo("Light dimming service available - enabling slider")
            self.__lights_slider.setDisabled(False)
            self.__dim_lights = lights_proxy
            self.__lights_slider.valueChanged.connect(lambda val:self.__dim_lights(int(255*val/self.__lights_slider.maximum())))
        elif self.__dim_lights is not None and lights_proxy is None:
            self.logerr("Light dimming service unavailable - disabling slider")
            self.__lights_slider.setDisabled(True)
            self.__dim_lights = None

        # 3) motor_control/system_check
        motor_check_proxy = status.get('__motor_control_system_check')
        if self.__motor_control_system_check is None and motor_check_proxy is not None:
            self.loginfo("Manual control service available - enabling button")
            self.__manual_control_button.setDisabled(False)
            self.__motor_control_system_check = motor_check_proxy
        elif self.__motor_control_system_check is not None and motor_check_proxy is None:
            self.logerr("Manual control service unavailable - disabling button")
            self.__manual_control_button.setDisabled(True)
            self.__motor_control_system_check = None

        # 4) motor_control/bypass_lidar
        bypass_proxy = status.get('__bypass_lidar')
        if self.__bypass_lidar is None and bypass_proxy is not None:
            self.loginfo("Bypass‐LIDAR service available - enabling button")
            self.__manual_control_button.setDisabled(False)
            self.__bypass_lidar = bypass_proxy
        elif self.__bypass_lidar is not None and bypass_proxy is None:
            self.logerr("Bypass‐LIDAR service unavailable - disabling button")
            self.__manual_control_button.setDisabled(True)
            self.__bypass_lidar = None

    def __on_start_click(self):
        subject_id, ok = QInputDialog.getText(
            self,
            "Subject ID",
            "Please enter subject ID:"
        )
        if not ok or not subject_id:
            return
        
        self.__start_button.setDisabled(True)
        self.__pause_button.setDisabled(False)
        self.__reset_button.setDisabled(True)
        self.__close_button.setDisabled(True)

        self.__ongoing_trial = True
        
        self.__writer_control("start", subject_id, rospy.Time.now())

    def __on_continue_click(self):
        self.__start_button.setDisabled(True)
        self.__pause_button.setDisabled(False)
        self.__reset_button.setDisabled(True)
        self.__close_button.setDisabled(True)

        self.__ongoing_trial = True
        
    def __on_pause_click(self):
        self.__start_button.setDisabled(False)
        self.__pause_button.setDisabled(True)
        self.__reset_button.setDisabled(False)
        self.__close_button.setDisabled(False)

        self.__start_button.setText("Continue")
        self.__start_button.clicked.disconnect()
        self.__start_button.clicked.connect(self.__on_continue_click)

        self.__ongoing_trial = False
        self.__motor_control_vector.publish(Vector3(0, 0, 0))
        
    def __on_reset_click(self):
        self.__start_button.setDisabled(False)
        self.__pause_button.setDisabled(True)
        self.__reset_button.setDisabled(True)
        self.__close_button.setDisabled(False)

        self.__start_button.setText("Start")
        self.__start_button.clicked.disconnect()
        self.__start_button.clicked.connect(self.__on_start_click)

        self.__ongoing_trial = False
        self.__writer_control("stop", None, rospy.Time.now())

    def __on_close_click(self, event):
        self.__writer_control("stop", None, rospy.Time.now())
        QApplication.quit()
        rospy.signal_shutdown("Closing GUI")

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
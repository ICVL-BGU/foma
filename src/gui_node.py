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
    QMessageBox,
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
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import Float32, Int16MultiArray, Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, SetBool
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool


# Custom ROS messages
from foma.srv import Check, Write, Float, String
from foma.msg import FomaLocation
from etc.settings import *

class MainWindow(QMainWindow):
    fish_frame_ready = pyqtSignal(np.ndarray)
    room_frame_ready = pyqtSignal(np.ndarray)
    services_updated = pyqtSignal(dict)
    go_home_state_signal = pyqtSignal(bool)

    def __init__(self):
        super(MainWindow, self).__init__()

        # Rospy configs
        self.__init_subscriptions_and_services()
        self.bridge = CvBridge()

        self.setWindowTitle("FOMA Trial control")
        self.drag_start = self.pos()
        self.closeEvent = self.__on_close_click
        
        self.__init_attributes()
        self.__init_widgets()
        self.__init_layouts()
        self.__init_signals()
        self.__init_service_checker()

    def __init_signals(self):
        self.fish_frame_ready.connect(self.__update_left_display)
        self.room_frame_ready.connect(self.__update_right_display)
        self.services_updated.connect(self.__update_services)
        self.go_home_state_signal.connect(self.__update_go_home_state)

    def __init_attributes(self):
        # Images and locations
        self.__foma_img_location = None
        self.__foma_world_location = None
        self.__room_image = None
        self.__room_map = np.ones((ROOM_MAP_FRAME_SHAPE[1], ROOM_MAP_FRAME_SHAPE[0], 3), dtype=np.uint8) * 255
        self.__fish_state = None
        self.__fish_image = None

        # Windows
        self.__manual_control_window = None
        self.__feeding_load_window = None

        # Rotation control (closed-loop via LiDAR scan-matching)
        self.__lidar_ranges = None
        self.__rotation_target = None
        self.__rotation_prev_scan = None
        self.__rotation_accumulated = 0.0
        self.__rotation_control_timer = None

        # Motor Control
        self.__velocity = Twist()
        self.__foma_speed = Twist()
        self.__blocked_directions = None

        # Services
        self.__motor_control_system_check = None
        self.__dim_lights = None
        self.__feed = None
        self.__empty_feeder = None
        self.__bypass_lidar = None
        self.__go_home_enable = None
        self.__go_home_active = False
        self.__motor_set_speed = None
        self.__motor_set_mode = None

        # Trial Control
        self.__ongoing_trial = False
        self.__session_folder = None
        self.__current_trial = 0
        self.__event_log_file = None
        self.__event_log_writer = None

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
        self.__BL_layout.addWidget(self.__feed_button, 0, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__feed_loading_button, 1, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__manual_control_button, 0, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__go_home_button, 1, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__rotate_button, 2, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__show_fish_direction_cb, 0, 3, 1, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__show_foma_direction_cb, 1, 3, 1, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__room_display_group, 0, 4, 2, 1, alignment=Qt.AlignCenter)
        
        self.__BL_widget = QFrame()
        self.__BL_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__BL_widget.setLineWidth(2)
        self.__BL_widget.setLayout(self.__BL_layout)

        # Bottom-Center (Display)
        self.__BC_layout = QGridLayout()
        self.__BC_layout.addWidget(self.__subject_id_text_label, 0, 0, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__subject_id_value_label, 0, 1, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__trial_num_text_label, 1, 0, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__trial_num_value_label, 1, 1, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__times_fed_text_label, 2, 0, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__times_fed_value_label, 2, 1, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__foma_img_position_text_label, 3, 0, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__foma_img_position_value_label, 3, 1, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__foma_room_position_text_label, 4, 0, alignment=Qt.AlignCenter)
        self.__BC_layout.addWidget(self.__foma_room_position_value_label, 4, 1, alignment=Qt.AlignCenter)
        
        self.__BC_widget = QFrame()
        self.__BC_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__BC_widget.setLineWidth(2)
        self.__BC_widget.setLayout(self.__BC_layout)

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
        main_layout.addWidget(self.__TL_widget, 0, 0, 1, 2)
        main_layout.addWidget(self.__TR_widget, 0, 2, 1, 2)
        main_layout.addWidget(self.__BL_widget, 1, 0, 1, 2)
        main_layout.addWidget(self.__BC_widget, 1, 2, 1, 1)
        main_layout.addWidget(self.__BR_widget, 1, 3, 1, 1)

        # ensure all 4 columns participate in sizing
        for i in range(4):
            main_layout.setColumnStretch(i, 1)

        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setRowStretch(0, 5)
        main_layout.setRowStretch(1, 1)

        widget = QWidget()
        widget.setLayout(main_layout)
        self.setCentralWidget(widget)

    def __init_widgets(self):
        self.__init_settings_widgets()
        self.__init_data_widgets()
        self.__init_control_widgets()

    def __init_settings_widgets(self):
        # Feed button init
        self.__feed_button = QPushButton()
        self.__feed_button.setText("Feed")
        self.__feed_button.clicked.connect(lambda: self.__on_feed_click())
        self.__feed_button.setDisabled(True)

        # Lights slider init
        self.__lights_slider = QSlider(Qt.Horizontal)
        self.__lights_slider.setMinimum(0)
        self.__lights_slider.setMaximum(1)
        self.__lights_slider.setTickPosition(QSlider.TicksAbove|QSlider.TicksBelow)
        self.__lights_slider.setPageStep(1)
        self.__lights_slider.setMaximumHeight(50)
        self.__lights_slider.setDisabled(True)
        self.__lights_slider.setValue(0)

        # Lights slider label init
        self.__lights_label = QLabel("Lights dimming")
        font = self.__lights_label.font()
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
        self.__manual_control_label.setAlignment(Qt.AlignHCenter)

        # Go Home button init (main GUI)
        self.__go_home_button = QPushButton()
        self.__go_home_button.setText("Go Home")
        self.__go_home_button.setMaximumHeight(50)
        self.__go_home_button.clicked.connect(self.__start_go_home)
        self.__go_home_button.setDisabled(True)

        self.__feed_loading_button = QPushButton()
        self.__feed_loading_button.setText("Load Feeder")
        self.__feed_loading_button.setMaximumHeight(50)
        self.__feed_loading_button.clicked.connect(self.__init_feeding_load_window)
        self.__feed_loading_button.setDisabled(True)

        # Rotate button init
        self.__rotate_button = QPushButton()
        self.__rotate_button.setText("Rotate")
        self.__rotate_button.setMaximumHeight(50)
        self.__rotate_button.clicked.connect(self.__open_rotate_dialog)
        self.__rotate_button.setDisabled(True)

        # Fish image init
        self.__left_image_frame = QLabel()
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
        self.__show_fish_direction_cb = QCheckBox("Show Fish Direction")
        self.__show_fish_direction_cb.setChecked(True)

        # Add radio buttons for toggling direction display
        self.__show_foma_direction_cb = QCheckBox("Show FOMA Direction")
        self.__show_foma_direction_cb.setChecked(True)

        # Add radio buttons for toggling direction display
        self.__room_camera_display_rb = QRadioButton("Camera")
        self.__map_display_rb = QRadioButton("Map")
        self.__room_camera_display_rb.setChecked(True)

        # Create and add radio buttons to layout
        room_display_layout = QGridLayout()
        room_display_layout.addWidget(self.__room_camera_display_rb, 0, 0, alignment=Qt.AlignCenter)
        room_display_layout.addWidget(self.__map_display_rb, 0, 1, alignment=Qt.AlignCenter)
        
        # Create a group box for the radio buttons and set the layout
        self.__room_display_group = QGroupBox("Top-Right Display")
        self.__room_display_group.setLayout(room_display_layout)

    def __init_data_widgets(self):
        # Subject ID label init
        self.__subject_id_text_label = QLabel("Subject ID:")
        self.__subject_id_text_label.setAlignment(Qt.AlignHCenter)

        self.__subject_id_value_label = QLabel("N/A")
        self.__subject_id_value_label.setAlignment(Qt.AlignHCenter)

        # Trial number label init
        self.__trial_num_text_label = QLabel("Trial #:")
        self.__trial_num_text_label.setAlignment(Qt.AlignHCenter)

        self.__trial_num_value_label = QLabel("N/A")
        self.__trial_num_value_label.setAlignment(Qt.AlignHCenter)

        # Times fed label init
        self.__times_fed_text_label = QLabel("Times Fed:")
        self.__times_fed_text_label.setAlignment(Qt.AlignHCenter)

        self.__times_fed_value_label = QLineEdit("N/A")
        self.__times_fed_value_label.setAlignment(Qt.AlignHCenter)
        self.__times_fed_value_label.setValidator(QDoubleValidator(0, 9999, 0))
        self.__times_fed_value_label.editingFinished.connect(self.__on_times_fed_edited)

        # FOMA image position label init
        self.__foma_img_position_text_label = QLabel("FOMA Image Position(X,Y):")
        self.__foma_img_position_text_label.setAlignment(Qt.AlignHCenter)

        # FOMA position value label init
        self.__foma_img_position_value_label = QLabel("N/A")
        self.__foma_img_position_value_label.setAlignment(Qt.AlignHCenter)

        # FOMA room position label init
        self.__foma_room_position_text_label = QLabel("FOMA Room Position(X,Y):")
        self.__foma_room_position_text_label.setAlignment(Qt.AlignHCenter)

        # FOMA position value label init
        self.__foma_room_position_value_label = QLabel("N/A")
        self.__foma_room_position_value_label.setAlignment(Qt.AlignHCenter)

    def __init_control_widgets(self):
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

    def __init_subscriptions_and_services(self):
        rospy.Subscriber('fish_camera/image', CompressedImage, self.__update_fish_image, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('fish_detection/state', TwistStamped, self.__update_fish_state, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('ceiling_camera/image', CompressedImage, self.__update_room_image, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('localization/location', FomaLocation, self.__update_foma_location, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('lidar/blocked', Int16MultiArray, self.__update_blocked_directions, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('lidar/scans', LaserScan, self.__update_lidar_scan, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('motor_control/speed', TwistStamped, self.__update_foma_speed, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('go_home/enabled', Bool, self.__on_go_home_state, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('fish_feeder/feed_inc', TriggerResponse, self.__update_feeder_status, queue_size=1, tcp_nodelay=True)
        self.__motor_control_twist = rospy.Publisher('motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        self.__motor_control_dir = rospy.Publisher('motor_control/angle', Float32, queue_size=1, tcp_nodelay=True)
        self.__motor_control_vector = rospy.Publisher('motor_control/vector', Vector3, queue_size=1, tcp_nodelay=True)
        
        # Writer services - one for each writer node
        self.__writer_services = [
            rospy.ServiceProxy('csv_writer_foma_location/write', Write),
            rospy.ServiceProxy('csv_writer_fish_location/write', Write),
            rospy.ServiceProxy('csv_writer_foma_speed/write', Write),
            rospy.ServiceProxy('video_writer_room/write', Write),
            rospy.ServiceProxy('video_writer_foma/write', Write),
            rospy.ServiceProxy('video_writer_room_map/write', Write),
        ]
        
    def __init_manual_control_window(self):
        self.__motor_set_mode("manual")
        self.__motor_set_speed(1.0)
        self.__bypass_lidar(False)
        
        # Log manual control start
        self.__log_event("manual_control", "Manual control window opened")

        # [ADDED] Manual override always stops GoHome.
        if self.__go_home_enable is not None:
            self.__go_home_enable(False)

        # Log manual control start
        self.__log_event("manual_control", "Manual control window opened")

        self.__manual_control_window = QDialog(self)
        self.__manual_control_window.setWindowTitle("Manual Robot Control")
        self.__manual_control_window.setFixedSize(300, 300)
        self.__manual_control_window.setWindowModality(Qt.ApplicationModal)

        def on_close(event):
            if self.__ongoing_trial:
                self.__motor_set_mode("trial")
            else:
                self.__motor_set_mode("idle")
            self.__motor_control_twist.publish(Twist())
            self.__motor_set_speed(1.0)
            self.__bypass_lidar(False)
            self.__velocity_timer.stop()
            self.__velocity = Twist()
            
            # Log manual control end
            self.__log_event("manual_control", "Manual control window closed")

            self.__manual_control_window = None

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
                    self.__update_velocity(True, angle)
                    event.accept()
                elif event.type() == QEvent.KeyRelease:
                    self.__update_velocity(False)
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
        
        speed_control_textbox.setPlaceholderText("1.0")
        speed_control_textbox.setValidator(QDoubleValidator(0.0, 1.0, 2))
        def set_speed():
            try:
                speed = float(speed_control_textbox.text())
                self.loginfo(f"Speed set to: {speed}")
                self.__motor_set_speed(speed)
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

        forward_button.pressed.connect(lambda: self.__update_velocity(True,0))
        forward_button.released.connect(lambda: self.__update_velocity(False))
        backward_button.pressed.connect(lambda: self.__update_velocity(True, 180))
        backward_button.released.connect(lambda: self.__update_velocity(False))
        left_button.pressed.connect(lambda: self.__update_velocity(True, 90))
        left_button.released.connect(lambda: self.__update_velocity(False))
        right_button.pressed.connect(lambda: self.__update_velocity(True, 270))
        right_button.released.connect(lambda: self.__update_velocity(False))
        forward_left_button.pressed.connect(lambda: self.__update_velocity(True, 45))
        forward_left_button.released.connect(lambda: self.__update_velocity(False))
        forward_right_button.pressed.connect(lambda: self.__update_velocity(True, 315))
        forward_right_button.released.connect(lambda: self.__update_velocity(False))
        backward_left_button.pressed.connect(lambda: self.__update_velocity(True, 135))
        backward_left_button.released.connect(lambda: self.__update_velocity(False))
        backward_right_button.pressed.connect(lambda: self.__update_velocity(True, 225))
        backward_right_button.released.connect(lambda: self.__update_velocity(False))
        cw_button.pressed.connect(lambda: self.__update_velocity(True, -1))
        cw_button.released.connect(lambda: self.__update_velocity(False))
        ccw_button.pressed.connect(lambda: self.__update_velocity(True, -2))
        ccw_button.released.connect(lambda: self.__update_velocity(False))
        speed_control_button.clicked.connect(set_speed)

        bypass_lidar_checkbox.stateChanged.connect(lambda state: self.__bypass_lidar(state == Qt.Checked))
        
        self.__velocity_timer = QTimer(self)
        self.__velocity_timer.timeout.connect(self.__publish_velocity)
        self.__velocity_timer.start(50)  # Call every 100ms

        self.__manual_control_window.setLayout(control_layout)
        self.__manual_control_window.show()

    def __update_feeder_status(self, msg: TriggerResponse):
        if msg.success:
            try:
                times_fed = int(self.__times_fed_value_label.text()) if self.__times_fed_value_label.text() != "N/A" else 0
                times_fed += 1
                self.__times_fed_value_label.setText(str(times_fed))
            except ValueError:
                self.logwarn("Received invalid times fed value from feeder status update")
        else:
            self.logwarn(f"Feeder reported failure: {msg.message}")

    def __on_go_home_state(self, msg: Bool):
        self.go_home_state_signal.emit(bool(msg.data))

    def __update_go_home_state(self, active: bool):
        self.__go_home_active = active
        self.__go_home_button.setText("Stop Go Home" if active else "Go Home")

    def __start_go_home(self):
        """Toggle go home — starts if idle, stops if already running."""
        if self.__go_home_enable is None:
            self.logwarn("GoHome service not available")
            return
        if self.__go_home_active:
            try:
                self.__go_home_enable(False)
            except Exception as e:
                self.logwarn(f"Failed to stop GoHome: {e}")
        else:
            if self.__manual_control_window is not None:
                self.__motor_control_twist.publish(Twist())
                if hasattr(self, '_MainWindow__velocity_timer') and self.__velocity_timer is not None:
                    self.__velocity_timer.stop()
                self.__manual_control_window.close()
            try:
                if self.__motor_set_speed is not None:
                    self.__motor_set_speed(1.0)
                self.__go_home_enable(True)
            except Exception as e:
                self.logwarn(f"Failed to start GoHome: {e}")

    def __open_rotate_dialog(self):
        """Open dialog for selecting rotation angle."""
        if self.__go_home_active:
            self.logwarn("Cannot rotate while GoHome is active")
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("Select Rotation Angle")
        dialog.setFixedSize(260, 120)
        dialog.setWindowModality(Qt.ApplicationModal)

        layout = QGridLayout()
        for i, deg in enumerate([90, 180, 270, 360]):
            btn = QPushButton(f"{deg}°")
            btn.clicked.connect(lambda _, d=deg: (self.__execute_rotation(d), dialog.accept()))
            layout.addWidget(btn, 0, i)
        dialog.setLayout(layout)
        dialog.exec_()

    def __update_lidar_scan(self, msg):
        """Cache latest LiDAR ranges for closed-loop rotation."""
        self.__lidar_ranges = np.array(msg.ranges, dtype=np.float64)

    def __execute_rotation(self, degrees):
        """Start closed-loop rotation using LiDAR scan-matching."""
        if self.__lidar_ranges is None:
            self.logwarn("No LiDAR data available for rotation")
            return

        self.__rotate_button.setDisabled(True)
        self.__log_event("rotation", f"Rotating {degrees} degrees (CCW, closed-loop)")

        # Initialize scan-matching state
        self.__rotation_target = float(degrees)
        self.__rotation_accumulated = 0.0
        self.__rotation_prev_scan = self.__lidar_ranges.copy()

        # CCW rotation: negative angular.z per existing motor convention
        twist = Twist()
        twist.angular.z = -1.0
        self.__motor_control_twist.publish(twist)

        # Start closed-loop control at 20 Hz
        self.__rotation_control_timer = QTimer(self)
        self.__rotation_control_timer.timeout.connect(self.__rotation_control_step)
        self.__rotation_control_timer.start(50)

    def __rotation_control_step(self):
        """Measure rotation delta via FFT circular cross-correlation and stop at target."""
        if self.__lidar_ranges is None or self.__rotation_prev_scan is None:
            return

        current = self.__lidar_ranges.copy()
        prev = self.__rotation_prev_scan

        # FFT-based circular cross-correlation to find angular shift
        f_prev = np.fft.fft(prev)
        f_curr = np.fft.fft(current)
        cross_corr = np.fft.ifft(f_curr * np.conj(f_prev)).real
        shift = int(np.argmax(cross_corr))

        # Normalize shift to [-180, 180]
        if shift > 180:
            shift -= 360

        # Reject implausible jumps (noise/bad scan)
        if abs(shift) > 30:
            self.__rotation_prev_scan = current
            return

        # CCW rotation produces positive shift in scan indices
        self.__rotation_accumulated += abs(shift)
        self.__rotation_prev_scan = current

        # Check if target reached (within 2 deg tolerance)
        if self.__rotation_accumulated >= self.__rotation_target - 2.0:
            self.__finish_rotation()

    def __finish_rotation(self):
        """Stop rotation, clean up control state, re-enable button."""
        self.__motor_control_twist.publish(Twist())

        if self.__rotation_control_timer is not None:
            self.__rotation_control_timer.stop()
            self.__rotation_control_timer = None

        achieved = self.__rotation_accumulated
        target = self.__rotation_target
        self.__rotation_target = None
        self.__rotation_prev_scan = None
        self.__rotation_accumulated = 0.0

        self.__rotate_button.setDisabled(False)
        self.__log_event("rotation", f"Rotation completed (target={target:.0f}, achieved={achieved:.1f} deg)")
        self.loginfo(f"Rotation completed (target={target:.0f}, achieved={achieved:.1f} deg)")

    def __init_feeding_load_window(self):
        rospy.wait_for_service('fish_feeder/feed')
        self.__feeding_load_window = QDialog(self)
        self.__feeding_load_window.setWindowTitle("Load feeder")
        self.__feeding_load_window.setFixedSize(300, 300)
        self.__feeding_load_window.setWindowModality(Qt.ApplicationModal)
        step = -1

        def empty_feeder():
            if self.__empty_feeder is not None:
                self.__empty_feeder()

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
                # Feed 6 times for steps 0-3 (24 total)
                for _ in range(6):
                    self.__feed()
                step += 1
                step_label.setText(f"Step: {step}/5")
            elif step == 4:
                # Feed 2 times for step 4 (26 total = 32 holes - 6 visible slots)
                # for _ in range(2):
                #     self.__feed()
                step = -1
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
                ('__empty_feeder',            'fish_feeder/empty',       Trigger),
                ('__dim_lights',              'light_dimmer/change',     Float),
                ('__motor_control_system_check','motor_control/system_check', Check),
                ('__motor_set_speed',        'motor_control/set_speed', Float),
                ('__motor_set_mode',         'motor_control/set_mode',  String),
                ('__bypass_lidar',            'motor_control/bypass_lidar', SetBool),
                ('__go_home_enable', 'go_home/enable', SetBool),
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

    def __init_event_log(self, trial_folder):
        """Initialize event log CSV file for the trial"""
        event_log_path = os.path.join(trial_folder, "trial_events.csv")
        self.__event_log_file = open(event_log_path, 'w', newline='')
        self.__event_log_writer = csv.writer(self.__event_log_file)
        self.__event_log_writer.writerow(["timestamp", "event_type", "details"])
        self.__event_log_file.flush()
        rospy.loginfo(f"Created event log: {event_log_path}")

    def __on_times_fed_edited(self):
        """Log when the times fed counter is manually edited."""
        self.__log_event("times_fed_edited", f"Times fed manually set to {self.__times_fed_value_label.text()}")

    def __log_event(self, event_type, details=""):
        """Log an event to the trial events CSV"""
        if self.__event_log_writer is None:
            return
        
        timestamp = rospy.Time.now().to_sec()
        self.__event_log_writer.writerow([timestamp, event_type, details])
        self.__event_log_file.flush()
        self.loginfo(f"Event logged: {event_type} - {details}")

    def __close_event_log(self):
        """Close the event log file"""
        if self.__event_log_file is not None:
            self.__event_log_file.close()
            self.__event_log_file = None
            self.__event_log_writer = None

    def __update_velocity(self, is_pressed: bool, direction: int = 0):
        """
        Update robot velocity based on button presses.
        """
        if is_pressed:
            if direction >= 0:
                # Convert direction (degrees) to radians for vector calculation
                radians = math.radians(direction)
                self.__velocity.linear.x = -math.sin(radians)
                self.__velocity.linear.y = math.cos(radians)
                self.__velocity.angular.z = 0
    
            else:
                self.__velocity.linear.x = 0
                self.__velocity.linear.y = 0
                if direction == -1: # "cw"
                    self.__velocity.angular.z = 1
                elif direction == -2: # "ccw":
                    self.__velocity.angular.z = -1
        else:
            self.__velocity.linear.x = 0
            self.__velocity.linear.y = 0
            self.__velocity.angular.z = 0

    def __publish_velocity(self):
        self.__motor_control_twist.publish(self.__velocity)

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

    def __update_room_image(self, img_msg: CompressedImage):
        try:
            self.__room_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
            self.room_frame_ready.emit(self.__room_image.copy())
        except CvBridgeError as e:
            self.logwarn(f"Error converting image message: {e}")
        except Exception as e:
            self.logwarn(f"Unexpected error in update_room_image: {e}")

    def __update_foma_location(self, location: FomaLocation):
        # Convert normalized locations (0..1) to pixel coordinates
        self.__foma_img_location = Vector3(
            location.image.x * ROOM_CAMERA_FRAME_SHAPE[1],
            location.image.y * ROOM_CAMERA_FRAME_SHAPE[0],
            0
        )
        self.__foma_world_location = Vector3(
            location.world.x * ROOM_MAP_FRAME_SHAPE[1],
            location.world.y * ROOM_MAP_FRAME_SHAPE[0],
            0
        )
        
        if self.__room_map is not None:
            # Ensure coordinates stay within bounds
            x = np.clip(self.__foma_world_location.x, 0, ROOM_MAP_FRAME_SHAPE[1] - 1).astype(int)
            y = np.clip(self.__foma_world_location.y, 0, ROOM_MAP_FRAME_SHAPE[0] - 1).astype(int)
            cv2.circle(self.__room_map, (x, y), 2, (0, 255, 0), -1)

        # Update the FOMA room position label
        self.__foma_room_position_value_label.setText(f"({round(location.world.x * ROOM_FLOOR_MAP_SHAPE[1])}, {round(location.world.y * ROOM_FLOOR_MAP_SHAPE[0])})")
        self.__foma_img_position_value_label.setText(f"({round(self.__foma_img_location.x)}, {round(self.__foma_img_location.y)})")

        # Re-emit the latest room frame so the dot redraws at the new position without waiting for the next frame
        if self.__room_image is not None:
            self.room_frame_ready.emit(self.__room_image.copy())

    def __update_foma_speed(self, speed: TwistStamped):
        self.__foma_speed = speed.twist

    def __update_blocked_directions(self, blocked: Int16MultiArray):
        self.__blocked_directions = blocked.data

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
        if self.__show_fish_direction_cb.isChecked() and self.__fish_state is not None:
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
                        f"Dir: {angle:.1f}",
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
        if self.__show_foma_direction_cb.isChecked() and self.__foma_speed is not None:
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
            # Draw the FOMA location dot on the camera frame at current location
            if self.__foma_img_location is not None:
                cx = int(self.__foma_img_location.x)
                cy = int(self.__foma_img_location.y)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            frame = frame.data

        elif self.__map_display_rb.isChecked() and self.__room_map is not None:
            height, width, channel = self.__room_map.shape
            bytes_per_line = 3 * width
            map_frame = self.__room_map.copy()
            # Ensure coordinates stay within bounds
            if self.__foma_world_location is not None:
                x = np.clip(self.__foma_world_location.x, 0, ROOM_MAP_FRAME_SHAPE[1] - 1).astype(int)
                y = np.clip(self.__foma_world_location.y, 0, ROOM_MAP_FRAME_SHAPE[0] - 1).astype(int)
                cv2.circle(map_frame, (x, y), 2, (0, 0, 255), -1)
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

        # 1b) fish_feeder/empty
        empty_proxy = status.get('__empty_feeder')
        if self.__empty_feeder is None and empty_proxy is not None:
            self.loginfo("Empty feeder service available")
            self.__empty_feeder = empty_proxy
        elif self.__empty_feeder is not None and empty_proxy is None:
            self.logerr("Empty feeder service unavailable")
            self.__empty_feeder = None

        # 2) light_dimmer/change
        lights_proxy = status.get('__dim_lights')
        if self.__dim_lights is None and lights_proxy is not None:
            self.loginfo("Light dimming service available - enabling slider")
            self.__lights_slider.setDisabled(False)
            self.__dim_lights = lights_proxy
            self.__lights_slider.valueChanged.connect(lambda val: self.__on_lights_change(val))
        elif self.__dim_lights is not None and lights_proxy is None:
            self.logerr("Light dimming service unavailable - disabling slider")
            self.__lights_slider.setDisabled(True)
            self.__dim_lights = None

        # 3) motor_control/system_check
        motor_check_proxy = status.get('__motor_control_system_check')
        if self.__motor_control_system_check is None and motor_check_proxy is not None:
            self.loginfo("Manual control service available - enabling button")
            self.__manual_control_button.setDisabled(False)
            self.__rotate_button.setDisabled(False)
            self.__motor_control_system_check = motor_check_proxy
        elif self.__motor_control_system_check is not None and motor_check_proxy is None:
            self.logerr("Manual control service unavailable - disabling button")
            self.__manual_control_button.setDisabled(True)
            self.__rotate_button.setDisabled(True)
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

        # [ADDED] Store GoHome enable service proxy when available.
        go_home_proxy = status.get('__go_home_enable')
        if self.__go_home_enable is None and go_home_proxy is not None:
            self.loginfo("GoHome enable service available")
            self.__go_home_enable = go_home_proxy
            self.__go_home_button.setDisabled(False)
        elif self.__go_home_enable is not None and go_home_proxy is None:
            self.logerr("GoHome enable service unavailable")
            self.__go_home_enable = None
            self.__go_home_button.setDisabled(True)

        # [ADDED] Store motor_control/set_speed service proxy when available.
        motor_set_speed_proxy = status.get('__motor_set_speed')
        if self.__motor_set_speed is None and motor_set_speed_proxy is not None:
            self.loginfo("Motor set speed service available")
            self.__motor_set_speed = motor_set_speed_proxy
        elif self.__motor_set_speed is not None and motor_set_speed_proxy is None:
            self.logerr("Motor set speed service unavailable")
            self.__motor_set_speed = None

        motor_set_mode_proxy = status.get('__motor_set_mode')
        if self.__motor_set_mode is None and motor_set_mode_proxy is not None:
            self.loginfo("Motor set mode service available")
            self.__motor_set_mode = motor_set_mode_proxy
        elif self.__motor_set_mode is not None and motor_set_mode_proxy is None:
            self.logerr("Motor set mode service unavailable")
            self.__motor_set_mode = None

    def __on_feed_click(self):
        """Wrapper for feed action to log event"""
        if self.__feed is not None:
            self.__feed()
            self.__log_event("feeding", "Manual feed triggered")
            self.__times_fed_value_label.setText(str(int(self.__times_fed_value_label.text()) + 1))

    def __on_lights_change(self, val):
        """Wrapper for lights change to log event"""
        if self.__dim_lights is not None:
            brightness = int(255 * val / self.__lights_slider.maximum())
            self.__dim_lights(brightness)
            self.__log_event("light_control", f"Brightness set to {brightness}/255")

    def __find_latest_session(self):
        """
        Looks in ~/trial_output for the most recent session folder from today.
        Folder names are formatted as {%Y%m%d_%H%M}-{subject_id}.
        Returns (folder_path, subject_id, trial_count) or None if not found.
        """
        output_folder = os.path.expanduser('~/trial_output')
        if not os.path.exists(output_folder):
            return None
        
        today_prefix = datetime.datetime.now().strftime("%Y%m%d")
        candidates = []
        for name in os.listdir(output_folder):
            full_path = os.path.join(output_folder, name)
            if os.path.isdir(full_path) and name.startswith(today_prefix) and '-' in name:
                candidates.append((name, full_path))
        
        if not candidates:
            return None
        
        # Sort lexicographically — timestamp prefix makes this chronological
        candidates.sort(key=lambda x: x[0])
        latest_name, latest_path = candidates[-1]

        # Parse subject_id: everything after the first '-'
        subject_id = latest_name.split('-', 1)[1]

        # Count completed trial folders
        trial_nums = []
        for entry in os.listdir(latest_path):
            if os.path.isdir(os.path.join(latest_path, entry)) and entry.startswith('trial_'):
                try:
                    trial_nums.append(int(entry.split('_', 1)[1]))
                except ValueError:
                    pass
        trial_count = max(trial_nums) if trial_nums else 0

        return latest_path, subject_id, trial_count

    def __on_start_click(self):
        # Ask if New Session or New Trial        
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("Start Trial")
        msg_box.setText("What would you like to start?")
        new_session_btn = msg_box.addButton("New Session", QMessageBox.ActionRole)
        new_trial_btn = msg_box.addButton("New Trial", QMessageBox.ActionRole)
        load_session_btn = msg_box.addButton("Load Session", QMessageBox.ActionRole)
        msg_box.addButton(QMessageBox.Cancel)
        msg_box.exec_()
        
        if msg_box.clickedButton() == new_session_btn:
            # New Session: ask for subject ID and create session folder
            subject_id, ok = QInputDialog.getText(
                self,
                "Subject ID",
                "Please enter subject ID:"
            )
            if not ok or not subject_id:
                return
            
            # Create session folder with timestamp
            output_folder = os.path.expanduser('~/trial_output')
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
            self.__session_folder = os.path.join(output_folder, f"{timestamp}-{subject_id}")
            self.__current_trial = 1
            
            if not os.path.exists(self.__session_folder):
                os.makedirs(self.__session_folder)
                rospy.loginfo(f"Created session folder: {self.__session_folder}")

            # Update GUI labels
            self.__subject_id_value_label.setText(subject_id)
            self.__trial_num_value_label.setText(str(self.__current_trial))
            self.__times_fed_value_label.setText("0")
        
        elif msg_box.clickedButton() == new_trial_btn:
            # New Trial: check if session exists
            if self.__session_folder is None:
                QMessageBox.warning(self, "No Session", "Please create a New Session first!")
                return
            self.__current_trial += 1
            self.__trial_num_value_label.setText(str(self.__current_trial))

        elif msg_box.clickedButton() == load_session_btn:
            result = self.__find_latest_session()
            if result is None:
                QMessageBox.warning(self, "No Session Found",
                                    "No session folder from today was found in ~/trial_output.")
                return
            session_path, subject_id, trial_count = result
            self.__session_folder = session_path
            self.__current_trial = trial_count
            self.__subject_id_value_label.setText(subject_id)
            self.__trial_num_value_label.setText(str(trial_count) if trial_count > 0 else "N/A")

            # Count total feedings across all existing trial event logs
            total_fed = 0
            for entry in os.listdir(session_path):
                trial_events_path = os.path.join(session_path, entry, "trial_events.csv")
                if os.path.isfile(trial_events_path):
                    try:
                        with open(trial_events_path, 'r') as f:
                            reader = csv.reader(f)
                            next(reader, None)  # skip header
                            for row in reader:
                                if len(row) >= 2 and row[1] == 'feeding':
                                    total_fed += 1
                    except Exception as e:
                        self.logwarn(f"Could not read {trial_events_path}: {e}")
            self.__times_fed_value_label.setText(str(total_fed))

            folder_name = os.path.basename(session_path)
            QMessageBox.information(
                self,
                "Session Loaded",
                f"Loaded session: {folder_name}\n"
                f"Subject ID: {subject_id}\n"
                f"Trials completed: {trial_count}\n"
                f"Total feedings: {total_fed}\n\n"
                f"Press Start → New Trial to begin the next trial."
            )
            return

        else:
            # Cancelled
            return
        
        # Increment trial number
        trial_folder = os.path.join(self.__session_folder, f"trial_{self.__current_trial}")
        
        if not os.path.exists(trial_folder):
            os.makedirs(trial_folder)
            rospy.loginfo(f"Created trial folder: {trial_folder}")
        
        # Initialize event log
        self.__init_event_log(trial_folder)
        self.__log_event("trial_start", f"Trial {self.__current_trial} started")
        
        self.__room_map = np.ones((ROOM_MAP_FRAME_SHAPE[1], ROOM_MAP_FRAME_SHAPE[0], 3), dtype=np.uint8) * 255
        
        self.__start_button.setDisabled(True)
        self.__pause_button.setDisabled(False)
        self.__reset_button.setDisabled(False)
        self.__close_button.setDisabled(True)

        if self.__go_home_enable is not None:
            try:
                self.__go_home_enable(False)
            except Exception as e:
                self.logwarn(f"Failed disabling go_home: {e}")

        self.__ongoing_trial = True
        
        # Call all writer services with trial folder path
        for writer_service in self.__writer_services:
            try:
                writer_service("start", trial_folder, rospy.Time.now())
            except rospy.ServiceException as e:
                rospy.logerr(f"Writer service call failed: {e}")

        self.__log_event("light_control", f"Brightness set to 255/255")
        self.__lights_slider.setValue(self.__lights_slider.maximum())
        self.__motor_set_mode("trial")
        self.__motor_set_speed(1.0)

    def __on_continue_click(self):
        self.__start_button.setDisabled(True)
        self.__pause_button.setDisabled(False)
        self.__reset_button.setDisabled(False)
        self.__close_button.setDisabled(True)

        self.__ongoing_trial = True
        
        # Log continue event
        self.__log_event("trial_continue", "Trial resumed by user")
        
    def __on_pause_click(self):
        self.__start_button.setDisabled(False)
        self.__pause_button.setDisabled(True)
        self.__reset_button.setDisabled(False)
        self.__close_button.setDisabled(False)

        self.__start_button.setText("Continue")
        self.__start_button.clicked.disconnect()
        self.__start_button.clicked.connect(self.__on_continue_click)

        self.__ongoing_trial = False

        self.__motor_set_mode("idle")
        self.__motor_set_speed(0.0)

        self.__motor_control_vector.publish(Vector3(0, 0, 0))
        
        # Log pause event
        self.__log_event("trial_pause", "Trial paused by user")
        
    def __on_reset_click(self):
        self.__start_button.setDisabled(False)
        self.__pause_button.setDisabled(True)
        self.__reset_button.setDisabled(True)
        self.__close_button.setDisabled(False)

        self.__start_button.setText("Start")
        self.__start_button.clicked.disconnect()
        self.__start_button.clicked.connect(self.__on_start_click)

        self.__ongoing_trial = False

        self.__dim_lights(0)
        self.__log_event("light_control", f"Brightness set to 0/255")
        self.__lights_slider.setValue(0)

        #self.__motor_set_mode("idle")
        self.__motor_set_speed(1.0)
        
        # Log stop event
        self.__log_event("trial_stop", f"Trial {self.__current_trial} stopped")
        
        # Close event log
        self.__close_event_log()
        
        # Call all writer services to stop
        for writer_service in self.__writer_services:
            try:
                writer_service("stop", "", rospy.Time.now())
            except rospy.ServiceException as e:
                rospy.logerr(f"Writer service call failed: {e}")

        if self.__go_home_enable is not None:
            try:
                self.__go_home_enable(False)
            except Exception as e:
                self.logwarn(f"Failed disabling go_home: {e}")

    def __on_close_click(self, event):
        # Log stop event if trial is ongoing
        if self.__event_log_writer is not None:
            self.__log_event("trial_stop", "Trial stopped - GUI closing")
            self.__close_event_log()
        
        # Call all writer services to stop
        for writer_service in self.__writer_services:
            try:
                writer_service("stop", "", rospy.Time.now())
            except rospy.ServiceException as e:
                rospy.logerr(f"Writer service call failed: {e}")
        
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
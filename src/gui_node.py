#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
import math
import threading, time

from PyQt5.QtCore import Qt, QTimer, QEvent, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage, QDoubleValidator
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QFrame, QSizePolicy, QLabel,
    QPushButton, QSlider, QVBoxLayout, QDialog, QCheckBox, QLineEdit, QGroupBox,
    QRadioButton, QInputDialog
)

from geometry_msgs.msg import Twist, Vector3, TwistStamped
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, Int16MultiArray, Bool
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBoolRequest

from foma.srv import Light, Check, Write
from foma.msg import FomaLocation
from etc.settings import *


class MainWindow(QMainWindow):
    fish_frame_ready = pyqtSignal(np.ndarray)
    room_frame_ready = pyqtSignal(np.ndarray)
    services_updated = pyqtSignal(dict)

    def __init__(self):
        super(MainWindow, self).__init__()

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

    def __init_attributes(self):
        self.__foma_img_location = None
        self.__foma_world_location = None
        self.__room_image = None
        self.__room_map = np.ones((ROOM_MAP_FRAME_SHAPE[1], ROOM_MAP_FRAME_SHAPE[0], 3), dtype=np.uint8) * 255
        self.__fish_state = None
        self.__fish_image = None

        self.__room_frame_shape = (2560, 2560)
        self.__map_frame_shape = (1000, 1000)

        self.__manual_control_window = None
        self.__feeding_load_window = None

        self.__linear_velocity = Twist()
        self.__angular_velocity = Float32()
        self.__foma_speed = Twist()
        self.__direction_epsilon = 45
        self.__blocked_directions = None

        self.__motor_control_system_check = None
        self.__dim_lights = None
        self.__feed = None
        self.__bypass_lidar = None
        self.__go_home_enable = None

        self.__go_home_enabled = False
        self.__ongoing_trial = False

    def __init_widgets(self):
        self.__start_button = QPushButton("Start")
        self.__start_button.setDisabled(False)
        self.__start_button.setMaximumHeight(50)
        self.__start_button.clicked.connect(self.__on_start_click)

        self.__pause_button = QPushButton("Pause")
        self.__pause_button.setDisabled(True)
        self.__pause_button.setMaximumHeight(50)
        self.__pause_button.clicked.connect(self.__on_pause_click)

        self.__reset_button = QPushButton("Reset")
        self.__reset_button.setDisabled(True)
        self.__reset_button.setMaximumHeight(50)
        self.__reset_button.clicked.connect(self.__on_reset_click)

        self.__close_button = QPushButton("Close")
        self.__close_button.setDisabled(False)
        self.__close_button.setMaximumHeight(50)
        self.__close_button.clicked.connect(self.__on_close_click)

        self.__feed_button = QPushButton("Feed")
        self.__feed_button.clicked.connect(lambda: self.__feed())
        self.__feed_button.setDisabled(True)

        self.__feed_label = QLabel("Manual Feed")
        font = self.__feed_label.font()
        font.setPointSize(13)
        self.__feed_label.setFont(font)
        self.__feed_label.setAlignment(Qt.AlignHCenter)

        self.__lights_slider = QSlider(Qt.Horizontal)
        self.__lights_slider.setMinimum(0)
        self.__lights_slider.setMaximum(1)
        self.__lights_slider.setTickPosition(QSlider.TicksAbove | QSlider.TicksBelow)
        self.__lights_slider.setPageStep(1)
        self.__lights_slider.setMaximumHeight(50)
        self.__lights_slider.setDisabled(True)
        self.__lights_slider.setValue(1)

        self.__lights_label = QLabel("Lights dimming")
        font = self.__lights_label.font()
        font.setPointSize(13)
        self.__lights_label.setFont(font)
        self.__lights_label.setAlignment(Qt.AlignHCenter)

        self.__manual_control_button = QPushButton("Manual Control")
        self.__manual_control_button.setMaximumHeight(50)
        self.__manual_control_button.clicked.connect(self.__init_manual_control_window)
        self.__manual_control_button.setDisabled(True)

        self.__feed_loading_button = QPushButton("Load Feeder")
        self.__feed_loading_button.setMaximumHeight(50)
        self.__feed_loading_button.clicked.connect(self.__init_feeding_load_window)
        self.__feed_loading_button.setDisabled(True)

        self.__left_image_frame = QLabel()
        self.__left_image_frame.setScaledContents(True)
        self.__left_image_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.__fish_image_label = QLabel("Fish Camera")
        font = self.__fish_image_label.font()
        font.setPointSize(15)
        self.__fish_image_label.setFont(font)
        self.__fish_image_label.setAlignment(Qt.AlignHCenter)

        self.__top_right_image = QLabel()
        self.__top_right_image.setScaledContents(True)
        self.__top_right_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.__room_image_label = QLabel("Room Camera")
        font = self.__room_image_label.font()
        font.setPointSize(15)
        self.__room_image_label.setFont(font)
        self.__room_image_label.setAlignment(Qt.AlignHCenter)
        self.__room_image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.__show_fish_direction_rb = QRadioButton("Yes")
        self.__hide_fish_direction_rb = QRadioButton("No")
        self.__show_fish_direction_rb.setChecked(True)

        fish_direction_layout = QGridLayout()
        fish_direction_layout.addWidget(self.__show_fish_direction_rb, 0, 0, alignment=Qt.AlignCenter)
        fish_direction_layout.addWidget(self.__hide_fish_direction_rb, 1, 0, alignment=Qt.AlignCenter)
        self.__fish_direction_group = QGroupBox("Display Fish Direction")
        self.__fish_direction_group.setLayout(fish_direction_layout)

        self.__show_foma_direction_rb = QRadioButton("Yes")
        self.__hide_foma_direction_rb = QRadioButton("No")
        self.__show_foma_direction_rb.setChecked(True)

        foma_direction_layout = QGridLayout()
        foma_direction_layout.addWidget(self.__show_foma_direction_rb, 0, 0, alignment=Qt.AlignCenter)
        foma_direction_layout.addWidget(self.__hide_foma_direction_rb, 1, 0, alignment=Qt.AlignCenter)
        self.__foma_direction_group = QGroupBox("Display FOMA Direction")
        self.__foma_direction_group.setLayout(foma_direction_layout)

        self.__room_camera_display_rb = QRadioButton("Camera")
        self.__map_display_rb = QRadioButton("Map")
        self.__room_camera_display_rb.setChecked(True)

        room_display_layout = QGridLayout()
        room_display_layout.addWidget(self.__room_camera_display_rb, 0, 0, alignment=Qt.AlignCenter)
        room_display_layout.addWidget(self.__map_display_rb, 1, 0, alignment=Qt.AlignCenter)
        self.__room_display_group = QGroupBox("Top-Right Display")
        self.__room_display_group.setLayout(room_display_layout)

    def __init_layouts(self):
        self.__TL_layout = QVBoxLayout()
        self.__TL_layout.addWidget(self.__fish_image_label, alignment=Qt.AlignCenter)
        self.__TL_layout.addWidget(self.__left_image_frame)

        self.__TL_widget = QFrame()
        self.__TL_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__TL_widget.setLineWidth(2)
        self.__TL_widget.setLayout(self.__TL_layout)

        self.__TR_layout = QVBoxLayout()
        self.__TR_layout.addWidget(self.__room_image_label, alignment=Qt.AlignCenter)
        self.__TR_layout.addWidget(self.__top_right_image)

        self.__TR_widget = QFrame()
        self.__TR_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__TR_widget.setLineWidth(2)
        self.__TR_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.__TR_widget.setLayout(self.__TR_layout)

        self.__BL_layout = QGridLayout()
        self.__BL_layout.addWidget(self.__lights_label, 0, 0, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__lights_slider, 1, 0, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__feed_label, 0, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__feed_button, 1, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__manual_control_button, 0, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__feed_loading_button, 1, 2, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__fish_direction_group, 0, 3, 2, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__foma_direction_group, 0, 4, 2, 1, alignment=Qt.AlignCenter)
        self.__BL_layout.addWidget(self.__room_display_group, 0, 5, 2, 1, alignment=Qt.AlignCenter)

        self.__BL_widget = QFrame()
        self.__BL_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__BL_widget.setLineWidth(2)
        self.__BL_widget.setLayout(self.__BL_layout)

        self.__BR_layout = QGridLayout()
        self.__BR_layout.addWidget(self.__start_button, 0, 0, alignment=Qt.AlignCenter)
        self.__BR_layout.addWidget(self.__pause_button, 0, 1, alignment=Qt.AlignCenter)
        self.__BR_layout.addWidget(self.__reset_button, 1, 0, alignment=Qt.AlignCenter)
        self.__BR_layout.addWidget(self.__close_button, 1, 1, alignment=Qt.AlignCenter)

        self.__BR_widget = QFrame()
        self.__BR_widget.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.__BR_widget.setLineWidth(2)
        self.__BR_widget.setLayout(self.__BR_layout)

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

    def __init_subscriptions_and_services(self):
        rospy.Subscriber('fish_camera/image', CompressedImage, self.__update_fish_image)
        rospy.Subscriber('fish_detection/state', TwistStamped, self.__update_fish_state)
        rospy.Subscriber('ceiling_camera/image', Image, self.__update_room_image)
        rospy.Subscriber('localization/location', FomaLocation, self.__update_foma_location)
        rospy.Subscriber('motor_control/blocked', Int16MultiArray, self.__update_blocked_directions)
        rospy.Subscriber('motor_control/speed', TwistStamped, self.__update_foma_speed)
        rospy.Subscriber('go_home/enabled', Bool, self.__on_go_home_enabled)

        self.__motor_control_twist = rospy.Publisher('motor_control/twist', Twist, queue_size=10)
        self.__motor_control_dir = rospy.Publisher('motor_control/angle', Float32, queue_size=10)
        self.__motor_control_vector = rospy.Publisher('motor_control/vector', Vector3, queue_size=10)
        self.__motor_control_rotate = rospy.Publisher('motor_control/rotate', Float32, queue_size=10)
        self.__motor_set_speed = rospy.Publisher('motor_control/set_speed', Float32, queue_size=10)

        self.__writer_control = rospy.ServiceProxy('writer_node/write', Write)

    def __on_go_home_enabled(self, msg: Bool):
        self.__go_home_enabled = bool(msg.data)

    def __init_manual_control_window(self):
        self.__motor_set_speed.publish(Float32(1.0))
        if self.__bypass_lidar is not None:
            self.__bypass_lidar(False)

        self.__manual_control_window = QDialog(self)
        self.__manual_control_window.setWindowTitle("Manual Robot Control")
        self.__manual_control_window.setFixedSize(300, 360)
        self.__manual_control_window.setWindowModality(Qt.ApplicationModal)

        def on_close(event):
            self.__motor_control_twist.publish(Twist())
            self.__motor_control_rotate.publish(Float32(0.0))
            self.__motor_set_speed.publish(Float32(1.0))
            if self.__bypass_lidar is not None:
                self.__bypass_lidar(False)
            self.__manual_control_window = None
            event.accept()

        def on_key_press(event):
            key = event.key()
            key_to_angle = {
                Qt.Key_8: 0,
                Qt.Key_2: 180,
                Qt.Key_4: 90,
                Qt.Key_6: 270,
                Qt.Key_7: 45,
                Qt.Key_9: 315,
                Qt.Key_1: 135,
                Qt.Key_3: 225,
                Qt.Key_Plus: -2,
                Qt.Key_Minus: -1
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

        go_home_button = QPushButton("Go Home")
        go_home_button.setMaximumHeight(40)

        speed_control_textbox.setPlaceholderText("1.0")
        speed_control_textbox.setValidator(QDoubleValidator(0.0, 1.0, 2))

        def set_speed():
            try:
                speed = float(speed_control_textbox.text())
                self.__motor_set_speed.publish(Float32(speed))
            except ValueError:
                self.logwarn("Invalid speed value")

        def start_go_home():
            if self.__go_home_enable is None:
                self.logwarn("GoHome service not available")
                return
            self.__motor_control_twist.publish(Twist())
            self.__motor_control_rotate.publish(Float32(0.0))
            try:
                self.__go_home_enable(True)
            except Exception as e:
                self.logwarn(f"Failed enabling go_home: {e}")

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
        control_layout.addWidget(go_home_button, 5, 0, 1, 4)

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
        go_home_button.clicked.connect(start_go_home)

        bypass_lidar_checkbox.stateChanged.connect(lambda state: self.__bypass_lidar(state == Qt.Checked) if self.__bypass_lidar is not None else None)

        self.__manual_control_window.setLayout(control_layout)

        self.__velocity_timer = QTimer(self)
        self.__velocity_timer.timeout.connect(self.__publish_velocity)
        self.__velocity_timer.start(50)

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
            key = event.key()
            if key == Qt.Key_Return and step != -1:
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
            services = [
                ('__feed', 'fish_feeder/feed', Trigger),
                ('__dim_lights', 'light_dimmer/change', Light),
                ('__motor_control_system_check', 'motor_control/system_check', Check),
                ('__bypass_lidar', 'motor_control/bypass_lidar', SetBool),
                ('__go_home_enable', 'go_home/enable', SetBool),
            ]
            while not rospy.is_shutdown():
                status = {}
                for attr, name, srv_type in services:
                    try:
                        rospy.wait_for_service(name, timeout=0.2)
                        status[attr] = rospy.ServiceProxy(name, srv_type)
                    except Exception:
                        status[attr] = None
                self.services_updated.emit(status)
                time.sleep(1)

        t = threading.Thread(target=checker_loop, daemon=True)
        t.start()

    def __update_velocity(self, direction: int, is_pressed: bool):
        if direction >= 0:
            radians = math.radians(direction)
            if is_pressed:
                self.__linear_velocity.linear.x = -math.sin(radians)
                self.__linear_velocity.linear.y = math.cos(radians)
            else:
                self.__linear_velocity.linear.x = 0
                self.__linear_velocity.linear.y = 0
        else:
            if direction == -1:
                self.__angular_velocity.data = 1 if is_pressed else 0
            elif direction == -2:
                self.__angular_velocity.data = -1 if is_pressed else 0
            self.__motor_control_rotate.publish(self.__angular_velocity)

    def __publish_velocity(self):
        if self.__go_home_enabled:
            return
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
            fish_x = int(state.linear.x)
            fish_y = int(state.linear.y)

            height, width, _ = self.__fish_image.shape
            center_x = width // 2
            center_y = height // 2
            vector_x = fish_x - center_x
            vector_y = center_y - fish_y

            center_to_fish_angle = math.degrees(math.atan2(vector_y, vector_x))

            direction_angle = math.degrees(
                math.atan2(state.angular.y, state.angular.x)
            )

            def shortest_angle_diff(a, b):
                d = (a - b + 180) % 360 - 180
                return d

            diff = shortest_angle_diff(direction_angle, center_to_fish_angle)

            if abs(diff) <= self.__direction_epsilon:
                self.__motor_control_dir.publish(direction_angle)
            else:
                self.__motor_control_vector.publish(Vector3(0, 0, 0))

    def __update_room_image(self, img_msg: Image):
        try:
            self.__room_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
            frame = self.__room_image.copy()
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
            x = np.clip(self.__foma_world_location.x, 0, self.__room_frame_shape[1] - 1).astype(int)
            y = np.clip(self.__foma_world_location.y, 0, self.__room_frame_shape[0] - 1).astype(int)
            cv2.circle(self.__room_map, (x, y), 2, (0, 255, 0), -1)

    def __update_foma_speed(self, speed: TwistStamped):
        self.__foma_speed = speed.twist

    def __update_blocked_directions(self, blocked: Int16MultiArray):
        self.__blocked_directions = blocked.data

    def __update_left_display(self, frame: np.ndarray):
        if frame is None:
            return

        h, w, ch = frame.shape

        if self.__blocked_directions is not None:
            for angle in self.__blocked_directions:
                cx, cy = w // 2, h // 2
                delta_deg = 0.5
                angles = (angle - delta_deg, angle + delta_deg)
                pts = []
                for ang in angles:
                    math_ang = math.radians(ang + 90)
                    dx = math.cos(math_ang)
                    dy = -math.sin(math_ang)
                    ts = []
                    if abs(dx) > 1e-6:
                        t1 = (0 - cx) / dx
                        t2 = (w - cx) / dx
                        if t1 > 0: ts.append(t1)
                        if t2 > 0: ts.append(t2)
                    if abs(dy) > 1e-6:
                        t3 = (0 - cy) / dy
                        t4 = (h - cy) / dy
                        if t3 > 0: ts.append(t3)
                        if t4 > 0: ts.append(t4)
                    if not ts:
                        continue
                    t_min = min(ts)
                    x_edge = int(cx + t_min * dx)
                    y_edge = int(cy + t_min * dy)
                    pts.append((x_edge, y_edge))

                if len(pts) == 2:
                    cv2.line(frame, pts[0], pts[1], (255, 0, 0), 5)
                elif len(pts) == 1:
                    cv2.circle(frame, pts[0], 3, (255, 0, 0), -1)

        if self.__show_fish_direction_rb.isChecked() and self.__fish_state is not None:
            px = int(self.__fish_state.linear.x)
            py = int(self.__fish_state.linear.y)
            dx = self.__fish_state.angular.x
            dy = self.__fish_state.angular.y
            angle = math.degrees(math.atan2(dy, dx))

            cv2.circle(frame, (px, py), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Dir: {angle:.1f}", (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            length = 30
            ex = px + int(length * math.cos(math.radians(angle)))
            ey = py - int(length * math.sin(math.radians(angle)))
            cv2.arrowedLine(frame, (px, py), (ex, ey), (0, 255, 0), 2, tipLength=0.3)

        if self.__show_foma_direction_rb.isChecked() and self.__foma_speed is not None:
            linear_speed = math.sqrt(self.__foma_speed.linear.x**2 + self.__foma_speed.linear.y**2)
            angular_speed = self.__foma_speed.angular.z
            center_x, center_y = w // 2, h // 2

            if linear_speed > 0:
                angle = math.degrees(math.atan2(self.__foma_speed.linear.y, self.__foma_speed.linear.x))
                arrow_length = 30
                end_x = int(center_x + arrow_length * math.cos(math.radians(angle)))
                end_y = int(center_y - arrow_length * math.sin(math.radians(angle)))
                cv2.arrowedLine(frame, (center_x, center_y), (end_x, end_y), (0, 255, 255), 2, tipLength=0.3)
                cv2.putText(frame, f"{linear_speed:.2f}", (end_x + 10, end_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if angular_speed != 0:
                radius = 30
                start_angle = 0
                end_angle = 270 if angular_speed > 0 else -270
                color = (255, 255, 0)
                cv2.ellipse(frame, (center_x, center_y), (radius, radius), 0, start_angle, end_angle, color, 2)
                cv2.putText(frame, f"{angular_speed:.2f}", (center_x + radius + 10, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            self.__left_image_frame.width(),
            self.__left_image_frame.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.__left_image_frame.setPixmap(pix)

    def __update_right_display(self, frame: np.ndarray):
        if self.__room_camera_display_rb.isChecked() and frame is not None:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            frame = frame.data
        elif self.__map_display_rb.isChecked() and self.__room_map is not None:
            height, width, channel = self.__room_map.shape
            bytes_per_line = 3 * width
            map_frame = self.__room_map.copy()
            if self.__foma_world_location is not None:
                x = np.clip(self.__foma_world_location.x, 0, self.__room_frame_shape[1] - 1).astype(int)
                y = np.clip(self.__foma_world_location.y, 0, self.__room_frame_shape[0] - 1).astype(int)
                cv2.circle(map_frame, (x, y), 2, (0, 0, 255), -1)
            frame = map_frame.data
        else:
            return

        q_image = QImage(frame, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        scaled_pixmap = pixmap.scaled(
            self.__top_right_image.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.__top_right_image.setPixmap(scaled_pixmap)

    def __update_services(self, status: dict):
        feed_proxy = status.get('__feed')
        if self.__feed is None and feed_proxy is not None:
            self.__feed_button.setDisabled(False)
            self.__feed_loading_button.setDisabled(False)
            self.__feed = feed_proxy
        elif self.__feed is not None and feed_proxy is None:
            self.__feed_button.setDisabled(True)
            self.__feed_loading_button.setDisabled(True)
            self.__feed = None

        lights_proxy = status.get('__dim_lights')
        if self.__dim_lights is None and lights_proxy is not None:
            self.__lights_slider.setDisabled(False)
            self.__dim_lights = lights_proxy
            self.__lights_slider.valueChanged.connect(lambda val: self.__dim_lights(int(255 * val / self.__lights_slider.maximum())))
        elif self.__dim_lights is not None and lights_proxy is None:
            self.__lights_slider.setDisabled(True)
            self.__dim_lights = None

        motor_check_proxy = status.get('__motor_control_system_check')
        if self.__motor_control_system_check is None and motor_check_proxy is not None:
            self.__manual_control_button.setDisabled(False)
            self.__motor_control_system_check = motor_check_proxy
        elif self.__motor_control_system_check is not None and motor_check_proxy is None:
            self.__manual_control_button.setDisabled(True)
            self.__motor_control_system_check = None

        bypass_proxy = status.get('__bypass_lidar')
        if self.__bypass_lidar is None and bypass_proxy is not None:
            self.__manual_control_button.setDisabled(False)
            self.__bypass_lidar = bypass_proxy
        elif self.__bypass_lidar is not None and bypass_proxy is None:
            self.__manual_control_button.setDisabled(True)
            self.__bypass_lidar = None

        go_home_proxy = status.get('__go_home_enable')
        if self.__go_home_enable is None and go_home_proxy is not None:
            self.__go_home_enable = go_home_proxy
        elif self.__go_home_enable is not None and go_home_proxy is None:
            self.__go_home_enable = None

    def __on_start_click(self):
        subject_id, ok = QInputDialog.getText(self, "Subject ID", "Please enter subject ID:")
        if not ok or not subject_id:
            return

        self.__room_map = np.ones((ROOM_MAP_FRAME_SHAPE[1], ROOM_MAP_FRAME_SHAPE[0], 3), dtype=np.uint8) * 255

        self.__start_button.setDisabled(True)
        self.__pause_button.setDisabled(False)
        self.__reset_button.setDisabled(False)
        self.__close_button.setDisabled(True)

        if self.__go_home_enable is not None:
            try:
                self.__go_home_enable(False)
            except Exception:
                pass

        self.__ongoing_trial = True
        self.__writer_control("start", subject_id, rospy.Time.now())

    def __on_continue_click(self):
        self.__start_button.setDisabled(True)
        self.__pause_button.setDisabled(False)
        self.__reset_button.setDisabled(False)
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

        if self.__go_home_enable is not None:
            try:
                self.__go_home_enable(False)
            except Exception:
                pass

    def __on_close_click(self, event=None):
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
        self.drag_start = event.globalPos()

    def mouseMoveEvent(self, event):
        move_distance = event.globalPos() - self.drag_start
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

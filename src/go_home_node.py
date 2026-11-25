#!/usr/bin/env python3
"""Room-centring and go-to-point behaviour for FOMA, built from reusable
phase primitives."""

import contextlib
import math
import threading
import time
from collections import deque, namedtuple

import cv2
import numpy as np
import rospy
from abstract_node import AbstractNode
from etc import room_mapping as rm
from etc import scan_geometry as geom
from foma.msg import FomaLocation
from foma.srv import Float, GoTo, GoToResponse
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

#  derived once per scan.  `fresh_t` = when the scan contents last changed
#  (lidar_node keeps republishing a dead sensor's buffer).
Measurement = namedtuple(
    'Measurement',
    'ranges walls err_x err_y dist ctr_valid ang_err ang_valid ang_conf '
    'fresh_t')


def _signed_permutations():
    """The 8 maps between two square frames: axes swapped or not, each signed."""
    for swap in (False, True):
        for sx in (1, -1):
            for sy in (1, -1):
                r = np.zeros((2, 2), dtype=int)
                r[0, 1 if swap else 0] = sx
                r[1, 0 if swap else 1] = sy
                yield r


#  Body (right, forward) -> room (x, y), once the robot is square to the walls.
AXIS_MAPS = tuple(_signed_permutations())


class Status:
    SUCCESS   = 'success'
    TIMEOUT   = 'timeout'
    CANCELLED = 'cancelled'
    NO_SCAN   = 'no_scan'
    UNSETTLED = 'unsettled'
    BUSY      = 'busy'
    DRIFT     = 'drift'
    NO_LOC    = 'no_location'
    PROBE     = 'probe_failed'
    STUCK     = 'stuck'


class PhaseResult(namedtuple('PhaseResult', 'status error elapsed')):
    """Outcome of a phase.  `error` is the residual it finished on."""

    __slots__ = ()

    @property
    def ok(self):
        return self.status == Status.SUCCESS

    def __bool__(self):
        return self.ok

    def __str__(self):
        return "%s (err=%.4f, %.1fs)" % (self.status, self.error, self.elapsed)


class GoHomeNode(AbstractNode):
    #  Phase labels
    ST_IDLE   = 0
    ST_ALIGN  = 1
    ST_CENTER = 2
    ST_VERIFY = 3
    ST_PROBE  = 4
    ST_GOTO   = 5

    STATE_NAMES = {ST_IDLE: 'IDLE', ST_ALIGN: 'ALIGN',
                   ST_CENTER: 'CENTER', ST_VERIFY: 'VERIFY',
                   ST_PROBE: 'PROBE', ST_GOTO: 'GOTO'}

    # Time, not frames: at 100 Hz a frame count confirms before the drive settles.
    # >= 2 lidar revolutions (~0.18 s each) plus the motor ramp-down.
    ALIGN_SETTLE_S  = 0.8
    CENTER_SETTLE_S = 0.8

    #  Tolerances
    ALIGN_TOL         = 2.5     # degrees; ~ shortest rotation the drive can do
    CENTER_TOL        = 0.03    # metres from room centre
    VERIFY_ALIGN_TOL  = 2.0     # between FINE_ALIGN_TOL and ALIGN_TOL
    VERIFY_CENTER_TOL = 0.05
    RELEASE_SCALE     = 1.5     # hysteresis: settling survives noise below this
    CENTER_DRIFT_DEG  = 6.0     # heading lost while centring -> re-align
    MIN_HEADING_CONF  = 0.5     # inlier fraction to trust the wall heading

    #  Velocity limits: motor duty in [-1, 1], not m/s.
    MAX_ANG_V = 0.45
    MIN_ANG_V = 0.20
    KP_ANG    = 0.015           # duty per degree of heading error
    # Wiring decides which angular.z sign reduces the error (/ALIGN_SIGN);
    # optionally checked during the first turn, see _check_sign.
    ALIGN_SIGN      = 1
    SIGN_AUTODETECT = True
    SIGN_PROBE_DEG  = 1.0       # x3 = error growth that proves the sign wrong
    # Smooth turn: min_ang_v only to break away, then a lower creep duty
    # (kinetic friction < static), and coast once the measured turn rate
    # would carry the robot to the target within STOP_LEAD_S.
    CREEP_ANG_V     = 0.17
    MOVING_DPS      = 3.0       # deg/s above which the robot counts as turning
    STOP_LEAD_S     = 0.25      # ~ lidar lag + motor ramp-down
    RATE_ALPHA      = 0.6
    RATE_WINDOW_S   = 0.15      # ~ one lidar revolution
    LAG_S           = 0.20      # scan age + half the rate window
    STALL_S         = 0.4       # not turning this long while commanded ->
    STALL_STEP      = 0.03      # raise the floor by this much
    # Final in-place squaring after centring (motor_control's heading hold
    # keeps the robot square while it drives, so this is usually a no-op).
    FINE_ALIGN_TOL  = 1.0       # degrees
    FINE_ALIGN_TIMEOUT_S = 12.0
    HEADING_OFFSET  = 0.0       # deg; lidar mounting yaw vs the chassis

    MAX_LIN_V = 0.95
    MIN_LIN_V = 0.25
    KP_LIN    = 5.0             # P-gain: error → linear velocity

    #  Beam sampling
    BEAM_HALF_WIN = 10          # ± rays around target angle
    RANGE_MIN_M   = 0.10       # reject closer than 10 cm
    RANGE_MAX_M   = 12.0       # reject farther than 12 m

    #  Routine
    GLOBAL_TIMEOUT_S = 90.0
    CONTROL_HZ       = 100.0    # matches the lidar publish rate
    GO_HOME_PASSES   = 3        # align+centre retries before giving up
    SCAN_WAIT_S      = 2.0      # grace period for the first scan
    STALE_S          = 0.5      # unchanged scan this long -> stop driving
    VERIFY_SCANS     = 5        # fresh scans the final check is a median of
    VERIFY_WINDOW_S  = 1.5

    #  go_to.  The LIDAR drives; the camera only fixes where the room is.
    GOTO_TOL         = 0.035    # m
    VERIFY_GOTO_TOL  = 0.07
    GOTO_SETTLE_S    = 0.8
    GOTO_KP          = 8.0      # duty per metre
    GOTO_MAX_V       = 0.95
    GOTO_CAM_KP      = 2.0      # camera only (a wall unseen): slower, it lags
    GOTO_CAM_MAX_V   = 0.5
    GOTO_BRAKE_M     = 0.30     # closer: at most GOTO_APPROACH_V
    GOTO_APPROACH_V  = 0.45
    GOTO_DIAG_M      = 0.15     # per-axis friction floor only further than this
    GOTO_LIDAR_CLAMP = True     # keep the target inside the seen walls (false in a maze)
    ANCHOR_N         = 9        # camera frames per anchor
    ANCHOR_WINDOW_S  = 2.0
    ANCHOR_SPREAD_M  = 0.05     # warn above
    CAMERA_CHECK_M   = 0.15     # camera vs LIDAR, warn above
    LIDAR_JUMP_M     = 0.08     # wall distance jump in one scan = another wall
    LIDAR_HIST_S     = 1.0      # > camera lag
    LOC_STALE_S      = 0.5      # camera position unchanged this long = lost
    LOC_WAIT_S       = 3.0
    VERIFY_LOCS      = 5
    VERIFY_LOC_WINDOW_S = 2.0
    #  Probe: a short move along a body axis shows which room axis it is.
    PROBE_V          = 0.35
    PROBE_DIST_M     = 0.12
    PROBE_LOCS       = 7        # camera frames per end
    PROBE_MIN_LOCS   = 3
    PROBE_LOC_WINDOW_S = 3.0    # the detector drops frames for seconds
    PROBE_MIN_M      = 0.05
    PROBE_TIMEOUT_S  = 3.0
    PROBE_SETTLE_S   = 0.6      # camera lag
    PROBE_AXIS_TOL_DEG = 30.0
    #  Axis map kept between runs by unwrapping the wall heading.
    TRACK_GAP_S      = 2.0      # no heading this long -> map lost
    TRACK_MAX_STEP_DEG = 20.0   # bigger jump = bad fit, skipped (more after a gap)
    TRACK_MAX_DPS    = 40.0     # turn rate assumed across a gap
    LAST_POS_FIT_M   = 0.25     # axes from where the robot last stopped:
    LAST_POS_MARGIN_M = 0.30    # best within this, next this much worse
    #  Axis map from one camera fix (room centre known).
    AXES_FIT_M       = 0.20     # best map within this of the camera
    AXES_MARGIN_M    = 0.25     # and the next one this much worse
    #  Walls
    ESCAPE_CLEAR_M   = 0.46     # half the robot diagonal + 5 cm: room to turn
    ESCAPE_V         = 0.45
    ESCAPE_TIMEOUT_S = 4.0
    STUCK_S          = 2.0      # no STUCK_PROGRESS_M closer this long = stuck
    STUCK_PROGRESS_M = 0.02

    #  Visualisation
    VIZ_ENABLED  = True
    VIZ_FPS      = 20.0
    VIZ_SIZE     = 720
    VIZ_RANGE_M  = 0.0          # 0 → auto-fit the view to the scan
    ROBOT_SIZE_M = 0.576        # footprint, matches motor_control_node

    VIZ_WINDOW = "GoHome - room model"

    # Overlay colours (BGR)
    C_BG       = (24, 24, 24)
    C_GRID     = (58, 58, 58)
    C_POINTS   = (0, 200, 255)
    C_ROOM     = (0, 235, 130)
    C_ROBOT    = (245, 245, 245)
    C_HEADING  = (0, 150, 255)
    C_CENTRE   = (60, 60, 255)
    C_CARDINAL = (215, 130, 255)
    C_PROBE    = (110, 190, 110)
    C_TEXT     = (235, 235, 235)
    C_TEXT_DIM = (150, 150, 150)
    C_DRIVE    = (0, 220, 255)      # commanded drive direction (yellow)
    C_MOTION   = (255, 80, 255)     # measured motion direction (magenta)
    C_TRAIL    = (255, 220, 0)      # path history (cyan)

    TRAIL_S      = 6.0              # path history drawn behind the robot
    MOTION_WIN_S = 0.5              # window the measured velocity spans

    FONT = cv2.FONT_HERSHEY_SIMPLEX

    def __init__(self):
        super().__init__('go_home', 'Go Home')

        #  Tuning (private params, defaults above)
        g = rospy.get_param
        self.align_settle  = float(g('~align_settle', self.ALIGN_SETTLE_S))
        self.center_settle = float(g('~center_settle', self.CENTER_SETTLE_S))
        self.align_tol         = float(g('~align_tol', self.ALIGN_TOL))
        self.center_tol        = float(g('~center_tol', self.CENTER_TOL))
        self.verify_align_tol  = float(g('~verify_align_tol',
                                         self.VERIFY_ALIGN_TOL))
        self.verify_center_tol = float(g('~verify_center_tol',
                                         self.VERIFY_CENTER_TOL))
        self.max_ang_v = float(g('~max_ang_v', self.MAX_ANG_V))
        self.min_ang_v = float(g('~min_ang_v', self.MIN_ANG_V))
        self.kp_ang    = float(g('~kp_ang', self.KP_ANG))
        self.max_lin_v = float(g('~max_lin_v', self.MAX_LIN_V))
        self.min_lin_v = float(g('~min_lin_v', self.MIN_LIN_V))
        self.kp_lin    = float(g('~kp_lin', self.KP_LIN))
        self.timeout_s  = float(g('~timeout', self.GLOBAL_TIMEOUT_S))
        self.control_hz = float(g('~control_hz', self.CONTROL_HZ))
        self.passes     = int(g('~go_home_passes', self.GO_HOME_PASSES))
        self.release_scale = float(g('~release_scale', self.RELEASE_SCALE))
        self.center_drift  = float(g('~center_drift_deg',
                                     self.CENTER_DRIFT_DEG))
        self.min_conf      = float(g('~min_heading_conf',
                                     self.MIN_HEADING_CONF))
        # Shared with motor_control's heading hold, hence global.
        self.align_sign      = 1 if int(g('/ALIGN_SIGN', self.ALIGN_SIGN)) >= 0 else -1
        self.heading_offset  = float(g('/HEADING_OFFSET_DEG',
                                       self.HEADING_OFFSET))
        self.sign_autodetect = bool(g('~sign_autodetect', self.SIGN_AUTODETECT))
        self._sign_checked   = False    # sign check runs once per node lifetime
        self.creep_ang_v     = float(g('~creep_ang_v', self.CREEP_ANG_V))
        self.stop_lead_s     = float(g('~stop_lead_s', self.STOP_LEAD_S))
        self.fine_align_tol  = float(g('~fine_align_tol', self.FINE_ALIGN_TOL))
        self._reset_rate()

        #  go_to tuning
        self.goto_tol        = float(g('~goto_tol', self.GOTO_TOL))
        self.verify_goto_tol = float(g('~verify_goto_tol',
                                       self.VERIFY_GOTO_TOL))
        self.goto_settle     = float(g('~goto_settle', self.GOTO_SETTLE_S))
        self.goto_kp         = float(g('~goto_kp', self.GOTO_KP))
        self.goto_max_v      = float(g('~goto_max_v', self.GOTO_MAX_V))
        self.goto_cam_kp     = float(g('~goto_cam_kp', self.GOTO_CAM_KP))
        self.goto_cam_max_v  = float(g('~goto_cam_max_v', self.GOTO_CAM_MAX_V))
        self.goto_brake_m    = float(g('~goto_brake_m', self.GOTO_BRAKE_M))
        self.goto_approach_v = float(g('~goto_approach_v', self.GOTO_APPROACH_V))
        self.goto_diag_m     = float(g('~goto_diag_m', self.GOTO_DIAG_M))
        self.goto_lidar_clamp = bool(g('~goto_lidar_clamp', self.GOTO_LIDAR_CLAMP))
        self.probe_v         = float(g('~probe_v', self.PROBE_V))
        self.escape_clear    = float(g('~escape_clear_m', self.ESCAPE_CLEAR_M))
        self.escape_v        = float(g('~escape_v', self.ESCAPE_V))
        self.stuck_s         = float(g('~stuck_s', self.STUCK_S))
        # Targets and limits are global: the GUI previews the clamped target.
        floor = tuple(g('/ROOM_FLOOR_MAP_SHAPE'))
        self._floor_m = (floor[1] / 1000.0, floor[0] / 1000.0)   # x, y extent
        robot_size = float(g('/GOTO_ROBOT_SIZE_M', 1.0))
        wall_margin = float(g('/GOTO_WALL_MARGIN_M', 0.2))
        self.goto_bounds = rm.target_bounds(
            g('/GOTO_ROOM_BOUNDS_MM', [0, floor[1], 0, floor[0]]), floor,
            robot_size, wall_margin)
        self._goto_keep_m = robot_size / 2.0 + wall_margin   # centre to wall
        self.goto_targets = dict(g('/GOTO_TARGETS', {}))
        # Room centre (where GoHome ends) in camera coordinates: a constant of
        # the room, so go_to needs no camera frames to anchor the LIDAR.
        centre = g('/GOTO_ROOM_CENTER', None)
        self._room_centre_m = (
            (float(centre[0]) * self._floor_m[0], float(centre[1]) * self._floor_m[1])
            if centre else None)
        # Handedness of the room frame seen from the body: 0 = unknown, so the
        # first go_to probes two axes and logs the value to put in the yaml.
        det = int(g('/GOTO_WORLD_DET', 0))
        self._world_det = det if det in (-1, 1) else 0

        #  Runtime state
        self.enabled = False            # a go_home() routine is running
        self.state   = self.ST_IDLE
        self._latest   = None           # most recent Measurement
        self._prev_ranges = None
        self._fresh_t  = 0.0
        self._last_cmd = Twist()
        self._measuring = 0             # >0 while any phase needs measurements
        self._phase_start = None
        self._cancel = threading.Event()
        self._lock   = threading.Lock()
        self._thread = None
        self._loc    = None             # camera position, metres (x, y)
        self._loc_t  = 0.0              # when it last changed
        self._goto   = None             # go_to target, metres
        self._axes   = None             # body -> room axis map for go_to
        # LIDAR position + _off = camera position, both along the body axes.
        self._off    = None
        self._anchor_ds = None          # camera - LIDAR samples while anchoring
        self._lidar_hist = deque()      # (ros time, LIDAR position) for the lag
        self._fused  = None             # which source GOTO drives on, for the log
        self._wall_clamped = False      # logged once per run
        # Heading tracking. Tuples replaced whole: other threads read them.
        self._track = None              # (epoch, unwrapped wall heading deg, t)
        self._epoch = 0                 # +1 each time the heading is lost
        self._axes_kept = None          # (body -> room map, epoch, heading then)
        self._goto_best = None          # (residual, time) for the stuck check
        self._direct = None             # (axes, heading then, epoch) while driving direct
        self._last_pos = None           # camera-frame m where the last go_to ended
        self._fit = None                # (measurement, wall_fit) for it

        #  Visualisation state
        self.viz_fps = float(g('~viz_fps', self.VIZ_FPS))
        self._viz_on = bool(g('~visualize', self.VIZ_ENABLED))
        self._viz_size = int(g('~viz_size', self.VIZ_SIZE))
        self._viz_fixed_range = float(g('~viz_range_m', self.VIZ_RANGE_M))
        # Text band on top, room below.
        self._viz_band = 200
        self._viz_cx = self._viz_size // 2
        self._viz_cy = self._viz_band + self._viz_size // 2
        self._viz_ppm = None            # set once
        self._viz_room = None           # (width, height) m, averaged slowly
        self._viz_side = {}             # screen side -> wall distance
        self._viz_epoch = 0             # tracker epoch the view follows
        self._viz_q = 0                 # quarter-turn correction after a restart
        self._viz_last_pos = None
        self._viz_last = 0.0
        self._viz_frame = None

        #  ROS I/O
        self.cmd_pub = rospy.Publisher(
            'motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        self.pub_enabled = rospy.Publisher(
            'go_home/enabled', Bool, queue_size=1, latch=True)

        rospy.Subscriber(
            'lidar/scans', LaserScan, self._on_lidar,
            queue_size=1, tcp_nodelay=True)

        # Both behaviours drive motor_control/twist, so they must not overlap.
        self._rotate_active = False
        rospy.Subscriber(
            'rotate/active', Bool, self._on_rotate_active, queue_size=1)

        # Drive-vs-motion overlay: what motor_control commands (any source:
        # joystick, fish, go_home) against how the robot really moves.
        self._drive_cmd = (0.0, 0.0)    # (forward, right) duty
        self._motion_hist = deque()     # (t, forward_m, right_m)
        rospy.Subscriber('motor_control/speed', TwistStamped,
                         self._on_drive_speed, queue_size=1)

        rospy.Subscriber('localization/location', FomaLocation,
                         self._on_location, queue_size=1, tcp_nodelay=True)

        self.srv = rospy.Service(
            '/go_home/enable', SetBool, self._on_enable)
        self.goto_srv = rospy.Service('/go_home/go_to', GoTo, self._on_go_to)
        # motor_control multiplies every command by its speed setting, which
        # the joystick changes; the tuned duty floors only hold at 1.0.
        self._set_speed = rospy.ServiceProxy('motor_control/set_speed', Float)
        # Go Home outranks a quarter turn: 0 quarter turns cancels one.
        self._rotate_srv = rospy.ServiceProxy('rotate/quarter_turns', Float)

        rospy.on_shutdown(self.cancel)

        self._publish_enabled()
        rospy.loginfo(
            "[GoHome] Node started.  max_lin_v=%.2f kp_lin=%.1f "
            "center_tol=%.3f m  align_tol=%.1f deg  align_sign=%+d  viz=%s",
            self.max_lin_v, self.kp_lin, self.center_tol, self.align_tol,
            self.align_sign, self._viz_on)

    #  Service / lifecycle

    def _publish_enabled(self):
        self.pub_enabled.publish(Bool(data=self.enabled))

    def _on_enable(self, req):
        """SetBool entry point.  Returns immediately; the run is threaded."""
        if bool(req.data):
            started = self.start()
            running = self._thread is not None and self._thread.is_alive()
            msg = ("GoHome started" if started
                   else "GoHome already running" if running
                   else "GoHome refused")
        else:
            self.cancel()
            msg = "GoHome stopped"
        rospy.loginfo("[GoHome] %s", msg)
        return SetBoolResponse(success=True, message=msg)

    def _on_go_to(self, req):
        """GoTo entry point.  Returns once started; the run is threaded and
        go_home/enable False stops it."""
        name = req.name.strip()
        if name:
            if name not in self.goto_targets:
                msg = "unknown target '%s' (known: %s)" % (
                    name, ', '.join(sorted(self.goto_targets)) or 'none')
                rospy.logwarn("[GoHome] go_to refused: %s", msg)
                return GoToResponse(False, msg, req.x, req.y)
            x, y = (float(v) for v in self.goto_targets[name])
        else:
            x, y = req.x, req.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return GoToResponse(False, "target is not a number", x, y)
        cx, cy, moved = rm.clamp_target(x, y, self.goto_bounds)
        if moved:
            rospy.loginfo("[GoHome] go_to target (%.3f, %.3f) clamped to "
                          "(%.3f, %.3f)", x, y, cx, cy)
        started = self.start((cx, cy))
        msg = ("go_to (%.3f, %.3f) started" % (cx, cy) if started
               else "go_to refused: already running")
        rospy.loginfo("[GoHome] %s", msg)
        return GoToResponse(started, msg, cx, cy)

    def arm(self):
        """Clear a latched cancel so the phases may run again."""
        self._cancel.clear()

    def _on_rotate_active(self, msg):
        self._rotate_active = bool(msg.data)

    def _cancel_rotation(self, wait_s=3.0):
        """Stop a running quarter turn so Go Home can take the motors."""
        rospy.loginfo("[GoHome] cancelling the rotation in progress")
        try:
            self._rotate_srv.wait_for_service(timeout=0.5)
            self._rotate_srv(0.0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("[GoHome] could not cancel rotation: %s", e)
        end = time.monotonic() + wait_s
        while self._rotate_active and time.monotonic() < end:
            rospy.sleep(0.05)
        return not self._rotate_active

    def start(self, target=None):
        """Kick off go_home(), or go_to(*target), on a worker thread.  False
        if already running."""
        if self._rotate_active and not self._cancel_rotation():
            rospy.logwarn("[GoHome] refused: the rotation did not stop")
            return False
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._cancel.clear()
            self._thread = threading.Thread(
                target=self._routine, args=(target,), name='go_home',
                daemon=True)
            self._thread.start()
            return True

    def cancel(self, join_timeout=1.0):
        """Ask a running routine to stop and bring the robot to rest."""
        self._cancel.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=join_timeout)
        self._hard_stop()

    def _routine(self, target):
        if target is None:
            result = self.go_home()
        else:
            result = self.go_to(*target)
        level = rospy.loginfo if result.ok else rospy.logwarn
        level("[GoHome] routine finished: %s", result)

    #  Measurement

    def _get_dist(self, ranges_m, angle_deg):
        """Median of ±BEAM_HALF_WIN rays around *angle_deg*."""
        n = len(ranges_m)
        indices = [(int(round(angle_deg)) + i) % n
                   for i in range(-self.BEAM_HALF_WIN,
                                   self.BEAM_HALF_WIN + 1)]
        vals = ranges_m[indices]
        good = vals[(vals > self.RANGE_MIN_M)
                     & (vals < self.RANGE_MAX_M)
                     & np.isfinite(vals)]
        if len(good) < 3:
            return None
        return float(np.median(good))

    def _compute_alignment_error(self, ranges_m):
        """Body yaw vs the walls -> (degrees, valid, confidence).

        Position independent, so the robot can square up before centring
        (the old symmetry probes needed it centred and read 0 at 45 deg).
        """
        heading, conf = geom.wall_heading_fit(ranges_m, r_min=self.RANGE_MIN_M,
                                          r_max=self.RANGE_MAX_M)
        if heading is None or conf < self.min_conf:
            return 0.0, False, conf
        return geom.fold_angle(heading - self.heading_offset), True, conf

    def _wall_distances(self, ranges_m):
        """Cardinal wall distances (forward, backward, left, right) in metres."""
        return (self._get_dist(ranges_m, 0),     # forward wall
                self._get_dist(ranges_m, 180),   # backward wall
                self._get_dist(ranges_m, 90),    # left wall
                self._get_dist(ranges_m, 270))   # right wall

    def _compute_center_error(self, walls):
        """(error_x, error_y, dist) from the four wall distances.

        error_x = (right - left)/2 -> linear.x  (+ = right)
        error_y = (fwd - back)/2   -> linear.y  (+ = forward)
        Measured along the robot's own axes, so it stays valid while crooked.
        """
        d_fwd, d_bwd, d_left, d_right = walls

        if None in walls:
            return 0.0, 0.0, 0.0, False

        # linear.x > 0 → rightward (toward 270°): drive right when right wall is farther
        error_x = (d_right - d_left) / 2.0
        # linear.y > 0 → forward (toward 0°): drive forward when forward wall is farther
        error_y = (d_fwd - d_bwd) / 2.0
        dist    = math.hypot(error_x, error_y)
        return error_x, error_y, dist, True

    def _measure(self, ranges_m, fresh_t):
        walls = self._wall_distances(ranges_m)
        err_x, err_y, dist, ctr_valid = self._compute_center_error(walls)
        ang_err, ang_valid, ang_conf = self._compute_alignment_error(ranges_m)
        return Measurement(ranges_m, walls, err_x, err_y, dist, ctr_valid,
                           ang_err, ang_valid, ang_conf, fresh_t)

    def _on_lidar(self, msg):
        """Sensor sink only — never blocks and never drives the motors."""
        ranges_m = np.array(msg.ranges, dtype=np.float64) / 1000.0
        self._track_heading(ranges_m)
        # ~0.5 ms a scan at 100 Hz: only sample when a phase or the overlay needs it.
        if not self._measuring and not self._viz_due():
            return

        if self._prev_ranges is None or not np.array_equal(ranges_m,
                                                           self._prev_ranges):
            self._fresh_t = time.monotonic()
        self._prev_ranges = ranges_m
        m = self._measure(ranges_m, self._fresh_t)
        if self._direct is not None:
            # Perpendicular wall distances: right however the robot is turned.
            self._fit = (m, geom.wall_fit(ranges_m, r_min=self.RANGE_MIN_M,
                                          r_max=self.RANGE_MAX_M))
        self._latest = m
        if self._axes is not None:
            self._track_lidar(m)
        self._render_viz(m, self._last_cmd, self._elapsed())

    def _track_heading(self, ranges_m):
        """Unwrap the raw wall heading scan to scan, so the turns made between
        routines (joystick, quarter turns, fish) keep the axis map known."""
        now = time.monotonic()
        track = self._track
        if track is not None and now - track[2] > self.TRACK_GAP_S:
            if self._axes_kept is not None:
                rospy.logwarn("[GoHome] wall heading lost for %.1f s: the room "
                              "axes will be found again", now - track[2])
            self._epoch += 1
            self._track = track = None
        heading, conf = geom.wall_heading(ranges_m, r_min=self.RANGE_MIN_M,
                                          r_max=self.RANGE_MAX_M)
        if heading is None or conf < self.min_conf:
            return
        if track is None:
            self._track = (self._epoch, heading, now)
            return
        step = geom.unwrap_folded_delta(heading - geom.fold_angle(track[1]))
        # A short gap (heading untrusted for a moment) is bridged, not reset.
        max_step = max(self.TRACK_MAX_STEP_DEG,
                       min(44.0, self.TRACK_MAX_DPS * (now - track[2])))
        if abs(step) > max_step:
            return
        self._track = (track[0], track[1] + step, now)

    def _tracked_axes(self):
        """(body -> room map at the kept heading, that heading, the heading
        now), all from the same tracking epoch, or None."""
        track, kept = self._track, self._axes_kept
        if track is None or kept is None or kept[1] != track[0]:
            return None
        return kept[0], kept[2], track[1]

    @staticmethod
    def _square_axes(tracked):
        """The integer axis map for the robot squared up now."""
        axes, yaw_ref, yaw = tracked
        # The walls turn against the body: a CCW turn lowers the heading.
        k = int(round(-(yaw - yaw_ref) / 90.0))
        quarter = np.array([[0, -1], [1, 0]], dtype=int)    # body, 90 deg CCW
        return axes @ np.linalg.matrix_power(quarter, k % 4)

    def _keep_axes(self, axes):
        track = self._track
        if track is not None:
            self._axes_kept = (axes, track[0], track[1])

    def _collect(self, n, window_s):
        """Up to *n* distinct fresh measurements within *window_s* seconds."""
        out = []
        last_t = None
        end = time.monotonic() + window_s
        while time.monotonic() < end and len(out) < n:
            if self._cancel.is_set() or rospy.is_shutdown():
                break
            m = self._latest
            if m is not None and m.fresh_t != last_t:
                last_t = m.fresh_t
                out.append(m)
            rospy.sleep(0.03)
        return out

    def _elapsed(self):
        if self._phase_start is None:
            return 0.0
        return time.monotonic() - self._phase_start

    def _begin_measuring(self):
        """Ask the scan callback to keep deriving measurements."""
        with self._lock:
            self._measuring += 1

    def _end_measuring(self):
        """Release one claim; True while some other caller still needs them."""
        with self._lock:
            self._measuring = max(0, self._measuring - 1)
            return self._measuring > 0

    #  Motor helpers

    def _publish(self, cmd):
        self._last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def _hard_stop(self):
        zero = Twist()
        for _ in range(3):
            self._publish(zero)
            rospy.sleep(0.05)

    def stop_robot(self):
        self._publish(Twist())

    #  Phase primitives. _run_phase holds what they share, sense/drive what they do not.

    def _sense_center(self, m):
        return m.ctr_valid, m.dist

    def _drive_center(self, m):
        """P-control toward the room centre along the robot's own axes."""
        cmd = Twist()
        speed = self.kp_lin * m.dist
        if 0 < speed < self.min_lin_v:      # deadband: beat static friction
            speed = self.min_lin_v
        speed = min(speed, self.max_lin_v)
        cmd.linear.x = (m.err_x / m.dist) * speed
        cmd.linear.y = (m.err_y / m.dist) * speed
        return cmd

    def _guard_center(self, m):
        """Abort centring once the heading has walked off: cardinal distances
        taken along a crooked body point the robot at the wrong centre."""
        if m.ang_valid and abs(m.ang_err) > self.center_drift:
            rospy.logwarn("[GoHome] heading drifted to %+.1f deg while "
                          "driving -> re-align", m.ang_err)
            return Status.DRIFT
        return None

    def _sense_align(self, m):
        return m.ang_valid, abs(m.ang_err)

    def _align_duty(self, ang_err, magnitude):
        """angular.z that turns *against* a heading error of this sign."""
        return -self.align_sign * math.copysign(magnitude, ang_err)

    def _reset_rate(self):
        self._rate = 0.0                # filtered heading rate, deg/s
        self._rate_h = None
        self._rate_t = None
        self._best_abs = None           # smallest |error| this phase
        self._boost = 0.0               # floor raise after a stall
        self._prev_err = None
        self._stall_since = None

    def _check_sign(self, m):
        """Flip align_sign once if the turn goes the wrong way: the error
        grows past its best, or wraps through the +/-45 fold."""
        err = m.ang_err
        mag, side = abs(err), err > 0
        prev = self._prev_err
        self._prev_err = err
        if self._best_abs is None:
            self._best_abs = mag
            self._crossed = False
            return
        if prev is not None and (prev > 0) != side:
            if mag < 20.0 and abs(prev) < 20.0:
                self._crossed = True    # passed the target: the sign works
            elif mag > 30.0 and abs(prev) > 30.0:
                self._flip_sign("wrapped through 45 deg")
                return
        if mag < self._best_abs:
            self._best_abs = mag
        elif (not self._crossed
              and mag > self._best_abs + self.SIGN_PROBE_DEG * 3.0):
            self._flip_sign("error grew")

    def _flip_sign(self, why):
        if not self.sign_autodetect or self._sign_checked or self._crossed:
            return
        self.align_sign = -self.align_sign
        self._sign_checked = True
        self._best_abs = None
        self._rate = 0.0
        rospy.logwarn("[GoHome] heading moved the wrong way (%s): align_sign "
                      "flipped to %+d. Set ALIGN_SIGN in foma_params.yaml.",
                      why, self.align_sign)

    def _update_rate(self, m):
        """Heading rate over RATE_WINDOW_S: scan-to-scan (10 ms) noise
        reads as tens of deg/s and hides a stall."""
        if self._rate_h is None:
            self._rate_h, self._rate_t = m.ang_err, m.fresh_t
            return
        dt = m.fresh_t - self._rate_t
        if dt < self.RATE_WINDOW_S:
            return
        d = geom.unwrap_folded_delta(m.ang_err - self._rate_h)
        self._rate += self.RATE_ALPHA * (d / dt - self._rate)
        self._rate_h, self._rate_t = m.ang_err, m.fresh_t

    def _drive_align(self, m):
        """Smooth turn: break away at min_ang_v, creep once moving, and coast
        early enough that the ramp-down and lidar lag land on the target."""
        self._update_rate(m)
        self._check_sign(m)
        cmd = Twist()
        # The scan is a revolution old and the rate a window old: project
        # the error forward before deciding (as rotate_node does).
        err = m.ang_err + self._rate * self.LAG_S
        # deg/s the error is shrinking at (negative = growing).
        closing = -self._rate * math.copysign(1.0, err)
        if closing > 0 and abs(err) <= closing * self.stop_lead_s:
            return cmd                  # coast in
        if abs(err) <= self.align_tol and closing >= 0:
            return cmd                  # projected inside: let it settle
        moving = abs(self._rate) > self.MOVING_DPS
        now = time.monotonic()
        if moving:
            self._stall_since = None
        elif self._stall_since is None:
            self._stall_since = now
        elif now - self._stall_since > self.STALL_S:
            # Commanded but not turning: this floor needs more duty.
            self._boost = min(self._boost + self.STALL_STEP,
                              self.max_ang_v - self.min_ang_v)
            self._stall_since = now
        # The boost is for breaking away only: once turning, kinetic friction
        # is lower and a boosted floor just overshoots.
        floor = self.creep_ang_v if moving else self.min_ang_v + self._boost
        mag = min(max(self.kp_ang * abs(err), floor), self.max_ang_v)
        cmd.angular.z = self._align_duty(err, mag)
        return cmd

    def _run_phase(self, label, state, sense, drive, tol, settle_s, timeout,
                   guard=None):
        """Closed loop shared by every phase.  *guard(m)* may return a Status
        to end the phase early."""
        prev_state = self.state
        self._begin_measuring()
        self.state = state
        started = time.monotonic()
        self._phase_start = started
        deadline = started + timeout
        rate = rospy.Rate(self.control_hz)
        inside_since = None
        residual = float('inf')

        try:
            while not rospy.is_shutdown():
                now = time.monotonic()

                if self._cancel.is_set():
                    self._hard_stop()
                    return PhaseResult(Status.CANCELLED, residual,
                                       now - started)
                if now > deadline:
                    self._hard_stop()
                    rospy.logwarn("[GoHome] %s timed out after %.1f s "
                                  "(residual %.4f)", label, now - started,
                                  residual)
                    return PhaseResult(Status.TIMEOUT, residual, now - started)

                m = self._latest
                if m is None:
                    # No scan yet — hold still rather than driving blind.
                    if now - started > self.SCAN_WAIT_S:
                        self._hard_stop()
                        return PhaseResult(Status.NO_SCAN, residual,
                                           now - started)
                    self._publish(Twist())
                    rate.sleep()
                    continue

                # Before the validity check: a lost input reads as invalid
                # and only a guard can end the phase on it.
                if guard is not None:
                    stop = guard(m)
                    if stop is not None:
                        self._hard_stop()
                        return PhaseResult(stop, residual, now - started)

                valid, residual = sense(m)
                if not valid:
                    self._publish(Twist())
                    inside_since = None
                    rate.sleep()
                    continue

                # Hysteresis: once settling, noise just above tol must not
                # restart the clock and send the robot hunting.
                inside = residual <= tol or (
                    inside_since is not None
                    and residual <= tol * self.release_scale)
                if inside:
                    self._publish(Twist())      # coast to a stop and settle
                    if inside_since is None:
                        inside_since = now
                    elif now - inside_since >= settle_s:
                        self._hard_stop()
                        rospy.loginfo(
                            "[GoHome] %s done: residual=%.4f after %.1f s",
                            label, residual, now - started)
                        return PhaseResult(Status.SUCCESS, residual,
                                           now - started)
                elif now - m.fresh_t > self.STALE_S:
                    # Never drive on a frozen buffer.
                    self._publish(Twist())
                    inside_since = None
                    if now - m.fresh_t > self.SCAN_WAIT_S:
                        self._hard_stop()
                        rospy.logwarn("[GoHome] %s: lidar scan stopped "
                                      "changing", label)
                        return PhaseResult(Status.NO_SCAN, residual,
                                           now - started)
                    rospy.logwarn_throttle(
                        1.0, "[GoHome] %s: scan stale, holding", label)
                else:
                    inside_since = None
                    self._publish(drive(m))
                    rospy.loginfo_throttle(
                        2.0, "[GoHome] %s residual=%.4f (tol %.4f) %.0fs",
                        label, residual, tol, now - started)

                rate.sleep()

            self._hard_stop()
            return PhaseResult(Status.CANCELLED, residual,
                               time.monotonic() - started)
        finally:
            still_driving = self._end_measuring()
            self.state = prev_state if still_driving else self.ST_IDLE
            self._phase_start = None

    def drive_to_center(self, tol=None, settle_s=None, timeout=None):
        """Translate the robot to the centre of the room."""
        return self._run_phase(
            'CENTER', self.ST_CENTER, self._sense_center, self._drive_center,
            self.center_tol if tol is None else tol,
            self.center_settle if settle_s is None else settle_s,
            self.timeout_s if timeout is None else timeout,
            guard=self._guard_center)

    def align_to_axes(self, tol=None, settle_s=None, timeout=None):
        """Rotate in place until the robot is square with the room axes."""
        self._reset_rate()
        return self._run_phase(
            'ALIGN', self.ST_ALIGN, self._sense_align, self._drive_align,
            self.align_tol if tol is None else tol,
            self.align_settle if settle_s is None else settle_s,
            self.timeout_s if timeout is None else timeout)

    def _reset_speed(self):
        try:
            self._set_speed.wait_for_service(timeout=0.5)
            self._set_speed(1.0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("[GoHome] could not reset motor speed to 1.0 "
                          "(%s); duty floors may not hold", e)

    def _verify(self):
        """Median of several fresh scans -> (centred, square, dist, ang)."""
        ms = self._collect(self.VERIFY_SCANS, self.VERIFY_WINDOW_S)
        dists = [m.dist for m in ms if m.ctr_valid]
        angs = [m.ang_err for m in ms if m.ang_valid]
        dist = float(np.median(dists)) if dists else float('inf')
        ang = float(np.median(angs)) if angs else float('inf')
        return (dist < self.verify_center_tol, abs(ang) < self.verify_align_tol,
                dist, ang)

    def go_home(self, timeout=None, passes=None):
        """GOHOME: square up, drive to the centre, square up finely, verify.
        Aligning first works from any pose; cardinal distances read along a
        crooked body point at the wrong centre."""
        budget = self.timeout_s if timeout is None else timeout
        passes = self.passes if passes is None else passes
        deadline = time.monotonic() + budget

        started = time.monotonic()
        result = PhaseResult(Status.UNSETTLED, float('inf'), 0.0)

        def out_of_time():
            return PhaseResult(Status.TIMEOUT, result.error,
                               time.monotonic() - started)

        with self._run_scope():
            self._reset_speed()

            for attempt in range(1, passes + 1):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return out_of_time()

                self.escape_walls()
                result = self.align_to_axes(timeout=deadline - time.monotonic())
                if not result.ok:
                    return result

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return out_of_time()

                result = self.drive_to_center(timeout=remaining)
                if result.status == Status.DRIFT:
                    continue            # next pass re-aligns first
                if not result.ok:
                    return result

                # Square up in place once more. Usually already done: the
                # heading hold kept the robot square while it drove.
                fine = self.align_to_axes(
                    tol=self.fine_align_tol,
                    timeout=min(deadline - time.monotonic(),
                                self.FINE_ALIGN_TIMEOUT_S))
                if fine.status in (Status.CANCELLED, Status.NO_SCAN):
                    return fine

                # Confirm rather than assume — this is what guarantees the end state.
                self.state = self.ST_VERIFY
                centred, square, dist, ang = self._verify()
                if self._cancel.is_set():
                    return PhaseResult(Status.CANCELLED, dist,
                                       time.monotonic() - started)
                if centred and square:
                    rospy.loginfo(
                        "[GoHome] VERIFY PASS  dist=%.3f m  ang=%+.1f deg  "
                        "(pass %d/%d)", dist, ang, attempt, passes)
                    self._log_room_centre()
                    return PhaseResult(Status.SUCCESS, dist,
                                       time.monotonic() - started)

                rospy.loginfo(
                    "[GoHome] VERIFY FAIL  dist=%.3f m (%s)  ang=%+.1f deg (%s) "
                    "— repeating (pass %d/%d)",
                    dist, "ok" if centred else "off",
                    ang, "ok" if square else "off", attempt, passes)

            return PhaseResult(Status.UNSETTLED, result.error,
                               time.monotonic() - started)

    def _log_room_centre(self):
        """The camera position at the room centre, for GOTO_ROOM_CENTER."""
        loc = self._median_loc(self.VERIFY_LOCS, 1.0)
        if loc is None:
            return
        if self._room_centre_m is None:
            self._room_centre_m = loc       # until a go_to anchor refines it
        rospy.loginfo("[GoHome] room centre in camera coordinates: "
                      "GOTO_ROOM_CENTER: [%.4f, %.4f]",
                      loc[0] / self._floor_m[0], loc[1] / self._floor_m[1])

    @contextlib.contextmanager
    def _run_scope(self):
        """What every routine shares: the enabled flag, measurements on, and
        the robot left at rest however it ends."""
        # Does NOT clear _cancel: start() arms the run before the worker exists.
        self.enabled = True
        self._publish_enabled()
        self._latest = None             # never start on a previous run's scan
        self._begin_measuring()
        try:
            yield
        finally:
            self._end_measuring()
            self.state = self.ST_IDLE
            self.enabled = False
            self._publish_enabled()
            self._hard_stop()

    #  go_to: the camera gives the position, the LIDAR keeps the robot square.

    def _on_location(self, msg):
        """Camera position in metres.  localization re-publishes its last fix
        when it loses the robot, so freshness is when the value last changed."""
        if msg.world.x == 0.0 and msg.world.y == 0.0:
            return                      # no fix since localization started
        loc = (msg.world.x * self._floor_m[0], msg.world.y * self._floor_m[1])
        if loc == self._loc:
            return
        self._loc_t = time.monotonic()
        self._loc = loc
        if self._axes is None:
            return
        samples = self._anchor_ds
        if samples is None:
            return
        # Compare with the LIDAR position at the camera's capture time.
        pb = self._lidar_at(msg.header.stamp.to_sec())
        if pb is not None:
            samples.append(self._axes.T @ np.array(loc) - pb)

    def _anchor_fixed(self):
        """Anchor from GOTO_ROOM_CENTER: the LIDAR position is 0 there.  The
        camera, if it sees the robot now, is only a cross-check."""
        self._off = self._axes.T @ np.array(self._room_centre_m)
        rospy.loginfo("[GoHome] anchor from GOTO_ROOM_CENTER")
        loc = self._median_loc(3, 1.0)
        pb = self._lidar_pos(self._latest) if self._latest is not None else None
        if loc is not None and pb is not None:
            gap = float(np.hypot(*(self._axes.T @ np.array(loc) - (pb + self._off))))
            level = rospy.logwarn if gap > self.CAMERA_CHECK_M else rospy.loginfo
            level("[GoHome] anchor: camera vs GOTO_ROOM_CENTER %.3f m", gap)

    def _anchor(self):
        """Fix the LIDAR -> room offset from camera frames, robot at rest.
        False (drive on the camera alone) without enough frames."""
        self._anchor_ds = []
        end = time.monotonic() + self.ANCHOR_WINDOW_S
        while (time.monotonic() < end and len(self._anchor_ds) < self.ANCHOR_N
               and not self._cancel.is_set() and not rospy.is_shutdown()):
            rospy.sleep(0.02)
        samples, self._anchor_ds = self._anchor_ds, None
        if len(samples) < 3:
            rospy.logwarn("[GoHome] anchor: %d camera frames, driving on the "
                          "camera alone", len(samples))
            return False
        ds = np.array(samples)
        off = np.median(ds, axis=0)
        spread = float(np.median(np.hypot(*(ds - off).T)))
        level = rospy.logwarn if spread > self.ANCHOR_SPREAD_M else rospy.loginfo
        level("[GoHome] anchor from %d camera frames, spread %.3f m",
              len(samples), spread)
        self._off = off
        # LIDAR position 0 = the room centre: exact, unlike GoHome's end point.
        centre = self._axes @ off
        rospy.loginfo("[GoHome] room centre in camera coordinates: "
                      "GOTO_ROOM_CENTER: [%.4f, %.4f]  (best from a start "
                      "near the middle)", centre[0] / self._floor_m[0],
                      centre[1] / self._floor_m[1])
        return True

    @staticmethod
    def _lidar_pos(m):
        """Position along the body (right, forward) from the wall distances,
        metres from the room centre, or None if a wall is not seen."""
        if not m.ctr_valid:
            return None
        d_fwd, d_bwd, d_left, d_right = m.walls
        return np.array([(d_left - d_right) / 2.0, (d_bwd - d_fwd) / 2.0])

    def _track_lidar(self, m):
        """Keep the LIDAR position history, and keep the fused position
        continuous when a distance jumps to another wall or obstacle."""
        pb = self._lidar_pos(m)
        if pb is None:
            return
        hist = self._lidar_hist
        if hist and self._off is not None:
            jump = pb - hist[-1][1]
            if np.hypot(*jump) > self.LIDAR_JUMP_M:
                self._off = self._off - jump
        t = rospy.get_time()
        hist.append((t, pb))
        while hist and t - hist[0][0] > self.LIDAR_HIST_S:
            hist.popleft()

    def _lidar_at(self, t):
        """LIDAR position closest to ros time *t*, None outside the history."""
        best = None
        for ti, pb in self._lidar_hist:
            if best is None or abs(ti - t) < abs(best[0] - t):
                best = (ti, pb)
        if best is None or abs(best[0] - t) > 0.05:
            return None
        return best[1]

    def _fresh_loc(self, max_age=None):
        age = self.LOC_STALE_S if max_age is None else max_age
        if self._loc is None or time.monotonic() - self._loc_t > age:
            return None
        return self._loc

    def _median_loc(self, n, window_s, min_n=1):
        """Median of up to *n* fresh positions within *window_s*, or None
        with fewer than *min_n*."""
        out = []
        last_t = None
        end = time.monotonic() + window_s
        while time.monotonic() < end and len(out) < n:
            if self._cancel.is_set() or rospy.is_shutdown():
                break
            loc = self._fresh_loc()
            if loc is not None and self._loc_t != last_t:
                last_t = self._loc_t
                out.append(loc)
            rospy.sleep(0.02)
        if len(out) < max(1, min_n):
            return None
        return tuple(float(v) for v in np.median(np.array(out), axis=0))

    def _free_axes(self):
        """Body axes (right, forward) from the most free space to the least."""
        m = self._latest
        d_fwd, d_bwd, d_left, d_right = (
            (d or 0.0) for d in (m.walls if m is not None else (None,) * 4))
        room = {(1, 0): d_right, (0, 1): d_fwd, (-1, 0): d_left, (0, -1): d_bwd}
        return sorted(room, key=room.get, reverse=True)

    def _probe(self, axis):
        """Drive a few cm along one body axis -> the room axis the camera saw
        it move along, as a unit (x, y), or None."""
        # The LIDAR measures how far (precise), the camera only which way: a
        # single camera frame can jump by cm when one tag drops out.
        start = self._median_loc(self.PROBE_LOCS, self.PROBE_LOC_WINDOW_S,
                                 self.PROBE_MIN_LOCS)
        if start is None:
            rospy.logwarn("[GoHome] probe: no camera position")
            return None
        m0 = self._latest
        lidar0 = self._lidar_pos(m0) if m0 is not None else None
        cmd = Twist()
        cmd.linear.x = axis[0] * self.probe_v
        cmd.linear.y = axis[1] * self.probe_v
        rate = rospy.Rate(self.control_hz)
        end = time.monotonic() + self.PROBE_TIMEOUT_S
        while (time.monotonic() < end and not rospy.is_shutdown()
               and not self._cancel.is_set()):
            m = self._latest
            pb = self._lidar_pos(m) if m is not None else None
            if lidar0 is not None and pb is not None:
                if np.hypot(*(pb - lidar0)) >= self.PROBE_DIST_M:
                    break
            else:
                loc = self._loc         # no walls seen: the camera alone
                if loc is not None and math.hypot(
                        loc[0] - start[0], loc[1] - start[1]) >= self.PROBE_DIST_M:
                    break
            self._publish(cmd)
            rate.sleep()
        self._hard_stop()
        if self._cancel.is_set():
            return None
        rospy.sleep(self.PROBE_SETTLE_S)
        stop = self._median_loc(self.PROBE_LOCS, self.PROBE_LOC_WINDOW_S,
                                self.PROBE_MIN_LOCS)
        if stop is None:
            rospy.logwarn("[GoHome] probe: lost the camera position")
            return None
        dx, dy = stop[0] - start[0], stop[1] - start[1]
        moved = math.hypot(dx, dy)
        m1 = self._latest
        pb1 = self._lidar_pos(m1) if m1 is not None else None
        lidar_moved = (float(np.hypot(*(pb1 - lidar0)))
                       if lidar0 is not None and pb1 is not None else float('nan'))
        if moved < self.PROBE_MIN_M:
            rospy.logwarn("[GoHome] probe along body %s: camera saw %.3f m, "
                          "LIDAR %.3f m", axis, moved, lidar_moved)
            return None
        ang = math.degrees(math.atan2(dy, dx))
        k = int(round(ang / 90.0))
        off = ang - 90.0 * k
        if abs(off) > self.PROBE_AXIS_TOL_DEG:
            rospy.logwarn("[GoHome] probe moved %.0f deg off any room axis",
                          off)
            return None
        unit = ((1, 0), (0, 1), (-1, 0), (0, -1))[k % 4]
        rospy.loginfo("[GoHome] probe: body %s -> room %s (%.3f m, %+.0f deg "
                      "off axis)", axis, unit, moved, off)
        return unit

    def _find_axes(self):
        """Body -> room axis map from probe moves.  One probe fixes it once
        the room frame's handedness is known (/GOTO_WORLD_DET), else two."""
        self.state = self.ST_PROBE
        cands = [r for r in AXIS_MAPS
                 if self._world_det in (0, int(round(np.linalg.det(r))))]
        tried = []
        for axis in self._free_axes():
            if len(cands) == 1:
                break
            # A second probe must be perpendicular to tell the frames apart.
            if any(axis[0] * t[0] + axis[1] * t[1] for t in tried):
                continue
            unit = self._probe(axis)
            if unit is None:
                return None
            tried.append(axis)
            cands = [r for r in cands if tuple(int(v) for v in r @ axis) == unit]
            if not cands:
                rospy.logwarn("[GoHome] probes disagree on the room axes")
                return None
        if len(cands) != 1:
            return None
        axes = cands[0]
        if self._world_det == 0:
            self._world_det = int(round(np.linalg.det(axes)))
            rospy.logwarn("[GoHome] room frame handedness found: set "
                          "GOTO_WORLD_DET: %d in foma_params.yaml so go_to "
                          "probes once", self._world_det)
        return axes

    def _axes_from_fix(self, tracked=None):
        """Axis map whose LIDAR position best matches one camera fix (room
        centre known).  Checks *tracked* if given.  Falls back to *tracked*
        when the camera is missing or the maps are too close to tell."""
        m = self._latest
        pb = self._lidar_pos(m) if m is not None else None
        if pb is None or self._room_centre_m is None:
            return tracked
        loc = self._median_loc(5, 2.0, min_n=2)
        if loc is None:
            return tracked
        centre, cam = np.array(self._room_centre_m), np.array(loc)
        cands = [r for r in AXIS_MAPS
                 if self._world_det in (0, int(round(np.linalg.det(r))))]
        gaps = sorted((float(np.hypot(*(centre + r @ pb - cam))), i)
                      for i, r in enumerate(cands))
        (best, i), runner = gaps[0], gaps[1][0]
        if tracked is not None:
            gap = float(np.hypot(*(centre + tracked @ pb - cam)))
            if gap <= self.AXES_FIT_M or runner - best < self.AXES_MARGIN_M:
                rospy.loginfo("[GoHome] tracked room axes, camera agrees "
                              "within %.2f m", gap)
                return tracked
            rospy.logwarn("[GoHome] tracked room axes off by %.2f m on the "
                          "camera: replaced", gap)
        if best > self.AXES_FIT_M or runner - best < self.AXES_MARGIN_M:
            rospy.loginfo("[GoHome] camera fix cannot pick the room axes "
                          "(best %.2f m, next %.2f m): probing", best, runner)
            return tracked
        rospy.loginfo("[GoHome] room axes from one camera fix (%.2f m, next "
                      "%.2f m)", best, runner)
        return cands[i]

    def _near_walls(self, clear):
        """Body (right, forward) directions and depths of returns within *clear*."""
        m = self._latest
        if m is None:
            return None, None
        r = m.ranges
        idx = np.flatnonzero(np.isfinite(r) & (r > self.RANGE_MIN_M) & (r < clear))
        a = np.radians(idx.astype(np.float64) - self.heading_offset)
        return np.column_stack((-np.sin(a), np.cos(a))), clear - r[idx]

    def _wait_scan(self):
        """Wait for the run's first scan (_run_scope clears _latest)."""
        end = time.monotonic() + self.SCAN_WAIT_S
        while (self._latest is None and time.monotonic() < end
               and not self._cancel.is_set() and not rospy.is_shutdown()):
            rospy.sleep(0.01)

    def escape_walls(self, clear=None, timeout=None):
        """Move away from anything within *clear*, so the robot can turn."""
        clear = self.escape_clear if clear is None else clear
        self._wait_scan()
        end = time.monotonic() + (self.ESCAPE_TIMEOUT_S if timeout is None else timeout)
        rate = rospy.Rate(self.control_hz)
        started = False
        while not rospy.is_shutdown() and not self._cancel.is_set():
            dirs, depth = self._near_walls(clear)
            if dirs is None or not len(depth):
                break
            if not started:
                started = True
                rospy.loginfo("[GoHome] %.2f m from a wall: backing off before "
                              "turning", clear - float(depth.max()))
            if time.monotonic() > end:
                rospy.logwarn("[GoHome] could not back off the wall")
                break
            away = -(dirs * depth[:, None]).sum(axis=0)
            mag = float(np.hypot(*away))
            if mag < 1e-6:
                break                   # boxed in evenly
            away /= mag
            # Drop small components: below friction they only stall.
            away[np.abs(away) < 0.3] = 0.0
            away *= self.escape_v / max(np.abs(away).max(), 1e-6)
            cmd = Twist()
            cmd.linear.x, cmd.linear.y = float(away[0]), float(away[1])
            self._publish(cmd)
            rate.sleep()
        if started:
            self._hard_stop()

    def _goto_error(self, m):
        """(target - position along the body (right, forward) in metres,
        fused), or (None, False).  Fused = the LIDAR position anchored by the
        camera, lag free.  Else the camera alone."""
        target = self._axes.T @ np.array(self._goto)
        pb = self._lidar_pos(m)
        if self._off is not None and pb is not None:
            if self.goto_lidar_clamp:
                target = self._clamp_to_walls(target, pb, m)
            return target - (pb + self._off), True
        loc = self._fresh_loc()
        if loc is None:
            return None, False
        return target - self._axes.T @ np.array(loc), False

    def _clamp_to_walls(self, target, pb, m):
        """Target (body axes, camera frame) kept _goto_keep_m inside the walls
        the LIDAR sees.  The bounds parameter may not match the real room."""
        d_fwd, d_bwd, d_left, d_right = m.walls
        half = np.array([(d_left + d_right) / 2.0 - self._goto_keep_m,
                         (d_fwd + d_bwd) / 2.0 - self._goto_keep_m])
        half = np.maximum(half, 0.0)
        local = target - self._off              # in LIDAR coordinates
        clamped = np.clip(local, -half, half) + self._off
        moved = float(np.hypot(*(clamped - target)))
        if not self._wall_clamped and moved > self.goto_tol:
            # Once per run, and kept as the target so the verify agrees.
            self._wall_clamped = True
            self._goto = tuple(float(v) for v in self._axes @ clamped)
            rospy.logwarn("[GoHome] GOTO target moved %.2f m away from the "
                          "walls: they are closer than GOTO_ROOM_BOUNDS_MM "
                          "says", moved)
        return clamped

    def _sense_goto(self, m):
        err, _ = self._goto_error(m)
        if err is None:
            return False, float('inf')
        return True, float(math.hypot(err[0], err[1]))

    def _drive_goto(self, m):
        """P-control toward the target along the robot's own axes."""
        cmd = Twist()
        err, fused = self._goto_error(m)
        if err is None:
            return cmd
        if fused != self._fused:
            self._fused = fused
            rospy.loginfo("[GoHome] GOTO driving on %s", "the anchored LIDAR"
                          if fused else "the camera alone (slower)")
        kp, max_v = ((self.goto_kp, self.goto_max_v) if fused
                     else (self.goto_cam_kp, self.goto_cam_max_v))
        dist = math.hypot(err[0], err[1])
        speed = min(max(kp * dist, self.min_lin_v), max_v)
        if dist < self.goto_brake_m:
            speed = min(speed, self.goto_approach_v)
        out = [(e / dist) * speed for e in err]
        # Each wheel pair only moves above static friction: a small share of
        # a diagonal would stall and turn it into one axis after the other.
        if dist > self.goto_diag_m:
            for i, e in enumerate(err):
                if abs(e) > self.goto_tol and abs(out[i]) < self.min_lin_v:
                    out[i] = math.copysign(self.min_lin_v, e)
        cmd.linear.x, cmd.linear.y = out
        return cmd

    def _guard_goto(self, m):
        """Re-align on drift.  Give up once the camera has lost the robot,
        but only when driving on it: anchored, the LIDAR does not need it."""
        anchored = self._off is not None and self._lidar_pos(m) is not None
        if not anchored and self._fresh_loc(self.LOC_WAIT_S) is None:
            rospy.logwarn("[GoHome] GOTO: no camera position for %.1f s",
                          self.LOC_WAIT_S)
            return Status.NO_LOC
        # No progress: blocked near a wall, or pushing into something.
        err, _ = self._goto_error(m)
        if err is not None:
            res, now = float(math.hypot(*err)), time.monotonic()
            best = self._goto_best
            if best is None or res < best[0] - self.STUCK_PROGRESS_M:
                self._goto_best = (res, now)
            elif res > self.goto_tol * self.release_scale and now - best[1] > self.stuck_s:
                rospy.logwarn("[GoHome] GOTO stuck at %.2f m from the target",
                              res)
                return Status.STUCK
        return self._guard_center(m)

    #  Direct drive (room axes known): straight to the target from any pose.
    #  Wall distances are perpendicular, so a crooked robot reads them right.

    def _direct_pose(self, m):
        """(M body->room, position in m, half room size) or None.
        Needs both walls on each axis."""
        fit, track, direct = self._fit, self._track, self._direct
        if fit is None or fit[0] is not m or track is None or direct is None:
            return None
        coarse, _, dists = fit[1]
        if coarse is None:
            return None
        axes, yaw_ref, epoch = direct
        if track[0] != epoch:
            return None
        yaw = track[1]
        theta = coarse + 90.0 * round((yaw - coarse) / 90.0)
        psi = math.radians(-(theta - yaw_ref))       # turned since the map
        c, s_ = math.cos(psi), math.sin(psi)
        M = axes @ np.array([[c, -s_], [s_, c]])
        near = {}                       # (room axis, +side) -> wall distance
        for k, d in enumerate(dists):
            if d is None:
                continue
            b = math.radians(coarse + 90.0 * k - self.heading_offset)
            r = M @ np.array([-math.sin(b), math.cos(b)])
            i = int(abs(r[1]) > abs(r[0]))
            near[(i, r[i] > 0)] = d
        rel, half = np.zeros(2), np.zeros(2)
        for i in (0, 1):
            if (i, True) not in near or (i, False) not in near:
                return None
            lo, hi = near[(i, False)], near[(i, True)]
            rel[i], half[i] = (lo - hi) / 2.0, (lo + hi) / 2.0
        return M, np.array(self._room_centre_m) + rel, half

    def _direct_error(self, m):
        """Target - position in body axes, target kept inside the walls."""
        pose = self._direct_pose(m)
        if pose is None:
            return None
        M, pos, half = pose
        centre = np.array(self._room_centre_m)
        target = np.array(self._goto)
        if self.goto_lidar_clamp:
            keep = np.maximum(half - self._goto_keep_m, 0.0)
            target = centre + np.clip(target - centre, -keep, keep)
        return M.T @ (target - pos)

    def _sense_direct(self, m):
        err = self._direct_error(m)
        if err is None:
            return False, float('inf')
        return True, float(math.hypot(*err))

    def _drive_direct(self, m):
        err = self._direct_error(m)
        cmd = Twist()
        if err is None:
            return cmd
        dist = math.hypot(*err)
        speed = min(max(self.goto_kp * dist, self.min_lin_v), self.goto_max_v)
        if dist < self.goto_brake_m:
            speed = min(speed, self.goto_approach_v)
        out = [(e / dist) * speed for e in err]
        if dist > self.goto_diag_m:
            for i, e in enumerate(err):
                if abs(e) > self.goto_tol and abs(out[i]) < self.min_lin_v:
                    out[i] = math.copysign(self.min_lin_v, e)
        cmd.linear.x, cmd.linear.y = out
        return cmd

    def _guard_direct(self, m):
        """End on walls unseen for LOC_WAIT_S, or no progress (stuck)."""
        err = self._direct_error(m)
        now = time.monotonic()
        if err is None:
            if self._goto_best is not None and now - self._goto_best[1] > self.LOC_WAIT_S:
                return Status.NO_LOC
            if self._goto_best is None:
                self._goto_best = (float('inf'), now)
            return None
        res, best = float(math.hypot(*err)), self._goto_best
        if best is None or res < best[0] - self.STUCK_PROGRESS_M:
            self._goto_best = (res, now)
        elif res > self.goto_tol * self.release_scale and now - best[1] > self.stuck_s:
            rospy.logwarn("[GoHome] GOTO stuck at %.2f m from the target", res)
            return Status.STUCK
        return None

    def _direct_camera_agrees(self):
        """False (and forget the axes) if the camera sees the robot far
        from where the tracked axes put it."""
        loc = self._fresh_loc()
        if loc is None:
            return True
        end = time.monotonic() + 0.5
        pose = None
        while pose is None and time.monotonic() < end:
            pose = self._direct_pose(self._latest) if self._latest else None
            rospy.sleep(0.02)
        if pose is None:
            return True
        gap = math.hypot(pose[1][0] - loc[0], pose[1][1] - loc[1])
        if gap <= self.CAMERA_CHECK_M * 2:
            return True
        rospy.logwarn("[GoHome] tracked axes put the robot %.2f m from the "
                      "camera: finding them again", gap)
        self._axes_kept = None
        return False

    def go_direct(self, tracked, timeout):
        """Drive straight to self._goto, square up, verify."""
        started = time.monotonic()
        self._direct = (tracked[0], tracked[1], self._track[0] if self._track else -1)
        self._goto_best = None
        try:
            if not self._direct_camera_agrees():
                return PhaseResult(Status.PROBE, float('inf'), 0.0)
            rospy.loginfo("[GoHome] GOTO direct: room axes known, no camera "
                          "needed")
            result = self._run_phase(
                'GOTO', self.ST_GOTO, self._sense_direct, self._drive_direct,
                self.goto_tol, self.goto_settle, timeout, guard=self._guard_direct)
            if not result.ok:
                return result
            fine = self.align_to_axes(
                tol=self.fine_align_tol,
                timeout=min(started + timeout - time.monotonic(),
                            self.FINE_ALIGN_TIMEOUT_S))
            if fine.status in (Status.CANCELLED, Status.NO_SCAN):
                return fine
            self.state = self.ST_VERIFY
            # Per scan as it arrives: only the latest one has a wall fit.
            errs, poss, last = [], [], None
            end = time.monotonic() + self.VERIFY_WINDOW_S
            while (time.monotonic() < end and len(errs) < self.VERIFY_SCANS
                   and not self._cancel.is_set() and not rospy.is_shutdown()):
                m = self._latest
                if m is not None and m is not last:
                    last = m
                    e = self._direct_error(m)
                    pose = self._direct_pose(m)
                    if e is not None and pose is not None:
                        errs.append(math.hypot(*e))
                        poss.append(pose[1])
                rospy.sleep(0.03)
            _, square, _, ang = self._verify()
            dist = float(np.median(errs)) if errs else float('inf')
            elapsed = time.monotonic() - started
            if dist < self.verify_goto_tol and square:
                rospy.loginfo("[GoHome] GOTO VERIFY PASS  dist=%.3f m  "
                              "ang=%+.1f deg  (direct)", dist, ang)
                tracked = self._tracked_axes()
                if tracked is not None:
                    self._remember_end(self._square_axes(tracked),
                                       np.median(np.array(poss), axis=0))
                return PhaseResult(Status.SUCCESS, dist, elapsed)
            rospy.loginfo("[GoHome] GOTO VERIFY FAIL  dist=%.3f m  ang=%+.1f "
                          "deg  (direct)", dist, ang)
            return PhaseResult(Status.UNSETTLED, dist, elapsed)
        finally:
            self._direct = None
            self._fit = None

    def _verify_loc(self):
        """Final position in room metres: the anchored LIDAR (median of
        scans), the camera only as a cross-check.  Camera alone if not
        anchored."""
        if self._off is None:
            return self._median_loc(self.VERIFY_LOCS, self.VERIFY_LOC_WINDOW_S)
        pbs = [self._lidar_pos(m) for m in
               self._collect(self.VERIFY_SCANS, self.VERIFY_WINDOW_S)]
        pbs = [pb for pb in pbs if pb is not None]
        if not pbs:
            return self._median_loc(self.VERIFY_LOCS, self.VERIFY_LOC_WINDOW_S)
        loc = self._axes @ (np.median(np.array(pbs), axis=0) + self._off)
        cam = self._median_loc(self.VERIFY_LOCS, self.VERIFY_LOC_WINDOW_S)
        if cam is not None:
            gap = math.hypot(cam[0] - loc[0], cam[1] - loc[1])
            level = (rospy.logwarn if gap > self.CAMERA_CHECK_M
                     else rospy.loginfo)
            level("[GoHome] GOTO camera vs LIDAR at the end: %.3f m", gap)
        return float(loc[0]), float(loc[1])

    def drive_to_point(self, target, tol=None, settle_s=None, timeout=None):
        """Translate the robot to a floor point (metres) on the camera
        position.  Needs the axis map from _find_axes in self._axes."""
        self._goto = target
        self._goto_best = None
        return self._run_phase(
            'GOTO', self.ST_GOTO, self._sense_goto, self._drive_goto,
            self.goto_tol if tol is None else tol,
            self.goto_settle if settle_s is None else settle_s,
            self.timeout_s if timeout is None else timeout,
            guard=self._guard_goto)

    def _remember_end(self, axes, pos):
        """Square at a known spot: keep the axes for the current tracking
        epoch (it may have restarted on the way) and the position."""
        self._keep_axes(axes)
        self._last_pos = (float(pos[0]), float(pos[1]))

    def _axes_from_last_pos(self):
        """Axis map whose LIDAR position matches where the last go_to ended
        (the robot has not moved much since).  No camera needed."""
        m = self._latest
        pb = self._lidar_pos(m) if m is not None else None
        if pb is None or self._last_pos is None or self._room_centre_m is None:
            return None
        centre, last = np.array(self._room_centre_m), np.array(self._last_pos)
        cands = [r for r in AXIS_MAPS
                 if self._world_det in (0, int(round(np.linalg.det(r))))]
        gaps = sorted((float(np.hypot(*(centre + r @ pb - last))), i)
                      for i, r in enumerate(cands))
        (best, i), runner = gaps[0], gaps[1][0]
        if best > self.LAST_POS_FIT_M or runner - best < self.LAST_POS_MARGIN_M:
            return None
        rospy.loginfo("[GoHome] room axes from where the last go_to ended "
                      "(%.2f m, next %.2f m)", best, runner)
        return cands[i]

    def _acquire_axes(self, aligned):
        """Axis map and anchor, cheapest first: tracked from the last run,
        one camera fix, where the last run ended, probe moves."""
        started = time.monotonic()
        tracked = self._tracked_axes()
        axes = self._axes_from_fix(
            self._square_axes(tracked) if tracked is not None else None)
        if axes is None:
            axes = self._axes_from_last_pos()
        if axes is None:
            axes = self._find_axes()
        if axes is None:
            status = Status.CANCELLED if self._cancel.is_set() else Status.PROBE
            return PhaseResult(status, aligned.error,
                               aligned.elapsed + time.monotonic() - started)
        self._axes = axes
        self._keep_axes(axes)
        if self._room_centre_m is not None:
            self._anchor_fixed()
        elif self._anchor():
            # Learned once: later runs need no camera.
            self._room_centre_m = tuple(float(v) for v in self._axes @ self._off)
        return aligned

    def go_to(self, x, y, timeout=None, passes=None):
        """GOTO: square up, find the room axes, drive to (x, y), square up
        finely, verify.  x, y normalised like localization/location.world."""
        target = (x * self._floor_m[0], y * self._floor_m[1])
        budget = self.timeout_s if timeout is None else timeout
        passes = self.passes if passes is None else passes
        deadline = time.monotonic() + budget
        started = time.monotonic()
        result = PhaseResult(Status.UNSETTLED, float('inf'), 0.0)

        def elapsed():
            return time.monotonic() - started

        with self._run_scope():
            self._reset_speed()
            self._axes = None
            self._off = None
            self._fused = None
            self._wall_clamped = False
            self._lidar_hist.clear()
            self._goto = target
            self._wait_scan()

            tracked = self._tracked_axes()
            if tracked is not None and self._room_centre_m is not None:
                result = self.go_direct(tracked, deadline - time.monotonic())
                if result.ok or result.status == Status.CANCELLED:
                    return PhaseResult(result.status, result.error, elapsed())
                rospy.logwarn("[GoHome] direct drive ended %s: squaring up "
                              "and trying again", result.status)

            for attempt in range(1, passes + 1):
                if deadline - time.monotonic() <= 0:
                    return PhaseResult(Status.TIMEOUT, result.error, elapsed())
                self.escape_walls()
                result = self.align_to_axes(timeout=deadline - time.monotonic())
                if not result.ok:
                    return result

                # Once per run: a drift re-align returns to the same axis.
                if self._axes is None:
                    result = self._acquire_axes(result)
                    if not result.ok:
                        return result

                if deadline - time.monotonic() <= 0:
                    return PhaseResult(Status.TIMEOUT, result.error, elapsed())
                result = self.drive_to_point(   # a clamped target stays
                    self._goto, timeout=deadline - time.monotonic())
                if result.status in (Status.DRIFT, Status.STUCK):
                    continue                    # back off, square up, again
                if not result.ok:
                    return result

                fine = self.align_to_axes(
                    tol=self.fine_align_tol,
                    timeout=min(deadline - time.monotonic(),
                                self.FINE_ALIGN_TIMEOUT_S))
                if fine.status in (Status.CANCELLED, Status.NO_SCAN):
                    return fine

                self.state = self.ST_VERIFY
                loc = self._verify_loc()
                if self._cancel.is_set():
                    return PhaseResult(Status.CANCELLED, result.error,
                                       elapsed())
                if loc is None:
                    return PhaseResult(Status.NO_LOC, result.error, elapsed())
                target = self._goto     # after any clamp to the walls
                dist = math.hypot(target[0] - loc[0], target[1] - loc[1])
                # Square as well, as go_home checks: the fine align may time out.
                _, square, _, ang = self._verify()
                there = dist < self.verify_goto_tol
                if there and square:
                    rospy.loginfo("[GoHome] GOTO VERIFY PASS  dist=%.3f m  "
                                  "ang=%+.1f deg  (pass %d/%d)",
                                  dist, ang, attempt, passes)
                    self._remember_end(self._axes, loc)
                    return PhaseResult(Status.SUCCESS, dist, elapsed())
                rospy.loginfo("[GoHome] GOTO VERIFY FAIL  dist=%.3f m (%s)  "
                              "ang=%+.1f deg (%s) -- repeating (pass %d/%d)",
                              dist, "ok" if there else "off",
                              ang, "ok" if square else "off", attempt, passes)
                result = PhaseResult(Status.UNSETTLED, dist, elapsed())

            return PhaseResult(Status.UNSETTLED, result.error, elapsed())

    #  Top-down view: the room fixed on screen, the robot moving in it.
    #  Drawn on the scan thread, shown by run().  (u, w) = screen right/up,
    #  metres from the room centre, oriented by the heading tracker.

    def _viz_to_px(self, u, w):
        return (int(round(self._viz_cx + u * self._viz_ppm)),
                int(round(self._viz_cy - w * self._viz_ppm)))

    @staticmethod
    def _viz_dir(alpha_deg):
        """Room unit vector alpha degrees CCW from screen up."""
        a = math.radians(alpha_deg)
        return np.array([-math.sin(a), math.cos(a)])

    def _viz_pose(self, ranges_m):
        """(theta, (u, w) or None).  theta: screen up in the lidar frame, deg."""
        coarse, _, dists = geom.wall_fit(ranges_m, r_min=self.RANGE_MIN_M,
                                         r_max=self.RANGE_MAX_M)
        if coarse is None:
            return None, None
        track = self._track
        if track is None:
            return None, None
        if track[0] != self._viz_epoch:
            # Tracker restarted: pick the quarter turn that keeps the picture.
            self._viz_epoch = track[0]
            self._viz_q = self._viz_pick_quarter(coarse, dists, track[1])
        n = int(round((track[1] - coarse) / 90.0)) + self._viz_q
        theta = coarse + 90.0 * n
        side = self._viz_sides(dists, n)
        self._viz_side = side
        self._viz_update_room(side)
        pos = self._viz_position(side)
        if pos is not None:
            self._viz_last_pos = pos
        return theta, pos

    @staticmethod
    def _viz_sides(dists, n):
        """{screen side (0 up, 1 left, 2 down, 3 right): wall distance}."""
        return {(k - n) % 4: d for k, d in enumerate(dists) if d is not None}

    def _viz_position(self, side):
        if self._viz_room is None:
            return None
        half_w, half_h = self._viz_room[0] / 2.0, self._viz_room[1] / 2.0
        us = ([-half_w + side[1]] if 1 in side else []) + \
             ([half_w - side[3]] if 3 in side else [])
        ws = ([half_h - side[0]] if 0 in side else []) + \
             ([-half_h + side[2]] if 2 in side else [])
        if not us or not ws:
            return None
        return float(np.mean(us)), float(np.mean(ws))

    def _viz_pick_quarter(self, coarse, dists, yaw):
        if self._viz_room is None or self._viz_last_pos is None:
            return 0
        n0 = int(round((yaw - coarse) / 90.0))
        best, best_q = None, 0
        for q in range(4):
            side = self._viz_sides(dists, n0 + q)
            cost = 0.0
            for i, (a, b) in enumerate(((1, 3), (0, 2))):
                if a in side and b in side:
                    cost += abs(side[a] + side[b] - self._viz_room[i])
            pos = self._viz_position(side)
            if pos is not None:
                cost += math.hypot(pos[0] - self._viz_last_pos[0],
                                   pos[1] - self._viz_last_pos[1])
            if best is None or cost < best:
                best, best_q = cost, q
        return best_q

    def _viz_update_room(self, side):
        """Room size from opposite walls, averaged slowly.  Scale set once."""
        size = list(self._viz_room) if self._viz_room is not None else [None, None]
        for i, (a, b) in enumerate(((1, 3), (0, 2))):
            if a in side and b in side:
                m = side[a] + side[b]
                size[i] = m if size[i] is None else 0.98 * size[i] + 0.02 * m
        if None in size:
            return
        self._viz_room = tuple(size)
        if self._viz_ppm is None:
            extent = (self._viz_fixed_range if self._viz_fixed_range > 0
                      else max(size) / 2.0 + 0.1)
            self._viz_ppm = (self._viz_size * 0.47) / extent

    def _viz_draw_room(self, img):
        half_w, half_h = self._viz_room[0] / 2.0, self._viz_room[1] / 2.0
        for g in np.arange(-math.floor(half_w), half_w, 1.0):
            cv2.line(img, self._viz_to_px(g, -half_h), self._viz_to_px(g, half_h),
                     self.C_GRID, 1)
        for g in np.arange(-math.floor(half_h), half_h, 1.0):
            cv2.line(img, self._viz_to_px(-half_w, g), self._viz_to_px(half_w, g),
                     self.C_GRID, 1)
        cv2.rectangle(img, self._viz_to_px(-half_w, half_h),
                      self._viz_to_px(half_w, -half_h), self.C_TEXT_DIM, 1)
        cv2.drawMarker(img, self._viz_to_px(0.0, 0.0), self.C_CENTRE,
                       cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
        cv2.putText(img, "room %.2f x %.2f m   grid 1 m" % self._viz_room,
                    (10, self._viz_band - 14), self.FONT, 0.42,
                    self.C_TEXT_DIM, 1, cv2.LINE_AA)

    def _viz_draw_points(self, img, ranges_m, theta, pos):
        idx = np.flatnonzero(np.isfinite(ranges_m)
                             & (ranges_m > self.RANGE_MIN_M)
                             & (ranges_m < self.RANGE_MAX_M))
        if idx.size == 0:
            return
        a = np.radians(idx.astype(np.float64) - theta)
        r = ranges_m[idx]
        xs = np.rint(self._viz_cx + (pos[0] - r * np.sin(a)) * self._viz_ppm).astype(int)
        ys = np.rint(self._viz_cy - (pos[1] + r * np.cos(a)) * self._viz_ppm).astype(int)
        inside = ((xs >= 0) & (xs < img.shape[1])
                  & (ys >= self._viz_band) & (ys < img.shape[0]))
        for x, y in zip(xs[inside], ys[inside]):
            cv2.circle(img, (int(x), int(y)), 2, self.C_POINTS, -1, cv2.LINE_AA)

    def _viz_draw_model(self, img, theta, pos, walls):
        """The current measurement: fitted walls (on the outline when right)
        and the four beam distances go_home uses."""
        p = np.array(pos)
        half_w, half_h = self._viz_room[0] / 2.0, self._viz_room[1] / 2.0
        for j, d in self._viz_side.items():
            n = self._viz_dir(90.0 * j)             # wall normal
            t = np.array([-n[1], n[0]])
            c = p + n * d
            span = half_w if j in (0, 2) else half_h
            cv2.line(img, self._viz_to_px(*(c - t * span)),
                     self._viz_to_px(*(c + t * span)), self.C_PROBE, 2, cv2.LINE_AA)
        # Along their lidar beams, not the body axes (mounting offset).
        for d, beam in zip(walls, (0.0, 180.0, 90.0, 270.0)):
            if d is None:
                continue
            end = self._viz_to_px(*(p + self._viz_dir(beam - theta) * d))
            cv2.line(img, self._viz_to_px(*p), end, self.C_CARDINAL, 1, cv2.LINE_AA)
            cv2.circle(img, end, 4, self.C_CARDINAL, -1, cv2.LINE_AA)
            cv2.putText(img, "%.2f" % d, (end[0] + 6, end[1] - 6),
                        self.FONT, 0.42, self.C_CARDINAL, 1, cv2.LINE_AA)

    def _viz_body(self, theta):
        """Body forward and right as screen vectors."""
        fwd = self._viz_dir(self.heading_offset - theta)
        return fwd, np.array([fwd[1], -fwd[0]])

    def _viz_draw_robot(self, img, theta, pos):
        fwd, right = self._viz_body(theta)
        p, half = np.array(pos), self.ROBOT_SIZE_M / 2.0
        corners = [p + half * (a * right + b * fwd)
                   for a, b in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
        cv2.polylines(img, [np.array([self._viz_to_px(*c) for c in corners])],
                      True, self.C_ROBOT, 2, cv2.LINE_AA)
        cv2.arrowedLine(img, self._viz_to_px(*p),
                        self._viz_to_px(*(p + fwd * half * 2.2)),
                        self.C_HEADING, 2, cv2.LINE_AA, tipLength=0.3)

    def _on_drive_speed(self, msg):
        # motor_control publishes linear.x = right (v), linear.y = forward (h).
        self._drive_cmd = (msg.twist.linear.y, msg.twist.linear.x)
        if not self.enabled and math.hypot(*self._drive_cmd) > 0.05:
            self._last_pos = None       # driven since the last go_to ended

    def _viz_track_motion(self, pos):
        """Room positions for TRAIL_S -> velocity (u, w) m/s or None."""
        now = time.monotonic()
        if pos is not None:
            self._motion_hist.append((now, pos[0], pos[1]))
        while self._motion_hist and now - self._motion_hist[0][0] > self.TRAIL_S:
            self._motion_hist.popleft()
        if len(self._motion_hist) < 2:
            return None
        t1, u1, w1 = self._motion_hist[-1]
        old = [p for p in self._motion_hist if t1 - p[0] >= self.MOTION_WIN_S]
        if not old:
            return None
        t0, u0, w0 = old[-1]
        return np.array([(u1 - u0) / (t1 - t0), (w1 - w0) / (t1 - t0)])

    def _viz_arrow(self, img, pos, vec, colour, length_m):
        mag = float(np.hypot(*vec))
        if mag < 1e-9:
            return
        p = np.array(pos)
        cv2.arrowedLine(img, self._viz_to_px(*p),
                        self._viz_to_px(*(p + vec / mag * length_m)), colour, 3,
                        cv2.LINE_AA, tipLength=0.2)

    def _viz_draw_motion(self, img, theta, pos, vel):
        """Trail, commanded vs measured direction.  -> (cmd_deg, motion_deg),
        both in the body frame: 0 = forward, + = right."""
        for _, u, w in self._motion_hist:
            cv2.circle(img, self._viz_to_px(u, w), 2, self.C_TRAIL, -1, cv2.LINE_AA)
        fwd, right = self._viz_body(theta)
        cf, cr = self._drive_cmd
        cmd_deg = mot_deg = None
        if math.hypot(cf, cr) > 0.02:
            cmd_deg = math.degrees(math.atan2(cr, cf))
            self._viz_arrow(img, pos, cf * fwd + cr * right, self.C_DRIVE, 0.6)
        if vel is not None and math.hypot(*vel) > 0.02:
            mot_deg = math.degrees(math.atan2(vel @ right, vel @ fwd))
            self._viz_arrow(img, pos, vel, self.C_MOTION, 0.5)
        return cmd_deg, mot_deg

    def _viz_draw_legend(self, img):
        items = (("LIDAR returns", self.C_POINTS),
                 ("room (averaged)", self.C_TEXT_DIM),
                 ("walls as fitted now", self.C_PROBE),
                 ("wall distances", self.C_CARDINAL),
                 ("room centre", self.C_CENTRE),
                 ("drive command", self.C_DRIVE),
                 ("actual motion", self.C_MOTION),
                 ("path (6 s)", self.C_TRAIL))
        for i, (label, colour) in enumerate(items):
            y = 22 + i * 18
            cv2.line(img, (self._viz_size - 190, y - 4),
                     (self._viz_size - 172, y - 4), colour, 2)
            cv2.putText(img, label, (self._viz_size - 166, y), self.FONT,
                        0.42, self.C_TEXT_DIM, 1, cv2.LINE_AA)

    def _viz_due(self):
        """True when the overlay is enabled and its next frame is owed."""
        return (self._viz_on and
                time.monotonic() - self._viz_last >= 1.0 / max(1.0, self.viz_fps))

    def _render_viz(self, m, cmd, elapsed):
        """Build the next overlay frame.  No-op above the configured FPS."""
        if not self._viz_due():
            return
        self._viz_last = time.monotonic()

        img = np.full((self._viz_band + self._viz_size, self._viz_size, 3),
                      self.C_BG, dtype=np.uint8)
        cv2.line(img, (0, self._viz_band - 1), (self._viz_size, self._viz_band - 1),
                 self.C_GRID, 1)
        theta, pos = self._viz_pose(m.ranges)
        if self._viz_room is not None and self._viz_ppm is not None:
            self._viz_draw_room(img)
            if pos is not None:
                self._viz_draw_points(img, m.ranges, theta, pos)
                self._viz_draw_model(img, theta, pos, m.walls)
                vel = self._viz_track_motion(pos)
                cmd_deg, mot_deg = self._viz_draw_motion(img, theta, pos, vel)
                self._viz_draw_robot(img, theta, pos)
            else:
                vel, cmd_deg, mot_deg = None, None, None
                cv2.putText(img, "walls not seen: position unknown",
                            (10, self._viz_cy), self.FONT, 0.5,
                            self.C_CENTRE, 1, cv2.LINE_AA)
        else:
            vel, cmd_deg, mot_deg = None, None, None
            cv2.putText(img, "waiting for all four walls...",
                        (10, self._viz_cy), self.FONT, 0.6,
                        self.C_TEXT, 1, cv2.LINE_AA)

        if cmd_deg is not None and mot_deg is not None:
            diff = (mot_deg - cmd_deg + 180.0) % 360.0 - 180.0
            motion_txt = ("drive %+.1f deg  actual %+.1f deg  DIFF %+.1f deg  %.2f m/s"
                          % (cmd_deg, mot_deg, diff, math.hypot(*vel)))
        elif cmd_deg is not None:
            motion_txt = "drive %+.1f deg  actual --" % cmd_deg
        else:
            motion_txt = "drive --  (0 = forward, + = right)"

        walls_txt = "  ".join(
            "%s=%s" % (name, "--" if d is None else "%.2f" % d)
            for name, d in zip(("F", "B", "L", "R"), m.walls))
        driving = bool(self._measuring)
        lines = [
            ("%s%s" % (self.STATE_NAMES[self.state],
                       "" if driving else "  (idle)"),
             self.C_TEXT if driving else self.C_TEXT_DIM),
            ("centre  dist=%s  err_x=%+.3f  err_y=%+.3f"
             % ("--" if not m.ctr_valid else "%.3f m" % m.dist,
                m.err_x, m.err_y), self.C_TEXT),
            ("heading err=%s  conf=%.2f  (tol %.1f deg)"
             % ("--" if not m.ang_valid else "%+.1f deg" % m.ang_err,
                m.ang_conf, self.align_tol), self.C_TEXT),
            ("cmd  x=%+.2f  y=%+.2f  w=%+.2f"
             % (cmd.linear.x, cmd.linear.y, cmd.angular.z), self.C_TEXT),
            (motion_txt, self.C_DRIVE),
            ("walls  %s" % walls_txt, self.C_TEXT_DIM),
            ("stop tol %.3f m   elapsed %.1f s" % (self.center_tol, elapsed),
             self.C_TEXT_DIM),
        ]
        for i, (text, colour) in enumerate(lines):
            cv2.putText(img, text, (10, 24 + i * 20), self.FONT, 0.5, colour,
                        1, cv2.LINE_AA)
        self._viz_draw_legend(img)

        self._viz_frame = img

    def run(self):
        """Pump the visualisation window from the main thread."""
        if not self._viz_on:
            rospy.spin()
            return

        rate = rospy.Rate(self.viz_fps)
        while not rospy.is_shutdown():
            frame = self._viz_frame
            if frame is not None:
                try:
                    cv2.imshow(self.VIZ_WINDOW, frame)
                    cv2.waitKey(1)
                except cv2.error as e:
                    # No display available — keep driving the robot headless.
                    rospy.logwarn(
                        "[GoHome] Visualisation unavailable (%s); "
                        "continuing without it.", e)
                    self._viz_on = False
                    rospy.spin()
                    return
            rate.sleep()
        cv2.destroyWindow(self.VIZ_WINDOW)


# ====================================================================
if __name__ == "__main__":
    rospy.init_node("go_home_node")
    try:
        node = GoHomeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
"""Scripted drive patterns for drift measurement (drift compensation, stage 0).

!!! THIS SCRIPT MOVES THE ROBOT. Run it only with a person next to the robot,
!!! ready to stop it (Ctrl+C here, or the joystick / killing motor_control).

It drives exactly like the GUI manual buttons: geometry_msgs/Twist on
motor_control/twist, with the GUI's angle mapping (0 = Forward, 90 = Left,
180 = Backward, 270 = Right; rotation +1 = the GUI "cw" button). Everything in
motor_control_node (ramp, 1 s watchdog, LiDAR blocking) applies unchanged.

Each segment is announced on drift_test/segment (std_msgs/String, JSON) so
analyze_bag.py can cut the recording into runs.

Before running:
  * Close the GUI manual-control window (it publishes zeros every 50 ms).
  * Joystick must NOT be in joystick mode (it publishes at 20 Hz).
  * No trial running (fish mode), Go Home off.
  The script listens first and aborts if anyone else publishes motion commands.

Examples (on any machine of the ROS network, after sourcing the workspace):
  python3 test_patterns.py --pattern single --direction forward --duration 1.5
  python3 test_patterns.py --pattern axes --duration 2.0 --repeat 3
  python3 test_patterns.py --pattern rotate --duration 2.0
"""

import argparse
import json
import math
import sys
import time

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String

DIRECTIONS = {
    'forward': 0, 'fwd_left': 45, 'left': 90, 'back_left': 135,
    'backward': 180, 'back_right': 225, 'right': 270, 'fwd_right': 315,
}

# Out-and-back pairs keep the robot near where it started.
PATTERNS = {
    'axes':      ['forward', 'backward', 'right', 'left'],
    'diagonals': ['fwd_left', 'back_right', 'fwd_right', 'back_left'],
    'all':       ['forward', 'backward', 'right', 'left',
                  'fwd_left', 'back_right', 'fwd_right', 'back_left'],
    'rotate':    ['cw', 'ccw'],
}

COMMAND_TOPICS = [
    ('motor_control/twist', Twist),
    ('motor_control/vector', Vector3),
    ('motor_control/angle', Float32),
    ('motor_control/rotate', Float32),
]

PUBLISH_HZ = 20.0


class Aborted(Exception):
    pass


class PatternRunner:
    def __init__(self, args):
        self.args = args
        self.me = rospy.get_name()
        self.foreign = None
        self.cmd_pub = rospy.Publisher('motor_control/twist', Twist, queue_size=1, tcp_nodelay=True)
        self.seg_pub = rospy.Publisher('drift_test/segment', String, queue_size=10, latch=False)
        for topic, mtype in COMMAND_TOPICS:
            rospy.Subscriber(topic, mtype, self._watch, callback_args=topic, queue_size=10)

    # -- safety ------------------------------------------------------------ #
    def _watch(self, msg, topic):
        caller = getattr(msg, '_connection_header', {}).get('callerid', '')
        if caller and caller != self.me and self.foreign is None:
            self.foreign = (caller, topic)

    def _check(self):
        if self.foreign is not None:
            caller, topic = self.foreign
            raise Aborted(f"another node ({caller}) is publishing on {topic}")

    def stop(self, seconds=0.5):
        """Publish zero commands for a short while (best effort)."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                return
            time.sleep(1.0 / PUBLISH_HZ)

    # -- helpers ----------------------------------------------------------- #
    def _sleep_tick(self):
        time.sleep(1.0 / PUBLISH_HZ)
        if rospy.is_shutdown():
            raise Aborted("ROS shut down")
        self._check()

    def _hold(self, twist, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd_pub.publish(twist)
            self._sleep_tick()

    def _marker(self, event, **info):
        info['event'] = event
        info['wall_time'] = time.time()
        self.seg_pub.publish(String(data=json.dumps(info)))

    @staticmethod
    def twist_for(name, speed):
        t = Twist()
        if name in ('cw', 'ccw'):
            t.angular.z = speed if name == 'cw' else -speed   # GUI: cw = +1
        else:
            a = math.radians(DIRECTIONS[name])
            t.linear.x = -math.sin(a) * speed                  # GUI mapping
            t.linear.y = math.cos(a) * speed
        return t

    # -- main -------------------------------------------------------------- #
    def preflight(self):
        deadline = time.monotonic() + 5.0
        while self.cmd_pub.get_num_connections() == 0:
            if time.monotonic() > deadline:
                raise Aborted("motor_control_node is not subscribed to motor_control/twist")
            time.sleep(0.1)
        rospy.loginfo("Listening 2 s for other motion-command publishers...")
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self._sleep_tick()
        if not self.args.keep_speed:
            try:
                from foma.srv import Float
                rospy.wait_for_service('motor_control/set_speed', timeout=2.0)
                rospy.ServiceProxy('motor_control/set_speed', Float)(1.0)
                rospy.loginfo("motor_control speed factor set to 1.0")
            except Exception as e:
                raise Aborted(f"could not set motor_control speed factor to 1.0: {e}")
        for k in range(3, 0, -1):
            rospy.loginfo(f"Starting in {k}...")
            self._hold(Twist(), 1.0)

    def run(self):
        a = self.args
        names = [a.direction] if a.pattern == 'single' else PATTERNS[a.pattern]
        self._marker('session_start', pattern=a.pattern, speed=a.speed,
                     duration=a.duration, pause=a.pause, repeat=a.repeat,
                     note=a.note)
        index = 0
        for rep in range(a.repeat):
            for name in names:
                kind = 'rotate' if name in ('cw', 'ccw') else 'translate'
                info = dict(index=index, repeat=rep, label=name, kind=kind,
                            speed=a.speed, duration=a.duration,
                            angle_deg=DIRECTIONS.get(name))
                rospy.loginfo(f"[{index}] {name}  speed={a.speed}  {a.duration:.1f} s")
                self._marker('start', **info)
                self._hold(self.twist_for(name, a.speed), a.duration)
                self._marker('end', **info)
                self._hold(Twist(), a.pause)
                index += 1
        self._marker('session_end', segments=index)


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pattern', choices=['single'] + sorted(PATTERNS), default='single')
    p.add_argument('--direction', choices=sorted(DIRECTIONS) + ['cw', 'ccw'], default='forward',
                   help="for --pattern single")
    p.add_argument('--speed', type=float, default=1.0, help="0..1, like the GUI (default 1.0)")
    p.add_argument('--duration', type=float, default=2.0, help="seconds of command per segment")
    p.add_argument('--pause', type=float, default=3.0, help="seconds of zero command after each segment")
    p.add_argument('--repeat', type=int, default=1)
    p.add_argument('--note', default='', help="free text stored in the recording (e.g. 'tank full')")
    p.add_argument('--keep-speed', action='store_true',
                   help="do not reset motor_control speed factor to 1.0")
    a = p.parse_args(argv)
    if not 0.0 < a.speed <= 1.0:
        p.error("--speed must be in (0, 1]")
    if a.duration <= 0 or a.duration > 6.0:
        p.error("--duration must be in (0, 6] s (room is ~5 m)")
    if a.pause < 1.5:
        p.error("--pause must be >= 1.5 s (ramp-down + LiDAR settling)")
    if a.repeat < 1:
        p.error("--repeat must be >= 1")
    return a


def main():
    args = parse_args(rospy.myargv(sys.argv)[1:])
    # disable_signals: Ctrl+C raises KeyboardInterrupt here, so we can still
    # publish the stop commands before ROS shuts down.
    rospy.init_node('drift_test_patterns', disable_signals=True)
    runner = PatternRunner(args)
    try:
        runner.preflight()
        runner.run()
        rospy.loginfo("Pattern finished.")
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted - stopping the robot.")
    except Aborted as e:
        rospy.logerr(f"ABORTED: {e} - stopping the robot.")
    finally:
        runner.stop(0.6)
        rospy.signal_shutdown("done")


if __name__ == '__main__':
    main()

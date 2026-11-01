"""Run the real motor_control_node / lidar_node code against stubs.

Catches crashes in the new telemetry before it reaches the robot, and checks
that the motor commands are unchanged by the telemetry.
"""

import math
import unittest

import _stubs

PARAMS = {
    '/FOMA_CAMERA_FRAME_SHAPE': [640, 640], '/DIRECTION_EPSILON': 45, '/SAFETY_DISTANCE': 400,
    '/MOTOR_TOP_BOTTOM_RESET': 24, '/MOTOR_RIGHT_LEFT_RESET': 23,
    '/MOTOR_TOP_CHA': 5, '/MOTOR_TOP_CHB': 6, '/MOTOR_BOTTOM_CHA': 13, '/MOTOR_BOTTOM_CHB': 19,
    '/MOTOR_LEFT_CHA': 20, '/MOTOR_LEFT_CHB': 21, '/MOTOR_RIGHT_CHA': 12, '/MOTOR_RIGHT_CHB': 16,
    '/MOTOR_PORT': 'fake', '/MOTOR_SPEED': 127,
    '/LIDAR_OFFSET': 90, '/LIDAR_PORT': 'fake', '/LIDAR_PORT_SPEED': 256000,
}


def _pub(rospy, topic):
    return [p for p in rospy.Publisher.instances if p.topic == topic][-1]


class MotorNodeSmoke(unittest.TestCase):
    def test_loop_publishes_wheel_state(self):
        rospy = _stubs.install(PARAMS, shutdown_after=30)
        mcn = _stubs.fresh_import('motor_control_node')
        geo = __import__('geometry_msgs.msg', fromlist=['Twist'])
        node = mcn.MotorControlNode()
        mc = node._MotorControlNode__motor_control
        node._MotorControlNode__handle_twist(geo.Twist(linear=geo.Vector3(0, 1.0, 0),
                                                       angular=geo.Vector3(0, 0, 0)))
        node._MotorControlNode__last_cmd_time = rospy.Time(1e9)   # keep command fresh
        # simulate encoder motion between ticks
        orig = mc.move_by_components

        def moving(h, v, speed=None):
            mc.encoderLeft.steps += 3
            mc.encoderRight.steps -= 3
            return orig(h, v, speed)
        mc.move_by_components = moving

        node.run()
        ws = _pub(rospy, 'motor_control/wheel_state').sent
        self.assertGreater(len(ws), 20)
        last = ws[-1]
        self.assertEqual(last.name, ['left', 'right', 'top', 'bottom'])
        self.assertEqual(len(last.position), 4)
        self.assertGreater(last.position[0], 0)
        self.assertLess(last.position[1], 0)
        self.assertTrue(all(math.isfinite(v) for v in last.velocity))
        self.assertGreater(last.effort[0], 0)            # left driven forward
        self.assertFalse([l for l in rospy.logs if 'telemetry failed' in str(l[1])])

    def test_rotation_branch_publishes_too(self):
        rospy = _stubs.install(PARAMS, shutdown_after=10)
        mcn = _stubs.fresh_import('motor_control_node')
        std = __import__('std_msgs.msg', fromlist=['Float32'])
        node = mcn.MotorControlNode()
        node._MotorControlNode__handle_rotate(std.Float32(1.0))
        node._MotorControlNode__last_cmd_time = rospy.Time(1e9)
        node.run()
        ws = _pub(rospy, 'motor_control/wheel_state').sent
        self.assertGreater(len(ws), 5)
        self.assertEqual(len(set(ws[-1].effort)), 1)    # rotation: all four equal

    def test_telemetry_failure_does_not_stop_loop(self):
        rospy = _stubs.install(PARAMS, shutdown_after=5)
        mcn = _stubs.fresh_import('motor_control_node')
        node = mcn.MotorControlNode()

        def boom():
            raise RuntimeError('encoder read failed')
        node._MotorControlNode__motor_control.read_encoders = boom
        node.run()                                        # must not raise
        self.assertTrue([l for l in rospy.logs if 'telemetry failed' in str(l[1])])


class FakeLidar:
    def __init__(self, revolutions, stop):
        self.revolutions, self.stop = revolutions, stop

    def iter_measurements(self):
        for rev in range(self.revolutions):
            for k in range(400):
                yield (k == 0, 15, k * 0.9, 1500.0 + rev)
        self.stop()
        return


class LidarNodeSmoke(unittest.TestCase):
    def test_scan_full_one_message_per_complete_revolution(self):
        rospy = _stubs.install(PARAMS)
        ln = _stubs.fresh_import('lidar_node')
        node = ln.LIDARNode()                              # stub sensor fails -> lidar None
        stop_event = node._LIDARNode__stop_event
        node.lidar = FakeLidar(4, stop_event.set)
        node._scan_loop()
        full = _pub(rospy, 'lidar/scan_full').sent
        # 4 revolutions start-flagged: first boundary only opens, 3 complete ones published
        self.assertEqual(len(full), 3)
        msg = full[-1]
        self.assertEqual(len(msg.ranges), 720)
        vals = [r for r in msg.ranges if not math.isnan(r)]
        self.assertGreater(len(vals), 300)
        self.assertTrue(all(abs(v - 1.502) < 1e-9 for v in vals))   # metres, revolution 2
        self.assertAlmostEqual(msg.angle_increment, 2 * math.pi / 720)
        # legacy topic buffer still filled in mm
        self.assertTrue(any(abs(x - 1503.0) < 1e-9 for x in node.scan_msg.ranges))


if __name__ == '__main__':
    unittest.main()

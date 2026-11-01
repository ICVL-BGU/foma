"""The motor driver on this branch must send byte-identical commands to main.

Loads src/etc/MotorControl.py as it is on `main` (via git) next to the
working copy, drives both with the same commands and encoder states, and
compares every byte written to the (fake) UART.
"""

import importlib.util
import itertools
import os
import subprocess
import tempfile
import unittest

import _stubs

_stubs.install()


def _load_main_version():
    try:
        src = subprocess.run(['git', 'show', 'main:src/etc/MotorControl.py'],
                             cwd=_stubs.REPO, capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    path = os.path.join(tempfile.mkdtemp(), 'MotorControl_main.py')
    with open(path, 'wb') as f:
        f.write(src)
    spec = importlib.util.spec_from_file_location('MotorControl_main', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _make(mod):
    return mod.MotorControl(resetPins=(23, 24),
                            encoderChannels=((5, 6), (13, 19), (20, 21), (12, 16)),
                            accl=5, brake=0, port='fake', speed=127)


def _set_steps(mc, top, bottom, left, right):
    mc.encoderTop.steps, mc.encoderBottom.steps = top, bottom
    mc.encoderLeft.steps, mc.encoderRight.steps = left, right


class LegacyByteIdentity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main_mod = _load_main_version()
        cls.branch_mod = _stubs.fresh_import('etc.MotorControl')

    def setUp(self):
        if self.main_mod is None:
            self.skipTest('git or the main branch is not available')
        self.old, self.new = _make(self.main_mod), _make(self.branch_mod)
        self.assertEqual(bytes(self.old.written), bytes(self.new.written), 'init bytes differ')

    def _compare(self, call):
        """Same bytes AND same outcome (main's move_by_angle crashes on float
        speeds; that pre-existing behaviour must be preserved, not fixed)."""
        outcomes = []
        for mc in (self.old, self.new):
            mc.written.clear()
            try:
                call(mc)
                outcomes.append(None)
            except Exception as e:          # noqa: BLE001
                outcomes.append(type(e).__name__)
        self.assertEqual(outcomes[0], outcomes[1])
        self.assertEqual(bytes(self.old.written), bytes(self.new.written))

    def test_move_by_components_all_branches(self):
        comps = [-1.0, -0.6, -0.03, 0.0, 0.03, 0.5, 1.0]
        steps = [-37, -5, 0, 5, 37]
        speeds = [None, 127, 254, 60]
        for h, v, sp in itertools.product(comps, comps, speeds):
            for st in itertools.product(steps, repeat=4):
                _set_steps(self.old, *st)
                _set_steps(self.new, *st)
                self._compare(lambda mc: mc.move_by_components(h, v, sp))

    def test_rotate(self):
        for d in [-2.0, -1.0, -0.4, -0.01, 0.0, 0.01, 0.4, 1.0, 2.0]:
            self._compare(lambda mc: mc.rotate(d))

    def test_move_by_angle_and_wheel(self):
        for ang in range(0, 360, 15):
            self._compare(lambda mc: mc.move_by_angle(ang, 100))
        for w in ('top', 'bottom', 'left', 'right'):
            self._compare(lambda mc: mc.move_by_wheel(w, 90))

    def test_reset_encoders_still_zeroes_control_counters(self):
        _set_steps(self.new, 3, -4, 5, -6)
        self.new.reset_encoders()
        self.assertEqual((self.new.encoderTop.steps, self.new.encoderBottom.steps,
                          self.new.encoderLeft.steps, self.new.encoderRight.steps), (0, 0, 0, 0))


class Telemetry(unittest.TestCase):
    def setUp(self):
        self.mc = _make(_stubs.fresh_import('etc.MotorControl'))

    def test_totals_continuous_across_reset(self):
        _set_steps(self.mc, top=10, bottom=-12, left=7, right=-9)
        self.assertEqual(self.mc.read_encoders(), (7, -9, 10, -12))
        self.mc.reset_encoders()
        self.assertEqual(self.mc.read_encoders(), (7, -9, 10, -12))
        _set_steps(self.mc, top=1, bottom=-1, left=2, right=-2)
        self.assertEqual(self.mc.read_encoders(), (9, -11, 11, -13))

    def test_last_duties_match_commands(self):
        self.mc.move_by_components(0.5, 0.0, 100)          # forward, no encoder history
        self.assertEqual(self.mc.last_duties, (50, -50, 0, 0))
        self.mc.move_by_components(0.0, -1.0, 100)         # left
        self.assertEqual(self.mc.last_duties, (0, 0, -100, 100))
        self.mc.rotate(1.0)
        self.assertEqual(self.mc.last_duties, (127, 127, 127, 127))


if __name__ == '__main__':
    unittest.main()

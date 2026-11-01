import math
import unittest

import _stubs

_stubs.install()
from etc.lidar_revolution import bin_revolution, body_angle_deg, SCAN_FULL_BINS  # noqa: E402


class LidarRevolution(unittest.TestCase):
    def test_same_convention_as_lidar_scans(self):
        """Rounded body angle must equal the index lidar_node uses for lidar/scans."""
        for offset in (0, 90, 270):
            for raw10 in range(0, 3600, 7):
                raw = raw10 / 10.0
                legacy_idx = 359 - round(raw + offset) % 360
                b = body_angle_deg(raw, offset)
                diff = (b - legacy_idx + 180) % 360 - 180
                self.assertLessEqual(abs(diff), 0.5 + 1e-9, (raw, offset, b, legacy_idx))

    def test_binning_units_and_gaps(self):
        samples = [(0.0, 1000.0), (90.0, 2500.0), (180.25, 4000.0)]
        r = bin_revolution(samples, lidar_offset_deg=90)
        self.assertEqual(len(r), SCAN_FULL_BINS)
        self.assertEqual(sum(1 for x in r if not math.isnan(x)), 3)
        # raw 0 with offset 90 -> body 269 deg -> bin 538
        self.assertAlmostEqual(r[538], 1.0)
        # raw 90 -> body 179 -> bin 358 ; raw 180.25 -> body 88.75 -> bin 178 (round 177.5 -> 178)
        self.assertAlmostEqual(r[358], 2.5)
        self.assertAlmostEqual(r[178], 4.0)

    def test_last_sample_wins(self):
        r = bin_revolution([(10.0, 1000.0), (10.1, 2000.0)], 0)
        self.assertEqual(sorted(x for x in r if not math.isnan(x)), [2.0])


if __name__ == '__main__':
    unittest.main()

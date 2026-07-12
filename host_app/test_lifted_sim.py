import unittest

try:
    from .lifted_sim import LiftedCarSimulator
    from .telemetry import TelemetryFrame
except ImportError:
    from lifted_sim import LiftedCarSimulator
    from telemetry import TelemetryFrame


class LiftedCarSimulatorTest(unittest.TestCase):
    def test_straight_encoder_counts_move_forward_without_heading_change(self):
        sim = LiftedCarSimulator()

        frame = sim.apply(TelemetryFrame(t=10, adc=1785, l=100, r=100))

        self.assertGreater(frame.x, 0.0)
        self.assertAlmostEqual(frame.y, 0.0, places=4)
        self.assertAlmostEqual(frame.h, 0.0, places=4)

    def test_steered_encoder_counts_change_heading(self):
        sim = LiftedCarSimulator()

        frames = [sim.apply(TelemetryFrame(t=i * 10, adc=2300, l=100, r=100))
                  for i in range(20)]

        self.assertGreater(abs(frames[-1].h), 1.0)
        self.assertNotAlmostEqual(frames[-1].y, 0.0, places=4)

    def test_distance_delta_drives_lifted_motion_when_encoder_counts_are_zero(self):
        sim = LiftedCarSimulator()

        sim.apply(TelemetryFrame(t=10, d=1000.0, adc=2300, l=0, r=0))
        frame = sim.apply(TelemetryFrame(t=20, d=1010.0, adc=2300, l=0, r=0))

        self.assertGreater(frame.x, 0.0)
        self.assertGreater(abs(frame.h), 1.0)
        self.assertNotAlmostEqual(frame.y, 0.0, places=4)

    def test_first_cumulative_distance_frame_sets_baseline_without_motion(self):
        sim = LiftedCarSimulator()

        frame = sim.apply(TelemetryFrame(t=10, d=1744.5, adc=1785, l=0, r=0))

        self.assertAlmostEqual(frame.x, 0.0)
        self.assertAlmostEqual(frame.y, 0.0)
        self.assertAlmostEqual(frame.d, 0.0)
        self.assertEqual(sim.last_distance_source, "base")

    def test_debug_values_report_distance_source_and_steer_angle(self):
        sim = LiftedCarSimulator()

        sim.apply(TelemetryFrame(t=10, d=1000.0, adc=1206, l=0, r=0))
        sim.apply(TelemetryFrame(t=20, d=1010.0, adc=1206, l=0, r=0))

        self.assertEqual(sim.last_distance_source, "d")
        self.assertAlmostEqual(sim.last_distance_delta_cm, 10.0)
        self.assertLess(sim.last_steer_deg, -20.0)

    def test_passthrough_keeps_non_pose_fields(self):
        sim = LiftedCarSimulator()

        frame = sim.apply(TelemetryFrame(mode="RUN", v=25, st=40, p=3, n=9,
                                         adc=2300, l=100, r=80))

        self.assertEqual(frame.mode, "RUN")
        self.assertEqual(frame.v, 25)
        self.assertEqual(frame.st, 40)
        self.assertEqual(frame.p, 3)
        self.assertEqual(frame.n, 9)


if __name__ == "__main__":
    unittest.main()

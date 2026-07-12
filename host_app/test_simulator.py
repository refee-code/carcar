import unittest

try:
    from .simulator import (
        CRUISE_25_SPEED_CM_S,
        MAX_STEER_DEG,
        REAR_TRACK_CM,
        PathSimulator,
        subject3_u_route,
    )
except ImportError:
    from simulator import (
        CRUISE_25_SPEED_CM_S,
        MAX_STEER_DEG,
        REAR_TRACK_CM,
        PathSimulator,
        subject3_u_route,
    )


class PathSimulatorTest(unittest.TestCase):
    def test_front_wheel_total_range_sets_half_angle_limit(self):
        self.assertEqual(MAX_STEER_DEG, 29.0)

    def test_rear_track_width_matches_vehicle(self):
        self.assertEqual(REAR_TRACK_CM, 62.5)

    def test_cruise_25_speed_uses_measured_midpoint(self):
        self.assertEqual(CRUISE_25_SPEED_CM_S, 41.5)

    def test_subject3_route_has_u_shape_points(self):
        route = subject3_u_route()

        self.assertGreaterEqual(len(route), 4)
        self.assertEqual(route[0], (0.0, 0.0))
        self.assertGreater(route[-1][1], route[0][1])

    def test_record_frames_are_emitted_before_run_frames(self):
        sim = PathSimulator(subject3_u_route())

        frames = [sim.step() for _ in range(40)]

        self.assertEqual(frames[0].mode, "REC")
        self.assertIn("RUN", {frame.mode for frame in frames})

    def test_run_moves_vehicle_and_reports_control_values(self):
        sim = PathSimulator(subject3_u_route(), record_frames=1)

        frames = [sim.step() for _ in range(90)]
        run_frames = [frame for frame in frames if frame.mode == "RUN"]

        self.assertGreater(len(run_frames), 5)
        self.assertGreater(max(frame.d for frame in run_frames), 0.0)
        self.assertTrue(any(abs(frame.st) > 1.0 for frame in run_frames))
        self.assertTrue(all(frame.n == len(subject3_u_route()) for frame in run_frames))
        self.assertTrue(any(frame.l != 0 or frame.r != 0 for frame in run_frames))
        self.assertTrue(any(frame.l != frame.r for frame in run_frames))


if __name__ == "__main__":
    unittest.main()

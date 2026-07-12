import unittest

try:
    from .telemetry import TelemetryFrame, csv_header, frame_to_csv_row, parse_telemetry_line
except ImportError:
    from telemetry import TelemetryFrame, csv_header, frame_to_csv_row, parse_telemetry_line


class TelemetryParseTest(unittest.TestCase):
    def test_parse_full_tel_line(self):
        frame = parse_telemetry_line(
            "TEL t=12345 mode=RUN x=12.3 y=-4.5 h=18 d=120 "
            "v=25 st=40 p=12 n=80 e=15 adc=1785 se=20 so=2800 l=30 r=-32 "
            "wh=1.2 sa=-8.5 th=0.9 hc=0.1 co=1"
        )

        self.assertEqual(
            frame,
            TelemetryFrame(
                t=12345,
                mode="RUN",
                x=12.3,
                y=-4.5,
                h=18.0,
                d=120.0,
                v=25.0,
                st=40.0,
                p=12,
                n=80,
                e=15.0,
                adc=1785,
                se=20,
                so=2800,
                l=30,
                r=-32,
                wh=1.2,
                sa=-8.5,
                th=0.9,
                hc=0.1,
                co=1,
            ),
        )

    def test_ignores_non_tel_lines(self):
        self.assertIsNone(parse_telemetry_line("STEER_CALIB ADC=1785"))

    def test_partial_line_uses_defaults(self):
        frame = parse_telemetry_line("TEL mode=IDLE adc=1790")

        self.assertEqual(frame.mode, "IDLE")
        self.assertEqual(frame.adc, 1790)
        self.assertEqual(frame.x, 0.0)

    def test_csv_helpers_match_dataclass_fields(self):
        frame = TelemetryFrame(t=1, mode="RUN", adc=1800)

        self.assertEqual(csv_header()[0:3], ["t", "mode", "x"])
        self.assertEqual(frame_to_csv_row(frame)[0], 1)
        self.assertEqual(frame_to_csv_row(frame)[1], "RUN")
        self.assertIn(1800, frame_to_csv_row(frame))


if __name__ == "__main__":
    unittest.main()

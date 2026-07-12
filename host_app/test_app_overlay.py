import sys
import unittest
from pathlib import Path

HOST_APP_DIR = Path(__file__).resolve().parent
if str(HOST_APP_DIR) not in sys.path:
    sys.path.insert(0, str(HOST_APP_DIR))

from app import format_map_overlay_lines
from telemetry import TelemetryFrame


class MapOverlayFormatTest(unittest.TestCase):
    def test_overlay_contains_photo_debug_fields(self):
        frame = TelemetryFrame(
            t=1200,
            mode="AUTO",
            x=12.3,
            y=-4.5,
            h=18.6,
            d=125.0,
            v=25.0,
            st=40.0,
            p=7,
            n=33,
            e=-12.5,
            adc=1789,
            se=-6,
            so=2950,
            l=123,
            r=-121,
            wh=1.5,
            sa=-8.0,
            th=1.2,
            hc=0.1,
            co=1,
        )

        lines = format_map_overlay_lines(
            frame,
            raw_count=50,
            tel_count=49,
            lifted_enabled=False,
            recorded_count=15,
            actual_count=20,
        )
        text = "\n".join(lines)

        self.assertIn("MODE AUTO", text)
        self.assertIn("TEL 49 RAW 50", text)
        self.assertIn("LIFT OFF", text)
        self.assertIn("x=12.3 y=-4.5 h=18.6 d=125.0cm", text)
        self.assertIn("v=25.0 st=40.0 e=-12.5 p=7/33", text)
        self.assertIn("adc=1789 se=-6 so=2950", text)
        self.assertIn("L=123 R=-121", text)
        self.assertIn("wh=1.5 sa=-8.0 th=1.2 hc=0.10 co=1", text)
        self.assertIn("rec=15 run=20", text)


if __name__ == "__main__":
    unittest.main()

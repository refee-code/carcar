import re
import unittest
from pathlib import Path


CONFIG_PATH = Path(__file__).resolve().parents[1] / "auto" / "auto_config.h"


def final_define_value(name: str) -> str:
    pattern = re.compile(rf"^\s*#define\s+{re.escape(name)}\s+\(?([0-9.]+)f?\)?")
    value = None
    for line in CONFIG_PATH.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = pattern.match(line)
        if match:
            value = match.group(1)
    if value is None:
        raise AssertionError(f"{name} is not defined")
    return value


class FirmwareConfigTest(unittest.TestCase):
    def test_subject3_replay_uses_tighter_turn_settings(self):
        self.assertEqual(final_define_value("AUTO_LOOKAHEAD_DISTANCE_CM"), "25.0")
        self.assertEqual(final_define_value("AUTO_RECORD_INTERVAL_CM"), "20.0")
        self.assertEqual(final_define_value("AUTO_NAV_MAX_HEADING_ERROR_DEG"), "60.0")
        self.assertEqual(final_define_value("AUTO_CROSS_TRACK_STEER_LIMIT"), "35.0")


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import math
from dataclasses import replace

try:
    from .simulator import MAX_STEER_DEG, WHEELBASE_CM
    from .telemetry import TelemetryFrame
except ImportError:
    from simulator import MAX_STEER_DEG, WHEELBASE_CM
    from telemetry import TelemetryFrame


STEER_ADC_LEFT = 1117
STEER_ADC_CENTER = 1785
STEER_ADC_RIGHT = 2482
ENCODER_CM_PER_COUNT = 0.003


def _normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _adc_to_steer_deg(adc: int) -> float:
    if adc < STEER_ADC_CENTER:
        span = max(STEER_ADC_CENTER - STEER_ADC_LEFT, 1)
        percent = (adc - STEER_ADC_CENTER) / span
    else:
        span = max(STEER_ADC_RIGHT - STEER_ADC_CENTER, 1)
        percent = (adc - STEER_ADC_CENTER) / span
    percent = max(-1.0, min(1.0, percent))
    return percent * MAX_STEER_DEG


class LiftedCarSimulator:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.heading_deg = 0.0
        self.distance_cm = 0.0
        self._last_frame_distance_cm: float | None = None
        self.last_distance_delta_cm = 0.0
        self.last_distance_source = "--"
        self.last_steer_deg = 0.0

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.heading_deg = 0.0
        self.distance_cm = 0.0
        self._last_frame_distance_cm = None
        self.last_distance_delta_cm = 0.0
        self.last_distance_source = "--"
        self.last_steer_deg = 0.0

    def apply(self, frame: TelemetryFrame) -> TelemetryFrame:
        distance_delta_cm = self._distance_delta_from_frame(frame)
        steer_deg = _adc_to_steer_deg(frame.adc)
        self.last_distance_delta_cm = distance_delta_cm
        self.last_steer_deg = steer_deg

        heading_rate_rad_per_cm = math.tan(math.radians(steer_deg)) / WHEELBASE_CM
        heading_delta_deg = math.degrees(heading_rate_rad_per_cm * distance_delta_cm)

        mid_heading = self.heading_deg + heading_delta_deg * 0.5
        self.x += math.cos(math.radians(mid_heading)) * distance_delta_cm
        self.y += math.sin(math.radians(mid_heading)) * distance_delta_cm
        self.heading_deg = _normalize_angle_deg(self.heading_deg + heading_delta_deg)
        self.distance_cm += distance_delta_cm

        return replace(
            frame,
            x=self.x,
            y=self.y,
            h=self.heading_deg,
            d=self.distance_cm,
        )

    def _distance_delta_from_frame(self, frame: TelemetryFrame) -> float:
        if self._last_frame_distance_cm is None:
            self._last_frame_distance_cm = frame.d
            if abs(frame.d) > 0.0001:
                self.last_distance_source = "base"
                return 0.0
        else:
            delta = frame.d - self._last_frame_distance_cm
            self._last_frame_distance_cm = frame.d
            if abs(delta) > 0.0001:
                self.last_distance_source = "d"
                return delta

        self.last_distance_source = "enc"
        return (frame.l + frame.r) * 0.5 * ENCODER_CM_PER_COUNT

from __future__ import annotations

import math
from dataclasses import dataclass

try:
    from .telemetry import TelemetryFrame
except ImportError:
    from telemetry import TelemetryFrame


WHEELBASE_CM = 62.5
REAR_TRACK_CM = 62.5
MAX_STEER_DEG = 29.0
CRUISE_25_SPEED_CM_S = 41.5
SPEED_CM_S_PER_PERCENT = CRUISE_25_SPEED_CM_S / 25.0


def subject3_u_route() -> list[tuple[float, float]]:
    return [
        (0.0, 0.0),
        (80.0, 0.0),
        (160.0, 0.0),
        (230.0, 20.0),
        (270.0, 80.0),
        (270.0, 150.0),
        (230.0, 210.0),
        (160.0, 230.0),
        (80.0, 230.0),
        (0.0, 230.0),
    ]


def _normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


@dataclass
class VehicleState:
    x: float
    y: float
    heading_deg: float
    distance_cm: float = 0.0


class PathSimulator:
    def __init__(
        self,
        route: list[tuple[float, float]] | None = None,
        *,
        wheelbase_cm: float = WHEELBASE_CM,
        record_frames: int | None = None,
        step_ms: int = 50,
    ) -> None:
        self.route = route or subject3_u_route()
        self.wheelbase_cm = wheelbase_cm
        self.step_ms = step_ms
        self.record_frames = record_frames if record_frames is not None else len(self.route)
        self._frame_index = 0
        self._target_index = 1
        self._done = False
        self.state = VehicleState(
            x=self.route[0][0],
            y=self.route[0][1],
            heading_deg=self._initial_heading(),
        )

    def step(self) -> TelemetryFrame:
        if self._frame_index < self.record_frames:
            return self._record_frame()
        return self._run_frame()

    def _record_frame(self) -> TelemetryFrame:
        route_index = min(self._frame_index, len(self.route) - 1)
        x, y = self.route[route_index]
        heading = self._route_heading(route_index)
        frame = TelemetryFrame(
            t=self._frame_index * self.step_ms,
            mode="REC",
            x=x,
            y=y,
            h=heading,
            d=self._route_distance_to(route_index),
            p=route_index,
            n=len(self.route),
            adc=1785,
        )
        self._frame_index += 1
        return frame

    def _run_frame(self) -> TelemetryFrame:
        dt = self.step_ms / 1000.0
        target = self.route[self._target_index]
        dx = target[0] - self.state.x
        dy = target[1] - self.state.y
        target_distance = math.hypot(dx, dy)

        if target_distance < 18.0 and self._target_index < len(self.route) - 1:
            self._target_index += 1
            target = self.route[self._target_index]
            dx = target[0] - self.state.x
            dy = target[1] - self.state.y
            target_distance = math.hypot(dx, dy)

        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error = _normalize_angle_deg(target_heading - self.state.heading_deg)
        steer_percent = max(-100.0, min(100.0, heading_error * 2.0))

        if self._done:
            speed_percent = 0.0
        else:
            speed_percent = 25.0

        speed_cms = speed_percent * SPEED_CM_S_PER_PERCENT
        steer_angle_deg = steer_percent / 100.0 * MAX_STEER_DEG
        heading_rate = (
            speed_cms / self.wheelbase_cm *
            math.tan(math.radians(steer_angle_deg)) *
            180.0 / math.pi
        )
        yaw_rate_rad_s = math.radians(heading_rate)

        old_x = self.state.x
        old_y = self.state.y
        self.state.heading_deg = _normalize_angle_deg(
            self.state.heading_deg + heading_rate * dt
        )
        self.state.x += math.cos(math.radians(self.state.heading_deg)) * speed_cms * dt
        self.state.y += math.sin(math.radians(self.state.heading_deg)) * speed_cms * dt
        self.state.distance_cm += math.hypot(self.state.x - old_x, self.state.y - old_y)

        if self._target_index >= len(self.route) - 1 and target_distance < 12.0:
            self._done = True

        adc = int(1785 + steer_percent * 6.5)
        left_speed_cms = speed_cms - yaw_rate_rad_s * REAR_TRACK_CM * 0.5
        right_speed_cms = speed_cms + yaw_rate_rad_s * REAR_TRACK_CM * 0.5
        left_encoder_count = int(left_speed_cms * dt / 0.003)
        right_encoder_count = int(right_speed_cms * dt / 0.003)
        if self._done:
            left_encoder_count = 0
            right_encoder_count = 0

        frame = TelemetryFrame(
            t=self._frame_index * self.step_ms,
            mode="DONE" if self._done else "RUN",
            x=self.state.x,
            y=self.state.y,
            h=self.state.heading_deg,
            d=self.state.distance_cm,
            v=speed_percent,
            st=steer_percent,
            p=self._target_index,
            n=len(self.route),
            e=heading_error,
            adc=adc,
            se=int(1785 - adc),
            so=int(steer_percent * 60.0),
            l=left_encoder_count,
            r=right_encoder_count,
        )
        self._frame_index += 1
        return frame

    def _initial_heading(self) -> float:
        if len(self.route) < 2:
            return 0.0
        return math.degrees(
            math.atan2(
                self.route[1][1] - self.route[0][1],
                self.route[1][0] - self.route[0][0],
            )
        )

    def _route_heading(self, index: int) -> float:
        if index + 1 < len(self.route):
            a = self.route[index]
            b = self.route[index + 1]
        elif index > 0:
            a = self.route[index - 1]
            b = self.route[index]
        else:
            return 0.0
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

    def _route_distance_to(self, index: int) -> float:
        total = 0.0
        for i in range(0, min(index, len(self.route) - 1)):
            total += _distance(self.route[i], self.route[i + 1])
        return total

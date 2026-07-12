from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Dict, Optional


@dataclass
class TelemetryFrame:
    t: int = 0
    mode: str = "--"
    x: float = 0.0
    y: float = 0.0
    h: float = 0.0
    d: float = 0.0
    v: float = 0.0
    st: float = 0.0
    p: int = 0
    n: int = 0
    e: float = 0.0
    adc: int = 0
    se: int = 0
    so: int = 0
    l: int = 0
    r: int = 0
    wh: float = 0.0
    sa: float = 0.0
    th: float = 0.0
    hc: float = 0.0
    co: int = 0


_FIELD_TYPES = {field.name: field.type for field in fields(TelemetryFrame)}


def parse_telemetry_line(line: str) -> Optional[TelemetryFrame]:
    text = line.strip()
    if not text or not text.startswith("TEL "):
        return None

    values: Dict[str, object] = {}
    for token in text.split()[1:]:
        if "=" not in token:
            continue
        key, raw_value = token.split("=", 1)
        if key not in _FIELD_TYPES:
            continue

        try:
            if key == "mode":
                values[key] = raw_value
            elif key in {"t", "p", "n", "adc", "se", "so", "l", "r", "co"}:
                values[key] = int(float(raw_value))
            else:
                values[key] = float(raw_value)
        except ValueError:
            continue

    return TelemetryFrame(**values)


def frame_to_csv_row(frame: TelemetryFrame) -> list[object]:
    return [getattr(frame, field.name) for field in fields(TelemetryFrame)]


def csv_header() -> list[str]:
    return [field.name for field in fields(TelemetryFrame)]

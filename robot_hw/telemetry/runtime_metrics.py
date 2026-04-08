"""Runtime telemetry helpers for structured logs, metrics snapshots, and health events.

These utilities are intentionally lightweight and file-friendly so they can be
used from simulation, benchmarks, and the live orchestrator without external
infrastructure. They produce deterministic dictionaries ready for JSON logging,
telemetry transport, or CI regression artifact capture.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class HealthEvent:
    """A structured runtime health event."""

    severity: str
    source: str
    message: str
    cycle: int
    timestamp: float | None
    details: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "severity": self.severity,
            "source": self.source,
            "message": self.message,
            "cycle": self.cycle,
            "timestamp": self.timestamp,
            "details": dict(self.details),
        }


def build_runtime_metrics(
    *,
    cycle: int,
    loop_duration_s: float,
    state: dict[str, Any],
    telemetry_count: int,
    fault_count: int,
    hazard_count: int,
) -> dict[str, Any]:
    """Build a deterministic runtime metrics snapshot."""

    navigation = state.get("navigation", {}) if isinstance(state, dict) else {}
    map_summary = state.get("map", {}) if isinstance(state, dict) else {}
    return {
        "cycle": int(cycle),
        "loop_duration_s": float(loop_duration_s),
        "timestamp": state.get("timestamp") if isinstance(state, dict) else None,
        "hazard_count": int(hazard_count),
        "fault_count": int(fault_count),
        "telemetry_count": int(telemetry_count),
        "navigation_status": navigation.get("status") if isinstance(navigation, dict) else None,
        "global_plan_waypoints": len(state.get("global_plan", [])) if isinstance(state, dict) else 0,
        "local_plan_waypoints": len(state.get("local_plan", [])) if isinstance(state, dict) else 0,
        "occupied_cells": map_summary.get("occupied_cells") if isinstance(map_summary, dict) else None,
        "inflated_cells": map_summary.get("inflated_cells") if isinstance(map_summary, dict) else None,
    }


def build_health_events(
    *,
    cycle: int,
    timestamp: float | None,
    battery_ok: bool,
    thermal_ok: bool,
    consistency_ok: bool,
    faults: list[str],
    hazards: dict[str, Any],
) -> list[HealthEvent]:
    """Create structured health events from runtime state."""

    events: list[HealthEvent] = []
    if not battery_ok:
        events.append(
            HealthEvent(
                severity="warning",
                source="battery",
                message="Battery manager reported degraded operating margin.",
                cycle=cycle,
                timestamp=timestamp,
            )
        )
    if not thermal_ok:
        events.append(
            HealthEvent(
                severity="warning",
                source="thermal",
                message="Thermal manager reported limit exceedance or cooling requirement.",
                cycle=cycle,
                timestamp=timestamp,
            )
        )
    if not consistency_ok:
        events.append(
            HealthEvent(
                severity="error",
                source="consistency",
                message="Consistency verification failed for the current state snapshot.",
                cycle=cycle,
                timestamp=timestamp,
            )
        )
    for fault in faults:
        events.append(
            HealthEvent(
                severity="error",
                source="fault_detector",
                message=str(fault),
                cycle=cycle,
                timestamp=timestamp,
            )
        )
    for name, payload in hazards.items():
        events.append(
            HealthEvent(
                severity="warning",
                source="hazard_manager",
                message=f"Hazard active: {name}",
                cycle=cycle,
                timestamp=timestamp,
                details=payload if isinstance(payload, dict) else {"value": payload},
            )
        )
    return events


def build_structured_log(
    *,
    cycle: int,
    state: dict[str, Any],
    metrics: dict[str, Any],
    events: list[HealthEvent],
) -> dict[str, Any]:
    """Build a single structured log entry for a control-loop iteration."""

    return {
        "cycle": int(cycle),
        "timestamp": state.get("timestamp") if isinstance(state, dict) else None,
        "metrics": dict(metrics),
        "events": [event.as_dict() for event in events],
        "state_summary": {
            "hazard_flags": state.get("hazard_flags", {}) if isinstance(state, dict) else {},
            "navigation": state.get("navigation", {}) if isinstance(state, dict) else {},
            "map": state.get("map", {}) if isinstance(state, dict) else {},
        },
    }

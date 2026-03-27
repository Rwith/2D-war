"""
Dual-servo redundancy manager for a single control surface.

One servo is ACTIVE and drives the surface.
The other is STANDBY and tracks the same position passively.
If the active servo fails, the standby is promoted automatically.

Failure is detected by three independent checks (any one triggers failover):
  1. Position error  — actual position deviates too far from commanded
  2. Current draw    — outside the expected operating band
  3. Heartbeat       — servo stops responding within a timeout window

Usage
-----
    from servo_redundancy import ServoRedundancyManager, ServoInterface

    left  = ServoInterface(servo_id="left",  channel=0)
    right = ServoInterface(servo_id="right", channel=1)
    mgr   = ServoRedundancyManager(primary=left, secondary=right)

    # In your control loop:
    mgr.set_position(degrees)   # send command
    mgr.update()                # call every loop tick — handles monitoring/failover
    status = mgr.get_status()
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


# ---------------------------------------------------------------------------
# Configurable thresholds
# ---------------------------------------------------------------------------

# How far (degrees) actual position may lag command before treated as failure
POSITION_ERROR_THRESHOLD_DEG: float = 10.0

# Current draw limits (Amps). Outside this band = failure.
CURRENT_MIN_A: float = 0.01   # below this → servo unplugged / signal lost
CURRENT_MAX_A: float = 3.0    # above this → stalled / jammed

# Seconds without a valid heartbeat response before failure is declared
HEARTBEAT_TIMEOUT_S: float = 0.5

# Consecutive failed checks required before triggering failover (debounce)
FAILURE_COUNT_THRESHOLD: int = 3


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

class ServoState(Enum):
    ACTIVE    = auto()   # driving the surface
    STANDBY   = auto()   # tracking position, ready to take over
    FAILED    = auto()   # confirmed fault, removed from service
    DISABLED  = auto()   # manually taken offline


class FailureReason(Enum):
    NONE             = auto()
    POSITION_ERROR   = auto()
    OVERCURRENT      = auto()
    UNDERCURRENT     = auto()
    HEARTBEAT_LOSS   = auto()
    MANUAL_OVERRIDE  = auto()


@dataclass
class ServoHealth:
    position_deg:     float = 0.0    # last known actual position
    current_a:        float = 0.0    # last measured current draw
    last_heartbeat:   float = field(default_factory=time.monotonic)
    failure_count:    int   = 0
    failure_reason:   FailureReason = FailureReason.NONE


@dataclass
class RedundancyStatus:
    active_servo_id:   str
    standby_servo_id:  Optional[str]
    commanded_deg:     float
    failover_count:    int
    failed_servos:     list[str]
    log:               list[str]


# ---------------------------------------------------------------------------
# Servo interface — replace the body of each method with real hardware calls
# ---------------------------------------------------------------------------

class ServoInterface:
    """
    Thin wrapper around a physical servo.

    In production, replace the placeholder bodies with your actual
    servo driver calls (e.g. PCA9685, flight-controller MAVLink, etc.)
    """

    def __init__(self, servo_id: str, channel: int):
        self.servo_id = servo_id
        self.channel  = channel

        # Internal simulation state — remove when using real hardware
        self._actual_position_deg: float = 0.0
        self._current_a:           float = 0.2
        self._alive:               bool  = True

    # --- Commands -----------------------------------------------------------

    def send_position(self, degrees: float) -> None:
        """Send a position command to the servo (–90° … +90°)."""
        # TODO: replace with real driver call, e.g.:
        #   self._pwm.set_pulse(self.channel, degrees_to_pulse(degrees))
        if self._alive:
            self._actual_position_deg = degrees      # simulated instant response

    def enable(self) -> None:
        """Power / enable torque on the servo."""
        # TODO: self._pwm.enable(self.channel)
        self._alive = True

    def disable(self) -> None:
        """Release torque (servo goes limp — only call on FAILED servo)."""
        # TODO: self._pwm.disable(self.channel)
        self._alive = False

    # --- Telemetry ----------------------------------------------------------

    def read_position(self) -> float:
        """Return actual shaft position in degrees."""
        # TODO: return self._encoder.read_degrees(self.channel)
        return self._actual_position_deg

    def read_current(self) -> float:
        """Return current draw in Amps."""
        # TODO: return self._current_sensor.read_amps(self.channel)
        return self._current_a if self._alive else 0.0

    def ping(self) -> bool:
        """Return True if the servo acknowledges a heartbeat request."""
        # TODO: return self._bus.ping(self.channel)
        return self._alive


# ---------------------------------------------------------------------------
# Redundancy manager
# ---------------------------------------------------------------------------

class ServoRedundancyManager:
    """
    Manages two servos on the same control surface.

    primary   — starts as ACTIVE
    secondary — starts as STANDBY

    Call set_position() and update() on every control loop tick.
    """

    def __init__(
        self,
        primary:   ServoInterface,
        secondary: ServoInterface,
    ):
        self._servos = {
            primary.servo_id:   primary,
            secondary.servo_id: secondary,
        }
        self._health: dict[str, ServoHealth] = {
            sid: ServoHealth() for sid in self._servos
        }
        self._state: dict[str, ServoState] = {
            primary.servo_id:   ServoState.ACTIVE,
            secondary.servo_id: ServoState.STANDBY,
        }

        self._commanded_deg: float = 0.0
        self._failover_count: int  = 0
        self._failed_ids:    list[str] = []
        self._log:           list[str] = []

        # Enable both immediately so standby is always ready
        for servo in self._servos.values():
            servo.enable()

    # --- Public API ---------------------------------------------------------

    def set_position(self, degrees: float) -> None:
        """
        Command a position.

        ACTIVE  servo receives the command immediately.
        STANDBY servo tracks the same value so it can take over seamlessly.
        """
        self._commanded_deg = degrees
        for sid, state in self._state.items():
            if state in (ServoState.ACTIVE, ServoState.STANDBY):
                self._servos[sid].send_position(degrees)

    def update(self) -> None:
        """
        Run one monitoring cycle. Call this every control loop tick.

        Reads telemetry, checks health, and triggers failover if needed.
        """
        now = time.monotonic()

        for sid, servo in self._servos.items():
            if self._state[sid] in (ServoState.FAILED, ServoState.DISABLED):
                continue

            health = self._health[sid]
            reason = self._check_health(servo, health, now)

            if reason != FailureReason.NONE:
                health.failure_count += 1
                health.failure_reason = reason
                if health.failure_count >= FAILURE_COUNT_THRESHOLD:
                    self._declare_failure(sid, reason)
            else:
                health.failure_count = 0   # reset debounce on clean reading

    def get_status(self) -> RedundancyStatus:
        active_id  = self._active_id()
        standby_id = self._standby_id()
        return RedundancyStatus(
            active_servo_id  = active_id or "NONE",
            standby_servo_id = standby_id,
            commanded_deg    = self._commanded_deg,
            failover_count   = self._failover_count,
            failed_servos    = list(self._failed_ids),
            log              = list(self._log),
        )

    def force_failover(self) -> bool:
        """
        Manually trigger failover (e.g. from ground-station command).
        Returns True if a standby servo was available to promote.
        """
        active = self._active_id()
        if active:
            self._declare_failure(active, FailureReason.MANUAL_OVERRIDE)
            return True
        return False

    def is_surface_controllable(self) -> bool:
        """Return True as long as at least one servo is ACTIVE."""
        return self._active_id() is not None

    # --- Internal -----------------------------------------------------------

    def _check_health(
        self,
        servo:  ServoInterface,
        health: ServoHealth,
        now:    float,
    ) -> FailureReason:
        """
        Run all three health checks. Return the first failure found,
        or FailureReason.NONE if the servo is healthy.
        """
        # 1. Heartbeat
        if servo.ping():
            health.last_heartbeat = now
        elif (now - health.last_heartbeat) > HEARTBEAT_TIMEOUT_S:
            return FailureReason.HEARTBEAT_LOSS

        # 2. Position error (only meaningful for the ACTIVE servo)
        if self._state[servo.servo_id] == ServoState.ACTIVE:
            actual = servo.read_position()
            health.position_deg = actual
            if abs(actual - self._commanded_deg) > POSITION_ERROR_THRESHOLD_DEG:
                return FailureReason.POSITION_ERROR

        # 3. Current draw
        current = servo.read_current()
        health.current_a = current
        if current < CURRENT_MIN_A:
            return FailureReason.UNDERCURRENT
        if current > CURRENT_MAX_A:
            return FailureReason.OVERCURRENT

        return FailureReason.NONE

    def _declare_failure(self, failed_id: str, reason: FailureReason) -> None:
        """Mark a servo as failed, disable it, and promote standby if available."""
        self._state[failed_id] = ServoState.FAILED
        self._failed_ids.append(failed_id)
        self._servos[failed_id].disable()

        msg = (f"[{time.monotonic():.3f}s] FAIL: {failed_id} "
               f"— reason: {reason.name}")
        self._log.append(msg)
        print(msg)

        # Promote standby → active
        standby = self._standby_id()
        if standby:
            self._state[standby] = ServoState.ACTIVE
            self._failover_count += 1
            ok_msg = (f"[{time.monotonic():.3f}s] FAILOVER #{self._failover_count}: "
                      f"{standby} is now ACTIVE")
            self._log.append(ok_msg)
            print(ok_msg)
        else:
            crit = "[CRITICAL] No standby servo available — surface uncontrolled!"
            self._log.append(crit)
            print(crit)

    def _active_id(self) -> Optional[str]:
        for sid, state in self._state.items():
            if state == ServoState.ACTIVE:
                return sid
        return None

    def _standby_id(self) -> Optional[str]:
        for sid, state in self._state.items():
            if state == ServoState.STANDBY:
                return sid
        return None


# ---------------------------------------------------------------------------
# Demo / smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import math

    left  = ServoInterface(servo_id="left",  channel=0)
    right = ServoInterface(servo_id="right", channel=1)
    mgr   = ServoRedundancyManager(primary=left, secondary=right)

    print("=== Dual-servo redundancy demo ===\n")

    # Normal sweep — both servos healthy
    print("-- Phase 1: Normal operation --")
    for i in range(6):
        angle = math.sin(i * 0.5) * 15.0
        mgr.set_position(angle)
        mgr.update()
        s = mgr.get_status()
        print(f"  cmd={angle:+6.1f}°  active={s.active_servo_id}  "
              f"failovers={s.failover_count}")

    # Simulate left servo failure (stall → overcurrent)
    print("\n-- Phase 2: Inject left servo stall --")
    left._current_a = 5.0   # over CURRENT_MAX_A
    left._alive = False      # heartbeat loss too

    for i in range(6):
        mgr.set_position(-10.0)
        mgr.update()
        s = mgr.get_status()
        print(f"  cmd=-10.0°  active={s.active_servo_id}  "
              f"failovers={s.failover_count}  failed={s.failed_servos}")
        if not mgr.is_surface_controllable():
            print("  SURFACE UNCONTROLLED")
            break

    print("\n-- Final status --")
    s = mgr.get_status()
    print(f"  Active servo  : {s.active_servo_id}")
    print(f"  Standby servo : {s.standby_servo_id}")
    print(f"  Failover count: {s.failover_count}")
    print(f"  Failed servos : {s.failed_servos}")
    print("\n  Event log:")
    for entry in s.log:
        print(f"    {entry}")

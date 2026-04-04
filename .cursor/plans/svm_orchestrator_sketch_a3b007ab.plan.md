---
name: SVM orchestrator sketch
overview: Python-level orchestrator that maps dq voltage setpoint to three half-bridge duties via inverse Park, inverse Clarke, and d = V_phase/V_dc (duty 0..1 maps to 0..Vdc), with gate_kill clamping all phases to ground (low-side). No PI here—current loop lives in foc_u_guy.
todos:
  - id: implement-inverse-clarke-duty
    content: inverse_park (theta_e) + inverse_clarke + per-phase d = V_target / V_dc; HalfBridge duty [0,1] per SWC
    status: pending
  - id: wire-fault-snapshot
    content: FaultSnapshot fields match ISR/task; gate_kill applies three-phase ground clamp (low-side on), not Hi-Z
    status: pending
  - id: atomic-commit-hw
    content: Single _commit path in Python sketch; FW uses shadow registers + one trigger when platform requires it
    status: pending
isProject: false
---

# Space vector modulation orchestrator — revised sketch

## Behavior

- On `gate_kill` (or `system_enable=False`), command all three bridges to **phase-low clamp** (ground / DC-), not high-Z.
- No PI loop inside orchestrator. `foc_u_guy` closes the current loop and emits `**V_d`, `V_q`** (and provides or shares `**theta_e`** for the same dq frame the controller uses).
- Orchestrator runs **inverse Park → inverse Clarke → per-phase duty** and publishes three half-bridge commands.
- `_commit` uses shared-memory **pending pointers + commit sequence handshake** (no toggle ACK bit).

## Shared-memory handoff

- `pending` is a trio of pointers/references to `HalfBridgeCommand` (`A,B,C`) plus a shared `commit_seq`.
- Orchestrator write flow:
  1. Write/update pending command objects.
  2. Publish pointers atomically (or under a tiny critical section).
  3. Increment `commit_seq` (release-store semantics).
- Half-bridge read flow:
  1. Poll/wait for `commit_seq != last_seen_seq`.
  2. Read command pointer for its phase and consume command.
  3. Record `last_seen_seq = commit_seq`.
- Health monitoring:
  - Each bridge exposes `consumed_seq`.
  - Orchestrator reads loopback (`commit_seq - min(consumed_seq[])`) and flags out-of-sync when threshold exceeded.
- This is closer to a **sequence-latched mailbox/event counter** than a semaphore.

## Math

- **Inverse Park** (dq → αβ), with electrical angle `theta_e` **radians**, aligned with `foc_u_guy` Park convention:
  - `V_alpha = V_d * cos(theta_e) - V_q * sin(theta_e)`
  - `V_beta  = V_d * sin(theta_e) + V_q * cos(theta_e)`
- **Inverse Clarke** (αβ → abc):
  - `V_a = V_alpha`
  - `V_b = -0.5 * V_alpha + (sqrt(3)/2) * V_beta`
  - `V_c = -0.5 * V_alpha - (sqrt(3)/2) * V_beta`
- Per-phase normalized command:
  - `d_x = V_x / V_dc` for `x in {a,b,c}`
- Half-bridge ICD is **duty in [0,1]** with `**duty * V_dc` ≈ average leg voltage** (0..1 → 0..Vdc). This plan uses `clamp(d, 0, 1)` at orchestrator output.
- `**theta_e` source:** must be the same angle used inside `foc_u_guy` for Park/Inverse Park (rotor vs stator frame, offset, and wrap semantics must match the ICD).

## Python sketch

```python
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, Sequence


@dataclass(frozen=True)
class AlphaBeta:
    alpha: float
    beta: float


@dataclass(frozen=True)
class FaultSnapshot:
    gate_kill: bool
    reason_bits: int = 0


class AsyncFaultBuffer(Protocol):
    def read_snapshot(self) -> FaultSnapshot: ...


@dataclass
class HalfBridgeCommand:
    duty: float  # [0,1]
    enable: bool = True
    brake: bool = False
    clamp_phase_low: bool = False


class HalfBridge(Protocol):
    def apply(self, cmd: HalfBridgeCommand) -> None: ...


def inverse_park(v: Dq, theta_e: float) -> AlphaBeta:
    c = math.cos(theta_e)
    s = math.sin(theta_e)
    return AlphaBeta(alpha=v.d * c - v.q * s, beta=v.d * s + v.q * c)


def inverse_clarke(v: AlphaBeta) -> tuple[float, float, float]:
    k = math.sqrt(3.0) / 2.0
    va = v.alpha
    vb = -0.5 * v.alpha + k * v.beta
    vc = -0.5 * v.alpha - k * v.beta
    return va, vb, vc


def duty01_from_dq(v_cmd: Dq, theta_e: float, v_dc: float) -> tuple[float, float, float]:
    if v_dc <= 0.0:
        raise ValueError("v_dc must be positive")
    ab = inverse_park(v_cmd, theta_e)
    va, vb, vc = inverse_clarke(ab)
    da = max(0.0, min(1.0, va / v_dc))
    db = max(0.0, min(1.0, vb / v_dc))
    dc = max(0.0, min(1.0, vc / v_dc))
    return da, db, dc


class SvmOrchestrator:
    def __init__(self, bridges: Sequence[HalfBridge], fault_buffer: AsyncFaultBuffer) -> None:
        if len(bridges) != 3:
            raise ValueError("requires 3 half-bridges")
        self._bridges = tuple(bridges)
        self._fault_buffer = fault_buffer
        self._pending: tuple[HalfBridgeCommand, HalfBridgeCommand, HalfBridgeCommand] | None = None
        self._commit_seq: int = 0

    def step(
        self,
        v_cmd: Dq,
        theta_e: float,
        v_dc: float,
        *,
        system_enable: bool = True,
    ) -> None:
        faults = self._fault_buffer.read_snapshot()
        if faults.gate_kill or not system_enable:
            self._fault_ground_all_phases()
            self._commit()
            return

        da, db, dc = duty01_from_dq(v_cmd, theta_e, v_dc)
        self._pending = (
            HalfBridgeCommand(duty=da),
            HalfBridgeCommand(duty=db),
            HalfBridgeCommand(duty=dc),
        )
        self._commit()

    def _fault_ground_all_phases(self) -> None:
        low = HalfBridgeCommand(duty=0.0, enable=True, brake=True, clamp_phase_low=True)
        self._pending = (low, low, low)

    def _commit(self) -> None:
        if self._pending is None:
            return
        self._commit_seq += 1
        for bridge, cmd in zip(self._bridges, self._pending):
            bridge.apply(cmd)
```

## Notes

- If you later want better linear range near saturation, replace per-phase clamp with min-max zero-sequence injection.
- If you need timer-edge simultaneity, keep Python API unchanged and implement sync trigger in bridge/PWM backend.


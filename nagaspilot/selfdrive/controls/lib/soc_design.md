# SOC (Smart Offset Controller) Implementation Design (Current)

## Overview
SOC provides small, rate-limited lateral offsets to maintain safe clearance from lane lines, road edges, and adjacent vehicles while respecting lane constraints. It reuses proven vision (modelV2) signals and injects a bounded curvature delta into the lateral controller.

## Core Concept
```
Original Trajectory:     ——————————————————————————————————>
SOC Adjusted Trajectory: ———————————————————————————————————>
                                      ↗ small offset (20–50 cm)
                                    when objects get too close
```

## Signals and Confidence
- Lane lines: `modelV2.laneLines[i].y[0]` (boundary); used only when `laneLineProbs[i] ≥ 0.5`.
- Road edges: `modelV2.roadEdges[i].y[0]` (boundary); used when `min(roadEdgeStds) ≤ 0.5`.
- Adjacent objects (vision-only): `modelV2.leadsV3` (vehicles with lateral |y| beyond side threshold).
- Road curvature: `modelV2.action.desiredCurvature` (for curve gating and comfort).

## Parameters (np_lat_)
- `np_lat_soc_speed` (km/h): enable threshold; 0 = disabled.
- `np_lat_soc_adjacent_offset` (m): min center-to-center spacing vs adjacent vehicles (default 2.8).
- `np_lat_soc_edge_offset` (m): min clearance to road edges (default 1.6).
- `np_lat_soc_lane_offset` (m): min clearance to lane lines (default 1.2).
- `np_lat_soc_tta_sec` (s): fallback default TTA threshold (behavior mapping preferred: 4/6/8 s).

## Constraints and Balancing
We build linear constraints on the offset `o` (sign: + right, − left):
- Lane lines: if left line distance < lane_offset → min_right_shift increases; if right line distance < lane_offset → max_right_shift decreases.
- Road edges: if left edge distance < edge_offset → min_right_shift increases; if right edge distance < edge_offset → max_right_shift decreases.
- Adjacent objects (vision-only): if `|y|` < adjacent_offset on left/right (via model leads with 0.5 ≤ x ≤ 80 m), adjust constraints accordingly.

Feasible interval: LB ≤ o ≤ UB.
- If 0 ∈ [LB, UB], choose 0 (no offset).
- If LB ≤ UB, choose midpoint for balanced clearance.
- If LB > UB (infeasible), choose the side with the larger deficit (LB vs −UB) and always cap by lane-space limits.

## Lane-Space Caps (including laneless)
- Prefer lane-line distances to compute max allowed offset (vehicle half-width + margin, then use 60% of remaining space). Clamp to ±0.5 m global cap.
- If lane lines are missing/unreliable, use road edges to derive caps (so laneless still has hard bounds).

## Adjacent Trigger: TTA-Gated
- TTA (time-to-approach) by side (vision-only): `TTA ≈ x_long / closing_speed` using model leads where closing_speed = max(0.5, v_ego − v_lead).
- Personality mapping sets the TTA trigger threshold:
  - aggressive → 4 s
  - standard (default) → 6 s
  - relaxed → 8 s
- Gating applies only when the offset demand is purely adjacent-object-driven (no lane/edge deficits) to prevent offsetting for far-away adjacent vehicles.

## Maintain Timer After Activation
- Maintain offset for `2 × max(TTA_left, TTA_right)` (clamped within safe bounds), ensuring we don’t return early while still alongside.

## State Machine (Anti-Chatter)
- States: IDLE, OFFSETTING, MAINTAINING, RETURNING.
- Hysteresis thresholds: activate ≥ 0.06 m; release ≤ 0.03 m; steady tolerance 0.02 m.
- Timers: min hold 1.0 s; min return 0.5 s; cooldown 0.5 s after RETURNING.
- Lane-change gating: SOC returns to center when `laneChangeState != off`.

## Curve Gating (Road Curvature)
- Compute lateral acceleration: `ay = v^2 × |desiredCurvature|` from `modelV2.action.desiredCurvature`.
- Disable SOC when `ay > 1.2 m/s²` to avoid offsetting on sharper turns (speed-scaled, platform-agnostic).

## Actuation and Limits
- Offset rate limit: 0.15 m/s for smoothness.
- Return rate limit: up to 0.25 m/s in RETURNING (tunable via `np_lat_soc_return_rate`) to recenter decisively when safe.
- Curvature conversion: `k_delta = 2 × offset / preview^2` with `preview = max(20 m, 2 s × v)`. Clamp `k_delta` to ±0.05.
- Curvature delta is added to `model_v2.action.desiredCurvature` before `clip_curvature` in `controlsd`.

- `controlsd.py`: computes `new_desired_curvature = model_v2.action.desiredCurvature + soc_k_delta` (no radar dependency).
- SOC disabled below `np_lat_soc_speed`, during lane changes, and on sharp curves (ay threshold).
- No changes to `modeld`, `radard`, or MPC internals.

- Uses proven vision signals only (modelV2); no detectors added.
- Confidence-aware constraints and caps protect against weak perception.
- TTA gating and state machine prevent reacting to far-away objects and avoid chattering.
- Laneless safe: caps fall back to edges when lines are missing.

## Defaults and Tuning
- Start with `np_lat_soc_speed = 30` km/h, standard personality (TTA=6 s).
- Keep distances at defaults: adjacent=2.8 m, edge=1.6 m, lane=1.2 m.
- Adjust personality for earlier/later triggering (aggressive/relaxed).

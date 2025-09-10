#!/usr/bin/env python3
import math
import time
from typing import Optional

from cereal import log, car
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


LaneChangeState = log.LaneChangeState


class NpSOCController:
  """
  Smart Offset Controller (SOC) without YOLO integration.

  - Uses modelV2 lane lines and road edges to maintain configurable clearances.
  - Vision-only: no radar dependency; adjacent vehicles approximated via model leads.
  - Produces a curvature delta to be added to model desired curvature.

  Parameters (np_lat_ prefix):
    - np_lat_soc_speed (km/h; 0 disables)
    - np_lat_soc_adjacent_offset (m)
    - np_lat_soc_edge_offset (m)
    - np_lat_soc_lane_offset (m)
  """

  def __init__(self) -> None:
    self.params = Params()
    self._last_param_read_t = 0.0

    # Defaults aligned with UI
    self.speed_kph_enable = 0
    self.adjacent_target = 2.8
    self.edge_target = 1.6
    self.lane_target = 1.2

    # Behavior
    self._lane_side_threshold = 1.2
    self._max_offset_m = 0.5
    self._offset_rate_mps = 0.15
    self._return_rate_mps = 0.25  # faster return for decisive recenters
    self._max_curvature = 0.05
    self._ay_thresh = 1.2   # m/s^2, lateral acceleration threshold for curve gating
    self._rel_k_frac = 0.5  # relative clamp fraction: |SOC k| <= frac * |model k|

    # Debug/telemetry
    self._dbg_state = 0
    self._dbg_offset = 0.0
    self._dbg_kdelta = 0.0
    self._dbg_ay = 0.0
    self._dbg_tta_left = 0.0
    self._dbg_tta_right = 0.0
    self._dbg_tta_thresh = 0.0

    # State
    self._current_offset_m = 0.0
    self._last_update_t = time.monotonic()
    # State machine to avoid chattering
    self._state = 0  # 0=IDLE, 1=OFFSETTING, 2=MAINTAINING, 3=RETURNING
    self._state_enter_t = self._last_update_t
    self._cooldown_until_t = 0.0
    self._maintain_until_t = 0.0
    self._active_side_sign = 0  # +1 right, -1 left, 0 none
    # Hysteresis thresholds and timers
    self._activate_thresh_m = 0.06   # need this much to engage
    self._release_thresh_m = 0.03    # below this we can release
    self._steady_tol_m = 0.02        # close enough to target to consider steady
    self._min_hold_s = 1.0           # maintain offset at least this long
    self._min_return_s = 0.5         # spend at least this long returning
    self._cooldown_s = 0.5           # cooldown after returning before re-engaging

  def _read_params(self) -> None:
    now = time.monotonic()
    if now - self._last_param_read_t < 1.0:
      return
    self._last_param_read_t = now

    try:
      v = self.params.get_int("np_lat_soc_speed")
      self.speed_kph_enable = max(0, int(v)) if v is not None else 0
    except Exception:
      self.speed_kph_enable = 0

    def _get_f(name: str, lo: float, hi: float, default: float) -> float:
      try:
        s = self.params.get(name, encoding='utf8')
        if not s:
          return default
        return max(lo, min(hi, float(s)))
      except Exception:
        return default

    self.adjacent_target = _get_f("np_lat_soc_adjacent_offset", 1.5, 5.0, 2.8)
    self.edge_target = _get_f("np_lat_soc_edge_offset", 1.0, 3.0, 1.6)
    self.lane_target = _get_f("np_lat_soc_lane_offset", 0.8, 2.0, 1.2)
    # TTA threshold (seconds) to start trigger for adjacent objects
    self.tta_thresh_s = _get_f("np_lat_soc_tta_sec", 1.0, 10.0, 4.0)
    # Optional faster return rate when RETURNING
    try:
      s = self.params.get("np_lat_soc_return_rate", encoding='utf8')
      if s:
        self._return_rate_mps = max(0.05, min(0.5, float(s)))
    except Exception:
      pass

  @staticmethod
  def _nearest_side_dist(vals, positive: bool) -> Optional[float]:
    side = [y for y in vals if (y > 0 if positive else y < 0)]
    if not side:
      return None
    return abs(min(side, key=lambda y: abs(y)))

  def _adaptive_offset_limits(self, left_dist: Optional[float], right_dist: Optional[float]) -> tuple[float, float]:
    """Compute maximum allowed offset to left/right based on lane-line distances
       and a conservative vehicle width + safety margin model.

       Returns (left_limit, right_limit) as positive magnitudes in meters.
    """
    # Vehicle half width + margin; constants tuned conservatively
    vehicle_half = 1.9 / 2.0
    margin = 0.20
    need = vehicle_half + margin
    # Use 60% of the remaining space conservatively
    left_limit = 0.0
    right_limit = 0.0
    if left_dist is not None:
      rem = max(0.0, left_dist - need)
      left_limit = rem * 0.6
    if right_dist is not None:
      rem = max(0.0, right_dist - need)
      right_limit = rem * 0.6
    # Clamp by controller-wide hard cap
    return (min(self._max_offset_m, left_limit), min(self._max_offset_m, right_limit))

  def _constraints_from_model(self, model_v2) -> tuple[float, float, float, float]:
    """Derive inequality constraints from lane lines and road edges.
       Returns (LB, UB, left_limit, right_limit) where:
         - offset o must satisfy: LB <= o <= UB
         - and o is bounded by [-left_limit, +right_limit]
    """
    LB = 0.0   # need at least this much to the right
    UB = 0.0   # need at most this much to the left (negative value)
    left_line = None
    right_line = None
    left_edge_cap = None
    right_edge_cap = None
    # Confidence thresholds
    lane_prob_min = 0.5
    edge_stds_max = 0.5
    # Lane lines
    if hasattr(model_v2, 'laneLines') and len(model_v2.laneLines) >= 1:
      y_with_prob = []
      # Try to fetch per-line probability if available; otherwise assume 1.0
      probs = []
      try:
        probs = list(getattr(model_v2, 'laneLineProbs', []))
      except Exception:
        probs = []
      for idx, line in enumerate(model_v2.laneLines):
        try:
          if len(line.y) > 0:
            p = probs[idx] if idx < len(probs) else 1.0
            y_with_prob.append((line.y[0], p))
        except Exception:
          pass
      # Model coordinate frame: x forward, y left-positive.
      # So left candidates have y > 0, right candidates have y < 0.
      left_cands = [abs(y) for (y, p) in y_with_prob if y > 0.0 and p >= lane_prob_min]
      right_cands = [abs(y) for (y, p) in y_with_prob if y < 0.0 and p >= lane_prob_min]
      if left_cands:
        left_line = min(left_cands)
      if right_cands:
        right_line = min(right_cands)
      if left_line is not None:
        deficit = self.lane_target - left_line
        if deficit > 0:
          LB = max(LB, deficit)
      if right_line is not None:
        deficit = self.lane_target - right_line
        if deficit > 0:
          UB = min(UB, -deficit)

    # Road edges
    if hasattr(model_v2, 'roadEdges') and len(model_v2.roadEdges) >= 1:
      # Global confidence on edges using stds if available
      edges_conf_ok = True
      try:
        re_stds = list(getattr(model_v2, 'roadEdgeStds', []))
        if len(re_stds) > 0:
          # use min std as best confidence proxy
          edges_conf_ok = (min(re_stds) <= edge_stds_max)
      except Exception:
        edges_conf_ok = True
      if edges_conf_ok:
        y_samples = []
        for edge in model_v2.roadEdges:
          try:
            if len(edge.y) > 0:
              y_samples.append(edge.y[0])
          except Exception:
            pass
        # Model frame: y left-positive
        left_edge = self._nearest_side_dist(y_samples, positive=True)
        right_edge = self._nearest_side_dist(y_samples, positive=False)
        if left_edge is not None:
          deficit = self.edge_target - left_edge
          if deficit > 0:
            LB = max(LB, deficit)
          left_edge_cap = left_edge
        if right_edge is not None:
          deficit = self.edge_target - right_edge
          if deficit > 0:
            UB = min(UB, -deficit)
          right_edge_cap = right_edge

    # Adaptive allowed limits from lane lines if available
    # Derive caps: prefer lane lines; if missing, use road edges for caps
    cap_left_source = left_line if left_line is not None else left_edge_cap
    cap_right_source = right_line if right_line is not None else right_edge_cap
    left_limit, right_limit = self._adaptive_offset_limits(cap_left_source, cap_right_source)
    return LB, UB, left_limit, right_limit

  def _constraints_from_radar(self, live_tracks: Optional[car.RadarData]) -> tuple[float, float]:
    """Vision-only mode: radar constraints disabled."""
    return 0.0, 0.0

  def _rate_limit(self, target: float, dt: float) -> float:
    if dt <= 0:
      return self._current_offset_m
    # Use a faster rate when in RETURNING state
    rate = self._return_rate_mps if self._state == 3 else self._offset_rate_mps
    max_step = rate * dt
    delta = target - self._current_offset_m
    if abs(delta) <= max_step:
      self._current_offset_m = target
    else:
      self._current_offset_m += math.copysign(max_step, delta)
    if self._current_offset_m > self._max_offset_m:
      self._current_offset_m = self._max_offset_m
    elif self._current_offset_m < -self._max_offset_m:
      self._current_offset_m = -self._max_offset_m
    return self._current_offset_m

  def compute_curvature_delta(self, sm, lat_active: bool) -> float:
    self._read_params()

    CS = sm['carState']
    v_ego = CS.vEgo
    v_kph = v_ego * 3.6

    if not lat_active:
      self._current_offset_m = 0.0
      self._last_update_t = time.monotonic()
      self._enter_state(0)
      return 0.0

    if self.speed_kph_enable <= 0 or v_kph < float(self.speed_kph_enable):
      self._current_offset_m = 0.0
      self._last_update_t = time.monotonic()
      self._enter_state(0)
      return 0.0

    # Curve gating: use model road curvature to disable SOC on sharp turns
    try:
      k_model = abs(sm['modelV2'].action.desiredCurvature)
      ay = (v_ego * v_ego) * k_model
      if ay > self._ay_thresh:
        self._enter_state(3)
        self._current_offset_m = 0.0
        return 0.0
    except Exception:
      pass

    model_v2 = sm['modelV2']
    try:
      if model_v2.meta.laneChangeState != LaneChangeState.off:
        self._current_offset_m = 0.0
        self._last_update_t = time.monotonic()
        self._enter_state(3)  # RETURNING while lane change
        return 0.0
    except Exception:
      pass

    # ------------------------------------------------------------------
    # Build offset constraints from model (lanes/edges) only (vision-only).
    # Sign conventions:
    #   - Model frame: y > 0 is LEFT, y < 0 is RIGHT.
    #   - Offset o (SOC): POSITIVE = SHIFT LEFT, NEGATIVE = SHIFT RIGHT.
    #
    # We solve for o in the left-positive frame:
    #   min_left_shift <= o <= max_left_shift
    #   - min_left_shift: minimum REQUIRED left shift from RIGHT-side constraints
    #   - max_left_shift: maximum ALLOWED left shift from LEFT-side constraints
    # Physical lane-space caps: -right_limit <= o <= left_limit
    #   - left_limit/right_limit are positive magnitudes derived from lanes/edges.
    # ------------------------------------------------------------------
    LB_model, UB_model, left_limit, right_limit = self._constraints_from_model(model_v2)

    # Convert to the model's left-positive offset frame for clarity:
    # right-frame constraints [LB_model, UB_model] become left-frame [-UB_model, -LB_model]
    min_left_shift_from_model = -UB_model
    max_left_shift_from_model = -LB_model
    # Apply physical caps in left-positive frame: -right_limit <= o <= left_limit
    min_left_shift = max(min_left_shift_from_model, -right_limit)
    max_left_shift = min(max_left_shift_from_model, left_limit)

    # Pick a target within (or nearest to) the feasible interval with clear priority:
    #   1) Prefer 0 if feasible
    #   2) If feasible, choose midpoint for balance
    #   3) If infeasible, choose side with larger deficit
    target_offset = self._choose_target_offset(min_left_shift, max_left_shift)

    # TTA gating to prevent offset for far adjacent objects (radar/model-based)
    # Only gate when there is no lane/edge deficit (pure adjacent-object case)
    if target_offset != 0.0 and LB_model == 0.0 and UB_model == 0.0:
      tta_left, tta_right, have_tta = self._estimate_tta_lr(sm)
      self._dbg_tta_left, self._dbg_tta_right = tta_left, tta_right
      if have_tta:
        tta_thresh = self._get_tta_threshold(sm)
        self._dbg_tta_thresh = tta_thresh
        if target_offset > 0.0 and tta_right > tta_thresh:
          target_offset = 0.0
        elif target_offset < 0.0 and tta_left > tta_thresh:
          target_offset = 0.0
    else:
      # No adjacent-only case; clear TTA telemetry to zeros
      self._dbg_tta_left = 0.0
      self._dbg_tta_right = 0.0
      self._dbg_tta_thresh = 0.0

    # State machine to avoid continuous triggering
    target_offset = self._apply_state_machine(target_offset, sm)

    now = time.monotonic()
    dt = now - self._last_update_t
    self._last_update_t = now
    current_offset = self._rate_limit(target_offset, dt)
    self._dbg_offset = current_offset
    self._dbg_state = self._state

    preview_m = max(20.0, v_ego * 2.0)
    if preview_m <= 0.0:
      return 0.0
    k_delta = 2.0 * current_offset / (preview_m ** 2)
    if k_delta > self._max_curvature:
      k_delta = self._max_curvature
    elif k_delta < -self._max_curvature:
      k_delta = -self._max_curvature
    # Curvature-relative clamp to never dominate on curves
    try:
      model_k = abs(sm['modelV2'].action.desiredCurvature)
    except Exception:
      model_k = 0.0
    if model_k > 1e-4:
      rel_cap = self._rel_k_frac * model_k
      if k_delta > rel_cap:
        k_delta = rel_cap
      elif k_delta < -rel_cap:
        k_delta = -rel_cap

    # Store telemetry
    self._dbg_kdelta = k_delta
    try:
      self._dbg_ay = (v_ego * v_ego) * abs(sm['modelV2'].action.desiredCurvature)
    except Exception:
      self._dbg_ay = 0.0
    return float(k_delta)

  def _choose_target_offset(self, min_left_shift: float, max_left_shift: float) -> float:
    """Choose a target offset in the model's left-positive frame: [min_left_shift, max_left_shift].

    - If 0 is feasible (min_left_shift <= 0 <= max_left_shift), prefer 0.
    - If feasible (min_left_shift <= max_left_shift), choose midpoint for balance.
    - If infeasible (min_left_shift > max_left_shift), choose the side with larger deficit.
    """
    if min_left_shift <= 0.0 <= max_left_shift:
      return 0.0
    if min_left_shift <= max_left_shift:
      return (min_left_shift + max_left_shift) / 2.0
    # infeasible: pick side with larger deficit (compare min_left vs -max_left)
    return min_left_shift if min_left_shift >= -max_left_shift else max_left_shift

  def get_telemetry(self):
    return {
      'state': int(self._dbg_state),
      'offset': float(self._dbg_offset),
      'k_delta': float(self._dbg_kdelta),
      'ay': float(self._dbg_ay),
      'tta_left': float(self._dbg_tta_left),
      'tta_right': float(self._dbg_tta_right),
      'tta_thresh': float(self._dbg_tta_thresh),
      'active': bool(self._dbg_state in (1, 2) and abs(self._dbg_offset) > 1e-3),
    }

  def _enter_state(self, new_state: int):
    if new_state != self._state:
      self._state = new_state
      self._state_enter_t = time.monotonic()

  def _apply_state_machine(self, demanded_offset: float, sm) -> float:
    """Return the commanded target offset after applying hysteresis and state gating."""
    now = time.monotonic()
    need = abs(demanded_offset)
    sign = 1.0 if demanded_offset >= 0.0 else -1.0

    # Cooldown prevents immediate re-trigger after returning
    if now < self._cooldown_until_t:
      demanded_offset = 0.0
      need = 0.0

    if self._state == 0:  # IDLE
      if need >= self._activate_thresh_m and now >= self._cooldown_until_t:
        self._enter_state(1)  # OFFSETTING
        self._active_side_sign = 1 if demanded_offset >= 0.0 else -1
        # Compute smart maintain time: taking-over time * 2
        self._maintain_until_t = now + self._compute_overtake_delay(sm, self._active_side_sign)
      else:
        return 0.0

    if self._state == 1:  # OFFSETTING
      # Move toward demanded offset
      if need < self._release_thresh_m:
        if now >= self._maintain_until_t:
          self._enter_state(3)  # RETURNING
        else:
          # Hold minimal offset until maintain time elapses
          return self._active_side_sign * max(self._release_thresh_m, need)
      elif abs(self._current_offset_m - demanded_offset) <= self._steady_tol_m:
        self._enter_state(2)  # MAINTAINING
      return demanded_offset

    if self._state == 2:  # MAINTAINING
      held_s = now - self._state_enter_t
      if need < self._release_thresh_m:
        if now >= self._maintain_until_t and held_s >= self._min_hold_s:
          self._enter_state(3)  # RETURNING
          return 0.0
        else:
          # Continue to maintain until maintain time elapses
          return self._active_side_sign * max(self._release_thresh_m, need)
      # Keep following demanded (may vary slightly as constraints shift)
      return demanded_offset

    if self._state == 3:  # RETURNING
      # Command return to center
      returning_s = now - self._state_enter_t
      if abs(self._current_offset_m) <= self._release_thresh_m and returning_s >= self._min_return_s:
        self._enter_state(0)
        self._cooldown_until_t = now + self._cooldown_s
        return 0.0
      return 0.0

    # Fallback
    return 0.0

  def _compute_overtake_delay(self, sm, side_sign: int) -> float:
    """Estimate maintain time = 2 * max(TTA_left, TTA_right) (vision-only).
       TTA ≈ x_long / max(0.5, v_ego - v_lead) using model leads.
    """
    try:
      min_delay = 1.0
      max_delay = 8.0
      tta_left, tta_right, _ = self._estimate_tta_lr(sm)

      tta_max = max(tta_left, tta_right)
      if tta_max <= 0.0:
        # fallback: modest default when no estimation available
        tta_max = 2.0

      delay = 2.0 * tta_max
      return max(min_delay, min(max_delay, delay))
    except Exception:
      return 2.0

  def _estimate_tta_lr(self, sm) -> tuple[float, float, bool]:
    """Estimate left/right time-to-approach using model leads only (vision-only).
       Returns (tta_left, tta_right, have_estimation)."""
    try:
      v_ego = sm['carState'].vEgo
      min_closing = 0.5
      # Use model leads (leadsV3) as proxy for adjacent when lateral |y| exceeds side threshold
      tta_left = 0.0
      tta_right = 0.0
      model_v2 = sm['modelV2']
      if hasattr(model_v2, 'leadsV3') and model_v2.leadsV3:
        for lead in model_v2.leadsV3:
          try:
            if len(lead.x) == 0 or len(lead.v) == 0 or len(lead.y) == 0:
              continue
            x = lead.x[0]
            v_lead = lead.v[0]
            y = lead.y[0]
            if not (0.5 <= x <= 80.0):
              continue
            closing = max(min_closing, v_ego - v_lead)
            tta = x / closing
            # Model y: left positive; treat |y| beyond threshold as adjacent on that side
            if y > self._lane_side_threshold:
              tta_left = max(tta_left, tta)
            elif y < -self._lane_side_threshold:
              tta_right = max(tta_right, tta)
          except Exception:
            continue
      return tta_left, tta_right, (tta_left > 0.0 or tta_right > 0.0)
    except Exception:
      return 0.0, 0.0, False

  def _get_tta_threshold(self, sm) -> float:
    """Map longitudinal behavior to TTA threshold.
       aggressive -> 4s, standard/default -> 6s, relaxed -> 8s.
       Falls back to param if personality not available.
    """
    try:
      pers = sm['selfdriveState'].personality
      # Prefer named enum if available
      try:
        if pers == log.LongitudinalPersonality.aggressive:
          return 4.0
        if pers == log.LongitudinalPersonality.relaxed:
          return 8.0
        return 6.0
      except Exception:
        # Fallback to numeric raw value if present
        val = getattr(pers, 'raw', None)
        if val is None:
          return self.tta_thresh_s
        if val == 1:  # commonly aggressive=1
          return 4.0
        if val == 2:  # commonly relaxed=2
          return 8.0
        return 6.0
    except Exception:
      return self.tta_thresh_s

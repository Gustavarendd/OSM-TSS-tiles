"""Directed lane acquisition & tracking utilities.

Implements the state machine logic described in the user's specification:

Lane polyline P = [p0, p1, ..., pN] (lat, lon tuples)
Legal travel direction: from p0 toward pN only.

States:
  NOT_ON_LANE -> attempt acquisition near p0 only (distance + optional heading gate).
  ON_LANE     -> progress forward only; along-track distance must be non-decreasing.
  COMPLETED   -> passed beyond final vertex (optional terminal state).

Key guarantees:
* Never acquire mid‑lane (snap only permitted near first coordinate p0).
* After acquisition, along-track distance monotonically increases (within tolerance).
* Reverse motion, excessive lateral (cross-track) deviation, or large sustained
  heading deviation cause loss of lock (reverting to NOT_ON_LANE). Reacquisition is
  again only possible at p0.

Public entry points:
  DirectedLane(coords)                -> preprocess bearings & cumulative distances.
  LaneTracker(lane, thresholds...)    -> encapsulates thresholds and tracking state.
  tracker.try_acquire(pos, heading)   -> attempt initial lock.
  tracker.update(pos, heading)        -> advance or drop lock.

The geometry helpers use pyproj.Geod (WGS84) for accurate segment bearings & distances,
and great‑circle cross-track / along-track separation formulae.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple, Literal
import math
from pyproj import Geod

GEOD = Geod(ellps="WGS84")
EARTH_R_M = 6371000.0  # mean earth radius (meters) for spherical xtrack formulas

LatLon = Tuple[float, float]  # (lat, lon)


def _bearing(a: LatLon, b: LatLon) -> float:
    """Return forward azimuth degrees 0-360 from a to b (lat,lon)."""
    az, _, _ = GEOD.inv(a[1], a[0], b[1], b[0])
    if az < 0:
        az += 360.0
    if az >= 360.0:
        az -= 360.0
    return az


def _distance_m(a: LatLon, b: LatLon) -> float:
    _, _, d = GEOD.inv(a[1], a[0], b[1], b[0])
    return float(d)


def _angle_diff_deg(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return d if d <= 180.0 else 360.0 - d


def _project_point_onto_segment(p: LatLon, a: LatLon, b: LatLon) -> Tuple[float, float, bool]:
    """Great-circle projection of point p onto segment a->b.

    Returns (along_track_m, cross_track_m, on_segment_bool).

    along_track_m: distance from a along the great-circle path toward b to the
                   closest projected point (can be <0 or >segment_len if beyond).
    cross_track_m: signed lateral distance (+ left of course, - right) using the
                   spherical cross-track formula (approx; acceptable for navigation scale).
    on_segment_bool: True if projection lies between endpoints (0 <= along <= seg_len).
    """
    if a == b:
        return 0.0, _distance_m(p, a), False
    az_ab, _, dist_ab = GEOD.inv(a[1], a[0], b[1], b[0])
    az_ap, _, dist_ap = GEOD.inv(a[1], a[0], p[1], p[0])
    # Convert to radians for spherical formulas
    δ13 = dist_ap / EARTH_R_M
    θ13 = math.radians(az_ap)
    θ12 = math.radians(az_ab)
    # Cross-track angular distance
    δxt = math.asin(max(-1.0, min(1.0, math.sin(δ13) * math.sin(θ13 - θ12))))
    cross_track = δxt * EARTH_R_M
    # Along-track angular distance
    δat = math.acos(max(-1.0, min(1.0, math.cos(δ13) / (math.cos(δxt) if abs(math.cos(δxt)) > 1e-12 else 1.0))))
    along = δat * EARTH_R_M
    on_seg = 0.0 <= along <= dist_ab
    return along, cross_track, on_seg


@dataclass
class DirectedLane:
    coords: Sequence[LatLon]
    bearings: List[float]
    cumulative: List[float]
    length_m: float

    @classmethod
    def from_coords(cls, coords: Sequence[LatLon]) -> "DirectedLane":
        if len(coords) < 2:
            raise ValueError("Lane must have at least 2 coordinates")
        cumulative: List[float] = [0.0]
        bearings: List[float] = []
        total = 0.0
        for a, b in zip(coords[:-1], coords[1:]):
            bearings.append(_bearing(a, b))
            d = _distance_m(a, b)
            total += d
            cumulative.append(total)
        return cls(coords=list(coords), bearings=bearings, cumulative=cumulative, length_m=total)


@dataclass
class LaneState:
    status: Literal["NOT_ON_LANE", "ON_LANE", "COMPLETED"]
    index: int = 0               # current segment start index ( between index and index+1 )
    along_track_m: float = 0.0   # distance from lane start p0 along the lane

    def as_dict(self):  # convenience for JSON serialization
        return {"status": self.status, "index": self.index, "along_track_m": self.along_track_m}


class LaneTracker:
    """Stateful tracker enforcing forward-only traversal of a directed lane."""

    def __init__(
        self,
        lane: DirectedLane,
        entry_distance_threshold_m: float = 200.0,
        heading_tolerance_deg: float = 30.0,
        cross_track_threshold_m: float = 150.0,
        backward_tolerance_m: float = 15.0,
        enroute_heading_tolerance_deg: float = 75.0,
        max_heading_deviation_count: int = 3,
    ):
        self.lane = lane
        self.entry_distance_threshold_m = entry_distance_threshold_m
        self.heading_tolerance_deg = heading_tolerance_deg
        self.cross_track_threshold_m = cross_track_threshold_m
        self.backward_tolerance_m = backward_tolerance_m
        self.enroute_heading_tolerance_deg = enroute_heading_tolerance_deg
        self.max_heading_deviation_count = max_heading_deviation_count
        self._state = LaneState(status="NOT_ON_LANE")
        self._heading_dev_streak = 0

    # ---------------- Acquisition -----------------
    def try_acquire(self, vessel: LatLon, heading_deg: Optional[float]) -> LaneState:
        """Attempt to acquire lane (only permitted near p0, heading gate optional)."""
        if self._state.status == "ON_LANE":
            return self._state
        p0 = self.lane.coords[0]
        dist = _distance_m(vessel, p0)
        if dist > self.entry_distance_threshold_m:
            return self._state  # remain NOT_ON_LANE
        if heading_deg is not None:
            lane_bearing0 = self.lane.bearings[0]
            if _angle_diff_deg(heading_deg, lane_bearing0) > self.heading_tolerance_deg:
                return self._state
        # Acquire
        self._state = LaneState(status="ON_LANE", index=0, along_track_m=0.0)
        self._heading_dev_streak = 0
        return self._state

    # ---------------- Tracking --------------------
    def update(self, vessel: LatLon, heading_deg: Optional[float]) -> LaneState:
        """Update tracking given current vessel position (and optional heading).

        Returns updated LaneState; may revert to NOT_ON_LANE if lock lost.
        """
        if self._state.status == "NOT_ON_LANE":
            return self._state
        if self._state.status == "COMPLETED":
            return self._state
        prev_along = self._state.along_track_m
        k = self._state.index
        coords = self.lane.coords
        progressed = False
        # Try projecting onto current & forward segments until on one or past end
        while k < len(coords) - 1:
            a = coords[k]
            b = coords[k + 1]
            along_rel, cross, on_seg = _project_point_onto_segment(vessel, a, b)
            seg_len = _distance_m(a, b)
            if not on_seg and along_rel > seg_len:  # passed beyond this segment
                k += 1
                continue
            if on_seg:
                # Compute absolute along-track from lane start
                along_abs = self.lane.cumulative[k] + along_rel
                # Lateral deviation gate
                if abs(cross) > self.cross_track_threshold_m:
                    return self._drop_lock("cross_track_exceeded")
                # Backward check
                if along_abs + self.backward_tolerance_m < prev_along:
                    return self._drop_lock("reverse_motion")
                # Update
                self._state.index = k
                self._state.along_track_m = max(along_abs, prev_along)  # monotonic enforce
                progressed = True
                # Heading conformity (optional)
                if heading_deg is not None:
                    seg_bearing = self.lane.bearings[min(k, len(self.lane.bearings) - 1)]
                    if _angle_diff_deg(heading_deg, seg_bearing) > self.enroute_heading_tolerance_deg:
                        self._heading_dev_streak += 1
                        if self._heading_dev_streak >= self.max_heading_deviation_count:
                            return self._drop_lock("heading_deviation")
                    else:
                        self._heading_dev_streak = 0
                break
            else:
                # Off to the side of current segment but not beyond its end -> test cross-track vs threshold
                if abs(cross) > self.cross_track_threshold_m:
                    return self._drop_lock("cross_track_exceeded")
                break  # keep previous state
        # Past final segment?
        if k >= len(coords) - 1:
            # If vessel is beyond last vertex by forward motion we can mark completed.
            last = coords[-1]
            if _distance_m(vessel, last) < self.cross_track_threshold_m * 2:
                self._state.status = "COMPLETED"
            else:
                # Too far away laterally -> drop
                return self._drop_lock("beyond_end")
        elif not progressed:
            # If we failed to project onto any segment meaningfully, drop if drifted large distance
            if _distance_m(vessel, coords[self._state.index]) > self.cross_track_threshold_m * 2:
                return self._drop_lock("projection_failed")
        return self._state

    # ---------------- Helpers ---------------------
    def _drop_lock(self, reason: str) -> LaneState:
        self._state = LaneState(status="NOT_ON_LANE")
        self._heading_dev_streak = 0
        return self._state

    # Convenience wrappers -------------------------------------------------
    @property
    def state(self) -> LaneState:
        return self._state


# ---------------- Example usage (manual / debug) -----------------
if __name__ == "__main__":  # pragma: no cover - manual quick test
    lane_coords = [
        (35.9039255, -5.7496667),
        (35.9147556, -5.6076037),
        (35.9317689, -5.5436426),
    ]
    lane = DirectedLane.from_coords(lane_coords)
    tracker = LaneTracker(lane)
    # Simulated vessel positions roughly moving along lane direction
    samples = [
        (35.90390, -5.74960),  # near start
        (35.90700, -5.73000),
        (35.91000, -5.70000),
        (35.91470, -5.60760),  # near mid
        (35.92000, -5.58000),
        (35.93170, -5.54360),  # near end
    ]
    headings = []  # leave headings None for simplicity here
    print("Attempt acquire:")
    tracker.try_acquire(samples[0], heading_deg=None)
    print(tracker.state)
    for p in samples[1:]:
        tracker.update(p, heading_deg=None)
        print(tracker.state)

__all__ = [
    "DirectedLane",
    "LaneTracker",
    "LaneState",
]

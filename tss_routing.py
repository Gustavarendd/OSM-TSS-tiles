"""Enhanced TSS routing utilities with strict direction handling."""

import math
from typing import Tuple, Optional, List, Dict
from .geojson_backend import GeoGraph, geodesic_meters, _bearing

def calculate_lane_angle(start: Tuple[float, float], end: Tuple[float, float]) -> float:
    """Calculate the angle of a lane segment in degrees."""
    return _bearing(start, end)

def get_approach_angle(current: Tuple[float, float], target: Tuple[float, float]) -> float:
    """Calculate the approach angle to a target point."""
    return _bearing(current, target)

def angle_difference(angle1: float, angle2: float) -> float:
    """Calculate the smallest angle between two angles in degrees."""
    diff = abs(angle1 - angle2) % 360
    return min(diff, 360 - diff)

def is_compatible_direction(lane_angle: float, approach_angle: float, tolerance_deg: float = 45) -> bool:
    """Check if the lane direction is compatible with the approach angle."""
    return angle_difference(lane_angle, approach_angle) <= tolerance_deg

def find_best_lane_node(
    pt_latlon: Tuple[float, float],
    toward_latlon: Tuple[float, float],
    gg: GeoGraph,
    max_snap_m: Optional[float] = None,
    max_angle_diff: float = 45.0
) -> Optional[Tuple[float, float]]:
    """Find the best lane node considering both distance and direction.
    
    Args:
        pt_latlon: Current point (lat, lon)
        toward_latlon: Target point (lat, lon)
        gg: Graph with lane information
        max_snap_m: Maximum snapping distance
        max_angle_diff: Maximum allowed angle difference between desired direction and lane direction
    
    Returns:
        Best matching node or None if no suitable node found
    """
    desired_angle = get_approach_angle(pt_latlon, toward_latlon)
    
    # Get all nodes within snap distance
    candidates = []
    for node in gg.G.nodes():
        dist = geodesic_meters((pt_latlon[1], pt_latlon[0]), (node[1], node[0]))
        if max_snap_m is not None and dist > max_snap_m:
            continue
            
        # Check outgoing edges for direction
        for _, next_node, data in gg.G.edges(node, data=True):
            lane_angle = data.get('direction')
            if lane_angle is None:
                lane_angle = calculate_lane_angle(node, next_node)
                
            angle_diff = angle_difference(desired_angle, lane_angle)
            if angle_diff <= max_angle_diff:
                # Score based on both distance and angle match
                score = dist + (angle_diff * 100)  # Weight angle differences more heavily
                candidates.append((node, score))
                break  # Found a good direction from this node
    
    if not candidates:
        return None
        
    # Return the node with the best score
    return min(candidates, key=lambda x: x[1])[0]

def validate_tss_path(
    path: List[Tuple[float, float]],
    gg: GeoGraph,
    max_angle_change: float = 60.0
) -> bool:
    """Validate that a path follows TSS lanes and directions properly."""
    if len(path) < 2:
        return True
        
    for i in range(len(path) - 1):
        current = path[i]
        next_pt = path[i + 1]
        
        # Check if edge exists in graph
        if not gg.G.has_edge(current, next_pt):
            return False
            
        # Check direction consistency
        if i > 0:
            prev_angle = calculate_lane_angle(path[i-1], current)
            curr_angle = calculate_lane_angle(current, next_pt)
            if angle_difference(prev_angle, curr_angle) > max_angle_change:
                return False
                
    return True

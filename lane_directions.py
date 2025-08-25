"""Load and use directional information from separation_lanes_with_direction.geojson."""

import json
from typing import Dict, Tuple, List

def load_lane_directions(path: str) -> Dict[Tuple[float, float], float]:
    """Load lane directions from the separation_lanes_with_direction.geojson file.
    
    Returns:
        Dict mapping (lat, lon) coordinates to their direction in degrees
    """
    directions = {}
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    
    for feature in data.get("features", []):
        if feature.get("geometry", {}).get("type") == "LineString":
            coords = feature.get("geometry", {}).get("coordinates", [])
            direction = feature.get("properties", {}).get("direction", 0)  # Direction in degrees
            
            # Convert each coordinate to (lat, lon) and store direction
            for coord in coords:
                if len(coord) >= 2:
                    lat, lon = coord[1], coord[0]  # GeoJSON uses [lon, lat]
                    directions[(lat, lon)] = direction
    
    return directions

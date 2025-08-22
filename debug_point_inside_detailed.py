#!/usr/bin/env python3
"""Debug version of the point inside function."""

import json
from geojson_backend import _parse_seamark_type
from shapely.geometry import Point, Polygon

def debug_point_inside_separation_zone(
    geojson_path: str,
    point_latlon: tuple,
    buffer_meters: float = 10.0
) -> bool:
    """Debug version of the point inside separation zone function."""
    print(f"Checking point {point_latlon} in file {geojson_path}")
    
    try:
        with open(geojson_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Failed to load file: {e}")
        return False
    
    # Create a Point from the coordinates (lon, lat format for Shapely)
    point = Point(point_latlon[1], point_latlon[0])
    print(f"Created Shapely point: {point}")
    
    # Check each feature in the GeoJSON
    feature_count = 0
    sep_zone_count = 0
    
    for feat in data.get("features", []):
        feature_count += 1
        props = feat.get("properties", {})
        stype = _parse_seamark_type(props)
        
        print(f"Feature {feature_count}: seamark_type = '{stype}'")
        
        # Only check separation zones
        if stype != "separation_zone":
            continue
            
        sep_zone_count += 1
        print(f"Processing separation zone {sep_zone_count}")
        
        geom = feat.get("geometry", {})
        geom_type = geom.get("type")
        print(f"Geometry type: {geom_type}")
        
        if geom_type == "MultiPolygon":
            # Handle MultiPolygon geometry
            coordinates = geom.get("coordinates", [])
            print(f"MultiPolygon has {len(coordinates)} polygons")
            
            for i, polygon_coords in enumerate(coordinates):
                print(f"  Processing polygon {i+1}")
                try:
                    # Each polygon in MultiPolygon: first ring is exterior, others are holes
                    if polygon_coords and len(polygon_coords[0]) >= 3:
                        exterior_ring = polygon_coords[0]
                        holes = polygon_coords[1:] if len(polygon_coords) > 1 else []
                        print(f"    Exterior ring has {len(exterior_ring)} points")
                        print(f"    Has {len(holes)} holes")
                        
                        polygon = Polygon(exterior_ring, holes)
                        print(f"    Created polygon: {polygon}")
                        print(f"    Polygon is valid: {polygon.is_valid}")
                        
                        # Check if point is inside or very close to this separation zone polygon
                        if buffer_meters > 0:
                            # Convert meters to approximate degrees (rough approximation)
                            buffer_deg = buffer_meters / 111000.0  # approximately 111km per degree
                            print(f"    Buffer: {buffer_meters}m = {buffer_deg} degrees")
                            buffered_polygon = polygon.buffer(buffer_deg)
                            is_within = point.within(buffered_polygon)
                            print(f"    Point within buffered polygon: {is_within}")
                            if is_within:
                                print("    MATCH! Returning True")
                                return True
                        else:
                            is_within = point.within(polygon)
                            print(f"    Point within polygon: {is_within}")
                            if is_within:
                                print("    MATCH! Returning True")
                                return True
                except Exception as e:
                    print(f"    Error processing polygon: {e}")
                    continue
        elif geom_type == "Polygon":
            # Handle single Polygon geometry
            coordinates = geom.get("coordinates", [])
            print(f"Single Polygon")
            try:
                if coordinates and len(coordinates[0]) >= 3:
                    exterior_ring = coordinates[0]
                    holes = coordinates[1:] if len(coordinates) > 1 else []
                    print(f"  Exterior ring has {len(exterior_ring)} points")
                    print(f"  Has {len(holes)} holes")
                    
                    polygon = Polygon(exterior_ring, holes)
                    print(f"  Created polygon: {polygon}")
                    print(f"  Polygon is valid: {polygon.is_valid}")
                    
                    # Check if point is inside or very close to this separation zone polygon
                    if buffer_meters > 0:
                        buffer_deg = buffer_meters / 111000.0
                        print(f"  Buffer: {buffer_meters}m = {buffer_deg} degrees")
                        buffered_polygon = polygon.buffer(buffer_deg)
                        is_within = point.within(buffered_polygon)
                        print(f"  Point within buffered polygon: {is_within}")
                        if is_within:
                            print("  MATCH! Returning True")
                            return True
                    else:
                        is_within = point.within(polygon)
                        print(f"  Point within polygon: {is_within}")
                        if is_within:
                            print("  MATCH! Returning True")
                            return True
            except Exception as e:
                print(f"  Error processing polygon: {e}")
                continue
    
    print(f"Processed {feature_count} features, {sep_zone_count} separation zones")
    print("No matches found, returning False")
    return False

# Test with the known centroid
if __name__ == "__main__":
    poly_file = './seamarks-tss-polys.json'
    test_point = (1.1223944687258358, 103.64102759820038)  # lat, lon
    
    result = debug_point_inside_separation_zone(poly_file, test_point, 0.0)
    print(f"\nFinal result: {result}")

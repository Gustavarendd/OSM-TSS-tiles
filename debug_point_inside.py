#!/usr/bin/env python3
"""Debug script to test point inside separation zone function directly."""

import json
from geojson_backend import _point_inside_separation_zone, _parse_seamark_type
from shapely.geometry import Polygon, Point

def debug_point_inside_function():
    """Debug the _point_inside_separation_zone function step by step."""
    
    poly_file = './seamarks-tss-polys.json'
    
    try:
        with open(poly_file, 'r') as f:
            data = json.load(f)
            
        # Find first separation zone
        first_sep_zone = None
        for feat in data.get('features', []):
            props = feat.get('properties', {})
            stype = _parse_seamark_type(props)
            
            if stype == 'separation_zone':
                first_sep_zone = feat
                break
                
        if first_sep_zone:
            print("Found first separation zone")
            geom = first_sep_zone.get('geometry', {})
            coords = geom.get('coordinates', [])
            
            if coords:
                first_poly = coords[0]
                if first_poly:
                    first_ring = first_poly[0]
                    polygon = Polygon(first_ring)
                    centroid = polygon.centroid
                    
                    # Test point
                    test_point = (centroid.y, centroid.x)  # lat, lon
                    print(f"Testing point: {test_point}")
                    
                    # Manual check with our function's logic
                    print("\nManual checking logic:")
                    point_shapely = Point(test_point[1], test_point[0])  # lon, lat for Shapely
                    print(f"Shapely point: {point_shapely}")
                    print(f"Polygon: {polygon}")
                    print(f"Point within polygon (Shapely): {point_shapely.within(polygon)}")
                    
                    # Now test our function
                    print("\nTesting our function:")
                    result = _point_inside_separation_zone(poly_file, test_point)
                    print(f"Function result: {result}")
                    
                    # Let's also test with buffer
                    result_with_buffer = _point_inside_separation_zone(poly_file, test_point, 100.0)
                    print(f"Function result with 100m buffer: {result_with_buffer}")
        else:
            print("No separation zone found")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_point_inside_function()

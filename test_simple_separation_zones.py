#!/usr/bin/env python3
"""Test with separation zones that don't have holes."""

import json
from geojson_backend import _point_inside_separation_zone, _parse_seamark_type
from shapely.geometry import Polygon, Point

def test_with_simple_separation_zone():
    """Test with a separation zone that doesn't have holes."""
    
    poly_file = './seamarks-tss-polys.json'
    
    try:
        with open(poly_file, 'r') as f:
            data = json.load(f)
            
        # Find a separation zone without holes
        for feat in data.get('features', []):
            props = feat.get('properties', {})
            stype = _parse_seamark_type(props)
            
            if stype == 'separation_zone':
                geom = feat.get('geometry', {})
                if geom.get('type') == 'MultiPolygon':
                    coords = geom.get('coordinates', [])
                    if coords:
                        first_poly = coords[0]
                        if first_poly and len(first_poly) == 1:  # No holes (only exterior ring)
                            first_ring = first_poly[0]
                            print(f"Found separation zone without holes: {len(first_ring)} points")
                            
                            polygon = Polygon(first_ring)
                            print(f"Polygon bounds: {polygon.bounds}")
                            
                            # Calculate centroid
                            centroid = polygon.centroid
                            test_point = (centroid.y, centroid.x)  # lat, lon
                            print(f"Testing centroid: {test_point}")
                            
                            # Verify with Shapely
                            point_shapely = Point(test_point[1], test_point[0])
                            is_within = point_shapely.within(polygon)
                            print(f"Shapely test: {is_within}")
                            
                            # Test our function
                            result = _point_inside_separation_zone(poly_file, test_point)
                            print(f"Our function result: {result}")
                            
                            if result:
                                print("SUCCESS! Found a working test case")
                                return test_point
                            else:
                                print("Function still returned False")
                                
        print("No simple separation zones found")
        return None
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_manually_picked_point():
    """Test with a manually picked point in a known separation zone."""
    
    # Let's try a point in the large separation zone (feature 6) near France
    # Bounds: (-5.705 48.585, -5.245 48.81)
    # Let's pick a point roughly in the middle
    test_point = (48.7, -5.5)  # lat, lon
    
    poly_file = './seamarks-tss-polys.json'
    
    print(f"Testing manually picked point: {test_point}")
    
    # Test our function
    result = _point_inside_separation_zone(poly_file, test_point)
    print(f"Our function result: {result}")
    
    return result

if __name__ == "__main__":
    print("Testing with simple separation zones...")
    test_with_simple_separation_zone()
    
    print("\nTesting with manually picked point...")
    test_manually_picked_point()

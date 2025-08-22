#!/usr/bin/env python3
"""Test script to verify separation zone avoidance functionality."""

import json
from geojson_backend import (
    tss_correct_route_geojson, 
    _point_inside_separation_zone,
    _line_crosses_separation_zone_polygons
)

def test_separation_zone_functions():
    """Test separation zone detection functions."""
    
    poly_file = './seamarks-tss-polys.json'
    
    # Test with a known point inside a separation zone (from our previous tests)
    test_point_inside = (48.65315519836674, -5.470755158704827)  # lat, lon - centroid of large zone
    test_point_outside = (48.0, -5.0)  # lat, lon - definitely outside
    
    print(f"Testing point inside separation zone: {test_point_inside}")
    result_inside = _point_inside_separation_zone(poly_file, test_point_inside)
    print(f"Point inside separation zone: {result_inside}")
    
    print(f"Testing point outside separation zone: {test_point_outside}")
    result_outside = _point_inside_separation_zone(poly_file, test_point_outside)
    print(f"Point outside separation zone: {result_outside}")
    
    # Test line crossing - from outside to inside the separation zone
    start_point = (48.5, -5.0)  # Outside the zone
    end_point = test_point_inside  # Inside the zone
    
    print(f"Testing line from {start_point} to {end_point}")
    crosses = _line_crosses_separation_zone_polygons(poly_file, start_point, end_point)
    print(f"Line crosses separation zone: {crosses}")
    
    # Test line that should not cross - both points outside
    start_safe = (48.0, -5.0)
    end_safe = (48.1, -5.1)
    crosses_safe = _line_crosses_separation_zone_polygons(poly_file, start_safe, end_safe)
    print(f"Safe line crosses separation zone: {crosses_safe}")
    
    # Validation
    if result_inside and not result_outside and crosses and not crosses_safe:
        print("✓ All separation zone detection tests PASSED")
    else:
        print("✗ Some separation zone detection tests FAILED")
        print(f"  Inside test: {result_inside} (should be True)")
        print(f"  Outside test: {result_outside} (should be False)")
        print(f"  Line crossing test: {crosses} (should be True)")
        print(f"  Safe line test: {crosses_safe} (should be False)")

def test_routing_with_separation_zones():
    """Test routing that should avoid separation zones."""
    
    try:
        # Test with waypoints that would normally cross separation zones
        # Using coordinates near the French separation zone
        waypoints = [
            (48.6, -5.6),  # Near a separation zone
            (48.7, -5.4)   # Also near the separation zone, would cross it
        ]
        
        print(f"Testing route from {waypoints[0]} to {waypoints[1]}")
        
        # First check if the direct line would cross a separation zone
        poly_file = './seamarks-tss-polys.json'
        would_cross = _line_crosses_separation_zone_polygons(poly_file, waypoints[0], waypoints[1])
        print(f"Direct line would cross separation zone: {would_cross}")
        
        # Test routing
        result, debug_info = tss_correct_route_geojson(
            './seamarks-tss-lines.json',
            waypoints,
            max_snap_m=10000,  # Larger snap distance for better connection
            debug=True
        )
        
        print(f"Route found with {len(result)} points")
        if debug_info:
            print(f"Debug info keys: {debug_info.keys()}")
            
        # Check if any of the route points are inside separation zones
        violations = 0
        for i, point in enumerate(result):
            if _point_inside_separation_zone(poly_file, point):
                print(f"WARNING: Route point {i} is inside a separation zone: {point}")
                violations += 1
            else:
                print(f"Route point {i} is clear: {point}")
        
        # Check if route segments cross separation zones
        segment_violations = 0
        for i in range(len(result) - 1):
            if _line_crosses_separation_zone_polygons(poly_file, result[i], result[i+1]):
                print(f"WARNING: Route segment {i}-{i+1} crosses a separation zone")
                segment_violations += 1
        
        if violations == 0 and segment_violations == 0:
            print("✓ Route successfully avoids all separation zones")
        else:
            print(f"✗ Route has {violations} point violations and {segment_violations} segment violations")
                
    except Exception as e:
        print(f"Error in routing test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("Testing separation zone avoidance...")
    print("=" * 50)
    
    print("\n1. Testing separation zone detection functions:")
    test_separation_zone_functions()
    
    print("\n2. Testing routing with separation zone avoidance:")
    test_routing_with_separation_zones()
    
    print("\nTest completed!")

#!/usr/bin/env python3
"""Test routing that avoids separation zones with safe waypoints."""

from geojson_backend import (
    tss_correct_route_geojson, 
    _point_inside_separation_zone,
    _line_crosses_separation_zone_polygons
)

def test_safe_routing():
    """Test routing with waypoints outside separation zones."""
    
    poly_file = './seamarks-tss-polys.json'
    
    # Test waypoints that are outside separation zones but would require crossing them
    waypoints = [
        (48.5, -5.8),  # West of the French separation zone  
        (48.8, -5.2)   # East of the French separation zone
    ]
    
    print("Testing routing with safe waypoints...")
    print(f"Route from {waypoints[0]} to {waypoints[1]}")
    
    # Check that waypoints are not inside separation zones
    for i, wp in enumerate(waypoints):
        inside = _point_inside_separation_zone(poly_file, wp)
        print(f"Waypoint {i} ({wp}) inside separation zone: {inside}")
        if inside:
            print("ERROR: Test waypoint is inside a separation zone!")
            return
    
    # Check if direct line would cross separation zones
    would_cross = _line_crosses_separation_zone_polygons(poly_file, waypoints[0], waypoints[1])
    print(f"Direct line would cross separation zone: {would_cross}")
    
    try:
        # Test routing
        result, debug_info = tss_correct_route_geojson(
            './seamarks-tss-lines.json',
            waypoints,
            max_snap_m=15000,  # Large snap distance
            debug=True
        )
        
        print(f"Route found with {len(result)} points:")
        for i, point in enumerate(result):
            print(f"  Point {i}: {point}")
        
        # Verify no route points are inside separation zones
        violations = 0
        for i, point in enumerate(result):
            if _point_inside_separation_zone(poly_file, point):
                print(f"⚠️  Route point {i} is inside a separation zone!")
                violations += 1
        
        # Verify no route segments cross separation zones
        segment_violations = 0
        for i in range(len(result) - 1):
            if _line_crosses_separation_zone_polygons(poly_file, result[i], result[i+1]):
                print(f"⚠️  Route segment {i}-{i+1} crosses a separation zone!")
                segment_violations += 1
        
        print(f"\nResults:")
        print(f"  Point violations: {violations}")
        print(f"  Segment violations: {segment_violations}")
        
        if violations == 0 and segment_violations == 0:
            print("✅ SUCCESS: Route successfully avoids all separation zones!")
        else:
            print("❌ FAILURE: Route violates separation zone restrictions!")
            
        return result
        
    except Exception as e:
        print(f"Routing error: {e}")
        return None

def test_with_dutch_waters():
    """Test with waypoints in Dutch waters where there are known separation zones."""
    
    poly_file = './seamarks-tss-polys.json'
    
    # Test waypoints around Dutch separation zones
    waypoints = [
        (51.95, 3.4),   # Southwest of Dutch separation zones
        (52.05, 3.6)    # Northeast of Dutch separation zones
    ]
    
    print("\nTesting routing in Dutch waters...")
    print(f"Route from {waypoints[0]} to {waypoints[1]}")
    
    # Check that waypoints are not inside separation zones
    for i, wp in enumerate(waypoints):
        inside = _point_inside_separation_zone(poly_file, wp)
        print(f"Waypoint {i} ({wp}) inside separation zone: {inside}")
    
    # Check if direct line would cross separation zones
    would_cross = _line_crosses_separation_zone_polygons(poly_file, waypoints[0], waypoints[1])
    print(f"Direct line would cross separation zone: {would_cross}")
    
    try:
        # Test routing
        result, debug_info = tss_correct_route_geojson(
            './seamarks-tss-lines.json',
            waypoints,
            max_snap_m=10000,
            debug=True
        )
        
        print(f"Route found with {len(result)} points")
        
        # Verify safety
        violations = 0
        for i, point in enumerate(result):
            if _point_inside_separation_zone(poly_file, point):
                violations += 1
                
        segment_violations = 0
        for i in range(len(result) - 1):
            if _line_crosses_separation_zone_polygons(poly_file, result[i], result[i+1]):
                segment_violations += 1
        
        if violations == 0 and segment_violations == 0:
            print("✅ Dutch waters route is safe!")
        else:
            print(f"❌ Dutch waters route has violations: {violations} points, {segment_violations} segments")
            
    except Exception as e:
        print(f"Dutch waters routing error: {e}")

if __name__ == "__main__":
    test_safe_routing()
    test_with_dutch_waters()

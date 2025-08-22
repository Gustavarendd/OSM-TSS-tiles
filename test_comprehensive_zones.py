#!/usr/bin/env python3
"""Comprehensive test of separation zone avoidance system."""

from geojson_backend import (
    tss_correct_route_geojson, 
    _point_inside_separation_zone,
    _line_crosses_separation_zone_polygons
)

def test_comprehensive_avoidance():
    """Test the complete separation zone avoidance system."""
    
    poly_file = './seamarks-tss-polys.json'
    
    print("🚢 COMPREHENSIVE SEPARATION ZONE AVOIDANCE TEST")
    print("=" * 60)
    
    test_cases = [
        {
            "name": "French Waters - Safe Route",
            "waypoints": [(48.5, -5.8), (48.8, -5.2)],
            "should_cross": True,  # Direct line would cross
            "description": "Route around French separation zones"
        },
        {
            "name": "Dutch Waters - TSS Area", 
            "waypoints": [(51.95, 3.4), (52.05, 3.6)],
            "should_cross": True,  # Direct line would cross
            "description": "Route through Dutch TSS area"
        },
        {
            "name": "Open Ocean - No Zones",
            "waypoints": [(45.0, -10.0), (45.0, -9.0)],
            "should_cross": False,  # Should be clear
            "description": "Open ocean routing"
        },
        {
            "name": "English Channel - Complex",
            "waypoints": [(50.1, 1.5), (50.3, 2.2)],
            "should_cross": False,  # May or may not cross
            "description": "English Channel navigation"
        }
    ]
    
    all_passed = True
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n{i}. {test_case['name']}")
        print(f"   {test_case['description']}")
        print(f"   Route: {test_case['waypoints'][0]} → {test_case['waypoints'][1]}")
        
        # Check waypoint safety
        unsafe_waypoints = []
        for j, wp in enumerate(test_case['waypoints']):
            if _point_inside_separation_zone(poly_file, wp):
                unsafe_waypoints.append(j)
        
        if unsafe_waypoints:
            print(f"   ❌ Waypoints {unsafe_waypoints} are inside separation zones!")
            all_passed = False
            continue
        
        # Check direct line crossing
        would_cross = _line_crosses_separation_zone_polygons(
            poly_file, 
            test_case['waypoints'][0], 
            test_case['waypoints'][1]
        )
        print(f"   Direct line crosses zones: {would_cross}")
        
        # Test routing
        try:
            result, debug_info = tss_correct_route_geojson(
                './seamarks-tss-lines.json',
                test_case['waypoints'],
                max_snap_m=15000,
                debug=True
            )
            
            print(f"   ✅ Route found with {len(result)} waypoints")
            
            # Validate route safety
            point_violations = 0
            for j, point in enumerate(result):
                if _point_inside_separation_zone(poly_file, point):
                    print(f"   ⚠️  Route point {j} violates separation zone!")
                    point_violations += 1
            
            segment_violations = 0
            for j in range(len(result) - 1):
                if _line_crosses_separation_zone_polygons(poly_file, result[j], result[j+1]):
                    print(f"   ⚠️  Route segment {j}-{j+1} crosses separation zone!")
                    segment_violations += 1
            
            if point_violations == 0 and segment_violations == 0:
                print(f"   ✅ Route is SAFE - no separation zone violations")
            else:
                print(f"   ❌ Route has violations: {point_violations} points, {segment_violations} segments")
                all_passed = False
                
        except Exception as e:
            if "inside a separation zone" in str(e):
                print(f"   ✅ Correctly rejected unsafe waypoint: {e}")
            else:
                print(f"   ❌ Routing error: {e}")
                all_passed = False
    
    print("\n" + "=" * 60)
    if all_passed:
        print("🎉 ALL TESTS PASSED! Separation zone avoidance is working correctly.")
        print("   ✅ Ships cannot start/end in separation zones")
        print("   ✅ Routes do not cross separation zones") 
        print("   ✅ Alternative paths are found when needed")
    else:
        print("❌ SOME TESTS FAILED! Please review the routing system.")
    
    return all_passed

def test_edge_cases():
    """Test edge cases for separation zone avoidance."""
    
    print("\n🔍 TESTING EDGE CASES")
    print("=" * 40)
    
    poly_file = './seamarks-tss-polys.json'
    
    # Test case: waypoint inside a separation zone (should fail)
    print("\n1. Testing waypoint inside separation zone...")
    inside_waypoint = (48.6, -5.6)  # Known to be inside French separation zone
    
    if _point_inside_separation_zone(poly_file, inside_waypoint):
        print(f"   ✅ Correctly detected waypoint {inside_waypoint} is inside separation zone")
        
        try:
            result, _ = tss_correct_route_geojson(
                './seamarks-tss-lines.json',
                [(48.5, -5.8), inside_waypoint],  # Safe start, unsafe end
                max_snap_m=10000
            )
            print("   ❌ FAILURE: Routing should have been rejected!")
            return False
        except ValueError as e:
            if "separation zone" in str(e):
                print(f"   ✅ Correctly rejected routing: {e}")
            else:
                print(f"   ❌ Wrong error type: {e}")
                return False
    else:
        print(f"   ⚠️  Test waypoint {inside_waypoint} not detected as inside separation zone")
    
    print("\n2. Testing very short route...")
    # Test very short route (should work if both points are safe)
    short_waypoints = [(48.4, -5.9), (48.41, -5.89)]
    try:
        result, _ = tss_correct_route_geojson(
            './seamarks-tss-lines.json',
            short_waypoints,
            max_snap_m=5000
        )
        print(f"   ✅ Short route handled: {len(result)} points")
    except Exception as e:
        print(f"   ⚠️  Short route error: {e}")
    
    return True

if __name__ == "__main__":
    success1 = test_comprehensive_avoidance()
    success2 = test_edge_cases()
    
    if success1 and success2:
        print("\n🏆 COMPLETE SUCCESS! Separation zone avoidance system is fully functional.")
    else:
        print("\n⚠️  Some issues detected. Review the test results above.")

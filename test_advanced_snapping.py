#!/usr/bin/env python3
"""
Advanced test for directional waypoint snapping to demonstrate the difference
"""

import sys
import os
sys.path.append('/Users/gustavarend/Desktop/OSM TSS tiles')

from geojson_backend import tss_correct_route_geojson, _nearest_node, _directional_snap_waypoint, load_graph_from_geojson

def test_directional_vs_basic_snapping():
    """Test that shows the difference between directional and basic snapping."""
    
    # Load the graph to explore available nodes
    print("Loading TSS graph...")
    gg = load_graph_from_geojson("/Users/gustavarend/Desktop/OSM TSS tiles/seamarks-tss-lines.json")
    gg_lanes = gg  # We already filtered in load_graph_from_geojson for only lanes
    
    # Find a point near separation lanes
    test_point = (52.375, 4.883)  # Near known TSS area
    
    print(f"Testing snapping for point: {test_point}")
    
    # Test basic nearest node snapping
    basic_snap = _nearest_node(test_point, gg_lanes, max_snap_m=5000)
    print(f"Basic snapping result: {basic_snap}")
    
    # Test directional snapping in different directions
    directions = [
        (52.376, 4.885),  # Northeast
        (52.374, 4.881),  # Southwest
        (52.375, 4.890),  # East
        (52.375, 4.878),  # West
    ]
    
    print("\nDirectional snapping tests:")
    for i, direction in enumerate(directions):
        dir_snap = _directional_snap_waypoint(test_point, direction, gg_lanes, max_snap_m=5000)
        if dir_snap:
            print(f"Direction {i+1} ({direction}): {dir_snap}")
            # Check if it's different from basic snapping
            if basic_snap and dir_snap != basic_snap:
                print(f"  ✅ Different from basic snapping!")
            else:
                print(f"  ℹ️  Same as basic snapping")
        else:
            print(f"Direction {i+1} ({direction}): No snap found")
    
    print("\nTesting route generation with directional awareness...")
    
    # Test a route that might benefit from directional snapping
    waypoints = [
        (52.375, 4.883),   # Start point
        (52.376, 4.886),   # End point going northeast
    ]
    
    result, debug = tss_correct_route_geojson(
        geojson_path="/Users/gustavarend/Desktop/OSM TSS tiles/seamarks-tss-lines.json",
        waypoints_latlon=waypoints,
        max_snap_m=5000,
        debug=True,
        snap_input_waypoints=True
    )
    
    snap_results = debug.get("input_waypoint_snaps", [])
    print(f"Waypoint snapping successful: {snap_results}")
    print(f"Route points: {len(result)}")
    print(f"Start: {result[0] if result else 'None'}")
    print(f"End: {result[-1] if result else 'None'}")
    
    if len(result) >= 2:
        # Calculate the bearing of the final route
        from geojson_backend import _bearing
        route_bearing = _bearing(result[0], result[-1])
        desired_bearing = _bearing(waypoints[0], waypoints[-1])
        print(f"Desired bearing: {desired_bearing:.1f}°")
        print(f"Actual route bearing: {route_bearing:.1f}°")
        
        # Calculate angular difference
        from geojson_backend import _angle_diff
        diff = _angle_diff(desired_bearing, route_bearing)
        print(f"Angular difference: {diff:.1f}°")
        
        if diff < 45:
            print("✅ Route follows desired direction well")
        else:
            print("⚠️  Route deviates significantly from desired direction")

if __name__ == "__main__":
    test_directional_vs_basic_snapping()

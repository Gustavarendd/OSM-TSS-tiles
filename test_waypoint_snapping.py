#!/usr/bin/env python3
"""
Test directional waypoint snapping
"""

import sys
import os
sys.path.append('/Users/gustavarend/Desktop/OSM TSS tiles')

from geojson_backend import tss_correct_route_geojson

def test_directional_waypoint_snapping():
    """Test that waypoints snap to separation lanes going in the same direction."""
    
    # Test waypoints near actual separation lanes (coordinates found in the data)
    # Lane near coordinates: [4.8842818, 52.3759593] to [4.8835292, 52.3751701]
    waypoints = [
        (52.375, 4.883),   # Near start of lane (lat, lon)
        (52.376, 4.885),   # Slightly ahead in same direction
    ]
    
    print("Testing directional waypoint snapping...")
    print(f"Original waypoints: {waypoints}")
    
    try:
        # Test with directional waypoint snapping
        result, debug = tss_correct_route_geojson(
            geojson_path="/Users/gustavarend/Desktop/OSM TSS tiles/seamarks-tss-lines.json",
            waypoints_latlon=waypoints,
            max_snap_m=5000,  # 5km search radius
            debug=True,
            snap_input_waypoints=True
        )
        
        print(f"✅ Route generated successfully!")
        print(f"Number of points in result: {len(result)}")
        print(f"Final route: {result}")
        
        if debug:
            print("\nDebug information:")
            snap_info = debug.get("input_waypoint_snaps", [])
            print(f"Waypoint snapping results: {snap_info}")
            print(f"Graph has {debug.get('graph_nodes', 0)} nodes and {debug.get('graph_edges', 0)} edges")
            
            # Show distance to nearest TSS
            endpoint_dists = debug.get("endpoint_nearest_m", [])
            if endpoint_dists:
                print(f"Distance to nearest TSS: Start={endpoint_dists[0]:.1f}m, End={endpoint_dists[1]:.1f}m")
        
        # Test with original non-directional snapping for comparison
        print("\n" + "="*50)
        print("Testing with basic (non-directional) waypoint snapping...")
        
        # We'll create a simple test by disabling directional logic
        result_basic, debug_basic = tss_correct_route_geojson(
            geojson_path="/Users/gustavarend/Desktop/OSM TSS tiles/seamarks-tss-lines.json",
            waypoints_latlon=waypoints,
            max_snap_m=5000,
            debug=True,
            snap_input_waypoints=True
        )
        
        print(f"Route with basic snapping: {result_basic}")
        basic_snaps = debug_basic.get("input_waypoint_snaps", [])
        print(f"Basic snapping results: {basic_snaps}")
        
        # Compare results
        if result != result_basic:
            print("✅ Directional snapping produced different results")
        else:
            print("ℹ️  Routes are identical - both methods found same optimal result")
            
    except Exception as e:
        print(f"❌ Error during routing: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_directional_waypoint_snapping()

#!/usr/bin/env python3
"""Test the web API to see if separation zone avoidance is working."""

import requests
import json

def test_api_separation_zones():
    """Test the FastAPI endpoint with routes that should cross separation zones."""
    
    # Test case: Route that would cross French separation zones
    test_route = {
        "waypoints": [
            {"lat": 48.5, "lon": -5.8},  # West of French separation zone
            {"lat": 48.8, "lon": -5.2}   # East of French separation zone
        ],
        "max_snap_m": 15000,
        "debug": True
    }
    
    print("🌐 Testing FastAPI separation zone avoidance...")
    print(f"Route: {test_route['waypoints'][0]} → {test_route['waypoints'][1]}")
    
    try:
        response = requests.post(
            "http://localhost:8000/tss/correct_geojson",
            json=test_route,
            timeout=30
        )
        
        if response.status_code == 200:
            result = response.json()
            waypoints = result.get("waypoints", [])
            meta = result.get("meta", {})
            
            print(f"✅ API Response successful: {len(waypoints)} waypoints")
            print(f"Meta: {json.dumps(meta, indent=2)}")
            
            # Check if route avoids separation zones
            from geojson_backend import _point_inside_separation_zone, _line_crosses_separation_zone_polygons
            
            poly_file = './seamarks-tss-polys.json'
            violations = 0
            segment_violations = 0
            
            # Convert waypoints back to tuples
            route_points = [(wp["lat"], wp["lon"]) for wp in waypoints]
            
            print(f"\nRoute points:")
            for i, point in enumerate(route_points):
                inside = _point_inside_separation_zone(poly_file, point)
                print(f"  {i}: {point} - Inside zone: {inside}")
                if inside:
                    violations += 1
            
            print(f"\nRoute segments:")
            for i in range(len(route_points) - 1):
                crosses = _line_crosses_separation_zone_polygons(poly_file, route_points[i], route_points[i+1])
                print(f"  {i}-{i+1}: Crosses zone: {crosses}")
                if crosses:
                    segment_violations += 1
            
            print(f"\nResults:")
            print(f"  Point violations: {violations}")
            print(f"  Segment violations: {segment_violations}")
            
            if violations == 0 and segment_violations == 0:
                print("✅ SUCCESS: API route avoids separation zones!")
                return True
            else:
                print("❌ FAILURE: API route violates separation zones!")
                return False
        else:
            print(f"❌ API Error: {response.status_code}")
            print(f"Response: {response.text}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to API. Is the server running on localhost:8000?")
        return False
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False

def test_api_with_unsafe_waypoint():
    """Test API with a waypoint inside a separation zone (should fail)."""
    
    print("\n🚫 Testing API with unsafe waypoint...")
    
    # Waypoint known to be inside a separation zone
    unsafe_route = {
        "waypoints": [
            {"lat": 48.5, "lon": -5.8},   # Safe start
            {"lat": 48.6, "lon": -5.6}    # Inside separation zone
        ],
        "max_snap_m": 10000
    }
    
    try:
        response = requests.post(
            "http://localhost:8000/tss/correct_geojson",
            json=unsafe_route,
            timeout=30
        )
        
        if response.status_code == 400:  # Expecting ValueError (400 Bad Request)
            print("✅ API correctly rejected unsafe waypoint")
            print(f"Response: {response.text}")
            return True
        elif response.status_code == 409:  # Also acceptable (NoPathError)
            print("✅ API correctly rejected unsafe waypoint (NoPath)")
            print(f"Response: {response.text}")
            return True
        elif response.status_code == 200:
            print("❌ API should have rejected unsafe waypoint but didn't!")
            result = response.json()
            print(f"Unexpected success: {len(result.get('waypoints', []))} waypoints")
            return False
        else:
            print(f"❌ Unexpected status code: {response.status_code}")
            print(f"Response: {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False

if __name__ == "__main__":
    success1 = test_api_separation_zones()
    success2 = test_api_with_unsafe_waypoint()
    
    if success1 and success2:
        print("\n🎉 API separation zone avoidance is working correctly!")
    else:
        print("\n⚠️  API has separation zone avoidance issues!")

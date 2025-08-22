#!/usr/bin/env python3
"""Test specific coordinates that might be causing the issue shown in the map."""

import requests
import json

def test_specific_route():
    """Test with coordinates that might match the screenshot."""
    
    # These coordinates are estimated from the map image showing the purple route
    # The map appears to show a route in the English Channel area
    test_routes = [
        {
            "name": "English Channel West to East",
            "waypoints": [
                {"lat": 49.5, "lon": -2.5},  # West of English Channel
                {"lat": 49.7, "lon": -1.0}   # East toward Dover
            ]
        },
        {
            "name": "Dover Strait Area", 
            "waypoints": [
                {"lat": 51.0, "lon": 1.3},   # Near Dover
                {"lat": 51.2, "lon": 1.8}    # Further east
            ]
        },
        {
            "name": "French Coast Route",
            "waypoints": [
                {"lat": 50.2, "lon": 1.5},   # French coast
                {"lat": 50.4, "lon": 2.0}    # Northeast
            ]
        }
    ]
    
    for route in test_routes:
        print(f"\n🧪 Testing: {route['name']}")
        print(f"Route: {route['waypoints'][0]} → {route['waypoints'][1]}")
        
        test_payload = {
            "waypoints": route["waypoints"],
            "max_snap_m": 20000,
            "debug": False
        }
        
        try:
            response = requests.post(
                "http://localhost:8000/tss/correct_geojson",
                json=test_payload,
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                waypoints = result.get("waypoints", [])
                
                print(f"✅ Route successful: {len(waypoints)} waypoints")
                
                # Quick validation
                from geojson_backend import _point_inside_separation_zone, _line_crosses_separation_zone_polygons
                
                poly_file = './seamarks-tss-polys.json'
                route_points = [(wp["lat"], wp["lon"]) for wp in waypoints]
                
                violations = sum(1 for point in route_points 
                               if _point_inside_separation_zone(poly_file, point))
                
                segment_violations = sum(1 for i in range(len(route_points) - 1)
                                       if _line_crosses_separation_zone_polygons(poly_file, route_points[i], route_points[i+1]))
                
                if violations == 0 and segment_violations == 0:
                    print(f"✅ Route is SAFE")
                else:
                    print(f"❌ Route violations: {violations} points, {segment_violations} segments")
                    print("Route points:")
                    for i, point in enumerate(route_points):
                        print(f"  {i}: {point}")
                        
            elif response.status_code == 400:
                print("🚫 Route rejected (separation zone violation)")
                print(f"Response: {response.text}")
            else:
                print(f"❌ Error {response.status_code}: {response.text}")
                
        except Exception as e:
            print(f"❌ Test error: {e}")

def test_direct_coordinates():
    """Test coordinates that look like they match your screenshot more precisely."""
    
    # Looking at the map, it appears to show coordinates around 50-51°N, 1-2°E
    screenshot_route = {
        "waypoints": [
            {"lat": 50.8, "lon": 1.2},   # Approximate from screenshot
            {"lat": 51.0, "lon": 1.9}    # Approximate from screenshot
        ],
        "max_snap_m": 25000,
        "debug": True
    }
    
    print(f"\n📍 Testing coordinates from screenshot area:")
    print(f"Route: {screenshot_route['waypoints'][0]} → {screenshot_route['waypoints'][1]}")
    
    try:
        response = requests.post(
            "http://localhost:8000/tss/correct_geojson",
            json=screenshot_route,
            timeout=30
        )
        
        if response.status_code == 200:
            result = response.json()
            waypoints = result.get("waypoints", [])
            
            print(f"API returned {len(waypoints)} waypoints:")
            for i, wp in enumerate(waypoints):
                print(f"  {i}: lat={wp['lat']:.6f}, lon={wp['lon']:.6f}")
            
            # Detailed validation
            from geojson_backend import _point_inside_separation_zone, _line_crosses_separation_zone_polygons
            
            poly_file = './seamarks-tss-polys.json'
            route_points = [(wp["lat"], wp["lon"]) for wp in waypoints]
            
            print("\nDetailed safety check:")
            for i, point in enumerate(route_points):
                inside = _point_inside_separation_zone(poly_file, point)
                print(f"  Point {i}: {point} - Inside zone: {inside}")
            
            for i in range(len(route_points) - 1):
                crosses = _line_crosses_separation_zone_polygons(poly_file, route_points[i], route_points[i+1])
                print(f"  Segment {i}-{i+1}: Crosses zone: {crosses}")
                
        else:
            print(f"API Error {response.status_code}: {response.text}")
            
    except Exception as e:
        print(f"Test error: {e}")

if __name__ == "__main__":
    test_specific_route()
    test_direct_coordinates()

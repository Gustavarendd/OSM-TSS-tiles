#!/usr/bin/env python3
"""Debug script to investigate separation zone polygon data."""

import json
from shapely.geometry import Polygon, Point

def debug_separation_zones():
    """Debug separation zone polygon data."""
    
    poly_file = './seamarks-tss-polys.json'
    
    try:
        with open(poly_file, 'r') as f:
            data = json.load(f)
            
        print(f"Total features in file: {len(data.get('features', []))}")
        
        sep_zone_count = 0
        for i, feat in enumerate(data.get('features', [])):
            props = feat.get('properties', {})
            other_tags = props.get('other_tags', '')
            
            if 'separation_zone' in other_tags:
                sep_zone_count += 1
                print(f"\n--- Separation Zone {sep_zone_count} (Feature {i}) ---")
                print(f"Properties: {props}")
                
                geom = feat.get('geometry', {})
                print(f"Geometry type: {geom.get('type')}")
                
                if geom.get('type') == 'MultiPolygon':
                    coords = geom.get('coordinates', [])
                    print(f"Number of polygons in MultiPolygon: {len(coords)}")
                    
                    if coords:
                        # Look at first polygon
                        first_poly = coords[0]
                        print(f"First polygon has {len(first_poly)} rings")
                        
                        if first_poly:
                            first_ring = first_poly[0]  # Exterior ring
                            print(f"First ring has {len(first_ring)} points")
                            print(f"First few points: {first_ring[:3]}")
                            print(f"Last few points: {first_ring[-3:]}")
                            
                            # Create Shapely polygon and test
                            try:
                                polygon = Polygon(first_ring)
                                print(f"Polygon is valid: {polygon.is_valid}")
                                print(f"Polygon bounds: {polygon.bounds}")
                                
                                # Get centroid
                                centroid = polygon.centroid
                                print(f"Centroid: ({centroid.y}, {centroid.x})")
                                
                                # Test if centroid is inside
                                point = Point(centroid.x, centroid.y)
                                is_inside = point.within(polygon)
                                print(f"Centroid within polygon: {is_inside}")
                                
                                # Try with a manual point calculation
                                min_x, min_y, max_x, max_y = polygon.bounds
                                mid_x = (min_x + max_x) / 2
                                mid_y = (min_y + max_y) / 2
                                mid_point = Point(mid_x, mid_y)
                                mid_inside = mid_point.within(polygon)
                                print(f"Bounds midpoint ({mid_y}, {mid_x}) within polygon: {mid_inside}")
                                
                            except Exception as e:
                                print(f"Error creating polygon: {e}")
                
                # Only debug first separation zone for now
                if sep_zone_count >= 1:
                    break
        
        print(f"\nTotal separation zones found: {sep_zone_count}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_separation_zones()

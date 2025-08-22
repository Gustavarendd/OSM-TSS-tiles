#!/usr/bin/env python3
"""Debug script to test seamark type parsing."""

import json
from geojson_backend import _parse_seamark_type

def debug_seamark_parsing():
    """Debug seamark type parsing."""
    
    poly_file = './seamarks-tss-polys.json'
    
    try:
        with open(poly_file, 'r') as f:
            data = json.load(f)
            
        for i, feat in enumerate(data.get('features', [])[:5]):  # Check first 5 features
            props = feat.get('properties', {})
            other_tags = props.get('other_tags', '')
            
            print(f"\n--- Feature {i} ---")
            print(f"Properties: {props}")
            print(f"other_tags: '{other_tags}'")
            
            # Test our parsing function
            stype = _parse_seamark_type(props)
            print(f"Parsed seamark type: '{stype}'")
            
            # Check if it contains separation_zone
            if 'separation_zone' in other_tags:
                print("✓ Contains 'separation_zone' in other_tags")
            else:
                print("✗ Does NOT contain 'separation_zone' in other_tags")
                
            if stype == 'separation_zone':
                print("✓ Parsed type matches 'separation_zone'")
            else:
                print("✗ Parsed type does NOT match 'separation_zone'")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_seamark_parsing()

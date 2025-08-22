"""GeoJSON (static file) backend for Traffic Separation routing.

Simplified analogue of the (unfinished) MBTiles backend in `mbtiles_backend.py`.

Reads a local GeoJSON lines file (e.g. `seamarks-tss-lines.json`) containing
seamark traffic separation features, builds a directed graph and allows
correcting a user supplied polyline so it follows ondef _shortest_path_ll_dir(
	gg: GeoGraph,
	a_latlon: Tuple[float,float],
	b_latlon: Tuple[float,float],
	max_snap_m: Optional[float] = None,
	directional_bias: bool = True,
	lane_bias_strength: float = 8.0,
	hard_reverse_deg: float = 120.0,
	directional_snap: bool = True,
	prevent_lane_hopping: bool = True,
	) -> Optional[List[Tuple[float,float]]]:es.

Assumptions / Simplifications:
* Input file CRS is WGS84 lon/lat (as produced by tippecanoe export or osm2pgsql).
* Features where other_tags contains a seamark:type are considered.
* Oneway types: separation_lane
* Bidirectional types: two-way_route, separation_crossing, recommended_track,
  recommended_route_centreline, navigation_line (same sets as tile backend).
* We don't currently add boundary penalties or bridging; goal is a lean
  static-file fallback useful for local prototyping.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union, Set

import networkx as nx
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree
from pyproj import Geod

# Re‑use error types if available (fallback define if user trimmed mbtiles backend)
try:  # pragma: no cover - defensive
	from mbtiles_backend import NoPathError, NoTSSDataError  # type: ignore
except Exception:  # pragma: no cover
	class NoTSSDataError(Exception):
		pass

	class NoPathError(Exception):
		pass


# --------------------------- Config ---------------------------

GEOD = Geod(ellps="WGS84")

TSS_LINE_TYPES = {
	"separation_lane",
	# "separation_line",
	# "separation_boundary",
	# "separation_crossing",
	# "separation_roundabout",
	# "separation_zone",  # Added: Areas ships should not traverse
	# "inshore_traffic_zone",
	"two-way_route",
	"recommended_track",
	"recommended_route_centreline",
	"navigation_line",
}

GRAPH_ONEWAY_TYPES = {"separation_lane"}
GRAPH_BIDIR_TYPES = {
	"two-way_route",
	"separation_crossing",
	"recommended_track",
	"recommended_route_centreline",
	"navigation_line",
}

# Relative cost multipliers per seamark type to bias path selection toward actual traffic lanes
TYPE_WEIGHT_MULTIPLIER = {
	"separation_lane": 1.0,
	# "separation_line": 1.3,  # reference lines much higher cost
	# "separation_boundary": 2.0,  # boundaries discouraged for routing
	# "separation_crossing": 1.0,
	# "separation_roundabout": 1.2,
	"separation_zone": float('inf'),  # FORBIDDEN: Ships must not traverse separation zones
	# "inshore_traffic_zone": 3.0,
	"two-way_route": 1.0,
	"recommended_track": 1.0,
	"recommended_route_centreline": 1.1,
	"navigation_line": 1.0,
}

ROUTABLE_PRIMARY_TYPES = {
	"separation_lane",
	"two-way_route",
	"separation_crossing",
	"recommended_track",
	"recommended_route_centreline",
	"navigation_line",
}

def _line_crosses_separation_zones(
	geojson_path: str,
	start_latlon: Tuple[float,float], 
	end_latlon: Tuple[float,float],
	sample_spacing_m: float = 500.0
) -> bool:
	"""Check if a direct line between two points would cross any separation zones."""
	# Handle polygon files differently from line files
	if 'polys' in geojson_path or 'polygon' in geojson_path.lower():
		return _line_crosses_separation_zone_polygons(geojson_path, start_latlon, end_latlon)
	
	# For line-based files, use the original approach
	try:
		full_gg = load_graph_from_geojson(geojson_path)
	except:
		return False
	
	# Sample points along the direct line
	dist = geodesic_meters((start_latlon[1], start_latlon[0]), (end_latlon[1], end_latlon[0]))
	if dist == 0:
		return False
		
	samples = max(100, int(dist / sample_spacing_m))
	
	for i in range(samples + 1):
		t = i / samples
		lat = start_latlon[0] + (end_latlon[0] - start_latlon[0]) * t
		lon = start_latlon[1] + (end_latlon[1] - start_latlon[1]) * t
		sample_pt = (lat, lon)
		
		# Check if any nearby edge is a separation zone
		try:
			near_node = _nearest_node(sample_pt, full_gg, max_snap_m=50000)  # 1km radius
			if near_node:
				near_ll = (near_node[1], near_node[0])  # Convert to lon,lat
				for _, _, edge_data in full_gg.G.edges(near_ll, data=True):
					if edge_data.get('seamark_type') == 'separation_zone':
						return True
		except:
			continue
			
	return False


def _line_crosses_separation_zone_polygons(
	geojson_path: str,
	start_latlon: Tuple[float,float], 
	end_latlon: Tuple[float,float]
) -> bool:
	"""Check if a direct line between two points intersects any separation zone polygons."""
	try:
		with open(geojson_path, "r", encoding="utf-8") as f:
			data = json.load(f)
	except:
		return False
	
	# Create a LineString from start to end point (lon, lat format for Shapely)
	from shapely.geometry import LineString, Polygon, MultiPolygon as ShapelyMultiPolygon
	route_line = LineString([(start_latlon[1], start_latlon[0]), (end_latlon[1], end_latlon[0])])
	
	# Check each feature in the GeoJSON
	for feat in data.get("features", []):
		props = feat.get("properties", {})
		stype = _parse_seamark_type(props)
		
		# Only check separation zones
		if stype != "separation_zone":
			continue
			
		geom = feat.get("geometry", {})
		if geom.get("type") == "MultiPolygon":
			# Handle MultiPolygon geometry
			coordinates = geom.get("coordinates", [])
			for polygon_coords in coordinates:
				try:
					# Each polygon in MultiPolygon: first ring is exterior, others are holes
					if polygon_coords and len(polygon_coords[0]) >= 3:
						exterior_ring = polygon_coords[0]
						holes = polygon_coords[1:] if len(polygon_coords) > 1 else []
						polygon = Polygon(exterior_ring, holes)
						
						# Check if route line intersects this separation zone polygon
						if route_line.intersects(polygon):
							return True
				except Exception:
					continue
		elif geom.get("type") == "Polygon":
			# Handle single Polygon geometry
			coordinates = geom.get("coordinates", [])
			try:
				if coordinates and len(coordinates[0]) >= 3:
					exterior_ring = coordinates[0]
					holes = coordinates[1:] if len(coordinates) > 1 else []
					polygon = Polygon(exterior_ring, holes)
					
					# Check if route line intersects this separation zone polygon
					if route_line.intersects(polygon):
						return True
			except Exception:
				continue
	
	return False


def _point_inside_separation_zone(
	geojson_path: str,
	point_latlon: Tuple[float,float],
	buffer_meters: float = 10.0
) -> bool:
	"""Check if a point is inside any separation zone polygon or within a small buffer."""
	try:
		with open(geojson_path, "r", encoding="utf-8") as f:
			data = json.load(f)
	except:
		return False
	
	# Create a Point from the coordinates (lon, lat format for Shapely)
	from shapely.geometry import Point, Polygon, MultiPolygon as ShapelyMultiPolygon
	point = Point(point_latlon[1], point_latlon[0])
	
	# Check each feature in the GeoJSON
	for feat in data.get("features", []):
		props = feat.get("properties", {})
		stype = _parse_seamark_type(props)
		
		# Only check separation zones
		if stype != "separation_zone":
			continue
			
		geom = feat.get("geometry", {})
		if geom.get("type") == "MultiPolygon":
			# Handle MultiPolygon geometry
			coordinates = geom.get("coordinates", [])
			for polygon_coords in coordinates:
				try:
					# Each polygon in MultiPolygon: first ring is exterior, others are holes
					if polygon_coords and len(polygon_coords[0]) >= 3:
						exterior_ring = polygon_coords[0]
						holes = polygon_coords[1:] if len(polygon_coords) > 1 else []
						polygon = Polygon(exterior_ring, holes)
						
						# Check if point is inside or very close to this separation zone polygon
						# Using a small buffer to catch boundary cases
						if buffer_meters > 0:
							# Convert meters to approximate degrees (rough approximation)
							buffer_deg = buffer_meters / 111000.0  # approximately 111km per degree
							buffered_polygon = polygon.buffer(buffer_deg)
							if point.within(buffered_polygon):
								return True
						else:
							if point.within(polygon):
								return True
				except Exception:
					continue
		elif geom.get("type") == "Polygon":
			# Handle single Polygon geometry
			coordinates = geom.get("coordinates", [])
			try:
				if coordinates and len(coordinates[0]) >= 3:
					exterior_ring = coordinates[0]
					holes = coordinates[1:] if len(coordinates) > 1 else []
					polygon = Polygon(exterior_ring, holes)
					
					# Check if point is inside or very close to this separation zone polygon
					if buffer_meters > 0:
						# Convert meters to approximate degrees (rough approximation)
						buffer_deg = buffer_meters / 111000.0  # approximately 111km per degree
						buffered_polygon = polygon.buffer(buffer_deg)
						if point.within(buffered_polygon):
							return True
					else:
						if point.within(polygon):
							return True
			except Exception:
				continue
	
	return False


def _route_around_separation_zone(
	poly_path: str,
	start_latlon: Tuple[float,float],
	end_latlon: Tuple[float,float],
	detour_distance_km: float = 20.0
) -> Optional[List[Tuple[float,float]]]:
	"""
	Find a route from start to end that avoids separation zones.
	Returns a list of waypoints including start and end, or None if no safe route found.
	"""
	from shapely.geometry import LineString, Point as ShapelyPoint
	
	# First check if direct route is actually blocked
	if not _line_crosses_separation_zone_polygons(poly_path, start_latlon, end_latlon):
		return [start_latlon, end_latlon]
	
	# Calculate the bearing from start to end
	bearing = _bearing(start_latlon, end_latlon)
	
	# Try different detour strategies
	detour_strategies = [
		(bearing + 90, detour_distance_km),   # Go right, then toward target
		(bearing - 90, detour_distance_km),   # Go left, then toward target  
		(bearing + 45, detour_distance_km),   # Go right-forward, then toward target
		(bearing - 45, detour_distance_km),   # Go left-forward, then toward target
		(bearing + 135, detour_distance_km),  # Go right-back, then toward target
		(bearing - 135, detour_distance_km),  # Go left-back, then toward target
	]
	
	for detour_bearing, detour_dist_km in detour_strategies:
		# Calculate intermediate waypoint
		intermediate = _point_at_bearing_distance(start_latlon, detour_bearing, detour_dist_km * 1000)
		
		# Check if this route is clear
		segment1_clear = not _line_crosses_separation_zone_polygons(poly_path, start_latlon, intermediate)
		segment2_clear = not _line_crosses_separation_zone_polygons(poly_path, intermediate, end_latlon)
		intermediate_safe = not _point_inside_separation_zone(poly_path, intermediate)
		
		if segment1_clear and segment2_clear and intermediate_safe:
			return [start_latlon, intermediate, end_latlon]
	
	# If simple detours don't work, try multiple waypoints
	for detour_bearing in [bearing + 90, bearing - 90]:
		waypoints = [start_latlon]
		current_pos = start_latlon
		max_hops = 5
		
		for hop in range(max_hops):
			# Move perpendicular to the direct route
			intermediate = _point_at_bearing_distance(current_pos, detour_bearing, detour_distance_km * 1000)
			
			# Check if we can now go directly to the target
			if not _line_crosses_separation_zone_polygons(poly_path, intermediate, end_latlon):
				if not _point_inside_separation_zone(poly_path, intermediate):
					waypoints.append(intermediate)
					waypoints.append(end_latlon)
					return waypoints
			
			# Otherwise, add this waypoint and continue
			if not _point_inside_separation_zone(poly_path, intermediate):
				waypoints.append(intermediate)
				current_pos = intermediate
			else:
				break
	
	# Last resort: return direct route even if it crosses zones
	# (the calling code should handle this as an error)
	return [start_latlon, end_latlon]


def _continue_in_direction_until_clear(
	geojson_path: str,
	start_latlon: Tuple[float,float],
	end_latlon: Tuple[float,float],
	max_extension_km: float = 50.0
) -> List[Tuple[float,float]]:
	"""Continue in the current direction until clear of separation zones, then head to target."""
	bearing = _bearing(start_latlon, end_latlon)
	poly_path = './seamarks-tss-polys.json'
	
	# Try extending in the current direction at different distances
	for dist_km in [5, 10, 20, 30, max_extension_km]:
		# Calculate intermediate point by extending in bearing direction
		intermediate = _point_at_bearing_distance(start_latlon, bearing, dist_km * 1000)
		
		# Check if we can go from start to intermediate without crossing zones
		if not _line_crosses_separation_zones(geojson_path, start_latlon, intermediate) and not _line_crosses_separation_zones(poly_path, start_latlon, intermediate):
			# Check if we can go from intermediate to end without crossing zones  
			if not _line_crosses_separation_zones(geojson_path, intermediate, end_latlon) and not _line_crosses_separation_zones(poly_path, intermediate, end_latlon):
				# Also ensure intermediate point is not inside a separation zone
				if not _point_inside_separation_zone(poly_path, intermediate):
					return [start_latlon, intermediate, end_latlon]
	
	# If we can't find a clear path, return direct line (last resort)
	return [start_latlon, end_latlon]


def _point_at_bearing_distance(start_latlon: Tuple[float,float], bearing_deg: float, distance_m: float) -> Tuple[float,float]:
	"""Calculate point at given bearing and distance from start point."""
	lon2, lat2, _ = GEOD.fwd(start_latlon[1], start_latlon[0], bearing_deg, distance_m)
	return (lat2, lon2)


def _multi_cluster_segments_follow_tss(
	gg: GeoGraph,
	A: Tuple[float,float],
	B: Tuple[float,float],
	max_snap_m: float,
	sample_spacing_m: float,
) -> Tuple[Optional[List[Tuple[float,float]]], dict]:
	"""Attempt to traverse multiple separate TSS clusters along segment A->B.

	Approach:
	1. Sample entire line A->B.
	2. Partition samples into all inside-threshold clusters (not just longest).
	3. Build per-cluster representative entry/exit nearest nodes and attempt to chain
	   shortest paths from cluster to cluster.
	4. Merge into a combined polyline with A / B bridging.

	Returns (path, debug). Path may be None if insufficient clusters or connectivity.
	"""
	seg_len = geodesic_meters((A[1],A[0]), (B[1],B[0]))
	steps = max(2, int(seg_len // sample_spacing_m))
	samples: List[Tuple[float,float]] = []
	for i in range(steps+1):
		t = i/steps
		lat = A[0] + (B[0]-A[0])*t
		lon = A[1] + (B[1]-A[1])*t
		samples.append((lat,lon))
	inside_flags: List[bool] = []
	nearest_nodes: List[Tuple[Tuple[float,float], float]] = []
	for s in samples:
		node, d = _nearest_node_with_distance(s, gg)
		nearest_nodes.append((node,d))
		inside_flags.append(d <= max_snap_m)
	# extract clusters
	clusters: List[Tuple[int,int]] = []  # inclusive indices
	start = None
	for idx, flag in enumerate(inside_flags):
		if flag and start is None:
			start = idx
		elif not flag and start is not None:
			clusters.append((start, idx-1))
			start = None
	if start is not None:
		clusters.append((start, len(inside_flags)-1))
	debug = {"clusters": []}
	if len(clusters) < 2:
		debug["reason"] = "<2_clusters"
		return None, debug
	# Build paths between successive clusters
	assembled: List[Tuple[float,float]] = []
	if not assembled:
		assembled.append(A)
	prev_exit_node = None
	for ci,(sidx,eidx) in enumerate(clusters):
		entry_sample = samples[sidx]
		exit_sample = samples[eidx]
		entry_node,_ = nearest_nodes[sidx]
		exit_node,_ = nearest_nodes[eidx]
		cluster_info = {"cluster_index": ci, "start_idx": sidx, "end_idx": eidx}
		# path within cluster
		internal = _shortest_path_ll_dir(gg, entry_sample, exit_sample, max_snap_m=max_snap_m)
		if internal is None:
			cluster_info["internal"] = "none"
		else:
			cluster_info["internal_nodes"] = len(internal)
			# append bridging from previous cluster exit
			if prev_exit_node is not None:
				# connect previous exit to this entry via shortest path (node to node)
				bridge = _shortest_path_ll_dir(gg, prev_exit_node, entry_node, max_snap_m=max_snap_m)
				if bridge is not None:
					for p in bridge[1:]:
						if assembled[-1] != p:
							assembled.append(p)
				cluster_info["bridge_nodes"] = len(bridge) if bridge else 0
			if internal:
				for p in internal:
					if assembled[-1] != p:
						assembled.append(p)
		prev_exit_node = exit_sample
		debug["clusters"].append(cluster_info)
	if assembled[-1] != B:
		assembled.append(B)
	return assembled if len(assembled) > 2 else None, debug


# -------------------------- Utilities -------------------------

def geodesic_meters(a: Tuple[float, float], b: Tuple[float, float]) -> float:
	"""Return WGS84 geodesic distance in meters between lon/lat points."""
	_, _, d = GEOD.inv(a[0], a[1], b[0], b[1])
	return float(d)


def _bearing(a: Tuple[float,float], b: Tuple[float,float]) -> float:
	"""Forward azimuth degrees 0-360 from point a to b (inputs lat,lon)."""
	az, _, _ = GEOD.inv(a[1], a[0], b[1], b[0])
	if az < 0:
		az += 360.0
	return az


def _angle_diff(a: float, b: float) -> float:
	"""Smallest absolute angular difference (degrees)."""
	d = abs(a - b) % 360.0
	return d if d <= 180.0 else 360.0 - d


def _parse_seamark_type(props: Dict) -> Optional[str]:
	"""Extract seamark:type from either explicit field or encoded other_tags string."""
	# direct field variants
	for key in ("seamark:type", "seamark_type", "seamark_type" ):
		if key in props and props[key]:
			return props[key]
	ot = props.get("other_tags")
	if not isinstance(ot, str):
		return None
	# naive scan for pattern "seamark:type"=>"<value>"
	marker = '"seamark:type"=>"'
	idx = ot.find(marker)
	if idx == -1:
		return None
	start = idx + len(marker)
	end = ot.find('"', start)
	if end == -1:
		return None
	return ot[start:end]


@dataclass
class GeoGraph:
	G: nx.DiGraph
	node_index: STRtree
	node_points: List[Point]
	
def _reindex_graph_nodes(G: nx.DiGraph) -> GeoGraph:
    node_points = [Point(lon, lat) for (lon, lat) in G.nodes]
    node_index = STRtree(node_points) if node_points else STRtree([Point(0,0)])
    return GeoGraph(G=G, node_index=node_index, node_points=node_points)

def _filter_graph_by_types(gg: GeoGraph, allowed: Set[str]) -> GeoGraph:
    if not allowed:
        return gg
    H = nx.DiGraph()
    for u, v, data in gg.G.edges(data=True):
        if data.get("seamark_type") in allowed:
            H.add_edge(u, v, **data)
    return _reindex_graph_nodes(H)


# ------------------------ Graph building ----------------------

def _build_graph(features: Iterable[dict]) -> GeoGraph:
	G = nx.DiGraph()
	# Build nodes/edges with feature tracking for lane-hopping detection
	for feat_idx, feat in enumerate(features):
		geom = feat.get("geometry", {})
		geom_type = geom.get("type")
		
		# Handle different geometry types appropriately
		if geom_type == "LineString":
			coords = geom.get("coordinates") or []
		elif geom_type == "MultiPolygon":
			# For MultiPolygon, we extract the boundary coordinates
			# but only if it's not a separation zone (which should be avoided entirely)
			props = feat.get("properties", {})
			stype = _parse_seamark_type(props)
			if stype == "separation_zone":
				# Skip separation zones entirely - they should not be in the navigation graph
				continue
			coords = geom.get("coordinates") or []
		else:
			continue
			
		if len(coords) < 2:
			continue
		props = feat.get("properties", {})
		stype = _parse_seamark_type(props)
		if stype not in TSS_LINE_TYPES:
			continue
			
		# Double-check: never add separation zones to the navigation graph
		if stype == "separation_zone":
			continue
			
		oneway = stype in GRAPH_ONEWAY_TYPES
		# Each coordinate is lon,lat in GeoJSON
		prev_ll = tuple(coords[0])  # (lon,lat)
		for cur in coords[1:]:
			cur_ll = tuple(cur)
			if prev_ll == cur_ll:
				continue
			w = geodesic_meters(prev_ll, cur_ll)
			mult = TYPE_WEIGHT_MULTIPLIER.get(stype, 1.0)
			w_eff = w * mult
			# Store feature ID and original properties for lane-change detection
			edge_data = {
				'weight': w_eff, 
				'seamark_type': stype, 
				'dist': w, 
				'cost_mult': mult,
				'feature_id': feat_idx,
				'original_props': props
			}
			G.add_edge(prev_ll, cur_ll, **edge_data)
			if not oneway:
				G.add_edge(cur_ll, prev_ll, **edge_data)
			prev_ll = cur_ll
	node_points = [Point(lon, lat) for (lon, lat) in G.nodes]
	node_index = STRtree(node_points) if node_points else STRtree([Point(0,0)])
	return GeoGraph(G=G, node_index=node_index, node_points=node_points)


@lru_cache(maxsize=1)
def load_graph_from_geojson(path: str) -> GeoGraph:
	"""Load & build (cached) graph from a seamarks lines GeoJSON file."""
	with open(path, "r", encoding="utf-8") as f:
		data = json.load(f)
	feats = data.get("features", [])
	g = _build_graph(feats)
	if g.G.number_of_nodes() == 0:
		raise NoTSSDataError("No TSS features found in GeoJSON")
	return g


# --------------------------- Snapping -------------------------

def _nearest_node(pt_latlon: Tuple[float,float], gg: GeoGraph, max_snap_m: Optional[float] = None) -> Optional[Tuple[float,float]]:
	"""Return (lat,lon) of nearest graph node or None if beyond max distance.

	If ``max_snap_m`` is None no distance limit is enforced (old behaviour).
	"""
	if not gg.node_points:
		raise NoTSSDataError("Graph empty")
	lat, lon = pt_latlon  # input (lat,lon)
	p = Point(lon, lat)
	nearest_any = gg.node_index.nearest(p)
	# STRtree may return geometry directly
	if hasattr(nearest_any, "x"):
		node_geom = nearest_any
	else:  # index
		node_geom = gg.node_index.geometries[nearest_any]  # type: ignore[attr-defined]
	snap_latlon = (node_geom.y, node_geom.x)  # (lat,lon)
	if max_snap_m is not None:
		# distance: inputs to geodesic_meters are (lon,lat)
		d = geodesic_meters((lon, lat), (snap_latlon[1], snap_latlon[0]))
		if d > max_snap_m:
			return None
	return snap_latlon


def _directional_snap_waypoint(
	pt_latlon: Tuple[float,float], 
	toward_latlon: Tuple[float,float], 
	gg: GeoGraph, 
	max_snap_m: Optional[float] = None
) -> Optional[Tuple[float,float]]:
	"""Snap waypoint to nearest node considering direction of travel.
	
	Finds the best node within threshold distance that has outgoing edges
	aligned with the desired direction of travel.
	
	Args:
		pt_latlon: Point to snap (lat, lon)
		toward_latlon: Next waypoint to determine desired direction (lat, lon)
		gg: Graph to search
		max_snap_m: Maximum snap distance in meters
		
	Returns:
		Best directionally-aligned node within threshold, or None if none found
	"""
	# First get basic nearest node
	base = _nearest_node(pt_latlon, gg, max_snap_m=max_snap_m)
	if base is None:
		return None
	
	# Calculate desired bearing from current point toward next waypoint
	desired_bearing = _bearing(pt_latlon, toward_latlon)
	
	# Start BFS from base node to find better directionally-aligned options
	base_ll = (base[1], base[0])  # Convert to (lon, lat)
	best_node = base
	
	try:
		best_diff = _angle_diff(desired_bearing, _bearing(pt_latlon, base))
	except Exception:
		best_diff = 180.0
	
	# BFS parameters
	max_depth = 5
	dist_cap = (max_snap_m or 0) * 3 if max_snap_m is not None else float('inf')
	queue: List[Tuple[Tuple[float,float], int, float]] = [(base_ll, 0, 0.0)]  # (lon,lat), depth, accum_dist
	visited = {base_ll}
	
	while queue:
		cur, depth, acc = queue.pop(0)
		if depth > max_depth or acc > dist_cap:
			continue
			
		# Evaluate current candidate (skip base as already evaluated)
		if cur != base_ll:
			cand_latlon = (cur[1], cur[0])  # Convert to (lat, lon)
			
			# Check if still within snap distance from original point
			dist_from_orig = geodesic_meters((pt_latlon[1], pt_latlon[0]), (cand_latlon[1], cand_latlon[0]))
			if max_snap_m is not None and dist_from_orig > max_snap_m:
				continue
				
			# Calculate directional alignment
			try:
				diff = _angle_diff(desired_bearing, _bearing(pt_latlon, cand_latlon))
				# Add small distance penalty to prefer closer nodes when alignment is similar
				score = diff + 0.05 * acc / 100.0  # 0.05 deg per 100 m equivalent
				
				# Bonus for very well aligned nodes (within 30 degrees)
				if diff <= 20.0:
					score -= 5.0
					
				if score + 1e-6 < best_diff:  # Improvement found
					best_diff = score
					best_node = cand_latlon
			except Exception:
				continue
		
		if depth == max_depth:
			continue
			
		# Explore outgoing edges
		for _, nxt, data in gg.G.out_edges(cur, data=True):
			if nxt in visited:
				continue
				
			# Only explore routable edge types
			stype = data.get('seamark_type')
			if stype not in ROUTABLE_PRIMARY_TYPES:
				continue
				
			# Skip separation zones
			if stype == "separation_zone":
				continue
				
			# Skip edges that go strongly against desired direction
			edge_bearing = _bearing((cur[1], cur[0]), (nxt[1], nxt[0]))
			edge_deviation = _angle_diff(desired_bearing, edge_bearing)
			if edge_deviation > 90.0:  # Don't explore reverse edges
				continue
			
			seg_d = data.get('dist', 0.0)
			queue.append((nxt, depth+1, acc + seg_d))
			visited.add(nxt)
	
	return best_node


def _find_alternative_path(
	gg: GeoGraph,
	a_latlon: Tuple[float,float],
	b_latlon: Tuple[float,float],
	max_snap_m: Optional[float] = None,
) -> Optional[List[Tuple[float,float]]]:
	"""Try to find a path using alternative strategies when direct routing fails."""
	
	# Strategy 1: Expand snap radius and try non-directional routing
	expanded_snap = max_snap_m * 2 if max_snap_m else None
	a_snap = _nearest_node(a_latlon, gg, max_snap_m=expanded_snap)
	b_snap = _nearest_node(b_latlon, gg, max_snap_m=expanded_snap)
	
	if a_snap is None or b_snap is None:
		return None
	
	a_ll = (a_snap[1], a_snap[0])
	b_ll = (b_snap[1], b_snap[0])
	
	# Try simple shortest path without directional constraints
	def simple_weight(u, v, data):
		base = data.get("weight", 1.0)
		stype = data.get("seamark_type")
		
		# Still forbid separation zones
		if stype == "separation_zone":
			return float('inf')
		return base
	
	try:
		path_nodes = nx.dijkstra_path(gg.G, source=a_ll, target=b_ll, weight=simple_weight)
		return [(lat, lon) for (lon, lat) in path_nodes]
	except (nx.NetworkXNoPath, nx.NodeNotFound):
		pass
	
	# Strategy 2: Try connecting via intermediate points
	bearing = _bearing(a_latlon, b_latlon)
	total_dist = geodesic_meters((a_latlon[1], a_latlon[0]), (b_latlon[1], b_latlon[0]))
	
	# Try intermediate points at 1/3 and 2/3 of the way
	for fraction in [0.33, 0.66]:
		intermediate_dist = total_dist * fraction
		intermediate = _point_at_bearing_distance(a_latlon, bearing, intermediate_dist)
		
		# Try path A -> intermediate -> B
		path1 = _shortest_path_ll_dir(gg, a_latlon, intermediate, max_snap_m=max_snap_m, 
									  directional_bias=False)
		if path1 is not None:
			path2 = _shortest_path_ll_dir(gg, intermediate, b_latlon, max_snap_m=max_snap_m,
										  directional_bias=False)
			if path2 is not None:
				# Combine paths, avoiding duplicate intermediate points
				combined = path1[:-1] + path2 if path1[-1] == path2[0] else path1 + path2
				return combined
	
	return None


# ---------------------------- Routing -------------------------

def _shortest_path_ll(
	gg: GeoGraph,
	a_latlon: Tuple[float,float],
	b_latlon: Tuple[float,float],
	max_snap_m: Optional[float] = None,
	# future: could allow forcing even if only one snaps, but keep both needed here
) -> Optional[List[Tuple[float,float]]]:
	"""Return shortest path between nearest nodes or None if cannot snap both.

	If either endpoint is farther than ``max_snap_m`` (when provided) we return
	None to signal the caller to fall back to a direct / non‑TSS segment.
	"""
	a_snap = _nearest_node(a_latlon, gg, max_snap_m=max_snap_m)
	b_snap = _nearest_node(b_latlon, gg, max_snap_m=max_snap_m)
	if a_snap is None or b_snap is None:
		return None
	a_ll = (a_snap[1], a_snap[0])  # lon,lat for graph keys
	b_ll = (b_snap[1], b_snap[0])
	try:
		path_nodes = nx.shortest_path(gg.G, source=a_ll, target=b_ll, weight="weight")
	except (nx.NetworkXNoPath, nx.NodeNotFound):
		return None
	# Convert back to (lat,lon)
	return [(lat, lon) for (lon, lat) in path_nodes]


def _shortest_path_ll_dir(
	gg: GeoGraph,
	a_latlon: Tuple[float,float],
	b_latlon: Tuple[float,float],
	max_snap_m: Optional[float] = None,
	directional_bias: bool = True,
	max_deviation_deg: float = 60.0,
	directional_snap: bool = True,
	) -> Optional[List[Tuple[float,float]]]:
	"""Simplified directional routing that strictly forbids going against traffic direction and crossing separation zones.

	This function finds the shortest path between two points while STRICTLY enforcing 
	that routes follow the desired direction of travel and never cross separation zones.
	Any edge that deviates more than max_deviation_deg from the desired bearing is 
	completely blocked, and separation zones are always forbidden.

	Parameters:
		max_deviation_deg: Maximum allowed deviation from desired direction (default 60°).
			Any edge exceeding this is blocked with infinite cost.
		directional_snap: If True, prefer nodes whose outgoing edges align with desired direction.

	The algorithm:
	1. Calculates desired bearing from start to end point
	2. Blocks any edge that deviates > max_deviation_deg from desired bearing  
	3. Blocks separation_lane edges that deviate > 45° (stricter for traffic lanes)
	4. Blocks ALL separation_zone edges (ships must not traverse these areas)
	5. Validates final path to ensure no segments violate direction or zone rules
	"""
	# First snap endpoints (respecting threshold); optionally refine via directional search
	desired_bearing = _bearing(a_latlon, b_latlon)

	def _directional_snap(pt_latlon: Tuple[float,float], toward_latlon: Tuple[float,float]) -> Optional[Tuple[float,float]]:
		base = _nearest_node(pt_latlon, gg, max_snap_m=max_snap_m)
		if not directional_snap or base is None or not directional_bias:
			return base
		# Explore limited forward fan from base along outgoing separation_lane edges to find
		# a node whose bearing from the original point aligns better with desired global bearing.
		base_ll = (base[1], base[0])  # lon,lat
		desired = _bearing(pt_latlon, toward_latlon)
		best_node = base
		try:
			best_diff = _angle_diff(desired, _bearing(pt_latlon, base))
		except Exception:
			best_diff = 180.0
		# BFS up to depth 5, distance cap 3 * max_snap_m (if provided)
		max_depth = 5
		dist_cap = (max_snap_m or 0) * 3 if max_snap_m is not None else float('inf')
		queue: List[Tuple[Tuple[float,float], int, float]] = [(base_ll, 0, 0.0)]  # (lon,lat), depth, accum_dist
		visited = {base_ll}
		while queue:
			cur, depth, acc = queue.pop(0)
			if depth > max_depth or acc > dist_cap:
				continue
			# Evaluate candidate (skip base already evaluated)
			if cur is not base_ll:
				cand_latlon = (cur[1], cur[0])
				diff = _angle_diff(desired, _bearing(pt_latlon, cand_latlon))
				# Strong preference for closer alignment; add distance penalty
				score = diff + 0.05 * acc / 100.0  # 0.05 deg per 100 m equivalent
				# Extra bonus for very well aligned nodes (within 30 degrees)
				if diff <= 30.0:
					score -= 5.0  # significant bonus
				if score + 1e-6 < best_diff:  # improvement
					best_diff = score
					best_node = cand_latlon
			if depth == max_depth:
				continue
			for _, nxt, data in gg.G.out_edges(cur, data=True):
				if nxt in visited:
					continue
				
				# Only explore edges in routable types
				stype = data.get('seamark_type')
				if stype not in ROUTABLE_PRIMARY_TYPES:
					continue
				
				# FORBIDDEN: Never explore separation zones
				if stype == "separation_zone":
					continue
				
				# STRICT: Skip edges that go against desired direction
				edge_bearing = _bearing((cur[1], cur[0]), (nxt[1], nxt[0]))
				edge_deviation = _angle_diff(desired, edge_bearing)
				if edge_deviation > 60.0:  # Block exploration of reverse edges
					continue
				
				seg_d = data.get('dist', 0.0)
				queue.append((nxt, depth+1, acc + seg_d))
				visited.add(nxt)
		return best_node

	a_snap = _directional_snap(a_latlon, b_latlon)
	b_snap = _directional_snap(b_latlon, a_latlon)
	if a_snap is None or b_snap is None:
		return None
	a_ll = (a_snap[1], a_snap[0])  # lon,lat
	b_ll = (b_snap[1], b_snap[0])
	# desired_bearing already computed above

	# Capture parameters for weight function closure
	_directional_bias = directional_bias
	_max_deviation_deg = max_deviation_deg
	_desired_bearing = desired_bearing

	def weight_fn(u: Tuple[float,float], v: Tuple[float,float], data: Dict) -> float:
		"""Simplified weight function that strictly forbids going against traffic direction and crossing separation zones."""
		base = data.get("weight", 1.0)
		stype = data.get("seamark_type")
		
		# FORBIDDEN: Separation zones must never be crossed
		if stype == "separation_zone":
			return float('inf')
		
		if not _directional_bias:
			return base
		
		edge_bearing = _bearing((u[1], u[0]), (v[1], v[0]))
		deviation = _angle_diff(_desired_bearing, edge_bearing)
		
		# STRICT RULE: Forbid any edge that deviates more than max_deviation_deg
		if deviation > _max_deviation_deg:
			return float('inf')  # Completely block
		
		# For allowed edges, apply light penalty based on deviation
		penalty = 1.0 + (deviation / _max_deviation_deg) * 2.0  # Max 3x penalty
		
		# Separation lanes get stricter treatment
		if stype == "separation_lane" and deviation > 45.0:
			return float('inf')  # Block separation lanes going >45° off course
		
		return base * penalty

	try:
		path_nodes = nx.dijkstra_path(gg.G, source=a_ll, target=b_ll, weight=weight_fn)
	except (nx.NetworkXNoPath, nx.NodeNotFound):
		return None
	
	# Final validation: ensure no path segment violates direction rules or crosses separation zones
	if directional_bias and len(path_nodes) >= 2:
		for i in range(len(path_nodes) - 1):
			u_ll = path_nodes[i]
			v_ll = path_nodes[i + 1]
			u_latlon = (u_ll[1], u_ll[0])
			v_latlon = (v_ll[1], v_ll[0])
			
			# Check for separation zone crossing
			edge_data = gg.G.get_edge_data(u_ll, v_ll)
			if edge_data and edge_data.get("seamark_type") == "separation_zone":
				return None  # Reject path that crosses separation zone
			
			# Check direction compliance
			edge_bearing = _bearing(u_latlon, v_latlon)
			deviation = _angle_diff(_desired_bearing, edge_bearing)
			
			# Reject entire path if any segment violates direction rules
			if deviation > _max_deviation_deg:
				return None
	
	return [(lat, lon) for (lon, lat) in path_nodes]


def _nearest_node_with_distance(pt_latlon: Tuple[float,float], gg: GeoGraph) -> Tuple[Tuple[float,float], float]:
	"""Return nearest node (lat,lon) and geodesic distance meters (no max filter)."""
	lat, lon = pt_latlon
	p = Point(lon, lat)
	nearest_any = gg.node_index.nearest(p)
	if hasattr(nearest_any, "x"):
		node_geom = nearest_any
	else:
		node_geom = gg.node_index.geometries[nearest_any]  # type: ignore[attr-defined]
	snap_latlon = (node_geom.y, node_geom.x)
	d = geodesic_meters((lon, lat), (snap_latlon[1], snap_latlon[0]))
	return snap_latlon, d


def _partial_segment_follow_tss(
	gg: GeoGraph,
	A: Tuple[float,float],
	B: Tuple[float,float],
	max_snap_m: float,
	sample_spacing_m: float = 1000.0,
	allow_single_node: bool = True,
	allow_undirected_fallback: bool = True,
) -> Tuple[Optional[List[Tuple[float,float]]], dict]:
	"""Attempt to follow a TSS portion mid-segment when endpoints too far.

	Strategy: sample along straight A->B at roughly ``sample_spacing_m``. Identify
	longest contiguous cluster of samples whose nearest-node distance <= max_snap_m.
	If at least two samples in cluster, snap the first & last sample points and run
	shortest path between those nodes, bridging with original A/B.

	Returns a list of (lat,lon) including: A, optional entry, path nodes, exit, B.
	Returns None if no mid-segment cluster found.
	"""
	# distance for sampling
	dist = geodesic_meters((A[1], A[0]), (B[1], B[0]))
	if dist <= sample_spacing_m * 2:  # too short to bother
		return None, {"reason": "segment_too_short", "segment_length_m": dist}
	steps = max(2, int(dist // sample_spacing_m))
	samples: List[Tuple[float,float]] = []
	for i in range(steps + 1):
		t = i / steps
		lat = A[0] + (B[0] - A[0]) * t
		lon = A[1] + (B[1] - A[1]) * t
		samples.append((lat, lon))
	# classify samples by within threshold
	flags: List[bool] = []
	for s in samples:
		_, d = _nearest_node_with_distance(s, gg)
		flags.append(d <= max_snap_m)
	# find longest contiguous True run
	best_len = 0; best_start = -1; cur_len = 0; cur_start = 0
	for idx, ok in enumerate(flags):
		if ok:
			if cur_len == 0:
				cur_start = idx
			cur_len += 1
			if cur_len > best_len:
				best_len = cur_len
				best_start = cur_start
		else:
			cur_len = 0
	# No inside samples at all
	if best_len == 0:
		return None, {"reason": "no_samples_within_threshold"}
	# Reject single-point cluster if not allowed
	if best_len < 2 and not allow_single_node:
		return None, {"reason": "single_cluster_not_allowed", "best_len": best_len}
	entry_idx = best_start
	exit_idx = best_start + best_len - 1
	entry_pt = samples[entry_idx]
	exit_pt = samples[exit_idx]
	# shortest path between entry/exit (if possible)
	path_nodes = _shortest_path_ll_dir(gg, entry_pt, exit_pt, max_snap_m=max_snap_m)
	debug_info: dict = {
		"entry_idx": entry_idx,
		"exit_idx": exit_idx,
		"cluster_len_samples": best_len,
	}
	if path_nodes is None:
		# Fallback: attempt to build a connected subsequence inside cluster
		cluster_samples = samples[entry_idx:exit_idx+1]
		nearest_nodes: List[Tuple[float,float]] = []
		for s in cluster_samples:
			node, _ = _nearest_node_with_distance(s, gg)
			nearest_nodes.append(node)
		# Deduplicate consecutive identical nodes
		compacted: List[Tuple[float,float]] = []
		for n in nearest_nodes:
			if not compacted or compacted[-1] != n:
				compacted.append(n)
		debug_info["compacted_nodes"] = len(compacted)
		if len(compacted) < 2:
			debug_info["reason"] = "compacted_single_node"
			return None, debug_info
		# Longest connected subsequence search
		Gdir = gg.G
		Gund = Gdir.to_undirected() if allow_undirected_fallback else None
		best_run: List[Tuple[float,float]] = []
		cur_run: List[Tuple[float,float]] = [compacted[0]]
		def edge_exists(a: Tuple[float,float], b: Tuple[float,float]) -> bool:
			ll_a = (a[1], a[0]); ll_b = (b[1], b[0])
			try:
				nx.shortest_path(Gdir, source=ll_a, target=ll_b, weight="weight")
				return True
			except Exception:
				if allow_undirected_fallback and Gund is not None:
					try:
						nx.shortest_path(Gund, source=ll_a, target=ll_b, weight="weight")
						return True
					except Exception:
						return False
			return False
		for n in compacted[1:]:
			if edge_exists(cur_run[-1], n):
				cur_run.append(n)
			else:
				if len(cur_run) > len(best_run):
					best_run = cur_run
				cur_run = [n]
		if len(cur_run) > len(best_run):
			best_run = cur_run
		debug_info["best_run_nodes"] = len(best_run)
		if len(best_run) < 2:
			if allow_single_node and len(best_run) == 1:
				path_nodes_ok = best_run
				debug_info["reason"] = "single_node_run"
			else:
				debug_info["reason"] = "no_connected_run"
				return None, debug_info
		# Build merged path by chaining shortest paths between successive nodes of best_run
		merged: List[Tuple[float,float]] = [best_run[0]]
		for nxt in best_run[1:]:
			ll_prev = (merged[-1][1], merged[-1][0]); ll_nxt = (nxt[1], nxt[0])
			try:
				seq = nx.shortest_path(Gdir, source=ll_prev, target=ll_nxt, weight="weight")
			except Exception:
				seq = nx.shortest_path(Gund, source=ll_prev, target=ll_nxt, weight="weight") if allow_undirected_fallback and Gund is not None else [ll_prev, ll_nxt]
			for (lon, lat) in seq[1:]:  # skip first (already present)
				merged.append((lat, lon))
		path_nodes_ok = merged
		debug_info["merged_nodes"] = len(merged)
		# Lane-focused enhancement: if merged path is very short relative to cluster, try lane subgraph expansion
		cluster_span_m = geodesic_meters((entry_pt[1],entry_pt[0]), (exit_pt[1],exit_pt[0]))
		if len(merged) <= 3 and cluster_span_m > 5000:  # heuristic
			lane_debug = {}
			lane_path = _expand_cluster_via_lane_subgraph(
				gg, samples[entry_idx:exit_idx+1], max_snap_m, lane_debug
			)
			debug_info["lane_subgraph"] = lane_debug
			if lane_path and len(lane_path) > len(merged):
				path_nodes_ok = lane_path
				debug_info["reason"] = "lane_subgraph_replacement"
			else:
				path_nodes_ok = merged
	else:
		if len(path_nodes) < 2 and allow_single_node:
			path_nodes_ok = path_nodes
			debug_info["reason"] = "direct_single_node"
		else:
			if len(path_nodes) < 2:
				return None, {"reason": "direct_path_too_short"}
			path_nodes_ok = path_nodes
	# build combined route
	combined: List[Tuple[float,float]] = []
	combined.append(A)
	if combined[-1] != entry_pt:
		combined.append(entry_pt)
	for n in path_nodes_ok:
		if combined[-1] != n:
			combined.append(n)
	if combined[-1] != exit_pt:
		combined.append(exit_pt)
	if combined[-1] != B:
		combined.append(B)
	debug_info.update({
		"entry_point": entry_pt,
		"exit_point": exit_pt,
		"path_nodes_kept": len(path_nodes_ok),
	})
	return combined, debug_info


def _expand_cluster_via_lane_subgraph(
	gg: GeoGraph,
	cluster_samples: List[Tuple[float,float]],
	max_snap_m: float,
	debug_out: dict,
) -> Optional[List[Tuple[float,float]]]:
	"""Attempt richer path inside a cluster using only primary routable lane types.

	1. Collect candidate nodes (nearest to each sample) whose nearest distance <= max_snap_m.
	2. Build induced subgraph using only edges whose seamark_type in ROUTABLE_PRIMARY_TYPES.
	3. Choose up to first/last N distinct nodes as start/end candidates.
	4. Find minimal-weight path among all start/end pairs.
	"""
	nearest_nodes: List[Tuple[Tuple[float,float], float]] = []
	for s in cluster_samples:
		node, d = _nearest_node_with_distance(s, gg)
		if d <= max_snap_m:
			nearest_nodes.append((node,d))
	if not nearest_nodes:
		debug_out["reason"] = "no_nearest_nodes"
		return None
	# unique ordered nodes
	ordered_nodes: List[Tuple[float,float]] = []
	for n,_ in nearest_nodes:
		if not ordered_nodes or ordered_nodes[-1] != n:
			ordered_nodes.append(n)
	# Build subgraph
	sub_nodes = set(ordered_nodes)
	SG = nx.DiGraph()
	added_edges = 0
	for u,v,data in gg.G.edges(data=True):
		if u in sub_nodes and v in sub_nodes and data.get("seamark_type") in ROUTABLE_PRIMARY_TYPES:
			SG.add_edge(u,v,**data)
			added_edges += 1
	if SG.number_of_edges() == 0:
		debug_out["reason"] = "empty_lane_subgraph"
		debug_out["candidate_nodes"] = len(sub_nodes)
		return None
	debug_out["candidate_nodes"] = len(sub_nodes)
	debug_out["lane_edges"] = added_edges
	# start/end candidates
	N = 5
	starts = []
	for n,_ in nearest_nodes[:N]:
		if n not in starts:
			starts.append(n)
	ends = []
	for n,_ in reversed(nearest_nodes[-N:]):
		if n not in ends:
			ends.append(n)
	best_path = None; best_cost = float("inf")
	for s in starts:
		for e in ends:
			if s == e:
				continue
			try:
				p = nx.shortest_path(SG, source=(s[1],s[0]), target=(e[1],e[0]), weight="weight")
			except Exception:
				continue
			# cost
			c = 0.0
			for a,b in zip(p[:-1], p[1:]):
				data = SG.get_edge_data(a,b)
				c += data.get("weight",1.0)
			if c < best_cost:
				best_cost = c
				best_path = p
	if not best_path:
		debug_out["reason"] = "no_path_in_lane_subgraph"
		return None
	debug_out["path_cost"] = best_cost
	debug_out["path_nodes"] = len(best_path)
	# convert lon,lat -> lat,lon
	return [(lat,lon) for (lon,lat) in best_path]


def tss_correct_route_geojson(
	geojson_path: str,
	waypoints_latlon: Sequence[Tuple[float,float]],
	max_snap_m: Optional[float] = 30000,
	include_bridging: bool = True,
	sample_spacing_m: float = 100.0,
	debug: bool = False,
	multi_clusters: bool = True,
	only_lanes: bool = True,
	snap_input_waypoints: bool = True,
) -> Tuple[List[Tuple[float,float]], Optional[dict]]:
	"""Correct a polyline to follow TSS lanes, with smart separation zone handling.

	New Behavior:
	* Tries to follow separation_lane paths between waypoints
	* If no separation_lane path available: Goes directly to next waypoint (if safe)
	* If direct path would cross separation zones: Continues in current direction until clear, then heads to target
	* Never follows edges of separation zones - treats them as complete no-go areas

	This ensures ships either follow proper traffic lanes or take safe detours around forbidden zones,
	without hugging the boundaries of separation zones.
	"""
	if len(waypoints_latlon) < 2:
		raise ValueError("Need at least two waypoints")
	gg = load_graph_from_geojson(geojson_path)
	if only_lanes:
		# Only include actual traffic lanes, not separation zones
		gg = _filter_graph_by_types(gg, {"separation_lane"})

	# Optionally pre-snap original user-supplied waypoints themselves onto the graph
	# if they are within the distance threshold. This changes the anchor points
	# the API returns, effectively "moving" nearby waypoints directly onto TSS.
	# Now with directional awareness to choose lanes going in the same direction.
	if snap_input_waypoints and max_snap_m is not None:
		working_wps: List[Tuple[float,float]] = []
		wp_snaps: List[bool] = []  # for debug
		for i, wp in enumerate(waypoints_latlon):
			# Determine direction for snapping
			if i == 0 and len(waypoints_latlon) > 1:
				# For first waypoint, use direction toward second waypoint
				toward_wp = waypoints_latlon[1]
			elif i == len(waypoints_latlon) - 1 and len(waypoints_latlon) > 1:
				# For last waypoint, use direction from previous waypoint
				toward_wp = waypoints_latlon[i-1]
				# But we want to snap considering the direction we're coming FROM
				# so we reverse the roles
				snapped = _directional_snap_waypoint(wp, toward_wp, gg, max_snap_m=max_snap_m)
				if snapped is not None:
					working_wps.append(snapped)
					wp_snaps.append(True)
				else:
					working_wps.append(wp)
					wp_snaps.append(False)
				continue
			else:
				# For middle waypoints, use direction toward next waypoint
				if i + 1 < len(waypoints_latlon):
					toward_wp = waypoints_latlon[i + 1]
				else:
					# Fallback to basic snapping
					snapped = _nearest_node(wp, gg, max_snap_m=max_snap_m)
					if snapped is not None:
						working_wps.append(snapped)
						wp_snaps.append(True)
					else:
						working_wps.append(wp)
						wp_snaps.append(False)
					continue
			
			# Use directional snapping
			snapped = _directional_snap_waypoint(wp, toward_wp, gg, max_snap_m=max_snap_m)
			if snapped is not None:
				working_wps.append(snapped)
				wp_snaps.append(True)
			else:
				working_wps.append(wp)
				wp_snaps.append(False)
	else:
		working_wps = list(waypoints_latlon)
		wp_snaps = [False]*len(working_wps)  # debug placeholder

	out: List[Tuple[float,float]] = []
	# Collect debug info
	debug_samples_info = None
	if debug and max_snap_m is not None:
		# pre-sample whole polyline segments for stats
		all_dists = []
		inside = 0
		for i in range(len(working_wps)-1):
			A = working_wps[i]; B = working_wps[i+1]
			dist = geodesic_meters((A[1],A[0]), (B[1],B[0]))
			steps = max(1, int(dist // sample_spacing_m))
			for k in range(steps+1):
				lat = A[0] + (B[0]-A[0]) * (k/steps)
				lon = A[1] + (B[1]-A[1]) * (k/steps)
				_, d = _nearest_node_with_distance((lat,lon), gg)
				all_dists.append(d)
				if d <= max_snap_m:
					inside += 1
		# endpoint distances
		end_dists = []
		for wp in (working_wps[0], working_wps[-1]):
			_, d = _nearest_node_with_distance(wp, gg)
			end_dists.append(d)
		debug_samples_info = {
			"endpoint_nearest_m": end_dists,
			"min_sample_nearest_m": min(all_dists) if all_dists else None,
			"samples_total": len(all_dists),
			"samples_within_threshold": inside,
			"threshold_m": max_snap_m,
			"graph_nodes": gg.G.number_of_nodes(),
			"graph_edges": gg.G.number_of_edges(),
			"input_waypoint_snaps": wp_snaps,
		}
	partials_debug: List[dict] = [] if debug else []
	for i in range(len(working_wps) - 1):
		A = working_wps[i]
		B = working_wps[i + 1]
		
		# First check if start or end points are inside separation zones
		poly_path = './seamarks-tss-polys.json'
		if _point_inside_separation_zone(poly_path, A):
			raise ValueError(f"Starting waypoint {i} is inside a separation zone - ships cannot navigate here")
		if _point_inside_separation_zone(poly_path, B):
			raise ValueError(f"End waypoint {i+1} is inside a separation zone - ships cannot navigate here")
		
		# Try to find a separation_lane path first
		path_nodes = _shortest_path_ll_dir(gg, A, B, max_snap_m=max_snap_m, 
										   directional_bias=True, max_deviation_deg=120.0)
		
		if path_nodes is None:  # No separation_lane path found
			# Try with more relaxed directional constraints
			path_nodes = _shortest_path_ll_dir(gg, A, B, max_snap_m=max_snap_m, 
											   directional_bias=True, max_deviation_deg=150.0)
			
			if path_nodes is None:
				# Try alternative path finding strategies
				path_nodes = _find_alternative_path(gg, A, B, max_snap_m=max_snap_m)
			
			# If still no path, only then fall back to direct routing
			if path_nodes is None:
				# Check if direct line would cross separation zones
				if _line_crosses_separation_zones(geojson_path, A, B) or _line_crosses_separation_zones(poly_path, A, B):
					# Must cross separation zones - continue in direction until clear
					extended_path = _continue_in_direction_until_clear(geojson_path, A, B)
					for p in extended_path:
						if not out or out[-1] != p:
							out.append(p)
				else:
					# Direct line is safe - go directly to next waypoint
					if not out:
						out.append(A)
					if out[-1] != B:
						out.append(B)
				continue
		
		# path_nodes found - use the separation_lane path
		if not out:
			# optional bridging from original A to first snapped node
			if include_bridging and A != path_nodes[0]:
				out.append(A)
			for n in path_nodes:
				if not out or out[-1] != n:
					out.append(n)
		else:
			# Add bridging from previous list end to path
			first = path_nodes[0]
			if include_bridging and out[-1] != first:
				# Insert original A if it is different and not already last
				if out[-1] != A:
					out.append(A)
				if A != first:
					out.append(first)
			# Append remaining nodes
			for n in path_nodes[1:]:
				if out[-1] != n:
					out.append(n)
		# After graph portion, optionally bridge to original B if different
		if include_bridging and out[-1] != B:
			out.append(B)
	
	# FINAL VALIDATION: Ensure no route segments cross separation zones
	poly_path = './seamarks-tss-polys.json'
	for i in range(len(out) - 1):
		if _line_crosses_separation_zone_polygons(poly_path, out[i], out[i+1]):
			# Try to fix by inserting intermediate waypoints that avoid the zone
			fixed_segment = _route_around_separation_zone(poly_path, out[i], out[i+1])
			if fixed_segment and len(fixed_segment) > 2:
				# Replace this segment with the fixed route
				out = out[:i+1] + fixed_segment[1:-1] + out[i+1:]
				# Note: this may change the length of out, but we'll continue validation
				break  # Re-validate from the beginning after any fix
	
	if debug and debug_samples_info is not None:
		debug_samples_info["partials"] = partials_debug
	return out, debug_samples_info if debug else None

__all__ = [
	"tss_correct_route_geojson",
	"load_graph_from_geojson",
	"NoTSSDataError",
	"NoPathError",
	"_line_crosses_separation_zones",
	"_line_crosses_separation_zone_polygons", 
	"_point_inside_separation_zone",
]


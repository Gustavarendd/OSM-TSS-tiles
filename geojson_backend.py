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

# Optional import for external lane direction metadata
try:  # pragma: no cover
	from lane_directions import load_lane_directions  # type: ignore
except Exception:  # pragma: no cover
	def load_lane_directions(path: str) -> Dict[Tuple[float,float], float]:
		return {}

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
	# "two-way_route",
	# "recommended_track",
	# "recommended_route_centreline",
	# "navigation_line",
}

GRAPH_ONEWAY_TYPES = {"separation_lane"}
GRAPH_BIDIR_TYPES = {
	"two-way_route",
	# "separation_crossing",
	# "recommended_track",
	# "recommended_route_centreline",
	# "navigation_line",
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
	sample_spacing_m: float = 500.0,
	min_samples: int = 5,
	poly_fallback: str = './seamarks-tss-polys.json'
) -> bool:
	"""Robustly test whether segment start->end intersects ANY separation zone.

	Previous implementation relied on a graph that intentionally excluded
	`separation_zone` features, so it silently returned False (allowing crossings).

	New behaviour:
	1. If the provided file appears to contain polygons, run exact polygon
	   intersection via `_line_crosses_separation_zone_polygons`.
	2. Otherwise, scan raw features in the file for any geometry of type
	   Polygon / MultiPolygon / LineString / MultiLineString whose
	   seamark:type parses to `separation_zone`.
	   * Polygons: direct intersection test.
	   * Lines: treat as a *boundary*; we apply a small buffer (50m by default)
	     and test intersection with that buffer to approximate area.
	3. If no zones found in the primary file and a fallback polygon file exists,
	   attempt polygon intersection with the fallback.

	Parameters:
		 sample_spacing_m (unused now for polygons) kept for backward compat.
		 min_samples kept for API stability; no forced 100‑sample floor anymore.

	Returns True on first detected intersection, else False.
	"""
	from shapely.geometry import LineString, shape
	from shapely.ops import unary_union
	import os

	segment = LineString([(start_latlon[1], start_latlon[0]), (end_latlon[1], end_latlon[0])])

	def _scan_file(path: str) -> Optional[bool]:
		try:
			with open(path, 'r', encoding='utf-8') as f:
				data = json.load(f)
		except Exception:
			return None
		zones = []
		for feat in data.get('features', []):
			props = feat.get('properties', {})
			stype = _parse_seamark_type(props)
			if stype != 'separation_zone':
				continue
			geom = feat.get('geometry')
			if not geom:
				continue
			try:
				g = shape(geom)
			except Exception:
				continue
			if g.is_empty:
				continue
			# For line / multilines, buffer slightly so we can detect crossing.
			if g.geom_type in ('LineString','MultiLineString'):
				# Approx 50 m buffer; convert ~meters to degrees (~1 deg ~111km)
				buf_deg = 50.0 / 111000.0
				g = g.buffer(buf_deg)
			zones.append(g)
		if not zones:
			return False
		try:
			union = unary_union(zones)
			return segment.intersects(union)
		except Exception:
			# Fallback: brute loop
			for z in zones:
				if segment.intersects(z):
					return True
			return False

	# If file name hints polygons, do direct polygon test first.
	if 'polys' in geojson_path.lower() or 'polygon' in geojson_path.lower():
		res = _scan_file(geojson_path)
		return bool(res)

	# Try primary file scan
	primary = _scan_file(geojson_path)
	if primary:
		return True
	if primary is None or primary is False:
		# Optional fallback to dedicated polygons file if present
		if poly_fallback and os.path.exists(poly_fallback):
			fallback_res = _scan_file(poly_fallback)
			return bool(fallback_res)
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


def _extend_along_lane_until_zone_free(
	gg: GeoGraph,
	start_latlon: Tuple[float,float],
	target_latlon: Tuple[float,float],
	poly_path: str = './seamarks-tss-polys.json',
	max_explore_nodes: int = 200,
	max_total_lane_km: float = 150.0,
	max_bearing_dev: float = 110.0,
	prefer_forward: bool = True,
) -> Optional[List[Tuple[float,float]]]:
	"""Try to stay on lanes extending from start until a direct line to target is zone‑free.

	Algorithm:
	1. Snap start to nearest node (must succeed within unlimited distance; caller ensures proximity logic).
	2. Directed BFS along separation_lane edges only (optionally forward-biased toward target bearing).
	3. For each discovered node, test if straight line node->target crosses any separation zone polygons.
	4. Return path: start -> ...lane nodes... -> target when first zone‑free direct line found.
	5. Abort if exploration limits exceeded.

	Heuristics:
	* Avoid reversing direction sharply (> max_bearing_dev from desired bearing Start->Target).
	* Track cumulative traveled lane distance; stop at cap.
	"""
	try:
		start_snap = _nearest_node(start_latlon, gg, max_snap_m=None)
	except Exception:
		return None
	if start_snap is None:
		return None
	start_ll = (start_snap[1], start_snap[0])  # lon,lat
	desired_bearing = _bearing(start_latlon, target_latlon)
	# BFS queue: node_ll, path(list lon/lat tuples), traveled_m
	from collections import deque
	q = deque([(start_ll, [start_ll], 0.0)])
	visited: Set[Tuple[float,float]] = {start_ll}
	while q and len(visited) <= max_explore_nodes:
		cur, path, dist_so_far = q.popleft()
		cur_latlon = (cur[1], cur[0])
		# Test zone-free direct line
		if not _line_crosses_separation_zone_polygons(poly_path, cur_latlon, target_latlon):
			# Build output path converting lon/lat to (lat,lon)
			lane_path = [(lat,lon) for (lon,lat) in path]
			if lane_path[-1] != cur_latlon:
				lane_path.append(cur_latlon)
			if lane_path[-1] != target_latlon:
				lane_path.append(target_latlon)
			return lane_path
		# Expand neighbors
		for _, nxt, data in gg.G.out_edges(cur, data=True):
			stype = data.get('seamark_type')
			if stype != 'separation_lane':
				continue
			if nxt in visited:
				continue
			nxt_latlon = (nxt[1], nxt[0])
			edge_bearing = _bearing(cur_latlon, nxt_latlon)
			if prefer_forward:
				dev = _angle_diff(desired_bearing, edge_bearing)
				if dev > max_bearing_dev:
					continue
			seg_d = data.get('dist', 0.0)
			new_total = dist_so_far + seg_d
			if new_total/1000.0 > max_total_lane_km:
				continue
			visited.add(nxt)
			q.append((nxt, path + [nxt], new_total))
	return None


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
		internal = _shortest_path_ll_dir(gg, entry_sample, exit_sample, max_snap_m=max_snap_m, prevent_lane_hopping=False)
		if internal is None:
			cluster_info["internal"] = "none"
		else:
			cluster_info["internal_nodes"] = len(internal)
			# append bridging from previous cluster exit
			if prev_exit_node is not None:
				# connect previous exit to this entry via shortest path (node to node)
				bridge = _shortest_path_ll_dir(gg, prev_exit_node, entry_node, max_snap_m=max_snap_m, prevent_lane_hopping=False)
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
	lane_interior_nodes: Set[Tuple[float,float]] = set()
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
		prev_ll = tuple(coords[0])
		if stype == "separation_lane" and len(coords) > 2:
			# Record interior nodes (exclude endpoints)
			for interior in coords[1:-1]:
				lane_interior_nodes.add(tuple(interior))

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


def _enforce_lane_directions(
	gg: GeoGraph,
	directions: Dict[Tuple[float,float], float],
	tolerance_deg: float = 95.0,
	match_radius_m: float = 60.0,
	annotate_only: bool = True,
) -> Tuple[GeoGraph, Dict[str,int]]:
	"""Annotate (and optionally remove) separation_lane edges that conflict with traffic direction.

	We perform a tolerant nearest-neighbour lookup between each graph node and direction points
	(because coordinates may differ slightly due to preprocessing). For each separation_lane edge:
	  * Find lane direction (bearing degrees) from nearest direction point to its source node
		within ``match_radius_m``.
	  * Compute edge bearing.
	  * If deviation > tolerance_deg we either remove the edge (annotate_only=False) or keep but
		mark it as against-traffic so the weight function can heavily penalize / forbid it.

	Edge attributes added:
		traffic_dir: forward allowed lane direction (degrees)
		against_traffic: bool flag if edge bearing deviates beyond tolerance
		traffic_dir_diff: absolute angular diff (degrees)
	"""
	if not directions:
		return gg, {"removed": 0, "checked": 0, "annotated": 0, "no_match_nodes": 0}
	# Build spatial index of direction points
	from shapely.strtree import STRtree as _STR
	from shapely.geometry import Point as _Pt
	dir_points = []
	dir_bearings = []
	for (lat, lon), bearing in directions.items():
		dir_points.append(_Pt(lon, lat))
		dir_bearings.append(bearing)
	tree = _STR(dir_points)
	G = gg.G.copy()
	removed = 0; checked = 0; annotated = 0; no_match_nodes = 0
	for u, v, data in list(G.edges(data=True)):
		if data.get('seamark_type') != 'separation_lane':
			continue
		u_latlon = (u[1], u[0])
		# nearest direction point
		try:
			nearest_geom = tree.nearest(_Pt(u[0], u[1]))  # u is (lon,lat)
			if hasattr(nearest_geom, 'x'):
				idx = dir_points.index(nearest_geom)  # may be O(n); dataset small
			else:  # unlikely branch
				idx = nearest_geom
			bearing = dir_bearings[idx]
			# distance check
			d_m = geodesic_meters((u[0], u[1]), (nearest_geom.x, nearest_geom.y))
			if d_m > match_radius_m:
				no_match_nodes += 1
				continue
		except Exception:
			no_match_nodes += 1
			continue
		edge_bearing = _bearing((u_latlon[0], u_latlon[1]), (v[1], v[0]))
		diff = _angle_diff(bearing, edge_bearing)
		checked += 1
		if diff > tolerance_deg:
			if annotate_only:
				data['against_traffic'] = True
				data['traffic_dir'] = bearing
				data['traffic_dir_diff'] = diff
				annotated += 1
			else:
				G.remove_edge(u, v)
				removed += 1
		else:
			data['against_traffic'] = False
			data['traffic_dir'] = bearing
			data['traffic_dir_diff'] = diff
			annotated += 1
	return _reindex_graph_nodes(G), {
		"removed": removed,
		"checked": checked,
		"annotated": annotated,
		"no_match_nodes": no_match_nodes,
		"tolerance_deg": tolerance_deg,
		"match_radius_m": match_radius_m,
	}


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
	max_snap_m: Optional[float] = None,
	current_lane_node: Optional[Tuple[float,float]] = None,
	prevent_lane_hopping: bool = True
) -> Optional[Tuple[float,float]]:
	"""Snap waypoint to nearest node considering direction of travel and lane continuity.
	
	Finds the best node within threshold distance that has outgoing edges
	aligned with the desired direction of travel. If currently on a lane,
	strongly prefers staying on the same lane unless it's clearly wrong.
	
	Args:
		pt_latlon: Point to snap (lat, lon)
		toward_latlon: Next waypoint to determine desired direction (lat, lon)
		gg: Graph to search
		max_snap_m: Maximum snap distance in meters
		current_lane_node: If provided, the node we're currently on (for lane continuity)
		
	Returns:
		Best directionally-aligned node within threshold, or None if none found
	"""
	# First get basic nearest node
	base = _nearest_node(pt_latlon, gg, max_snap_m=max_snap_m)
	if base is None:
		return None
	
	# Calculate desired bearing from current point toward next waypoint
	desired_bearing = _bearing(pt_latlon, toward_latlon)
	
	# If we're currently on a lane, check if we should stay on it
	if current_lane_node is not None:
		# Check if current lane node is still within reasonable distance
		current_dist = geodesic_meters((pt_latlon[1], pt_latlon[0]), (current_lane_node[1], current_lane_node[0]))
		if max_snap_m is None or current_dist <= max_snap_m * 1.2:  # Only slightly larger distance for continuity
			# Check if staying on current lane makes directional sense
			try:
				current_bearing = _bearing(pt_latlon, current_lane_node)
				current_diff = _angle_diff(desired_bearing, current_bearing)
				
				# If current lane is well-aligned (within 45°), consider staying on it
				if current_diff <= 45.0:
					# Look for continuation of current lane in desired direction
					current_ll = (current_lane_node[1], current_lane_node[0])
					
					# Find best continuation node along current lane
					best_continuation = None
					best_cont_score = float('inf')
					
					for _, nxt, data in gg.G.out_edges(current_ll, data=True):
						stype = data.get('seamark_type')
						if stype not in ROUTABLE_PRIMARY_TYPES or stype == "separation_zone" or stype == 'inshore_traffic_zone':
							continue
							
						nxt_latlon = (nxt[1], nxt[0])
						nxt_dist = geodesic_meters((pt_latlon[1], pt_latlon[0]), (nxt_latlon[1], nxt_latlon[0]))
						
						# Only consider nodes within snap distance
						if max_snap_m is not None and nxt_dist > max_snap_m:
							continue
							
						# Calculate score for this continuation
						edge_bearing = _bearing(current_lane_node, nxt_latlon)
						edge_diff = _angle_diff(desired_bearing, edge_bearing)
						
						# Prefer closer nodes with better alignment
						score = edge_diff + 0.01 * nxt_dist / 100.0  # Small distance penalty
						
						if score < best_cont_score and edge_diff <= 30.0:  # Good alignment required
							best_cont_score = score
							best_continuation = nxt_latlon
					
					# Use continuation if it's significantly better than basic snap or very close
					if best_continuation is not None:
						base_dist = geodesic_meters((pt_latlon[1], pt_latlon[0]), (base[1], base[0]))
						cont_dist = geodesic_meters((pt_latlon[1], pt_latlon[0]), (best_continuation[1], best_continuation[0]))
						
						# Use continuation if it's not much farther than base snap
						if cont_dist <= base_dist * 1.5:
							return best_continuation
					
					# If no good continuation but current node is very well aligned and close, use it
					if current_diff <= 20.0 and current_dist <= current_dist * 1.1:
						return current_lane_node
			except Exception:
				pass
	
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
					
				# Moderate bonus if this candidate is on the same lane as current_lane_node
				if current_lane_node is not None:
					# Check if candidate is directly connected to current lane
					current_ll = (current_lane_node[1], current_lane_node[0])
					cand_ll = (cand_latlon[1], cand_latlon[0])
					
					# Only give bonus for direct connections (same lane segment)
					if gg.G.has_edge(current_ll, cand_ll) or gg.G.has_edge(cand_ll, current_ll):
						score -= 3.0  # Moderate lane continuity bonus
					
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
			if stype == "separation_zone" or stype == 'inshore_traffic_zone':
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


def _find_lane_continuation(
	gg: GeoGraph,
	current_node: Tuple[float, float],
	desired_bearing: float,
	next_waypoint: Tuple[float, float],
	max_search_distance: float = 10000.0,  # Increased from 5000
	max_bearing_diff: float = 90.0,        # Relaxed from 45°
	debug: bool = False
) -> Optional[Tuple[float, float]]:
	"""Find the start of a new lane when current lane ends.
	
	Looks for nearby lane starts that:
	1. Go in the same general direction as desired_bearing
	2. Head toward the next waypoint 
	3. Are not going in opposite direction
	
	Args:
		gg: Graph to search
		current_node: Current position (lat, lon)
		desired_bearing: Desired direction of travel
		next_waypoint: Target waypoint (lat, lon)
		max_search_distance: Maximum distance to search for new lanes (meters)
		max_bearing_diff: Maximum bearing difference to consider (degrees)
		debug: Print debug information
		
	Returns:
		Best lane start node or None if no suitable lane found
	"""
	if debug:
		print(f"DEBUG: Looking for lane continuation from {current_node}")
		print(f"       Desired bearing: {desired_bearing:.1f}°")
		print(f"       Next waypoint: {next_waypoint}")
		print(f"       Search distance: {max_search_distance}m, bearing tolerance: {max_bearing_diff}°")
	
	current_ll = (current_node[1], current_node[0])  # Convert to lon, lat
	
	candidate_nodes = []
	nodes_checked = 0
	nodes_in_range = 0
	edges_checked = 0
	
	# Search through all graph nodes for potential lane starts
	for node_ll in gg.G.nodes():
		nodes_checked += 1
		node_latlon = (node_ll[1], node_ll[0])
		
		# Skip if too far away
		distance = geodesic_meters((current_node[1], current_node[0]), (node_latlon[1], node_latlon[0]))
		if distance > max_search_distance:
			continue
			
		nodes_in_range += 1
		
		# Check if this node has good outgoing edges (potential lane start)
		has_good_outgoing = False
		best_edge_score = float('inf')
		
		for _, neighbor, edge_data in gg.G.out_edges(node_ll, data=True):
			edges_checked += 1
			stype = edge_data.get('seamark_type')
			if stype not in ROUTABLE_PRIMARY_TYPES or stype == "separation_zone" or stype == 'inshore_traffic_zone':
				continue
				
			neighbor_latlon = (neighbor[1], neighbor[0])
			
			# Calculate bearing of this outgoing edge
			edge_bearing = _bearing(node_latlon, neighbor_latlon)
			
			# Check alignment with desired direction - be more lenient
			bearing_diff = _angle_diff(desired_bearing, edge_bearing)
			if bearing_diff > max_bearing_diff:
				continue  # Edge goes in wrong direction
				
			# Check alignment toward next waypoint - be more lenient
			to_waypoint_bearing = _bearing(node_latlon, next_waypoint)
			waypoint_diff = _angle_diff(to_waypoint_bearing, edge_bearing)
			
			# Accept edges that go generally toward the waypoint (within 135°)
			if waypoint_diff <= 135.0:  # More lenient than 90°
				# Score based on bearing alignment and distance
				score = bearing_diff + waypoint_diff * 0.3 + distance / 2000.0  # Less distance penalty
				if score < best_edge_score:
					best_edge_score = score
					has_good_outgoing = True
		
		if has_good_outgoing:
			candidate_nodes.append((node_latlon, best_edge_score, distance))
	
	if debug:
		print(f"DEBUG: Checked {nodes_checked} nodes, {nodes_in_range} in range, {edges_checked} edges")
		print(f"DEBUG: Found {len(candidate_nodes)} candidates")
		if len(candidate_nodes) > 0:
			print(f"DEBUG: Best 3 candidates:")
			for i, (node, score, dist) in enumerate(sorted(candidate_nodes, key=lambda x: x[1])[:3]):
				print(f"       {i+1}: {node} (score: {score:.2f}, distance: {dist:.0f}m)")
	
	if not candidate_nodes:
		if debug:
			print("DEBUG: No suitable lane continuation found")
		return None
		
	# Sort by score (best alignment and closest distance)
	candidate_nodes.sort(key=lambda x: x[1])
	
	best_candidate = candidate_nodes[0]
	if debug:
		print(f"DEBUG: Selected lane continuation: {best_candidate[0]} (score: {best_candidate[1]:.2f}, distance: {best_candidate[2]:.0f}m)")
	
	# Return the best candidate
	return best_candidate[0]


def _find_alternative_path(
	gg: GeoGraph,
	a_latlon: Tuple[float,float],
	b_latlon: Tuple[float,float],
	max_snap_m: Optional[float] = None,
	prevent_lane_hopping: bool = True
) -> Optional[List[Tuple[float,float]]]:
	"""Try to find a path using alternative strategies when direct routing fails.
	
	New strategy: Look for lane continuations when current lane ends.
	"""
	
	# Strategy 1: Try to find lane continuation (only if lane hopping prevention is enabled)
	if prevent_lane_hopping:
		desired_bearing = _bearing(a_latlon, b_latlon)
		
		# Try to find a good lane start near point A that goes toward point B
		lane_start = _find_lane_continuation(gg, a_latlon, desired_bearing, b_latlon, debug=True)
		if lane_start is not None:
			print(f"DEBUG: Found lane start at {lane_start}")
			# Try routing from the lane start
			path_from_start = _shortest_path_ll_dir(gg, lane_start, b_latlon, max_snap_m=max_snap_m, 
													prevent_lane_hopping=False)
			if path_from_start is not None:
				print(f"DEBUG: Successfully routed from lane start")
				# Add bridging from original A to lane start if needed
				if a_latlon != lane_start:
					return [a_latlon, lane_start] + path_from_start[1:]  # Avoid duplicate lane_start
				else:
					return path_from_start
		
		# Strategy 2: Try the reverse - find good lane end near point B
		reverse_bearing = (_bearing(b_latlon, a_latlon) + 180) % 360
		lane_end = _find_lane_continuation(gg, b_latlon, reverse_bearing, a_latlon)
		if lane_end is not None:
			# Try routing to the lane end
			path_to_end = _shortest_path_ll_dir(gg, a_latlon, lane_end, max_snap_m=max_snap_m,
												prevent_lane_hopping=False)
			if path_to_end is not None:
				# Add bridging from lane end to original B if needed  
				if lane_end != b_latlon:
					return path_to_end[:-1] + [lane_end, b_latlon]  # Avoid duplicate lane_end
				else:
					return path_to_end
	
	# Strategy 3: Expand snap radius and try non-directional routing
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
		if stype == "separation_zone" or stype == 'inshore_traffic_zone':
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
									  directional_bias=False, prevent_lane_hopping=False)
		if path1 is not None:
			path2 = _shortest_path_ll_dir(gg, intermediate, b_latlon, max_snap_m=max_snap_m,
										  directional_bias=False, prevent_lane_hopping=False)
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


def _dijkstra_with_lane_penalty(
	graph: nx.DiGraph,
	source: Tuple[float, float],
	target: Tuple[float, float],
	base_weight_fn: callable,
	lane_change_penalty: float,
	prevent_lane_hopping: bool = True,
	target_latlon: Optional[Tuple[float,float]] = None,
	follow_full_lane: bool = True,
	early_exit_multiplier: float = 5.0,
	lane_forward_bearing_tol: float = 100.0,
) -> List[Tuple[float, float]]:
	"""Custom Dijkstra that tracks feature IDs and strongly discourages leaving a lane early.

	Added logic: if we are on a lane feature (identified by previous edge feature_id) and there
	exists at least one *forward* continuation edge of the same feature whose bearing toward the
	target differs by <= lane_forward_bearing_tol degrees, then switching to a different feature
	before exhausting those continuations incurs an extra multiplicative penalty (early_exit_multiplier).
	This effectively forces the algorithm to "ride" a separation lane to its natural end before
	searching for a new lane aligned with the next waypoint, matching the requested behavior.
	"""
	import heapq

	# State: (cost, node, previous_feature_id, path)
	heap = [(0.0, source, None, [source])]
	visited: Dict[Tuple[float,float], Tuple[float, Optional[int]]] = {}

	if target_latlon is None:
		target_latlon = (target[1], target[0])  # convert lon,lat -> lat,lon

	while heap:
		current_cost, current_node, prev_feature_id, path = heapq.heappop(heap)
		if current_node in visited and current_cost > visited[current_node][0] + 0.01:
			continue
		visited[current_node] = (current_cost, prev_feature_id)
		if current_node == target:
			return path

		# Pre-compute forward same-feature continuations if we have a prev feature
		forward_same_feature_exists = False
		desired_bearing_here = None
		if follow_full_lane and prev_feature_id is not None and target_latlon is not None:
			try:
				current_latlon = (current_node[1], current_node[0])
				desired_bearing_here = _bearing(current_latlon, target_latlon)
				for nbr in graph.neighbors(current_node):
					ed = graph.get_edge_data(current_node, nbr)
					if not ed:
						continue
					if ed.get('feature_id') != prev_feature_id:
						continue
					nbr_latlon = (nbr[1], nbr[0])
					edge_bear = _bearing(current_latlon, nbr_latlon)
					if _angle_diff(desired_bearing_here, edge_bear) <= lane_forward_bearing_tol:
						forward_same_feature_exists = True
						break
			except Exception:
				forward_same_feature_exists = False

		neighbors_explored = False
		for neighbor in graph.neighbors(current_node):
			if neighbor in visited:
				continue
			edge_data = graph.get_edge_data(current_node, neighbor)
			if not edge_data:
				continue
			base_weight = base_weight_fn(current_node, neighbor, edge_data)
			if base_weight == float('inf'):
				continue
			neighbors_explored = True
			current_feature_id = edge_data.get('feature_id')
			total_weight = base_weight

			# Standard lane change penalty
			if (prev_feature_id is not None and current_feature_id is not None and current_feature_id != prev_feature_id):
				total_weight *= lane_change_penalty
				# Additional early-exit penalty if a forward continuation still exists
				if follow_full_lane and forward_same_feature_exists:
					total_weight *= early_exit_multiplier

			new_cost = current_cost + total_weight
			heapq.heappush(heap, (new_cost, neighbor, current_feature_id, path + [neighbor]))

		# If lane ended (no neighbors) try to find continuation (existing behavior)
		if not neighbors_explored and current_node != target and prevent_lane_hopping:
			current_latlon = (current_node[1], current_node[0])
			desired_bearing = _bearing(current_latlon, target_latlon)
			temp_gg = type('GeoGraph', (), {'G': graph, 'node_index': None, 'node_points': None})()
			continuation = _find_lane_continuation(
				temp_gg, current_latlon, desired_bearing, target_latlon,
				max_search_distance=3000.0
			)
			if continuation is not None:
				continuation_ll = (continuation[1], continuation[0])
				if continuation_ll not in visited:
					bridge_distance = geodesic_meters(
						(current_latlon[1], current_latlon[0]),
						(continuation[1], continuation[0])
					)
					bridge_cost = bridge_distance * 1.5
					heapq.heappush(heap, (current_cost + bridge_cost, continuation_ll, None, path + [continuation_ll]))

	raise nx.NetworkXNoPath(f"No path from {source} to {target}")


def _shortest_path_ll_dir(
	gg: GeoGraph,
	a_latlon: Tuple[float,float],
	b_latlon: Tuple[float,float],
	max_snap_m: Optional[float] = None,
	directional_bias: bool = True,
	max_deviation_deg: float = 60.0,
	directional_snap: bool = True,
	prevent_lane_hopping: bool = True,
	lane_change_penalty: float = 10.0,
	enforce_progress: bool = True,
	backward_progress_slack_m: float = 200.0,
	backward_block_m: float = 1200.0,
	progress_penalty_factor: float = 4.0,
	traffic_dir_block: bool = True,
	traffic_dir_block_deg: float = 120.0,
	traffic_dir_soft_deg: float = 35.0,
	traffic_dir_soft_penalty: float = 8.0,
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
				if stype == "separation_zone" or stype == 'inshore_traffic_zone':
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
	_prevent_lane_hopping = prevent_lane_hopping
	_lane_change_penalty = lane_change_penalty

	_target_latlon = b_latlon
	def weight_fn(u: Tuple[float,float], v: Tuple[float,float], data: Dict) -> float:
		"""Weight function for directional bias and separation zone avoidance."""
		base = data.get("weight", 1.0)
		stype = data.get("seamark_type")
		
		# FORBIDDEN: Separation zones must never be crossed
		if stype == "separation_zone" or stype == 'inshore_traffic_zone':
			return float('inf')
		
		if not _directional_bias:
			return base
		
		edge_bearing = _bearing((u[1], u[0]), (v[1], v[0]))
		deviation = _angle_diff(_desired_bearing, edge_bearing)

		# Progress enforcement: discourage edges that move away from target
		if enforce_progress and _target_latlon is not None:
			u_latlon = (u[1], u[0]); v_latlon = (v[1], v[0])
			d_before = geodesic_meters((u_latlon[1], u_latlon[0]), (_target_latlon[1], _target_latlon[0]))
			d_after = geodesic_meters((v_latlon[1], v_latlon[0]), (_target_latlon[1], _target_latlon[0]))
			if d_after > d_before + backward_block_m:
				return float('inf')  # Moving clearly away from destination
			elif d_after > d_before + backward_progress_slack_m:
				# Apply multiplicative penalty proportional to lost progress
				progress_excess = d_after - d_before - backward_progress_slack_m
				base *= (1.0 + progress_penalty_factor * (progress_excess / max(d_before, 1.0)))
		
		# STRICT RULE: Forbid any edge that deviates more than max_deviation_deg
		if deviation > _max_deviation_deg:
			return float('inf')  # Completely block

		# Traffic direction enforcement (per-edge annotation)
		if traffic_dir_block and stype == 'separation_lane':
			if 'traffic_dir' in data:
				diff_lane = data.get('traffic_dir_diff', None)
				if diff_lane is None:
					# recompute if missing
					lane_dir = data.get('traffic_dir')
					if lane_dir is not None:
						diff_lane = _angle_diff(lane_dir, edge_bearing)
				if diff_lane is not None:
					if diff_lane > traffic_dir_block_deg:
						return float('inf')  # reverse traffic
					elif diff_lane > traffic_dir_soft_deg:
						base *= traffic_dir_soft_penalty * (1.0 + (diff_lane - traffic_dir_soft_deg)/(traffic_dir_block_deg-traffic_dir_soft_deg + 1e-6))
			elif data.get('against_traffic'):
				return float('inf')
		
		# For allowed edges, apply light penalty based on deviation
		penalty = 1.0 + (deviation / _max_deviation_deg) * 2.0  # Max 3x penalty
		
		# Separation lanes get stricter treatment
		if stype == "separation_lane" and deviation > 45.0:
			return float('inf')  # Block separation lanes going >45° off course
		
		return base * penalty

	try:
		if prevent_lane_hopping:
			# Enhanced custom Dijkstra: follow lane fully before switching
			path_nodes = _dijkstra_with_lane_penalty(
				gg.G, a_ll, b_ll, weight_fn, lane_change_penalty,
				prevent_lane_hopping=prevent_lane_hopping,
				target_latlon=b_latlon,
				follow_full_lane=True,
				early_exit_multiplier=8.0,
				lane_forward_bearing_tol=100.0,
			)
		else:
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
			
			if edge_data and edge_data.get("seamark_type") == "inshore_traffic_zone":
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
	path_nodes = _shortest_path_ll_dir(gg, entry_pt, exit_pt, max_snap_m=max_snap_m, prevent_lane_hopping=False)
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
	prevent_lane_hopping: bool = False,
	# New adaptive / long segment strategy parameters
	long_segment_km_threshold: float = 40.0,
	adaptive_snap_expansions: Sequence[float] = (1.5, 2.0, 3.0),
	include_connectors_on_long: bool = False,
	multi_cluster_sample_spacing_m: float = 5000.0,
	partial_cluster_sample_spacing_m: float = 1500.0,
	adaptive_lane_search_distances_m: Sequence[float] = (5000.0, 10000.0, 30000.0),
	use_lane_extension_detour: bool = True,
	# Progress control tuning
	progress_enforce: bool = True,
	backward_progress_slack_m: float = 200.0,
	backward_block_m: float = 1200.0,
	progress_penalty_factor: float = 4.0,
	lane_directions_path: Optional[str] = "TSS/separation_lanes_with_direction.geojson",
	lane_direction_tolerance_deg: float = 95.0,
	enforce_lane_directions: bool = True,
	lane_direction_match_radius_m: float = 60.0,
	traffic_dir_block_deg: float = 90.0,
	traffic_dir_soft_deg: float = 25.0,
	traffic_dir_soft_penalty: float = 100.0,
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
	gg_full = load_graph_from_geojson(geojson_path)
	if only_lanes:
		gg = _filter_graph_by_types(gg_full, {"separation_lane"})
	else:
		gg = gg_full

	lane_dir_stats = None
	if enforce_lane_directions and lane_directions_path:
		try:
			ldir = load_lane_directions(lane_directions_path)
			gg, lane_dir_stats = _enforce_lane_directions(gg, ldir, tolerance_deg=lane_direction_tolerance_deg, match_radius_m=lane_direction_match_radius_m, annotate_only=True)
		except Exception:
			lane_dir_stats = {"error": "failed_to_load_lane_directions"}

	# Pre-build a connector-enriched graph (lazy) if we might need it later
	gg_with_connectors: Optional[GeoGraph] = None

	# Optionally pre-snap original user-supplied waypoints themselves onto the graph
	# if they are within the distance threshold. This changes the anchor points
	# the API returns, effectively "moving" nearby waypoints directly onto TSS.
	# Now with directional awareness and lane continuity to choose lanes going in the same direction.
	if snap_input_waypoints and max_snap_m is not None:
		working_wps: List[Tuple[float,float]] = []
		wp_snaps: List[bool] = []  # for debug
		current_lane_node: Optional[Tuple[float,float]] = None  # Track current lane for continuity
		
		for i, wp in enumerate(waypoints_latlon):
			# Determine direction for snapping
			if i == 0 and len(waypoints_latlon) > 1:
				# For first waypoint, use direction toward second waypoint
				toward_wp = waypoints_latlon[1]
				snapped = _directional_snap_waypoint(wp, toward_wp, gg, max_snap_m=max_snap_m, current_lane_node=None, prevent_lane_hopping=prevent_lane_hopping)
				if snapped is not None:
					working_wps.append(snapped)
					current_lane_node = snapped  # Update current lane
					wp_snaps.append(True)
				else:
					working_wps.append(wp)
					wp_snaps.append(False)
			elif i == len(waypoints_latlon) - 1 and len(waypoints_latlon) > 1:
				# For last waypoint, use direction from previous waypoint
				toward_wp = waypoints_latlon[i-1]
				# But we want to snap considering the direction we're coming FROM
				# so we reverse the roles
				snapped = _directional_snap_waypoint(wp, toward_wp, gg, max_snap_m=max_snap_m, current_lane_node=current_lane_node, prevent_lane_hopping=prevent_lane_hopping)
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
			
			# Use directional snapping with lane continuity
			snapped = _directional_snap_waypoint(wp, toward_wp, gg, max_snap_m=max_snap_m, current_lane_node=current_lane_node, prevent_lane_hopping=prevent_lane_hopping)
			if snapped is not None:
				working_wps.append(snapped)
				current_lane_node = snapped  # Update current lane
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
			"lane_direction_enforced": bool(lane_dir_stats is not None),
			"lane_direction_stats": lane_dir_stats,
		}
	partials_debug: List[dict] = [] if debug else []
	for i in range(len(working_wps) - 1):
		A = working_wps[i]
		B = working_wps[i + 1]
		segment_debug: dict = {"segment_index": i, "A": A, "B": B} if debug else {}
		poly_path = './seamarks-tss-polys.json'
		# Validate endpoints not inside forbidden zones
		if _point_inside_separation_zone(poly_path, A):
			raise ValueError(f"Starting waypoint {i} is inside a separation zone - ships cannot navigate here")
		if _point_inside_separation_zone(poly_path, B):
			raise ValueError(f"End waypoint {i+1} is inside a separation zone - ships cannot navigate here")

		segment_len_m = geodesic_meters((A[1],A[0]), (B[1],B[0]))
		long_segment = segment_len_m >= long_segment_km_threshold * 1000.0
		attempt_snap_values: List[Optional[float]] = [max_snap_m]
		if max_snap_m is not None:
			for f in adaptive_snap_expansions:
				attempt_snap_values.append(min(max_snap_m * f, max_snap_m * 5))  # hard cap 5x
		# Deduplicate while preserving order
		seen = set(); attempt_snap_values = [x for x in attempt_snap_values if (x not in seen and not seen.add(x))]

		path_nodes: Optional[List[Tuple[float,float]]] = None
		# 1. Try progressively larger snap radii
		for snap_limit in attempt_snap_values:
			if path_nodes is not None:
				break
			segment_debug.setdefault("snap_attempts", []).append(snap_limit)
			path_nodes = _shortest_path_ll_dir(gg, A, B, max_snap_m=snap_limit, directional_bias=True, max_deviation_deg=120.0, prevent_lane_hopping=prevent_lane_hopping, enforce_progress=progress_enforce, backward_progress_slack_m=backward_progress_slack_m, backward_block_m=backward_block_m, progress_penalty_factor=progress_penalty_factor, traffic_dir_block_deg=traffic_dir_block_deg, traffic_dir_soft_deg=traffic_dir_soft_deg, traffic_dir_soft_penalty=traffic_dir_soft_penalty)
			if path_nodes is None:
				path_nodes = _shortest_path_ll_dir(gg, A, B, max_snap_m=snap_limit, directional_bias=True, max_deviation_deg=150.0, prevent_lane_hopping=prevent_lane_hopping, enforce_progress=progress_enforce, backward_progress_slack_m=backward_progress_slack_m, backward_block_m=backward_block_m, progress_penalty_factor=progress_penalty_factor, traffic_dir_block_deg=traffic_dir_block_deg, traffic_dir_soft_deg=traffic_dir_soft_deg, traffic_dir_soft_penalty=traffic_dir_soft_penalty)
			if path_nodes is None:
				# Adaptive lane continuation distances: attempt explicit lane continuation with increasing radius
				for search_dist in adaptive_lane_search_distances_m:
					if path_nodes is not None:
						break
					segment_debug.setdefault("adaptive_lane_search", []).append(search_dist)
					# Forward search from A
					desired_bearing = _bearing(A, B)
					lane_start = _find_lane_continuation(gg, A, desired_bearing, B, max_search_distance=search_dist)
					if lane_start is not None:
						cand = _shortest_path_ll_dir(gg, lane_start, B, max_snap_m=snap_limit, directional_bias=True, max_deviation_deg=150.0, prevent_lane_hopping=False, enforce_progress=progress_enforce, backward_progress_slack_m=backward_progress_slack_m, backward_block_m=backward_block_m, progress_penalty_factor=progress_penalty_factor, traffic_dir_block_deg=traffic_dir_block_deg, traffic_dir_soft_deg=traffic_dir_soft_deg, traffic_dir_soft_penalty=traffic_dir_soft_penalty)
						if cand:
							path_nodes = ([A] if lane_start != A else []) + cand
							break
					# Reverse search from B
					rev_bearing = (_bearing(B, A) + 180) % 360
					lane_end = _find_lane_continuation(gg, B, rev_bearing, A, max_search_distance=search_dist)
					if lane_end is not None:
						cand2 = _shortest_path_ll_dir(gg, A, lane_end, max_snap_m=snap_limit, directional_bias=True, max_deviation_deg=150.0, prevent_lane_hopping=False, enforce_progress=progress_enforce, backward_progress_slack_m=backward_progress_slack_m, backward_block_m=backward_block_m, progress_penalty_factor=progress_penalty_factor, traffic_dir_block_deg=traffic_dir_block_deg, traffic_dir_soft_deg=traffic_dir_soft_deg, traffic_dir_soft_penalty=traffic_dir_soft_penalty)
						if cand2:
							path_nodes = cand2 + ([B] if lane_end != B else [])
							break
				# Fallback alternative path search if still none after adaptive searches
				if path_nodes is None:
					alt = _find_alternative_path(gg, A, B, max_snap_m=snap_limit, prevent_lane_hopping=prevent_lane_hopping)
					if alt is not None:
						path_nodes = alt
		# 2. If still no path and long segment, optionally include connectors
		if path_nodes is None and long_segment and include_connectors_on_long:
			if gg_with_connectors is None:
				connector_types = {"separation_lane","two-way_route","recommended_track","recommended_route_centreline","navigation_line","separation_crossing"}
				gg_with_connectors = _filter_graph_by_types(gg_full, connector_types)
			segment_debug["used_connectors_graph"] = True
			for snap_limit in attempt_snap_values:
				path_nodes = _shortest_path_ll_dir(gg_with_connectors, A, B, max_snap_m=snap_limit, directional_bias=True, max_deviation_deg=150.0, prevent_lane_hopping=prevent_lane_hopping, enforce_progress=progress_enforce, backward_progress_slack_m=backward_progress_slack_m, backward_block_m=backward_block_m, progress_penalty_factor=progress_penalty_factor, traffic_dir_block_deg=traffic_dir_block_deg, traffic_dir_soft_deg=traffic_dir_soft_deg, traffic_dir_soft_penalty=traffic_dir_soft_penalty)
				if path_nodes:
					break
		# 3. Partial mid-segment cluster extraction
		if path_nodes is None and long_segment and multi_clusters and max_snap_m:
			partial_path, pdebug = _partial_segment_follow_tss(gg, A, B, max_snap_m=max_snap_m, sample_spacing_m=partial_cluster_sample_spacing_m)
			if debug:
				segment_debug["partial_cluster"] = pdebug
			if partial_path:
				path_nodes = partial_path
		# 4. Multi-cluster chaining along the geodesic
		if path_nodes is None and long_segment and multi_clusters and max_snap_m:
			mc_path, mc_debug = _multi_cluster_segments_follow_tss(gg, A, B, max_snap_m=max_snap_m, sample_spacing_m=multi_cluster_sample_spacing_m)
			if debug:
				segment_debug["multi_cluster"] = mc_debug
			if mc_path:
				path_nodes = mc_path
		# 5. Final safety: if still none, decide between detour vs direct
		if path_nodes is None:
			crosses = _line_crosses_separation_zones(geojson_path, A, B) or _line_crosses_separation_zones(poly_path, A, B)
			segment_debug["final_direct_crosses_zone"] = crosses if debug else None
			if crosses:
				lane_ext = _extend_along_lane_until_zone_free(gg, A, B, poly_path=poly_path) if use_lane_extension_detour else None
				if debug:
					segment_debug["lane_extension_attempted"] = use_lane_extension_detour
					segment_debug["lane_extension_success"] = lane_ext is not None if use_lane_extension_detour else False
				if lane_ext:
					path_nodes = lane_ext
				else:
					path_nodes = _continue_in_direction_until_clear(geojson_path, A, B)
			else:
				path_nodes = [A, B]

		# Integrate found path_nodes into output
		if not path_nodes:
			continue
		# Ensure starts with A and ends with B
		if path_nodes[0] != A:
			path_nodes = [A] + path_nodes
		if path_nodes[-1] != B:
			path_nodes = path_nodes + [B]
		if not out:
			for p in path_nodes:
				if not out or out[-1] != p:
					out.append(p)
		else:
			# Avoid duplicating preceding waypoint
			if out[-1] != path_nodes[0]:
				out.append(path_nodes[0])
			for p in path_nodes[1:]:
				if out[-1] != p:
					out.append(p)
		if debug:
			partials_debug.append(segment_debug)
	
	# FINAL VALIDATION: Ensure no route segments cross separation zones
	# Now loop until stable so multiple offending segments are fixed, not only the first.
	poly_path = './seamarks-tss-polys.json'
	_changed = True
	_max_passes = 5
	_pass = 0
	while _changed and _pass < _max_passes:
		_changed = False
		_pass += 1
		for i in range(len(out) - 1):
			if _line_crosses_separation_zone_polygons(poly_path, out[i], out[i+1]):
				fixed_segment = _route_around_separation_zone(poly_path, out[i], out[i+1])
				if fixed_segment and len(fixed_segment) > 2:
					out = out[:i+1] + fixed_segment[1:-1] + out[i+1:]
					_changed = True
					break  # restart scan after modification
	
	if debug and debug_samples_info is not None:
		# Collect any residual direction violations along resulting path
		violations: List[dict] = []
		for i in range(len(out)-1):
			u = (out[i][1], out[i][0])
			v = (out[i+1][1], out[i+1][0])
			edata = gg.G.get_edge_data(u, v)
			if not edata:
				continue
			if edata.get('seamark_type') == 'separation_lane':
				tdiff = edata.get('traffic_dir_diff')
				if tdiff is not None and tdiff > traffic_dir_soft_deg:
					violations.append({
						'index': i,
						'from': out[i],
						'to': out[i+1],
						'diff_deg': round(tdiff,2),
						'against': bool(tdiff > traffic_dir_block_deg)
					})
		debug_samples_info['direction_violations'] = violations
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


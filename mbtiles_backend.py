# mbtiles_backend.py
"""
TSS routing backend from MBTiles (MVT) seamark tiles.

Routable features:
- One-way:       'separation_lane'
- Bidirectional: 'two-way_route'

Context (non-routable, used for boundary penalty & diagnostics):
- 'separation_line', 'separation_boundary', 'inshore_traffic_zone',
  'separation_crossing', 'separation_roundabout'

Highlights:
- Stitches tile-chopped features by (type,id) across tiles.
- Directed graph, with geodesic edge weights.
- Optional boundary proximity penalty to keep paths inside TSS.
- Bridges near-touching endpoints to heal tiny gaps (spatial-indexed).
- Robust snapping with max snap distance guard.
- Staged fallback: directed → locally-relaxed (allow short reverse near endpoints) → undirected (optional).

Usage:
    from mbtiles_backend import tss_correct_route, route_on_tss_between

    pts = [(56.2, 7.0), (56.6, 6.2)]
    path = route_on_tss_between("./seamarks-tss.mbtiles", pts[0], pts[1], z=14, padding_tiles=2)

    corrected = tss_correct_route("./seamarks-tss.mbtiles", pts, z=14, padding_tiles=2)
"""

from __future__ import annotations

import math
import logging
import time
import sqlite3
import gzip
import zlib
import re
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx
from mapbox_vector_tile import decode as mvt_decode
from pyproj import Geod
from shapely.geometry import LineString, Point, MultiLineString, Polygon
from shapely.ops import linemerge, unary_union
from shapely.strtree import STRtree
from shapely.affinity import scale as _affine_scale


# --------------------------- Config ---------------------------

# All TSS-related we care to load from tiles
TSS_LINE_TYPES = {
    "separation_lane",
    "separation_line",
    "separation_boundary",
    "separation_crossing",
    "separation_roundabout",
    "inshore_traffic_zone",
    "two-way_route",
    "recommended_track",
    "recommended_route_centreline",
    "navigation_line",
}

# Routable sets
GRAPH_ONEWAY_TYPES = {"separation_lane"}
GRAPH_BIDIR_TYPES  = {"two-way_route", "separation_crossing","recommended_track",
    "recommended_route_centreline",
    "navigation_line"}
CONTEXT_TYPES      = {
    "separation_line",
    "separation_boundary",
    "inshore_traffic_zone",
    
    "separation_roundabout",
}

# Tile decoding
DEFAULT_EXTENT = 4096

# Geodesy
GEOD = Geod(ellps="WGS84")

# Logger
logger = logging.getLogger("tss.mbtiles")

# Parameters (tune for your data)
CONNECT_RADIUS_M      = 200.0   # merge/align nearby endpoints when building the graph
BRIDGE_RADIUS_M       = 800.0   # try 250–500; bridges bidirectional edges between close nodes
MAX_BRIDGES_PER_NODE  = 6
DENSIFY_MAX_SEG_M     = 200.0   # try 150–300
MAX_SNAP_M            = 8000.0  # max distance allowed when snapping start/end to nearest segment
BOUNDARY_PENALTY_MU   = 0.0     # set >0 to bias away from boundaries; e.g. 200–800
BOUNDARY_FLOOR_M      = 50.0    # minimum distance used in penalty denominator

# Fallback
ALLOW_BOUNDARY_ROUTE_FALLBACK = True
ALLOW_UNDIRECTED_FALLBACK = True
LOCAL_RELAX_RADIUS_M = 1200.0    # allow local reverse edges only near endpoints before undirected fallback

logging.basicConfig(level=logging.DEBUG)

# ------------------------ Utility helpers ---------------------

def _densify_coords_lonlat(coords: List[Tuple[float, float]], max_seg_m: float) -> List[Tuple[float, float]]:
    """Insert geodesic intermediate points so consecutive vertices are <= max_seg_m apart (lon,lat)."""
    if len(coords) < 2:
        return coords
    out = [coords[0]]
    for (ax, ay), (bx, by) in zip(coords[:-1], coords[1:]):
        _, _, dist = GEOD.inv(ax, ay, bx, by)
        n = int(dist // max_seg_m)
        if n > 0:
            # GEOD.npts excludes the endpoints; returns list of (lon,lat)
            out.extend(GEOD.npts(ax, ay, bx, by, n))
        out.append((bx, by))
    return out

def _maybe_decompress(buf: bytes) -> bytes:
    if len(buf) >= 2 and buf[:2] == b"\x1f\x8b":  # gzip
        return gzip.decompress(buf)
    try:
        return zlib.decompress(buf)
    except zlib.error:
        return buf

def _seamark_type_from_props(props: dict) -> Optional[str]:
    st = props.get("seamark:type") or props.get("seamark_type")
    if st:
        return st
    ot = props.get("other_tags")
    if not ot or not isinstance(ot, str):
        return None
    m = re.search(r'"seamark:type"\s*=>\s*"([^"]+)"', ot)
    return m.group(1) if m else None

def geodesic_meters(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    _, _, dist = GEOD.inv(a[0], a[1], b[0], b[1])
    return float(dist)

def _meters_to_degrees(lat_deg: float, meters: float) -> float:
    """
    Approx degrees of latitude for small-radius buffers.
    We use a simple conversion by local meridian length for tiny radii; good for < a few km.
    """
    # 1 deg latitude ~ 111,132 m; longitude scales by cos(lat)
    return meters / 111_132.0

def _approx_buffer_degrees(pt: Point, meters: float) -> Polygon:
    """
    Return an *elliptical* buffer around a lon/lat point using a tiny-degree circle
    scaled to the requested meters in x/y (lon/lat) directions. Good for ~sub-km radii.
    """
    # 1 deg latitude ~ 111_132 m; longitude shrinks by cos(lat)
    lat = pt.y
    deg_per_m_lat = 1.0 / 111_132.0
    r_deg_lat = meters * deg_per_m_lat
    r_deg_lon = r_deg_lat * max(1e-6, math.cos(math.radians(lat)))

    # Start as a unit circle centered at pt (in degrees), then scale about the point
    unit = Point(pt.x, pt.y).buffer(1.0)  # circle of radius 1 degree
    return _affine_scale(unit, xfact=r_deg_lon, yfact=r_deg_lat, origin=(pt.x, pt.y))

def _geom_from_tree(obj, geoms_list: List):
    """Shapely STRtree may return either geometry or index depending on version/build."""
    if hasattr(obj, "geom_type"):
        return obj
    try:
        idx = int(obj)
        return geoms_list[idx]
    except Exception:
        return obj

def _merge_line_parts(parts: List[LineString]) -> List[LineString]:
    """Merge a list of (possibly tile-chopped) LineStrings into continuous lines."""
    if not parts:
        return []
    u = unary_union(parts)
    lines: List[LineString] = []
    gt = u.geom_type
    if gt == "LineString":
        lines = [u]
    elif gt == "MultiLineString":
        lines = list(u.geoms)
    elif gt == "GeometryCollection":
        for g in u.geoms:
            if g.geom_type == "LineString":
                lines.append(g)
            elif g.geom_type == "MultiLineString":
                lines.extend(g.geoms)
    else:
        lines = parts

    if not lines:
        return []
    if len(lines) == 1:
        return lines

    merged = linemerge(MultiLineString(lines))
    if merged.geom_type == "LineString":
        return [merged]
    return list(merged.geoms)

# ------------------------ Tile math (XYZ) ---------------------

def lonlat_to_tile(lon: float, lat: float, z: int) -> Tuple[int, int]:
    lat = max(min(lat, 85.05112878), -85.05112878)
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.log(math.tan(math.radians(lat)) + 1 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n)
    return x, y

def tile_point_to_lonlat(z: int, x: int, y: int, px: float, py: float, extent: int) -> Tuple[float, float]:
    fx = x + (px / extent)
    fy = y + (py / extent)
    lon = fx / (2 ** z) * 360.0 - 180.0
    n = math.pi - 2.0 * math.pi * fy / (2 ** z)
    lat = math.degrees(math.atan(math.sinh(n)))
    return lon, lat

def tiles_for_bbox(bbox: Tuple[float, float, float, float], z: int, padding: int = 1) -> List[Tuple[int, int, int]]:
    min_lon, min_lat, max_lon, max_lat = bbox
    x_min, y_max = lonlat_to_tile(min_lon, min_lat, z)
    x_max, y_min = lonlat_to_tile(max_lon, max_lat, z)
    xs = range(min(x_min, x_max) - padding, max(x_min, x_max) + padding + 1)
    ys = range(min(y_min, y_max) - padding, max(y_min, y_max) + padding + 1)
    return [(z, x, y) for x in xs for y in ys]

# ---------------------- MBTiles I/O & decode ------------------

@lru_cache(maxsize=1)
def _open_mbtiles(path: str) -> sqlite3.Connection:
    uri = f"file:{path}?mode=ro"
    conn = sqlite3.connect(uri, uri=True, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA query_only = ON;")
    try:
        cur = conn.execute("SELECT COUNT(1) AS c FROM tiles")
        row = cur.fetchone()
        logger.info("Opened MBTiles: %s (tiles=%s)", path, row["c"] if row else "?")
    except Exception:
        logger.debug("Could not count tiles for %s", path)
    return conn

def _fetch_mvt(conn: sqlite3.Connection, z: int, x: int, y: int) -> Optional[bytes]:
    # MBTiles uses TMS (y flipped)
    tms_y = (2 ** z - 1) - y
    row = conn.execute(
        "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
        (z, x, tms_y),
    ).fetchone()
    return row["tile_data"] if row else None

def mbtiles_stats(path: str) -> Dict[str, Optional[str]]:
    """Return basic MBTiles stats for debugging."""
    conn = _open_mbtiles(path)
    out: Dict[str, Optional[str]] = {}
    try:
        row = conn.execute("SELECT COUNT(1) AS c, MIN(zoom_level) AS zmin, MAX(zoom_level) AS zmax FROM tiles").fetchone()
        out["tile_count"] = int(row["c"]) if row and row["c"] is not None else 0
        out["min_zoom"] = int(row["zmin"]) if row and row["zmin"] is not None else None
        out["max_zoom"] = int(row["zmax"]) if row and row["zmax"] is not None else None
    except Exception as e:
        logger.warning("mbtiles_stats: failed tiles query: %s", e)
    try:
        meta = {r["name"]: r["value"] for r in conn.execute("SELECT name, value FROM metadata").fetchall()}
    except Exception:
        meta = {}
    for k in ["name", "version", "description", "attribution", "bounds", "json"]:
        out[k] = meta.get(k)
    logger.debug("mbtiles_stats(%s): %s", path, out)
    return out

@dataclass
class LoadedCorridor:
    lanes_oneway: List[LineString]
    lanes_bidir: List[LineString]
    context_lines: List[LineString]

def load_tss_lines_from_mbtiles(
    mbtiles_path: str,
    corridor_points_latlon: Sequence[Tuple[float, float]],
    z: int = 12,
    padding_tiles: int = 1,
) -> LoadedCorridor:
    """
    Load TSS-related lines from tiles overlapping a route corridor.
    Stitches features by (seamark:type, id) across tiles into continuous lines.
    Returns lon/lat LineStrings split into oneway/bidir + context.
    """
    if len(corridor_points_latlon) < 2:
        return LoadedCorridor([], [], [])

    lats = [lat for (lat, _lon) in corridor_points_latlon]
    lons = [lon for (_lat, lon) in corridor_points_latlon]
    bbox = (min(lons), min(lats), max(lons), max(lats))

    conn = _open_mbtiles(mbtiles_path)

    buckets = {
        "oneway": {},  # (stype, fid) -> [LineString,...]
        "bidir":  {},
        "other":  {},
    }
    seen_types = set()

    for (Z, X, Y) in tiles_for_bbox(bbox, z=z, padding=padding_tiles):
        buf = _fetch_mvt(conn, Z, X, Y)
        if not buf:
            continue
        buf = _maybe_decompress(buf)
        decoded = mvt_decode(buf)  # dict[layer_name] -> {extent, features}

        for layer_name, layer in decoded.items():
            extent = int(layer.get("extent") or DEFAULT_EXTENT)
            feats = layer.get("features", []) or []
            for f in feats:
                props = f.get("properties") or {}
                stype = _seamark_type_from_props(props)
                if stype:
                    seen_types.add(stype)
                if not stype or stype not in TSS_LINE_TYPES:
                    continue
                geom = f.get("geometry") or {}
                gtype = geom.get("type")
                coords = geom.get("coordinates")
                if not coords or gtype not in {"LineString", "MultiLineString"}:
                    continue

                raw_id = props.get("id") or props.get("osm_id")
                key = (stype, str(raw_id) if raw_id is not None else f"{id(coords)}-{layer_name}")

                parts = [coords] if gtype == "LineString" else coords
                for part in parts:
                    ll = [tile_point_to_lonlat(Z, X, Y, px, py, extent) for (px, py) in part]
                    if len(ll) < 2:
                        continue
                    ls = LineString(ll)

                    if stype in GRAPH_ONEWAY_TYPES:
                        buckets["oneway"].setdefault(key, []).append(ls)
                    elif stype in GRAPH_BIDIR_TYPES:
                        buckets["bidir"].setdefault(key, []).append(ls)
                    else:
                        buckets["other"].setdefault(key, []).append(ls)

    oneway, bidir, context = [], [], []
    for parts in buckets["oneway"].values():
        oneway.extend(_merge_line_parts(parts))
    for parts in buckets["bidir"].values():
        bidir.extend(_merge_line_parts(parts))
    for parts in buckets["other"].values():
        context.extend(_merge_line_parts(parts))

    if seen_types:
        logger.debug("Seen seamark types (raw) z=%s: %s", z, sorted(seen_types))

    return LoadedCorridor(lanes_oneway=oneway, lanes_bidir=bidir, context_lines=context)

# -------------------------- Graph build -----------------------

@dataclass
class GraphData:
    G: nx.DiGraph
    segment_index: STRtree
    segment_geoms: List[LineString]
    node_index: STRtree
    node_points: List[Point]

def _build_boundary_index(context_lines: Sequence[LineString]) -> Optional[STRtree]:
    if not context_lines:
        return None
    # Keep only boundaries/lines for penalty
    bnds = []
    for ls in context_lines:
        bnds.append(ls)
    return STRtree(bnds) if bnds else None

def _boundary_penalty(mid: Tuple[float, float], boundary_tree: Optional[STRtree]) -> float:
    if boundary_tree is None or BOUNDARY_PENALTY_MU <= 0.0:
        return 0.0
    p = Point(mid[0], mid[1])
    # nearest candidate
    cand = boundary_tree.nearest(p)
    geom = _geom_from_tree(cand, boundary_tree.geometries if hasattr(boundary_tree, "geometries") else [])
    try:
        d_m = geodesic_meters((p.x, p.y), (geom.interpolate(geom.project(p))).coords[0])
    except Exception:
        d_m = BOUNDARY_FLOOR_M
    d_m = max(d_m, BOUNDARY_FLOOR_M)
    return BOUNDARY_PENALTY_MU / d_m

def build_lane_graph(
    lanes_oneway: Sequence[LineString],
    lanes_bidir: Sequence[LineString],
    context_lines: Sequence[LineString],
    connect_radius_m: float = CONNECT_RADIUS_M
) -> GraphData:
    G = nx.DiGraph()
    segment_geoms: List[LineString] = []
    boundary_tree = _build_boundary_index(context_lines)

    # Canonicalize nodes by merging close coordinates
    class _NodeCanon:
        def __init__(self, radius_m: float):
            self.radius_m = radius_m
            self.nodes: List[Tuple[float, float]] = []
        def get(self, lonlat: Tuple[float, float]) -> Tuple[float, float]:
            for n in self.nodes:
                if geodesic_meters(lonlat, n) <= self.radius_m:
                    return n
            self.nodes.append(lonlat)
            return lonlat

    canon = _NodeCanon(connect_radius_m)

    def add_edge(a: Tuple[float, float], b: Tuple[float, float]):
        if a == b:
            return
        a = canon.get(a)
        b = canon.get(b)
        base_w = geodesic_meters(a, b)
        mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
        w = base_w + _boundary_penalty(mid, boundary_tree)
        G.add_edge(a, b, weight=w)
        segment_geoms.append(LineString([a, b]))

    def add_lines(ls: LineString, bidir: bool):
        coords = _densify_coords_lonlat(list(ls.coords), DENSIFY_MAX_SEG_M)
        for i in range(len(coords) - 1):
            a = (coords[i][0],  coords[i][1])
            b = (coords[i+1][0], coords[i+1][1])
            add_edge(a, b)
            if bidir:
                add_edge(b, a)

    for ls in lanes_oneway:
        add_lines(ls, bidir=False)
    for ls in lanes_bidir:
        add_lines(ls, bidir=True)

    seg_tree = STRtree(segment_geoms) if segment_geoms else STRtree([LineString([(0,0),(0,0)])])
    node_points = [Point(n[0], n[1]) for n in G.nodes]
    node_tree = STRtree(node_points) if node_points else STRtree([Point(0,0)])

    return GraphData(G=G, segment_index=seg_tree, segment_geoms=segment_geoms,
                     node_index=node_tree, node_points=node_points)

def bridge_graph(gd: GraphData, radius_m: float = BRIDGE_RADIUS_M, per_node_limit: int = MAX_BRIDGES_PER_NODE) -> int:
    """
    Add bidirectional 'bridge' edges between graph nodes within radius_m using spatial queries.
    Returns number of new edges added (both directions counted).
    """
    if not gd.node_points:
        return 0

    added = 0
    for a_pt in gd.node_points:
        # Approximate ellipse buffer in lon/lat degrees at this latitude
        buf = _approx_buffer_degrees(a_pt, radius_m)

        # Shapely STRtree.query can return geometries *or* integer indexes depending on build.
        candidates_any = gd.node_index.query(buf)

        count_here = 0
        for b_any in candidates_any:
            b_pt = _geom_from_tree(b_any, gd.node_points)
            # If normalization failed, skip
            if not hasattr(b_pt, "geom_type"):
                continue
            if a_pt.equals(b_pt):
                continue

            a = (a_pt.x, a_pt.y)
            b = (b_pt.x, b_pt.y)
            d = geodesic_meters(a, b)
            if d <= radius_m:
                if not gd.G.has_edge(a, b):
                    gd.G.add_edge(a, b, weight=d); added += 1; count_here += 1
                if not gd.G.has_edge(b, a):
                    gd.G.add_edge(b, a, weight=d); added += 1
                if count_here >= per_node_limit:
                    break
    return added


# --------------------------- Snapping -------------------------

def snap_to_segment(
    pt_ll: Tuple[float, float],
    gd: GraphData,
    max_snap_m: Optional[float] = MAX_SNAP_M,
) -> Tuple[float, float]:
    p = Point(pt_ll[0], pt_ll[1])
    seg_any = gd.segment_index.nearest(p)
    seg = _geom_from_tree(seg_any, gd.segment_geoms)
    if seg is None or seg.is_empty:
        return pt_ll
    nearest = seg.interpolate(seg.project(p))
    snapped = (nearest.x, nearest.y)
    d = geodesic_meters(pt_ll, snapped)
    if (max_snap_m is not None) and (d > max_snap_m):
        raise NoTSSDataError(
            f"Point {tuple(round(v,6) for v in pt_ll)} is {int(d)} m from nearest TSS segment "
            f"(> {int(max_snap_m)} m)"
        )
    return snapped


def nearest_graph_node(pt_ll: Tuple[float, float], gd: GraphData) -> Tuple[float, float]:
    p = Point(pt_ll[0], pt_ll[1])
    node_any = gd.node_index.nearest(p)
    node_geom = _geom_from_tree(node_any, gd.node_points)
    snapped = (node_geom.x, node_geom.y)
    return snapped

# ---------------------------- Routing -------------------------

class NoTSSDataError(Exception):
    pass

class NoPathError(Exception):
    pass

def _add_local_reverse_edges_near(gd: GraphData, center_node: Tuple[float, float], radius_m: float) -> int:
    """
    Allow local reverse travel by mirroring edges whose both endpoints lie within radius_m of the center node.
    Works with STRtree builds that return geometries OR integer indexes.
    """
    added = 0
    cx, cy = center_node
    cpt = Point(cx, cy)
    buf = _approx_buffer_degrees(cpt, radius_m)

    # Get candidates, normalize to geometry objects
    candidates_any = gd.node_index.query(buf)
    nearby_pts: List[Point] = []
    for obj in candidates_any:
        pt = _geom_from_tree(obj, gd.node_points)
        if hasattr(pt, "geom_type"):  # ensure it's a geometry
            nearby_pts.append(pt)

    nearby_nodes = {(pt.x, pt.y) for pt in nearby_pts}

    for u, v, data in list(gd.G.edges(data=True)):
        if u in nearby_nodes and v in nearby_nodes:
            if not gd.G.has_edge(v, u):
                gd.G.add_edge(v, u, weight=data.get("weight", geodesic_meters(v, u)))
                added += 1
    return added


def route_on_tss_between(
    mbtiles_path: str,
    start_latlon: Tuple[float, float],  # (lat, lon)
    end_latlon: Tuple[float, float],    # (lat, lon)
    z: int = 12,
    padding_tiles: int = 1,
    max_snap_m: Optional[float] = MAX_SNAP_M
) -> List[Tuple[float, float]]:
    """
    Route strictly along TSS lanes (directed where available).
    Returns a list of (lat, lon) nodes along the computed path.
    """
    logger.info(
        "route_on_tss_between: start=%s end=%s z=%s padding=%s",
        tuple(round(v, 6) for v in start_latlon), tuple(round(v, 6) for v in end_latlon), z, padding_tiles,
    )
    t0 = time.perf_counter()
    corridor = load_tss_lines_from_mbtiles(
        mbtiles_path,
        [start_latlon, end_latlon],
        z=z,
        padding_tiles=padding_tiles,
    )
    if not corridor.lanes_oneway and not corridor.lanes_bidir:
        raise NoTSSDataError("No TSS lanes found near the corridor.")

    gd = build_lane_graph(corridor.lanes_oneway, corridor.lanes_bidir, corridor.context_lines, CONNECT_RADIUS_M)
    bridged = bridge_graph(gd, BRIDGE_RADIUS_M)
    if gd.G.number_of_nodes() == 0:
        raise NoTSSDataError("No TSS lane graph could be built from the tiles.")

    # Convert (lat,lon) -> (lon,lat) for geometry math
    s_ll = (start_latlon[1], start_latlon[0])
    e_ll = (end_latlon[1], end_latlon[0])

    s_snap = snap_to_segment(s_ll, gd, max_snap_m=max_snap_m)
    e_snap = snap_to_segment(e_ll, gd, max_snap_m=max_snap_m)
    s_node = nearest_graph_node(s_snap, gd)
    e_node = nearest_graph_node(e_snap, gd)

     # Try requested zoom, then lower zooms (more generalized, often more connected)
    for try_z in (z, max(10, z-1), max(10, z-2)):
        logger.debug(f"route_on_tss_between: attempting zoom {try_z}")

        corridor = load_tss_lines_from_mbtiles(
            mbtiles_path, [start_latlon, end_latlon], z=try_z, padding_tiles=padding_tiles
        )
        if not corridor.lanes_oneway and not corridor.lanes_bidir:
            continue

        lanes_oneway = corridor.lanes_oneway
        lanes_bidir  = corridor.lanes_bidir
        

        if (not lanes_oneway and not lanes_bidir and
            ALLOW_BOUNDARY_ROUTE_FALLBACK and corridor.context_lines):
            logger.debug("Fallback: using context_lines as bidirectional at z=%s", try_z)
            lanes_bidir = corridor.context_lines

        gd = build_lane_graph(lanes_oneway, lanes_bidir, corridor.context_lines, CONNECT_RADIUS_M)
        bridged = bridge_graph(gd, BRIDGE_RADIUS_M)
        if gd.G.number_of_nodes() == 0:
            continue

        s_snap = snap_to_segment(s_ll, gd, max_snap_m=max_snap_m)
        e_snap = snap_to_segment(e_ll, gd, max_snap_m=max_snap_m)
        s_node = nearest_graph_node(s_snap, gd)
        e_node = nearest_graph_node(e_snap, gd)

        # Optionally log connectivity to help debug
        try:
            UG = gd.G.to_undirected()
            logger.debug(
                "graph z=%s: nodes=%d edges=%d same_comp=%s",
                try_z, gd.G.number_of_nodes(), gd.G.number_of_edges(),
                nx.has_path(UG, s_node, e_node)
            )
        except Exception:
            pass

        # Directed
        try:
            path_lonlat = nx.shortest_path(gd.G, s_node, e_node, weight="weight")
            logger.info("route_on_tss_between: success directed z=%s points=%d bridged=%d", try_z, len(path_lonlat), bridged)
            return [(lon, lat) for (lon, lat) in path_lonlat]
        except nx.NetworkXNoPath:
            pass

        # Local relax
        add1 = _add_local_reverse_edges_near(gd, s_node, LOCAL_RELAX_RADIUS_M)
        add2 = _add_local_reverse_edges_near(gd, e_node, LOCAL_RELAX_RADIUS_M)
        try:
            path_lonlat = nx.shortest_path(gd.G, s_node, e_node, weight="weight")
            logger.info("route_on_tss_between: success relaxed z=%s points=%d bridged=%d added_rev=%d",
                        try_z, len(path_lonlat), bridged, add1+add2)
            return [(lon, lat) for (lon, lat) in path_lonlat]
        except nx.NetworkXNoPath:
            pass

        # Undirected fallback
        if ALLOW_UNDIRECTED_FALLBACK:
            UG = gd.G.to_undirected()
            try:
                undirected_path = nx.shortest_path(UG, s_node, e_node, weight="weight")
                logger.info("route_on_tss_between: success undirected z=%s points=%d bridged=%d", try_z, len(undirected_path), bridged)
                return [(lon, lat) for (lon, lat) in undirected_path]
            except nx.NetworkXNoPath:
                pass

    raise NoPathError("No directed/undirected TSS path found between endpoints.")

def tss_correct_route(
    mbtiles_path: str,
    waypoints_latlon: Sequence[Tuple[float, float]],
    z: int = 12,
    padding_tiles: int = 1,
    max_snap_m: Optional[float] = MAX_SNAP_M,   # add
) -> List[Tuple[float, float]]:
    """
    For a polyline of waypoints (lat,lon), replace each segment by a TSS-only route,
    concatenating them into a continuous corrected path.
    """
    if len(waypoints_latlon) < 2:
        return list(waypoints_latlon)

    corrected: List[Tuple[float, float]] = []
    for i in range(len(waypoints_latlon) - 1):
        a = waypoints_latlon[i]
        b = waypoints_latlon[i + 1]
        seg = route_on_tss_between(
            mbtiles_path, a, b, z=z, padding_tiles=padding_tiles, max_snap_m=max_snap_m
        )
        if not corrected:
            corrected.extend(seg)
        else:
            corrected.extend(seg[1:])  # avoid duplicate vertex
    return corrected

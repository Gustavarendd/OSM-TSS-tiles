# main.py
from fastapi import FastAPI, HTTPException, Body
from fastapi.middleware.cors import CORSMiddleware
import logging
import os
import csv
from pathlib import Path
from pydantic import BaseModel, Field   # ⬅️ use Field
from typing import List, Optional, Any

from mbtiles_backend import tss_correct_route, NoPathError, NoTSSDataError, mbtiles_stats
from geojson_backend import tss_correct_route_geojson as geojson_correct, NoPathError as GeoNoPathError, NoTSSDataError as GeoNoTSSDataError

MBTILES_PATH = "./seamarks-tss.mbtiles"

class Waypoint(BaseModel):
    lat: float
    lon: float

class RouteRequest(BaseModel):
    waypoints: List[Waypoint] = Field(..., min_length=2)  # ⬅️ v2 style
    z: int = 12
    padding_tiles: int = 1

class RouteResponse(BaseModel):
    waypoints: List[Waypoint]
    meta: dict

class GeoJSONRouteRequest(BaseModel):
    waypoints: List[Waypoint] = Field(..., min_length=2)
    max_snap_m: Optional[float] = Field(None, description="Maximum distance meters to snap endpoints to TSS graph; None=unlimited")
    include_bridging: bool = Field(True, description="Include original waypoints before/after snapped TSS path segments")
    sample_spacing_m: float = Field(1000.0, description="Spacing (m) for mid-segment sampling when endpoints outside snap radius")
    debug: bool = Field(False, description="Return debug statistics about snapping & sampling")
    multi_clusters: bool = Field(True, description="Attempt to chain multiple disjoint TSS clusters along a segment")

app = FastAPI()
# Allow any origin (dev / testing). Tighten in production if needed.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Basic logging config; respect LOG_LEVEL env var (default INFO)
log_level = os.getenv("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, log_level, logging.INFO),
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("tss.api")

@app.on_event("startup")
def _startup_log():
    try:
        stats = mbtiles_stats(MBTILES_PATH)
        logger.info("startup: mbtiles=%s tiles=%s z=[%s,%s] name=%s", MBTILES_PATH, stats.get("tile_count"), stats.get("min_zoom"), stats.get("max_zoom"), stats.get("name"))
    except Exception as e:
        logger.warning("startup: failed to read MBTiles stats: %s", e)

# @app.post("/tss/correct", response_model=RouteResponse)
# def correct(req: RouteRequest):
#     logger.info(
#         "/tss/correct: waypoints=%d z=%s padding=%s", len(req.waypoints), req.z, req.padding_tiles
#     )
#     pts = [(w.lat, w.lon) for w in req.waypoints]
#     try:
#         out = tss_correct_route(MBTILES_PATH, pts, z=req.z, padding_tiles=req.padding_tiles)
#     except NoTSSDataError as e:
#         logger.warning("/tss/correct: NoTSSDataError: %s", e)
#         raise HTTPException(404, str(e))
#     except NoPathError as e:
#         logger.warning("/tss/correct: NoPathError: %s", e)
#         raise HTTPException(409, str(e))
#     logger.info("/tss/correct: success points=%d", len(out))
#     return RouteResponse(
#         waypoints=[Waypoint(lat=lat, lon=lon) for (lat, lon) in out],
#         meta={"source": "mbtiles", "zoom": req.z, "segments": len(out)},
#     )


@app.post("/tss/correct_geojson", response_model=RouteResponse)
def correct_geojson(req: GeoJSONRouteRequest):
    """Correct a route using the static GeoJSON seamarks lines file.

    Only requires list of waypoints; file path can be overridden by env GEOJSON_PATH.
    """
    geo_path = os.getenv("GEOJSON_PATH", "./seamarks-tss-lines.json")
    pts = [(w.lat, w.lon) for w in req.waypoints]
    try:
        out, debug_info = geojson_correct(
            geo_path,
            pts,
            max_snap_m=req.max_snap_m,
            include_bridging=req.include_bridging,
            sample_spacing_m=req.sample_spacing_m,
            debug=req.debug,
            multi_clusters=req.multi_clusters,
            prevent_lane_hopping=True,  # Enforce strict lane direction following
            only_lanes=False,  # Include all routable types for better connectivity
        )
    except GeoNoTSSDataError as e:
        raise HTTPException(404, f"GeoJSON NoTSSDataError: {e}")
    except GeoNoPathError as e:
        raise HTTPException(409, f"GeoJSON NoPathError: {e}")
    except ValueError as e:
        # Handle separation zone violations
        if "separation zone" in str(e).lower():
            raise HTTPException(400, f"Route violates separation zone restrictions: {e}")
        else:
            raise HTTPException(400, f"Invalid request: {e}")
    meta: dict[str, Any] = {
        "source": "geojson",
        "segments": len(out),
        "file": geo_path,
        "max_snap_m": req.max_snap_m,
        "include_bridging": req.include_bridging,
        "sample_spacing_m": req.sample_spacing_m,
        "multi_clusters": req.multi_clusters,
    }
    if req.debug and debug_info:
        meta["debug"] = debug_info
    wp_in_csv_path = os.getenv("WAYPOINTS_CSV_PATH", "waypoints-in.csv")
    try:
        with open(wp_in_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["lat", "lon"])
            for lat, lon in pts:
                writer.writerow([lat, lon])
    except Exception as e:
        # Do not fail request if CSV write fails; just record the error
        meta["csv_write_error"] = str(e)

    wp_out_csv_path = os.getenv("WAYPOINTS_CSV_PATH", "waypoints-out.csv")
    try:
        with open(wp_out_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["lat", "lon"])
            for lat, lon in out:
                writer.writerow([lat, lon])
        meta["csv_file"] = wp_out_csv_path
    except Exception as e:
        # Do not fail request if CSV write fails; just record the error
        meta["csv_write_error"] = str(e)

    return RouteResponse(
        waypoints=[Waypoint(lat=lat, lon=lon) for (lat, lon) in out],
        meta=meta,
    )


@app.get("/debug/mbtiles")
def debug_mbtiles():
    """Return basic MBTiles metadata/stats for debugging."""
    try:
        stats = mbtiles_stats(MBTILES_PATH)
        return {"ok": True, "stats": stats}
    except Exception as e:
        raise HTTPException(500, f"Failed to read MBTiles stats: {e}")




import networkx as nx
from mbtiles_backend import load_tss_lines_from_mbtiles, build_lane_graph, bridge_graph, CONNECT_RADIUS_M, BRIDGE_RADIUS_M

@app.post("/debug/graph")
def debug_graph(payload=Body(...)):
    pts = [(wp["lat"], wp["lon"]) for wp in payload["waypoints"]]
    z = payload.get("z", 14); pad = payload.get("padding_tiles", 3)
    corr = load_tss_lines_from_mbtiles(MBTILES_PATH, pts, z=z, padding_tiles=pad)
    gd = build_lane_graph(corr.lanes_oneway, corr.lanes_bidir, connect_radius_m=CONNECT_RADIUS_M)
    before = nx.number_connected_components(gd.G.to_undirected())
    bridged = bridge_graph(gd, BRIDGE_RADIUS_M)
    after = nx.number_connected_components(gd.G.to_undirected())
    return {
        "lanes_oneway": len(corr.lanes_oneway),
        "lanes_bidir": len(corr.lanes_bidir),
        "nodes": gd.G.number_of_nodes(),
        "edges": gd.G.number_of_edges(),
        "components_before": before,
        "bridged_edges_added": bridged,
        "components_after": after,
        "z": z, "padding": pad
    }
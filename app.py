"""
Shopping Complex — A* Evacuation Planner
Backend: Flask REST API

Run:
    pip install flask flask-cors
    python app.py

API Endpoints:
    GET  /api/graph          → full graph (nodes + edges)
    POST /api/solve          → run A* and return optimal + all paths
    POST /api/block          → temporarily block an edge
    DELETE /api/block/<id>   → unblock an edge
    GET  /api/status         → server health check
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
import heapq
import math
import json
from typing import Optional

app = Flask(__name__)
CORS(app)  # Allow all origins for frontend connectivity

# ──────────────────────────────────────────────
# GRAPH DATA — Shopping Complex
# Rows A–E (bottom→top), Columns 1–7 (left→right)
# ──────────────────────────────────────────────

ROWS = ['A', 'B', 'C', 'D', 'E']
COLS = list(range(1, 8))  # 1..7

# SVG layout constants (must match frontend)
SVG_W, SVG_H = 960, 720
LEFT_PAD, RIGHT_PAD, TOP_PAD, BOT_PAD = 80, 80, 55, 55

def gx(col_idx: int) -> float:
    """SVG x-coordinate for column index 0..8"""
    return LEFT_PAD + col_idx * (SVG_W - LEFT_PAD - RIGHT_PAD) / 8

def gy(row_idx: int) -> float:
    """SVG y-coordinate for row index 0..6 (0=bottom)"""
    return SVG_H - BOT_PAD - row_idx * (SVG_H - TOP_PAD - BOT_PAD) / 6


def build_graph():
    """Build the Shopping Complex weighted graph and return nodes + adjacency list."""
    nodes = {}
    edges = []  # list of {id, a, b, w}

    # ── Interior nodes ──
    for r_idx, row in enumerate(ROWS):
        for c_idx, col in enumerate(COLS):
            nid = f"{row}{col}"
            nodes[nid] = {
                "id": nid,
                "x": round(gx(c_idx + 1), 2),
                "y": round(gy(r_idx + 1), 2),
                "is_exit": False,
                "label": nid,
                "h": 0.0,   # computed below
                "row": row,
                "col": col
            }

    # ── Exit nodes ──
    exits_def = [
        ("EXIT1", gx(0),  gy(3), "West wall, C-row"),
        ("EXIT2", gx(8),  gy(3), "East wall, C-row"),
        ("EXIT3", gx(4),  gy(0), "South wall, column 4"),
        ("EXIT4", gx(4),  gy(6), "North wall, column 4"),
        ("EXIT5", gx(0),  gy(5), "West wall, E-row"),
        ("EXIT6", gx(8),  gy(5), "East wall, E-row"),
        ("EXIT7", gx(0),  gy(1), "West wall, A-row"),
        ("EXIT8", gx(8),  gy(1), "East wall, A-row"),
    ]
    for eid, x, y, desc in exits_def:
        nodes[eid] = {
            "id": eid,
            "x": round(x, 2),
            "y": round(y, 2),
            "is_exit": True,
            "label": eid,
            "description": desc,
            "h": 0.0
        }

    def add_edge(a, b, w):
        eid = f"{a}--{b}"
        edges.append({"id": eid, "a": a, "b": b, "w": round(w, 2)})

    # ── Horizontal corridor edges ──
    # Weight pattern: slightly varies by position for realism
    horiz_weights = {
        'A': [1.5, 1.8, 1.6, 1.7, 1.5, 1.9],
        'B': [1.6, 1.7, 1.8, 1.6, 1.7, 1.8],
        'C': [2.0, 2.0, 2.0, 2.0, 2.0, 2.0],  # main corridor, wider
        'D': [1.6, 1.7, 1.8, 1.6, 1.7, 1.8],
        'E': [1.5, 1.8, 1.6, 1.7, 1.5, 1.9],
    }
    for row in ROWS:
        for c_idx in range(len(COLS) - 1):
            w = horiz_weights[row][c_idx]
            add_edge(f"{row}{COLS[c_idx]}", f"{row}{COLS[c_idx+1]}", w)

    # ── Vertical stairwell edges ──
    vert_weights = {
        1: [1.8, 2.0, 1.8, 2.0],
        2: [1.9, 2.1, 1.9, 2.1],
        3: [1.8, 2.0, 1.8, 2.0],
        4: [2.0, 2.2, 2.0, 2.2],  # main stairwell
        5: [1.8, 2.0, 1.8, 2.0],
        6: [1.9, 2.1, 1.9, 2.1],
        7: [1.8, 2.0, 1.8, 2.0],
    }
    for col in COLS:
        for r_idx in range(len(ROWS) - 1):
            w = vert_weights[col][r_idx]
            add_edge(f"{ROWS[r_idx]}{col}", f"{ROWS[r_idx+1]}{col}", w)

    # ── Exit connector edges ──
    exit_edges = [
        ("EXIT1", "C1", 1.2),
        ("EXIT2", "C7", 1.2),
        ("EXIT3", "A4", 1.5),
        ("EXIT4", "E4", 1.5),
        ("EXIT5", "E1", 1.2),
        ("EXIT6", "E7", 1.2),
        ("EXIT7", "A1", 1.2),
        ("EXIT8", "A7", 1.2),
    ]
    for a, b, w in exit_edges:
        add_edge(a, b, w)

    # ── Build adjacency list (undirected) ──
    adj = {nid: [] for nid in nodes}
    for e in edges:
        adj[e["a"]].append({"to": e["b"], "w": e["w"], "eid": e["id"]})
        adj[e["b"]].append({"to": e["a"], "w": e["w"], "eid": e["id"]})

    # ── Compute heuristics: scaled Euclidean distance to nearest exit ──
    exit_nodes = [n for n in nodes.values() if n["is_exit"]]
    SCALE = 55.0  # pixels per minute

    for nid, node in nodes.items():
        if node["is_exit"]:
            node["h"] = 0.0
            continue
        min_d = min(
            math.sqrt((node["x"] - e["x"])**2 + (node["y"] - e["y"])**2)
            for e in exit_nodes
        )
        node["h"] = round(min_d / SCALE, 2)

    return nodes, edges, adj


# Build graph once at startup
NODES, EDGES, ADJ = build_graph()
EXIT_IDS = [nid for nid, n in NODES.items() if n["is_exit"]]
BLOCKED_EDGES: set[str] = set()  # persistent blocked edges (session-level)


# ──────────────────────────────────────────────
# A* ALGORITHM
# ──────────────────────────────────────────────

def astar(start_id: str, blocked: set[str]) -> dict:
    """
    Run A* from start_id to nearest exit, avoiding blocked edges.

    Returns:
        {
          "path": [node_ids],
          "cost": float,
          "exit": str,
          "nodes_explored": int,
          "found": bool
        }
    """
    if start_id not in NODES:
        return {"found": False, "error": f"Node '{start_id}' not found"}

    if NODES[start_id]["is_exit"]:
        return {"found": True, "path": [start_id], "cost": 0.0,
                "exit": start_id, "nodes_explored": 1}

    # Priority queue: (f_score, node_id)
    open_heap = []
    heapq.heappush(open_heap, (NODES[start_id]["h"], start_id))

    g_score = {start_id: 0.0}
    came_from = {}
    explored = set()
    # Tie-breaking counter
    counter = 0

    while open_heap:
        f, current = heapq.heappop(open_heap)

        if current in explored:
            continue
        explored.add(current)

        if NODES[current]["is_exit"]:
            # Reconstruct path
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            path.reverse()
            return {
                "found": True,
                "path": path,
                "cost": round(g_score[current], 3),
                "exit": current,
                "nodes_explored": len(explored)
            }

        for nb in ADJ.get(current, []):
            # Check blockage (both directions)
            eid_fwd = f"{current}--{nb['to']}"
            eid_rev = f"{nb['to']}--{current}"
            if eid_fwd in blocked or eid_rev in blocked or nb["eid"] in blocked:
                continue

            tentative_g = g_score.get(current, math.inf) + nb["w"]
            if tentative_g < g_score.get(nb["to"], math.inf):
                came_from[nb["to"]] = current
                g_score[nb["to"]] = tentative_g
                f_new = tentative_g + NODES[nb["to"]]["h"]
                heapq.heappush(open_heap, (f_new, nb["to"]))

    return {
        "found": False,
        "path": [],
        "cost": math.inf,
        "exit": None,
        "nodes_explored": len(explored),
        "error": "No path to any exit found — all routes blocked!"
    }


def dijkstra_to_exit(start_id: str, target_id: str, blocked: set[str]) -> dict:
    """Dijkstra from start to a specific target exit."""
    dist = {nid: math.inf for nid in NODES}
    dist[start_id] = 0.0
    prev = {}
    heap = [(0.0, start_id)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        if u == target_id:
            break
        for nb in ADJ.get(u, []):
            eid_fwd = f"{u}--{nb['to']}"
            eid_rev = f"{nb['to']}--{u}"
            if eid_fwd in blocked or eid_rev in blocked or nb["eid"] in blocked:
                continue
            nd = dist[u] + nb["w"]
            if nd < dist[nb["to"]]:
                dist[nb["to"]] = nd
                prev[nb["to"]] = u
                heapq.heappush(heap, (nd, nb["to"]))

    if dist[target_id] == math.inf:
        return {"reachable": False, "path": [], "cost": math.inf}

    # Trace back path
    path = []
    node = target_id
    while node is not None:
        path.append(node)
        node = prev.get(node)
    path.reverse()
    return {"reachable": True, "path": path, "cost": round(dist[target_id], 3)}


def find_all_paths(start_id: str, blocked: set[str]) -> dict:
    """
    Find shortest paths from start to ALL exits.
    Returns all path edges (for blue rendering) and per-exit results.
    """
    all_path_edges = set()
    all_path_nodes = set()
    exit_results = {}

    for eid in EXIT_IDS:
        result = dijkstra_to_exit(start_id, eid, blocked)
        exit_results[eid] = result
        if result["reachable"]:
            path = result["path"]
            all_path_nodes.update(path)
            for i in range(len(path) - 1):
                all_path_edges.add(f"{path[i]}--{path[i+1]}")
                all_path_edges.add(f"{path[i+1]}--{path[i]}")

    return {
        "all_path_edges": list(all_path_edges),
        "all_path_nodes": list(all_path_nodes),
        "exit_results": exit_results
    }


# ──────────────────────────────────────────────
# REST API ENDPOINTS
# ──────────────────────────────────────────────

@app.route("/api/status", methods=["GET"])
def status():
    """Health check."""
    return jsonify({
        "status": "ok",
        "building": "Shopping Complex",
        "algorithm": "A* Search",
        "nodes": len(NODES),
        "edges": len(EDGES),
        "exits": len(EXIT_IDS),
        "blocked_edges": list(BLOCKED_EDGES)
    })


@app.route("/api/graph", methods=["GET"])
def get_graph():
    """Return full graph data: nodes + edges + adjacency info."""
    return jsonify({
        "nodes": list(NODES.values()),
        "edges": EDGES,
        "exit_ids": EXIT_IDS,
        "blocked_edges": list(BLOCKED_EDGES),
        "meta": {
            "rows": ROWS,
            "cols": COLS,
            "svg_width": SVG_W,
            "svg_height": SVG_H
        }
    })


@app.route("/api/solve", methods=["POST"])
def solve():
    """
    Run A* and return optimal path + all alternate paths.

    Request JSON:
        {
            "start": "C3",
            "blocked_edges": ["C1--C2"]   // optional, overrides server state
        }

    Response JSON:
        {
            "start": "C3",
            "optimal": { path, cost, exit, nodes_explored },
            "all_paths": { all_path_edges, all_path_nodes, exit_results },
            "blocked_edges": [...]
        }
    """
    data = request.get_json(force=True)

    start = data.get("start", "").strip().upper()
    if not start:
        return jsonify({"error": "Missing 'start' field"}), 400
    if start not in NODES:
        return jsonify({"error": f"Node '{start}' not in graph"}), 404

    # Merge server-level blocked edges with any request-level overrides
    extra = set(data.get("blocked_edges", []))
    combined_blocked = BLOCKED_EDGES | extra

    optimal = astar(start, combined_blocked)
    all_paths = find_all_paths(start, combined_blocked)

    # Build performance comparison
    dijkstra_result = _dijkstra_all_nodes(start, combined_blocked)

    return jsonify({
        "start": start,
        "optimal": optimal,
        "all_paths": all_paths,
        "blocked_edges": list(combined_blocked),
        "performance": {
            "astar_nodes_explored": optimal.get("nodes_explored", 0),
            "dijkstra_nodes_explored": dijkstra_result["nodes_explored"],
            "total_nodes": len(NODES),
            "reduction_pct": round(
                (1 - optimal.get("nodes_explored", 0) / max(dijkstra_result["nodes_explored"], 1)) * 100, 1
            )
        }
    })


def _dijkstra_all_nodes(start_id: str, blocked: set[str]) -> dict:
    """Run Dijkstra to ALL nodes (for comparison stats)."""
    dist = {nid: math.inf for nid in NODES}
    dist[start_id] = 0.0
    heap = [(0.0, start_id)]
    explored = 0

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        explored += 1
        for nb in ADJ.get(u, []):
            eid_fwd = f"{u}--{nb['to']}"
            eid_rev = f"{nb['to']}--{u}"
            if eid_fwd in blocked or eid_rev in blocked or nb["eid"] in blocked:
                continue
            nd = dist[u] + nb["w"]
            if nd < dist[nb["to"]]:
                dist[nb["to"]] = nd
                heapq.heappush(heap, (nd, nb["to"]))

    return {"nodes_explored": explored, "dist": dist}


@app.route("/api/block", methods=["POST"])
def block_edge():
    """
    Persistently block an edge on the server.

    Request JSON: { "edge_id": "C1--C2" }
    """
    data = request.get_json(force=True)
    eid = data.get("edge_id", "").strip()
    if not eid:
        return jsonify({"error": "Missing 'edge_id'"}), 400

    BLOCKED_EDGES.add(eid)
    return jsonify({"blocked": eid, "all_blocked": list(BLOCKED_EDGES)})


@app.route("/api/block/<path:edge_id>", methods=["DELETE"])
def unblock_edge(edge_id):
    """Unblock a previously blocked edge."""
    BLOCKED_EDGES.discard(edge_id)
    # Also try reverse direction
    parts = edge_id.split("--")
    if len(parts) == 2:
        BLOCKED_EDGES.discard(f"{parts[1]}--{parts[0]}")
    return jsonify({"unblocked": edge_id, "all_blocked": list(BLOCKED_EDGES)})


@app.route("/api/block", methods=["DELETE"])
def clear_all_blocks():
    """Remove all edge blocks."""
    BLOCKED_EDGES.clear()
    return jsonify({"message": "All blocks cleared", "all_blocked": []})


@app.route("/api/heuristics", methods=["GET"])
def get_heuristics():
    """Return heuristic values for all nodes (useful for debugging)."""
    return jsonify({
        nid: {"h": node["h"], "x": node["x"], "y": node["y"]}
        for nid, node in NODES.items()
    })


@app.route("/api/neighbors/<node_id>", methods=["GET"])
def get_neighbors(node_id):
    """Return all neighbors of a given node."""
    node_id = node_id.upper()
    if node_id not in NODES:
        return jsonify({"error": f"Node '{node_id}' not found"}), 404
    return jsonify({
        "node": node_id,
        "neighbors": ADJ.get(node_id, []),
        "blocked_neighbors": [
            nb for nb in ADJ.get(node_id, [])
            if nb["eid"] in BLOCKED_EDGES
               or f"{node_id}--{nb['to']}" in BLOCKED_EDGES
               or f"{nb['to']}--{node_id}" in BLOCKED_EDGES
        ]
    })


# ──────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  Shopping Complex — Evacuation Planner API")
    print("=" * 60)
    print(f"  Graph: {len(NODES)} nodes, {len(EDGES)} edges, {len(EXIT_IDS)} exits")
    print(f"  API: http://localhost:5000/api/")
    print(f"  Health: http://localhost:5000/api/status")
    print("=" * 60)
    app.run(debug=True, host="0.0.0.0", port=5000)

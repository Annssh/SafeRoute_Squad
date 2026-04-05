# 🏬 Shopping Complex — A* Evacuation Planner

An interactive evacuation route planner for a shopping complex, powered by the **A\* search algorithm**. The system finds the optimal path from any location inside the building to the nearest exit, with real-time visualization and dynamic route blocking.

---

## 📸 Overview

The project models a shopping complex as a weighted graph (5 rows × 7 columns = 35 interior nodes + 8 exits) and uses A\* search to compute the shortest, safest evacuation path. Users can simulate blocked corridors, compare A\* efficiency against Dijkstra, and interactively explore all alternate routes.

---

## ✨ Features

- **A\* Pathfinding** — Finds the optimal path to the nearest exit using Euclidean-distance heuristics
- **Interactive SVG Map** — Click any node on the map to set it as the evacuation start point
- **Edge Blocking** — Simulate blocked corridors or stairwells and watch the route recalculate
- **All-Routes View** — Visualizes shortest paths to every exit simultaneously (blue = alternate, red = optimal)
- **8 Exit Points** — Exits on all four walls; the planner always targets the nearest reachable one
- **Real-time API Status** — Connection indicator shows live backend health

---

## 🗂️ Project Structure

```
.
├── app.py        # Flask REST API — graph construction, A*, edge blocking
└── index.html    # Frontend — SVG map, controls, result panel (vanilla HTML/CSS/JS)
```

---

## 🚀 Getting Started

### Prerequisites

- Python 3.8+
- pip

### Installation

```bash
# 1. Clone the repository
git clone https://github.com/Annssh/SafeRoute_Squad

# 2. Install dependencies
pip install flask flask-cors

# 3. Start the backend
python app.py
```

The API will be available at `http://localhost:5000`.

### Running the Frontend

Open `index.html` directly in your browser — no build step required.

> Make sure the Flask server is running before opening the frontend.

---

## 🔌 API Reference

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/api/status` | Health check — returns node/edge/exit counts |
| `GET` | `/api/graph` | Full graph data (nodes, edges, adjacency) |
| `POST` | `/api/solve` | Run A\* from a start node; returns optimal + all paths |
| `POST` | `/api/block` | Persistently block an edge by ID |
| `DELETE` | `/api/block/<edge_id>` | Unblock a specific edge |
| `DELETE` | `/api/block` | Clear all blocked edges |
| `GET` | `/api/heuristics` | Heuristic values for all nodes (debug) |
| `GET` | `/api/neighbors/<node_id>` | Neighbors of a given node |

### Example: Solve from node C3

```bash
curl -X POST http://localhost:5000/api/solve \
  -H "Content-Type: application/json" \
  -d '{"start": "C3"}'
```

**Response:**
```json
{
  "start": "C3",
  "optimal": {
    "found": true,
    "path": ["C3", "C2", "C1", "EXIT1"],
    "cost": 5.2,
    "exit": "EXIT1",
    "nodes_explored": 8
  },
  "performance": {
    "astar_nodes_explored": 8,
    "dijkstra_nodes_explored": 43,
    "reduction_pct": 81.4
  }
}
```

### Example: Block an edge

```bash
curl -X POST http://localhost:5000/api/block \
  -H "Content-Type: application/json" \
  -d '{"edge_id": "C1--C2"}'
```

---

## 🧠 How It Works

### Graph Model

The shopping complex is represented as an undirected weighted graph:

- **Interior nodes:** 35 nodes in a 5×7 grid (rows A–E, columns 1–7)
- **Exit nodes:** 8 exits placed on the outer walls (EXIT1–EXIT8)
- **Edges:** Horizontal corridor edges, vertical stairwell edges, and exit connectors — each with a realistic traversal weight (in minutes)

### A\* Algorithm

The heuristic `h(n)` is the scaled Euclidean distance from a node to the nearest exit:

```
h(n) = min_distance_to_any_exit / SCALE
```

where `SCALE = 55.0` pixels/minute. This makes A\* admissible and guarantees an optimal path.

### Performance

Because A\* is guided by the heuristic, it typically explores only **15–25% of nodes** that a full Dijkstra search would visit — displayed live in the UI stats panel.

---

## 🖥️ UI Guide

| Element | Action |
|---------|--------|
| **Node on map** | Click to set as start location |
| **Start Node dropdown** | Manually select a start node |
| **Block Edge dropdown** | Select an edge to mark as blocked |
| **Find Optimal Path** | Run A\* and highlight the best route |
| **Show All Paths** | Highlight shortest paths to all 8 exits |
| **Reset** | Clear all results and blocks |

**Color legend:**
- 🟢 Green — Start node / target exit
- 🔴 Red — Optimal A\* path
- 🔵 Blue — Alternate paths to other exits
- ⬛ Dashed red — Blocked edge

---

## 🛠️ Tech Stack

| Layer | Technology |
|-------|------------|
| Backend | Python, Flask, flask-cors |
| Algorithm | A\* Search |
| Frontend | Vanilla HTML5, CSS3, JavaScript |
| Visualization | Inline SVG (dynamically generated) |---

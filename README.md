**Path Planning Environment — README**

**Overview**
- **Project:** A collection of grid- and sampling-based path planning implementations. The runner is [main/main.py](main/main.py).
- **What `main.py` does:** creates an `Environment`, places rectangular obstacles with `env.add_rect(x, y, w, h)`, sets `start` and `goal`, runs multiple planners, and plots results.

**How to run**
- **Install deps:** `pip install -r requirements.txt` (includes `numpy`, `matplotlib`).
- **Run (recommended):** `python -m main.main` from the repository root.
- **Run directly (also supported):** `python main/main.py` — the script includes a small `sys.path` fallback so `utils` imports work.

**Environment parameters that change planner behavior**
- **Map size:** configured in `Environment(width, height)` (see [main/main.py](main/main.py)). Larger maps increase search space and sampling area.
- **Obstacles:** `env.add_rect(x,y,w,h)` adds axis-aligned obstacles. Adding/removing or moving obstacles can block or open shortest paths; dynamic changes matter for incremental planners (D*, D* Lite).
- **Collision model:** planners use `env.check_collision` and `env.check_collision_line`. How obstacles are inflated or how collision is checked (point vs. line) affects feasible paths.
- **Resolution (`reso`) / grid granularity:** grid-based planners (Dijkstra, A*, D*, D* Lite, Hybrid A* when discretized) depend on the step/resolution used when expanding nodes. Finer resolution -> more precise paths but larger search.
- **Motion primitives / steering:** motion sets (e.g., 8-neighbor moves vs. continuous steering in Hybrid A*) and step sizes affect path smoothness and feasibility.
- **Sampling & parameters (RRT / RRT*):** `expand_dis`, `goal_sample_rate`, `max_iter`, and `connect_circle_dist` control exploration density, success probability, and runtime.

**Planner notes (what to tune to influence results)**
- **Dijkstra / A***: tune `reso`, heuristic weight (A* uses Euclidean cost), and obstacle inflation. A* is directed by heuristics (faster than Dijkstra when heuristic is admissible).
- **Hybrid A***: tune steering angle resolution, step length, and yaw resolution. It models vehicle orientation and non-holonomic constraints — good for car-like robots.
- **Field D*** (field-based interpolation): works on continuous fields derived from grid costs to produce smoother interpolated paths — tune interpolation parameters.
- **D* Lite**: designed for online replanning — useful when obstacles change during execution. Tune grid resolution and how you update changed cells.
- **RRT / RRT***: tune sampling iterations, `expand_dis` (step length), `goal_sample_rate` (bias), and `connect_circle_dist` (for RRT*). RRT is fast and finds a feasible path; RRT* adds rewiring to asymptotically improve path cost.

**Short theoretical summaries**
- **Dijkstra:** exhaustive shortest-path on weighted graphs; explores in order of increasing path cost; optimal but can be slow on large grids because it expands uniformly.
- **A***: best-first search that adds a heuristic (e.g., Euclidean distance) to guide exploration; when the heuristic is admissible and consistent, A* is optimal and faster than Dijkstra.
- **Hybrid A***: extends A* to a state space with orientation (x, y, theta) and uses motion primitives or steering integration; suitable for non-holonomic vehicles and produces more feasible trajectories.
- **Field D***: augments discrete planners with continuous interpolation over a potential field derived from grid costs; yields smoother trajectories by following field gradients rather than discrete cell centers.
- **D* Lite:** an incremental version of A*/D* useful for dynamic environments. It reuses previous search effort to quickly repair paths after environment changes.
- **RRT (Rapidly-exploring Random Trees):** a sampling-based planner that grows a tree by sampling random states and connecting them to the nearest tree node. Good for high-dimensional or continuous spaces; probabilistically complete.
- **RRT\*:** an optimal variant of RRT that performs rewiring within a neighborhood to improve path cost over time; asymptotically optimal (cost approaches optimum as samples → ∞).

**Practical tips**
- For deterministic, grid-based shortest paths use A* with an admissible Euclidean heuristic and an appropriate `reso`.
- For car-like robots prefer `hybrid_a_star` and tune steering/yaw resolution.
- For cluttered, continuous spaces use RRT/RRT* and increase `max_iter` and `connect_circle_dist` for better solutions.
- When running experiments, change one parameter at a time (e.g., `reso`, `expand_dis`, `goal_sample_rate`) and log runtime and path cost.

**Files of interest**
- Runner: [main/main.py](main/main.py)
- Utilities and planners: [utils/](utils/)

If you want, I can also:
- Add example configurations (YAML) for different scenarios.
- Add a small script to benchmark planners over varied obstacle maps and parameters.
# Path_Planning_env
General Path Planning algorithms implementation

import sys
import os

# When executing `python main/main.py` directly, the script's directory becomes
# sys.path[0] (the `main/` folder), so sibling package `utils` isn't found.
# Ensure project root is on sys.path so `from utils...` imports work both when
# running as a script and when running as a module (`python -m main.main`).
if __package__ is None:
    project_root = os.path.dirname(os.path.dirname(__file__))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

import matplotlib.pyplot as plt
from utils.common import Environment
from utils.djikstra import dijkstra_search
from utils.a_star import a_star_search
from utils.hybrid_a_star import hybrid_a_star_search
from utils.d_star import field_d_star_search
from utils.d_star_lite import d_star_lite_search
from utils.rrt import RRT
from utils.rrt_star import RRTStar

def main():
    # 1. Setup Environment
    env = Environment(50, 30)
    env.add_rect(10, 15, 10, 10)
    env.add_rect(30, 0, 5, 15)
    env.add_rect(10, 5, 5, 5)
    env.add_rect(25, 20, 10, 5)

    start = (2, 2)
    goal = (45, 25)
    
    # 2. Run All Algorithms
    print("Running Dijkstra...")
    path_dijkstra = dijkstra_search(env, start, goal)
    
    print("Running A*...")
    path_astar = a_star_search(env, start, goal)
    
    print("Running Hybrid A*...")
    path_hybrid = hybrid_a_star_search(env, start, goal)
    
    print("Running Field D*...")
    path_dstar = field_d_star_search(env, start, goal)
    
    print("Running D* Lite...")
    path_dlite = d_star_lite_search(env, start, goal)
    
    print("Running RRT...")
    rrt = RRT(env, start, goal)
    path_rrt = rrt.planning()
    
    print("Running RRT*...")
    rrt_star = RRTStar(env, start, goal)
    path_rrt_star = rrt_star.planning()

    # 3. Plotting
    fig, axes = plt.subplots(3, 3, figsize=(18, 15))
    axes = axes.flatten()
    
    algos = [
        ("Dijkstra", path_dijkstra, "blue"),
        ("A*", path_astar, "green"),
        ("Hybrid A*", path_hybrid, "orange"),
        ("Field D* (Interpolated)", path_dstar, "purple"),
        ("D* Lite (Grid)", path_dlite, "cyan"),
        ("RRT (Sampling)", path_rrt, "red"),
        ("RRT* (Optimized)", path_rrt_star, "magenta")
    ]
    
    # Plot Logic
    for i, (name, path, color) in enumerate(algos):
        ax = axes[i]
        ax.set_xlim(0, 50); ax.set_ylim(0, 30)
        
        # Draw Obstacles
        for (ox, oy, ow, oh) in env.obstacles:
            ax.add_patch(plt.Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5))
            
        ax.plot(start[0], start[1], "go", markersize=8, label="Start")
        ax.plot(goal[0], goal[1], "rx", markersize=8, label="Goal")
        
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, "-", color=color, linewidth=2, label="Path")
            if "RRT" not in name: # Dots for grid based
                ax.scatter(px, py, s=10, color=color)
        else:
            ax.text(25, 15, "No Path Found", ha='center')
            
        ax.set_title(name)
        ax.grid(True)

    # Hide empty subplots
    for j in range(len(algos), len(axes)):
        axes[j].axis('off')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
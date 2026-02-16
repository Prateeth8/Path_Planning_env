import heapq
import math
import numpy as np

def d_star_lite_search(env, start, goal):
    # Implemented as Backward A* for static map
    s_start = (round(start[0]), round(start[1]))
    s_goal = (round(goal[0]), round(goal[1]))
    
    g = {s_goal: 0}
    pq = [(0, s_goal)]
    came_from = {s_goal: None}
    visited = set()
    
    motions = [(1,0), (0,1), (-1,0), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]

    while pq:
        cost, curr = heapq.heappop(pq)
        
        if curr in visited: continue
        visited.add(curr)
        
        if curr == s_start:
            # Reconstruct Path
            path = []
            c = s_start
            while c is not None:
                path.append(c)
                if c == s_goal: break
                c = came_from.get(c)
            return path

        for dx, dy in motions:
            nx, ny = curr[0]+dx, curr[1]+dy
            if env.check_collision(nx, ny): continue
            
            new_cost = cost + math.hypot(dx, dy)
            if (nx, ny) not in g or new_cost < g[(nx, ny)]:
                g[(nx, ny)] = new_cost
                came_from[(nx, ny)] = curr
                h = math.hypot(nx-s_start[0], ny-s_start[1])
                heapq.heappush(pq, (new_cost+h, (nx, ny)))
    return []
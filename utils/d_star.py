import heapq
import math
import numpy as np

def field_d_star_search(env, start, goal):
    # Simplified Field D* / Theta* implementation for brevity
    # It allows Line-Of-Sight connections to parents
    
    s_start = (round(start[0], 2), round(start[1], 2))
    s_goal = (round(goal[0], 2), round(goal[1], 2))
    
    g = {s_start: 0}
    parent = {s_start: s_start}
    pq = []
    heapq.heappush(pq, (0, s_start))
    closed = set()
    
    while pq:
        _, curr = heapq.heappop(pq)
        if curr in closed: continue
        closed.add(curr)
        
        if math.hypot(curr[0]-s_goal[0], curr[1]-s_goal[1]) < 1.0:
            path = [s_goal]
            t = curr
            while t != s_start:
                path.append(t)
                t = parent[t]
            path.append(s_start)
            return path[::-1]
            
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx==0 and dy==0: continue
                nx, ny = curr[0]+dx, curr[1]+dy
                
                if env.check_collision(nx, ny): continue
                
                # Field D* Logic (Theta* shortcut):
                # Check line of sight to Parent
                p = parent[curr]
                dist_p = math.hypot(nx-p[0], ny-p[1])
                
                # If we can see the parent directly, skip the current node (create straight line)
                if not env.check_collision_line(nx, ny, p[0], p[1], step=0.5):
                    new_g = g[p] + dist_p
                    if (nx, ny) not in g or new_g < g[(nx, ny)]:
                        g[(nx, ny)] = new_g
                        parent[(nx, ny)] = p
                        h = math.hypot(nx-s_goal[0], ny-s_goal[1])
                        heapq.heappush(pq, (new_g + h, (nx, ny)))
                else:
                    # Standard Grid transition
                    dist_u = math.hypot(nx-curr[0], ny-curr[1])
                    new_g = g[curr] + dist_u
                    if (nx, ny) not in g or new_g < g[(nx, ny)]:
                        g[(nx, ny)] = new_g
                        parent[(nx, ny)] = curr
                        h = math.hypot(nx-s_goal[0], ny-s_goal[1])
                        heapq.heappush(pq, (new_g + h, (nx, ny)))
    return []
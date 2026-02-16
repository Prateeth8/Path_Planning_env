import heapq
import math
from .common import Node

def a_star_search(env, start, goal, reso=1.0):
    start_node = Node(start[0], start[1], 0, -1)
    
    open_set = { (round(start[0], 2), round(start[1], 2)): start_node }
    closed_set = {}
    pq = []
    heapq.heappush(pq, (0, start[0], start[1]))
    
    motions = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    while pq:
        _, cx, cy = heapq.heappop(pq)
        cx, cy = round(cx, 2), round(cy, 2)
        
        if (cx, cy) in closed_set: continue
        current = open_set.get((cx, cy))
        
        if math.hypot(cx - goal[0], cy - goal[1]) < reso:
            path = []
            while current.parent_index != -1:
                path.append((current.x, current.y))
                current = closed_set[current.parent_index]
            path.append(start)
            return path[::-1]

        del open_set[(cx, cy)]
        closed_set[(cx, cy)] = current

        for dx, dy in motions:
            nx, ny = cx + dx*reso, cy + dy*reso
            nx, ny = round(nx, 2), round(ny, 2)

            if env.check_collision_line(cx, cy, nx, ny): continue
            
            new_cost = current.cost + math.hypot(dx, dy)
            h_cost = math.hypot(nx - goal[0], ny - goal[1])
            
            if (nx, ny) in closed_set: continue
            
            if (nx, ny) not in open_set or open_set[(nx, ny)].cost > new_cost:
                new_node = Node(nx, ny, new_cost, (cx, cy))
                open_set[(nx, ny)] = new_node
                heapq.heappush(pq, (new_cost + h_cost, nx, ny))
    return []
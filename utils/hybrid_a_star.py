import heapq
import math
import numpy as np
from .common import Node

def hybrid_a_star_search(env, start, goal):
    reso = 1.0
    yaw_reso = np.deg2rad(15.0)
    
    start_node = Node(start[0], start[1], 0, -1, 0)
    
    pq = []
    open_set = {}
    closed_set = {}
    
    def get_idx(node):
        return (round(node.x/reso), round(node.y/reso), round(node.theta/yaw_reso))

    s_idx = get_idx(start_node)
    open_set[s_idx] = start_node
    heapq.heappush(pq, (0, s_idx))
    
    while pq:
        cost, c_idx = heapq.heappop(pq)
        if c_idx in closed_set: continue
        current = open_set[c_idx]
        closed_set[c_idx] = current
        
        if math.hypot(current.x - goal[0], current.y - goal[1]) < 2.0:
            path = []
            curr = current
            while curr.parent_index != -1:
                path.append((curr.x, curr.y))
                curr = closed_set.get(curr.parent_index)
            return path[::-1]

        # Steering inputs
        for steer in [-0.5, 0, 0.5]:
            step = 2.0
            theta = current.theta + steer
            nx = current.x + step * math.cos(theta)
            ny = current.y + step * math.sin(theta)
            
            # Check midpoint for collision (safety)
            mx = current.x + (step/2) * math.cos(theta)
            my = current.y + (step/2) * math.sin(theta)
            
            if env.check_collision(nx, ny) or env.check_collision(mx, my): continue
            
            new_node = Node(nx, ny, current.cost + step, c_idx, theta)
            n_idx = get_idx(new_node)
            
            if n_idx in closed_set: continue
            
            h = math.hypot(nx - goal[0], ny - goal[1])
            if n_idx not in open_set or open_set[n_idx].cost > new_node.cost:
                open_set[n_idx] = new_node
                heapq.heappush(pq, (new_node.cost + h, n_idx))
    return []
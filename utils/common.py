# common.py
import numpy as np
import math

class Node:
    def __init__(self, x, y, cost=0.0, parent_index=-1, theta=0.0):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.theta = theta

class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []
        self.grid_cost = np.ones((int(width) + 1, int(height) + 1)) 

    def add_rect(self, x, y, w, h):
        self.obstacles.append([x, y, w, h])
        # Mark grid cost for grid-based algos
        for i in range(int(x), int(x+w)+1):
            for j in range(int(y), int(y+h)+1):
                if 0 <= i <= self.width and 0 <= j <= self.height:
                    self.grid_cost[i, j] = float('inf')

    def check_collision(self, x, y):
        """Strict collision check (includes buffer)"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        for (ox, oy, ow, oh) in self.obstacles:
            # Buffer of 0.5 to prevent skimming edges
            if ox - 0.5 <= x <= ox + ow + 0.5 and oy - 0.5 <= y <= oy + oh + 0.5:
                return True
        return False

    def check_collision_line(self, x1, y1, x2, y2, step=0.5):
        """Checks a line segment for collision"""
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist == 0: return self.check_collision(x1, y1)
        
        n_checks = int(dist / step)
        for i in range(n_checks + 1):
            t = i / max(n_checks, 1)
            px = x1 + (x2 - x1) * t
            py = y1 + (y2 - y1) * t
            if self.check_collision(px, py):
                return True
        return False
import math
from .rrt import RRT

class RRTStar(RRT):
    def __init__(self, env, start, goal, expand_dis=2.0, connect_circle_dist=10.0):
        super().__init__(env, start, goal, expand_dis)
        self.connect_circle_dist = connect_circle_dist

    def planning(self):
        self.node_list = [self.start]
        for i in range(500):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            
            if self.env.check_collision_line(self.node_list[nearest_ind].x, self.node_list[nearest_ind].y, new_node.x, new_node.y):
                continue
                
            near_inds = self.find_near_nodes(new_node)
            new_node = self.choose_parent(new_node, near_inds)
            
            if new_node:
                self.node_list.append(new_node)
                self.rewire(new_node, near_inds)
        
        # Find best path to goal
        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        return []

    def choose_parent(self, new_node, near_inds):
        if not near_inds: return None
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            d, _ = self.calc_distance_and_angle(near_node, new_node)
            if not self.env.check_collision_line(near_node.x, near_node.y, new_node.x, new_node.y):
                costs.append(near_node.cost + d)
            else:
                costs.append(float("inf"))
        
        min_cost = min(costs)
        if min_cost == float("inf"): return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node.cost = min_cost
        new_node.parent_index = min_ind
        return new_node

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            d, _ = self.calc_distance_and_angle(new_node, near_node)
            if new_node.cost + d < near_node.cost:
                if not self.env.check_collision_line(new_node.x, new_node.y, near_node.x, near_node.y):
                    near_node.parent_index = self.node_list.index(new_node)
                    near_node.cost = new_node.cost + d

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        r = min(r, self.expand_dis * 2)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_inds = [i for i, d in enumerate(dist_list) if d <= r**2]
        return near_inds

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [i for i, d in enumerate(dist_to_goal_list) if d <= self.expand_dis]
        if not goal_inds: return None
        min_cost = min([self.node_list[i].cost for i in goal_inds])
        for i in goal_inds:
            if self.node_list[i].cost == min_cost: return i
        return None
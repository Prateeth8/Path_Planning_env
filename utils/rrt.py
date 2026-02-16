import math
import random
from .common import Node

class RRT:
    def __init__(self, env, start, goal, expand_dis=2.0, goal_sample_rate=5):
        self.env = env
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.node_list = []

    def planning(self):
        self.node_list = [self.start]
        for i in range(500): # max_iter
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if not self.env.check_collision_line(nearest_node.x, nearest_node.y, new_node.x, new_node.y):
                self.node_list.append(new_node)
                
                if self.calc_dist_to_goal(new_node.x, new_node.y) <= self.expand_dis:
                    final_node = self.steer(new_node, self.goal, self.expand_dis)
                    if not self.env.check_collision_line(new_node.x, new_node.y, final_node.x, final_node.y):
                         return self.generate_final_course(len(self.node_list) - 1)
        return []

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        
        if extend_length > d: extend_length = d
        
        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)
        new_node.parent_index = self.node_list.index(from_node)
        return new_node

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            return Node(random.uniform(0, self.env.width), random.uniform(0, self.env.height))
        return Node(self.goal.x, self.goal.y)

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        return dlist.index(min(dlist))

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
        
    def calc_dist_to_goal(self, x, y):
        return math.hypot(x - self.goal.x, y - self.goal.y)

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent_index != -1:
            path.append([node.x, node.y])
            node = self.node_list[node.parent_index]
        path.append([self.start.x, self.start.y])
        return path[::-1]
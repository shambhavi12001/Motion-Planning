import numpy as np
from scipy.spatial import cKDTree

class Node:
    def __init__(self, coord = None, parent = None, g = 0.0):
        self.coord = coord
        self.parent = parent
        self.g = g
        
class MyPlanner:
  __slots__ = ['boundary', 'blocks', 'nodes', 'coordinates', 'tree']
  
  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks
    self.nodes = None
    self.coordinates = None
    self.tree = None

  def sample_free(self, iteration, nodes, boundary, blocks, goal, bias):
    if np.random.rand() < bias:
        return Node(goal, None, float("inf"))
    while True:
        node_rand = Node(np.array([
            np.random.uniform(boundary[0, 0], boundary[0, 3]),
            np.random.uniform(boundary[0, 1], boundary[0, 4]),
            np.random.uniform(boundary[0, 2], boundary[0, 5])
        ]), None, float("inf"))
        collision = False
        for block in blocks:
            if (
                node_rand.coord[0] >= block[0] and node_rand.coord[0] <= block[3] and
                node_rand.coord[1] >= block[1] and node_rand.coord[1] <= block[4] and
                node_rand.coord[2] >= block[2] and node_rand.coord[2] <= block[5]
            ):
                collision = True
                break
        if not collision:
            return node_rand
  '''
  def nearest(self, nodes, node_rand):
    min_dist = float("inf")
    node_nearest = None
    for node in nodes:
        dist = np.linalg.norm(node.coord - node_rand.coord)
        if dist < min_dist:
            min_dist, node_nearest = dist, node
    return node_nearest
  '''
  
  def steer(self, node_nearest, node_rand, epsilon):
    dist = np.linalg.norm(node_rand.coord - node_nearest.coord)

    if dist <= epsilon:
        return Node(node_rand.coord, node_nearest, float("inf"))

    direction = (node_rand.coord - node_nearest.coord) / dist
    new_node = Node(node_nearest.coord + epsilon * direction, node_nearest, float("inf"))
    return new_node
    
  def collision_check(self, path, blocks, eps=1e-9):
    for i in range(len(path) - 1):
        for block in blocks:
            t_min, t_max = 0.0, 1.0
            collision = True
            for coord in range(3):
                d = (path[i + 1][coord] - path[i][coord])
                if abs(d) < eps:
                    if (path[i][coord] < block[coord]) or (path[i][coord] > block[coord + 3]):
                        collision = False
                        break
                else:
                    t_start = (block[coord] - path[i][coord])/d
                    t_end = (block[coord + 3] - path[i][coord])/d
                    if path[i][coord] > path[i+1][coord]:
                        t_start, t_end = t_end, t_start
                    t_min = max(t_min, t_start)
                    t_max = min(t_max, t_end)
                    if t_min > t_max:
                        collision = False
                        break
            if collision:
                return True
    return False

  def cost(self, node, new_node):
    return np.linalg.norm(node.coord - new_node.coord)
  
  def radius_threshold(self, boundary, blocks, n):
    d = 3
    vol_unit_3_ball = 4.0 / 3.0 * np.pi
    vol_boundary = (boundary[0, 3] - boundary[0,0]) * (boundary[0, 4] - boundary[0,1]) * (boundary[0, 5] - boundary[0,2])
    vol_blocks = 0
    for block in blocks:
        vol_blocks += ((block[3] - block[0]) * (block[4] - block[1]) * (block[5] - block[2]))
    vol_c_free = vol_boundary - vol_blocks
    n = max(n, 2)
    radius = 2 * ((1 + (1/d)) ** (1/d)) * ((vol_c_free/vol_unit_3_ball) ** (1/d)) * ((np.log(n)/n) ** (1/d))
    return radius
  
  '''
  def near(self, nodes, new_node, radius):
    near_nodes = []
    for node in nodes:
        if self.cost(node, new_node) < radius:
            near_nodes.append(node)
    return near_nodes
  '''
  
  def backtrack_path(self, goal_node, blocks):
    path = [goal_node.coord]
    curr = goal_node.parent
    prev = goal_node
    while curr:
        if self.collision_check([path[-1], curr.coord], blocks):
            path.append(prev.coord)
        prev = curr
        curr = curr.parent
    path.append(prev.coord)
    return path
  
  def plan(self, start, goal, epsilon=0.5, bias=0.1, max_iter=100000, return_info=False):
    
    if self.nodes is None:
        nodes = [Node(start, None, 0.0)]
        coordinates = np.array([start])
        tree = cKDTree(coordinates)
    else:
        nodes = self.nodes
        coordinates = self.coordinates
        tree = self.tree
    
    goal_node = None
    actual_iterations = max_iter
    
    for i in range(max_iter):
        node_rand = self.sample_free(i, nodes, self.boundary, self.blocks, goal, bias)
        _, nearest_i = tree.query(node_rand.coord)
        node_nearest = nodes[nearest_i]
        new_node = self.steer(node_nearest, node_rand, epsilon)

        if self.collision_check([new_node.coord, node_nearest.coord], self.blocks):
            continue

        radius = self.radius_threshold(self.boundary, self.blocks, len(nodes))
        near_i = tree.query_ball_point(new_node.coord, radius)
        near_nodes = [nodes[i] for i in near_i]

        cost_min = node_nearest.g + self.cost(node_nearest, new_node)
        node_min = node_nearest

        for near_node in near_nodes:
            if not self.collision_check([near_node.coord, new_node.coord], self.blocks):
                new_cost = near_node.g + self.cost(near_node, new_node)
                if new_cost < cost_min:
                    cost_min = new_cost
                    node_min = near_node

        new_node.parent = node_min
        new_node.g = cost_min
        nodes.append(new_node)
        
        coordinates = np.vstack((coordinates, new_node.coord))
        tree = cKDTree(coordinates)
        
        for near_node in near_nodes:
            if not self.collision_check([new_node.coord, near_node.coord], self.blocks):
                new_cost = new_node.g + self.cost(new_node, near_node)
                if new_cost < near_node.g:
                    near_node.parent = new_node
                    near_node.g = new_cost

        if np.linalg.norm(new_node.coord - goal) < 1e-9:
            goal_node = new_node
            actual_iterations = i + 1
            break
            
    self.nodes = nodes
    self.coordinates = coordinates
    self.tree = tree

    if goal_node is None:
        if return_info:
            return None, {
                "success": False,
                "iterations": actual_iterations,
                "explored_nodes": len(nodes),
                "epsilon": epsilon,
                "goal_bias": bias,
                "max_iter": max_iter,
                "path_length": None
            }
        return None

    path = self.backtrack_path(goal_node, self.blocks)
    path.reverse()
    path = np.array(path)

    path_length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))

    if return_info:
        return path, {
            "success": True,
            "iterations": actual_iterations,
            "explored_nodes": len(nodes),
            "epsilon": epsilon,
            "goal_bias": bias,
            "max_iter": max_iter,
            "path_length": path_length
        }

    return path

    
'''
  
  def plan(self,start,goal):
    path = [start]
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k]
        
        # Check if this direction is valid
        if( next[0] < self.boundary[0,0] or next[0] > self.boundary[0,3] or \
            next[1] < self.boundary[0,1] or next[1] > self.boundary[0,4] or \
            next[2] < self.boundary[0,2] or next[2] > self.boundary[0,5] ):
          continue
        
        valid = True
        for k in range(self.blocks.shape[0]):
          if( next[0] >= self.blocks[k,0] and next[0] <= self.blocks[k,3] and\
              next[1] >= self.blocks[k,1] and next[1] <= self.blocks[k,4] and\
              next[2] >= self.blocks[k,2] and next[2] <= self.blocks[k,5] ):
            valid = False
            break
        if not valid:
          continue
        
        # Update next node
        disttogoal = sum((next - goal)**2)
        if( disttogoal < mindisttogoal):
          mindisttogoal = disttogoal
          node = next
      
      if node is None:
        break
      
      path.append(node)
      
      # Check if done
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)

'''

# priority queue for OPEN list

from pqdict import pqdict
import math
import numpy as np

import copy

class environment:
    def __init__(self, boundary, blocks, start, goal):
        self.boundary = boundary
        self.blocks = blocks
        self.start = start
        self.goal = goal
        self.dR = self.makeGraph()

    def makeGraph(self):
        numofdirs = 26
        [dX, dY, dZ] = np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])
        dR = np.vstack((dX.flatten(), dY.flatten(), dZ.flatten()))
        dR = np.delete(dR, 13, axis=1)
        dR = dR

        return dR

    def getNeighbours(self, current_node, resolution=0.5):

        neighbours = []
        for k in range(len(self.dR[1])):

            next = current_node + self.dR[:,k]*resolution
            # check if next_node is within the boundary
            if( next[0] < self.boundary[0,0] or next[0] > self.boundary[0,3] or \
                next[1] < self.boundary[0,1] or next[1] > self.boundary[0,4] or \
                next[2] < self.boundary[0,2] or next[2] > self.boundary[0,5] ):
                continue
            # check if the path to next_node is blocked
            path = np.vstack((current_node, next))

            collision = collisionDetection(path, self.blocks)
            if collision:
                continue
            neighbours.append(next)
        return neighbours

    def getHeuristic(self, coord):
        return np.linalg.norm(coord - self.goal)


class AStarNode(object):
  def __init__(self, pqkey, coord, hval, epsilon=1):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.closed = False
    self.epsilon = epsilon

  def __lt__(self, other):
    return (self.g + self.h*self.epsilon) < (other.g + other.h*self.epsilon)


class AStar(object):
  def __init__(self, boundary, blocks, start, goal, epsilon=1, start_resolution=0.5, end_resolution=0.1):
    self.environment = environment(boundary, blocks, start, goal)
    self.epsilon = epsilon
    self.start_resolution = start_resolution
    self.end_resolution = end_resolution

  def plan(self,):
    # Initialize the open list (priority queue) and closed list (dictionary)
    OPEN = pqdict()
    CLOSED = {}

    # Initialize the start node
    start_coord = self.environment.start
    start_node = AStarNode(tuple(start_coord), start_coord, self.environment.getHeuristic(start_coord), self.epsilon)
    start_node.g = 0
    OPEN[start_node.pqkey] = start_node

    while OPEN:
      # Pop the item with the lowest f value
      curr_key, curr = OPEN.popitem()
      # If the goal is reached, reconstruct the path
      if np.linalg.norm(curr.coord-self.environment.goal) <= 0.1:
        print("the number of nodes expanded is: ", len(CLOSED))
        return AStar.reconstruct_path(curr)
      # Insert current node into CLOSED
      CLOSED[curr_key] = curr
      curr.closed = True
      if np.linalg.norm(curr.coord-self.environment.goal) <= 0.5:
          resolution = self.end_resolution
      else:
          resolution = self.start_resolution
      # Generate neighbors
      for neighbor_coord in self.environment.getNeighbours(curr.coord, resolution=resolution):
        neighbor_key = tuple(neighbor_coord)
        tentative_g = curr.g + np.linalg.norm(neighbor_coord - curr.coord)
        if neighbor_key in CLOSED:
            pass

        # check all the neighbors not in closed
        elif neighbor_key in OPEN:
            # If the neighbor is in OPEN, check if the new path is better
            neighbor = OPEN[neighbor_key]
            if tentative_g < neighbor.g:
                neighbor.g = tentative_g
                neighbor.parent_node = curr

        else:
            # If the neighbor is not in OPEN, add it to OPEN
            neighbor = AStarNode(neighbor_key, neighbor_coord, self.environment.getHeuristic(neighbor_coord), self.epsilon)
            neighbor.g = tentative_g
            neighbor.parent_node = curr
            OPEN[neighbor_key] = neighbor

    return None

  def reconstruct_path(node):
    path = []
    while node:
      path.append(node.coord)
      node = node.parent_node
    return np.array(path[::-1])

if __name__ == '__main__':
    from collision_detection import collisionDetection
    start = np.array([1, 1, 1])
    goal = np.array([10, 5, 5])
    boundary = np.array([[0, 0, 0, 20, 20, 20]])
    blocks = np.array([[2, 0, 0, 3, 1, 1],])
    import time
    start_time = time.time()
    MP = AStar(boundary, blocks, start, goal)
    path = MP.plan()
    print(time.time() - start_time)
else:
    from .collision_detection import collisionDetection






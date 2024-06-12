import numpy as np

from utils.rrt_algorithms.rrt.rrt import RRT
from utils.rrt_algorithms.search_space.search_space import SearchSpace


class MyRRT():
    def __init__(self, boundary, blocks, start, goal, q=1, r=0.01, max_samples=5e6, prc=0.99):
        boundary = boundary[0]

        self.boundary = np.array([(boundary[0], boundary[3]), (boundary[1], boundary[4]), (boundary[2], boundary[5])])

        self.blocks = np.array([tuple(i) for i in blocks])
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.q = q
        self.r = r
        self.max_samples = max_samples
        self.prc = prc

        # create Search Space
        self.X = SearchSpace(self.boundary, self.blocks)

    def plan(self):
        # create rrt_search
        rrt = RRT(self.X, self.q, self.start, self.goal, self.max_samples, self.r, self.prc)
        path = rrt.rrt_search()
        path = np.array([np.array(i) for i in path])
        return path




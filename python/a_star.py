"""
The route finding problem can be formulated as follows:
    - Initial state: denoted by S0, is any state that can be designated as the starting point.
    - Actions: denoted by: {a1, a2, a3, ...}, is a set of the possible actions available to the agent at any given state.
    - Results: denoted by S', is the new stage where the agent ends up after taking an action at a given state.
    - Goal Test: denoted by a boolean of True or False, checks whether the current state is the goal state.
    - Path Cost: is the sum of the cost of the individual steps. In the route finding problem, the cost could be the distance between two cities.

States consists of the following parts:
    # Frontier: the farthest path that has been explored
    # Explored: Locations visited
    # Unexplored: Locations not visited

A Tree search is a family of functions, not a single algorithm.
    - Breadth first search or shortest first search: chooses the shortes possible path rfom the frontier that hasn't been considered yet.
        It is a graph search when we keep track of explored explored path. we can do that with a explored set. This is optimal and complete. It takes space 2 to the power of `n`
    - Uniform Cost Search: Cheapest First Search. This is optimal and complete. It takes space 2 to the power of `n`
    - Depth First Search: Expands to till the end of a path. It is not optimal and incomplete. Incomplete means you can miss finding the goal, for example;
        if there's an infinite path on the frontier. However, it takes a space `n`, if we are not tracking explored locations
    - Greedy best-first search: is like a Uniform Cost search but takes into account more knowledge. Say we know approximately the distance, we can expand immediately
        towards the direction closer to the goal. it does not consider any routes in which it may need to temporarily take a further away path in order to arrive at an overall shorter path.
    - A Star: this is practically a combination of Greedy best-first and Uniform cost search. It will try to optimize with both the shortest path and the goal in mind.
        it works by always expanding the path with the minmum value of the function => `f = g + h`. where: g(path) = path cost, h(finalState) = estimated distance to the goal
    - a heuristic function is said to be admissible if it never overestimates the cost of reaching the goal,
        i.e. the cost it estimates to reach the goal is not higher than the lowest possible cost from the current point in the path
    - A Heuristic `h`, is a function that estimates the total cost from the current state to the goal state. for example, an estimated cost of a specific path from Rotterdam To Berlin.
        Optimistic heuristic function `h` will find the lowest cost path because it is always less than the actual cost.

"""

import math
import heapq
from helpers import load_map_40

class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None # includes any explored/visited nodes.
        self.openSet = self.create_openSet() if goal != None and start != None else None # are any nodes on our frontier for potential future exploration
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None #  will hold the previous node that best reaches a given node
        self.gScore = self.create_gScore() if goal != None and start != None else None # is the g in our f = g + h equation, or the actual cost to reach our current node
        self.fScore = self.create_fScore() if goal != None and start != None else None # is the combination of g and h, i.e. the gScore plus a heuristic; total cost to reach the goal
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None

    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path

    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            f_score, current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    heapq.heappush(self.openSet, (self.calculate_fscore(neighbor), neighbor))

                # The distance from start to a neighbor
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False

    def create_closedSet(self):
        """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
        return set()

    def create_openSet(self):
        """ Creates and returns a data structure suitable to hold the set of currently discovered nodes
        that are not evaluated yet. Initially, only the start node is known."""
        if self.start != None:
            frontier = []
            heapq.heappush(frontier, (self.heuristic_cost_estimate(self.start), self.start))
            return frontier

        raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")

    def create_cameFrom(self):
        """Creates and returns a data structure that shows which node can most efficiently be reached from another,
        for each node."""
        return {}

    def create_gScore(self):
        """Creates and returns a data structure that holds the cost of getting from the start node to that node,
        for each node. The cost of going from start to start is zero."""
        gScore = {node: float('inf') for node in self.map.intersections}
        gScore[self.start] = 0
        return gScore

    def create_fScore(self):
        """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
        by passing by that node, for each node. That value is partly known, partly heuristic.
        For the first node, that value is completely heuristic."""
        fScore = {node: float('inf') for node in self.map.intersections}
        fScore[self.start] = self.heuristic_cost_estimate(self.goal)
        return fScore

    def set_map(self, M):
        """Method used to set map attribute """
        self._reset(self)
        self.start = None
        self.goal = None
        self.map = M

    def set_start(self, start):
        """Method used to set start attribute """
        self._reset(self)
        self.start = start
        self.set_goal(None)

    def set_goal(self, goal):
        """Method used to set goal attribute """
        self._reset(self)
        self.goal = goal

    def is_open_empty(self):
        """returns True if the open set is empty. False otherwise. """
        return not self.openSet

    def get_current_node(self):
        """ Returns the node in the open set with the lowest value of f(node)."""
        return heapq.heappop(self.openSet)

    def get_neighbors(self, node):
        """Returns the neighbors of a node"""
        return self.map.roads[node]

    def get_gScore(self, node):
        """Returns the g Score of a node"""
        return self.gScore[node]

    def distance(self, node_1, node_2):
        """ Computes the Euclidean L2 Distance"""
        return math.sqrt((self.map.intersections[node_2][0] - self.map.intersections[node_1][0])**2 + (self.map.intersections[node_2][1] - self.map.intersections[node_1][1])** 2)

    def get_tentative_gScore(self, current, neighbor):
        """Returns the tentative g Score of a node"""
        return self.get_gScore(current) + self.distance(current, neighbor)

    def heuristic_cost_estimate(self, node):
        """ Returns the heuristic cost estimate of a node """
        return self.distance(node, self.goal)

    def calculate_fscore(self, node):
        """Calculate the f score of a node. """
        # REMEMBER F = G + H
        return self.gScore[node] + self.heuristic_cost_estimate(node)

    def record_best_path_to(self, current, neighbor):
        """Record the best path to a node """
        self.cameFrom[neighbor] = current
        self.gScore[neighbor] = self.get_tentative_gScore(current, neighbor)
        self.fScore[neighbor] = self.calculate_fscore(neighbor)


planner = PathPlanner(load_map_40(), 8, 24)
print("best estimated cost path: ", planner.path)

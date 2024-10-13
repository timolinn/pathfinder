import math
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
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    self.openSet.add(neighbor)

                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False

def create_closedSet(self):
    """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
    # EXAMPLE: return a data structure suitable to hold the set of nodes already evaluated
    return set()

def create_openSet(self):
    """ Creates and returns a data structure suitable to hold the set of currently discovered nodes
    that are not evaluated yet. Initially, only the start node is known."""
    if self.start != None:
        frontier = set()
        frontier.add(self.start)
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
    if len(self.openSet) == 0:
        return None
    node_with_least_fscore = next(iter(self.openSet))
    for node in self.openSet:
        if self.calculate_fscore(node) < self.calculate_fscore(node_with_least_fscore):
            node_with_least_fscore = node
    return node_with_least_fscore

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
    # F = G + H
    return self.gScore[node] + self.heuristic_cost_estimate(node)

def record_best_path_to(self, current, neighbor):
    """Record the best path to a node """
    self.cameFrom[neighbor] = current
    self.gScore[neighbor] = self.get_tentative_gScore(current, neighbor)
    self.fScore[neighbor] = self.calculate_fscore(neighbor)

PathPlanner.create_closedSet = create_closedSet
PathPlanner.create_openSet = create_openSet
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.create_gScore = create_gScore
PathPlanner.create_fScore = create_fScore
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.is_open_empty = is_open_empty
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.get_gScore = get_gScore
PathPlanner.distance = distance
PathPlanner.get_tentative_gScore = get_tentative_gScore
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.calculate_fscore = calculate_fscore
PathPlanner.record_best_path_to = record_best_path_to



planner = PathPlanner(load_map_40(), 8, 24)
print("best estimated cost path: ", planner.path)

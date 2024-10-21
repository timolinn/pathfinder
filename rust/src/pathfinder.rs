use std::{
    cmp::Reverse,
    collections::{BinaryHeap, HashMap, HashSet},
};

use crate::{
    helpers::Map,
    node::{FrontierNode, Node},
};

pub struct Pathfinder {
    map: HashMap<u8, Node>,
    start: u8,
    goal: u8,
    explored_set: HashSet<u8>,
    frontier_heap: BinaryHeap<Reverse<FrontierNode>>,
    frontier_set: HashSet<(u8, u8)>,
}

impl Pathfinder {
    pub fn new(map: Map, start: u8, goal: u8) -> Self {
        // transform Map to nodes
        let mut nodes = HashMap::new();
        for loc in map {
            nodes.insert(loc.0, Node::new(loc.0, loc.1 .0.clone(), loc.1 .1.clone()));
        }

        let start_node = nodes.get(&start).cloned().unwrap();

        let mut pathfinder = Self::default();
        pathfinder.map = nodes;
        pathfinder.start = start;
        pathfinder.goal = goal;
        pathfinder.explored_set = HashSet::new();
        pathfinder.frontier_set = HashSet::from([(start, start)]);

        let mut start_f_node =
            FrontierNode(0, pathfinder.heuristic_cost_estimate(start), start_node);
        start_f_node.2.g_score = 0.0;

        // initialize frontier min-heap
        pathfinder.frontier_heap = BinaryHeap::from([Reverse(start_f_node)]);

        pathfinder
    }

    pub fn run_search(&mut self) -> Option<Vec<u8>> {
        while self.frontier_set.len() > 0 {
            let lowest_cost_node = self.frontier_heap.pop().unwrap().0;
            let mut current_node = lowest_cost_node.2;

            if current_node.node == self.goal {
                current_node.actions.reverse();
                current_node.actions.push(self.goal);
                return Some(current_node.actions);
            } else {
                self.explored_set.insert(current_node.node);
                self.frontier_set
                    .remove(&(lowest_cost_node.0, current_node.node));
            }

            for conn in current_node.connections {
                if self.explored_set.contains(&conn) {
                    // node already explored, skip
                    continue;
                }

                if !self.frontier_set.contains(&(current_node.node, conn)) {
                    // new path discovered
                    // update frontier with new paths
                    let mut came_from = vec![current_node.node];
                    came_from.extend(current_node.actions.iter());
                    self.update_frontier(current_node.node, conn, came_from, current_node.g_score);
                }
            }
        }
        println!("no path found!");
        None
    }

    fn update_frontier(&mut self, current: u8, neighbor: u8, mut came_from: Vec<u8>, g_score: f64) {
        let mut connection = self.map.get(&neighbor).cloned().unwrap();
        connection.actions.append(&mut came_from);
        connection.g_score = g_score + self.euclidean_distance(current, neighbor); // add g_score so far (tentative g_score of the path)
        connection.f_score = self.calculate_fscore(&connection);

        self.frontier_set.insert((current, connection.node));
        self.frontier_heap.push(Reverse(FrontierNode(
            current,
            connection.f_score,
            connection,
        )));
    }

    pub fn set_map(&mut self, m: Map) {
        let mut nodes = HashMap::new();
        for loc in m {
            nodes.insert(loc.0, Node::new(loc.0, loc.1 .0.clone(), loc.1 .1.clone()));
        }
        self.map = nodes
    }

    pub fn set_goal(&mut self, goal: u8) {
        self.goal = goal
    }

    pub fn set_start(&mut self, start: u8) {
        self.start = start
    }

    /// Computes the Euclidean L2 Distance
    pub fn euclidean_distance(&self, node_1: u8, node_2: u8) -> f64 {
        // TODO add error handling
        let n = self.map.get(&node_1).unwrap();
        let n1 = self.map.get(&node_2).unwrap();
        ((n1.x() - n.x()).powf(2.0) + (n1.y() - n.y()).powf(2.0)).sqrt()
    }

    // pub fn _manhattan_distance(&self, node_1: u8, node_2: u8) -> f64 {
    //     0.0
    // }

    /// straight distance between the start node and goal must be less than actual distance by road
    /// hence straight line distance is an admissible heuristic
    fn heuristic_cost_estimate(&self, node: u8) -> f64 {
        self.euclidean_distance(node, self.goal)
    }

    /// f = g + h
    /// g = actual cost of path
    /// h = admissible heuristic cost emtimate from start -> goal
    fn calculate_fscore(&self, node: &Node) -> f64 {
        node.g_score + self.heuristic_cost_estimate(node.node)
    }
}

impl Default for Pathfinder {
    fn default() -> Self {
        Self {
            map: HashMap::default(),
            explored_set: HashSet::default(),
            frontier_heap: BinaryHeap::default(),
            frontier_set: HashSet::default(),
            start: u8::default(),
            goal: u8::default(),
        }
    }
}

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
        let mut nodes = HashMap::new();
        for loc in map.iter() {
            nodes.insert(
                *loc.0,
                Node::new(*loc.0, loc.1 .0.clone(), loc.1 .1.clone()),
            );
        }

        let mut start_node = FrontierNode(0, 0.0, nodes.get(&start).cloned().unwrap());
        start_node.2.g_score = 0.0;
        Self {
            map: nodes,
            start,
            goal,
            explored_set: HashSet::new(),
            frontier_heap: BinaryHeap::from([Reverse(start_node)]),
            frontier_set: HashSet::from([(start, start)]),
        }
    }

    // take the start item, add it to the frontier
    // take it out from the frontier and expand the paths
    // add the paths to the frontier
    // add the start to the explored
    // take the best item out of the frontier
    // expand the paths
    // add the paths to the frontier
    // add the best item to explored list
    // take best item again
    // repeat previous steps until you reach the goal node
    pub fn run_search(&mut self) -> Option<Vec<u8>> {
        while self.frontier_set.len() > 0 {
            let mut current_node = self.frontier_heap.pop().unwrap().0;

            if current_node.2.node == self.goal {
                current_node.2.actions.reverse();
                current_node.2.actions.push(self.goal);
                return Some(current_node.2.actions);
            } else {
                self.explored_set.insert(current_node.2.node);
                self.frontier_set
                    .remove(&(current_node.0, current_node.2.node));
            }

            for conn in current_node.2.connections {
                if self.explored_set.contains(&conn) {
                    // node already explored, skip
                    continue;
                }

                if !self.frontier_set.contains(&(current_node.2.node, conn)) {
                    // new path discovered
                    // update frontier with new paths
                    let mut came_from = vec![current_node.2.node];
                    came_from.extend(current_node.2.actions.iter());
                    self.update_frontier(
                        current_node.2.node,
                        conn,
                        came_from,
                        current_node.2.g_score,
                    );
                }
            }
        }
        println!("no path found!");
        None
    }

    fn update_frontier(&mut self, current: u8, neighbor: u8, mut came_from: Vec<u8>, g_score: f64) {
        let mut connection = self.map.get(&neighbor).cloned().unwrap();
        connection.actions.append(&mut came_from);
        connection.g_score = g_score + self.euclidean_distance(current, neighbor);
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
        for loc in m.iter() {
            nodes.insert(
                *loc.0,
                Node::new(*loc.0, loc.1 .0.clone(), loc.1 .1.clone()),
            );
        }
        self.map = nodes
    }

    pub fn set_goal(&mut self, goal: u8) {
        self.goal = goal
    }

    pub fn set_start(&mut self, start: u8) {
        self.start = start
    }

    // Computes the Euclidean L2 Distance
    pub fn euclidean_distance(&self, node_1: u8, node_2: u8) -> f64 {
        // TODO add error handling
        let n = self.map.get(&node_1).unwrap();
        let n1 = self.map.get(&node_2).unwrap();
        ((n1.x() - n.x()).powf(2.0) + (n1.y() - n.y()).powf(2.0)).sqrt()
    }

    // pub fn _manhattan_distance(&self, node_1: u8, node_2: u8) -> f64 {
    //     0.0
    // }

    // straight distance between the start node and goal must be less than actual distance by road
    // hence straight line distance is an admissible heuristic
    fn heuristic_cost_estimate(&self, node: u8) -> f64 {
        self.euclidean_distance(node, self.goal)
    }

    fn calculate_fscore(&self, node: &Node) -> f64 {
        // TODO add error handling
        node.g_score + self.heuristic_cost_estimate(node.node)
    }
}

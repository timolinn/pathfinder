use std::cmp::Ordering;

use crate::helpers::Position;

#[derive(Debug, Clone, PartialEq)]
pub struct Node {
    pub node: u8,
    pub pos: Position,
    pub connections: Vec<u8>,
    pub actions: Vec<u8>,
    pub f_score: f64, // cost from src to goal node
    pub g_score: f64, // cost from src to neighbor node
}

#[derive(Debug, Clone, PartialEq)]
pub struct FrontierNode(pub u8, pub f64, pub Node);

impl Node {
    pub fn new(node: u8, pos: Position, connections: Vec<u8>) -> Self {
        Self {
            node,
            pos,
            connections,
            actions: vec![],
            f_score: f64::INFINITY,
            g_score: f64::INFINITY,
        }
    }

    // cost from previous node to current node
    pub fn get_gscore(&self, dst: &Node) -> f64 {
        self.euclidean_distance(dst)
    }

    // actual cost from the node to the goal node
    pub fn get_fscore(&self, goal: &Node) -> f64 {
        self.heuristic_cost_estimate(goal)
    }

    pub fn x(&self) -> f64 {
        self.pos.0
    }

    pub fn y(&self) -> f64 {
        self.pos.1
    }

    // Computes the Euclidean L2 Distance
    pub fn euclidean_distance(&self, n2: &Node) -> f64 {
        // TODO add error handling
        (n2.x() - self.x()).powf(2.0) + (n2.y() - self.y()).powf(2.0)
    }

    // pub fn _manhattan_distance(&self, node_1: u8, node_2: u8) -> f64 {
    //     0.0
    // }

    // straight distance between the start node and goal must be less than actual distance by road
    // hence straight line distance is an admissible heuristic
    fn heuristic_cost_estimate(&self, goal: &Node) -> f64 {
        self.euclidean_distance(goal)
    }
}

// we must implement proper comparison which uses f_score for node comparison
impl PartialOrd for FrontierNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match self.1 > other.1 {
            true => Some(Ordering::Greater),
            _ => Some(Ordering::Less),
        }
    }
}

impl Ord for FrontierNode {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.1 > other.1 {
            true => Ordering::Greater,
            _ => Ordering::Less,
        }
    }
}

impl Eq for FrontierNode {}

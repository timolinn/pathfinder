use std::cmp::Ordering;

use crate::helpers::Position;

#[derive(Debug, Clone, PartialEq)]
pub struct Node {
    pub node: u8,
    pub pos: Position,
    pub connections: Vec<u8>,
    pub actions: Vec<u8>,
    pub f_score: f64, // cost from start to goal node
    pub g_score: f64, // cost from start to neighbor node
}

/// (prev node, f_score, current_node)
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

    pub fn x(&self) -> f64 {
        self.pos.0
    }

    pub fn y(&self) -> f64 {
        self.pos.1
    }
}

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

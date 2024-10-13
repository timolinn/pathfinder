use std::collections::HashMap;

use crate::{helpers::Position, node::Node};

pub struct Pathfinder {
    map: HashMap<u8, Node>,
    src: u8,
    dst: u8,
    explored: Vec<Node>,
    open_set: Vec<Node>,
}

impl Pathfinder {
    pub fn _new(map: HashMap<u8, (Position, Vec<u8>)>, src: u8, dst: u8) -> Self {
        let mut nodes = HashMap::new();
        for loc in map {
            nodes.insert(loc.0, Node::new(loc.1 .0, loc.1 .1));
        }

        let start = nodes.get(&src).cloned().unwrap();
        Self {
            map: nodes,
            src,
            dst,
            explored: vec![Node::new(start.pos, start.connections)],
            open_set: vec![],
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
    pub fn _run_search(self) -> Vec<Node> {
        vec![]
    }
}

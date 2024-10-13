use crate::helpers::Position;

#[derive(Debug, Clone)]
pub struct Node {
    pub pos: Position,
    pub connections: Vec<u8>,
}

impl Node {
    pub fn new(pos: Position, connections: Vec<u8>) -> Self {
        Self { pos, connections }
    }

    // cost from previous node to current node
    pub fn get_gscore(self, src: Node) -> usize {
        0
    }

    // heuristic function. returns estimated distance from the node to the goal node
    pub fn get_hscore(self, dst: Node) -> usize {
        0
    }

    // actual cost from the node to the goal node
    pub fn get_fscore(self, dst: Node) -> usize {
        0
    }
}

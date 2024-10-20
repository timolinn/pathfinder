use helpers::LAZY_MAP;
use pathfinder::Pathfinder;

pub mod helpers;
pub mod node;
pub mod pathfinder;

fn main() {
    // 5, 16, 37, 12, 34]
    // let mut p_finder = Pathfinder::new(LAZY_MAP.clone(), 5, 34);

    // [8, 14, 16, 37, 12, 17, 10, 24]
    // no point having LAZY_MAP if we clone here anyways
    let mut p_finder = Pathfinder::new(LAZY_MAP.clone(), 8, 24);

    println!("{:?}", p_finder.run_search());
}

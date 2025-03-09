#![feature(generic_const_exprs)]

use std::time::Instant;

use kdtrees::*;

fn main() {
    const N: usize = 1000;
    const K: usize = 3;
    let max_leaf_items: usize = f64::sqrt(N as f64) as usize;
    let ranges: [_; K] = std::array::from_fn(|_| 0.0..1.0);
    let mut tree = KdTree::new(ranges, max_leaf_items);
    let mut points: Vec<[f64; K]> = Vec::with_capacity(N);
    for _ in 0..N {
        let point: [f64; K] = rand::random();
        points.push(point);
    }

    let now = Instant::now();
    points.into_iter().for_each(|p| {
        tree.insert(p);
    });
    println!(
        "{N} items took {} ns / iter",
        now.elapsed().as_nanos() as f64 / N as f64
    );
    //println!("{tree:#?}");
}

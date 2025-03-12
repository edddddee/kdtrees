#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use std::time::Instant;

use kdtrees::KdTree;
use rand::Rng;

fn main() {
    let mut rng = rand::rng();

    const CAPACITY: usize = 16;
    const K: usize = 3;
    const N: usize = 10000;
    let bounds: [_; K] = std::array::from_fn(|_| 0.0..1.0);

    let mut tree = KdTree::new(bounds, CAPACITY, None);
    let points: Vec<_> = (0..N).map(|_| rng.random::<[f64; K]>()).collect();
    tree.insert_vec(points);

    let points: Vec<_> = (0..N).map(|_| rng.random::<[f64; K]>()).collect();
    let now = Instant::now();
    tree.insert_vec(points);
    let elapsed = now.elapsed().as_nanos();
    println!("Inserting {N} items into a {K}-d tree with CAPACITY={CAPACITY}");
    println!("Took: {} ns / iter", elapsed as f64 / N as f64);

    //println!("{:#?}", tree);
}

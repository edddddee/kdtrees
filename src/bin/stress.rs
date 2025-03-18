#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use std::time::Instant;

use kdtrees::*;
use rand::Rng;

const ITERS: usize = 1_000_000;

fn bench() {
    let mut rng = rand::rng();

    const CAPACITY: usize = 16;
    const K: usize = 3;
    const N: usize = 1_000;
    let bounds: [_; K] = std::array::from_fn(|_| 0.0..1.0);

    let mut tree = KdTree::new(bounds, CAPACITY, None);
    let points: Vec<_> = (0..N).map(|_| rng.random::<[f64; K]>()).collect();
    for point in points {
        tree.insert(point, 0);
    }

    let point: [f64; K] = rng.random();
    let now = Instant::now();
    for i in 0..ITERS {
        tree.insert(point, 0);
    }
    let total_time = now.elapsed().as_nanos();
    println!("Took: {} ns / iter", total_time as f64 / ITERS as f64);
}

fn bench2() {
    let mut rng = rand::rng();

    const CAPACITY: usize = 16;
    const K: usize = 3;
    const N: usize = 1_000;

    let mut tree = kdtree::KdTree::with_capacity(K, CAPACITY);
    let points: Vec<_> =
        (0..N).map(|_| rng.random::<([f64; K], f64)>()).collect();

    for point in points.iter() {
        tree.add(&point.0, point.1).unwrap();
    }

    let point: ([f64; K], f64) = rng.random();

    let now = Instant::now();
    for i in 0..ITERS {
        tree.add(&point.0, point.1).unwrap();
    }
    let total_time = now.elapsed().as_nanos();
    println!(
        "Took: {} ns / iter (external)",
        total_time as f64 / ITERS as f64
    );
}

fn bench_1mil() {
    fn internal() {
        let mut rng = rand::rng();

        const CAPACITY: usize = 16;
        const K: usize = 3;
        const N: usize = 1_000_000;
        let bounds: [_; K] = std::array::from_fn(|_| 0.0..1.0);

        let mut tree: Octree<f64> = Octree::new(bounds, CAPACITY, None);

        let points: Vec<_> = (0..N).map(|_| rng.random::<[f64; K]>()).collect();

        let now = Instant::now();
        for point in points {
            tree.insert(point, 0);
        }
        let time = now.elapsed().as_nanos();
        println!("Took: {} ns / iter", time as f64 / N as f64);
    }
    fn external() {
        let mut rng = rand::rng();

        const CAPACITY: usize = 16;
        const K: usize = 3;
        const N: usize = 1_000_000;

        let mut tree = kdtree::KdTree::with_capacity(K, CAPACITY);

        let points: Vec<_> = (0..N).map(|_| rng.random::<[f64; K]>()).collect();

        let now = Instant::now();
        for (i, point) in points.iter().enumerate() {
            tree.add(point, i);
        }
        let time = now.elapsed().as_nanos();
        println!("Took: {} ns / iter (external)", time as f64 / N as f64);
    }
    internal();
    external();
}

fn main() {
    bench_1mil();
}

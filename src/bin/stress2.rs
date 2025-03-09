#![feature(generic_const_exprs)]
use kdtrees::*;

fn main() {
    let mut root: KdTreeNode<f64, [f64; 2], 2> =
        KdTreeNode::new([0.0..1.0, 0.0..1.0], 5);
}

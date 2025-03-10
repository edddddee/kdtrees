# (Unfinished) K-D trees

Implementation of Quadtrees, Octrees and general K-dimensional trees in rust.
Currently only works on the nightly compiler toolchain because it uses
`#![feature(generic_const_exprs)]` (see: https://github.com/rust-lang/rust/issues/76560).

## Example: quadtree with realtime particle system
[examples/particle2d.rs](https://github.com/edddddee/kdtrees/blob/master/examples/particles2d.rs)

<img src=https://github.com/edddddee/kdtrees/blob/master/examples/quadtree.png width=750>

# K-D trees

Currently a rough implementation of general K-dimensional trees in rust.
For now, it only works on the nightly compiler toolchain because the implementation relies on
`#![feature(generic_const_exprs)]` (see: https://github.com/rust-lang/rust/issues/76560).

## Example: Particle System
This code uses a quadtree (2-d tree) to drastically speed up collision detection in a particle simulation.
[examples/particle2d.rs](https://github.com/edddddee/kdtrees/blob/master/examples/particles2d.rs)

<img src=https://github.com/edddddee/kdtrees/blob/master/examples/particles2d.png width=1080>

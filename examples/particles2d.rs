#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use std::collections::HashMap;
use std::ops::Range;

use itertools::Itertools;
use kdtrees::{Cell2, MidPoint, Node, Quadtree};
use macroquad::miniquad::window::show_mouse;
use macroquad::prelude::*;
use macroquad::rand::gen_range;
//use rayon::prelude::*;

// TODO: Make a function to spawn a cluster of points around some center.
//       i.e. a form of point cloud.
//       Spawn cluster with mouse click?

const START_SPEED: f32 = 300.0;
const PLOT_SCALE: f32 = 0.33;

#[inline]
const fn dist_squared(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
}

#[derive(Clone, Debug)]
struct Particle {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
    radius: f32,
    mass: f32,
}

impl Particle {
    #[allow(dead_code)]
    fn new(x: f32, y: f32, radius: f32, density: f32) -> Self {
        // let vx = gen_range(-30.0, 50.0);
        // let vy = gen_range(-50.0, 30.0);
        let (vx, vy) = if x < 900.0 / 2.0 {
            (-START_SPEED, 0.0)
        } else {
            (0.0, 0.0)
        };
        Self {
            x,
            y,
            vx,
            vy,
            radius,
            mass: density * radius * radius,
        }
    }

    #[inline]
    const fn dist_squared(&self, other: &Self) -> f32 {
        dist_squared(self.x, self.y, other.x, other.y)
    }

    #[inline]
    fn dist(&self, other: &Self) -> f32 {
        self.dist_squared(other).sqrt()
    }

    #[inline]
    const fn collides(&self, other: &Self) -> bool {
        self.dist_squared(other)
            < (self.radius + other.radius) * (self.radius + other.radius)
    }

    #[inline]
    fn speed_squared(&self) -> f32 {
        self.vx * self.vx + self.vy * self.vy
    }

    #[inline]
    fn speed(&self) -> f32 {
        self.speed_squared().sqrt()
    }

    fn resolve_collision(&mut self, other: &mut Self) {
        let inner_prod = (self.vx - other.vx) * (self.x - other.x)
            + (self.vy - other.vy) * (self.y - other.y);
        let num1 = 2.0 * other.mass * inner_prod;
        let num2 = 2.0 * self.mass * inner_prod;
        let div = (self.mass + other.mass) * self.dist_squared(other);

        let vx1 = self.vx - num1 * (self.x - other.x) / div;
        let vy1 = self.vy - num1 * (self.y - other.y) / div;
        let vx2 = other.vx - num2 * (other.x - self.x) / div;
        let vy2 = other.vy - num2 * (other.y - self.y) / div;

        // Update velocities
        self.vx = vx1;
        self.vy = vy1;
        other.vx = vx2;
        other.vy = vy2;

        // Handle overlap
        let dist = self.dist(other);
        let tangentx = (other.x - self.x) / dist;
        let tangenty = (other.y - self.y) / dist;
        let overlap = 0.5 * (self.radius + other.radius - dist);
        self.x -= overlap * tangentx;
        self.y -= overlap * tangenty;
        other.x += overlap * tangentx;
        other.y += overlap * tangenty;
    }
}

/*
impl From<&Particle> for [f32; 2] {
    fn from(p: &Particle) -> Self {
        [p.x, p.y]
    }
}
*/

impl From<&Particle> for [(f32, f32); 2] {
    fn from(p: &Particle) -> Self {
        [
            (p.x - p.radius, p.x + p.radius),
            (p.y - p.radius, p.y + p.radius),
        ]
    }
}

fn handle_collisions<Item>(
    node: &Node<f32, Item, 2>,
    particles: &mut Vec<Particle>,
) {
    match node {
        Node::Leaf { ids, .. } => {
            if ids.len() >= 2 {
                for pair in ids.iter().combinations(2) {
                    let id1: usize = *pair[0];
                    let id2: usize = *pair[1];
                    let (i, j) =
                        if id1 < id2 { (id1, id2) } else { (id2, id1) };
                    let (left, right) = particles.split_at_mut(j);
                    let (p1, p2) = (&mut left[i], &mut right[0]);
                    if p1.collides(p2) {
                        p1.resolve_collision(p2);
                    }
                }
            }
        }
        Node::Parent { children, .. } => {
            for child in children.iter() {
                handle_collisions(child, particles);
            }
        }
    }
}

fn handle_collisions_global(particles: &mut Vec<Particle>) {
    let len = particles.len();
    for i in 0..len {
        for j in (i + 1)..len {
            let (left, right) = particles.split_at_mut(j);
            let (p1, p2) = (&mut left[i], &mut right[0]);
            if p1.collides(p2) {
                p1.resolve_collision(p2);
            }
        }
    }
}

fn total_energy(particles: &Vec<Particle>) -> f32 {
    particles
        .iter()
        .map(|p| 0.5 * p.mass * p.speed_squared())
        .filter(|e| !e.is_nan())
        .fold(0.0, |acc, e| acc + e)
}

#[allow(non_snake_case)]
fn maxwell_boltzmann(v: f32, m: f32, E: f32, N: f32) -> f32 {
    let a = m * N / E;
    a * v * f32::exp(-0.5 * a * v * v)
}

#[allow(non_snake_case)]
fn most_probable_speed(m: f32, E: f32, N: f32) -> f32 {
    (2.0 * E / (m * N)).sqrt()
}

#[allow(non_snake_case)]
fn max_probability(m: f32, E: f32, N: f32) -> f32 {
    maxwell_boltzmann(most_probable_speed(m, E, N), m, E, N)
}

#[allow(dead_code)]
fn create_particles(
    width: f32,
    height: f32,
    radius: f32,
    n: usize,
) -> Vec<Particle> {
    (0..n)
        .map(|_| {
            let r = 0.0;
            Particle::new(
                gen_range(r * width, (1.0 - r) * width),
                gen_range(0.0, height),
                radius,
                1.0,
            )
        })
        .collect()
}

fn make_point_cluster(
    n: usize,
    xc: f32,
    yc: f32,
    cluster_r: f32,
    point_r: f32,
) -> Vec<Particle> {
    let mut particles = vec![];
    for _ in 0..n {
        let r: f32 = gen_range(0.0, cluster_r);
        let theta: f32 = gen_range(0.0, 2.0 * std::f32::consts::PI);
        let x = xc + r * theta.cos();
        let y = yc + r * theta.sin();
        let v = gen_range(0.0, START_SPEED);
        let vx = v * (x - xc) / cluster_r;
        let vy = v * (y - yc) / cluster_r;
        let mass = point_r * point_r;
        particles.push(Particle {
            x,
            y,
            vx,
            vy,
            radius: point_r,
            mass,
        });
    }
    particles
}

fn heat_gradient(value: f32) -> Color {
    let value = value.clamp(0.0, 1.0);
    if value > 0.8 {
        Color::new(1.0, 0.0, 0.0, 1.0)
    } else if value > 0.6 {
        let t = (value - 0.6) / 0.2;
        Color::new(1.0, 1.0 - t, 0.0, 1.0)
    } else if value > 0.5 {
        let t = (value - 0.5) / 0.1;
        Color::new(t, 1.0, 0.0, 1.0)
    } else if value > 0.25 {
        let t = (value - 0.25) / 0.25;
        Color::new(0.0, 1.0, 1.0 - t, 1.0)
    } else {
        let t = value / 0.25;
        Color::new(0.0, t, 1.0, 1.0)
    }
}

enum BoundaryConditions {
    Periodic,
    Collide,
    PeriodicHorizontal,
    PeriodicVertical,
}

#[derive(Debug)]
struct Histogram {
    bins: Vec<f32>,
    nbins: usize,
    counts: HashMap<usize, usize>,
    bounds: Range<f32>,
    total_count: usize,
}

impl Histogram {
    fn new(bounds: Range<f32>, nbins: usize) -> Self {
        let mut bins: Vec<f32> = vec![];
        let bin_width = (bounds.end - bounds.start) / nbins as f32;
        (0..nbins).for_each(|n| bins.push(n as f32 * bin_width));
        Self {
            bins,
            nbins,
            counts: HashMap::new(),
            bounds,
            total_count: 0,
        }
    }

    fn insert(&mut self, value: f32) {
        self.total_count += 1;
        if self.bounds.contains(&value) {
            for (bin, bound) in self.bins.iter().enumerate() {
                if value < *bound {
                    self.counts.entry(bin).and_modify(|c| *c += 1).or_insert(1);
                    break;
                }
            }
        }
    }
}

fn conf() -> Conf {
    Conf {
        window_title: "Quadtree of 2D particles".to_string(),
        window_width: 1600,
        window_height: 800,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(conf)]
async fn main() {
    // Function that traverses the quadtree and draws boundary lines
    fn draw_bounds<Item>(node: &Node<f32, Item, 2>) {
        match node {
            Node::Parent {
                ranges, children, ..
            } => {
                let [x1, x2, y] =
                    [ranges[0].start, ranges[0].end, ranges[1].mid_point()];
                let [y1, y2, x] =
                    [ranges[1].start, ranges[1].end, ranges[0].mid_point()];
                draw_line(x1, y, x2, y, 1.0, WHITE);
                draw_line(x, y1, x, y2, 1.0, WHITE);
                for child in children.iter() {
                    draw_bounds(child);
                }
            }
            _ => {}
        };
    }

    fn draw_histogram(
        hist: &Histogram,
        mass: f32,
        energy: f32,
        left: f32,
        top: f32,
        width: f32,
        height: f32,
    ) {
        // pixel width of each bin
        let bin_width = width / hist.nbins as f32;
        // bin width as in the width of its bounds
        let bin_delta =
            (hist.bounds.end - hist.bounds.start) / hist.nbins as f32;
        for i in 0..hist.nbins {
            let count = hist.counts.get(&i).unwrap_or(&0);
            let freq = *count as f32 / hist.total_count as f32;
            let x = left + i as f32 * bin_width;
            let max_pdf =
                max_probability(mass, energy, hist.total_count as f32);
            let freq_scl = freq * height * PLOT_SCALE / (bin_delta * max_pdf);
            let y = top + height - freq_scl;
            let color = heat_gradient(i as f32 * bin_delta / hist.bounds.end);
            draw_rectangle(x, y, bin_width, freq_scl * height, color);
        }
    }

    fn draw_maxwell_boltzmann(
        hist: &Histogram,
        mass: f32,
        energy: f32,
        num_particles: usize,
        left: f32,
        top: f32,
        width: f32,
        height: f32,
    ) {
        let mut last_x = left;
        let mut last_y = height;
        for x in 1..(width as usize) {
            let v = hist.bounds.start
                + (hist.bounds.end - hist.bounds.start) * x as f32 / width;
            let pdf = maxwell_boltzmann(v, mass, energy, num_particles as f32);
            let max_pdf = max_probability(mass, energy, num_particles as f32);
            let pdf_scl = pdf * height * PLOT_SCALE / max_pdf;
            let y = top + height - pdf_scl;
            let x = left + x as f32;

            draw_line(last_x, last_y, x, y, 2.0, YELLOW);

            last_x = x;
            last_y = y;
        }
    }

    // Particle system configuration
    let n = 200;
    let particle_radius = 4.0;
    let particle_mass = particle_radius * particle_radius;
    let cluster_radius = 25.0;
    let mut particles: Vec<Particle> = vec![];
    //create_particles(sim_width, sim_height, radius, n);

    // Screen space for simulation
    let sim_width = screen_width() / 2.0;
    let sim_height = screen_height();

    // Set max quadtree depth (doesnt make sense to subdivide pixels)
    let max_depth = {
        let min_cell_size = 3.0 * 2.0 * particle_radius;
        let divisions = sim_width.min(sim_height) / min_cell_size;
        let mut depth = 1;
        while ((2 << depth) as f32) < divisions {
            depth += 1;
        }
        depth
    };

    // Flags
    let mut use_quadtree = true;
    let mut boundary_conditions = BoundaryConditions::Periodic;
    let mut show_bounds = false;
    let mut show_stats = true;
    let mut bounding_box = true;

    show_mouse(false);
    let mut frame = 0;
    loop {
        // Get the mouse position
        let (mouse_x, mouse_y) = mouse_position();

        // Handle keyboard input
        //
        // Quadtree ON/OFF
        if is_key_pressed(KeyCode::Space) {
            use_quadtree = !use_quadtree;
            let status = if use_quadtree { "ON" } else { "OFF" };
            println!("[Quadtree]: {status}");
        }
        // Cycle boundary conditions
        if is_key_pressed(KeyCode::W) {
            let (status, new_bc) = match boundary_conditions {
                BoundaryConditions::Periodic => {
                    ("Collide", BoundaryConditions::Collide)
                }
                BoundaryConditions::Collide => (
                    "Horizontally periodic",
                    BoundaryConditions::PeriodicHorizontal,
                ),
                BoundaryConditions::PeriodicHorizontal => (
                    "Vertically periodic",
                    BoundaryConditions::PeriodicVertical,
                ),
                BoundaryConditions::PeriodicVertical => {
                    ("Periodic", BoundaryConditions::Periodic)
                }
            };
            boundary_conditions = new_bc;
            println!("[Boundary condition]: {status}");
        }
        // Show bounds ON/OFF
        if is_key_pressed(KeyCode::B) {
            show_bounds = !show_bounds;
            let status = if show_bounds { "ON" } else { "OFF" };
            println!("[Show bounds]: {status}");
        }
        // Drain particles
        if is_key_pressed(KeyCode::E) {
            particles.clear();
        }
        // Toggle stats
        if is_key_pressed(KeyCode::S) {
            show_stats = !show_stats;
            let status = if show_stats { "ON" } else { "OFF" };
            println!("[Show statistics]: {status}");
        }
        // Toggle between point quadtree and cell quadtree
        if is_key_pressed(KeyCode::P) {
            bounding_box = !bounding_box;
            let status = if show_stats { "Bounding box" } else { "Point" };
            println!("[Quadtree]: {status}");
        }
        // Mouse click
        if is_mouse_button_pressed(MouseButton::Left) {
            particles.extend(make_point_cluster(
                n,
                mouse_x,
                mouse_y,
                cluster_radius,
                particle_radius,
            ));
        }

        // Get elapsed time
        let dt = get_frame_time();

        // Create histogram for storing velocities
        let mut histogram = Histogram::new(0.0..START_SPEED * 1.0, 100);

        // Create KdTree
        let capacity = 10;
        let bounds = [(0.0..sim_width), (0.0..sim_height)];

        let mut tree =
            Quadtree::<f32, Cell2<f32>>::new(bounds, capacity, Some(max_depth));
        /*
        let mut tree = Quadtree::<f32, Point2<f32>>::new(
            bounds,
            capacity,
            Some(max_depth),
        );
        */

        // Handle particle collisions depending on whether quadtree approach
        // is used or not
        if use_quadtree {
            tree.insert_slice(&particles[..]);
            handle_collisions(&tree.root, &mut particles);
        } else {
            handle_collisions_global(&mut particles);
        }

        // Update particle states
        particles.iter_mut().for_each(|particle| {
            // Integrate each particle's position
            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;

            let speed = dist_squared(particle.vx, particle.vy, 0.0, 0.0).sqrt();
            if show_stats {
                histogram.insert(speed);
            }

            // Logic for handling boundary conditions
            match boundary_conditions {
                BoundaryConditions::Collide => {
                    if particle.x + particle.radius > sim_width {
                        particle.x = sim_width - particle.radius - 1.0;
                        particle.vx *= -1.0;
                    }
                    if particle.x - particle.radius < 0.0 {
                        particle.x = particle.radius + 1.0;
                        particle.vx *= -1.0;
                    }
                    if particle.y + particle.radius > sim_height {
                        particle.y = sim_height - particle.radius - 1.0;
                        particle.vy *= -1.0;
                    }
                    if particle.y - particle.radius < 0.0 {
                        particle.y = particle.radius + 1.0;
                        particle.vy *= -1.0;
                    }
                }
                BoundaryConditions::Periodic => {
                    particle.x %= sim_width;
                    if particle.x < 0.0 {
                        particle.x += sim_width;
                    }
                    particle.y %= sim_height;
                    if particle.y < 0.0 {
                        particle.y += sim_height;
                    }
                }
                BoundaryConditions::PeriodicHorizontal => {
                    particle.x %= sim_width;
                    if particle.x < 0.0 {
                        particle.x += sim_width;
                    }
                    if particle.y + particle.radius > sim_height {
                        particle.y = sim_height - particle.radius - 1.0;
                        particle.vy *= -1.0;
                    }
                    if particle.y - particle.radius < 0.0 {
                        particle.y = particle.radius + 1.0;
                        particle.vy *= -1.0;
                    }
                }
                BoundaryConditions::PeriodicVertical => {
                    particle.y %= sim_height;
                    if particle.y < 0.0 {
                        particle.y += sim_height;
                    }
                    if particle.x + particle.radius > sim_width {
                        particle.x = sim_width - particle.radius;
                        particle.vx *= -1.0;
                    }
                    if particle.x - particle.radius < 0.0 {
                        particle.x = particle.radius;
                        particle.vx *= -1.0;
                    }
                }
            }
        });
        // Calculate energy of particle system
        let energy = total_energy(&particles);

        // Get max particle speed (for calculating particle colors)
        let max_speed = particles
            .iter()
            .map(|p| p.speed())
            .filter(|v| !v.is_nan())
            .reduce(|acc, e| if e > acc { e } else { acc });

        // Logging
        if frame % 100 == 0 {
            //println!("frame time = {:.2} ms", dt * 1000.0);
        }

        // Drawing section
        clear_background(BLACK);

        // Draw each particle. If there are a lot of them, use step_by(k) to
        // draw a subset of the particles while still simulating all particles.
        particles.iter().step_by(1).for_each(|particle| {
            let color = match max_speed {
                Some(max_speed) => heat_gradient(particle.speed() / max_speed),
                None => Color::from_hex(0x1c73ff),
            };
            draw_circle(particle.x, particle.y, particle.radius, color);
        });

        // If quadtree is used and show_bounds is set to true, display the node
        // boundaries.
        if use_quadtree && show_bounds {
            draw_bounds(&tree.root);
        }

        // Draw black rectangle over statistics screen space to remove clipping
        // particles
        draw_rectangle(
            sim_width,
            0.0,
            screen_width() - sim_width,
            screen_height(),
            BLACK,
        );
        if show_stats {
            // Draw the velocity histogram
            draw_histogram(
                &histogram,
                particle_mass,
                energy,
                sim_width,
                0.0,
                screen_width() - sim_width,
                screen_height(),
            );
            // Draw the theoretical distribution
            draw_maxwell_boltzmann(
                &histogram,
                particle_mass,
                energy,
                particles.len(),
                sim_width,
                0.0,
                screen_width() - sim_width,
                screen_height(),
            );
        }

        // Draw boundary line for simulation space
        draw_line(sim_width, 0.0, sim_width, sim_height, 1.0, WHITE);

        // Display FPS in top left corner
        draw_text(format!("FPS: {}", get_fps()).as_str(), 0., 20., 32., GREEN);
        draw_text(
            format!("Particle count: {}", particles.len()).as_str(),
            sim_width + 5.,
            20.,
            32.,
            WHITE,
        );

        // Draw crosshair
        let crosshair_size = 10.0;
        draw_line(
            mouse_x - crosshair_size,
            mouse_y,
            mouse_x + crosshair_size,
            mouse_y,
            2.0,
            GREEN,
        );
        draw_line(
            mouse_x,
            mouse_y - crosshair_size,
            mouse_x,
            mouse_y + crosshair_size,
            2.0,
            GREEN,
        );

        frame += 1;
        next_frame().await
    }
}

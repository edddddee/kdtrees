#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use itertools::Itertools;
use kdtrees::{KdTree, MidPoint};
use macroquad::prelude::*;
use macroquad::rand::gen_range;
//use rayon::prelude::*;

// TODO: Make a function to spawn a cluster of points around some center.
//       i.e. a form of point cloud.
//       Spawn cluster with mouse click?

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
    fn new(x: f32, y: f32, radius: f32, density: f32) -> Self {
        // let vx = gen_range(-30.0, 50.0);
        // let vy = gen_range(-50.0, 30.0);
        let (vx, vy) = if x < 900.0 / 2.0 {
            (-75.0, 0.0)
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

impl From<&Particle> for [f32; 2] {
    fn from(p: &Particle) -> Self {
        [p.x, p.y]
    }
}

fn handle_collisions(
    node: &kdtrees::Node<f32, 2>,
    particles: &mut Vec<Particle>,
) {
    match node {
        kdtrees::Node::Leaf { ids, .. } => {
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
        kdtrees::Node::Parent { children, .. } => {
            for child in children.iter() {
                handle_collisions(child, &mut *particles);
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

fn create_particles(
    width: f32,
    height: f32,
    radius: f32,
    n: usize,
) -> Vec<Particle> {
    (0..n)
        .map(|_| {
            let r = 0.45;
            Particle::new(
                gen_range(r * width, (1.0 - r) * width),
                gen_range(0.0, height),
                radius,
                1.0,
            )
        })
        .collect()
}

fn conf() -> Conf {
    Conf {
        window_title: "Quadtree of 2D particles".to_string(),
        window_width: 900,
        window_height: 900,
        ..Default::default()
    }
}

#[macroquad::main(conf)]
async fn main() {
    fn draw_bounds(node: &kdtrees::Node<f32, 2>) {
        match node {
            kdtrees::Node::Parent {
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

    let width = screen_width();
    let height = screen_height();

    let n = 5000;
    let radius = 2.0;
    let mut particles = create_particles(width, height, radius, n);
    let capacity = 5;

    let mut use_quadtree = true;
    let mut wall_collision = true;
    let mut show_bounds = true;

    let mut frame = 0;
    loop {
        if is_key_pressed(KeyCode::Space) {
            use_quadtree = !use_quadtree;
            let status = if use_quadtree { "ON" } else { "OFF" };
            println!("Quadtree: {status}");
        }
        if is_key_pressed(KeyCode::W) {
            wall_collision = !wall_collision;
            let status = if wall_collision { "ON" } else { "OFF" };
            println!("Wall collision: {status}");
        }
        if is_key_pressed(KeyCode::B) {
            show_bounds = !show_bounds;
            let status = if show_bounds { "ON" } else { "OFF" };
            println!("Show bounds: {status}");
        }
        let dt = get_frame_time();

        let bounds = [(0.0..width), (0.0..height)];
        let mut tree = KdTree::new(bounds, capacity);

        if use_quadtree {
            tree.insert_slice(&particles[..]);
            handle_collisions(&tree.root, &mut particles);
        } else {
            handle_collisions_global(&mut particles);
        }

        particles.iter_mut().for_each(|particle| {
            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;

            if wall_collision {
                if particle.x + particle.radius > width {
                    particle.x = width - particle.radius;
                    particle.vx *= -1.0;
                }
                if particle.x - particle.radius < 0.0 {
                    particle.x = particle.radius;
                    particle.vx *= -1.0;
                }
                if particle.y + particle.radius > height {
                    particle.y = height - particle.radius;
                    particle.vy *= -1.0;
                }
                if particle.y - particle.radius < 0.0 {
                    particle.y = particle.radius;
                    particle.vy *= -1.0;
                }
            } else {
                particle.x %= width;
                if particle.x < 0.0 {
                    particle.x += width;
                }
                particle.y %= height;
                if particle.y < 0.0 {
                    particle.y += height;
                }
            }
        });

        if frame % 100 == 0 {
            println!("frame time = {:.2} ms", dt * 1000.0);
        }
        clear_background(BLACK);
        particles.iter().step_by(1).for_each(|particle| {
            draw_circle(
                particle.x,
                particle.y,
                particle.radius,
                Color::from_hex(0x1c73ff),
            );
        });
        if use_quadtree && show_bounds {
            draw_bounds(&tree.root);
        }
        draw_text(format!("FPS: {}", get_fps()).as_str(), 0., 16., 32., GREEN);

        frame += 1;
        next_frame().await
    }
}

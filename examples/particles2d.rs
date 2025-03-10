#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use kdtrees::{KdTree, MidPoint};
use macroquad::prelude::*;
use macroquad::rand::gen_range;
use rayon::prelude::*;

#[derive(Clone)]
struct Particle {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
    radius: f32,
}

impl Particle {
    fn new(x: f32, y: f32, radius: f32) -> Self {
        let vx = gen_range(-30.0, 50.0);
        let vy = gen_range(-20.0, 70.0);
        Self {
            x,
            y,
            vx,
            vy,
            radius,
        }
    }
}

impl From<&Particle> for [f32; 2] {
    fn from(p: &Particle) -> Self {
        [p.x, p.y]
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
            Particle::new(
                gen_range(0.2 * width, 0.75 * width),
                gen_range(0.0, height),
                radius,
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

    let n = 10000;
    let radius = 2.0;
    let mut particles = create_particles(width, height, radius, n);

    let capacity = 10;

    let mut frame = 0;
    loop {
        let dt = get_frame_time();

        let bounds = [(0.0..width), (0.0..height)];
        let mut tree = KdTree::new(bounds, capacity);
        tree.insert_slice(&particles[..]);

        if frame % 100 == 0 {
            println!("dt = {:.2} ms", dt * 1000.0);
        }

        clear_background(BLACK);
        particles.iter().step_by(1).for_each(|particle| {
            draw_circle(particle.x, particle.y, particle.radius, RED);
        });

        particles.iter_mut().for_each(|particle| {
            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;
            particle.x %= width;
            particle.y %= height;
        });
        draw_bounds(&tree.root);
        draw_text(format!("FPS: {}", get_fps()).as_str(), 0., 16., 32., GREEN);

        frame += 1;
        next_frame().await
    }
}

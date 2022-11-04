use std::hash::{Hash, Hasher};
use pathfinding::prelude::astar;
use bevy_math::Vec2;
use image::{Rgba, RgbaImage};
use noisy_bevy::simplex_noise_2d_seeded;

const MAX_TURN_ANGLE_DEG: i32 = 5;
const LEG_DISTANCE: f32 = 20.; // meters
const SEED: f32 = 0.;
const NODE_HEIGHT_MULTIPLIER: i32 = 1000; // since the PathNode struct can only take integers, we multiply the height by this and round it.

const NOISE_SCALE: f32 = 100.;
const NOISE_AMPLITUDE: f32 = 10.;

const WIDTH: u32 = 512;
const HEIGHT: u32 = 512;

fn get_absolute_slope(dist: f32, val1: f32, val2: f32) -> f32 {
    ((val2 - val1) / dist).abs()
}

fn angle_deg_between_vec2(first: Vec2, second: Vec2) -> i32 {
    (second.dot(first) / (second.length() * first.length())).acos().to_degrees() as i32
}

#[derive(Clone, Debug, Hash, Eq, PartialEq)]
struct PathNode {
    position: (i32, i32),
    height: i32,
    current_world_angle_deg: i32,
}

impl PathNode {
    fn get_world_vec2(&self) -> Vec2 {
        Vec2::new(self.position.0 as f32, self.position.1 as f32)
    }

    fn successors(&self) -> Vec<(Self, u32)> {
        let mut result = Vec::new();
        for angle_deg in ((self.current_world_angle_deg - MAX_TURN_ANGLE_DEG)..(self.current_world_angle_deg + MAX_TURN_ANGLE_DEG)).step_by(1) {
            let angle_rad = f32::to_radians(angle_deg as f32);
            let x = LEG_DISTANCE * angle_rad.cos();
            let y = LEG_DISTANCE * angle_rad.sin();
            let direction_vec2 = Vec2::new(x, y);
            let world_pos_f32 = self.get_world_vec2() + direction_vec2;
            let world_pos_int = (x as i32 + self.position.0, y as i32 + self.position.1);

            let height_here = NOISE_AMPLITUDE * simplex_noise_2d_seeded(world_pos_f32 / NOISE_SCALE, SEED);
            let slope = get_absolute_slope(LEG_DISTANCE, self.height as f32 / NODE_HEIGHT_MULTIPLIER as f32, height_here);

            // create new pathnode instance, determine cost, and add to the result vector
            let node = PathNode {
                position: world_pos_int,
                height: (height_here * NODE_HEIGHT_MULTIPLIER as f32) as i32,
                current_world_angle_deg: angle_deg_between_vec2(direction_vec2, Vec2::new(1.0, 0.0)),
            };
            let cost = (slope * 1000.) as u32;
            result.push((node, cost));
        }

        result
    }

    fn heuristic(&self) -> u32 {
        let dist = WIDTH as i32 - self.position.0;
        if dist < 0 {
            0
        } else {
            (WIDTH as i32 - self.position.0) as u32
        }
    }

    fn success(&self) -> bool {
        self.position.0 >= WIDTH as i32
    }
}

fn main() {
    let start_pos_int = (0, (HEIGHT as f32 / 2.) as i32);
    let start_height = NOISE_AMPLITUDE * simplex_noise_2d_seeded(Vec2::new(start_pos_int.0 as f32, start_pos_int.1 as f32) / NOISE_SCALE, SEED);
    let start_node = PathNode {
        position: start_pos_int,
        height: (start_height * NODE_HEIGHT_MULTIPLIER as f32) as i32,
        current_world_angle_deg: 0,
    };
    
    let result = astar(&start_node, |node| node.successors(), |node| node.heuristic(), |node| node.success());
    let nodes = result.unwrap().0;

    let mut image = RgbaImage::new(WIDTH, HEIGHT);
    for x in 0..WIDTH {
        for y in 0..HEIGHT {
            let noise = (NOISE_AMPLITUDE * simplex_noise_2d_seeded(Vec2::new(x as f32, y as f32) / NOISE_SCALE, SEED) * 10.);
            image.put_pixel(x as u32, y as u32, Rgba([255, 255, 255, (noise + 100.) as u8]));
        }
    }

    let red = Rgba([255, 0, 0, 255]);
    for i in 0..(nodes.len() - 1) {
        let f = &nodes[i];
        let s = &nodes[i + 1];
        imageproc::drawing::draw_line_segment_mut(&mut image, (f.position.0 as f32, f.position.1 as f32), (s.position.0 as f32, s.position.1 as f32), red);
    }

    image.save("assets/result_image.png").unwrap();
}

extern crate rand;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate nphysics;
extern crate nphysics_testbed2d;

use std::num::Float;
use std::rand::{StdRng, SeedableRng, Rng};
use std::sync::Arc;
use na::{Pnt2, Vec2, Translation};
use ncollide::shape::{Cuboid, Mesh, Mesh2};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed2d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0, 9.81));

    /*
     * Mesh
     */
    let num_split = 5;
    let begin     = -75.0;
    let max_h     = 15.0;
    let begin_h   = 15.0;
    let step      = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices = Vec::from_fn(num_split + 2, |i| Pnt2::new(begin + (i as f32) * step, 0.0));
    let mut indices  = Vec::new();
    let mut rng: StdRng = SeedableRng::from_seed([1, 2, 3, 4].as_slice());

    for i in range(0u, num_split) {
        let h: f32 = rng.gen();
        vertices[i + 1].y = begin_h - h * max_h;

        indices.push(i);
        indices.push(i + 1);
    }
    indices.push(num_split);
    indices.push(num_split + 1);

    let mesh: Mesh2<f32> = Mesh::new(Arc::new(vertices), Arc::new(indices), None, None);
    let rb = RigidBody::new_static(mesh, 0.3, 0.6);

    world.add_body(rb);

    /*
     * Create the boxes
     */
    let width   = 100;
    let height  = 20;
    let rad     = 0.5;
    let shift   = 2.0 * rad;
    let centerx = shift * (width as f32) / 2.0;

    for i in range(0u, height) {
        for j in range(0u, width) {
            let fj = j as f32;
            let fi = i as f32;
            let x = fj * 2.0 * rad - centerx;
            let y = -fi * 2.0 * rad;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vec2::new(rad, rad)), 1.0, 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            world.add_body(rb);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}

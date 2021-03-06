extern crate "nalgebra" as na;
extern crate ncollide;
extern crate nphysics;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::Vec3;
use ncollide::shape::{Mesh, Mesh3};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    let meshes = Testbed::load_obj("media/great_hall.obj");

    for (vertices, indices) in meshes.into_iter() {
        let vertices = vertices.iter().map(|v| *v * 3.0).collect();
        let mesh: Mesh3<f32> = Mesh::new(Arc::new(vertices), Arc::new(indices), None, None);

        world.add_body(RigidBody::new_static(mesh, 0.3, 0.6));
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}

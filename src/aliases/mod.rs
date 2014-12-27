//! Aliases for complicated parameterized types.

use std::sync::Arc;
use std::sync::RWLock;
use ncollide::bounding_volume::AABB;
use ncollide::broad_phase::DBVTBroadPhase;
// use integration::SweptBallMotionClamping;
use object::RigidBody;
use math::{Scalar, Point};

pub type DefaultBroadPhase = DBVTBroadPhase<Scalar, Point, Arc<RWLock<RigidBody>>, AABB<Point>>;
// pub type DefaultSweptBallMotionClamping = SweptBallMotionClamping<DefaultBroadPhase>;

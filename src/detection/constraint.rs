//! Data structure to describe a constraint between two rigid bodies.

use std::sync::Arc;
use std::sync::RWLock;
use ncollide::geometry::Contact;
use object::RigidBody;
use detection::joint::{Fixed, BallInSocket};
use math::{Scalar, Point, Vect};

/// A constraint between two rigid bodies.
pub enum Constraint {
    /// A contact.
    RBRB(Arc<RWLock<RigidBody>>, Arc<RWLock<RigidBody>>, Contact<Scalar, Point, Vect>),
    /// A ball-in-socket joint.
    BallInSocket(Arc<RWLock<BallInSocket>>),
    /// A fixed joint.
    Fixed(Arc<RWLock<Fixed>>),
}

impl Clone for Constraint {
    fn clone(&self) -> Constraint {
        match *self {
            Constraint::RBRB(ref a, ref b, ref c) => Constraint::RBRB(a.clone(), b.clone(), c.clone()),
            Constraint::BallInSocket(ref bis) => Constraint::BallInSocket(bis.clone()),
            Constraint::Fixed(ref f) => Constraint::Fixed(f.clone()),
        }
    }
}

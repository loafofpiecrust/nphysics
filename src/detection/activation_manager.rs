use std::num::FloatMath;
use std::sync::Arc;
use std::sync::RWLock;
use na;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use world::RigidBodyCollisionWorld;
use detection::constraint::Constraint;
use detection::joint::{JointManager, Joint};
use object::{RigidBody, ActivationState};
use utils::union_find::UnionFindSet;
use utils::union_find;
use math::Scalar;

/// Structure that monitors island-based activation/deactivation of objects.
///
/// It is responsible for making objects sleep or wake up.
pub struct ActivationManager {
    mix_factor:     Scalar,
    ufind:          Vec<UnionFindSet>,
    can_deactivate: Vec<bool>,
    to_activate:    Vec<Arc<RWLock<RigidBody>>>,
}

impl ActivationManager {
    /// Creates a new `ActivationManager`.
    ///
    /// # Arguments:
    /// * `thresold`   - the minimum energy required to keep an object awake.
    /// * `mix_factor` - the ratio of energy to keep between two frames.
    pub fn new(mix_factor: Scalar) -> ActivationManager {
        assert!(mix_factor >= na::zero(), "The energy mixing factor must be between 0.0 and 1.0.");

        ActivationManager {
            mix_factor:     mix_factor,
            ufind:          Vec::new(),
            can_deactivate: Vec::new(),
            to_activate:    Vec::new(),
        }
    }

    /// Notify the `ActivationManager` that is has to activate an object at the next update.
    // FIXME: this is not a very good name
    pub fn will_activate(&mut self, b: &Arc<RWLock<RigidBody>>) {
        if b.read().can_move() && !b.read().is_active() {
            self.to_activate.push(b.clone());
        }
    }

    fn update_energy(&self, b: &mut RigidBody) {
        match b.deactivation_threshold() {
            Some(threshold) => {
                // FIXME: take the time in account (to make a true RWA)
                let _1         = na::one::<Scalar>();
                let new_energy = (_1 - self.mix_factor) * b.activation_state().energy() +
                    self.mix_factor * (na::sqnorm(&b.lin_vel()) + na::sqnorm(&b.ang_vel()));

                b.activate(new_energy.min(threshold * na::cast(4.0f64)));
            },
            None => { }
        }
    }

    /// Update the activation manager, activating and deactivating objects when needed.
    pub fn update(&mut self,
                  world:  &mut RigidBodyCollisionWorld,
                  joints: &JointManager,
                  bodies: &HashMap<uint, Arc<RWLock<RigidBody>>, UintTWHash>) {
        /*
         *
         * Update bodies energy
         *
         */
        for (i, b) in bodies.elements().iter().enumerate() {
            let mut b = b.value.write();

            assert!(*b.activation_state() != ActivationState::Deleted);
            if b.is_active() {
                self.update_energy(&mut *b);
            }

            b.set_index(i as int);
        }

        /*
         *
         * Activate bodies that need it.
         *
         */
        for b in self.to_activate.iter() {
            let mut rb = b.write();

            match rb.deactivation_threshold() {
                Some(threshold) => rb.activate(threshold * na::cast(2.0f64)),
                None => { }
            }
        }

        self.to_activate.clear();

        /*
         *
         * Build islands to deactivate those with low-energy objects only.
         *
         */
        // Resize buffers.
        if bodies.len() > self.ufind.len() {
            let to_add = bodies.len() - self.ufind.len();
            self.ufind.grow(to_add, UnionFindSet::new(0));
            self.can_deactivate.grow(to_add, false);
        }
        else {
            self.ufind.truncate(bodies.len());
            self.can_deactivate.truncate(bodies.len());
        }

        // Init the union find.
        for (i, u) in self.ufind.iter_mut().enumerate() {
            u.reinit(i)
        }

        for d in self.can_deactivate.iter_mut() {
            *d = true
        }

        // Run the union-find.
        fn make_union(b1: &Arc<RWLock<RigidBody>>, b2: &Arc<RWLock<RigidBody>>, ufs: &mut [UnionFindSet]) {
            let rb1 = b1.read();
            let rb2 = b2.read();

            if rb1.can_move() && rb2.can_move() {
                union_find::union(rb1.index() as uint, rb2.index() as uint, ufs)
            }
        }

        world.contact_pairs(|b1, b2, cd| {
            if cd.num_colls() != 0 {
                make_union(b1, b2, self.ufind.as_mut_slice())
            }
        });

        for e in joints.joints().elements().iter() {
            match e.value {
                Constraint::RBRB(ref b1, ref b2, _) => make_union(b1, b2, self.ufind.as_mut_slice()),
                Constraint::BallInSocket(ref b)   => {
                    match (b.read().anchor1().body.as_ref(), b.read().anchor2().body.as_ref()) {
                        (Some(b1), Some(b2)) => make_union(b1, b2, self.ufind.as_mut_slice()),
                        _ => { }
                    }
                },
                Constraint::Fixed(ref f)   => {
                    match (f.read().anchor1().body.as_ref(), f.read().anchor2().body.as_ref()) {
                        (Some(b1), Some(b2)) => make_union(b1, b2, self.ufind.as_mut_slice()),
                        _ => { }
                    }
                }
            }
        }

        /*
         * Body activation/deactivation.
         */
        // Find deactivable islands.
        for i in range(0u, self.ufind.len()) {
            let root = union_find::find(i, self.ufind.as_mut_slice());
            let b    = bodies.elements()[i].value.read();

            self.can_deactivate[root] =
                match b.deactivation_threshold() {
                    Some(threshold) => {
                        self.can_deactivate[root] && b.activation_state().energy() < threshold
                    },
                    None => false
                };
        }

        // Activate/deactivate islands.
        for i in range(0u, self.ufind.len()) {
            let root = union_find::find(i, self.ufind.as_mut_slice());
            let mut b = bodies.elements()[i].value.write();

            if self.can_deactivate[root] { // Everybody in this set can be deactivacted.
                b.deactivate();
            }
            else { // Everybody in this set must be reactivated.
                if !b.is_active() && b.can_move() {
                    match b.deactivation_threshold() {
                        Some(threshold) => b.activate(threshold * na::cast(2.0f64)),
                        None => { }
                    }
                }
            }
        }
    }
}

use std::sync::Arc;
use std::sync::RWLock;
// use rand::RngUtil;
use na::{Translation, Transformation, RotationWithTranslation};
use na;
use math::{Scalar, Vect, Orientation, Matrix};
use detection::constraint::Constraint;
use detection::joint::Joint;
use object::RigidBody;
use resolution::constraint::velocity_constraint::VelocityConstraint;
use resolution::constraint::contact_equation;
use resolution::constraint::contact_equation::{CorrectionMode, CorrectionParameters};
use resolution::constraint::ball_in_socket_equation;
use resolution::constraint::fixed_equation;
use resolution::solver::Solver;
use resolution::constraint::projected_gauss_seidel_solver as pgs;
use resolution::constraint::projected_gauss_seidel_solver::Velocities;
use resolution::constraint::impulse_cache::ImpulseCache;


/// Constraint solver using the projected gauss seidel algorithm and warm-starting.
pub struct AccumulatedImpulseSolver {
	correction:              CorrectionParameters,
	cache:                   ImpulseCache,
	num_first_order_iter:    uint,
	num_second_order_iter:   uint,
	restitution_constraints: Vec<VelocityConstraint>,
	friction_constraints:    Vec<VelocityConstraint>,
	mj_lambda:               Vec<Velocities>
}

impl AccumulatedImpulseSolver {
	/// Creates a new `AccumulatedImpulseSolver`.
	pub fn new(step:                  Scalar,
			   correction_mode:       CorrectionMode,
			   joint_corr_factor:     Scalar,
			   rest_eps:              Scalar,
			   num_first_order_iter:  uint,
			   num_second_order_iter: uint)
			   -> AccumulatedImpulseSolver {
		AccumulatedImpulseSolver {
			num_first_order_iter:    num_first_order_iter,
			num_second_order_iter:   num_second_order_iter,
			restitution_constraints: Vec::new(),
			friction_constraints:    Vec::new(),
			mj_lambda:               Vec::new(),
			cache:                   ImpulseCache::new(step, na::dim::<Vect>()),

			correction: CorrectionParameters {
				corr_mode:  correction_mode,
				joint_corr: joint_corr_factor,
				rest_eps:   rest_eps
			}
		}
	}

	/// Gets the number of iteration done by the penetration depth correction solver.
	#[inline]
	pub fn num_first_order_iter(&self) -> uint {
		self.num_first_order_iter
	}

	/// Sets the number of iteration done by the penetration depth correction solver.
	#[inline]
	pub fn set_num_first_order_iter(&mut self, num: uint) {
		self.num_first_order_iter = num
	}

	/// Gets the number of iteration done by the velocity constraint solver.
	#[inline]
	pub fn num_second_order_iter(&self) -> uint {
		self.num_second_order_iter
	}

	/// Sets the number of iteration done by the velocity constraint solver.
	#[inline]
	pub fn set_num_second_order_iter(&mut self, num: uint) {
		self.num_second_order_iter = num
	}

	fn resize_buffers(&mut self, num_restitution_equations: uint, num_friction_equations: uint) {
		resize_buffer(&mut self.restitution_constraints,
					  num_restitution_equations,
					  VelocityConstraint::new());

		resize_buffer(&mut self.friction_constraints,
					  num_friction_equations,
					  VelocityConstraint::new());
	}

	fn do_solve(&mut self,
				dt:          Scalar,
				constraints: &[Constraint],
				joints:      &[uint],
				bodies:      &[Arc<RWLock<RigidBody>>]) {
		let num_friction_equations    = (na::dim::<Vect>() - 1) * self.cache.len();
		let num_restitution_equations = self.cache.len();
		let mut num_joint_equations = 0;

		for i in joints.iter() {
			match constraints[*i] {
				Constraint::BallInSocket(_) => {
					num_joint_equations = num_joint_equations + na::dim::<Vect>()
				},
				Constraint::Fixed(_) => {
					num_joint_equations = num_joint_equations + na::dim::<Vect>() + na::dim::<Orientation>()
				},
				Constraint::RBRB(_, _, _) => { }
			}
		}

		self.resize_buffers(num_restitution_equations + num_joint_equations, num_friction_equations);

		let mut friction_offset = 0;

		for (i, (_, &(ci, imp))) in self.cache.hash().iter().enumerate() {
			match constraints[ci] {
				Constraint::RBRB(ref rb1, ref rb2, ref c) => {
					contact_equation::fill_second_order_equation(
						dt.clone(),
						c,
						rb1.read().deref(), rb2.read().deref(),
						&mut self.restitution_constraints[i],
						i,
						self.friction_constraints.as_mut_slice(),
						friction_offset,
						self.cache.impulsions_at(imp),
						&self.correction);
				},
				_ => { }
			}

			friction_offset = friction_offset + na::dim::<Vect>() - 1;
		}

		let mut joint_offset = num_restitution_equations;
		for i in joints.iter() {
			let nconstraints = self.restitution_constraints.len();
			match constraints[*i] {
				Constraint::BallInSocket(ref bis) => {
					ball_in_socket_equation::fill_second_order_equation(
						dt.clone(),
						bis.read().deref(),
						self.restitution_constraints.slice_mut(joint_offset, nconstraints), // XXX
						&self.correction
					);

					joint_offset = joint_offset + na::dim::<Vect>();
				},
				Constraint::Fixed(ref f) => {
					fixed_equation::fill_second_order_equation(
						dt.clone(),
						f.read().deref(),
						self.restitution_constraints.slice_mut(joint_offset, nconstraints), // XXX
						&self.correction
					);

					joint_offset = joint_offset + na::dim::<Vect>() + na::dim::<Orientation>();
				},
				Constraint::RBRB(_, _, _) => { }
			}
		}

		resize_buffer(&mut self.mj_lambda, bodies.len(), Velocities::new());

		// FIXME: parametrize by the resolution algorithm?
		pgs::projected_gauss_seidel_solve(
			self.restitution_constraints.as_mut_slice(),
			self.friction_constraints.as_mut_slice(),
			self.mj_lambda.as_mut_slice(),
			bodies.len(),
			self.num_second_order_iter,
			false);

		// FIXME: this is _so_ ugly!
		self.resize_buffers(num_restitution_equations, num_friction_equations);

		for b in bodies.iter() {
			let mut rb = b.write();
			let i      = rb.index();

			let curr_lin_vel = rb.lin_vel();
			let curr_ang_vel = rb.ang_vel();

			rb.set_lin_vel(curr_lin_vel + self.mj_lambda[i as uint].lv);
			rb.set_ang_vel(curr_ang_vel + self.mj_lambda[i as uint].av);
		}

		for (i, dv) in self.restitution_constraints.iter().enumerate() {
			let imps = self.cache.push_impulsions();
			imps[0]  = dv.impulse * na::cast(0.85f64);

			for j in range(0u, na::dim::<Vect>() - 1) {
				let ref fc = self.friction_constraints[i * (na::dim::<Vect>() - 1) + j];
				imps[1 + j] = fc.impulse * na::cast(0.85f64);
			}
		}

		let offset = self.cache.reserved_impulse_offset();
		for (i, (_, kv)) in self.cache.hash_mut().iter_mut().enumerate() {
			*kv = (kv.val0(), offset + i * na::dim::<Vect>());
		}

		/*
		 * first order resolution
		 */
		let needs_correction = !na::is_zero(&self.correction.corr_mode.pos_corr_factor()) &&
			constraints.iter().any(|constraint| {
			match *constraint {
				Constraint::RBRB(_, _, ref c) =>
					c.depth >= self.correction.corr_mode.min_depth_for_pos_corr(),
				_ => false // no first order resolution for joints
			}
		});

		if needs_correction {
			self.resize_buffers(num_restitution_equations, num_friction_equations);

			for (i, (_, &(ci, _))) in self.cache.hash().iter().enumerate() {
				match constraints[ci] {
					Constraint::RBRB(_, _, ref c) => {
						contact_equation::reinit_to_first_order_equation(
							dt.clone(),
							c,
							&mut self.restitution_constraints[i],
							&self.correction);
					},
					_ => { }
				}
			}

			// FIXME: parametrize by the resolution algorithm?
			pgs::projected_gauss_seidel_solve(
				self.restitution_constraints.as_mut_slice(),
				[].as_mut_slice(),
				self.mj_lambda.as_mut_slice(),
				bodies.len(),
				self.num_first_order_iter,
				true);

			for b in bodies.iter() {
				let mut rb = b.write();
				let i      = rb.index();

				let translation = self.mj_lambda[i as uint].lv * dt;
				let rotation    = self.mj_lambda[i as uint].av * dt;

				let center = &rb.center_of_mass().clone();

				let mut delta: Matrix = na::one();
				delta.append_rotation_wrt_point(&rotation, center.as_vec());
				delta.append_translation(&translation);

				rb.append_transformation(&delta);
			}
		}
	}
}

impl Solver<Constraint> for AccumulatedImpulseSolver {
	fn solve(&mut self, dt: Scalar, constraints: &[Constraint]) {
		// FIXME: bodies index assignment is very ugly
		let mut bodies = Vec::new();

		if constraints.len() != 0 {
			/*
			 * Associate the constraints with the cached impulse.
			 */
			for (i, cstr) in constraints.iter().enumerate() {
				match *cstr {
					Constraint::RBRB(ref a, ref b, ref c) => {
						self.cache.insert(i,
										  a.deref() as *const RWLock<RigidBody> as uint,
										  b.deref() as *const RWLock<RigidBody> as uint,
										  na::center(&c.world1, &c.world2));
					},
					Constraint::BallInSocket(_) => {
						// XXX: cache for ball in socket?
					},
					Constraint::Fixed(_) => {
						// XXX: cache for fixed?
					}
				}
			}

			/*
			 * Assign an index to each body.
			 */
			// This is a two-passes assignation of index to the rigid bodies.
			// This is not very good, but is the only way to do that without having a separate list
			// of all rigid bodies.
			for c in constraints.iter() {
				match *c {
					Constraint::RBRB(ref a, ref b, _) => {
						a.write().set_index(-2);
						b.write().set_index(-2)
					},
					Constraint::BallInSocket(ref bis) => {
						let bbis = bis.read();
						match bbis.anchor1().body {
							Some(ref b) => {
								b.write().set_index(-2)
							},
							None    => { }
						};

						match bbis.anchor2().body {
							Some(ref b) => {
								b.write().set_index(-2)
							},
							None    => { }
						}
					}
					Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
						let bf = f.read();
						match bf.anchor1().body {
							Some(ref b) => {
								b.write().set_index(-2)
							},
							None    => { }
						};

						match bf.anchor2().body {
							Some(ref b) => {
								b.write().set_index(-2)
							},
							None    => { }
						}
					}
				}
			}

			let mut id = 0;

			fn set_body_index(a: &Arc<RWLock<RigidBody>>, bodies: &mut Vec<Arc<RWLock<RigidBody>>>, id: &mut int) {
				let mut ba = a.write();
				if ba.index() == -2 {
					if ba.can_move() {
						ba.set_index(*id);
						bodies.push(a.clone());
						*id = *id + 1;
					}
					else {
						ba.set_index(-1)
					}
				}
			}

			// FIXME: avoid allocation
			let mut joints = Vec::new();
			for (i, c) in constraints.iter().enumerate() {
				match *c {
					Constraint::RBRB(ref a, ref b, _) => {
						set_body_index(a, &mut bodies, &mut id);
						set_body_index(b, &mut bodies, &mut id);
					},
					Constraint::BallInSocket(ref bis) => {
						joints.push(i);
						let bbis = bis.read();
						match bbis.anchor1().body {
							Some(ref b) => set_body_index(b, &mut bodies, &mut id),
							None        => { }
						}

						match bbis.anchor2().body {
							Some(ref b) => set_body_index(b, &mut bodies, &mut id),
							None        => { }
						}
					},
					Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
						joints.push(i);
						let bf = f.read();
						match bf.anchor1().body {
							Some(ref b) => set_body_index(b, &mut bodies, &mut id),
							None        => { }
						}

						match bf.anchor2().body {
							Some(ref b) => set_body_index(b, &mut bodies, &mut id),
							None        => { }
						}
					}
				}
			}

			self.do_solve(dt.clone(), constraints, joints.as_slice(), bodies.as_slice());
			self.cache.swap();
		}
	}
}

fn resize_buffer<A: Clone>(buff: &mut Vec<A>, size: uint, val: A) {
	if buff.len() < size {
		let diff = size - buff.len();
		buff.grow(diff, val);
	}
	else {
		buff.truncate(size)
	}
}

use std::sync::RWLock;
use std::sync::Arc;
use std::ptr;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use detection::activation_manager::ActivationManager;
use detection::detector::Detector;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::joint::joint::Joint;
use detection::constraint::Constraint;
use object::RigidBody;

/// Structure that handles creation and removal of joints.
pub struct JointManager {
	joints:      HashMap<uint, Constraint, UintTWHash>,
	body2joints: HashMap<uint, Vec<Constraint>, UintTWHash>
}

impl JointManager {
	/// Creates a new `JointManager`.
	pub fn new() -> JointManager {
		JointManager {
			joints:      HashMap::new(UintTWHash::new()),
			body2joints: HashMap::new(UintTWHash::new())
		}
	}

	/// Joints handled by this manager.
	#[inline]
	pub fn joints(&self) -> &HashMap<uint, Constraint, UintTWHash> {
		&self.joints
	}

	/// List of joints attached to a specific body.
	#[inline]
	pub fn joints_with_body(&self, body: &Arc<RWLock<RigidBody>>) -> Option<&[Constraint]> {
		self.body2joints.find(&(body.deref() as *const RWLock<RigidBody> as uint)).map(|v| v.as_slice())
	}

	/// Add a `BallInSocket` joint to this manager.
	///
	/// This will force the activation of the two objects attached to the joint.
	pub fn add_ball_in_socket(&mut self,
							  joint:      Arc<RWLock<BallInSocket>>,
							  activation: &mut ActivationManager) {
		if self.joints.insert(joint.deref() as *const RWLock<BallInSocket> as uint,
							  Constraint::BallInSocket(joint.clone())) {
			match joint.read().anchor1().body.as_ref() {
				Some(b) => {
					activation.will_activate(b);
					let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RWLock<RigidBody> as uint,
																  || Some(Vec::new()));
					js.unwrap().push(Constraint::BallInSocket(joint.clone()));
				},
				_ => { }
			}

			match joint.read().anchor2().body.as_ref() {
				Some(b) => {
					activation.will_activate(b);
					let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RWLock<RigidBody> as uint,
																  || Some(Vec::new()));
					js.unwrap().push(Constraint::BallInSocket(joint.clone()));
				},
				_ => { }
			}
		}
	}

	/// Removes a `BallInSocket` joint from this manager.
	///
	/// This will force the activation of the two objects attached to the joint.
	pub fn remove_ball_in_socket(&mut self, joint: &Arc<RWLock<BallInSocket>>, activation: &mut ActivationManager) {
		if self.joints.remove(&(joint.deref() as *const RWLock<BallInSocket> as uint)) {
			let _  = joint.read().anchor1().body.as_ref().map(|b| activation.will_activate(b));
			let _  = joint.read().anchor2().body.as_ref().map(|b| activation.will_activate(b));
		}
	}

	/// Add a `Fixed` joint to this manager.
	///
	/// This will force the activation of the two objects attached to the joint.
	pub fn add_fixed(&mut self, joint: Arc<RWLock<Fixed>>, activation: &mut ActivationManager) {
		if self.joints.insert(joint.deref() as *const RWLock<Fixed> as uint, Constraint::Fixed(joint.clone())) {
			match joint.read().anchor1().body.as_ref() {
				Some(b) => {
					activation.will_activate(b);
					let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RWLock<RigidBody> as uint,
																  || Some(Vec::new()));
					js.unwrap().push(Constraint::Fixed(joint.clone()));
				},
				_ => { }
			}

			match joint.read().anchor2().body.as_ref() {
				Some(b) => {
					activation.will_activate(b);
					let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RWLock<RigidBody> as uint,
																  || Some(Vec::new()));
					js.unwrap().push(Constraint::Fixed(joint.clone()));
				},
				_ => { }
			}
		}
	}

	/// Removes a joint from this manager.
	///
	/// This will force the activation of the two objects attached to the joint.
	pub fn remove_joint<T: Joint<M>+Send+Sync, M>(&mut self,
										joint:      &Arc<RWLock<T>>,
										activation: &mut ActivationManager) {
		if self.joints.remove(&(joint.deref() as *const RWLock<T> as uint)) {
			self.remove_joint_for_body(joint, joint.read().anchor1().body.as_ref(), activation);
			self.remove_joint_for_body(joint, joint.read().anchor2().body.as_ref(), activation);
		}
	}

	fn remove_joint_for_body<T: Joint<M>, M>(&mut self,
											 joint:      &Arc<RWLock<T>>,
											 body:       Option<&Arc<RWLock<RigidBody>>>,
											 activation: &mut ActivationManager) {
		match body {
			Some(b) => {
				activation.will_activate(b);
				let key = b.deref() as *const RWLock<RigidBody> as uint;
				match self.body2joints.find_mut(&key) {
					Some(ref mut js) => {
						let jkey = joint.deref() as *const RWLock<T>;
						js.retain(|j| {
							// we do not know the type of the joint, so cast it to uint for
							// comparison.
							let id = match *j {
								Constraint::RBRB(_, _, _) => ptr::null::<uint>() as uint,
								Constraint::BallInSocket(ref b) => b.deref() as *const RWLock<BallInSocket> as uint,
								Constraint::Fixed(ref f) => f.deref() as *const RWLock<Fixed> as uint
							};

							id != jkey as uint
						});
					}
					None => { }
				}
			}
			None => { }
		}
	}

	/// Removes every joint attached to a given rigid body.
	///
	/// This will force the activation of every object attached to the deleted joints.
	pub fn remove(&mut self, b: &Arc<RWLock<RigidBody>>, activation: &mut ActivationManager) {
		for joints in self.body2joints.get_and_remove(&(b.deref() as *const RWLock<RigidBody> as uint)).iter() {
			for joint in joints.value.iter() {
				fn do_remove<T: Joint<M>+Send+Sync, M>(_self:      &mut JointManager,
											 joint:      &Arc<RWLock<T>>,
											 b:          &Arc<RWLock<RigidBody>>,
											 activation: &mut ActivationManager) {
					let bj    = joint.read();
					let body1 = bj.anchor1().body.as_ref();
					let body2 = bj.anchor2().body.as_ref();

					for body in bj.anchor1().body.as_ref().iter() {
						if (*body).deref() as *const RWLock<RigidBody> == b.deref() as *const RWLock<RigidBody> {
							_self.remove_joint_for_body(joint, body2, activation);
						}
						else {
							_self.remove_joint_for_body(joint, body1, activation);
						}
					}
				}

				match *joint {
					Constraint::BallInSocket(ref bis) => do_remove(self, bis, b, activation),
					Constraint::Fixed(ref f)          => do_remove(self, f, b, activation),
					Constraint::RBRB(_, _, _) => panic!("Internal error: a contact RBRB should not be here.")
				}
			}
		}
	}

	// FIXME: do we really want to handle this here instead of in the activation manager directly?
	/// Activates the objects that interact with an activated object through a joint.
	pub fn update(&mut self, activation: &mut ActivationManager) {
		for joint in self.joints.elements().iter() {
			match joint.value {
				Constraint::BallInSocket(ref bis) => {
					let mut bbis = bis.write();
					if !bbis.up_to_date() {
						// the joint has been invalidated by the user: wake up the attached bodies
						bbis.update();
						match bbis.anchor1().body {
							Some(ref b) => activation.will_activate(b),
							None        => { }
						}
						match bbis.anchor2().body {
							Some(ref b) => activation.will_activate(b),
							None        => { }
						}
					}
				},
				Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
					let mut bf = f.write();
					if !bf.up_to_date() {
						// the joint has been invalidated by the user: wake up the attached bodies
						bf.update();
						match bf.anchor1().body {
							Some(ref b) => activation.will_activate(b),
							None        => { }
						}
						match bf.anchor2().body {
							Some(ref b) => activation.will_activate(b),
							None        => { }
						}
					}
				},
				Constraint::RBRB(_, _, _) => panic!("Internal error:��a contact RBRB should not be here.")

			}
		}
	}

	/// Collects all the constraints caused by joints.
	pub fn interferences(&mut self, constraint: &mut Vec<Constraint>) {
		for joint in self.joints.elements().iter() {
			constraint.push(joint.value.clone())
		}
	}
}

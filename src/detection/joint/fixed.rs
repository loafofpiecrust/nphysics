use math::Matrix;
use detection::joint::anchor::Anchor;
use detection::joint::joint::Joint;

/// A joint that prevents any relative movement (linear and angular) between two objects.
pub struct Fixed {
    up_to_date: bool,
    anchor1:    Anchor<Matrix>,
    anchor2:    Anchor<Matrix>,
}

impl Fixed {
    /// Creates a new `Fixed` joint.
    pub fn new(anchor1: Anchor<Matrix>, anchor2: Anchor<Matrix>) -> Fixed {
        Fixed {
            up_to_date: false,
            anchor1:    anchor1,
            anchor2:    anchor2
        }
    }

    /// Tells if the joint has been modified by the user.
    pub fn up_to_date(&self) -> bool {
        self.up_to_date
    }

    #[doc(hidden)]
    pub fn update(&mut self) {
        self.up_to_date = true
    }

    /// Sets the the second anchor position.
    ///
    /// The position is expressed in the second attached body’s local coordinates.
    pub fn set_local1(&mut self, local1: Matrix) {
        if local1 != self.anchor1.position {
            self.up_to_date = false;
            self.anchor1.position = local1
        }
    }

    /// Sets the the second anchor position.
    ///
    /// The position is expressed in the second attached body’s local coordinates.
    pub fn set_local2(&mut self, local2: Matrix) {
        if local2 != self.anchor2.position {
            self.up_to_date = false;
            self.anchor2.position = local2
        }
    }
}

impl Joint<Matrix> for Fixed {
    /// The first anchor affected by this joint.
    #[inline]
    fn anchor1(&self) -> &Anchor<Matrix> {
        &self.anchor1
    }

    /// The second anchor affected by this joint.
    #[inline]
    fn anchor2(&self) -> &Anchor<Matrix> {
        &self.anchor2
    }

    /// The first attach point in global coordinates.
    #[inline]
    fn anchor1_pos(&self) -> Matrix {
        match self.anchor1.body {
            Some(ref b) => {
                *b.read().position() * self.anchor1.position
            },
            None => self.anchor1.position.clone()
        }
    }

    /// The second attach point in global coordinates.
    #[inline]
    fn anchor2_pos(&self) -> Matrix {
        match self.anchor2.body {
            Some(ref b) => {
                *b.read().position() * self.anchor2.position
            },
            None => self.anchor2.position.clone()
        }
    }
}

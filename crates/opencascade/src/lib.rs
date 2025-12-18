use std::sync::LazyLock;

use nalgebra::{vector, Point3, RealField, Scalar, UnitQuaternion, UnitVector3, Vector3};
use simba::scalar::SubsetOf;
use thiserror::Error;

pub mod angle;
pub mod bounding_box;
pub mod mesh;
pub mod primitives;
pub mod section;
pub mod workplane;

mod law_function;
mod make_pipe_shell;

#[derive(Error, Debug)]
pub enum Error {
    #[error("failed to write STL file")]
    StlWriteFailed,
    #[error("failed to read STEP file")]
    StepReadFailed,
    #[error("failed to read IGES file")]
    IgesReadFailed,
    #[error("failed to write STEP file")]
    StepWriteFailed,
    #[error("failed to write IGES file")]
    IgesWriteFailed,
    #[error("failed to triangulate Shape")]
    TriangulationFailed,
    #[error("encountered a face with no triangulation")]
    UntriangulatedFace,
    #[error("at least 2 points are required for creating a wire")]
    NotEnoughPoints,
}

pub(crate) const X_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![1.0, 0.0, 0.0]);
pub(crate) const Y_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![0.0, 1.0, 0.0]);
pub(crate) const Z_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![0.0, 0.0, 1.0]);
pub(crate) const BASE_NORMAL: UnitVector3<f64> = Z_NORMAL;
pub(crate) static INTER_QUAT: LazyLock<UnitQuaternion<f64>> =
    LazyLock::new(|| UnitQuaternion::rotation_between(&Z_NORMAL, &X_NORMAL).unwrap());

#[derive(Debug, Clone, Copy)]
pub struct TandR<F: Scalar + RealField + Clone + Copy> {
    pub translation: Vector3<F>,
    pub rotation_quat: UnitQuaternion<F>,
    pub inverse: bool,
}

impl<F: Scalar + RealField + Clone + Copy> Default for TandR<F> {
    fn default() -> Self {
        Self {
            translation: Vector3::<F>::zeros(),
            rotation_quat: UnitQuaternion::<F>::identity(),
            inverse: false,
        }
    }
}

impl<F: Scalar + RealField + Clone + Copy> TandR<F> {
    pub fn new(translation: Vector3<F>, rotation_quat: UnitQuaternion<F>) -> Self {
        Self { translation, rotation_quat, inverse: false }
    }

    pub fn translation(mut self, translation: Vector3<F>) -> Self {
        self.translation = translation;

        self
    }

    pub fn from_rotation_between(a: &UnitVector3<F>, b: &UnitVector3<F>) -> Self {
        let rotation_quat = match UnitQuaternion::rotation_between(&a, &b) {
            Some(quat) => quat,
            None => {
                let quat_1 = INTER_QUAT.cast();
                let inter_norm = quat_1 * a;

                let quat_2 = UnitQuaternion::rotation_between(&inter_norm, &b).unwrap();

                quat_2 * quat_1
            },
        };

        Self { translation: Vector3::zeros(), rotation_quat, inverse: false }
    }

    pub fn transform_point(&self, mut point: Point3<F>) -> Point3<F> {
        if self.inverse {
            point = point + self.translation;
            point = self.rotation_quat * point;

            point
        } else {
            point = self.rotation_quat * point;
            point = point + self.translation;

            point
        }
    }

    pub fn transform_tandr(&self, mut tandr: Self) -> Self {
        tandr.rotation_quat = self.rotation_quat * tandr.rotation_quat;
        tandr.translation = self.translation + (self.rotation_quat * tandr.translation);

        tandr
    }

    pub fn rotate_norm(&self, mut normal: UnitVector3<F>) -> UnitVector3<F> {
        normal = self.rotation_quat * normal;

        normal
    }

    pub fn inverse(&self) -> Self {
        Self {
            translation: -self.translation,
            rotation_quat: self.rotation_quat.inverse(),
            inverse: !self.inverse,
        }
    }

    pub fn cast<T: Scalar + RealField + Clone + Copy>(&self) -> TandR<T>
    where
        F: SubsetOf<T>,
    {
        TandR {
            translation: self.translation.cast(),
            rotation_quat: self.rotation_quat.cast(),
            inverse: self.inverse,
        }
    }
}

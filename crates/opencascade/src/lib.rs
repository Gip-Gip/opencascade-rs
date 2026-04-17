use std::sync::LazyLock;

use crate::primitives::Edge;
use crate::primitives::Shape;
use crate::primitives::Wire;
use cxx::UniquePtr;
use nalgebra::vector;
use nalgebra::Matrix4;
use nalgebra::Point3;
use nalgebra::RealField;
use nalgebra::Scalar;
use nalgebra::UnitQuaternion;
use nalgebra::UnitVector3;
use nalgebra::Vector3;
use opencascade_sys::ffi::gp_Trsf;
use opencascade_sys::ffi::TopoDS_Shape;
use opencascade_sys::ffi::{self};
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
    #[error("builder would throw StdFail_NotDone")]
    NotDone,
    #[error("start and end point are identical")]
    StackedPoints,
}

pub(crate) const X_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![1.0, 0.0, 0.0]);
pub(crate) const Y_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![0.0, 1.0, 0.0]);
pub(crate) const Z_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![0.0, 0.0, 1.0]);
pub(crate) const BASE_NORMAL: UnitVector3<f64> = Z_NORMAL;
pub(crate) static INTER_QUAT: LazyLock<UnitQuaternion<f64>> =
    LazyLock::new(|| UnitQuaternion::rotation_between(&Z_NORMAL, &X_NORMAL).unwrap());

pub struct TopExpExplorerIter {
    explorer: UniquePtr<ffi::TopExp_Explorer>,
}

impl Iterator for TopExpExplorerIter {
    type Item = Shape;
    fn next(&mut self) -> Option<Self::Item> {
        if self.explorer.More() {
            let topo_shape = self.explorer.Current();

            let shape = Shape::from_shape(topo_shape);

            self.explorer.pin_mut().Next();

            Some(shape)
        } else {
            None
        }
    }
}

impl TopExpExplorerIter {
    pub fn new(shape: &Shape, to_find: ffi::TopAbs_ShapeEnum) -> Self {
        let explorer = ffi::TopExp_Explorer_ctor(&shape.inner, to_find);

        Self { explorer }
    }
}

pub struct WireExplorerIter {
    explorer: UniquePtr<ffi::BRepTools_WireExplorer>,
}

impl Iterator for WireExplorerIter {
    type Item = Edge;
    fn next(&mut self) -> Option<Self::Item> {
        if self.explorer.More() {
            let topo_edge = self.explorer.Current();

            let edge = Edge::from_edge(topo_edge);

            self.explorer.pin_mut().Next();

            Some(edge)
        } else {
            None
        }
    }
}

impl WireExplorerIter {
    pub fn new(wire: &Wire) -> Self {
        let explorer = ffi::BRepTools_WireExplorer_ctor(&wire.inner);

        Self { explorer }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
        Self {
            translation,
            rotation_quat,
            inverse: false,
        }
    }

    pub fn translation(mut self, translation: Vector3<F>) -> Self {
        self.translation = translation;

        self
    }

    pub fn from_rotation_between(a: &UnitVector3<F>, b: &UnitVector3<F>) -> Self {
        let rotation_quat = match UnitQuaternion::rotation_between(a, b) {
            Some(quat) => quat,
            None => {
                let quat_1 = INTER_QUAT.cast();
                let inter_norm = quat_1 * a;

                let quat_2 = UnitQuaternion::rotation_between(&inter_norm, b).unwrap();

                quat_2 * quat_1
            }
        };

        Self {
            translation: Vector3::zeros(),
            rotation_quat,
            inverse: false,
        }
    }

    pub fn from_axis_angle(axis: &UnitVector3<F>, angle: F) -> Self {
        let rotation_quat = UnitQuaternion::from_axis_angle(axis, angle);

        Self {
            translation: Vector3::zeros(),
            rotation_quat,
            inverse: false,
        }
    }

    pub fn transform_point(&self, mut point: Point3<F>) -> Point3<F> {
        if self.inverse {
            point += self.translation;
            point = self.rotation_quat * point;

            point
        } else {
            point = self.rotation_quat * point;
            point += self.translation;

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

impl TandR<f64> {
    pub fn transform_shape(&self, shape: &ffi::TopoDS_Shape) -> UniquePtr<TopoDS_Shape> {
        let transform: UniquePtr<ffi::gp_Trsf> = self.into();

        let mut transformer = ffi::BRepBuilderAPI_Transform_ctor(shape, &transform, false);

        let new_shape = transformer.pin_mut().Shape();

        ffi::TopoDS_Shape_to_owned(new_shape)
    }
}

impl From<&TandR<f64>> for UniquePtr<gp_Trsf> {
    fn from(value: &TandR<f64>) -> Self {
        let is_inverse = value.inverse;

        let non_inverse_tandr = match is_inverse {
            true => value.inverse(),
            false => *value,
        };

        let mut occ_transform = ffi::new_transform();

        let vec = non_inverse_tandr.translation;
        let quat = non_inverse_tandr.rotation_quat;

        let occ_vec = ffi::new_vec(vec.x, vec.y, vec.z);
        let occ_quat = ffi::new_quaternion(quat.i, quat.j, quat.k, quat.w);

        occ_transform
            .pin_mut()
            .SetTransformation(&occ_quat, &occ_vec);

        if is_inverse {
            occ_transform.pin_mut().Invert();
        }

        occ_transform
    }
}

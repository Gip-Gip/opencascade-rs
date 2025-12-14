use std::sync::LazyLock;

use crate::primitives::{Edge, Wire};
use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector3, point, vector};

const X_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![1.0, 0.0, 0.0]);
const Y_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![0.0, 1.0, 0.0]);
const Z_NORMAL: UnitVector3<f64> = UnitVector3::new_unchecked(vector![0.0, 0.0, 1.0]);
const BASE_NORMAL: UnitVector3<f64> = Z_NORMAL;
static INTER_QUAT: LazyLock<UnitQuaternion<f64>> = LazyLock::new(|| {UnitQuaternion::rotation_between(&Z_NORMAL, &X_NORMAL).unwrap()});

#[derive(Debug, Clone, Copy)]
pub struct TandR {
    pub translation: Vector3<f64>,
    pub rotation_quat: UnitQuaternion<f64>,
    pub inverse: bool,
}

impl TandR {
    pub fn new(translation: Vector3<f64>, rotation_quat: UnitQuaternion<f64>) -> Self {
        Self { translation, rotation_quat, inverse: false }
    }

    /// Do no translation
    pub fn noop() -> Self {
        Self {
            translation: Vector3::zeros(),
            rotation_quat: UnitQuaternion::identity(),
            inverse: false
        }
    }

    pub fn from_rotation_between(a: &UnitVector3<f64>, b: &UnitVector3<f64>) -> Self {
        let rotation_quat = match UnitQuaternion::rotation_between(&a, &b) {
            Some(quat) => quat,
            None => {
                let quat_1 = *INTER_QUAT;
                let inter_norm = quat_1 * a;

                let quat_2 = UnitQuaternion::rotation_between(&inter_norm, &b).unwrap();

                quat_2 * quat_1
            }
        };

        Self {
            translation: Vector3::zeros(),
            rotation_quat,
            inverse: false,
        }
    }

    pub fn transform_point(&self, mut point: Point3<f64>) -> Point3<f64> {
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

    pub fn inverse(&self) -> Self {
        Self { translation: -self.translation, rotation_quat: self.rotation_quat.inverse(), inverse: !self.inverse }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Plane {
    XY,
    YZ,
    ZX,
    XZ,
    YX,
    ZY,
    Custom { x_dir: UnitVector3<f64>, normal_dir: UnitVector3<f64> },
}

impl Plane {
    pub fn transform_point(&self, point: Point3<f64>) -> Point3<f64> {
        self.transform().transform_point(point)
    }

    pub fn transform(&self) -> TandR {
        //match self {
        //    Self::XY => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::x(), Vector4::y(), Vector4::z(), Vector4::zeros()])),
        //    Self::YZ => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::y(), Vector4::z(), Vector4::x(), Vector4::zeros()])),
        //    Self::ZX => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::z(), Vector4::x(), Vector4::y(), Vector4::zeros()])),
        //    Self::XZ => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::x(), Vector4::z(), -Vector4::y(), Vector4::zeros()])),
        //    Self::YX => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::y(), Vector4::x(), -Vector4::z(), Vector4::zeros()])),
        //    Self::ZY => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::z(), Vector4::y(), -Vector4::x(), Vector4::zeros()])),
        //    Self::Front => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::x(), Vector4::y(), Vector4::z(), Vector4::zeros()])),
        //    Self::Back => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[-Vector4::x(), Vector4::y(), -Vector4::z(), Vector4::zeros()])),
        //    Self::Left => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::z(), Vector4::y(), -Vector4::x(), Vector4::zeros()])),
        //    Self::Right => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[-Vector4::z(), Vector4::y(), Vector4::x(), Vector4::zeros()])),
        //    Self::Top => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::x(), -Vector4::z(), Vector4::y(), Vector4::zeros()])),
        //    Self::Bottom => Affine3::from_matrix_unchecked(Matrix4::from_columns(&[Vector4::x(), Vector4::z(), -Vector4::y(), Vector4::zeros()])),
        //    Self::Custom { x_dir, normal_dir } => {
        //        let x_axis = vector![x_dir[0], x_dir[1], x_dir[2], 0.0].normalize();
        //        let z_axis = vector![normal_dir[0], normal_dir[1], normal_dir[2], 0.0].normalize();
        //        let y_axis = z_axis.cross(&x_axis).normalize();

        //        Affine3::from_matrix_unchecked(Matrix4::from_columns(&[x_axis, y_axis, z_axis, Vector4::zeros()]))
        //    },
        //}

        match self {
            Self::XY => TandR::noop(),
            Self::YZ => TandR::from_rotation_between(&BASE_NORMAL, &X_NORMAL),
            Self::ZX => TandR::from_rotation_between(&BASE_NORMAL, &Y_NORMAL),
            Self::YX => TandR::from_rotation_between(&BASE_NORMAL, &-BASE_NORMAL),
            Self::ZY => TandR::from_rotation_between(&BASE_NORMAL, &-X_NORMAL),
            Self::XZ => TandR::from_rotation_between(&BASE_NORMAL, &-Y_NORMAL),
            _ => todo!(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Workplane {
    transform: TandR,
}

impl Workplane {
    pub fn new(x_dir: UnitVector3<f64>, normal_dir: UnitVector3<f64>) -> Self {
        Self {
            transform: Plane::Custom {
                x_dir,
                normal_dir,
            }
            .transform(),
        }
    }

    pub fn xy() -> Self {
        Self { transform: Plane::XY.transform() }
    }

    pub fn yz() -> Self {
        Self { transform: Plane::YZ.transform() }
    }

    pub fn zx() -> Self {
        Self { transform: Plane::ZX.transform() }
    }

    pub fn xz() -> Self {
        Self { transform: Plane::XZ.transform() }
    }

    pub fn zy() -> Self {
        Self { transform: Plane::ZY.transform() }
    }

    pub fn yx() -> Self {
        Self { transform: Plane::YX.transform() }
    }

    pub fn origin(&self) -> Vector3<f64> {
        self.transform.translation
    }

    pub fn normal(&self) -> UnitVector3<f64> {
        self.transform.rotation_quat * UnitVector3::new_unchecked(Vector3::z())
    }

    pub fn x_dir(&self) -> UnitVector3<f64> {
        self.transform.rotation_quat * UnitVector3::new_unchecked(Vector3::x())
    }

    pub fn y_dir(&self) -> UnitVector3<f64> {
        self.transform.rotation_quat * UnitVector3::new_unchecked(Vector3::y())
    }

    pub fn set_rotation(&mut self, rotation_quat: UnitQuaternion<f64>) {
        self.transform.rotation_quat = rotation_quat
    }

    pub fn rotate_by(&mut self, rotation_quat: UnitQuaternion<f64>) {
        self.transform.rotation_quat = rotation_quat * self.transform.rotation_quat;
    }

    pub fn set_translation(&mut self, pos: Vector3<f64>) {
        self.transform.translation = pos;
    }

    pub fn translate_by(&mut self, offset: Vector3<f64>) {
        self.transform.translation += offset;
    }

    pub fn transformed(&self, tandr: TandR) -> Self {
        let mut new = self.clone();
        new.transform.translation += tandr.translation;

        new.rotate_by(tandr.rotation_quat);

        new
    }

    pub fn translated(&self, offset: Vector3<f64>) -> Self {
        let mut new = self.clone();
        let new_origin = new.to_world_pos(offset.into());
        new.transform.translation = new_origin.coords;

        new
    }

    pub fn rotated(&self, rotation_quat: UnitQuaternion<f64>) -> Self {
        let mut new = self.clone();
        new.rotate_by(rotation_quat);

        new
    }

    pub fn to_world_pos(&self, pos: Point3<f64>) -> Point3<f64> {
        self.transform.transform_point(pos)
    }

    pub fn to_local_pos(&self, pos: Point3<f64>) -> Point3<f64> {
        self.transform.inverse().transform_point(pos)
    }

    pub fn rect(&self, width: f64, height: f64) -> Wire {
        let half_width = width / 2.0;
        let half_height = height / 2.0;

        let p1 = self.to_world_pos(point![-half_width, half_height, 0.0]);
        let p2 = self.to_world_pos(point![half_width, half_height, 0.0]);
        let p3 = self.to_world_pos(point![half_width, -half_height, 0.0]);
        let p4 = self.to_world_pos(point![-half_width, -half_height, 0.0]);

        let top = Edge::segment(p1, p2);
        let right = Edge::segment(p2, p3);
        let bottom = Edge::segment(p3, p4);
        let left = Edge::segment(p4, p1);

        Wire::from_edges([&top, &right, &bottom, &left])
    }

    pub fn circle(&self, x: f64, y: f64, radius: f64) -> Wire {
        let center = self.to_world_pos(point![x, y, 0.0]);

        let circle = Edge::circle(center, self.normal(), radius);

        Wire::from_edges([&circle])
    }

    pub fn sketch(&self) -> Sketch {
        let cursor = self.to_world_pos(Point3::origin());
        Sketch::new(cursor, self.clone())
    }
}

pub struct Sketch {
    first_point: Option<Point3<f64>>,
    cursor: Point3<f64>, // cursor is in global coordinates
    workplane: Workplane,
    edges: Vec<Edge>,
}

impl Sketch {
    fn new(cursor: Point3<f64>, workplane: Workplane) -> Self {
        Self { first_point: None, cursor, workplane, edges: Vec::new() }
    }

    fn add_edge(&mut self, edge: Edge) {
        if self.first_point.is_none() {
            self.first_point = Some(edge.start_point());
        }

        self.edges.push(edge);
    }

    pub fn move_to(mut self, x: f64, y: f64) -> Self {
        self.cursor = self.workplane.to_world_pos(point![x, y, 0.0]);
        self
    }

    pub fn line_to(mut self, x: f64, y: f64) -> Self {
        let new_point = self.workplane.to_world_pos(point![x, y, 0.0]);
        let new_edge = Edge::segment(self.cursor, new_point);
        self.cursor = new_point;

        self.add_edge(new_edge);

        self
    }

    pub fn line_dx(mut self, dx: f64) -> Self {
        let cursor = self.workplane.to_local_pos(self.cursor);
        let new_point = self.workplane.to_world_pos(point![cursor.x + dx, cursor.y, 0.0]);
        let new_edge = Edge::segment(self.cursor, new_point);
        self.cursor = new_point;

        self.add_edge(new_edge);

        self
    }

    pub fn line_dy(mut self, dy: f64) -> Self {
        let cursor = self.workplane.to_local_pos(self.cursor);
        let new_point = self.workplane.to_world_pos(point![cursor.x, cursor.y + dy, 0.0]);
        let new_edge = Edge::segment(self.cursor, new_point);
        self.cursor = new_point;

        self.add_edge(new_edge);

        self
    }

    pub fn line_dx_dy(mut self, dx: f64, dy: f64) -> Self {
        let cursor = self.workplane.to_local_pos(self.cursor);
        let new_point = self.workplane.to_world_pos(point![cursor.x + dx, cursor.y + dy, 0.0]);
        let new_edge = Edge::segment(self.cursor, new_point);
        self.cursor = new_point;

        self.add_edge(new_edge);

        self
    }

    pub fn arc(mut self, (x1, y1): (f64, f64), (x2, y2): (f64, f64), (x3, y3): (f64, f64)) -> Self {
        let p1 = self.workplane.to_world_pos(point![x1, y1, 0.0]);
        let p2 = self.workplane.to_world_pos(point![x2, y2, 0.0]);
        let p3 = self.workplane.to_world_pos(point![x3, y3, 0.0]);

        let new_arc = Edge::arc(p1, p2, p3);

        self.cursor = p3;

        self.add_edge(new_arc);

        self
    }

    pub fn three_point_arc(self, p2: (f64, f64), p3: (f64, f64)) -> Self {
        let cursor = self.workplane.to_local_pos(self.cursor);
        self.arc((cursor.x, cursor.y), p2, p3)
    }

    pub fn wire(self) -> Wire {
        Wire::from_edges(&self.edges)
    }

    pub fn close(mut self) -> Wire {
        let start_point = self.first_point.unwrap();

        let new_edge = Edge::segment(self.cursor, start_point);
        self.add_edge(new_edge);
        Wire::from_edges(&self.edges)
    }
}

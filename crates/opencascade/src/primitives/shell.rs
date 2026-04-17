use cxx::UniquePtr;
use opencascade_sys::ffi;

use crate::primitives::FaceIterator;
use crate::primitives::Wire;

pub struct Shell {
    pub(crate) inner: UniquePtr<ffi::TopoDS_Shell>,
}

impl AsRef<Shell> for Shell {
    fn as_ref(&self) -> &Shell {
        self
    }
}

impl Shell {
    pub(crate) fn from_shell(shell: &ffi::TopoDS_Shell) -> Self {
        let inner = ffi::TopoDS_Shell_to_owned(shell);

        Self { inner }
    }

    pub fn loft<T: AsRef<Wire>>(wires: impl IntoIterator<Item = T>) -> Self {
        let is_solid = false;
        let mut make_loft = ffi::BRepOffsetAPI_ThruSections_ctor(is_solid);

        for wire in wires.into_iter() {
            make_loft.pin_mut().AddWire(&wire.as_ref().inner);
        }

        // Set CheckCompatibility to `true` to avoid twisted results.
        make_loft.pin_mut().CheckCompatibility(true);

        let shape = make_loft.pin_mut().Shape();
        let shell = ffi::TopoDS_cast_to_shell(shape);

        Self::from_shell(shell)
    }

    pub fn from_faces(faces: FaceIterator) -> Self {
        let brep_builder = ffi::BRep_Builder_ctor();

        let topods_builder = ffi::BRep_Builder_upcast_to_topods_builder(&brep_builder);
        let mut shell_ptr = ffi::TopoDS_Shell_ctor();
        topods_builder.MakeShell(shell_ptr.pin_mut());

        let mut shell_shape = ffi::TopoDS_Shape_to_owned(ffi::cast_shell_to_shape(&shell_ptr));

        for face in faces {
            let face_shape = ffi::cast_face_to_shape(&face.inner);
            topods_builder.Add(shell_shape.pin_mut(), face_shape);
        }

        let shell = ffi::TopoDS_cast_to_shell(&shell_shape);

        Self::from_shell(shell)
    }
}

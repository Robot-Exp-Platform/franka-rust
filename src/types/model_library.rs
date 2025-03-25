use libloading::{Library, Symbol, os};
use robot_behavior::{RobotException, RobotResult};
use std::path::Path;

pub struct ModelLibrary {
    lib: Library,
}

macro_rules! lib_fn {
    (fn $name:ident($($arg:ident: $arg_ty:ty),*) from $symbol:ident) => {
        pub fn $name(&self, $($arg: $arg_ty),*) {
            unsafe {
                let func: Symbol<unsafe extern "C" fn($($arg_ty),*)> = self.lib.get(stringify!($symbol).as_bytes()).unwrap();
                func($($arg),*);
            }
        }

    };
}

impl ModelLibrary {
    pub fn new(model_filename: &Path) -> RobotResult<Self> {
        let libm = if cfg!(target_os = "linux") {
            unsafe {
                os::unix::Library::open(Path::new("libm.so.6"), libc::RTLD_NOW | libc::RTLD_GLOBAL)
            }
        } else {
            return Err(RobotException::ModelException("Unsupported OS".to_string()));
        };

        let lib = unsafe { Library::new(model_filename) }
            .map_err(|e| RobotException::ModelException(e.to_string()))?;

        Ok(ModelLibrary { lib })
    }

    lib_fn!(fn mass(q: &[f64; 7], i_load: &[f64; 9], m_load: &f64, x_load: &[f64; 3], m_ne: &mut [f64; 49]) from M_NE);
    lib_fn!(fn body_jacobian_joint1(body_ji_for_j1: &mut [f64; 42]) from Ji_J_J_J1);
    lib_fn!(fn body_jacobian_joint2(q: &[f64; 7], body_ji_for_j2: &mut [f64; 42]) from Ji_J_J_J2);
    lib_fn!(fn body_jacobian_joint3(q: &[f64; 7], body_ji_for_j3: &mut [f64; 42]) from Ji_J_J_J3);
    lib_fn!(fn body_jacobian_joint4(q: &[f64; 7], body_ji_for_j4: &mut [f64; 42]) from Ji_J_J_J4);
    lib_fn!(fn body_jacobian_joint5(q: &[f64; 7], body_ji_for_j5: &mut [f64; 42]) from Ji_J_J_J5);
    lib_fn!(fn body_jacobian_joint6(q: &[f64; 7], body_ji_for_j6: &mut [f64; 42]) from Ji_J_J_J6);
    lib_fn!(fn body_jacobian_joint7(q: &[f64; 7], body_ji_for_j7: &mut [f64; 42]) from Ji_J_J_J7);
    lib_fn!(fn body_jacobian_flange(q: &[f64; 7], body_ji_for_j8: &mut [f64; 42]) from Ji_J_J_J8);
    lib_fn!(fn body_jacobian_ee(q: &[f64; 7], pose_f_to_ee: &[f64; 16], body_ji_for_j9: &mut [f64; 42]) from Ji_J_J_J9);

    lib_fn!(fn zero_jacobian_joint1(zero_jo_for_j1: &mut [f64; 42]) from O_J_J_J1);
    lib_fn!(fn zero_jacobian_joint2(q: &[f64; 7], zero_jo_for_j2: &mut [f64; 42]) from O_J_J_J2);
    lib_fn!(fn zero_jacobian_joint3(q: &[f64; 7], zero_jo_for_j3: &mut [f64; 42]) from O_J_J_J3);
    lib_fn!(fn zero_jacobian_joint4(q: &[f64; 7], zero_jo_for_j4: &mut [f64; 42]) from O_J_J_J4);
    lib_fn!(fn zero_jacobian_joint5(q: &[f64; 7], zero_jo_for_j5: &mut [f64; 42]) from O_J_J_J5);
    lib_fn!(fn zero_jacobian_joint6(q: &[f64; 7], zero_jo_for_j6: &mut [f64; 42]) from O_J_J_J6);
    lib_fn!(fn zero_jacobian_joint7(q: &[f64; 7], zero_jo_for_j7: &mut [f64; 42]) from O_J_J_J7);
    lib_fn!(fn zero_jacobian_flange(q: &[f64; 7], zero_jo_for_j8: &mut [f64; 42]) from O_J_J_J8);
    lib_fn!(fn zero_jacobian_ee(q: &[f64; 7], pose_f_to_ee: &[f64; 16], zero_jo_for_j9: &mut [f64; 42]) from O_J_J_J9);

    lib_fn!(fn joint1(q: &[f64; 7], pose_f_to_j1: &mut [f64; 16]) from O_T_J1);
    lib_fn!(fn joint2(q: &[f64; 7], pose_f_to_j2: &mut [f64; 16]) from O_T_J2);
    lib_fn!(fn joint3(q: &[f64; 7], pose_f_to_j3: &mut [f64; 16]) from O_T_J3);
    lib_fn!(fn joint4(q: &[f64; 7], pose_f_to_j4: &mut [f64; 16]) from O_T_J4);
    lib_fn!(fn joint5(q: &[f64; 7], pose_f_to_j5: &mut [f64; 16]) from O_T_J5);
    lib_fn!(fn joint6(q: &[f64; 7], pose_f_to_j6: &mut [f64; 16]) from O_T_J6);
    lib_fn!(fn joint7(q: &[f64; 7], pose_f_to_j7: &mut [f64; 16]) from O_T_J7);
    lib_fn!(fn flange(q: &[f64; 7], pose_f_to_j8: &mut [f64; 16]) from O_T_J8);
    lib_fn!(fn ee(q: &[f64; 7], pose_f_to_ee: &[f64; 16], pose_f_to_j9: &mut [f64; 16]) from O_T_J9);

    lib_fn!(fn coriolis(q: &[f64; 7], qd: &[f64; 7], i_load: &[f64; 9], m_load: &f64, x_load: &[f64; 3], c_ne: &mut [f64; 7]) from c_NE);
    lib_fn!(fn gravity(q: &[f64; 7], g_earth: &[f64; 3], m_load: &f64, x_load: &[f64; 3], g_ne: &mut [f64; 7]) from g_NE);
}

#[cfg(target_os = "linux")]
use libloading::os;
use libloading::{Library, Symbol};
use robot_behavior::{RobotException, RobotResult};
use std::path::Path;

pub struct ModelLibrary {
    lib: Library,
    #[cfg(target_os = "linux")]
    _libm: os::unix::Library,
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
    ($symbol:ident, fn $name:ident() -> $out_ty:ty) => {
        pub fn $name(&self, out: &mut $out_ty) {
            unsafe {
                let func: Symbol<unsafe extern "C" fn(*mut libc::c_double)> = self.lib.get(stringify!($symbol).as_bytes()).unwrap();
                func(out.as_mut_ptr());
            }
        }

    };
    ($symbol:ident, fn $name:ident($($arg:ident: $arg_ty:ty),*) -> $out_ty:ty) => {
        pub fn $name(&self, $($arg: $arg_ty),*, out: &mut $out_ty) {
            unsafe {
                let func: Symbol<unsafe extern "C" fn($($arg_ty),*, *mut libc::c_double)> = self.lib.get(stringify!($symbol).as_bytes()).unwrap();
                func($($arg),*, out.as_mut_ptr());
            }
        }

    };
}

impl ModelLibrary {
    pub fn new(model_filename: &Path) -> RobotResult<Self> {
        #[cfg(target_os = "linux")]
        unsafe {
            let _libm = os::unix::Library::open(
                Some(Path::new("libm.so.6")),
                libc::RTLD_NOW | libc::RTLD_GLOBAL,
            )
            .map_err(|e| RobotException::ModelException(e.to_string()))?;
            let lib = Library::new(model_filename)
                .map_err(|e| RobotException::ModelException(e.to_string()))?;

            Ok(ModelLibrary { lib, _libm })
        }
        #[cfg(not(target_os = "linux"))]
        unsafe {
            let lib = Library::new(model_filename)
                .map_err(|e| RobotException::ModelException(e.to_string()))?;

            Ok(ModelLibrary { lib })
        }
    }

    lib_fn!(Ji_J_J1, fn body_jacobian_joint1() -> [f64; 42]);
    lib_fn!(Ji_J_J2, fn body_jacobian_joint2(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(Ji_J_J3, fn body_jacobian_joint3(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(Ji_J_J4, fn body_jacobian_joint4(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(Ji_J_J5, fn body_jacobian_joint5(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(Ji_J_J6, fn body_jacobian_joint6(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(Ji_J_J7, fn body_jacobian_joint7(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(Ji_J_J8, fn body_jacobian_flange(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(Ji_J_J9, fn body_jacobian_ee(q: &[f64; 7], pose_f_to_ee: &[f64; 16]) -> [f64; 42]);

    lib_fn!(O_J_J1, fn zero_jacobian_joint1() -> [f64; 42]);
    lib_fn!(O_J_J2, fn zero_jacobian_joint2(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(O_J_J3, fn zero_jacobian_joint3(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(O_J_J4, fn zero_jacobian_joint4(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(O_J_J5, fn zero_jacobian_joint5(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(O_J_J6, fn zero_jacobian_joint6(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(O_J_J7, fn zero_jacobian_joint7(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(O_J_J8, fn zero_jacobian_flange(q: &[f64; 7]) -> [f64; 42]);
    lib_fn!(O_J_J9, fn zero_jacobian_ee(q: &[f64; 7], pose_f_to_ee: &[f64; 16]) -> [f64; 42]);

    lib_fn!(O_T_J1, fn joint1(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J2, fn joint2(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J3, fn joint3(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J4, fn joint4(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J5, fn joint5(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J6, fn joint6(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J7, fn joint7(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J8, fn flange(q: &[f64; 7]) -> [f64; 16]);
    lib_fn!(O_T_J9, fn ee(q: &[f64; 7], pose_f_to_ee: &[f64; 16]) -> [f64; 16]);

    // lib_fn!(M_NE, fn mass(q: &[f64; 7], i_load: &[f64; 9], m_load: &f64, x_load: &[f64; 3]) -> [f64; 49]);
    // lib_fn!(c_NE, fn coriolis(q: &[f64; 7], dq: &[f64; 7], i_load: &[f64; 9], m_load: &f64, x_load: &[f64; 3]) -> [f64; 7]);
    // lib_fn!(g_NE, fn gravity(q: &[f64; 7], g_earth: &[f64; 3], m_load: &f64, x_load: &[f64; 3]) -> [f64; 7]);

    pub fn mass(
        &self,
        q: &[f64; 7],
        i_load: &[f64; 9],
        m_load: &f64,
        x_load: &[f64; 3],
        out: &mut [f64; 49],
    ) {
        unsafe {
            let func: Symbol<
                unsafe extern "C" fn(
                    *const libc::c_double,
                    *const libc::c_double,
                    libc::c_double,
                    *const libc::c_double,
                    *mut libc::c_double,
                ),
            > = self.lib.get(b"M_NE").unwrap();
            func(
                q.as_ptr(),
                i_load.as_ptr(),
                *m_load,
                x_load.as_ptr(),
                out.as_mut_ptr(),
            );
        }
    }

    pub fn coriolis(
        &self,
        q: &[f64; 7],
        dq: &[f64; 7],
        i_load: &[f64; 9],
        m_load: &f64,
        x_load: &[f64; 3],
        out: &mut [f64; 7],
    ) {
        unsafe {
            let func: Symbol<
                unsafe extern "C" fn(
                    *const libc::c_double,
                    *const libc::c_double,
                    *const libc::c_double,
                    libc::c_double,
                    *const libc::c_double,
                    *mut libc::c_double,
                ),
            > = self.lib.get(b"c_NE").unwrap();
            func(
                q.as_ptr(),
                dq.as_ptr(),
                i_load.as_ptr(),
                *m_load,
                x_load.as_ptr(),
                out.as_mut_ptr(),
            );
        }
    }

    pub fn gravity(
        &self,
        q: &[f64; 7],
        g_earth: &[f64; 3],
        m_load: &f64,
        x_load: &[f64; 3],
        out: &mut [f64; 7],
    ) {
        unsafe {
            let func: Symbol<
                unsafe extern "C" fn(
                    *const libc::c_double,
                    *const libc::c_double,
                    libc::c_double,
                    *const libc::c_double,
                    *mut libc::c_double,
                ),
            > = self.lib.get(b"g_NE").unwrap();
            func(
                q.as_ptr(),
                g_earth.as_ptr(),
                *m_load,
                x_load.as_ptr(),
                out.as_mut_ptr(),
            );
        }
    }
}

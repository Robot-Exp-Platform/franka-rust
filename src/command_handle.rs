use std::{
    fmt::Display,
    sync::{Arc, Mutex},
    time::Duration,
};

#[derive(Clone, Default)]
pub struct CommandHandle<R, S>
where
    R: Display + Clone + Send + Sync + 'static,
    S: Display + Clone + Default + Send + Sync + 'static,
{
    target: Arc<Mutex<Option<R>>>,
    closure: Arc<Mutex<Option<Box<dyn Fn(S, Duration) -> R + Send>>>>,
}

impl<R, S> CommandHandle<R, S>
where
    R: Display + Clone + Send + Sync + 'static,
    S: Display + Clone + Default + Send + Sync + 'static,
{
    pub fn new() -> Self {
        CommandHandle {
            target: Arc::new(Mutex::new(None)),
            closure: Arc::new(Mutex::new(None)),
        }
    }

    #[allow(unused)]
    pub fn set_target<T: Into<R>>(&self, target: T) {
        let mut target_lock = self.target.lock().unwrap();
        *target_lock = Some(target.into());
    }

    pub fn get_target(&self) -> Option<R> {
        let target_lock = self.target.lock().unwrap();
        target_lock.clone()
    }

    pub fn set_closure<F: Fn(S, Duration) -> R + Send + 'static>(&self, closure: F) {
        let mut closure_lock = self.closure.lock().unwrap();
        *closure_lock = Some(Box::new(closure));
    }

    pub fn run_closure(&self, state: S, duration: Duration) -> Option<R> {
        let closure_lock = self.closure.lock().unwrap();
        if let Some(ref closure) = *closure_lock {
            Some(closure(state, duration))
        } else {
            None
        }
    }

    pub fn command(&self, state: S, duration: Duration) -> Option<R> {
        match (self.run_closure(state, duration), self.get_target()) {
            (Some(res), None) => Some(res),
            (None, Some(target)) => Some(target),
            (Some(_), Some(_)) => {
                println!("There are both closures and target values, ignoring all of them");
                None
            }
            (None, None) => None,
        }
    }
}

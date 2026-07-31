use std::{
    fmt::Display,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

use crate::types::robot_types::CommandFilter;

type ControlClosure<R, S> = Option<Box<dyn FnMut(&S, Duration) -> R + Send>>;

pub(crate) struct ClosureActivation {
    active: Arc<AtomicBool>,
}

impl ClosureActivation {
    pub(crate) fn activate(self) {
        self.active.store(true, Ordering::Release);
    }
}

#[derive(Clone, Default)]
pub struct CommandHandle<R, S>
where
    R: Display + Clone + Send + Sync + 'static,
    S: Display + Clone + Default + Send + Sync + 'static,
{
    target: Arc<Mutex<Option<R>>>,
    closure: Arc<Mutex<ControlClosure<R, S>>>,
}

impl<R, S> CommandHandle<R, S>
where
    R: CommandFilter<S> + Display + Clone + Send + Sync + 'static,
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

    pub fn set_closure<F: FnMut(&S, Duration) -> R + Send + 'static>(&self, closure: F) {
        let mut closure_lock = self.closure.lock().unwrap();
        *closure_lock = Some(Box::new(closure));
    }

    pub(crate) fn set_paused_closure<F>(&self, initial: R, mut closure: F) -> ClosureActivation
    where
        F: FnMut(&S, Duration) -> R + Send + 'static,
    {
        let active = Arc::new(AtomicBool::new(false));
        let closure_active = active.clone();
        self.set_closure(move |state, duration| {
            if closure_active.load(Ordering::Acquire) {
                closure(state, duration)
            } else {
                initial.clone()
            }
        });
        ClosureActivation { active }
    }

    pub fn remove_closure(&self) {
        let mut closure_lock = self.closure.lock().unwrap();
        *closure_lock = None;
    }

    pub fn run_closure(&self, state: &S, duration: Duration) -> Option<R> {
        let mut closure_lock = self.closure.lock().unwrap();
        (*closure_lock)
            .as_mut()
            .map(|closure| closure(state, duration))
    }

    pub fn command(&self, state: &S, duration: Duration) -> Option<R> {
        match (self.run_closure(state, duration), self.get_target()) {
            (Some(res), None) => Some(res),
            (None, Some(target)) => Some(target),
            (Some(_), Some(_)) => {
                println!("There are both closures and target values, ignoring all of them");
                None
            }
            (None, None) => None,
        }
        .map(|res| res.filter(state))
    }
}

#[cfg(test)]
mod tests {
    use std::{
        fmt::{Display, Formatter},
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
    };

    use super::*;

    #[derive(Clone, Default)]
    struct TestCommand(u8);

    impl Display for TestCommand {
        fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
            write!(formatter, "{}", self.0)
        }
    }

    #[derive(Clone, Default)]
    struct TestState;

    impl Display for TestState {
        fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("test")
        }
    }

    impl CommandFilter<TestState> for TestCommand {
        fn filter(self, _state: &TestState) -> Self {
            self
        }
    }

    #[test]
    fn paused_closure_holds_until_activated() {
        let handle = CommandHandle::<TestCommand, TestState>::new();
        let calls = Arc::new(AtomicUsize::new(0));
        let closure_calls = calls.clone();
        let activation = handle.set_paused_closure(TestCommand(7), move |_, _| {
            closure_calls.fetch_add(1, Ordering::Relaxed);
            TestCommand(9)
        });

        assert_eq!(
            handle
                .command(&TestState, Duration::ZERO)
                .map(|value| value.0),
            Some(7)
        );
        assert_eq!(calls.load(Ordering::Relaxed), 0);

        activation.activate();

        assert_eq!(
            handle
                .command(&TestState, Duration::ZERO)
                .map(|value| value.0),
            Some(9)
        );
        assert_eq!(calls.load(Ordering::Relaxed), 1);
    }
}

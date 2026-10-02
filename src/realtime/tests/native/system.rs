//! Connect the real Franka capability, native rhythm and System macro using
//! only the loopback FCI peer. The application's executor has one thread.
use super::{gated_fixture, public_robot, run_bounded, runtime};
use crate::FrankaEmika;
use robot_behavior::{JointPositionControl, JointState, RobotResult, roplat::AsyncControlRhythm};
use roplat::{Lifecycle, Node, RoplatError, RoplatResult};
use std::{
    sync::{Arc, Mutex, atomic::Ordering},
    time::Duration,
};

type NativeRhythm = AsyncControlRhythm<FrankaEmika, JointPositionControl<7>>;
type Events = Arc<Mutex<Vec<&'static str>>>;

// Deliberately not Clone. Returning the same allocation verifies the externally
// created controller state survived the native session's asynchronous callback.
struct State {
    calls: usize,
}
struct Controller {
    state: Box<State>,
    events: Events,
}
impl Lifecycle for Controller {
    type Error = RoplatError;
    async fn on_init(&mut self) -> RoplatResult<()> {
        self.events.lock().unwrap().push("controller-init");
        Ok(())
    }
    async fn on_shutdown(&mut self) -> RoplatResult<()> {
        self.events.lock().unwrap().push("controller-shutdown");
        Ok(())
    }
}
impl Node for Controller {
    type Input = (JointState<7>, Duration);
    type Output = ([f64; 7], bool);
    async fn process(&mut self, _: Self::Input) -> Self::Output {
        tokio::task::yield_now().await;
        self.state.calls += 1;
        self.events.lock().unwrap().push("controller-process");
        ([0.0; 7], true)
    }
}

struct Source {
    robot: Option<FrankaEmika>,
    events: Events,
}
impl Lifecycle for Source {
    type Error = RoplatError;
    async fn on_init(&mut self) -> RoplatResult<()> {
        self.events.lock().unwrap().push("source-init");
        Ok(())
    }
    async fn on_shutdown(&mut self) -> RoplatResult<()> {
        assert!(self.robot.is_none());
        self.events.lock().unwrap().push("source-shutdown");
        Ok(())
    }
}
impl Node for Source {
    type Input = ();
    type Output = RobotResult<FrankaEmika>;
    async fn process(&mut self, (): ()) -> Self::Output {
        Ok(self.robot.take().expect("source executes once"))
    }
}

#[roplat::system]
async fn native_system(
    robot: FrankaEmika,
    mut controller: Controller,
    events: Events,
) -> RoplatResult<(FrankaEmika, Controller)> {
    let mut source = Source { robot: Some(robot), events };
    let mut control = NativeRhythm::new();

    source >> control >> |state| state >> controller;

    Ok((control.output, controller))
}

fn assert_send<T: Send>(value: T) -> T {
    value
}

#[test]
fn native_franka_system_is_send_returns_nodes_and_keeps_creator_lifecycle() {
    run_bounded(|| {
        let (robot, server, stage, release) = gated_fixture(false);
        let robot = public_robot(robot);
        let events = Events::default();
        let mut controller =
            Controller { state: Box::new(State { calls: 0 }), events: events.clone() };
        let state_address = (&*controller.state) as *const State as usize;

        runtime().block_on(async {
            // This is the controller's creating layer. The child System and
            // its native drive must neither reactivate nor close the controller.
            controller.on_init().await.unwrap();
            let system = tokio::spawn(assert_send(native_system(
                robot,
                controller,
                events.clone(),
            )));
            let progress = async {
                let mut last = 0;
                loop {
                    tokio::time::sleep(Duration::from_millis(1)).await;
                    let next = stage.load(Ordering::SeqCst);
                    if next != last {
                        release.send(next).unwrap();
                        last = next;
                    }
                    if next == 3 {
                        return;
                    }
                }
            };
            // The peer withholds each handshake fragment until this sibling
            // progresses. Blocking device I/O on this executor would fail.
            let (result, ()) = tokio::time::timeout(Duration::from_secs(5), async {
                tokio::join!(system, progress)
            })
            .await
            .expect("native System blocked its sibling");
            let (robot, mut controller) = result.unwrap().unwrap();
            assert_eq!(controller.state.calls, 1);
            assert_eq!((&*controller.state) as *const State as usize, state_address);
            assert!(!robot.session_flag());
            assert!(robot.robot_impl.motion_command_id.is_none());
            assert_eq!(
                *events.lock().unwrap(),
                [
                    "controller-init",
                    "source-init",
                    "controller-process",
                    "source-shutdown"
                ]
            );
            controller.on_shutdown().await.unwrap();
            assert_eq!(
                *events.lock().unwrap(),
                [
                    "controller-init",
                    "source-init",
                    "controller-process",
                    "source-shutdown",
                    "controller-shutdown"
                ]
            );
        });
        server.join().unwrap();
    });
}

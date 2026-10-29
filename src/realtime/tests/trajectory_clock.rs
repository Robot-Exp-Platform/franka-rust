//! Loopback-only observation of the actual public dense-trajectory execution clock.
use super::*;
use crate::FrankaEmika;
use roplat_behavior::{Joints,JointSpace,MoveTraj};
fn observe_clock(ids:&[u64])->Vec<(u64,f64,bool)>{
 let listener=TcpListener::bind("127.0.0.1:0").unwrap();let network=Network::new("127.0.0.1",listener.local_addr().unwrap().port());
 let client=UdpSocket::bind("127.0.0.1:0").unwrap();client.set_read_timeout(Some(Duration::from_secs(2))).unwrap();let addr=client.local_addr().unwrap();
 let peer=UdpSocket::bind("127.0.0.1:0").unwrap();peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
 let q=FrankaEmika::JOINT_DEFAULT;let cached=RobotStateInter{q_d:q,q,..Default::default()};let ids=ids.to_vec();
 let server=thread::spawn(move||{
 let (mut stream,_)=listener.accept().unwrap();stream.set_read_timeout(Some(Duration::from_secs(2))).unwrap();let(cmd,id)=read_request(&mut stream);assert_eq!(cmd,Command::Move as u32);stream.write_all(&response(Command::Move,id,1)).unwrap();
 let mut state=RobotStateInter{q_d:q,q,motion_generator_mode:MotionGeneratorMode::JointPosition,controller_mode:ControllerMode::JointImpedance,..Default::default()};let mut observed=Vec::new();let mut buf=[0;4096];
 for tick in ids{
 state.message_id=tick;peer.send_to(&bincode::serialize(&state).unwrap(),addr).unwrap();let(n,_)=peer.recv_from(&mut buf).unwrap();let c:RobotCommand=bincode::deserialize(&buf[..n]).unwrap();let v=c.motion.q_c;observed.push((tick,v[0],c.motion.motion_generation_finished));
 state.q_d=v;
 if c.motion.motion_generation_finished{state.message_id+=1;state.motion_generator_mode=MotionGeneratorMode::Idle;state.controller_mode=ControllerMode::Other;peer.send_to(&bincode::serialize(&state).unwrap(),addr).unwrap();stream.write_all(&response(Command::Move,id,0)).unwrap();return observed;}
 }panic!("fixture did not reach end");
 });
 let mut robot=FrankaEmika::from_test_impl(FrankaRobotImpl{network,udp_socket:client,robot_state:Arc::new(RwLock::new(cached)),motion_command_id:None});
 let path=(0..5).map(|i|{let mut p=q;p[0]+=i as f64*1e-7;p}).collect();
 <FrankaEmika as MoveTraj<JointSpace<7>>>::move_traj(&mut robot,path).unwrap();server.join().unwrap()
}
#[test]
fn dense_trajectory_gap_uses_robot_elapsed_time(){
 let out=observe_clock(&[1,4,5,6,7,8,9]);
 let expected=crate::utils::low_pass_filter(0.001,100.,0.,3e-7);
 assert!((out[1].1-expected).abs()<1e-14,"3ms elapsed must sample index3, got{} expected{}",out[1].1,expected);
 assert_eq!(out.last().unwrap().0,6,"finish at trajectory duration5ms, not callback count");
}
#[test]
fn dense_trajectory_nominal_1ms_sequence_keeps_existing_samples(){
 let out=observe_clock(&[1,2,3,4,5,6,7]);assert_eq!(out.len(),6);assert!(out.last().unwrap().2);
 let mut prev=0.;for (i,row) in out.iter().enumerate(){let raw=i.min(4) as f64*1e-7;let expected=crate::utils::low_pass_filter(0.001,100.,prev,raw);assert!((row.1-expected).abs()<1e-14);prev=expected;}
}

use franka_rust::FrankaEmika;
use robot_behavior::{MotionType, behavior::*};
use serde_json;
use std::io::Read;
use std::net::TcpListener;

fn main() {
    let mut robot = FrankaEmika::new("172.16.0.2");
    let tcp_listener = TcpListener::bind("127.0.0.1:8080").unwrap();
    let (mut stream, _addr) = tcp_listener.accept().unwrap();

    let mut handle = robot.start_streaming().unwrap();
    loop {
        let mut buffer = Vec::new();
        let mut temp_buf = [0; 1024];
        let bytes_read = stream.read(&mut temp_buf).unwrap();
        buffer.extend_from_slice(&temp_buf[..bytes_read]);

        let json_str = String::from_utf8(buffer).unwrap();
        let target: [f64; 7] = serde_json::from_str(&json_str).unwrap();
        handle.move_to(MotionType::Joint(target)).unwrap();
    }
}

use robot_behavior::RobotException;
use serde::{Serialize, de::DeserializeOwned};
use std::{
    io::{Read, Write},
    net::{TcpStream, UdpSocket},
    sync::{Arc, Mutex},
};

use crate::{exception::FrankaResult, types::robot_types::CommandIDConfig};

#[derive(Default)]
pub struct Network {
    tcp_stream: Option<TcpStream>,
    udp_socket: Option<UdpSocket>,

    command_counter: Arc<Mutex<u32>>,
}

impl Network {
    pub fn new(tcp_ip: &str, tcp_port: u16, udp_ip: &str, udp_port: u16) -> Self {
        let tcp_stream = TcpStream::connect(format!("{}:{}", tcp_ip, tcp_port)).ok();
        let udp_socket = UdpSocket::bind(format!("{}:{}", udp_ip, udp_port)).ok();
        Network {
            tcp_stream,
            udp_socket,
            command_counter: Arc::new(Mutex::new(0)),
        }
    }

    pub fn from_addr(ip: &str, port: u16) -> Self {
        Network::new(ip, port, "0.0.0.0", 0)
    }

    /// 发送并阻塞接收 tcp 指令
    pub fn tcp_send_and_recv<R, S>(&mut self, request: &mut R) -> FrankaResult<S>
    where
        R: Serialize + CommandIDConfig,
        S: DeserializeOwned + CommandIDConfig,
    {
        if let Some(stream) = &mut self.tcp_stream {
            let command_id = {
                let mut counter = self.command_counter.lock().unwrap();
                *counter += 1;
                *counter
            };
            request.set_command_id(command_id);
            stream.write_all(&bincode::serialize(&request).unwrap())?;
            let mut buffer = [0; 1024];
            let size = stream.read(&mut buffer)?;
            bincode::deserialize(&buffer[..size])
                .map_err(|e| RobotException::DeserializeError(e.to_string()))
        } else {
            Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ))
        }
    }

    /// 检查是否有可接收的 tcp 数据,如果有，则解析为对应的类型，如果没有，直接退出
    pub fn tcp_recv<S>(&mut self) -> FrankaResult<Option<S>>
    where
        S: DeserializeOwned + CommandIDConfig,
    {
        if let Some(stream) = &mut self.tcp_stream {
            let mut buffer = [0; 1024];
            let size = stream.read(&mut buffer)?;
            if size == 0 {
                return Ok(None);
            }
            bincode::deserialize(&buffer[..size])
                .map(|data| Some(data))
                .map_err(|e| RobotException::DeserializeError(e.to_string()))
        } else {
            Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ))
        }
    }

    /// 发送 udp 数据
    pub fn udp_send<R>(&mut self, request: R) -> FrankaResult<()>
    where
        R: Serialize,
    {
        if let Some(socket) = &mut self.udp_socket {
            let command_id = {
                let mut counter = self.command_counter.lock().unwrap();
                *counter += 1;
                *counter
            };
            let mut data = bincode::serialize(&request).unwrap();
            data.extend_from_slice(&command_id.to_be_bytes());
            socket.send(&data)?;
            Ok(())
        } else {
            Err(RobotException::NetworkError(
                "No active udp connection".to_string(),
            ))
        }
    }

    /// 阻塞接受 udp 数据
    pub fn udp_recv<S>(&mut self) -> FrankaResult<S>
    where
        S: DeserializeOwned,
    {
        if let Some(socket) = &mut self.udp_socket {
            let mut buffer = [0; 1024];
            let (size, _) = socket.recv_from(&mut buffer)?;
            bincode::deserialize(&buffer[..size])
                .map_err(|e| RobotException::DeserializeError(e.to_string()))
        } else {
            Err(RobotException::NetworkError(
                "No active udp connection".to_string(),
            ))
        }
    }
}

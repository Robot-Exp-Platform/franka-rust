use robot_behavior::{RobotException, RobotResult};
use serde::{Serialize, de::DeserializeOwned};
use std::{
    cmp::max,
    fmt::Debug,
    io::{Read, Write},
    net::TcpStream,
    sync::{Arc, Mutex},
};

use crate::types::robot_types::CommandIDConfig;

#[derive(Default, Clone)]
pub struct Network {
    tcp_stream: Option<Arc<Mutex<TcpStream>>>,
    command_counter: Arc<Mutex<u32>>,
}

impl Network {
    pub fn new(tcp_ip: &str, tcp_port: u16) -> Self {
        let tcp_stream = TcpStream::connect(format!("{tcp_ip}:{tcp_port}")).ok();

        if let Some(stream) = &tcp_stream {
            stream
                .set_write_timeout(Some(std::time::Duration::from_millis(3)))
                .unwrap();
        }

        Network {
            tcp_stream: tcp_stream.map(|stream| Arc::new(Mutex::new(stream))),
            command_counter: Arc::new(Mutex::new(0)),
        }
    }

    pub fn tcp_send_and_recv<R, S>(&mut self, request: &mut R) -> RobotResult<S>
    where
        R: Serialize + CommandIDConfig<u32> + Debug,
        S: DeserializeOwned + CommandIDConfig<u32>,
    {
        #[cfg(feature = "debug")]
        println!("tcp send {:?}", request);

        let command_id = self.next_command_id();
        let Some(stream) = &mut self.tcp_stream else {
            return Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ));
        };

        let mut stream = stream.lock().unwrap();
        request.set_command_id(command_id);
        let request = bincode::serialize(request).unwrap();
        stream.write_all(&request)?;

        let mut buffer = [0; 1024];
        let size = stream.read(&mut buffer)?;
        bincode::deserialize(&buffer[..size])
            .map_err(|err| RobotException::DeserializeError(err.to_string()))
    }

    pub fn tcp_blocking_recv<S>(&mut self) -> RobotResult<S>
    where
        S: DeserializeOwned + CommandIDConfig<u32> + Debug,
    {
        let Some(stream) = &mut self.tcp_stream else {
            return Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ));
        };

        let mut stream = stream.lock().unwrap();
        let mut buffer = vec![0_u8; size_of::<S>() + 4];
        stream.read_exact(&mut buffer)?;

        bincode::deserialize(&buffer)
            .map_err(|err| RobotException::DeserializeError(err.to_string()))
    }

    pub fn tcp_send_and_recv_buffer<R, S>(&mut self, request: &mut R) -> RobotResult<(S, Vec<u8>)>
    where
        R: Serialize + CommandIDConfig<u32> + Debug,
        S: DeserializeOwned + CommandIDConfig<u32>,
    {
        #[cfg(feature = "debug")]
        println!("tcp send {:?}", request);

        let command_id = self.next_command_id();
        let Some(stream) = &mut self.tcp_stream else {
            return Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ));
        };

        let mut stream = stream.lock().unwrap();
        request.set_command_id(command_id);
        let request = bincode::serialize(request).unwrap();

        #[cfg(feature = "debug")]
        println!("request :{:?}", request);

        stream.write_all(&request)?;
        let mut buffer = vec![0_u8; size_of::<S>() + 4];
        stream.read_exact(&mut buffer)?;
        let res = bincode::deserialize(&buffer)
            .map_err(|err| RobotException::DeserializeError(err.to_string()))?;

        let mut receive_buffer = Vec::new();
        let mut size_max = 0;
        loop {
            let mut buffer = vec![0_u8; 1024 * 5];
            if let Ok(size) = stream.read(&mut buffer) {
                receive_buffer.append(&mut buffer[..size].to_vec());
                if size < size_max {
                    break;
                }
                size_max = max(size_max, size);

                #[cfg(feature = "debug")]
                println!("size:{size}");
            } else {
                break;
            }
        }

        #[cfg(feature = "debug")]
        println!("receive size:{}", receive_buffer.len());

        Ok((res, receive_buffer))
    }

    fn next_command_id(&self) -> u32 {
        let mut counter = self.command_counter.lock().unwrap();
        *counter += 1;
        *counter
    }
}

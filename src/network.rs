use robot_behavior::{RobotException, RobotResult};
use serde::{Serialize, de::DeserializeOwned};
use std::{
    collections::VecDeque,
    fmt::Debug,
    io::{Read, Write},
    net::TcpStream,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use crate::types::robot_types::{Command, CommandHeader, CommandIDConfig};

#[derive(Clone)]
pub struct Network {
    tcp_stream: Option<Arc<Mutex<TcpStream>>>,
    command_bytes: usize,
    receive_deadline: Option<Instant>,
    command_counter: Arc<Mutex<u32>>,
    pending_responses: Arc<Mutex<VecDeque<Vec<u8>>>>,
}

impl Default for Network {
    fn default() -> Self {
        Self {
            tcp_stream: None,
            command_bytes: 4,
            receive_deadline: None,
            command_counter: Arc::new(Mutex::new(0)),
            pending_responses: Arc::new(Mutex::new(VecDeque::new())),
        }
    }
}

impl Network {
    pub(crate) fn replace_receive_deadline(
        &mut self,
        deadline: Option<Instant>,
    ) -> Option<Instant> {
        std::mem::replace(&mut self.receive_deadline, deadline)
    }

    pub(crate) fn for_gripper(mut self) -> Self {
        self.command_bytes = 2;
        self
    }

    pub fn new(tcp_ip: &str, tcp_port: u16) -> Self {
        let tcp_stream = TcpStream::connect(format!("{tcp_ip}:{tcp_port}")).ok();

        if let Some(stream) = &tcp_stream {
            stream
                .set_write_timeout(Some(std::time::Duration::from_millis(3)))
                .unwrap();
        }

        Network {
            command_bytes: 4,
            receive_deadline: None,
            tcp_stream: tcp_stream.map(|stream| Arc::new(Mutex::new(stream))),
            command_counter: Arc::new(Mutex::new(0)),
            pending_responses: Arc::new(Mutex::new(VecDeque::new())),
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
        let request = bincode::serialize(request)
            .map_err(|err| RobotException::CommandException(err.to_string()))?;
        stream.write_all(&request)?;

        // A Move terminal response can arrive before StopMove's response.
        // Preserve it for the session owner, and read whole TCP frames rather
        // than assuming one read is one response.
        let frame = receive_response(
            &mut stream,
            &mut self.pending_responses.lock().unwrap(),
            self.command_bytes,
            command_id,
            self.receive_deadline,
        )?;
        bincode::deserialize(&frame)
            .map_err(|err| RobotException::DeserializeError(err.to_string()))
    }

    pub fn tcp_blocking_recv<S>(&mut self, command_id: u32) -> RobotResult<S>
    where
        S: DeserializeOwned + CommandIDConfig<u32> + Debug,
    {
        let Some(stream) = &mut self.tcp_stream else {
            return Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ));
        };

        let mut stream = stream.lock().unwrap();
        let frame = receive_response(
            &mut stream,
            &mut self.pending_responses.lock().unwrap(),
            self.command_bytes,
            command_id,
            self.receive_deadline,
        )?;

        bincode::deserialize(&frame)
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
        let request = bincode::serialize(request)
            .map_err(|err| RobotException::CommandException(err.to_string()))?;

        #[cfg(feature = "debug")]
        println!("request :{:?}", request);

        stream.write_all(&request)?;
        let response_size = size_of::<S>() + 4;
        let mut response_buffer = vec![0_u8; response_size];
        stream.read_exact(&mut response_buffer)?;
        let res: S = bincode::deserialize(&response_buffer)
            .map_err(|err| RobotException::DeserializeError(err.to_string()))?;

        let header_size = size_of::<CommandHeader<{ Command::LoadModelLibrary }>>() + 4;
        let header: CommandHeader<{ Command::LoadModelLibrary }> =
            bincode::deserialize(&response_buffer[..header_size])
                .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
        if header.command_id != command_id {
            return Err(RobotException::NetworkError(format!(
                "unexpected TCP response command id: expected {command_id}, got {}",
                header.command_id
            )));
        }

        let total_size = header.size as usize;
        if total_size < response_size {
            return Err(RobotException::DeserializeError(format!(
                "invalid TCP response size: header reports {total_size} bytes, fixed response is {response_size} bytes"
            )));
        }

        let mut receive_buffer = vec![0_u8; total_size - response_size];
        stream.read_exact(&mut receive_buffer)?;

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

fn receive_response(
    stream: &mut TcpStream,
    pending: &mut VecDeque<Vec<u8>>,
    command_bytes: usize,
    command_id: u32,
    deadline: Option<Instant>,
) -> RobotResult<Vec<u8>> {
    if let Some(deadline) = deadline {
        remaining(deadline)?;
    }
    if let Some(index) = pending
        .iter()
        .position(|frame| response_id(frame, command_bytes) == command_id)
    {
        return Ok(pending.remove(index).unwrap());
    }
    loop {
        let frame = read_response_frame(stream, command_bytes, deadline)?;
        if response_id(&frame, command_bytes) == command_id {
            return Ok(frame);
        }
        pending.push_back(frame);
    }
}

pub(crate) fn remaining(deadline: Instant) -> RobotResult<Duration> {
    deadline
        .checked_duration_since(Instant::now())
        .filter(|value| !value.is_zero())
        .ok_or_else(|| {
            RobotException::NetworkError(
                "control-session cleanup deadline expired; physical stop is not confirmed".into(),
            )
        })
}

fn read_exact_until(
    stream: &mut TcpStream,
    buffer: &mut [u8],
    deadline: Option<Instant>,
) -> RobotResult<()> {
    let Some(deadline) = deadline else {
        stream.read_exact(buffer)?;
        return Ok(());
    };
    let previous = stream.read_timeout()?;
    let result = (|| {
        let mut offset = 0;
        while offset < buffer.len() {
            let timeout = remaining(deadline)?;
            stream.set_read_timeout(Some(previous.map_or(timeout, |old| old.min(timeout))))?;
            match stream.read(&mut buffer[offset..]) {
                Ok(0) => return Err(std::io::Error::from(std::io::ErrorKind::UnexpectedEof).into()),
                Ok(size) => {
                    offset += size;
                }
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => {}
                Err(error) => return Err(error.into()),
            }
        }
        Ok(())
    })();
    let restore: RobotResult<()> = stream.set_read_timeout(previous).map_err(Into::into);
    match (result, restore) {
        (Ok(()), result) | (result, Ok(())) => result,
        (Err(primary), Err(cleanup)) => Err(RobotException::ControlSession {
            primary: Box::new(primary),
            cleanup: Box::new(cleanup),
        }),
    }
}

// FCI command headers contain command, request id and total size (three u32s).
// This framing helper is only on the TCP command plane, not the UDP cycle path.
fn read_response_frame(
    stream: &mut TcpStream,
    command_bytes: usize,
    deadline: Option<Instant>,
) -> RobotResult<Vec<u8>> {
    let header_size = command_bytes + 8;
    const MAX_RESPONSE_SIZE: usize = 1024 * 1024;
    let mut header = [0_u8; 12];
    read_exact_until(stream, &mut header[..header_size], deadline)?;
    let size =
        u32::from_le_bytes(header[command_bytes + 4..header_size].try_into().unwrap()) as usize;
    if !(header_size..=MAX_RESPONSE_SIZE).contains(&size) {
        return Err(RobotException::DeserializeError(format!(
            "invalid TCP response size: {size}"
        )));
    }
    let mut frame = vec![0_u8; size];
    frame[..header_size].copy_from_slice(&header[..header_size]);
    read_exact_until(stream, &mut frame[header_size..], deadline)?;
    Ok(frame)
}

fn response_id(frame: &[u8], command_bytes: usize) -> u32 {
    u32::from_le_bytes(frame[command_bytes..command_bytes + 4].try_into().unwrap())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::robot_types::{MoveResponse, StopMoveResponse, StopMoveStatus};
    use std::{net::TcpListener, thread, time::Duration};

    fn receive_bytes(bytes: Vec<u8>, command_bytes: usize) -> RobotResult<Vec<u8>> {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut client = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let server = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            stream.write_all(&bytes).unwrap();
        });
        let result = read_response_frame(&mut client, command_bytes, None);
        server.join().unwrap();
        result
    }

    #[test]
    fn gripper_header_remains_ten_bytes() {
        let mut frame = Vec::new();
        frame.extend_from_slice(&4_u16.to_le_bytes());
        frame.extend_from_slice(&7_u32.to_le_bytes());
        frame.extend_from_slice(&12_u32.to_le_bytes());
        frame.extend_from_slice(&0_u16.to_le_bytes());
        let decoded = receive_bytes(frame.clone(), 2).unwrap();
        assert_eq!(decoded, frame);
        assert_eq!(response_id(&decoded, 2), 7);
    }

    #[test]
    fn invalid_and_truncated_frames_are_errors() {
        for size in [0_u32, 11, 1024 * 1024 + 1] {
            let mut header = vec![0; 8];
            header.extend_from_slice(&size.to_le_bytes());
            assert!(matches!(
                receive_bytes(header, 4),
                Err(RobotException::DeserializeError(_))
            ));
        }
        assert!(receive_bytes(vec![0; 5], 4).is_err());
        let mut header = vec![0; 8];
        header.extend_from_slice(&13_u32.to_le_bytes());
        assert!(receive_bytes(header, 4).is_err());
    }

    #[test]
    fn stop_status_matches_selected_fci_protocol() {
        #[cfg(not(feature = "fci_v8"))]
        let emergency_byte = 2;
        #[cfg(feature = "fci_v8")]
        let emergency_byte = 3;
        assert!(matches!(
            bincode::deserialize::<StopMoveStatus>(&[emergency_byte]).unwrap(),
            StopMoveStatus::EmergencyAborted
        ));
    }

    #[test]
    fn response_command_is_checked_before_interpreting_status() {
        let mut frame = (Command::Move as u32).to_le_bytes().to_vec();
        frame.extend_from_slice(&1_u32.to_le_bytes());
        frame.extend_from_slice(&13_u32.to_le_bytes());
        frame.push(2); // Preempted is not a StopMove success/rejection.
        assert!(bincode::deserialize::<MoveResponse>(&frame).is_ok());
        assert!(bincode::deserialize::<StopMoveResponse>(&frame).is_err());
    }
}

use robot_behavior::{RobotException, RobotResult};
use serde::{Serialize, de::DeserializeOwned};
use std::{
    collections::VecDeque,
    fmt::Debug,
    io::{Read, Write},
    net::TcpStream,
    time::{Duration, Instant},
};

use crate::types::robot_types::{Command, CommandHeader, CommandIDConfig};

pub struct Network {
    tcp_stream: Option<TcpStream>,
    command_bytes: usize,
    interrupted: bool,
    receive_deadline: Option<Instant>,
    command_counter: u32,
    pending_responses: VecDeque<Vec<u8>>,
}

impl Default for Network {
    fn default() -> Self {
        Self {
            tcp_stream: None,
            command_bytes: 4,
            interrupted: false,
            receive_deadline: None,
            command_counter: 0,
            pending_responses: VecDeque::new(),
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
            interrupted: false,
            receive_deadline: None,
            tcp_stream,
            command_counter: 0,
            pending_responses: VecDeque::new(),
        }
    }

    pub fn tcp_send_and_recv<R, S>(&mut self, request: &mut R) -> RobotResult<S>
    where
        R: Serialize + CommandIDConfig<u32> + Debug,
        S: DeserializeOwned + CommandIDConfig<u32>,
    {
        #[cfg(feature = "debug")]
        println!("tcp send {:?}", request);

        self.ensure_ready()?;
        let command_id = self.next_command_id();
        let Some(stream) = &mut self.tcp_stream else {
            return Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ));
        };

        request.set_command_id(command_id);
        let request = bincode::serialize(request)
            .map_err(|err| RobotException::CommandException(err.to_string()))?;
        stream.write_all(&request)?;

        // A Move terminal response can arrive before StopMove's response.
        // Preserve it for the session owner, and read whole TCP frames rather
        // than assuming one read is one response.
        let frame = receive_response(
            stream,
            &mut self.pending_responses,
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
        self.ensure_ready()?;
        let Some(stream) = &mut self.tcp_stream else {
            return Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ));
        };

        let frame = receive_response(
            stream,
            &mut self.pending_responses,
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

        self.ensure_ready()?;
        let command_id = self.next_command_id();
        let Some(stream) = &mut self.tcp_stream else {
            return Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ));
        };

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

    fn next_command_id(&mut self) -> u32 {
        self.command_counter += 1;
        self.command_counter
    }

    fn ensure_ready(&self) -> RobotResult<()> {
        if self.interrupted {
            return Err(RobotException::NetworkError(
                "TCP operation was interrupted; reconnect before reuse".into(),
            ));
        }
        Ok(())
    }

    pub(crate) fn async_session(&mut self) -> RobotResult<AsyncNetwork<'_>> {
        self.ensure_ready()?;
        tokio::runtime::Handle::try_current()
            .map_err(|error| RobotException::NetworkError(error.to_string()))?;
        let stream = self
            .tcp_stream
            .as_ref()
            .ok_or_else(|| RobotException::NetworkError("No active tcp connection".into()))?;
        let cloned = stream.try_clone()?;
        cloned.set_nonblocking(true)?;
        let stream = match tokio::net::TcpStream::from_std(cloned) {
            Ok(stream) => stream,
            Err(error) => {
                self.tcp_stream.as_ref().unwrap().set_nonblocking(false)?;
                return Err(error.into());
            }
        };
        Ok(AsyncNetwork {
            network: self,
            stream,
            operation_pending: false,
            restored: false,
        })
    }
}

/// Exclusive TCP control-plane lease. No mutex is held across await and the
/// synchronous connection cannot be used while this mutable borrow is alive.
pub(crate) struct AsyncNetwork<'a> {
    network: &'a mut Network,
    stream: tokio::net::TcpStream,
    operation_pending: bool,
    restored: bool,
}

impl AsyncNetwork<'_> {
    pub(crate) fn restore(&mut self) -> RobotResult<()> {
        if self.restored {
            return Ok(());
        }
        let result = self
            .network
            .tcp_stream
            .as_ref()
            .unwrap()
            .set_nonblocking(false);
        self.restored = result.is_ok();
        if result.is_err() {
            self.network.interrupted = true;
        }
        result.map_err(Into::into)
    }

    pub(crate) fn next_command_id(&mut self) -> u32 {
        self.network.next_command_id()
    }

    pub(crate) async fn send_and_recv<R, S>(&mut self, request: &R) -> RobotResult<S>
    where
        R: Serialize + CommandIDConfig<u32>,
        S: DeserializeOwned,
    {
        use tokio::io::AsyncWriteExt;
        self.operation_pending = true;
        let data = bincode::serialize(request)
            .map_err(|error| RobotException::CommandException(error.to_string()))?;
        // SO_SNDTIMEO does not bound Tokio's nonblocking write. Preserve the
        // configured synchronous write limit explicitly in the async path.
        let timeout = self.network.tcp_stream.as_ref().unwrap().write_timeout()?;
        if let Some(timeout) = timeout {
            tokio::time::timeout(timeout, self.stream.write_all(&data))
                .await
                .map_err(|_| {
                    RobotException::NetworkError("TCP request write timed out".into())
                })??;
        } else {
            self.stream.write_all(&data).await?;
        }
        self.receive(request.command_id()).await
    }

    pub(crate) async fn receive<S: DeserializeOwned>(&mut self, command_id: u32) -> RobotResult<S> {
        self.operation_pending = true;
        let frame = if let Some(index) = self
            .network
            .pending_responses
            .iter()
            .position(|frame| response_id(frame, self.network.command_bytes) == command_id)
        {
            self.network.pending_responses.remove(index).unwrap()
        } else {
            loop {
                let frame = self.read_frame().await?;
                if response_id(&frame, self.network.command_bytes) == command_id {
                    break frame;
                }
                self.network.pending_responses.push_back(frame);
            }
        };
        // Framing is complete even if typed decoding rejects this response.
        self.operation_pending = false;
        bincode::deserialize(&frame)
            .map_err(|error| RobotException::DeserializeError(error.to_string()))
    }

    async fn read_frame(&mut self) -> RobotResult<Vec<u8>> {
        use tokio::io::AsyncReadExt;
        let header_size = self.network.command_bytes + 8;
        let mut header = [0; 12];
        self.stream.read_exact(&mut header[..header_size]).await?;
        let size = frame_size(&header, self.network.command_bytes)?;
        let mut frame = vec![0; size];
        frame[..header_size].copy_from_slice(&header[..header_size]);
        self.stream.read_exact(&mut frame[header_size..]).await?;
        Ok(frame)
    }
}

impl Drop for AsyncNetwork<'_> {
    fn drop(&mut self) {
        // Dropping a future is not a session stop. A partially consumed TCP
        // frame cannot safely be interpreted by the next synchronous request.
        self.network.interrupted |= self.operation_pending;
        let _ = self.restore();
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
    let mut header = [0_u8; 12];
    read_exact_until(stream, &mut header[..header_size], deadline)?;
    let size = frame_size(&header, command_bytes)?;
    let mut frame = vec![0_u8; size];
    frame[..header_size].copy_from_slice(&header[..header_size]);
    read_exact_until(stream, &mut frame[header_size..], deadline)?;
    Ok(frame)
}

fn frame_size(header: &[u8; 12], command_bytes: usize) -> RobotResult<usize> {
    let header_size = command_bytes + 8;
    let size =
        u32::from_le_bytes(header[command_bytes + 4..header_size].try_into().unwrap()) as usize;
    if !(header_size..=1024 * 1024).contains(&size) {
        return Err(RobotException::DeserializeError(format!(
            "invalid TCP response size: {size}"
        )));
    }
    Ok(size)
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

    #[cfg(unix)]
    fn assert_tcp_mode(stream: &TcpStream, nonblocking: bool) {
        use std::os::fd::AsRawFd;
        // Read flags from a live owned descriptor, without mutating it.
        let flags = unsafe { libc::fcntl(stream.as_raw_fd(), libc::F_GETFL) };
        assert_ne!(flags, -1);
        assert_eq!(flags & libc::O_NONBLOCK != 0, nonblocking);
    }

    #[cfg(unix)]
    #[test]
    fn async_tcp_lease_restores_mode_and_socket_timeouts() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
        let (_peer, _) = listener.accept().unwrap();
        network
            .tcp_stream
            .as_ref()
            .unwrap()
            .set_read_timeout(Some(Duration::from_millis(17)))
            .unwrap();
        let before_read = network.tcp_stream.as_ref().unwrap().read_timeout().unwrap();
        let before_write = network
            .tcp_stream
            .as_ref()
            .unwrap()
            .write_timeout()
            .unwrap();
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        runtime.block_on(async {
            for explicit_restore in [false, true] {
                let mut lease = network.async_session().unwrap();
                assert_tcp_mode(lease.network.tcp_stream.as_ref().unwrap(), true);
                if explicit_restore {
                    lease.restore().unwrap();
                }
                drop(lease);
                let stream = network.tcp_stream.as_ref().unwrap();
                assert_tcp_mode(stream, false);
                assert_eq!(stream.read_timeout().unwrap(), before_read);
                assert_eq!(stream.write_timeout().unwrap(), before_write);
            }
        });
    }

    #[test]
    fn async_write_preserves_the_configured_deadline() {
        #[derive(Serialize)]
        struct LargeRequest {
            id: u32,
            payload: Vec<u8>,
        }
        impl CommandIDConfig<u32> for LargeRequest {
            fn command_id(&self) -> u32 {
                self.id
            }
            fn set_command_id(&mut self, id: u32) {
                self.id = id;
            }
        }
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let mut network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
        let (_peer, _) = listener.accept().unwrap(); // Intentionally never reads.
        let request = LargeRequest { id: 1, payload: vec![0; 16 * 1024 * 1024] };
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let start = Instant::now();
        runtime.block_on(async {
            let mut lease = network.async_session().unwrap();
            let result: RobotResult<MoveResponse> = lease.send_and_recv(&request).await;
            assert!(matches!(result, Err(error) if error.to_string().contains("write timed out")));
        });
        assert!(start.elapsed() < Duration::from_secs(2));
        assert!(network.interrupted);
        #[cfg(unix)]
        assert_tcp_mode(network.tcp_stream.as_ref().unwrap(), false);
    }
}

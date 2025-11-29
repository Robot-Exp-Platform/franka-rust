use robot_behavior::{RobotException, RobotResult};
use robot_behavior::{is_hardware_realtime, set_realtime_priority};
use serde::{Serialize, de::DeserializeOwned};
use std::{
    fmt::{Debug, Display},
    io::{Read, Write},
    net::TcpStream,
    sync::{Arc, Mutex, RwLock},
};
use std::{net::UdpSocket, thread};

use crate::command_handle::CommandHandle;
use crate::types::robot_types::{CommandFilter, CommandIDConfig};

#[derive(Default)]
pub struct Network {
    tcp_stream: Option<TcpStream>,
    command_counter: Arc<Mutex<u32>>,
}

impl Network {
    pub fn new(tcp_ip: &str, tcp_port: u16) -> Self {
        let tcp_stream = TcpStream::connect(format!("{tcp_ip}:{tcp_port}")).ok();

        if let Some(steam) = &tcp_stream {
            // steam
            //     .set_read_timeout(Some(std::time::Duration::from_millis(10)))
            //     .unwrap();
            steam
                .set_write_timeout(Some(std::time::Duration::from_millis(3)))
                .unwrap();
        }
        Network { tcp_stream, command_counter: Arc::new(Mutex::new(0)) }
    }

    /// 发送并阻塞接收 tcp 指令
    pub fn tcp_send_and_recv<R, S>(&mut self, request: &mut R) -> RobotResult<S>
    where
        R: Serialize + CommandIDConfig<u32> + Debug,
        S: DeserializeOwned + CommandIDConfig<u32>,
    {
        #[cfg(feature = "debug")]
        println!("tcp send {:?}", request);
        if let Some(stream) = &mut self.tcp_stream {
            let command_id = {
                let mut counter = self.command_counter.lock().unwrap();
                *counter += 1;
                *counter
            };
            request.set_command_id(command_id);
            let request = bincode::serialize(&request).unwrap();
            stream.write_all(&request)?;
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

    pub fn tcp_blocking_recv<S>(&mut self) -> RobotResult<S>
    where
        S: DeserializeOwned + CommandIDConfig<u32> + Debug,
    {
        if let Some(stream) = &mut self.tcp_stream {
            let mut buffer = vec![0_u8; size_of::<S>() + 4];
            stream.read_exact(&mut buffer)?;

            let response = bincode::deserialize(&buffer);
            response.map_err(|e| RobotException::DeserializeError(e.to_string()))
        } else {
            Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ))
        }
    }

    pub fn tcp_send_and_recv_buffer<R, S>(&mut self, request: &mut R) -> RobotResult<(S, Vec<u8>)>
    where
        R: Serialize + CommandIDConfig<u32> + Debug,
        S: DeserializeOwned + CommandIDConfig<u32>,
    {
        #[cfg(feature = "debug")]
        println!("tcp send {:?}", request);
        if let Some(stream) = &mut self.tcp_stream {
            let command_id = {
                let mut counter = self.command_counter.lock().unwrap();
                *counter += 1;
                *counter
            };
            request.set_command_id(command_id);
            let request = bincode::serialize(&request).unwrap();
            #[cfg(feature = "debug")]
            println!("request :{:?}", request);
            stream.write_all(&request)?;
            let mut buffer = vec![0_u8; size_of::<S>() + 4];
            stream.read_exact(&mut buffer)?;
            let res = bincode::deserialize(&buffer)
                .map_err(|e| RobotException::DeserializeError(e.to_string()))?;
            let mut receive_buffer = Vec::new();
            loop {
                let mut buffer = vec![0_u8; 1024 * 5];
                if let Ok(size) = stream.read(&mut buffer) {
                    receive_buffer.append(&mut buffer[..size].to_vec());
                    println!("size:{size}");
                } else {
                    break;
                }
            }
            println!("receive size:{}", receive_buffer.len());
            Ok((res, receive_buffer))
        } else {
            Err(RobotException::NetworkError(
                "No active tcp connection".to_string(),
            ))
        }
    }

    pub fn spawn_udp_thread<R, S>(port: u16) -> (CommandHandle<R, S>, Arc<RwLock<S>>)
    where
        R: Serialize
            + CommandIDConfig<u64>
            + CommandFilter<S>
            + Clone
            + Display
            + Send
            + Sync
            + 'static,
        S: DeserializeOwned
            + CommandIDConfig<u64>
            + Clone
            + Display
            + Default
            + Send
            + Sync
            + 'static,
    {
        use std::time::Duration;

        #[cfg(target_os = "windows")]
        {
            if !is_firewall_rule_active(port) {
                let (title, fix_step, cleanup_info) = get_localized_message(port);
                eprintln!("\n{title}\n\n{fix_step}\n\n{cleanup_info}\n");
                std::process::exit(1);
            }
        }

        let cmd = CommandHandle::<R, S>::new();
        let res = Arc::new(RwLock::new(S::default()));
        let cmd_handle = cmd.clone();
        let res_handle = res.clone();

        thread::spawn(move || {
            if is_hardware_realtime() {
                println!("you have realtime permission, enjoy it");
                set_realtime_priority().unwrap();
            } else {
                println!(
                    "you don't have realtime permission, which may cause communication latency"
                );
                let _ = thread_priority::set_current_thread_priority(
                    thread_priority::ThreadPriority::Max,
                );
            }

            #[cfg(feature = "debug")]
            let start_time = std::time::Instant::now();

            let udp_socket = UdpSocket::bind(format!("{}:{}", "0.0.0.0", port)).unwrap();
            // udp_socket
            //     .set_read_timeout(Some(Duration::from_micros(1200)))
            //     .unwrap();
            let mut buffer = vec![0u8; size_of::<S>() * 5];
            loop {
                let (size, addr) = match udp_socket.recv_from(&mut buffer) {
                    Ok(res) => res,
                    Err(_) => {
                        // 超时继续循环
                        continue;
                    }
                };

                let response: S = match bincode::deserialize(&buffer[..size]) {
                    Ok(resp) => resp,
                    Err(err) => {
                        #[cfg(feature = "debug")]
                        eprintln!("bincode deserialize error: {:?}", err);
                        // 尝试按 JSON 格式解析并输出，便于调试非 bincode 的数据
                        if let Ok(json_val) =
                            serde_json::from_slice::<serde_json::Value>(&buffer[..size])
                        {
                            eprintln!("Received non-bincode message (as JSON): {}", json_val);
                        } else {
                            eprintln!(
                                "Received non-bincode message raw bytes: {:?}",
                                &buffer[..size]
                            );
                        }
                        continue;
                    }
                };
                #[cfg(feature = "debug")]
                println!("{:?} >{}", start_time.elapsed(), response);

                if let Some(data) = &mut cmd.command(&response, Duration::from_millis(1)) {
                    data.set_command_id(response.command_id());
                    #[cfg(feature = "debug")]
                    println!("{:?} >{}", start_time.elapsed(), data);
                    let data = bincode::serialize(&data).unwrap();
                    let send_size = udp_socket.send_to(&data, addr).unwrap();
                    if send_size != size_of::<R>() {
                        eprintln!("udp send error");
                    }
                }

                *res.write().unwrap() = response;
            }
        });
        (cmd_handle, res_handle)
    }
}

#[cfg(target_os = "windows")]
fn is_firewall_rule_active(port: u16) -> bool {
    use std::process::Command;

    let command = format!(
        "Get-NetFirewallRule -DisplayName 'LibFranka_UDP_{port}' -ErrorAction SilentlyContinue | Where-Object {{ $_.Enabled -eq 'True' }}",
    );

    let output = Command::new("powershell")
        .args(["-Command", &command])
        .output();

    match output {
        Ok(o) => o.status.success(),
        Err(_) => false,
    }
}

#[cfg(target_os = "windows")]
fn get_localized_message(port: u16) -> (String, String, String) {
    use sys_locale::get_locale;

    let lang = get_locale().unwrap_or_default();
    if lang.starts_with("zh") {
        // 中文提示
        let title = "⚠️ 网络配置要求".to_string();
        let fix_step = format!(
            r"
1. 右键开始菜单 → 选择 [Windows PowerShell (管理员)]
2. 执行以下命令 (已复制到剪贴板):
   New-NetFirewallRule -DisplayName 'LibFranka_UDP_{port}' -Direction Inbound -Protocol UDP -LocalPort {port} -Action Allow
3. 按回车 → 完成后关闭窗口
4. 重新启动本程序
            "
        );
        let cleanup_info =
            format!("后续清理: 执行命令 Remove-NetFirewallRule -DisplayName 'RoboLib_UDP_{port}'");
        (title, fix_step, cleanup_info)
    } else {
        // 英文提示 (默认)
        let title = "⚠️ Network Configuration Required".to_string();
        let fix_step = format!(
            r"
1. Right-click Start Menu → Select [Windows PowerShell (Admin)]
2. Run this command (copied to clipboard):
   New-NetFirewallRule -DisplayName 'LibFranka_UDP_{port}' -Direction Inbound -Protocol UDP -LocalPort {port} -Action Allow
3. Press Enter → Close window after success
4. Restart this application
            ",
        );
        let cleanup_info =
            format!("Cleanup later: Run Remove-NetFirewallRule -DisplayName 'RoboLib_UDP_{port}'");
        (title, fix_step, cleanup_info)
    }
}

use robot_behavior::{RobotException, RobotResult};
#[cfg(not(feature = "async"))]
use robot_behavior::{is_hardware_realtime, set_realtime_priority};
use serde::{Serialize, de::DeserializeOwned};
use std::{
    fmt::{Debug, Display},
    io::{Read, Write},
    net::TcpStream,
    sync::{Arc, Mutex, RwLock},
};
#[cfg(not(feature = "async"))]
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
        let tcp_stream = TcpStream::connect(format!("{}:{}", tcp_ip, tcp_port)).ok();

        if let Some(steam) = &tcp_stream {
            steam
                .set_read_timeout(Some(std::time::Duration::from_millis(10)))
                .unwrap();
            steam
                .set_write_timeout(Some(std::time::Duration::from_millis(3)))
                .unwrap();
        }
        Network {
            tcp_stream,
            command_counter: Arc::new(Mutex::new(0)),
        }
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
            stream.read(&mut buffer)?;
            let res = bincode::deserialize(&buffer)
                .map_err(|e| RobotException::DeserializeError(e.to_string()))?;
            let mut receive_buffer = Vec::new();
            loop {
                let mut buffer = vec![0_u8; 1024 * 5];
                if let Ok(size) = stream.read(&mut buffer) {
                    receive_buffer.append(&mut buffer[..size].to_vec());
                    println!("size:{}", size);
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

    // /// 检查是否有可接收的 tcp 数据,如果有，则解析为对应的类型，如果没有，直接退出
    // pub fn tcp_recv<S>(&mut self) -> RobotResult<Option<S>>
    // where
    //     S: DeserializeOwned + CommandIDConfig,
    // {
    //     if let Some(stream) = &mut self.tcp_stream {
    //         let mut buffer = [0; 1024];
    //         let size = stream.read(&mut buffer)?;
    //         if size == 0 {
    //             return Ok(None);
    //         }
    //         bincode::deserialize(&buffer[..size])
    //             .map(|data| Some(data))
    //             .map_err(|e| RobotException::DeserializeError(e.to_string()))
    //     } else {
    //         Err(RobotException::NetworkError(
    //             "No active tcp connection".to_string(),
    //         ))
    //     }
    // }

    #[cfg(not(feature = "async"))]
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
                eprintln!("\n{}\n\n{}\n\n{}\n", title, fix_step, cleanup_info);
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
            let mut duration = Duration::from_millis(0);

            let udp_socket = UdpSocket::bind(format!("{}:{}", "0.0.0.0", port)).unwrap();
            // udp_socket
            //     .set_read_timeout(Some(Duration::from_micros(1200)))
            //     .unwrap();
            let mut buffer = vec![0u8; size_of::<S>() * 5];
            loop {
                let (size, addr) = udp_socket.recv_from(&mut buffer).unwrap();

                let response: S = bincode::deserialize(&buffer[..size]).unwrap();
                #[cfg(feature = "debug")]
                println!("{:?} >{}", start_time.elapsed(), response);

                if let Some(data) = &mut cmd.command(&response, duration) {
                    data.set_command_id(response.command_id());
                    #[cfg(feature = "debug")]
                    println!("{:?} >{}", start_time.elapsed(), data);
                    let data = bincode::serialize(&data).unwrap();
                    let send_size = udp_socket.send_to(&data, addr).unwrap();
                    if send_size != size_of::<R>() {
                        eprintln!("udp send error");
                    }
                    duration += Duration::from_millis(1);
                } else {
                    duration = Duration::from_millis(0);
                }

                *res.write().unwrap() = response;
            }
        });
        (cmd_handle, res_handle)
    }

    #[cfg(feature = "async")]
    pub fn spawn_udp_thread<R, S>(port: u16) -> (Arc<ArrayQueue<R>>, Arc<RwLock<S>>)
    where
        R: Serialize + CommandIDConfig<u64> + Send + Sync + std::fmt::Debug + 'static,
        S: DeserializeOwned + CommandIDConfig<u64> + Default + Send + Sync + 'static,
    {
        #[cfg(target_os = "windows")]
        {
            if !is_firewall_rule_active(port) {
                let (title, fix_step, cleanup_info) = get_localized_message(port);
                eprintln!("\n{}\n\n{}\n\n{}\n", title, fix_step, cleanup_info);
                std::process::exit(1);
            }
        }
        // 创建共享队列
        let (request_queue, response_queue) = (
            Arc::new(ArrayQueue::<R>::new(1)),
            Arc::new(RwLock::new(S::default())),
        );

        // 克隆用于闭包捕获
        let req_queue = request_queue.clone();
        let res_queue = response_queue.clone();

        // 启动异步任务
        tokio::spawn(async move {
            // 绑定 UDP 端口
            let socket = match UdpSocket::bind(format!("0.0.0.0:{}", port)) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[UDP] Bind failed: {}", e);
                    return;
                }
            };

            let mut buffer = vec![0u8; size_of::<S>()];
            let start_time = std::time::Instant::now();

            loop {
                // 异步接收数据
                let (size, addr) = match socket.recv_from(&mut buffer) {
                    Ok(res) => res,
                    Err(e) => {
                        eprintln!("[UDP] Receive error: {}", e);
                        continue;
                    }
                };

                // 反序列化处理
                let response = match bincode::deserialize::<S>(&buffer[..size]) {
                    Ok(v) => v,
                    Err(e) => {
                        eprintln!("[Deserialize] Error: {}", e);
                        continue;
                    }
                };

                // 处理请求队列
                if let Some(mut request) = req_queue.pop() {
                    request.set_command_id(response.command_id());
                    let serialized = match bincode::serialize(&request) {
                        Ok(v) => v,
                        Err(e) => {
                            eprintln!("[Serialize] Error: {}", e);
                            return;
                        }
                    };

                    match socket.send_to(&serialized, addr) {
                        Ok(_) => {
                            println!("{:?} > UDP response sent", start_time.elapsed());
                        }
                        Err(e) => {
                            eprintln!("[UDP] Send error: {}", e);
                        }
                    }
                }

                // 更新响应状态
                *res_queue.write().unwrap() = response;
            }
        });

        (request_queue, response_queue)
    }
}

#[cfg(target_os = "windows")]
fn is_firewall_rule_active(port: u16) -> bool {
    use std::process::Command;

    let command = format!(
        "Get-NetFirewallRule -DisplayName 'LibFranka_UDP_{}' -ErrorAction SilentlyContinue | Where-Object {{ $_.Enabled -eq 'True' }}",
        port
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
            r#"
1. 右键开始菜单 → 选择 [Windows PowerShell (管理员)]
2. 执行以下命令 (已复制到剪贴板):
   New-NetFirewallRule -DisplayName 'LibFranka_UDP_{}' -Direction Inbound -Protocol UDP -LocalPort {} -Action Allow
3. 按回车 → 完成后关闭窗口
4. 重新启动本程序
            "#,
            port, port
        );
        let cleanup_info = format!(
            "后续清理: 执行命令 Remove-NetFirewallRule -DisplayName 'RoboLib_UDP_{}'",
            port
        );
        (title, fix_step, cleanup_info)
    } else {
        // 英文提示 (默认)
        let title = "⚠️ Network Configuration Required".to_string();
        let fix_step = format!(
            r#"
1. Right-click Start Menu → Select [Windows PowerShell (Admin)]
2. Run this command (copied to clipboard):
   New-NetFirewallRule -DisplayName 'LibFranka_UDP_{}' -Direction Inbound -Protocol UDP -LocalPort {} -Action Allow
3. Press Enter → Close window after success
4. Restart this application
            "#,
            port, port
        );
        let cleanup_info = format!(
            "Cleanup later: Run Remove-NetFirewallRule -DisplayName 'RoboLib_UDP_{}'",
            port
        );
        (title, fix_step, cleanup_info)
    }
}

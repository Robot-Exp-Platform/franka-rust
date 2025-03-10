use robot_behavior::{RobotException, RobotResult};
use serde::{Serialize, de::DeserializeOwned};
use std::{
    io::{Read, Write},
    net::{TcpStream, UdpSocket},
    process::Command,
    sync::{Arc, Mutex},
};

use crate::types::robot_types::CommandIDConfig;

#[derive(Default)]
pub struct Network {
    tcp_stream: Option<TcpStream>,
    udp_socket: Option<UdpSocket>,
    command_counter: Arc<Mutex<u32>>,
}

impl Network {
    pub fn new(tcp_ip: &str, tcp_port: u16, udp_port: u16) -> Self {
        #[cfg(target_os = "windows")]
        {
            if !is_firewall_rule_active(udp_port) {
                let (title, fix_step, cleanup_info) = get_localized_message(udp_port);
                eprintln!("\n{}\n\n{}\n\n{}\n", title, fix_step, cleanup_info);
                std::process::exit(1);
            }
        }

        let tcp_stream = TcpStream::connect(format!("{}:{}", tcp_ip, tcp_port)).ok();
        let udp_socket = UdpSocket::bind(format!("{}:{}", "0.0.0.0", udp_port)).ok();

        Network {
            tcp_stream,
            udp_socket,
            command_counter: Arc::new(Mutex::new(0)),
        }
    }

    /// 发送并阻塞接收 tcp 指令
    pub fn tcp_send_and_recv<R, S>(&mut self, request: &mut R) -> RobotResult<S>
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
    pub fn tcp_recv<S>(&mut self) -> RobotResult<Option<S>>
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
    pub fn udp_send<R>(&mut self, request: R) -> RobotResult<()>
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
    pub fn udp_recv<S>(&mut self) -> RobotResult<S>
    where
        S: DeserializeOwned,
    {
        if let Some(socket) = &mut self.udp_socket {
            let mut buffer = [0; 1024];
            let (size, _) = socket.recv_from(&mut buffer)?;
            println!("udp recv data: {:?}", &buffer[..size]);
            bincode::deserialize(&buffer[..size])
                .map_err(|e| RobotException::DeserializeError(e.to_string()))
        } else {
            Err(RobotException::NetworkError(
                "No active udp connection".to_string(),
            ))
        }
    }
}

#[cfg(target_os = "windows")]
fn is_firewall_rule_active(port: u16) -> bool {
    let command = format!(
        "Get-NetFirewallRule -DisplayName 'LibFranka_UDP_{}' -ErrorAction SilentlyContinue | Where-Object {{ $_.Enabled -eq 'True' }}",
        port
    );

    let output = Command::new("powershell")
        .args(&["-Command", &command])
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

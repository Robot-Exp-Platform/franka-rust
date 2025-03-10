use std::io::{Read, Write};
use std::net::{TcpStream, UdpSocket};

fn main() {
    let udp_socket = UdpSocket::bind("0.0.0.0:0").unwrap();

    let udp_port = udp_socket.local_addr().unwrap().port();

    println!("UDP port: {}", udp_port);

    let mut tcp_stream = TcpStream::connect("172.16.0.3:1337").unwrap();

    let udp_port_bytes = &udp_port.to_be_bytes();

    assert_eq!(
        udp_port,
        udp_port_bytes[0] as u16 * 256 + udp_port_bytes[1] as u16
    );

    println!("UDP port bytes: {:?}", udp_port_bytes);

    let connect_request = [
        0,
        0,
        0,
        0,
        0,
        0,
        14,
        0,
        0,
        0,
        3,
        0,
        udp_port_bytes[1],
        udp_port_bytes[0],
    ];

    tcp_stream.write_all(&connect_request).unwrap();

    println!("Sent connect request");

    let mut buffer = [0; 1024];
    let size = udp_socket.recv(&mut buffer).unwrap();
    println!("Received: {:?}", &buffer[..size]);
}

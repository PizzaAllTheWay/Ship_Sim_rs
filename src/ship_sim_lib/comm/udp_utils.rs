use socket2::{Socket, Domain, Type, Protocol};
use std::net::{UdpSocket, SocketAddr, Ipv4Addr};
use std::io;
use serde::{Deserialize, Serialize};

/// Multicast group address (must be in 224.0.0.0 to 239.255.255.255)
const MULTICAST_ADDR: &str = "239.0.0.1";

/// Sends a byte message to a multicast group
pub fn publish(port: u16, msg: &[u8]) -> io::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:0")?; // ephemeral port
    socket.set_multicast_loop_v4(true)?;
    let target = format!("{}:{}", MULTICAST_ADDR, port);
    socket.send_to(msg, target)?;
    Ok(())
}

/// Joins a multicast group and receives a single message
pub fn subscribe(port: u16) -> io::Result<Vec<u8>> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;

    socket.set_reuse_address(true)?;
    #[cfg(target_family = "unix")]
    socket.set_reuse_port(true)?;

    let addr = SocketAddr::from(([0, 0, 0, 0], port));
    socket.bind(&addr.into())?;

    let std_socket: UdpSocket = socket.into();

    // Join multicast group
    let multi_ip = MULTICAST_ADDR.parse::<Ipv4Addr>().unwrap();
    std_socket.join_multicast_v4(&multi_ip, &Ipv4Addr::UNSPECIFIED)?;

    let mut buf = [0u8; 1024];
    let (amt, _src) = std_socket.recv_from(&mut buf)?;
    Ok(buf[..amt].to_vec())
}

// Encode/Decode incoming JSON
pub fn encode_json<T: Serialize>(data: &T) -> String {
    serde_json::to_string(data).expect("Failed to serialize")
}

pub fn decode_json<'a, T>(json: &'a str) -> T
where
    T: for<'de> Deserialize<'de>,
{
    serde_json::from_str(json).expect("Failed to deserialize JSON into struct")
}

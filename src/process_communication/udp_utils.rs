use std::net::UdpSocket;
use std::io;

/// Sends a byte message to a specified broadcast port
pub fn publish(port: u16, msg: &[u8]) -> io::Result<()> {
    let socket = UdpSocket::bind("127.0.0.1:0")?; // use ephemeral port
    socket.set_broadcast(true)?;
    let broadcast_addr = format!("0.0.0.0:{}", port);
    socket.send_to(msg, &broadcast_addr)?;
    Ok(())
}

/// Binds to the port and waits to receive a single message (blocking)
pub fn subscribe(port: u16) -> io::Result<Vec<u8>> {
    let socket = UdpSocket::bind(("127.0.0.1", port))?;
    let mut buf = [0u8; 1024];
    let (amt, _src) = socket.recv_from(&mut buf)?;
    Ok(buf[..amt].to_vec())
}
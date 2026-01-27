pub fn tun_alloc(devname: &str) -> Result<i32, ()>
{
    todo!()
}

pub fn set_ip(devname: &str, ip_address: std::net::Ipv4Addr, netmask: std::net::Ipv4Addr) -> Result<(), ()>
{
    todo!()
}

pub fn read_from_tun(fd: i32)
{
    todo!()
}

unsafe extern "C" {
}

static TUN_MTU: usize = 1500;

pub fn tun_alloc(devname: &str) -> Result<i32, ()>
{
    let ifreq_uninit: std::mem::MaybeUninit<libc::ifreq> = std::mem::MaybeUninit::zeroed();
    let mut ifreq = unsafe { ifreq_uninit.assume_init() };
    let fd = unsafe { libc::open(std::ffi::CString::new("/dev/net/tun").unwrap().as_ptr(), libc::O_RDWR) };
    if fd < 0 {
        return Err(());
    }
    ifreq.ifr_ifru.ifru_flags = libc::IFF_TUN as i16;
    unsafe { libc::strncpy(std::ptr::addr_of_mut!(ifreq.ifr_name[0]), std::ffi::CString::new(devname).unwrap().as_ptr(), libc::IFNAMSIZ) };
    let err = unsafe { libc::ioctl(fd, libc::TUNSETIFF, std::ptr::addr_of!(ifreq)) };
    if err < 0 {
        println!("1");
        unsafe { libc::perror(std::ffi::CString::new("ioctl()").unwrap().as_ptr()) };
        return Err(());
    }

    Ok(fd)
}

pub fn set_ip(devname: &str, ip_address: std::net::Ipv4Addr, peer: std::net::Ipv4Addr, netmask: std::net::Ipv4Addr) -> Result<(), ()>
{
    let ifreq_uninit: std::mem::MaybeUninit<libc::ifreq> = std::mem::MaybeUninit::zeroed();
    let mut ifreq = unsafe { ifreq_uninit.assume_init() };
    let mut err;
    let fd = unsafe { libc::socket(libc::PF_INET, libc::SOCK_STREAM, libc::IPPROTO_IP) };
    if fd < 0 {
        return Err(());
    }

    unsafe { libc::strncpy(std::ptr::addr_of_mut!(ifreq.ifr_name[0]), std::ffi::CString::new(devname).unwrap().as_ptr(), libc::IFNAMSIZ) };

    // IP address
    ifreq.ifr_ifru.ifru_addr.sa_family = libc::AF_INET as u16;
    let addr = std::ptr::addr_of_mut!(ifreq.ifr_ifru.ifru_addr) as *mut libc::sockaddr_in;
    let addr_ptr = unsafe { &mut *addr };
    let sin_addr = &mut addr_ptr.sin_addr;
    let sin_addr_ptr = std::ptr::addr_of_mut!(*sin_addr);
    let ip_address_octets = ip_address.as_octets();
    unsafe { libc::memcpy(sin_addr_ptr as *mut libc::c_void, std::ptr::addr_of!(ip_address_octets[0]) as *const libc::c_void, 4) };
    err = unsafe { libc::ioctl(fd, libc::SIOCSIFADDR, std::ptr::addr_of!(ifreq)) };
    if err < 0 {
        println!("2");
        unsafe { libc::perror(std::ffi::CString::new("ioctl()").unwrap().as_ptr()) };
        return Err(());
    }

    // Peer address
    let peer_octets = peer.as_octets();
    unsafe { libc::memcpy(sin_addr_ptr as *mut libc::c_void, std::ptr::addr_of!(peer_octets[0]) as *const libc::c_void, 4) };
    err = unsafe { libc::ioctl(fd, libc::SIOCSIFDSTADDR, std::ptr::addr_of!(ifreq)) };
    if err < 0 {
        println!("3");
        unsafe { libc::perror(std::ffi::CString::new("ioctl()").unwrap().as_ptr()) };
        return Err(());
    }

    // Netmask
    let netmask_octets = netmask.as_octets();
    unsafe { libc::memcpy(sin_addr_ptr as *mut libc::c_void, std::ptr::addr_of!(netmask_octets[0]) as *const libc::c_void, 4) };
    err = unsafe { libc::ioctl(fd, libc::SIOCSIFNETMASK, std::ptr::addr_of!(ifreq)) };
    if err < 0 {
        println!("4");
        unsafe { libc::perror(std::ffi::CString::new("ioctl()").unwrap().as_ptr()) };
        return Err(());
    }

    // Flags
    unsafe { libc::ioctl(fd, libc::SIOCGIFFLAGS, std::ptr::addr_of_mut!(ifreq)) };
    unsafe { libc::strncpy(std::ptr::addr_of_mut!(ifreq.ifr_name[0]), std::ffi::CString::new(devname).unwrap().as_ptr(), libc::IFNAMSIZ) };
    unsafe { ifreq.ifr_ifru.ifru_flags |= libc::IFF_UP as i16 | libc::IFF_RUNNING as i16 | libc::IFF_POINTOPOINT as i16 };
    unsafe { libc::ioctl(fd, libc::SIOCSIFFLAGS, std::ptr::addr_of_mut!(ifreq)) };

    Ok(())
}

pub fn read_from_tun(fd: i32) -> Vec<u8>
{
    let mut buf = [0u8; TUN_MTU];
    let nbytes = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
    buf[..nbytes as usize].to_vec()
}

pub fn write_to_tun(fd: i32, data: &Vec<u8>)
{
    unsafe { libc::write(fd, data.as_ptr() as *const libc::c_void, data.len()) };
}

unsafe extern "C" {
}

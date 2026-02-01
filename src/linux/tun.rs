static TUN_MTU: usize = 1500;

pub struct TunDevice {
    fd: i32,
    devname: String,
}

impl TunDevice {
    /// Allocates a new TUN device according to devname
    /// This function tries to allocate the device of type TUN (ip packets)
    /// On success, the file descriptor of the /dev/net/tun file is returned
    /// Note that function needs root privileges or CAP_NET_ADMIN to run successfully
    /// 
    /// # Errors
    /// It may fail if /dev/net/tun cannot be opened read write, or if the call to ioctl() fails
    /// 
    /// # Examples
    /// ```
    /// let tun_res = TunDevice::new("tun0");
    /// if let Ok(tun) = tun_res {
    ///     // Do something with fd
    /// }
    /// ```
    pub fn new(devname: String) -> Result<TunDevice, ()>
    {
        // Open /dev/net/tun
        let ifreq_uninit: std::mem::MaybeUninit<libc::ifreq> = std::mem::MaybeUninit::zeroed();
        let mut ifreq = unsafe { ifreq_uninit.assume_init() };
        let fd = unsafe { libc::open(std::ffi::CString::new("/dev/net/tun").unwrap().as_ptr(), libc::O_RDWR) };
        if fd < 0 {
            return Err(());
        }

        // Request to create a tun device with name devname
        ifreq.ifr_ifru.ifru_flags = libc::IFF_TUN as i16;
        unsafe { libc::strncpy(std::ptr::addr_of_mut!(ifreq.ifr_name[0]), std::ffi::CString::new(devname.as_str()).unwrap().as_ptr(), libc::IFNAMSIZ) };
        let err = unsafe { libc::ioctl(fd, libc::TUNSETIFF, std::ptr::addr_of!(ifreq)) };
        if err < 0 {
            unsafe { libc::perror(std::ffi::CString::new("ioctl()").unwrap().as_ptr()) };
            return Err(());
        }

        Ok(TunDevice { fd, devname })
    }

    /// Sets the IP address, peer IP address, netmask and flags of the newly created network interface
    /// 
    /// # Errors
    /// It may fail if any of the calls to socket() or ioctl()) fail
    /// 
    /// # Examples
    /// ```
    /// let tun_device = TunDevice::new("tun0".to_owned()).unwrap();
    /// let res = tun_device.set_ip(IPv4Addr::new(10, 0, 0, 1), IPv4Addr::new(10, 0, 0, 2), IPv4Addr::new(255, 255, 255, 0));
    /// if res.is_err() {
    ///     // Handle error
    /// }
    /// ```
    pub fn set_ip(&self, ip_address: std::net::Ipv4Addr, peer: std::net::Ipv4Addr, netmask: std::net::Ipv4Addr) -> Result<(), ()>
    {
        let ifreq_uninit: std::mem::MaybeUninit<libc::ifreq> = std::mem::MaybeUninit::zeroed();
        let mut ifreq = unsafe { ifreq_uninit.assume_init() };
        let mut err;
        let fd = unsafe { libc::socket(libc::PF_INET, libc::SOCK_STREAM, libc::IPPROTO_IP) };
        if fd < 0 {
            return Err(());
        }

        unsafe { libc::strncpy(std::ptr::addr_of_mut!(ifreq.ifr_name[0]), std::ffi::CString::new(self.devname.as_str()).map_err(|_| ())?.as_ptr(), libc::IFNAMSIZ) };

        // Set IP address
        ifreq.ifr_ifru.ifru_addr.sa_family = libc::AF_INET as u16;
        let addr = std::ptr::addr_of_mut!(ifreq.ifr_ifru.ifru_addr) as *mut libc::sockaddr_in;
        let addr_ptr = unsafe { &mut *addr };
        let sin_addr = &mut addr_ptr.sin_addr;
        let sin_addr_ptr = std::ptr::addr_of_mut!(*sin_addr);
        let ip_address_octets = ip_address.as_octets();
        unsafe { libc::memcpy(sin_addr_ptr as *mut libc::c_void, std::ptr::addr_of!(ip_address_octets[0]) as *const libc::c_void, 4) };
        err = unsafe { libc::ioctl(fd, libc::SIOCSIFADDR, std::ptr::addr_of!(ifreq)) };
        if err < 0 {
            unsafe { libc::perror(std::ffi::CString::new("ioctl()").map_err(|_| ())?.as_ptr()) };
            return Err(());
        }

        // Set peer address
        let peer_octets = peer.as_octets();
        unsafe { libc::memcpy(sin_addr_ptr as *mut libc::c_void, std::ptr::addr_of!(peer_octets[0]) as *const libc::c_void, 4) };
        err = unsafe { libc::ioctl(fd, libc::SIOCSIFDSTADDR, std::ptr::addr_of!(ifreq)) };
        if err < 0 {
            unsafe { libc::perror(std::ffi::CString::new("ioctl()").map_err(|_| ())?.as_ptr()) };
            return Err(());
        }

        // Set netmask
        let netmask_octets = netmask.as_octets();
        unsafe { libc::memcpy(sin_addr_ptr as *mut libc::c_void, std::ptr::addr_of!(netmask_octets[0]) as *const libc::c_void, 4) };
        err = unsafe { libc::ioctl(fd, libc::SIOCSIFNETMASK, std::ptr::addr_of!(ifreq)) };
        if err < 0 {
            unsafe { libc::perror(std::ffi::CString::new("ioctl()").map_err(|_| ())?.as_ptr()) };
            return Err(());
        }

        // Set flags
        unsafe { libc::ioctl(fd, libc::SIOCGIFFLAGS, std::ptr::addr_of_mut!(ifreq)) };
        unsafe { libc::strncpy(std::ptr::addr_of_mut!(ifreq.ifr_name[0]), std::ffi::CString::new(self.devname.as_str()).map_err(|_| ())?.as_ptr(), libc::IFNAMSIZ) };
        unsafe { ifreq.ifr_ifru.ifru_flags |= libc::IFF_UP as i16 | libc::IFF_RUNNING as i16 | libc::IFF_POINTOPOINT as i16 };
        unsafe { libc::ioctl(fd, libc::SIOCSIFFLAGS, std::ptr::addr_of_mut!(ifreq)) };

        Ok(())
    }

    /// Reads up to TUN_MTU bytes from the TUN device as a Vec<u8>
    /// 
    /// # Examples
    /// ```
    /// let tun_device = TunDevice::new("tun0".to_owned()).unwrap();
    /// let data = tun_device.read_from_tun();
    /// ```
    pub fn read_from_tun(&self) -> Vec<u8>
    {
        let mut buf = [0u8; TUN_MTU];
        let nbytes = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        buf[..nbytes as usize].to_vec()
    }

    /// Writes a Vec<u8> to the TUN device
    /// 
    /// # Examples
    /// ```
    /// let tun_device = TunDevice::new("tun0".to_owned()).unwrap();
    /// let data = vec![0, 1, 2, 3];
    /// tun_device.write_to_tun(&data);
    /// ```
    pub fn write_to_tun(&self, data: &Vec<u8>)
    {
        unsafe { libc::write(self.fd, data.as_ptr() as *const libc::c_void, data.len()) };
    }
}

pub fn geteuid() -> u32
{
    unsafe { libc::geteuid() }
}

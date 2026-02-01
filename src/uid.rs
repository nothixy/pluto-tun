/// This function is a safe wrapper for the unsafe funcion geteuid()
/// 
/// It returns the id of the process owner as a u32
/// 
/// # Examples
/// ```
/// let current_id = geteuid();
/// // current_id will usually be 1000 or 0 if root
/// ```
pub fn geteuid() -> u32
{
    unsafe { libc::geteuid() }
}

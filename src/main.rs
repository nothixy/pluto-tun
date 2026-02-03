#![feature(ip_as_octets)]
#![feature(addr_parse_ascii)]
#![feature(iter_advance_by)]
#![feature(never_type)]

pub extern crate adi;

use adi::traits::*;
use num::complex::Complex32;

mod symbol;

#[cfg_attr(target_os = "linux", path = "linux/tun.rs")]
#[cfg_attr(target_os = "macos", path = "macos/tun.rs")]
mod tun;

mod uid;

static TX_HARDWARE_GAIN: f32 = -65.0;
static RX_HARDWARE_GAIN: f32 = 70.0;
static CENTER_FREQUENCY_CLIENT: u64 = 2_400_000_000;
static CENTER_FREQUENCY_SERVER: u64 = 2_483_500_000;
static SAMPLE_RATE: u32 = 1_000_000;
static NUM_SAMPS: usize = 2_000_000;
static OVERSAMPLING: usize = 25;
static NOISE_FLOOR: f32 = 1000f32;
static SAMPLING_MARGIN: usize = OVERSAMPLING / 5;
static BYTES_PER_CONTROL: usize = 3;
static FRAME_SPLIT_LENGTH: usize = 12;
static FRAME_TRANSMIT_REPEAT_COUNT: usize = 2;

/// Sends the bytes passed as an argument to the pluto device passed as argument
/// 
/// # Errors
/// This function may fail if the pluto tx function fails
/// 
/// # Panics
/// This function may panic if the mutex holding the pluto device was poisoned
/// 
/// # Examples
/// ```
/// let pluto = Arc::new(Mutex::new(Pluto::new(Some("ip:192.168.2.1".to_owned()))).unwrap();
/// let message = vec![1, 2, 3, 4];
/// let res = function_tx(&message, &pluto);
/// if res.is_err() {
///     // Handle error
/// }
/// ```
fn function_tx(message: &Vec<u8>, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<(), ()>
{
    // Compute size of TX buffer and allocate it once
    let size = (2 + 4 * BYTES_PER_CONTROL) * OVERSAMPLING * ((message.len() + BYTES_PER_CONTROL - 1) / BYTES_PER_CONTROL);
    let mut final_buffer = Vec::<Complex32>::with_capacity(size);

    // Populate buffer
    for i in (0..message.len()).step_by(BYTES_PER_CONTROL)
    {
        // Indicate beginning of transmission
        for _ in 0..OVERSAMPLING
        {
            final_buffer.push(Complex32::new(0.0, 1.0));
        }
        for _ in 0..OVERSAMPLING
        {
            final_buffer.push(Complex32::new(0.0, -1.0));
        }

        // Encode message bytes
        for &c in &message[i..std::cmp::min(i+BYTES_PER_CONTROL, message.len())]
        {
            for offset in [6, 4, 2, 0]
            {
                for _ in 0..OVERSAMPLING
                {
                    final_buffer.push(symbol::symbol_to_qpsk((c >> offset) & 0b11));
                }
            }
        }
    }

    // Final 0 bytes to indicate the end
    for _ in 0..(message.len() % BYTES_PER_CONTROL)
    {
        for _ in 0..OVERSAMPLING
        {
            final_buffer.push(symbol::symbol_to_qpsk(0b00));
        }
    }

    // Send data to the pluto device
    pluto.lock().unwrap().tx(Some(vec![final_buffer]))?;
    Ok(())
}

/// Receives bytes from the pluto device and sends them to the tun device
/// 
/// # Errors
/// This function may fail if the pluto device's rx_complex function fails
/// 
/// # Panics
/// This function may panic if the mutex holding the pluto device was poisoned
/// 
/// # Examples
/// ```
/// let pluto = Arc::new(Mutex::new(Pluto::new(Some("ip:192.168.2.1".to_owned()))).unwrap();
/// let tun_device = TunDevice::new("tun0".to_owned()).unwrap();
/// let res = function_rx(&tun_device, &pluto);
/// if res.is_err() {
///     // Handle error
/// }
/// ```
fn function_rx(tun_device: &tun::TunDevice, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<!, ()>
{
    let mut buffer = vec![];
    let mut previous_packet = -1;
    let mut buffer_supposed_length = 0;
    loop {
        // Get data from the pluto device
        let data = pluto.lock().unwrap().rx_complex().map_err(|_| ())?;
        let samples = &data[0];

        // If we are in the middle of a transmission, wait for the end
        let mut i = 0;
        while i < samples.len() && samples[i].norm() > NOISE_FLOOR
        {
            i += 1;
        }

        while i < samples.len()
        {
            // Check for bytes above the noise floor, this is the beginning of a new message
            if samples[i].norm() > NOISE_FLOOR
            {
                let mut msg_bytes = vec![];
                let mut estimated_angle_opt = None as Option<f32>;
                while i < samples.len() && msg_bytes.len() < FRAME_SPLIT_LENGTH
                {
                    let max_point = std::cmp::min(i + OVERSAMPLING - SAMPLING_MARGIN, samples.len());
                    let min_point = std::cmp::min(i + SAMPLING_MARGIN, samples.len());

                    // Get the mean signal, it will allow to compute the phase shift
                    let reference_points = &samples[min_point..max_point];
                    let mut reference = num::Complex::new(0.0, 0.0);
                    let angle;
                    for point in reference_points {
                        reference += point;
                    }
                    reference /= reference_points.len() as f32;

                    i += OVERSAMPLING;
                    if let Some(estimated_angle) = estimated_angle_opt && symbol::closest_symb(reference * num::Complex::from_polar(1.0, -estimated_angle)) != symbol::closest_symb(num::Complex::new(0.0, 1.0))
                    {
                        // Phase shifted too much, don't trust this reference
                        i += OVERSAMPLING;
                        angle = estimated_angle;
                    }
                    else
                    {
                        // Compute the phase correction needed
                        angle = (reference * num::Complex::new(0.0, 1.1).conj()).arg();

                        // Synchronize by finding the switch between 1j to -1j
                        let mut j = i - (OVERSAMPLING / 4);
                        while j < samples.len() && j < i + (OVERSAMPLING / 4) && (samples[j] * num::Complex::from_polar(1.0, -angle)).arg() > 0.0 {
                            j += 1;
                        }

                        let max_point_inner = std::cmp::min(j + OVERSAMPLING - SAMPLING_MARGIN, samples.len());
                        let min_point_inner = std::cmp::min(j + SAMPLING_MARGIN, samples.len());

                        let some_points = &samples[min_point_inner..max_point_inner];
                        let mut some = num::Complex::new(0.0, 0.0);
                        for point in some_points {
                            some += point;
                        }
                        some /= some_points.len() as f32;
                        if j < i + (OVERSAMPLING / 4) && symbol::closest_symb(some * num::Complex::from_polar(1.0, -angle)) == symbol::closest_symb(num::Complex::new(0.0, -1.0))
                        {
                            // Time synchronization is good
                            i = j + OVERSAMPLING;
                        }
                        else
                        {
                            // Time synchronization is bad, use last computed value
                            i += OVERSAMPLING;
                        }
                    }

                    estimated_angle_opt = Some(angle);

                    for _ in 0..BYTES_PER_CONTROL
                    {
                        let mut byte = 0;
                        for _ in 0..4
                        {
                            let max_point = std::cmp::min(i + OVERSAMPLING - SAMPLING_MARGIN, samples.len());
                            let min_point = std::cmp::min(i + SAMPLING_MARGIN, samples.len());

                            // Get real data, correct it
                            let data_points = &samples[min_point..max_point];
                            let mut data = num::Complex::new(0.0, 0.0);
                            for point in data_points {
                                data += point;
                            }
                            data /= data_points.len() as f32;

                            data *= num::Complex::from_polar(1.0, -angle);

                            i += OVERSAMPLING;

                            byte <<= 2;
                            byte |= symbol::closest_symb(data);
                        }
                        msg_bytes.push(byte);
                    }

                    i += 1;
                }

                println!("{:02x?}", msg_bytes.iter().map(|f| *f as u8 as char).collect::<Vec<char>>());

                // Check if the received frame is long enough
                if msg_bytes.len() < FRAME_SPLIT_LENGTH {
                    continue;
                }

                // Verify the checksum
                let checksum_correct = msg_bytes.iter().fold(0, |acc, e| acc ^ e);
                if checksum_correct != 7 {
                    println!("Refused because of checksum : {:02x?}", msg_bytes);
                    continue;
                }

                // Check for packets in order
                if msg_bytes[1] as i16 == previous_packet {
                    println!("Ignored because of packet index : {:02x?}", msg_bytes);
                    continue;
                }

                if msg_bytes[1] as i16 != previous_packet + 1 {
                    previous_packet = -1;
                    buffer = vec![];
                    println!("Refused because of packet index : {:02x?}", msg_bytes);
                    continue;
                }

                // Check frame type (1 or N)
                if msg_bytes[1] == 0
                {
                    buffer.append(&mut msg_bytes[4..].to_vec());
                    buffer_supposed_length = u16::from_be_bytes([msg_bytes[2] as u8, msg_bytes[3] as u8]);
                }
                else
                {
                    buffer.append(&mut msg_bytes[2..].to_vec());
                }

                previous_packet = msg_bytes[1] as i16;

                // Frame fully reconstructed
                if buffer.len() >= buffer_supposed_length as usize {
                    println!("GOT IT");
                    tun_device.write_to_tun(&buffer[..buffer_supposed_length as usize].to_vec());
                    buffer = vec![];
                    buffer_supposed_length = 0;
                    previous_packet = -1;
                }

            }

            i += 1;
        }
    }
}

/// Proxy function for function_rx
/// Sends the bytes passed as an argument to the pluto device passed as argument
/// 
/// # Errors
/// This function may fail if the pluto tx function fails
/// 
/// # Panics
/// This function may panic if the mutex holding the pluto device was poisoned
/// 
/// # Examples
/// ```
/// let pluto = Arc::new(Mutex::new(Pluto::new(Some("ip:192.168.2.1".to_owned()))).unwrap();
/// let tun_device = TunDevice::new("tun0".to_owned()).unwrap();
/// let res = tun_recv_and_wrte(&tun_device, &pluto);
/// if res.is_err() {
///     // Handle error
/// }
/// ```
fn tun_recv_and_wrte(tun_device: &tun::TunDevice, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<!, ()>
{
    function_rx(tun_device, pluto)
}

/// Reads bytes from the TUN device, encodes them, and sends them via function_tx
/// 
/// # Errors
/// This function may fail if function_tx fails
/// 
/// # Panics
/// This function may panic if the mutex holding the pluto device was poisoned
/// 
/// # Examples
/// ```
/// let pluto = Arc::new(Mutex::new(Pluto::new(Some("ip:192.168.2.1".to_owned()))).unwrap();
/// let tun_device = TunDevice::new("tun0".to_owned()).unwrap();
/// let res = tun_read_and_send(&tun_device, &pluto);
/// if res.is_err() {
///     // Handle error
/// }
/// ```
fn tun_read_and_send(tun_device: &tun::TunDevice, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<!, ()>
{
    println!("Starting function TX");

    // let mut packet = vec![0u8, 0, 0, 5, 'H' as u8, 'E' as u8, 'L' as u8, 'L' as u8, 'O' as u8, 0, 0, 0];
    // let sum = packet.iter().fold(0, |acc, e| acc ^ e);
    // packet[0] = sum ^ 7;
    // loop {
    //     function_tx(&packet, pluto);
    // }
    // return Err(());

    loop {
        // Get bytes to send and length
        let mut first_packet = tun_device.read_from_tun();
        let packet_length = first_packet.len() as u16;

        println!("Packet length = {}", packet_length);

        if packet_length > tun::TUN_MTU as u16 {
            continue;
        }

        // Frame 1
        let packet_length_bytes: [u8; 2] = packet_length.to_be_bytes();

        // Create initial header
        let mut first_header = vec![0u8, 0u8, packet_length_bytes[0], packet_length_bytes[1]];
        let buf = first_packet.drain((FRAME_SPLIT_LENGTH - first_header.len())..).collect::<Vec<u8>>();

        // Append payload
        first_header.append(&mut first_packet);

        // Compute checksum
        let sum = first_header.iter().fold(0, |acc, e| acc ^ e);
        first_header[0] = sum ^ 7;

        // Transmit twice
        for _ in 0..FRAME_TRANSMIT_REPEAT_COUNT {
            function_tx(&first_header, &pluto)?;
        }

        // Frame N
        let mut slice = buf.as_slice();
        for i in 1..
        {
            // Create header
            let mut header = vec![0u8, i];

            // Take following bytes
            let mut bytes = slice.iter().take(FRAME_SPLIT_LENGTH - header.len()).map(|f| *f).collect::<Vec<u8>>();
            if bytes.len() == 0 {
                break;
            }

            // Append payload and padding bytes
            header.append(&mut bytes);
            while header.len() != FRAME_SPLIT_LENGTH {
                header.push(0);
            }

            // Compute checksum
            let sum = header.iter().fold(0, |acc, e| acc ^ e);
            header[0] = sum ^ 7;

            // Transmit twice
            for _ in 0..FRAME_TRANSMIT_REPEAT_COUNT {
                function_tx(&header, &pluto)?;
            }

            // Move iterator forward
            if FRAME_SPLIT_LENGTH - 2 > slice.len() {
                break;
            }
            slice = &slice[FRAME_SPLIT_LENGTH - 2..];
        }
    }
}

fn main() -> Result<(), i32> {
    // Check permission to interact with TUN
    if uid::geteuid() != 0
    {
        eprintln!("Error, you must be root");
        return Err(1);
    }

    // Use command-line arguments
    let args = std::env::args().collect::<Vec<String>>();
    if args.len() != 2 {
        eprintln!("Usage: {} <server|client>", args[0]);
        return Err(1);
    }
    let (id, other_id, freq_snd, freq_rcv) = match &args[1] {
        f if f == "server" => (2, 3, CENTER_FREQUENCY_SERVER, CENTER_FREQUENCY_CLIENT),
        f if f == "client" => (3, 2, CENTER_FREQUENCY_CLIENT, CENTER_FREQUENCY_SERVER),
        _ => {
            eprintln!("Error");
            return Err(1);
        }
    };

    // Configure TUN device with IP address, peer address and netmask
    let ip_address_str = format!("10.0.0.{}", id);
    let peer_ip_address_str = format!("10.0.0.{}", other_id);

    let ip_address = ip_address_str.as_bytes();
    let peer_ip_address = peer_ip_address_str.as_bytes();
    let netmask = b"255.255.255.255";

    let pluto_ip_address_string = format!("ip:192.168.{}.1", id);

    let tun_device_name = format!("tun{}", id);
    
    let tun_device = tun::TunDevice::new(tun_device_name).map_err(|_| 1)?;
    tun_device.set_ip(std::net::Ipv4Addr::parse_ascii(ip_address).unwrap(), std::net::Ipv4Addr::parse_ascii(peer_ip_address).unwrap(), std::net::Ipv4Addr::parse_ascii(netmask).unwrap()).map_err(|_| 1)?;

    // Connect to Pluto device and set parameters
    let mut pluto = adi::pluto::Pluto::new(Some(pluto_ip_address_string)).map_err(|_| 1)?;

    // TX + RX
    pluto.set_gain_control_mode_chan0("manual").map_err(|_| 1)?;
    pluto.set_sample_rate(SAMPLE_RATE).map_err(|_| 1)?;

    // TX
    pluto.set_tx_rf_bandwidth(SAMPLE_RATE).map_err(|_| 1)?;
    pluto.set_tx_lo(freq_snd).map_err(|_| 1)?;
    pluto
        .set_tx_hardwaregain_chan0(TX_HARDWARE_GAIN)
        .map_err(|_| 1)?;

    // RX
    pluto
        .set_rx_hardwaregain_chan0(RX_HARDWARE_GAIN)
        .map_err(|_| 1)?;
    pluto.set_rx_lo(freq_rcv).map_err(|_| 1)?;
    pluto.set_rx_rf_bandwidth(SAMPLE_RATE).map_err(|_| 1)?;
    pluto.set_rx_buffer_size(NUM_SAMPS);

    let pluto_mutex = std::sync::Arc::new(std::sync::Mutex::new(pluto));
    let pluto_mutex_clone = std::sync::Arc::clone(&pluto_mutex);

    // Run TX and RX threasds
    let mut error_status = 0;
    std::thread::scope(|s| {
        let thread_send_status = s.spawn(|| {
            tun_read_and_send(&tun_device, &pluto_mutex_clone)
        });

        let thread_recv_status = s.spawn(|| {
            tun_recv_and_wrte(&tun_device, &pluto_mutex)
        });

        if thread_send_status.join().is_err() {
            error_status = 1;
        }

        if thread_recv_status.join().is_err() {
            error_status = 1;
        }
    });

    if error_status != 0 {
        return Err(error_status);
    }

    Ok(())
}

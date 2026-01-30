#![feature(ascii_char)]
#![feature(int_from_ascii)]
#![feature(ip_as_octets)]
#![feature(addr_parse_ascii)]
#![feature(iter_advance_by)]

pub extern crate adi;

use std::os::fd;

use adi::traits::*;
use num::complex::Complex32;

use crate::tun::read_from_tun;

mod symbol;

#[cfg_attr(target_os = "linux", path = "linux/tun.rs")]
#[cfg_attr(target_os = "macos", path = "macos/tun.rs")]
mod tun;

mod uid;

static TX_HARDWARE_GAIN: f32 = -50.0;
static RX_HARDWARE_GAIN: f32 = 60.0;
static CENTER_FREQUENCY_CLIENT: u64 = 2_400_000_000;
static CENTER_FREQUENCY_SERVER: u64 = 2_483_500_000;
static SAMPLE_RATE: u32 = 5_000_000;
static NUM_SAMPS: usize = 1_000_000;
static OVERSAMPLING: usize = 25;
static NOISE_FLOOR: f32 = 1300f32;
static SAMPLING_MARGIN: usize = OVERSAMPLING / 5;
static BYTES_PER_CONTROL: usize = 2;

fn function_tx(message: &Vec<u8>, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<(), ()>
{
    let size = (2 + 4 * BYTES_PER_CONTROL) * OVERSAMPLING * ((message.len() + BYTES_PER_CONTROL - 1) / BYTES_PER_CONTROL);
    let mut final_buffer = Vec::<Complex32>::with_capacity(size);
    for i in (0..message.len()).step_by(BYTES_PER_CONTROL)
    {
        for _ in 0..OVERSAMPLING
        {
            final_buffer.push(Complex32::new(0.0, 1.0));
        }
        for _ in 0..OVERSAMPLING
        {
            final_buffer.push(Complex32::new(0.0, -1.0));
        }
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
    for _ in 0..(message.len() % BYTES_PER_CONTROL)
    {
        for _ in 0..OVERSAMPLING
        {
            final_buffer.push(symbol::symbol_to_qpsk(0b00));
        }
    }

    pluto.lock().unwrap().tx(Some(vec![final_buffer]))?;
    Ok(())
}

fn function_rx(fd: i32, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<(), ()>
{
    let mut buffer = vec![];
    let mut previous_packet = -1;
    let mut buffer_supposed_length = 0;
    loop {
        let data = pluto.lock().unwrap().rx_complex().map_err(|_| ())?;
        let samples = &data[0];

        let mut i = 0;
        while i < samples.len() && samples[i].norm() > NOISE_FLOOR
        {
            i += 1;
        }

        while i < samples.len()
        {
            if samples[i].norm() > NOISE_FLOOR
            {
                let mut msg_bytes = vec![];
                let mut estimated_angle_opt = None as Option<f32>;
                while i < samples.len() && msg_bytes.len() < 20
                {
                    let max_point = std::cmp::min(i + OVERSAMPLING - SAMPLING_MARGIN, samples.len());
                    let min_point = std::cmp::min(i + SAMPLING_MARGIN, samples.len());

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
                        i += OVERSAMPLING;
                        angle = estimated_angle;
                    }
                    else
                    {
                        angle = (reference * num::Complex::new(0.0, 1.1).conj()).arg();

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
                            i = j + OVERSAMPLING;
                        }
                        else
                        {
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

                println!("{:02x?}", msg_bytes.iter().map(|f| *f as u8).collect::<Vec<u8>>());

                if msg_bytes.len() < 20 {
                    continue;
                }

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

                let checksum_correct = msg_bytes.iter().fold(0, |acc, e| acc ^ e);
                if checksum_correct != 0 {
                    println!("Refused because of checksum : {:02x?}", msg_bytes);
                    continue;
                }

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

                println!("Expected {}, got {}", buffer_supposed_length, buffer.len());

                if buffer.len() >= buffer_supposed_length as usize {
                    println!("GOT IT");
                    tun::write_to_tun(fd, &buffer[..buffer_supposed_length as usize].to_vec());
                }

            }

            i += 1;
        }
    }
}

fn tun_recv_and_wrte(fd: i32, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<(), ()>
{
    function_rx(fd, pluto)
}

fn tun_read_and_send(fd: i32, pluto: &std::sync::Arc<std::sync::Mutex<adi::pluto::Pluto>>) -> Result<(), ()>
{
    println!("Starting function TX");

    loop {
        let mut first_packet = tun::read_from_tun(fd);

        println!("Got new packet of length = {}", first_packet.len());

        let packet_length = first_packet.len() as u16;
        let packet_length_bytes: [u8; 2] = packet_length.to_be_bytes();
        let mut first_header = vec![0u8, 0u8, packet_length_bytes[0], packet_length_bytes[1]];
        let mut buf = first_packet.drain(16..).collect::<Vec<u8>>();
        first_header.append(&mut first_packet);
        let sum = first_header.iter().fold(0, |acc, e| acc ^ e);
        first_header[0] = sum;
        // SEND THE PACKET HERE
        for _ in 0..4 {
            function_tx(&first_header, &pluto);
        }
        // println!("Bytes {:?}", first_header);
        let mut iterator = buf.iter();
        for i in 1..
        {
            let mut bytes = iterator.clone().take(18).map(|f| *f).collect::<Vec<u8>>();
            if bytes.len() == 0 {
                break;
            }
            let mut header = vec![0u8, i];
            header.append(&mut bytes);
            while header.len() != 20 {
                header.push(0);
            }
            let sum = header.iter().fold(0, |acc, e| acc ^ e);
            header[0] = sum;
            // SEND THE PACKET HERE
            for _ in 0..4 {
                function_tx(&header, &pluto);
            }
            // println!("Bytes {:?}", header);
            let res = iterator.advance_by(18);
            if res.is_err() {
                break;
            }
        }
    }
}

fn main() -> Result<(), i32> {
    // TUN
    if uid::geteuid() != 0
    {
        eprintln!("Error, you must be root");
        return Err(1);
    }

    let args = std::env::args().collect::<Vec<String>>();
    if args.len() != 2 {
        eprintln!("Usage: rx tx");
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

    let ip_address_str = format!("10.0.0.{}", id);
    let peer_ip_address_str = format!("10.0.0.{}", other_id);

    let ip_address = ip_address_str.as_bytes();
    let peer_ip_address = peer_ip_address_str.as_bytes();
    let netmask = b"255.255.255.255";

    let pluto_ip_address_string = format!("ip:192.168.{}.1", id);

    let tun_device = format!("tun{}", id);
    
    let fd = tun::tun_alloc(tun_device.as_str()).map_err(|_| 1)?;
    tun::set_ip(tun_device.as_str(), std::net::Ipv4Addr::parse_ascii(ip_address).unwrap(), std::net::Ipv4Addr::parse_ascii(peer_ip_address).unwrap(), std::net::Ipv4Addr::parse_ascii(netmask).unwrap()).map_err(|_| 1)?;

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

    let thread_status = std::thread::spawn(move || {
        tun_read_and_send(fd, &pluto_mutex_clone)
    });

    std::thread::sleep(std::time::Duration::new(1, 0));
    tun_recv_and_wrte(fd, &pluto_mutex).map_err(|_| 1)?;

    thread_status.join().map_err(|_| 1)?;

    // let pluto_mutex = std::sync::Arc::new(std::sync::Mutex::new(pluto));

    // BIDIRECTIONAL
    // let stop_mutex = std::sync::Arc::new(std::sync::Mutex::new(false));
    // let stop_mutex_clone = std::sync::Arc::clone(&stop_mutex);

    // let thread_status = std::thread::spawn(|| {
    //     function_tx(stop_mutex_clone)
    // });
    // std::thread::sleep(std::time::Duration::new(1, 0));
    // function_rx()?;
    // *(stop_mutex.lock().unwrap()) = true;
    // thread_status.join().map_err(|_| 1)?
    Ok(())
}

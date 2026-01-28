#![feature(ascii_char)]
#![feature(int_from_ascii)]
#![feature(ip_as_octets)]
#![feature(addr_parse_ascii)]
#![feature(iter_advance_by)]

pub extern crate adi;

use adi::traits::*;
use num::complex::Complex32;

use crate::symbol::symbol_to_qpsk;

mod symbol;

#[cfg_attr(target_os = "linux", path = "linux/tun.rs")]
#[cfg_attr(target_os = "macos", path = "macos/tun.rs")]
mod tun;

mod uid;

static TX_HARDWARE_GAIN: f32 = -30.0;
static RX_HARDWARE_GAIN: f32 = 50.0;
static CENTER_FREQUENCY: u64 = 2_400_000_000;
static SAMPLE_RATE: u32 = 5_000_000;
static NUM_SAMPS: usize = 600_000;
static OVERSAMPLING: usize = 25;
static NOISE_FLOOR: i32 = 1500;
static SAMPLING_MARGIN: usize = OVERSAMPLING / 5;
static BYTES_PER_CONTROL: usize = 2;

fn function_tx() -> Result<(), ()>
{
    println!("Starting function TX");

    let mut pluto = adi::pluto::Pluto::new(Some("ip:192.168.3.1")).map_err(|_| ())?;

    pluto.set_gain_control_mode_chan0("manual").map_err(|_| ())?;
    pluto.set_sample_rate(SAMPLE_RATE).map_err(|_| ())?;

    pluto.set_tx_rf_bandwidth(SAMPLE_RATE).map_err(|_| ())?;
    pluto.set_tx_lo(CENTER_FREQUENCY).map_err(|_| ())?;
    pluto
        .set_tx_hardwaregain_chan0(TX_HARDWARE_GAIN)
        .map_err(|_| ())?;

    let message = "Hello world lorem ipsum dolor si";
    loop {
        let size = (2 + 4 * BYTES_PER_CONTROL) * OVERSAMPLING * ((message.as_bytes().len() + BYTES_PER_CONTROL - 1) / BYTES_PER_CONTROL);
        let mut final_buffer = Vec::<Complex32>::with_capacity(size);
        for i in (0..message.as_bytes().len()).step_by(BYTES_PER_CONTROL)
        {
            for _ in 0..OVERSAMPLING
            {
                final_buffer.push(Complex32::new(0.0, 1.0));
            }
            for _ in 0..OVERSAMPLING
            {
                final_buffer.push(Complex32::new(0.0, -1.0));
            }
            for &c in &message.as_bytes()[i..std::cmp::min(i+BYTES_PER_CONTROL, message.as_bytes().len())]
            {
                for offset in [6, 4, 2, 0]
                {
                    for _ in 0..OVERSAMPLING
                    {
                        final_buffer.push(symbol_to_qpsk((c >> offset) & 0b11));
                    }
                }
            }
        }
        for _ in 0..(message.as_bytes().len() % BYTES_PER_CONTROL)
        {
            for _ in 0..OVERSAMPLING
            {
                final_buffer.push(symbol_to_qpsk(0b00));
            }
        }

        pluto.tx(Some(vec![final_buffer]))?;
    }
}

// fn function_rx() -> Result<(), i32>
// {
//     println!("Starting function RX");

//     let mut pluto = adi::pluto::Pluto::new(Some("ip:192.168.2.1")).map_err(|_| 1)?;

//     pluto.set_gain_control_mode_chan0("manual").map_err(|_| 1)?;
//     pluto.set_sample_rate(SAMPLE_RATE).map_err(|_| 1)?;

//     pluto
//         .set_rx_hardwaregain_chan0(RX_HARDWARE_GAIN)
//         .map_err(|_| 1)?;
//     pluto.set_rx_lo(CENTER_FREQUENCY).map_err(|_| 1)?;
//     pluto.set_rx_rf_bandwidth(SAMPLE_RATE).map_err(|_| 1)?;
//     pluto.set_rx_buffer_size(NUM_SAMPS);

//     let data = pluto.rx_complex().map_err(|_| 1)?;
//     let useful = data[0]
//         .iter()
//         .map(|f| num::Complex::new(f.0 as f64, f.1 as f64))
//         .collect::<Vec<symbol::PlutoComplex>>();
//     let messages = symbol::decode_message(OVERSAMPLING, NOISE_FLOOR, SAMPLING_MARGIN, useful)
//         .map_err(|_| 1)?;
//     println!("Received messages = {:?}", messages);

//     println!("Stopping function RX");

//     Ok(())
// }

fn main() -> Result<(), i32> {
    // let mut pluto = adi::pluto::Pluto::new(Some("ip:192.168.3.1")).map_err(|_| 1)?;

    // TX + RX
    // pluto.set_gain_control_mode_chan0("manual").map_err(|_| 1)?;
    // pluto.set_sample_rate(SAMPLE_RATE).map_err(|_| 1)?;

    // TX
    // pluto.set_tx_rf_bandwidth(SAMPLE_RATE).map_err(|_| 1)?;
    // pluto.set_tx_lo(CENTER_FREQUENCY).map_err(|_| 1)?;
    // pluto
    //     .set_tx_hardwaregain_chan0(TX_HARDWARE_GAIN)
    //     .map_err(|_| 1)?;

    // RX
    // pluto
    //     .set_rx_hardwaregain_chan0(RX_HARDWARE_GAIN)
    //     .map_err(|_| 1)?;
    // pluto.set_rx_lo(CENTER_FREQUENCY).map_err(|_| 1)?;
    // pluto.set_rx_rf_bandwidth(SAMPLE_RATE).map_err(|_| 1)?;
    // pluto.set_rx_buffer_size(NUM_SAMPS);


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

    // TUN
    // if uid::geteuid() != 0
    // {
    //     eprintln!("Error, you must be root");
    //     return Err(1);
    // }
    //
    // let ip_address = b"10.0.0.11";
    // let netmask = b"255.255.255.0";
    //
    // let fd = tun::tun_alloc("tun0").map_err(|_| 1)?;
    // println!("FD = {}", fd);
    // tun::set_ip("tun0", std::net::Ipv4Addr::parse_ascii(ip_address).unwrap(), std::net::Ipv4Addr::parse_ascii(netmask).unwrap()).map_err(|_| 1)?;
    // std::thread::sleep(std::time::Duration::new(10, 0));
    // loop {
    //     let mut buf = tun::read_from_tun(fd);
    //     let mut preamble = vec![(buf.len() / 10) as u8];
    //     preamble.append(&mut buf);
    //     println!("READ BYTES = {:?}", preamble.iter().map(|f| *f as char).collect::<Vec<char>>());
    //     let mut iterator = preamble.iter();
    //     loop {
    //         let symbols = symbol::message_to_symbols(iterator.clone().take(10)).map_err(|_| 1)?;
    //         println!("Symbols = {:?}", symbols);
    //         let res = iterator.advance_by(10);
    //         if res.is_err() {
    //             break;
    //         }
    //     }
    // }

    function_tx().map_err(|_| 1)?;
    Ok(())
}

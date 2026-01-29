#![feature(ascii_char)]
#![feature(int_from_ascii)]
#![feature(ip_as_octets)]
#![feature(addr_parse_ascii)]
#![feature(iter_advance_by)]

pub extern crate adi;

use adi::traits::*;
use num::complex::Complex32;

mod symbol;

#[cfg_attr(target_os = "linux", path = "linux/tun.rs")]
#[cfg_attr(target_os = "macos", path = "macos/tun.rs")]
mod tun;

mod uid;

static TX_HARDWARE_GAIN: f32 = -30.0;
static RX_HARDWARE_GAIN: f32 = 70.0;
static CENTER_FREQUENCY: u64 = 2_400_000_000;
static SAMPLE_RATE: u32 = 5_000_000;
static NUM_SAMPS: usize = 200_000;
static OVERSAMPLING: usize = 25;
static NOISE_FLOOR: f32 = 1000f32;
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

    let message = "Hello world lorem ipsum dolor sit amet!!";
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
                        final_buffer.push(symbol::symbol_to_qpsk((c >> offset) & 0b11));
                    }
                }
            }
        }
        for _ in 0..(message.as_bytes().len() % BYTES_PER_CONTROL)
        {
            for _ in 0..OVERSAMPLING
            {
                final_buffer.push(symbol::symbol_to_qpsk(0b00));
            }
        }

        pluto.tx(Some(vec![final_buffer]))?;
    }
}

fn function_rx() -> Result<(), i32>
{
    println!("Starting function RX");

    let mut pluto = adi::pluto::Pluto::new(Some("ip:192.168.2.1")).map_err(|_| 1)?;

    pluto.set_gain_control_mode_chan0("manual").map_err(|_| 1)?;
    pluto.set_sample_rate(SAMPLE_RATE).map_err(|_| 1)?;

    pluto
        .set_rx_hardwaregain_chan0(RX_HARDWARE_GAIN)
        .map_err(|_| 1)?;
    pluto.set_rx_lo(CENTER_FREQUENCY).map_err(|_| 1)?;
    pluto.set_rx_rf_bandwidth(SAMPLE_RATE).map_err(|_| 1)?;
    pluto.set_rx_buffer_size(NUM_SAMPS);

    let data = pluto.rx_complex().map_err(|_| 1)?;
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
                let reference_points = &samples[(i + SAMPLING_MARGIN)..(i + OVERSAMPLING - SAMPLING_MARGIN)];
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
                    while j < i + (OVERSAMPLING / 4) && (samples[j] * num::Complex::from_polar(1.0, -angle)).arg() > 0.0 {
                        j += 1;
                    }
                    let some_points = &samples[(j + SAMPLING_MARGIN)..(j + OVERSAMPLING - SAMPLING_MARGIN)];
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
                        let data_points = &samples[(i + SAMPLING_MARGIN)..(i + OVERSAMPLING - SAMPLING_MARGIN)];

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

            println!("{:?}", msg_bytes.iter().map(|f| *f as u8 as char).collect::<Vec<char>>());
        }

        i += 1;
    }

    println!("Stopping function RX");

    Ok(())
}

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

    let args = std::env::args().collect::<Vec<String>>();
    if args.len() != 2 {
        eprintln!("Usage: rx tx");
        return Err(1);
    }
    match &args[1] {
        f if f == "tx" => function_tx().map_err(|_| 1)?,
        f if f == "rx" => function_rx().map_err(|_| 1)?,
        _ => {
            eprintln!("Error");
            return Err(1);
        }
    };
    Ok(())
}

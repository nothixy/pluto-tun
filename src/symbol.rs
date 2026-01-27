pub type PlutoComplex = num::Complex<f64>;
static QPSK_TABLE: [PlutoComplex; 4] = [
    num::Complex::new(1.0, 0.0),
    num::Complex::new(0.0, 1.0),
    num::Complex::new(0.0, -1.0),
    num::Complex::new(-1.0, 0.0),
];

pub fn symbol_to_qpsk(symb: i8) -> Result<PlutoComplex, ()> {
    if symb < 0b00 || symb > 0b11 {
        return Err(());
    }
    Ok(QPSK_TABLE[symb as usize])
}

pub fn symb_sample(oversampling: i32, symb: PlutoComplex) -> Vec<PlutoComplex> {
    let mut ones_vec = vec![num::Complex::new(1.0, 0.0); oversampling as usize];
    let mut symb_vec = vec![symb; oversampling as usize];
    ones_vec.append(&mut symb_vec);
    ones_vec
        .iter()
        .map(|f| f * (1 << 14) as f64)
        .collect::<Vec<PlutoComplex>>()
}

pub fn closest_symb(qpsk: PlutoComplex) -> usize {
    let mut index = 0;
    let mut min_diff = (qpsk - QPSK_TABLE[0]).norm();
    for (i, item) in QPSK_TABLE.iter().enumerate().skip(1) {
        let diff = (qpsk - item).norm();
        if diff < min_diff {
            min_diff = diff;
            index = i;
        }
    }
    index
}

pub fn message_to_symbols<'a>(bidule: impl Iterator<Item = &'a u8>) -> Result<Vec<i8>, ()> {
    // let text_as_ascii = data.as_ascii().ok_or(())?;
    // let bidule = text_as_ascii.iter().map(|f| f.to_u8());
    let mut number = num::BigUint::from(0u32);
    for num in bidule {
        number *= 256u32;
        number += num::BigUint::from(*num);
    }
    let mut digits = vec![];
    while number > num::BigUint::from(0u32) {
        digits.push((1u32 + number.clone() % 3u32).to_u32_digits()[0] as i8);
        number /= 3u32;
    }
    digits.push(0b00);

    digits.reverse();

    Ok(digits)
}

pub fn symbols_to_messages(digits: Vec<usize>) -> Result<Vec<Vec<u8>>, ()> {
    let mut i = 0;
    while i < digits.len() && digits[i] != 0b00 {
        i += 1;
    }
    let mut messages = vec![];
    while i < digits.len() {
        while i < digits.len() && digits[i] == 0b00 {
            i += 1;
        }
        let mut num = num::BigUint::from(0usize);
        while i < digits.len() && digits[i] != 0b00 {
            num = num * num::BigUint::from(3usize) + digits[i] - num::BigUint::from(1usize);
            i += 1;
        }

        if num == num::BigUint::from(0usize) {
            messages.push(vec![]);
        }

        let string = num.to_bytes_be();
        // let string = String::from_utf8_lossy(num.to_bytes_be().as_slice()).to_string();
        messages.push(string);
    }

    Ok(messages)
}

pub fn decode_message(
    oversampling: i32,
    noise_floor: i32,
    sampling_margin: i32,
    samples: Vec<PlutoComplex>,
) -> Result<Vec<Vec<u8>>, ()> {
    let mut i = 0;
    while i < samples.len() - (2 * oversampling) as usize && samples[i].norm() > noise_floor as f64
    {
        i += 1;
    }

    let mut symbols = vec![];

    while i < samples.len() - (2 * oversampling) as usize {
        if samples[i].norm() > noise_floor as f64 {
            let reference_points = &samples[(i + sampling_margin as usize)
                ..(i + oversampling as usize - sampling_margin as usize)];
            if reference_points.is_empty() {
                return Err(());
            }
            let mut reference = num::Complex::new(0.0, 0.0);
            for elt in reference_points {
                reference += elt;
            }
            reference /= reference_points.len() as f64;

            let amplitude = reference.norm();
            let angle = (reference * (num::Complex::new(1.0, 0.0).conj())).arg();

            i += oversampling as usize;

            if amplitude < noise_floor as f64 {
                continue;
            }

            let data_points = &samples[(i + sampling_margin as usize)
                ..(i + oversampling as usize - sampling_margin as usize)];
            if data_points.is_empty() {
                return Err(());
            }
            let mut data = num::Complex::new(0.0, 0.0);
            for elt in data_points {
                data += elt;
            }
            data /= data_points.len() as f64;

            data *= num::Complex::from_polar(1.0, -angle);

            i += oversampling as usize;

            if data.norm() < noise_floor as f64 {
                continue;
            }

            data /= amplitude;

            symbols.push(closest_symb(data));
        }

        i += 1;
    }

    symbols_to_messages(symbols)
}

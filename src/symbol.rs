pub type PlutoComplex = num::Complex<f32>;
static QPSK_TABLE: [PlutoComplex; 4] = [
    num::Complex::new(1.0, 0.0),
    num::Complex::new(0.0, 1.0),
    num::Complex::new(0.0, -1.0),
    num::Complex::new(-1.0, 0.0),
];

pub fn symbol_to_qpsk(symb: u8) -> PlutoComplex {
    assert!(symb <= 0b11);
    QPSK_TABLE[symb as usize]
}

pub fn symb_sample(oversampling: i32, symb: PlutoComplex) -> Vec<PlutoComplex> {
    let mut ones_vec = vec![num::Complex::new(1.0, 0.0); oversampling as usize];
    let mut symb_vec = vec![symb; oversampling as usize];
    ones_vec.append(&mut symb_vec);
    ones_vec
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


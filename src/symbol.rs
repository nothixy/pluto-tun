static QPSK_TABLE: [adi::datatypes::PlutoComplex; 4] = [
    num::Complex::new(1.0, 0.0),
    num::Complex::new(0.0, 1.0),
    num::Complex::new(0.0, -1.0),
    num::Complex::new(-1.0, 0.0),
];

/// Returns the complex associated to symb
/// 
/// # Panics
/// The function might panic if the symbol passed as argument is more than 0b11
/// 
/// # Examples
/// ```
/// assert_eq!(symbol_to_qpsk(0b00), PlutoComplex::new(1.0, 0.0));
/// assert_eq!(symbol_to_qpsk(0b01), PlutoComplex::new(0.0, 1.0));
/// assert_eq!(symbol_to_qpsk(0b10), PlutoComplex::new(0.0, -1.0));
/// assert_eq!(symbol_to_qpsk(0b11), PlutoComplex::new(-1.0, 0.0));
/// ```
pub const fn symbol_to_qpsk(symb: u8) -> adi::datatypes::PlutoComplex {
    assert!(symb <= 0b11);
    QPSK_TABLE[symb as usize]
}

/// Returns the symbol associated to qpsk
/// 
/// # Panics
/// The function might panic if the complex passed as argument is 0
/// 
/// # Examples
/// ```
/// assert_eq!(PlutoComplex::new(5.0, 0.0), 0b00);
/// assert_eq!(PlutoComplex::new(5.0, 8.0), 0b01);
/// ```
pub fn closest_symb(qpsk: adi::datatypes::PlutoComplex) -> u8 {
    let qpsk_norm = qpsk / qpsk.norm();
    let mut index = 0u8;
    let mut min_diff = (qpsk_norm - QPSK_TABLE[0]).norm();
    for (i, item) in QPSK_TABLE.iter().enumerate().skip(1) {
        let diff = (qpsk_norm - item).norm();
        if diff < min_diff {
            min_diff = diff;
            index = i as u8;
        }
    }
    index
}


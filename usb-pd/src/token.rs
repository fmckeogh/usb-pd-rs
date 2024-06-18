#[repr(u8)]
pub enum Token {
    TxOn = 0xa1,
    Sop1 = 0x12,
    Sop2 = 0x13,
    Sop3 = 0x1b,
    Reset1 = 0x15,
    Reset2 = 0x16,
    PackSym = 0x80,
    JamCrc = 0xff,
    Eop = 0x14,
    TxOff = 0xfe,
}

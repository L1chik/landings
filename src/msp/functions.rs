#[repr(u16)]
#[derive(Debug)]
pub enum Functions {
    RangeFinder = 7937,
    OpticalFlow = 7938,
    Other(u16) = 0,
}

impl From<u16> for Functions {
    fn from(value: u16) -> Self {
        match value {
            7937 => Functions::RangeFinder,
            7938 => Functions::OpticalFlow,
            _ => Functions::Other(value)
        }
    }
}

#[derive(Debug)]
pub struct MspOpticalFlow {
    pub xm: i32,
    pub ym: i32,
    pub quality: u8,
}

impl From<&[u8]> for MspOpticalFlow {
    fn from(raw: &[u8]) -> Self {
        Self {
            xm: i32::from_le_bytes([raw[1], raw[2], raw[3], raw[4]]),
            ym: i32::from_le_bytes([raw[5], raw[6], raw[7], raw[8]]),
            quality: raw[0],
        }
    }
}

impl From<MspOpticalFlow> for [u8; 9] {
    fn from(val: MspOpticalFlow) -> [u8; 9] {
        let mut payload = [0u8; 9];
        payload[0] = val.quality;
        payload[1..=4].copy_from_slice(&val.xm.to_le_bytes());
        payload[5..=8].copy_from_slice(&val.ym.to_le_bytes());
        payload
    }
}

impl MspOpticalFlow {
    pub fn new(xm: i32, ym: i32, quality: u8) -> Self {
        Self { xm, ym, quality }
    }
}
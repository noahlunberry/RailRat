use embedded_hal::serial::{Read, Write};
use nb::block;

pub struct ScanPoint {
    pub distance_mm: f32,
    pub angle_deg: f32,
    pub quality: u8,
}

pub struct LiDAR<UART> {
    uart: UART,
}




impl LiDAR {
    
}
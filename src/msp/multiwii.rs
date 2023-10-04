#![allow(unused)]

use std::io;
use std::io::{Error, ErrorKind};
use async_trait::async_trait;
use crc::{Crc, CRC_8_DVB_S2};

const MSP_V2_HEADER: &[u8] = &[b'$', b'X'];

#[derive(Debug)]
pub struct MspPacket<'d> {
    pub flag: u8,
    pub function: u16,
    pub payload: &'d [u8],
}

pub struct MspComposer {
    flag: u8,
    function: u16,
}

impl MspComposer {
    pub fn new(flag: u8, function: u16) -> Self {
        Self { flag, function }
    }

    pub fn set_payload<'d>(&self, payload: &'d [u8]) -> MspPacket<'d> {
        MspPacket {
            flag: self.flag,
            function: self.function,
            payload,
        }
    }
}

pub struct Msp<RX, TX, const LEN: usize> {
    receiver: RX,
    com: TX,

    crc: Crc<u8>,

    inbound: [u8; LEN],
    outbound: [u8; LEN],
}

#[async_trait]
pub trait MSPReceiver {
    async fn receive(&mut self, buf: &mut [u8]);
}

#[async_trait]
pub trait MSPSender {
    async fn send(&mut self, packet: &[u8]);
}

impl<const LEN: usize, RX, TX> Msp<RX, TX, LEN> {
    pub fn new(receiver: RX, com: TX) -> Self {
        Self {
            receiver,
            com,
            crc: Crc::<u8>::new(&CRC_8_DVB_S2),
            inbound: [0; LEN],
            outbound: [0; LEN],
        }
    }
}

impl<const LEN: usize, RX, TX> Msp<RX, TX, LEN>
    where
        TX: MSPSender
{
    pub async fn send<'d>(&mut self, packet: &MspPacket<'d>) {
        self.outbound[0..=1].copy_from_slice(MSP_V2_HEADER);
        self.outbound[2..=3].copy_from_slice(&[b'<', packet.flag]);
        self.outbound[4..=5].copy_from_slice(&packet.function.to_le_bytes());
        self.outbound[6..=7].copy_from_slice(&(packet.payload.len() as u16).to_le_bytes());
        self.outbound[8..=7 + packet.payload.len()].copy_from_slice(packet.payload);
        self.outbound[8 + packet.payload.len()] =
            self.crc.checksum(&self.outbound[3..8 + packet.payload.len()]);

        self.com.send(&self.outbound[..(9 + packet.payload.len())]).await;
    }
}

impl<const LEN: usize, RX, TX> Msp<RX, TX, LEN>
    where
        RX: MSPReceiver
{
    pub fn receive(&mut self) -> io::Result<MspPacket> {
        self.receiver.receive(&mut self.inbound);
        let length = if &self.inbound[0..2] != MSP_V2_HEADER {
            Err(Error::new(ErrorKind::Unsupported, "Not a valid MSPv2 packet"))
        } else if self.inbound[2] as char != '<' {
            Err(Error::new(ErrorKind::Other, "Not a recv packet"))
        } else {
            self.validate_ingress()
        }?;

        Ok(MspPacket {
            flag: self.inbound[3],
            function: u16::from_le_bytes([self.inbound[4], self.inbound[5]]),
            payload: &self.inbound[8..length],
        })
    }

    fn validate_ingress(&mut self) -> io::Result<usize> {
        let len = u16::from_le_bytes([self.inbound[6], self.inbound[7]]) as usize + 8;
        let checksum = self.inbound[len];
        if checksum == self.crc.checksum(&self.inbound[3..len]) {
            Ok(len)
        } else {
            Err(Error::new(ErrorKind::InvalidData, "Invalid checksum"))
        }
    }
}

struct CustomRx<'a>(&'a [u8]);

struct CustomTx();

#[allow(unused)]
pub struct NoTx();

#[allow(unused)]
pub struct NoRx();

#[async_trait]
impl<'a> MSPReceiver for CustomRx<'a> {
    async fn receive(&mut self, buf: &mut [u8]) {
        buf[0..self.0.len()].copy_from_slice(self.0);
    }
}

#[async_trait]
impl MSPSender for CustomTx {
    async fn send(&mut self, packet: &[u8]) {
        println!("{:?}", packet)
    }
}

fn main() {
    let data = [36, 88, 60, 0, 1, 0x1f, 0x5, 0x0, 0xff, 0x2d, 0x0, 0x0, 0x0, 0xca];

    let payload = &[255, 45, 0, 0, 0];

    let msp_composer = MspComposer::new(0, 7937);

    let receiver = CustomRx(&data);

    let packet = msp_composer.set_payload(payload);

    let mut msp: Msp<_, _, 50> = Msp::new(receiver, CustomTx());

    msp.send(&packet);

    println!("{:?}", msp.receive().unwrap());
}
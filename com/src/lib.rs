use std::error::Error;
use std::fmt::{Display, Formatter};
use std::sync::{Arc, mpsc, Mutex};
use std::thread::spawn;
use std::time::Duration;

use mavlink::*;
use mavlink::ardupilotmega::CopterMode::COPTER_MODE_GUIDED;
use mavlink::error::MessageWriteError;
pub use mavlink::ardupilotmega::MavMessage;
pub use mavlink::MavlinkVersion as Mavlink;

pub struct MavlinkPi {
    vehicle: Arc<Mutex<Connection>>,
    header: MavHeader,
}

pub enum Speed {
    Slow = 57600,
    Medium = 115200,
    Fast = 921600,
}

type Connection = Box<dyn MavConnection<MavMessage> + Sync + Send>;
type MavRequest = Result<usize, MessageWriteError>;
type MavResponse = Result<(MavHeader, MavMessage), Box<dyn Error>>;

const TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Debug, Clone)]
pub struct NoMavlinkDeviceFoundError;

impl Display for NoMavlinkDeviceFoundError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.write_str("No MAVLink device found (disconnected)")
    }
}

impl Error for NoMavlinkDeviceFoundError {}

impl MavlinkPi {
    pub fn connect(serial_id: u8,
                   baud_rate: Speed,
                   version: MavlinkVersion) -> Result<MavlinkPi, Box<dyn Error>> {
        let loc = format!("serial:/dev/serial{serial_id}:{}", baud_rate as u32);

        let mut vehicle = connect::<MavMessage>(&loc)?;
        vehicle.set_protocol_version(version);

        println!("Establishing the MAVLink {version:?} connection... ({TIMEOUT:?})");
        Ok(MavlinkPi {
            vehicle: Self::dial(Mutex::new(vehicle).into(),
                                |connection| {
                                    connection.lock().unwrap().recv().unwrap();
                                    connection
                                },
            )?,
            header: MavHeader::default(),
        })
    }

    fn dial<F>(connection: Arc<Mutex<Connection>>,
               action: fn(Arc<Mutex<Connection>>) -> F) -> Result<F, Box<dyn Error>>
        where F: Sync + Send + 'static {
        let (tx, rx) = mpsc::channel();

        spawn(move || { tx.send(action(connection)) });
        match rx.recv_timeout(TIMEOUT) {
            Ok(payload) => Ok(payload),
            Err(_) => Err(Box::try_from(NoMavlinkDeviceFoundError).unwrap())
        }
    }

    pub fn send(&self, msg: MavMessage) -> MavRequest {
        self.vehicle.lock().unwrap().send(&self.header, &msg)
    }

    pub fn receive(&self) -> MavResponse {
        let (rtx, rrx) = mpsc::channel();
        let receiver = self.vehicle.clone();

        spawn(move || rtx.send(receiver.lock().unwrap().recv().unwrap()));
        match rrx.recv_timeout(TIMEOUT) {
            Ok(payload) => Ok(payload),
            Err(_) => Err(Box::try_from(NoMavlinkDeviceFoundError).unwrap())
        }
    }
}

pub fn guided() -> MavMessage {
    MavMessage::SET_MODE(ardupilotmega::SET_MODE_DATA {
        custom_mode: COPTER_MODE_GUIDED as u32,
        target_system: 0,
        base_mode: Default::default(),
    })
}

pub fn heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: ardupilotmega::MavType::MAV_TYPE_QUADROTOR,
        autopilot: ardupilotmega::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: ardupilotmega::MavModeFlag::empty(),
        system_status: ardupilotmega::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

pub fn request_parameters() -> MavMessage {
    MavMessage::PARAM_REQUEST_LIST(
        ardupilotmega::PARAM_REQUEST_LIST_DATA {
            target_system: 0,
            target_component: 0,
        },
    )
}

pub fn request_stream() -> MavMessage {


    MavMessage::MESSAGE_INTERVAL(
        ardupilotmega::MESSAGE_INTERVAL_DATA {
            interval_us: 100,
            message_id: 0,
        },
    )
}
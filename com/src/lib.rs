use std::error::Error;
use mavlink::*;
use mavlink::ardupilotmega::CopterMode::COPTER_MODE_GUIDED;
use mavlink::ardupilotmega::MavMessage;
use mavlink::error::{MessageReadError, MessageWriteError};
pub use mavlink::MavlinkVersion as Mavlink;

pub struct MavlinkPi {
    vehicle: Connection,
    header: MavHeader,
}

pub enum Speed {
    Slow = 57600,
    Medium = 115200,
    Fast = 921600,
}

type Connection = Box<dyn MavConnection<MavMessage>>;
type MavRequest = Result<usize, MessageWriteError>;
type MavResponse = Result<(MavHeader, MavMessage), MessageReadError>;

impl MavlinkPi {
    pub fn connect(serial_id: u8,
                   baud_rate: Speed,
                   version: MavlinkVersion) -> Result<MavlinkPi, Box<dyn Error>> {
        let loc = format!("serial:/dev/serial{serial_id}:{}", baud_rate as u32);
        let mut vehicle = connect::<MavMessage>(&loc)?;
        vehicle.set_protocol_version(version);

        Ok(MavlinkPi {
            vehicle,
            header: MavHeader::default(),
        })
    }

    pub fn send(&self, msg: MavMessage) -> MavRequest {
        self.vehicle.send(&self.header, &msg)
    }

    pub fn receive(&self) -> MavResponse {
        self.vehicle.recv()
    }
}

// TODO: Resolve the blocking `recv` call

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
    MavMessage::REQUEST_DATA_STREAM(
        ardupilotmega::REQUEST_DATA_STREAM_DATA {
            target_system: 0,
            target_component: 0,
            req_stream_id: 0,
            req_message_rate: 10,
            start_stop: 1,
        },
    )
}
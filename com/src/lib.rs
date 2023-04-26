use std::sync::{Arc, mpsc, Mutex};
use std::thread::spawn;
use std::time::Duration;

use mavlink::*;
use mavlink::ardupilotmega::{COMMAND_LONG_DATA, MavResult};
pub use mavlink::ardupilotmega::CopterMode::{COPTER_MODE_GUIDED, COPTER_MODE_LAND};
use mavlink::ardupilotmega::MavCmd::{self, *};

pub use mavlink::ardupilotmega::MavMessage;
use mavlink::common::MavMode::MAV_MODE_STABILIZE_ARMED;
use mavlink::common::MavResult::{MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_FAILED};

pub use mavlink::MavlinkVersion as Mavlink;

use anyhow::Result;
use thiserror::Error;
use crate::MavError::{Failed, NoMavlinkDeviceFoundError, InvalidCommand, SendError};

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
type MavRequest = Result<usize, MavError>;
type MavResponse = Result<MavResult, MavError>;

const TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Error, Debug)]
pub enum MavError {
    #[error("Failed")] Failed(),
    #[error("No response")] NoResponseError(),
    #[error("Command rejected")] InvalidCommand(),
    #[error("Unable to communicate")] SendError(),
    #[error("No MAVLink device found (disconnected)")] NoMavlinkDeviceFoundError(),
}

impl MavlinkPi {
    pub fn connect_sim(port: u32, version: MavlinkVersion) -> Result<MavlinkPi> {
        let loc = format!("udpin:127.0.0.1:{}", port);

        let mut connection = connect::<MavMessage>(&loc)?;
        connection.set_protocol_version(version);
        establish(connection)
    }

    pub fn connect(serial_id: u8,
                   baud_rate: Speed,
                   version: MavlinkVersion) -> Result<MavlinkPi> {
        let loc = format!("serial:/dev/serial{serial_id}:{}", baud_rate as u32);

        let mut connection = connect::<MavMessage>(&loc)?;
        connection.set_protocol_version(version);
        establish(connection)
    }

    pub fn send(&self, msg: MavMessage) -> MavRequest {
        self.vehicle.lock().unwrap().send(&self.header, &msg).map_err(|_| SendError())
    }

    pub fn receive(&self) -> Result<(MavHeader, MavMessage), MavError> {
        let (rtx, rrx) = mpsc::channel();
        let receiver = self.vehicle.clone();

        spawn(move || rtx.send(receiver.lock().unwrap().recv().unwrap()));
        match rrx.recv_timeout(TIMEOUT) {
            Ok(payload) => Ok(payload),
            Err(_) => Err(NoMavlinkDeviceFoundError())
        }
    }

    pub fn stabilize_armed(&self) -> MavResponse {
        self.command(MAV_CMD_DO_SET_MODE,
                     &[MAV_MODE_STABILIZE_ARMED as i32 as f32, 0., 0., 0., 0., 0., 0.])
    }

    pub fn enter_guided(&self) -> MavResponse {
        self.command(MAV_CMD_DO_SET_MODE,
                     &[COPTER_MODE_GUIDED as i32 as f32, 0., 0., 0., 0., 0., 0.])
    }

    pub fn land(&self) -> MavResponse {
        self.command(MAV_CMD_DO_SET_MODE,
                     &[COPTER_MODE_LAND as i32 as f32, 0., 0., 0., 0., 0., 0.])
    }

    pub fn takeoff(&self, height: f32) -> MavResponse {
        self.command(MAV_CMD_NAV_TAKEOFF,
                     &[0., 0., 0., 0., 0., 0., height])
    }

    pub fn arm(&self) -> MavResponse {
        self.command(MAV_CMD_COMPONENT_ARM_DISARM,
                     &[1., 0., 0., 0., 0., 0., 0.])
    }

    pub fn disarm(&self) -> MavResponse {
        self.command(MAV_CMD_COMPONENT_ARM_DISARM, DEFAULT_PARAMS)
    }

    pub fn force_arm(&self) -> MavResponse {
        self.command(MAV_CMD_COMPONENT_ARM_DISARM,
                     &[1., 21996., 0., 0., 0., 0., 0.])
    }

    pub fn force_disarm(&self) -> MavResponse {
        self.command(MAV_CMD_COMPONENT_ARM_DISARM,
                     &[0., 21996., 0., 0., 0., 0., 0., ])
    }

    fn command(&self, r#type: MavCmd, params: &[f32]) -> MavResponse {
        let data = MavMessage::COMMAND_LONG(
            COMMAND_LONG_DATA {
                param1: params[0],
                param2: params[1],
                param3: params[2],
                param4: params[3],
                param5: params[4],
                param6: params[5],
                param7: params[6],
                command: r#type,
                target_system: 0,
                target_component: 0,
                confirmation: 0,
            }
        );
        self.send(data)?;
        loop {
            let (_, resp) = self.receive()?;
            match resp {
                MavMessage::COMMAND_ACK(result) =>
                    if result.command == r#type {
                        if result.result as i32 == MAV_RESULT_ACCEPTED as i32 {
                            return Ok(result.result);
                        } else if result.result as i32 == MAV_RESULT_FAILED as i32 {
                            return Err(Failed());
                        } else if result.result as i32 == MAV_RESULT_DENIED as i32 {
                            return Err(InvalidCommand());
                        }
                    },
                _ => ()
            }
        }
    }
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

const DEFAULT_PARAMS: &[f32] = &[0., 0., 0., 0., 0., 0., 0.];

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

fn dial<F>(connection: Arc<Mutex<Connection>>,
           action: fn(Arc<Mutex<Connection>>) -> F) -> Result<F, MavError>
    where F: Sync + Send + 'static {
    let (tx, rx) = mpsc::channel();

    spawn(move || { tx.send(action(connection)) });
    match rx.recv_timeout(TIMEOUT) {
        Ok(payload) => Ok(payload),
        Err(_) => Err(NoMavlinkDeviceFoundError())
    }
}

fn establish(connection: Connection) -> Result<MavlinkPi> {
    println!("Establishing a MAVLink connection... ({TIMEOUT:?})");
    Ok(MavlinkPi {
        vehicle: dial(Mutex::new(connection).into(),
                      |connection| {
                          connection.lock().unwrap().recv().unwrap();
                          connection
                      },
        )?,
        header: MavHeader::default(),
    })
}

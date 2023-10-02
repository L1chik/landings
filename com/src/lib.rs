use std::sync::{Arc, mpsc, Mutex};
use std::thread::spawn;
use std::time::{Duration, Instant};

use anyhow::Result;
use mavlink::{
    ardupilotmega::{
        self,
        COMMAND_LONG_DATA,
        CopterMode::{COPTER_MODE_GUIDED, COPTER_MODE_LAND, COPTER_MODE_STABILIZE},
        MavCmd::{
            self,
            MAV_CMD_COMPONENT_ARM_DISARM,
            MAV_CMD_DO_SET_MODE,
            MAV_CMD_NAV_TAKEOFF,
        },
        MavResult::{self,
                    MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED,
                    MAV_RESULT_FAILED, MAV_RESULT_UNSUPPORTED,
        },
    },
    ardupilotmega::CopterMode,
    ardupilotmega::MavCmd::MAV_CMD_NAV_LAND,
    connect,
    MavConnection,
    MavHeader,
    MavlinkVersion,
};
pub use mavlink::{
    ardupilotmega::MavMessage,
    MavlinkVersion as Mavlink,
};
use num_traits::FromPrimitive;
use thiserror::Error;

use crate::MavError::{Failed, InvalidCommand, ModeNotReady, NoMavlinkDeviceFoundError,
                      SendError, UnsupportedCommand};

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
const WAIT_FOR_RESPONSE: Duration = Duration::from_secs(1);

#[derive(Error, Debug)]
pub enum MavError {
    #[error("Failed")] Failed(),
    #[error("No response")] NoResponseError(),
    #[error("Command rejected")] InvalidCommand(),
    #[error("Command unsupported")] UnsupportedCommand(),
    #[error("Couldn't enter the {target:?} mode: check parameters")] ModeNotReady {
        target: CopterMode,
        current: CopterMode,
    },
    #[error("Unable to communicate")] SendError(),
    #[error("No MAVLink device found (disconnected)")] NoMavlinkDeviceFoundError(),
}

pub struct GuidedCopter<'a> {
    interface: &'a MavlinkPi,
}

impl GuidedCopter<'_> {
    pub fn takeoff(&self, height: f32) -> MavResponse {
        self.interface.command(MAV_CMD_NAV_TAKEOFF,
                               &[0., 0., 0., 0., 0., 0., height])
    }

    pub fn land(&self) -> MavResponse {
        self.interface.command(MAV_CMD_NAV_LAND, &DEFAULT_PARAMS)
    }
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

    fn check_mode(&self, target: CopterMode, valid_modes: &[CopterMode]) -> Result<(), MavError> {
        loop {
            let (_, data) = self.receive().unwrap();
            return match data {
                MavMessage::HEARTBEAT(hb_data) => {
                    let current = CopterMode::from_u32(hb_data.custom_mode).unwrap();
                    if current == target || valid_modes.contains(&current) {
                        Ok(())
                    } else {
                        Err(ModeNotReady { target, current })
                    }
                }
                _ => continue,
            };
        }
    }

    pub fn enter_stabilize(&self) -> MavResponse {
        self.command(MAV_CMD_DO_SET_MODE,
                     &[1., COPTER_MODE_STABILIZE as i32 as f32, 0., 0., 0., 0., 0.])
    }

    pub fn enter_guided(&self) -> Result<GuidedCopter, MavError> {
        self.check_mode(COPTER_MODE_GUIDED, &[COPTER_MODE_STABILIZE, COPTER_MODE_LAND]).unwrap();
        self.command(MAV_CMD_DO_SET_MODE,
                     &[1., COPTER_MODE_GUIDED as i32 as f32, 0., 0., 0., 0., 0.]).unwrap();
        Ok(GuidedCopter { interface: self })
    }

    pub fn mode_land(&self) -> MavRequest {
        self.check_mode(COPTER_MODE_LAND, &[COPTER_MODE_STABILIZE, COPTER_MODE_GUIDED]).unwrap();
        self.command_unchecked(MAV_CMD_DO_SET_MODE,
                               &[1., COPTER_MODE_LAND as i32 as f32, 0., 0., 0., 0., 0.])
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
        self.command_unchecked(r#type, params)?;
        let start = Instant::now();
        loop {
            if start.elapsed() > WAIT_FOR_RESPONSE { return Ok(MavResult::default()); }
            let (_, resp) = self.receive()?;
            match resp {
                MavMessage::COMMAND_ACK(result) =>
                    if result.command == r#type {
                        return match result.result {
                            MAV_RESULT_ACCEPTED => Ok(result.result),
                            MAV_RESULT_DENIED => Err(InvalidCommand()),
                            MAV_RESULT_FAILED => Err(Failed()),
                            MAV_RESULT_UNSUPPORTED => Err(UnsupportedCommand()),
                            MavResult::MAV_RESULT_TEMPORARILY_REJECTED => continue,
                            MavResult::MAV_RESULT_IN_PROGRESS => continue,
                            MavResult::MAV_RESULT_CANCELLED => continue,
                        };
                    },
                _ => ()
            }
        }
    }

    fn command_unchecked(&self, r#type: MavCmd, params: &[f32]) -> MavRequest {
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
        self.send(data)
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
    log::info!("Establishing a MAVLink connection... ({TIMEOUT:?})");
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

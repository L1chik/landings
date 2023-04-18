use std::error::Error;
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use mavlink::*;
use mavlink::ardupilotmega::CopterMode::COPTER_MODE_GUIDED;
use mavlink::ardupilotmega::MavMessage;
use mavlink::error::MessageReadError;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Mavlink:|Begin");
    let loc = "serial:/dev/serial0:921600";
    let mut mavconn = connect::<MavMessage>(loc)?;
    mavconn.set_protocol_version(MavlinkVersion::V2);

    let vehicle = Arc::new(mavconn);
    vehicle
        .send(&MavHeader::default(), &request_parameters())
        .unwrap();
    vehicle
        .send(&MavHeader::default(), &request_stream())
        .unwrap();

    thread::spawn({
        let vehicle = vehicle.clone();
        move || loop {
            let res = vehicle.send_default(&guided());
            if res.is_ok() {
                thread::sleep(Duration::from_secs(1));
            } else {
                println!("send failed: {res:?}");
            }
        }
    });

    loop {
        match vehicle.recv() {
            Ok((_header, msg)) => {
                match msg {
                    MavMessage::MOUNT_CONFIGURE(_) => {}
                    MavMessage::FENCE_FETCH_POINT(_) => {}
                    MavMessage::GOPRO_SET_REQUEST(_) => {}
                    MavMessage::UAVIONIX_ADSB_OUT_CFG(_) => {}
                    MavMessage::SENSOR_OFFSETS(_) => {}
                    MavMessage::MISSION_CLEAR_ALL(_) => {}
                    MavMessage::GPS_GLOBAL_ORIGIN(_) => {}
                    MavMessage::MANUAL_SETPOINT(_) => {}
                    MavMessage::PARAM_MAP_RC(_) => {}
                    MavMessage::CAMERA_IMAGE_CAPTURED(_) => {}
                    MavMessage::DEVICE_OP_WRITE(_) => {}
                    MavMessage::PING(_) => {}
                    MavMessage::LOG_DATA(_) => {}
                    MavMessage::VIDEO_STREAM_STATUS(_) => {}
                    MavMessage::HIL_CONTROLS(_) => {}
                    MavMessage::ADAP_TUNING(_) => {}
                    MavMessage::RAW_PRESSURE(_) => {}
                    MavMessage::CURRENT_EVENT_SEQUENCE(_) => {}
                    MavMessage::LOG_REQUEST_LIST(_) => {}
                    MavMessage::GIMBAL_MANAGER_INFORMATION(_) => {}
                    MavMessage::OPEN_DRONE_ID_SYSTEM(_) => {}
                    MavMessage::CELLULAR_STATUS(_) => {}
                    MavMessage::CAN_FRAME(_) => {}
                    MavMessage::SCALED_IMU(imu) => {
                        println!("{imu:?}")
                    }
                    MavMessage::HIL_RC_INPUTS_RAW(_) => {}
                    MavMessage::AIS_VESSEL(_) => {}
                    MavMessage::SAFETY_ALLOWED_AREA(_) => {}
                    MavMessage::RAW_RPM(_) => {}
                    MavMessage::AHRS2(_) => {}
                    MavMessage::MISSION_REQUEST_LIST(_) => {}
                    MavMessage::DEVICE_OP_WRITE_REPLY(_) => {}
                    MavMessage::SCALED_IMU3(_) => {}
                    MavMessage::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(_) => {}
                    MavMessage::SERIAL_CONTROL(_) => {}
                    MavMessage::CAMERA_CAPTURE_STATUS(_) => {}
                    MavMessage::CAN_FILTER_MODIFY(_) => {}
                    MavMessage::VISION_SPEED_ESTIMATE(_) => {}
                    MavMessage::ORBIT_EXECUTION_STATUS(_) => {}
                    MavMessage::SAFETY_SET_ALLOWED_AREA(_) => {}
                    MavMessage::NAV_CONTROLLER_OUTPUT(_) => {}
                    MavMessage::CAMERA_STATUS(_) => {}
                    MavMessage::HERELINK_TELEM(_) => {}
                    MavMessage::UAVCAN_NODE_INFO(_) => {}
                    MavMessage::MAG_CAL_REPORT(_) => {}
                    MavMessage::TERRAIN_DATA(_) => {}
                    MavMessage::PARAM_REQUEST_READ(_) => {}
                    MavMessage::PLAY_TUNE_V2(_) => {}
                    MavMessage::GIMBAL_DEVICE_INFORMATION(_) => {}
                    MavMessage::V2_EXTENSION(_) => {}
                    MavMessage::CELLULAR_CONFIG(_) => {}
                    MavMessage::TRAJECTORY_REPRESENTATION_BEZIER(_) => {}
                    MavMessage::WHEEL_DISTANCE(_) => {}
                    MavMessage::SCALED_IMU2(_) => {}
                    MavMessage::CAMERA_INFORMATION(_) => {}
                    MavMessage::REQUEST_EVENT(_) => {}
                    MavMessage::CAMERA_TRACKING_IMAGE_STATUS(_) => {}
                    MavMessage::GIMBAL_MANAGER_SET_MANUAL_CONTROL(_) => {}
                    MavMessage::RESPONSE_EVENT_ERROR(_) => {}
                    MavMessage::OPTICAL_FLOW(_) => {}
                    MavMessage::REQUEST_DATA_STREAM(_) => {}
                    MavMessage::ATTITUDE_TARGET(_) => {}
                    MavMessage::ATTITUDE_QUATERNION(_) => {}
                    MavMessage::MISSION_REQUEST(_) => {}
                    MavMessage::CHANGE_OPERATOR_CONTROL_ACK(_) => {}
                    MavMessage::OBSTACLE_DISTANCE(_) => {}
                    MavMessage::PARAM_VALUE(_) => {}
                    MavMessage::GPS2_RTK(_) => {}
                    MavMessage::PID_TUNING(_) => {}
                    MavMessage::ESTIMATOR_STATUS(_) => {}
                    MavMessage::PLAY_TUNE(_) => {}
                    MavMessage::LOG_ERASE(_) => {}
                    MavMessage::ADSB_VEHICLE(_) => {}
                    MavMessage::MISSION_ACK(_) => {}
                    MavMessage::DATA_TRANSMISSION_HANDSHAKE(_) => {}
                    MavMessage::WIFI_CONFIG_AP(_) => {}
                    MavMessage::HIGH_LATENCY2(_) => {}
                    MavMessage::MISSION_ITEM_REACHED(_) => {}
                    MavMessage::ICAROUS_KINEMATIC_BANDS(_) => {}
                    MavMessage::CUBEPILOT_RAW_RC(_) => {}
                    MavMessage::LED_CONTROL(_) => {}
                    MavMessage::DEEPSTALL(_) => {}
                    MavMessage::MISSION_SET_CURRENT(_) => {}
                    MavMessage::EFI_STATUS(_) => {}
                    MavMessage::SETUP_SIGNING(_) => {}
                    MavMessage::VISION_POSITION_ESTIMATE(_) => {}
                    MavMessage::ESC_TELEMETRY_1_TO_4(_) => {}
                    MavMessage::GPS_RTCM_DATA(_) => {}
                    MavMessage::FENCE_STATUS(_) => {}
                    MavMessage::UAVCAN_NODE_STATUS(_) => {}
                    MavMessage::HIL_GPS(_) => {}
                    MavMessage::MISSION_CURRENT(_) => {}
                    MavMessage::RC_CHANNELS_OVERRIDE(_) => {}
                    MavMessage::CANFD_FRAME(_) => {}
                    MavMessage::LOGGING_DATA(_) => {}
                    MavMessage::OPEN_DRONE_ID_MESSAGE_PACK(_) => {}
                    MavMessage::EXTENDED_SYS_STATE(_) => {}
                    MavMessage::AUTH_KEY(_) => {}
                    MavMessage::ACTUATOR_CONTROL_TARGET(_) => {}
                    MavMessage::MOUNT_ORIENTATION(_) => {}
                    MavMessage::TERRAIN_REPORT(_) => {}
                    MavMessage::PARAM_REQUEST_LIST(_) => {}
                    MavMessage::GIMBAL_MANAGER_SET_PITCHYAW(_) => {}
                    MavMessage::OSD_PARAM_SHOW_CONFIG(_) => {}
                    MavMessage::BATTERY2(_) => {}
                    MavMessage::DEVICE_OP_READ_REPLY(_) => {}
                    MavMessage::SCALED_PRESSURE(_) => {}
                    MavMessage::RPM(_) => {}
                    MavMessage::GLOBAL_POSITION_INT(_) => {}
                    MavMessage::WIND_COV(_) => {}
                    MavMessage::CAMERA_FOV_STATUS(_) => {}
                    MavMessage::RADIO_STATUS(_) => {}
                    MavMessage::MEMORY_VECT(_) => {}
                    MavMessage::GLOBAL_VISION_POSITION_ESTIMATE(_) => {}
                    MavMessage::VFR_HUD(_) => {}
                    MavMessage::TIMESYNC(_) => {}
                    MavMessage::ESC_STATUS(_) => {}
                    MavMessage::DEVICE_OP_READ(_) => {}
                    MavMessage::SIMSTATE(_) => {}
                    MavMessage::VICON_POSITION_ESTIMATE(_) => {}
                    MavMessage::LOCAL_POSITION_NED_COV(_) => {}
                    MavMessage::OSD_PARAM_CONFIG_REPLY(_) => {}
                    MavMessage::RALLY_POINT(_) => {}
                    MavMessage::LOG_ENTRY(_) => {}
                    MavMessage::REMOTE_LOG_DATA_BLOCK(_) => {}
                    MavMessage::GOPRO_SET_RESPONSE(_) => {}
                    MavMessage::RESOURCE_REQUEST(_) => {}
                    MavMessage::AOA_SSA(_) => {}
                    MavMessage::OSD_PARAM_CONFIG(_) => {}
                    MavMessage::HIGH_LATENCY(_) => {}
                    MavMessage::PARAM_EXT_SET(_) => {}
                    MavMessage::MOUNT_CONTROL(_) => {}
                    MavMessage::RALLY_FETCH_POINT(_) => {}
                    MavMessage::AHRS3(_) => {}
                    MavMessage::DEBUG(_) => {}
                    MavMessage::HIL_STATE(_) => {}
                    MavMessage::SET_MAG_OFFSETS(_) => {}
                    MavMessage::BUTTON_CHANGE(_) => {}
                    MavMessage::DATA16(_) => {}
                    MavMessage::MISSION_ITEM_INT(_) => {}
                    MavMessage::ONBOARD_COMPUTER_STATUS(_) => {}
                    MavMessage::AHRS(_) => {}
                    MavMessage::RC_CHANNELS_SCALED(_) => {}
                    MavMessage::HIL_SENSOR(_) => {}
                    MavMessage::SERVO_OUTPUT_RAW(_) => {}
                    MavMessage::DIGICAM_CONFIGURE(_) => {}
                    MavMessage::UTM_GLOBAL_POSITION(_) => {}
                    MavMessage::COMMAND_ACK(_) => {}
                    MavMessage::GPS2_RAW(_) => {}
                    MavMessage::SIM_STATE(_) => {}
                    MavMessage::MISSION_ITEM(_) => {}
                    MavMessage::REMOTE_LOG_BLOCK_STATUS(_) => {}
                    MavMessage::FILE_TRANSFER_PROTOCOL(_) => {}
                    MavMessage::SUPPORTED_TUNES(_) => {}
                    MavMessage::AUTOPILOT_VERSION_REQUEST(_) => {}
                    MavMessage::POSITION_TARGET_GLOBAL_INT(_) => {}
                    MavMessage::HIL_ACTUATOR_CONTROLS(_) => {}
                    MavMessage::SET_POSITION_TARGET_LOCAL_NED(_) => {}
                    MavMessage::ACTUATOR_OUTPUT_STATUS(_) => {}
                    MavMessage::DISTANCE_SENSOR(_) => {}
                    MavMessage::GIMBAL_REPORT(_) => {}
                    MavMessage::OPEN_DRONE_ID_OPERATOR_ID(_) => {}
                    MavMessage::LANDING_TARGET(_) => {}
                    MavMessage::GPS_STATUS(_) => {}
                    MavMessage::GIMBAL_DEVICE_SET_ATTITUDE(_) => {}
                    MavMessage::GPS_INJECT_DATA(_) => {}
                    MavMessage::HYGROMETER_SENSOR(_) => {}
                    MavMessage::MEMINFO(_) => {}
                    MavMessage::TUNNEL(_) => {}
                    MavMessage::OBSTACLE_DISTANCE_3D(_) => {}
                    MavMessage::GPS_INPUT(_) => {}
                    MavMessage::ISBD_LINK_STATUS(_) => {}
                    MavMessage::LOGGING_ACK(_) => {}
                    MavMessage::CHANGE_OPERATOR_CONTROL(_) => {}
                    MavMessage::WIND(_) => {}
                    MavMessage::OPEN_DRONE_ID_AUTHENTICATION(_) => {}
                    MavMessage::ATTITUDE(_) => {}
                    MavMessage::CAMERA_FEEDBACK(_) => {}
                    MavMessage::DIGICAM_CONTROL(_) => {}
                    MavMessage::BATTERY_STATUS(_) => {}
                    MavMessage::PARAM_EXT_REQUEST_LIST(_) => {}
                    MavMessage::COMMAND_INT(_) => {}
                    MavMessage::SET_HOME_POSITION(_) => {}
                    MavMessage::EVENT(_) => {}
                    MavMessage::VIBRATION(_) => {}
                    MavMessage::GPS_RAW_INT(_) => {}
                    MavMessage::ODOMETRY(_) => {}
                    MavMessage::TRAJECTORY_REPRESENTATION_WAYPOINTS(_) => {}
                    MavMessage::MISSION_COUNT(_) => {}
                    MavMessage::GIMBAL_CONTROL(_) => {}
                    MavMessage::AUTOPILOT_STATE_FOR_GIMBAL_DEVICE(_) => {}
                    MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(_) => {}
                    MavMessage::SET_GPS_GLOBAL_ORIGIN(_) => {}
                    MavMessage::UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT(_) => {}
                    MavMessage::HWSTATUS(_) => {}
                    MavMessage::COMPASSMOT_STATUS(_) => {}
                    MavMessage::CUBEPILOT_FIRMWARE_UPDATE_START(_) => {}
                    MavMessage::CONTROL_SYSTEM_STATE(_) => {}
                    MavMessage::COMPONENT_METADATA(_) => {}
                    MavMessage::DEBUG_FLOAT_ARRAY(_) => {}
                    MavMessage::MESSAGE_INTERVAL(_) => {}
                    MavMessage::LOG_REQUEST_DATA(_) => {}
                    MavMessage::DATA_STREAM(_) => {}
                    MavMessage::POSITION_TARGET_LOCAL_NED(_) => {}
                    MavMessage::RADIO(_) => {}
                    MavMessage::ESC_TELEMETRY_9_TO_12(_) => {}
                    MavMessage::WATER_DEPTH(_) => {}
                    MavMessage::STATUSTEXT(_) => {}
                    MavMessage::VIDEO_STREAM_INFORMATION(_) => {}
                    MavMessage::SET_POSITION_TARGET_GLOBAL_INT(_) => {}
                    MavMessage::GOPRO_HEARTBEAT(_) => {}
                    MavMessage::GOPRO_GET_RESPONSE(_) => {}
                    MavMessage::HERELINK_VIDEO_STREAM_INFORMATION(_) => {}
                    MavMessage::MOUNT_STATUS(_) => {}
                    MavMessage::POWER_STATUS(_) => {}
                    MavMessage::ESC_INFO(_) => {}
                    MavMessage::FLIGHT_INFORMATION(_) => {}
                    MavMessage::UAVIONIX_ADSB_OUT_DYNAMIC(_) => {}
                    MavMessage::MAG_CAL_PROGRESS(_) => {}
                    MavMessage::RC_CHANNELS_RAW(_) => {}
                    MavMessage::AIRSPEED_AUTOCAL(_) => {}
                    MavMessage::AUTOPILOT_VERSION(_) => {}
                    MavMessage::COLLISION(_) => {}
                    MavMessage::DATA64(_) => {}
                    MavMessage::FOLLOW_TARGET(_) => {}
                    MavMessage::COMMAND_CANCEL(_) => {}
                    MavMessage::TERRAIN_CHECK(_) => {}
                    MavMessage::OPEN_DRONE_ID_SYSTEM_UPDATE(_) => {}
                    MavMessage::MISSION_REQUEST_INT(_) => {}
                    MavMessage::PARAM_EXT_REQUEST_READ(_) => {}
                    MavMessage::OPEN_DRONE_ID_BASIC_ID(_) => {}
                    MavMessage::STORAGE_INFORMATION(_) => {}
                    MavMessage::CAMERA_TRIGGER(_) => {}
                    MavMessage::LOG_REQUEST_END(_) => {}
                    MavMessage::SET_ACTUATOR_CONTROL_TARGET(_) => {}
                    MavMessage::VISION_POSITION_DELTA(_) => {}
                    MavMessage::ICAROUS_HEARTBEAT(_) => {}
                    MavMessage::SYS_STATUS(_) => {}
                    MavMessage::CAMERA_TRACKING_GEO_STATUS(_) => {}
                    MavMessage::GLOBAL_POSITION_INT_COV(_) => {}
                    MavMessage::OPTICAL_FLOW_RAD(_) => {}
                    MavMessage::GPS_RTK(_) => {}
                    MavMessage::ATT_POS_MOCAP(_) => {}
                    MavMessage::EKF_STATUS_REPORT(_) => {}
                    MavMessage::MANUAL_CONTROL(_) => {}
                    MavMessage::LOGGING_DATA_ACKED(_) => {}
                    MavMessage::PARAM_EXT_ACK(_) => {}
                    MavMessage::ALTITUDE(_) => {}
                    MavMessage::LOCAL_POSITION_NED(_) => {}
                    MavMessage::TIME_ESTIMATE_TO_TARGET(_) => {}
                    MavMessage::DEBUG_VECT(_) => {}
                    MavMessage::SET_MODE(_) => {}
                    MavMessage::WINCH_STATUS(_) => {}
                    MavMessage::LIMITS_STATUS(_) => {}
                    MavMessage::MCU_STATUS(_) => {}
                    MavMessage::MISSION_REQUEST_PARTIAL_LIST(_) => {}
                    MavMessage::OSD_PARAM_SHOW_CONFIG_REPLY(_) => {}
                    MavMessage::ESC_TELEMETRY_5_TO_8(_) => {}
                    MavMessage::GOPRO_GET_REQUEST(_) => {}
                    MavMessage::SYSTEM_TIME(_) => {}
                    MavMessage::TERRAIN_REQUEST(_) => {}
                    MavMessage::NAMED_VALUE_FLOAT(_) => {}
                    MavMessage::NAMED_VALUE_INT(_) => {}
                    MavMessage::OPEN_DRONE_ID_ARM_STATUS(_) => {}
                    MavMessage::CAMERA_SETTINGS(_) => {}
                    MavMessage::SMART_BATTERY_INFO(_) => {}
                    MavMessage::FENCE_POINT(_) => {}
                    MavMessage::DATA32(_) => {}
                    MavMessage::RC_CHANNELS(_) => {}
                    MavMessage::ATTITUDE_QUATERNION_COV(_) => {}
                    MavMessage::HOME_POSITION(_) => {}
                    MavMessage::RANGEFINDER(_) => {}
                    MavMessage::GIMBAL_TORQUE_CMD_REPORT(_) => {}
                    MavMessage::GIMBAL_MANAGER_SET_ATTITUDE(_) => {}
                    MavMessage::LINK_NODE_STATUS(_) => {}
                    MavMessage::SCALED_PRESSURE3(_) => {}
                    MavMessage::SET_ATTITUDE_TARGET(_) => {}
                    MavMessage::DATA96(_) => {}
                    MavMessage::PARAM_SET(_) => {}
                    MavMessage::RAW_IMU(imu_raw) => {
                        println!("{imu_raw:?}")
                    }
                    MavMessage::ENCAPSULATED_DATA(_) => {}
                    MavMessage::GIMBAL_MANAGER_STATUS(_) => {}
                    MavMessage::SCALED_PRESSURE2(_) => {}
                    MavMessage::PARAM_EXT_VALUE(_) => {}
                    MavMessage::HEARTBEAT(_) => {}
                    MavMessage::HIGHRES_IMU(_) => {}
                    MavMessage::HIL_STATE_QUATERNION(_) => {}
                    MavMessage::HIL_OPTICAL_FLOW(_) => {}
                    MavMessage::COMMAND_LONG(_) => {}
                    MavMessage::OPEN_DRONE_ID_SELF_ID(_) => {}
                    MavMessage::GENERATOR_STATUS(_) => {}
                    MavMessage::CUBEPILOT_FIRMWARE_UPDATE_RESP(_) => {}
                    MavMessage::MISSION_WRITE_PARTIAL_LIST(_) => {}
                    MavMessage::OPEN_DRONE_ID_LOCATION(_) => {}
                    MavMessage::AP_ADC(_) => {}
                    MavMessage::PROTOCOL_VERSION(_) => {}
                    MavMessage::COMPONENT_INFORMATION(_) => {}
                }
            }
            Err(MessageReadError::Io(e)) => {
                if let std::io::ErrorKind::WouldBlock = e.kind() {
                    thread::sleep(Duration::from_secs(1));
                    continue;
                } else {
                    println!("recv error: {e:?}");
                    break;
                }
            }
            _ => {}
        }
    }
    Ok(())
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
use std::error::Error;
use std::str::from_utf8;
use std::thread::sleep;
use std::time::Duration;

use com::*;

fn main_handler(msg: MavMessage) -> Result<(), Box<dyn Error>> {
    Ok(match msg {
        MavMessage::STATUSTEXT(status) =>
            eprintln!("{}", from_utf8(status.text.into_iter()
                .filter(|&c| c != 0).collect::<Vec<u8>>().as_slice())?),
        MavMessage::COMMAND_ACK(result) => println!("{:?}", result),
        MavMessage::HEARTBEAT(_) => (),
        _ => ()
    })
}

fn main() {
    match MavlinkPi::connect_sim(14550, Mavlink::V2) {
        Ok(vehicle) => {
            println!("\nConnected");
            let copter = vehicle.enter_guided().unwrap();
            sleep(Duration::from_millis(5));

            // Arm and takeoff
            println!("\nTakeoff");
            vehicle.arm().unwrap();
            sleep(Duration::from_secs(1));
            copter.takeoff(5.).unwrap();
            sleep(Duration::from_secs(7));

            // Landing
            println!("\nLanding");
            copter.land().unwrap(); // todo: resolve blocking here
            sleep(Duration::from_secs(7));

            loop {
                let (_, data) = vehicle.receive().unwrap();
                main_handler(data).unwrap();
                sleep(Duration::from_millis(10));
            }
        }
        Err(e) => { eprintln!("{e}") }
    }
}
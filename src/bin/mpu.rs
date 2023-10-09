use std::thread::sleep;
use std::time::Duration;
use linux_embedded_hal::{Delay, I2cdev};
use gy91::Mpu6050;

fn main() {
    let i2c = I2cdev::new("/dev/i2c-1")
        // .map_err(Mpu6050Error::I2c)
        .unwrap();

    let mut mpu = Mpu6050::new(i2c);

    let mut delay = Delay;

    mpu.init(&mut delay).unwrap();

    loop {
        println!("{}", mpu.get_acc().unwrap());
        sleep(Duration::from_millis(100));
    }
    // Ok(())
}
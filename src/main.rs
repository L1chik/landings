#[cfg(feature = "stream")]
use std::{sync, thread};
use std::sync::Arc;
use async_trait::async_trait;

use opencv::{
    calib3d::{
        draw_frame_axes,
        SOLVEPNP_IPPE_SQUARE,
        rodrigues,
        solve_pnp,
    },
    core::{self, Vec3f},
    imgproc::{arrowed_line, put_text},
    objdetect::{
        ArucoDetector,
        DetectorParameters,
        draw_detected_markers,
        get_predefined_dictionary,
        PredefinedDictionaryType::DICT_6X6_50,
        RefineParameters,
    },
    prelude::*,
    Result,
    types::{VectorOfi32, VectorOfVectorOfPoint2f},
    videoio,
    videoio::{CAP_PROP_FRAME_HEIGHT, CAP_PROP_FRAME_WIDTH},
};
use opencv::core::CV_32F;

#[cfg(feature = "viz")]
use opencv::highgui;

#[cfg(feature = "stream")]
use opencv::core::Vector;

#[cfg(feature = "stream")]
use opencv::imgcodecs::imencode;

#[cfg(feature = "pi")]
use rppal::uart::{Parity, Uart};

#[cfg(feature = "pi")]
use tokio::sync::mpsc;

use tokio::sync::{Mutex, oneshot};
use crate::msp::functions::MspOpticalFlow;

const M_LEN: f32 = 24.;
const M_ID: i32 = 44;

#[cfg(feature = "pi")]
const C_DST: &[f32] = &[0.22, -0.66, 0., 0., 0.66];

#[cfg(not(feature = "pi"))]
const C_DST: &[f32] = &[0., 0., 0., 0., 0.];

#[cfg(not(feature = "pi"))]
const C_MAT: &[f32] = &[383.571, 0., 326.973, 0., 383.571, 234.758, 0., 0., 1.];


#[cfg(feature = "pi")]
const C_MAT: &[f32] = &[447.5, 0., 325., 0., 447.5, 240., 0., 0., 1.];

fn estimate(
    frame: &Mat,
    xfr: &mut Mat,
    intrinsics: (&Mat, &Mat),
    detector: &ArucoDetector,
    draw_marker_info: bool,
) -> Result<Option<(f32, f32)>> {
    let mut ids = VectorOfi32::new();
    let mut cn = VectorOfVectorOfPoint2f::new();

    let ipp: [Vec3f; 4] = [
        [-M_LEN / 2., M_LEN / 2., 0.].into(),
        [M_LEN / 2., M_LEN / 2., 0.].into(),
        [M_LEN / 2., -M_LEN / 2., 0.].into(),
        [-M_LEN / 2., -M_LEN / 2., 0.].into(),
    ];

    let ipv = Mat::from_slice_rows_cols(&ipp, 4, 1)?;
    detector.detect_markers(frame, &mut cn, &mut ids, &mut core::no_array())?;

    if !ids.is_empty() && ids.get(0)? == M_ID {
        let (mut rod, mut trans) = (Vec3f::default(), Vec3f::default());
        solve_pnp(
            &ipv,
            &cn.get(0)?,
            intrinsics.0,
            intrinsics.1,
            &mut rod,
            &mut trans,
            false,
            SOLVEPNP_IPPE_SQUARE,
        )?;

        if draw_marker_info {
            draw_frame_axes(xfr, intrinsics.0, intrinsics.1, &rod, &trans, 10., 2)?;

            draw_detected_markers(xfr, &cn, &ids, (0., 1., 0.).into())?;
            arrowed_line(
                xfr,
                (320, 240).into(),
                (320 + trans[0] as i32, 240 + trans[1] as i32).into(),
                (33., 28., 50.).into(),
                4, 16, 0, 0.1,
            )?;

            let mut rot = Mat::new_rows_cols_with_default(3, 3, CV_32F, 0.into())?;

            rodrigues(&rod, &mut rot, &mut core::no_array())?;

            put_text(
                xfr,
                format!("x: {:.2} y: {:.2} dist: {:.2}", trans[0], trans[1], trans[2]).as_str(),
                (25, 35).into(),
                0,
                1.2,
                (0., 1., 0.).into(),
                1,
                16,
                false,
            )?;
        }
        // dbg!(trans);
        Ok(Some((trans[0], trans[1])))
    } else {
        Ok(None)
    }
}

mod stream;
mod msp;

#[allow(unused_mut)]
#[allow(unused_assignments)]
#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init_from_env(env_logger::Env::new().default_filter_or("info"));

    #[cfg(feature = "stream")]
    use actix_web::dev::ServerHandle;

    #[cfg(feature = "stream")]
    use actix_web::rt;

    #[cfg(feature = "stream")]
        let (input, frames) = sync::mpsc::sync_channel(25);

    #[cfg(feature = "stream")]
        let (serv_l, rx) = sync::mpsc::channel::<ServerHandle>();

    #[cfg(feature = "stream")] {
        use stream::broadcast::*;

        thread::spawn(move || {
            let future = start("0.0.0.0", 8091, frames, serv_l);
            rt::System::new().block_on(future)
        });
    }

    #[cfg(feature = "stream")]
        #[allow(unused_variables)]
        let serv_handle = rx.recv().unwrap();

    let mut cap_device = 4;
    let mut stream_api_preference = videoio::CAP_ANY;

    #[cfg(feature = "pi")]
    {
        stream_api_preference = videoio::CAP_V4L2;
        cap_device = 0;
    }

    let mut cam = videoio::VideoCapture::new(cap_device, stream_api_preference)?;
    cam.set(CAP_PROP_FRAME_WIDTH, 640.)?;
    cam.set(CAP_PROP_FRAME_HEIGHT, 480.)?;

    let mut frame = Mat::default();
    cam.retrieve(&mut frame, 0)?;

    let dictionary = get_predefined_dictionary(DICT_6X6_50)?;
    let parameters = DetectorParameters::default()?;
    let refinement = RefineParameters {
        min_rep_distance: 0.0,
        error_correction_rate: 0.0,
        check_all_orders: false,
    };

    let c_mat = Mat::from_slice_rows_cols(C_MAT, 3, 3)?;
    let c_dst = Mat::from_slice_rows_cols(C_DST, 1, 5)?;
    let detector = ArucoDetector::new(&dictionary, &parameters, refinement)?;

    let (cap_tx, mut _cap_rx) = oneshot::channel::<u8>();

    #[cfg(feature = "pi")]
        let (com_tx, mut com_rx) = oneshot::channel::<u8>();

    #[cfg(feature = "pi")]
        let (dtx, mut drx) = mpsc::channel::<(f32, f32)>(2);

    const LOST_COUNTER_INIT: u32 = 15;

    let _capture = tokio::spawn(async move {
        let mut last_valid = (0., 0.);

        let mut lost_counter = LOST_COUNTER_INIT;

        while cam.grab().unwrap() {
            cam.retrieve(&mut frame, 0).unwrap();

            // let mut viz = Mat::new_size_with_default(
            //     frame.size().unwrap(),
            //     CV_32FC3,
            //     (0.2, 0.15, 0.13).into(),
            // ).unwrap();

            let mut viz = frame.clone();

            let opt_xy = estimate(&frame, &mut viz, (&c_mat, &c_dst), &detector, true).unwrap();

            #[cfg(feature = "viz")]
            {
                highgui::imshow("Platform", &viz).unwrap();
                if highgui::wait_key(1).unwrap() == 'q' as i32 { break; }
            }

            #[cfg(feature = "stream")]
            {
                let mut img_buffer = Vector::new();
                if imencode(".jpg", &viz, &mut img_buffer, &Vector::new()).is_err() {
                    break;
                }
                input.send(img_buffer).unwrap();
            }

            if lost_counter == 0 {
                last_valid = (0., 0.);
                lost_counter = LOST_COUNTER_INIT;
            }

            let mut output = (0., 0.);

            if let Some((x, y)) = opt_xy {
                output = (x, y);
                last_valid = (x, y);
                lost_counter = LOST_COUNTER_INIT;
            } else if lost_counter > 0 && last_valid != (0., 0.) {
                log::info!("RESET in {}", lost_counter);
                lost_counter -= 1;
                output = last_valid;
            }

            #[cfg(not(feature = "pi"))]
            log::info!("{:?}", output);

            #[cfg(feature = "pi")]
            dtx.send(output).await.unwrap();
        }
        let _ = cap_tx.send(0);
    });

    #[cfg(feature = "pi")] {
        use msp::multiwii::*;

        struct RX(Arc<Mutex<Uart>>);
        struct TX(Arc<Mutex<Uart>>);

        let mut uart = Uart::new(115200, Parity::None, 8, 1).unwrap();

        uart.set_write_mode(false).unwrap();

        // impl<'a> MSPReceiver for RX<'a> {
        //     async fn receive(&mut self, buf: &mut [u8]) {
        //         self.0.lock().await.read(buf).unwrap();
        //     }
        // }

        #[async_trait]
        impl MSPSender for TX {
            async fn send(&mut self, packet: &[u8]) {
                self.0.lock().await.write(packet).unwrap();
            }
        }

        let mut com_if_rx = Arc::new(Mutex::new(
            Uart::new(115200, Parity::None, 8, 1).unwrap()
        ));

        let mut com_if_tx = Arc::clone(&com_if_rx);

        let msp_composer = MspComposer::new(0, 7938);

        tokio::spawn(async move {
            let mut msp: Msp<_, _, 25> = Msp::new(RX(com_if_rx), TX(com_if_tx));
            use std::time::Duration;

            let mut interval = tokio::time::interval(
                Duration::from_millis(((1. / 10.) * 1000.) as u64));
            while !com_tx.is_closed() {
                // let p = msp.receive();
                //
                // if let Ok(packet) = p {
                //     match Functions::from(packet.function) {
                //         Functions::RangeFinder => {}
                //         Functions::OpticalFlow => {
                //             let data: MspOpticalFlow = packet.payload.into();
                //
                //             println!("{:?}", data);
                //         }
                //         Functions::Other(_) => {}
                //     };
                // }

                let (mut x, mut y) = drx.try_recv().unwrap_or((0., 0.));

                x = if x < -10. { 2. } else if x > 10. { -2. } else { 0. };
                y = if y < -10. { 2. } else if y > 10. { -2. } else { 0. };

                if x != 0. && y != 0. {
                    log::info!("{} {}", x, y);
                }

                let of_packet = MspOpticalFlow::new(y as i32, x as i32, 150);

                let data: [u8; 9] = of_packet.into();

                msp.send(&msp_composer.set_payload(&data)).await;

                // uart.write(&[0xfe, 0x00, x[0], x[1], y[0], y[1], 0, 255, 0xaa]).unwrap();

                // uart.write(&[])

                interval.tick().await;
            }
            let _ = com_tx.send(0);
        });
    }

    #[cfg(feature = "pi")]
    tokio::select! {
        _val = _cap_rx => {},
        _val = com_rx => {},
    }

    #[cfg(not(feature = "pi"))]
    _capture.await.unwrap();

    // #[cfg(feature = "stream")]
    // rt::System::new().block_on(serv_handle.stop(true));

    Ok(())
}

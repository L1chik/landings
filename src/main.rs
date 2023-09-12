use std::{path::PathBuf, time::Instant};

use opencv::{
    calib3d::solve_pnp,
    calib3d::{draw_frame_axes, rodrigues, SOLVEPNP_IPPE_SQUARE},
    core::{self, Point3_, Vec3f, CV_32FC3},
    // highgui,
    imgproc::{arrowed_line, put_text},
    objdetect::{
        draw_detected_markers, get_predefined_dictionary, ArucoDetector, DetectorParameters,
        PredefinedDictionaryType::DICT_6X6_50, RefineParameters,
    },
    prelude::*,
    types::{VectorOfVectorOfPoint2f, VectorOfi32},
    videoio,
    videoio::{CAP_PROP_FRAME_HEIGHT, CAP_PROP_FRAME_WIDTH},
    Result,
};

#[cfg(feature = "pi")]
use rppal::uart::{Parity, Uart};

use video_rs::{Encoder, EncoderSettings, Locator, Time};

const M_LEN: f32 = 24.;
const M_ID: i32 = 44;

const CDST: &[f32] = &[0.22, -0.66, 0., 0., 0.66];
const CMAT: &[f32] = &[447.5, 0., 325., 0., 447.5, 240., 0., 0., 1.];

fn estimate(
    frame: &mut Mat,
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
        println!("{:?}", ids.get(0).unwrap());

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
            draw_frame_axes(frame, intrinsics.0, intrinsics.1, &rod, &trans, 10., 2)?;

            draw_detected_markers(frame, &cn, &ids, (0., 1., 0.).into())?;

            put_text(
                frame,
                format!("x: {:.2} y: {:.2} dist: {}", trans[0], trans[1], trans[2]).as_str(),
                (25, 35).into(),
                0,
                0.75,
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

fn frame_to_frame(frame: &Mat) -> ndarray::Array3<u8> {
    let size = Mat::size(frame).unwrap();

    let res = ndarray::Array3::from_shape_fn(
        (size.height as usize, size.width as usize, 3),
        |(height, width, channel)| {
            let p = *frame
                .at_2d::<Point3_<u8>>(height as i32, width as i32)
                .unwrap();
            match channel {
                0 => p.x,
                1 => p.y,
                2 => p.z,
                _ => panic!("Invalid channel"),
            }
        },
    );

    res
}

fn init_video_recordig(frame: &Mat) -> (Encoder, Time) {
    use std::time::SystemTime;
    let destination: Locator = PathBuf::from(format!("{:?}.mp4", SystemTime::now())).into();
    let settings = EncoderSettings::for_h264_yuv420p(
        frame.mat_size()[1] as usize,
        frame.mat_size()[0] as usize,
        true,
    );

    let encoder = Encoder::new(&destination, settings).expect("failed to create encoder");

    let position = Time::zero();

    (encoder, position)
}

fn main() -> Result<()> {
    let args = std::env::args().collect::<Vec<_>>();
    let draw_marker_info = args.iter().find(|&arg| arg == "--no-marker-info").is_none();

    #[cfg(feature = "pi")]
    let mut uart = Uart::new(19200, Parity::None, 8, 1).unwrap();
    #[cfg(feature = "pi")]
    uart.set_write_mode(true).unwrap();

    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_V4L2)?;
    cam.set(CAP_PROP_FRAME_WIDTH, 640.)?;
    cam.set(CAP_PROP_FRAME_HEIGHT, 480.)?;

    let mut frame = Mat::default();
    cam.retrieve(&mut frame, 0)?;
    println!("{:?}", frame.mat_size()[0]);

    video_rs::init().unwrap();

    let dictionary = get_predefined_dictionary(DICT_6X6_50)?;
    let parameters = DetectorParameters::default()?;
    let refinement = RefineParameters {
        min_rep_distance: 0.0,
        error_correction_rate: 0.0,
        check_all_orders: false,
    };

    let cmat = Mat::from_slice_rows_cols(CMAT, 3, 3)?;
    let cdst = Mat::from_slice_rows_cols(CDST, 1, 5)?;
    let detector = ArucoDetector::new(&dictionary, &parameters, refinement)?;

    let mut prev_x = 0.0;
    let mut prev_y = 0.0;

    let hz = std::time::Duration::from_secs_f32(0.04);
    const CXOF_PIXEL_SCALING: f32 = 1.76e-3;

    let (tx, rx) = std::sync::mpsc::sync_channel::<(Mat, Instant)>(1);

    std::thread::spawn(move || {
        let (frame, _) = rx.recv().unwrap();
        let mut video_start = Instant::now();
        let mut _empty_frame_count = 0;

        let (mut encoder, mut position) = init_video_recordig(&frame);
        loop {
            let (frame, _) = rx.recv().unwrap();
            encoder.encode(&frame_to_frame(&frame), &position).unwrap();
            // position = position.aligned_with(&frame_start.elapsed().into()).add();
            let duration: Time = Time::from_nth_of_a_second(25);
            position = position.aligned_with(&duration).add();

            if video_start.elapsed() > std::time::Duration::from_secs(10) {
                encoder.finish().unwrap();
                video_start = std::time::Instant::now();
                (encoder, position) = init_video_recordig(&frame);
            }
        }
    });

    loop {
        let frame_start = Instant::now();
        cam.grab()?;
        cam.retrieve(&mut frame, 0)?;

        let opt_xy = estimate(&mut frame, (&cmat, &cdst), &detector, draw_marker_info)?;

        tx.send((frame.clone(), frame_start)).unwrap();

        if let Some((x, y)) = opt_xy {
            // println!(
            //     "xm: {:.3}\nym: {:.3}\n\n",
            //     x - prev_x,
            //     y - prev_y,
            //     // start.elapsed().as_secs_f32()
            // );
            let x_to_send = ((x - prev_x) / CXOF_PIXEL_SCALING) as i16;
            let y_to_send = ((y - prev_y) / CXOF_PIXEL_SCALING) as i16;

            // [0xFE _ xml xmh yml ymh tm quality 0xAA]
            #[cfg(feature = "pi")]
            uart.write(&[
                0xFE,
                0,
                x_to_send.to_le_bytes()[0],
                x_to_send.to_le_bytes()[1],
                y_to_send.to_le_bytes()[0],
                y_to_send.to_le_bytes()[1],
                0,
                5,
                0xAA,
            ])
            .unwrap();

            println!("{}\n{}\n\n", x_to_send, y_to_send);

            prev_x = x;
            prev_y = y;
        }

        std::thread::sleep(hz.saturating_sub(frame_start.elapsed()));
        println!("{}", 1. / frame_start.elapsed().as_secs_f32());
    }
}

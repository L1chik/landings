use opencv::{
    calib3d::{draw_frame_axes, SOLVEPNP_IPPE_SQUARE},
    calib3d::rodrigues,
    calib3d::solve_pnp,
    core::{self, Vec3f},
    core::CV_32FC3,
    imgproc::arrowed_line,
    imgproc::put_text,
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

#[cfg(feature = "viz")]
use opencv::highgui;

#[cfg(feature = "live_recording")]
use video_rs::{Encoder, EncoderSettings, Locator, Time};

#[cfg(feature = "pi")]
use rppal::uart::{Parity, Uart};
use tokio::sync::{mpsc, oneshot};

const M_LEN: f32 = 24.;
const M_ID: i32 = 44;

const C_DST: &[f32] = &[0.22, -0.66, 0., 0., 0.66];
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
                (320 + 10 * trans[0] as i32, 240 + 10 * trans[1] as i32).into(),
                (33., 28., 50.).into(),
                1, 16, 0, 0.1,
            )?;

            let mut rot = Mat::new_rows_cols_with_default(3, 3, CV_32FC3, 0.into())?;

            rodrigues(&rod, &mut rot, &mut core::no_array())?;

            put_text(
                xfr,
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

#[cfg(feature = "live_recording")]
fn frame_to_frame(frame: &Mat) -> ndarray::Array3<u8> {
    let size = Mat::size(frame).unwrap();

    ndarray::Array3::from_shape_fn(
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
    )
}


#[cfg(feature = "live_recording")]
fn init_video_recording(frame: &Mat) -> (Encoder, Time) {
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

#[allow(unused_mut)]
#[allow(unused_assignments)]
#[tokio::main]
async fn main() -> Result<()> {
    #[cfg(feature = "pi")]
        let mut uart = Uart::new(19200, Parity::None, 8, 1).unwrap();

    let mut cap_device = 4;
    let mut stream_api_preference = videoio::CAP_ANY;

    #[cfg(feature = "pi")]
    {
        stream_api_preference = videoio::CAP_V4L2;
        cap_device = 0;
        uart.set_write_mode(true).unwrap();
    }

    let mut cam = videoio::VideoCapture::new(cap_device, stream_api_preference)?;
    cam.set(CAP_PROP_FRAME_WIDTH, 640.)?;
    cam.set(CAP_PROP_FRAME_HEIGHT, 480.)?;

    let mut frame = Mat::default();
    cam.retrieve(&mut frame, 0)?;

    #[cfg(feature = "video_recording")]
    video_rs::init().unwrap();

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

    #[cfg(feature = "live_recording")]
        let (tx, rx) = mpsc::sync_channel::<(Mat, Instant)>(1);

    #[cfg(feature = "live_recording")]
    std::thread::spawn(move || {
        let (frame, _) = rx.recv().unwrap();
        let mut video_start = Instant::now();
        let _empty_frame_count = 0;

        let (mut encoder, mut position) = init_video_recording(&frame);
        loop {
            let (frame, _) = rx.recv().unwrap();
            encoder.encode(&frame_to_frame(&frame), &position).unwrap();
            // position = position.aligned_with(&frame_start.elapsed().into()).add();
            let duration: Time = Time::from_nth_of_a_second(25);
            position = position.aligned_with(&duration).add();

            if video_start.elapsed() > std::time::Duration::from_secs(10) {
                encoder.finish().unwrap();
                video_start = Instant::now();
                (encoder, position) = init_video_recording(&frame);
            }
        }
    });

    let (com_tx, mut com_rx) = oneshot::channel::<u8>();
    let (cap_tx, mut cap_rx) = oneshot::channel::<u8>();
    let (dtx, mut drx) = mpsc::channel(2);

    tokio::spawn(async move {
        while cam.grab().unwrap() {
            cam.retrieve(&mut frame, 0).unwrap();

            let mut viz = Mat::new_size_with_default(
                frame.size().unwrap(),
                CV_32FC3,
                (0.2, 0.15, 0.13).into(),
            ).unwrap();

            let opt_xy = estimate(&frame, &mut viz, (&c_mat, &c_dst), &detector, true).unwrap();

            #[cfg(feature = "viz")]
            {
                highgui::imshow("Platform", &viz).unwrap();
                if highgui::wait_key(1).unwrap() == 'q' as i32 { break; }
            }

            #[cfg(feature = "live_recording")]
            tx.send((frame.clone(), frame_start)).unwrap();

            if let Some((x, y)) = opt_xy {
                dtx.send((x, y)).await.unwrap();
            } else {
                dtx.send((0., 0.)).await.unwrap();
            }
        }
        let _ = cap_tx.send(0);
    });

    #[cfg(feature = "pi")]
    tokio::spawn(async move {
        use std::time::Duration;

        let mut interval = tokio::time::interval(
            Duration::from_millis(((1. / 25.) * 1000.) as u64));
        while !com_tx.is_closed() {
            let (x, y) = drx.try_recv().unwrap_or((0., 0.));
            println!("{} {}", x, y);

            let x = (x as i16).to_le_bytes();
            let y = (y as i16).to_le_bytes();

            uart.write(&[0xfe, 0x00, x[0], x[1], y[0], y[1], 0, 199, 0xaa]).unwrap();

            interval.tick().await;
        }
        let _ = com_tx.send(0);
    });

    tokio::select! {
        _val = cap_rx => {},
        _val = com_rx => {},
    }

    Ok(())
}

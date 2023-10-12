use anyhow::{ensure, Result};
use realsense_rust::{
    config::Config,
    context::Context,
    kind::{
        Rs2CameraInfo,
        Rs2Format,
        Rs2ProductLine,
        Rs2StreamKind,
        Rs2Option,
    },
    pipeline::InactivePipeline,
    frame::{AccelFrame, ColorFrame, InfraredFrame, PixelKind},
};
use std::{
    collections::HashSet,
    convert::TryFrom,
    time::Duration,
};
use std::f32::consts::PI;
use opencv::{
    calib3d::{draw_frame_axes, rodrigues, solve_pnp, SOLVEPNP_IPPE_SQUARE},
    core::{self, CV_32F, Mat, Vec3f},
    highgui::{imshow, wait_key},
    imgproc::{arrowed_line, put_text},
    objdetect::{
        ArucoDetector,
        DetectorParameters,
        draw_detected_markers,
        get_predefined_dictionary,
        RefineParameters,
        PredefinedDictionaryType::DICT_6X6_50,
    },
    prelude::*,
    types::{VectorOfi32, VectorOfVectorOfPoint2f},
};

const TO_DEG: f32 = 180. / PI;

const C_DST: &[f32] = &[0.18, -0.56, 0., 0., 0.5];

const CPR_X: f32 = 323.5;
const CPR_Y: f32 = 186.5;

const C_MAT: &[f32] = &[433.5, 0., CPR_X, 0., 433.5, CPR_Y, 0., 0., 1.];

const M_LEN: f32 = 24.;

const M_ID: i32 = 44;

fn estimate(
    frame: &Mat,
    xfr: &mut Mat,
    intrinsics: (&Mat, &Mat),
    detector: &ArucoDetector,
    tilt: &[f32],
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

    #[cfg(feature = "viz")]
    put_text(
        xfr,
        format!("[ {:>5.2} {:>5.2} {:>5.2} ]", tilt[0], tilt[1], tilt[2]).as_str(),
        (25, 80).into(),
        0,
        1.2,
        (0., 255., 0.).into(),
        4,
        16,
        false,
    )?;

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

        trans.iter_mut().for_each(|t| *t /= 5.);

        #[cfg(feature = "viz")]
        if draw_marker_info {
            draw_frame_axes(xfr, intrinsics.0, intrinsics.1, &rod, &trans, 10., 2)?;

            draw_detected_markers(xfr, &cn, &ids, (0., 1., 0.).into())?;
            arrowed_line(
                xfr,
                (CPR_X as i32, CPR_Y as i32).into(),
                ((CPR_X + trans[0]) as i32, (CPR_Y + trans[1]) as i32).into(),
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
                (0., 255., 0.).into(),
                4,
                16,
                false,
            )?;
        }

        let b = trans[2] * trans[2];

        let a = trans[1];
        let beta = tilt[1] / TO_DEG;
        let sq_l_r = a * beta.sin() + trans[2] * beta.cos() * ((b - a * a) / b).sqrt();

        let g = trans[0];
        let beta = tilt[0] / TO_DEG;
        let sq_l_d = g * beta.sin() + trans[2] * beta.cos() * ((b - g * g) / b).sqrt();

        let m2 = (b - sq_l_r * sq_l_r).sqrt();

        let m1 = (b - sq_l_d * sq_l_d).sqrt();

        #[cfg(feature = "viz")]
        put_text(
            xfr,
            format!("x: {:.2} y: {:.2} dist: {:.2}", m1, m2, trans[2]).as_str(),
            (25, 120).into(),
            0,
            1.2,
            (0., 255., 0.).into(),
            4,
            16,
            false,
        )?;

        Ok(Some((m1, m2)))
    } else {
        Ok(None)
    }
}

fn ocv_mat_from_color(color_frame: &ColorFrame) -> Mat {
    let mut color_mat = Mat::new_rows_cols_with_default(
        color_frame.height() as i32,
        color_frame.width() as i32,
        core::CV_8UC3,
        0.into(),
    ).unwrap();

    for (i, rs) in color_frame.iter().enumerate() {
        match rs {
            PixelKind::Bgr8 { b, g, r } => {
                *color_mat.at_mut::<core::Vec3b>(i as i32).unwrap() = [*b, *g, *r].into();
            }
            _ => panic!("We got our types wrong!"),
        }
    }

    color_mat
}


fn _ocv_mat_from_ir(ir_frame: &InfraredFrame) -> Mat {
    let mut ir_mat = Mat::new_rows_cols_with_default(
        ir_frame.height() as i32,
        ir_frame.width() as i32,
        core::CV_8UC3,
        0.into(),
    ).unwrap();

    for (i, rs) in ir_frame.iter().enumerate() {
        match rs {
            PixelKind::Y8 { y } => {
                *ir_mat.at_mut::<core::Vec3b>(i as i32).unwrap() = [*y, *y, *y].into();
            }
            _ => panic!("We got our types wrong!"),
        }
    }

    ir_mat
}

pub fn main() -> Result<()> {
    let mut queried_devices = HashSet::new();
    queried_devices.insert(Rs2ProductLine::D400);
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);

    ensure!(!devices.is_empty(), "No RealSense devices found");

    let mut ix = 0;

    for sensor in devices[0].sensors().iter_mut() {
        if sensor.supports_info(Rs2CameraInfo::Name) {
            ix += 1;
            if ix == 1 {
                sensor.set_option(Rs2Option::EnableAutoExposure, 1.)?;
                sensor.set_option(Rs2Option::AutoExposureLimit, 5000.)?;
                sensor.set_option(Rs2Option::EmitterEnabled, 0.)?
            } else if ix == 2 {
                sensor.set_option(Rs2Option::Exposure, 200.)?;
                // sensor.set_option(Rs2Option::Saturation, 1000.)?;
                // sensor.set_option(Rs2Option::EnableAutoExposure, 1.)?;
                // sensor.set_option(Rs2Option::Exposure, 7000.)?;
            }
        }
    }

    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();

    let usb_cstr = devices[0].info(Rs2CameraInfo::UsbTypeDescriptor).unwrap();
    let usb_val: f32 = usb_cstr.to_str().unwrap().parse().unwrap();
    if usb_val >= 3.0 {
        config
            .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
            .disable_all_streams()?
            // .enable_stream(Rs2StreamKind::Infrared, Some(1), 640, 480, Rs2Format::Y8, 60)?
            .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Rgb8, 60)?
            // .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?
            .enable_stream(Rs2StreamKind::Accel, None, 0, 0, Rs2Format::Any, 0)?;
        // .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 60)?
    } else {
        config
            .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
            .disable_all_streams()?
            .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?
            .enable_stream(Rs2StreamKind::Accel, None, 0, 0, Rs2Format::Any, 0)?;
    }

    let mut pipeline = pipeline.start(Some(config))?;
    let mut tilt: [f32; 3];

    let mut frame: Mat;

    let timeout = Duration::from_millis(10000);

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

    loop {
        let frames = pipeline.wait(Some(timeout))?;

        let ref_frames = frames.frames_of_type::<AccelFrame>();
        let color_frames = frames.frames_of_type::<ColorFrame>();
        // let ir_frames = frames.frames_of_type::<InfraredFrame>();

        if !color_frames.is_empty() {
            tilt = *ref_frames[0].acceleration();
            // frame = ocv_mat_from_ir(&ir_frames[0]);
            frame = ocv_mat_from_color(&color_frames[0]);

            let mut viz = frame.clone();

            if !ref_frames.is_empty() {
                let nr = (tilt[0] * tilt[0] + tilt[1] * tilt[1] + tilt[2] * tilt[2]).sqrt();

                tilt[0] = (-tilt[0] / nr).asin() * TO_DEG;
                tilt[1] = (-tilt[1] / nr).asin() * TO_DEG;
                tilt[2] = (tilt[2] / nr).acos() * TO_DEG - 180.;
                estimate(&frame, &mut viz, (&c_mat, &c_dst), &detector, &tilt, true)?;
            } else {
                estimate(&frame, &mut viz, (&c_mat, &c_dst), &detector, &[0., 0., 0.], true)?;
            };

            imshow("", &viz)?;
        }

        if wait_key(1)? != -1 { break; }
    }

    Ok(())
}

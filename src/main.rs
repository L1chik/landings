use std::time::Instant;

use opencv::{calib3d::solve_pnp,
             prelude::*, Result,
             core::{self, Vec3f, CV_32FC3},
             types::{VectorOfi32, VectorOfVectorOfPoint2f},
             imgproc::{arrowed_line, put_text},
             calib3d::{draw_frame_axes, rodrigues, SOLVEPNP_IPPE_SQUARE},
             objdetect::{ArucoDetector, DetectorParameters, RefineParameters,
                         get_predefined_dictionary, draw_detected_markers,
                         PredefinedDictionaryType::DICT_6X6_50},
             videoio, videoio::{CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT},
    highgui
};

const M_LEN: f32 = 24.;
const M_ID: i32 = 44;

const CDST: &[f32] = &[0.22, -0.66, 0., 0., 0.66];
const CMAT: &[f32] = &[447.5, 0., 325., 0., 447.5, 240., 0., 0., 1.];

fn estimate(
    frame: &Mat,
    xfr: &mut Mat,
    intrinsics: (&Mat, &Mat),
    detector: &ArucoDetector,
) -> Result<()> {
    let mut ids = VectorOfi32::new();
    let mut cn = VectorOfVectorOfPoint2f::new();

    let ipp: [Vec3f; 4] = [
        [-M_LEN / 2., M_LEN / 2., 0.].into(),
        [M_LEN / 2., M_LEN / 2., 0.].into(),
        [M_LEN / 2., -M_LEN / 2., 0.].into(),
        [-M_LEN / 2., -M_LEN / 2., 0.].into(),
    ];

    let ipv = Mat::from_slice_rows_cols(&ipp, 4, 1)?;
    detector.detect_markers(&frame, &mut cn, &mut ids, &mut core::no_array())?;

    if !ids.is_empty() && ids.get(0)? == M_ID {
        println!("{:?}", ids.get(0).unwrap());

        let (mut rod, mut trans) = (Vec3f::default(), Vec3f::default());
        solve_pnp(
            &ipv,
            &cn.get(0)?,
            intrinsics.0, intrinsics.1,
            &mut rod, &mut trans,
            false,
            SOLVEPNP_IPPE_SQUARE,
        )?;

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
        dbg!(trans);
    }
    Ok(())
}

fn main() -> Result<()> {
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_V4L2)?;
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

    let cmat = Mat::from_slice_rows_cols(CMAT, 3, 3)?;
    let cdst = Mat::from_slice_rows_cols(CDST, 1, 5)?;
    let detector = ArucoDetector::new(&dictionary, &parameters, refinement)?;

    while cam.grab()? {
        let start = Instant::now();
        cam.retrieve(&mut frame, 0)?;

        let mut res = Mat::new_size_with_default(
            frame.size()?,
            CV_32FC3,
            (0.2, 0.15, 0.13).into(),
        )?;

        estimate(&mut frame, &mut res, (&cmat, &cdst), &detector)?;

        let duration = start.elapsed();
        println!("Time elapsed : {:?}", duration);
    }
    Ok(())
}
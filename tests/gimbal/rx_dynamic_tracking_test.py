#!/usr/bin/env python3
"""UTC 동기 없이 Rx 짐벌 동적 추적과 색상 기록을 점검한다."""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.dynamic_tracking.common import (
    ALIGNMENT_PERIOD_NS,
    ALIGNMENT_PERIOD_SEC,
    LatestUwbReceiver,
    calculate_alignment,
    distance_trajectory,
)
from experiment.dynamic_tracking.rx_dynamic_tracking import (
    RX_FIELDS,
    RxResultWriter,
    failure_frame_path,
    iso_now,
)


EXPERIMENT_CODE = "gimbal_rx_dynamic_tracking_test"


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Start the Rx gimbal immediately, track UWB at 0.2-second "
            "intervals, and record red detection until Ctrl+C."
        )
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--servo-channel", type=int, default=0)
    parser.add_argument(
        "--pca9685-address",
        type=lambda value: int(value, 0),
        default=0x40,
    )
    parser.add_argument("--initial-deg", type=float, default=0.0)
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--crop-scale", type=float, default=0.6)
    parser.add_argument("--camera-warmup", type=float, default=5.0)
    parser.add_argument(
        "--target-color",
        choices=("red", "orange", "yellow", "green", "blue", "purple"),
        default="red",
    )
    parser.add_argument("--color-min-area", type=float, default=125.0)
    parser.add_argument(
        "--color-min-component-area",
        type=float,
        default=50.0,
    )
    parser.add_argument(
        "--save-failure-frames",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--live-stream", action="store_true")
    parser.add_argument(
        "--experiment-id",
        help="result directory ID (default: generated from the current time)",
    )
    parser.add_argument(
        "--distance",
        type=int,
        choices=(0, 1, 2, 3),
        default=0,
        help="value recorded in distance_m (default: 0)",
    )
    return parser


def validate_args(parser, args):
    if not 0 <= args.servo_channel < 16:
        parser.error("--servo-channel must be between 0 and 15")
    if not 0 <= args.pca9685_address <= 0x7F:
        parser.error("--pca9685-address must be a 7-bit I2C address")
    if not -90 <= args.initial_deg <= 90:
        parser.error("--initial-deg must be between -90 and 90")
    if not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.camera_warmup < 0:
        parser.error("--camera-warmup must be 0 or greater")
    if args.color_min_area < 0 or args.color_min_component_area < 0:
        parser.error("color area thresholds must be 0 or greater")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    import cv2
    from camera.realsense_scanner import HardwareScanner
    from gimbal.gimbal_controller_yaw import GimbalController
    from logger.result_logger import ResultLogger

    experiment_id = args.experiment_id
    if experiment_id is None:
        experiment_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")

    gimbal = None
    scanner = None
    uwb = None
    logger = None
    result_writer = None
    camera_started = False
    sample_index = 0

    try:
        gimbal = GimbalController(
            servo_channel=args.servo_channel,
            pca9685_address=args.pca9685_address,
        )
        gimbal.move_to(args.initial_deg)
        scanner = HardwareScanner(
            device_index=args.device_index,
            crop_scale=args.crop_scale,
            live_stream=args.live_stream,
            color_min_area_px=args.color_min_area,
            color_min_component_area_px=args.color_min_component_area,
            target_color=args.target_color,
        )
        print(
            f"[CAMERA] starting capture; color warmup={args.camera_warmup:g}s"
        )
        if not scanner.start_capture(warmup_sec=args.camera_warmup):
            raise RuntimeError(f"failed to open /dev/video{args.device_index}")
        camera_started = True

        uwb = LatestUwbReceiver(args.host, args.port, reuse_latest=True)
        uwb.start()
        logger = ResultLogger(
            target_dir_name="result",
            experiment_code=EXPERIMENT_CODE,
            experiment_id=experiment_id,
            node_id="rx",
            fieldnames=RX_FIELDS,
        )
        result_writer = RxResultWriter(logger, cv2)
        print(f"[LOG] {logger.csv_path}")
        print(
            "[START] Immediate Rx tracking test started; "
            f"target_color={args.target_color}; press Ctrl+C to stop"
        )

        test_start_ns = time.monotonic_ns()
        while True:
            scheduled_ns = (
                test_start_ns + sample_index * ALIGNMENT_PERIOD_NS
            )
            next_scheduled_ns = scheduled_ns + ALIGNMENT_PERIOD_NS
            remaining_ns = scheduled_ns - time.monotonic_ns()
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1_000_000_000)
            result_writer.raise_if_failed()

            row = {field: "" for field in RX_FIELDS}
            row.update(
                {
                    "sample_index": sample_index,
                    "distance_m": args.distance,
                    "trajectory_mode": distance_trajectory(args.distance),
                    "alignment_period_s": ALIGNMENT_PERIOD_SEC,
                    "scheduled_elapsed_s": (
                        sample_index * ALIGNMENT_PERIOD_SEC
                    ),
                    "target_color": args.target_color,
                    "color_success": 0,
                    "status": "error",
                    "started_at": iso_now(),
                }
            )
            failure_image = None

            try:
                uwb_selection = uwb.take_latest_after(test_start_ns)
                if uwb_selection is None:
                    row["status"] = "uwb_unavailable"
                else:
                    uwb_sample = uwb_selection.sample
                    previous_deg = gimbal.current_degree
                    alignment = calculate_alignment(
                        previous_deg,
                        uwb_sample.raw_azimuth_deg,
                    )
                    baseline = scanner.get_latest_frame()
                    baseline_frame_id = (
                        baseline.frame_id if baseline is not None else 0
                    )
                    commanded_ns = time.monotonic_ns()
                    applied_command = gimbal.move_to(
                        alignment.gimbal_command_ros_deg
                    )
                    row.update(
                        {
                            "command_elapsed_s": (
                                commanded_ns - test_start_ns
                            )
                            / 1_000_000_000,
                            "previous_gimbal_ros_deg": previous_deg,
                            "uwb_source": uwb_sample.source,
                            "uwb_received_monotonic_ns": (
                                uwb_sample.received_monotonic_ns
                            ),
                            "uwb_packet_reused": int(uwb_selection.reused),
                            "uwb_raw_azimuth_deg": (
                                uwb_sample.raw_azimuth_deg
                            ),
                            "uwb_ros_azimuth_deg": (
                                alignment.uwb_ros_azimuth_deg
                            ),
                            "correction_ros_deg": (
                                alignment.correction_ros_deg
                            ),
                            "target_calculated_ros_deg": (
                                alignment.target_calculated_ros_deg
                            ),
                            "gimbal_command_ros_deg": applied_command,
                            "servo_clipped": int(alignment.servo_clipped),
                        }
                    )

                    color_result = scanner.detect_color_presence_until(
                        next_scheduled_ns,
                        target_color=args.target_color,
                        after_frame_id=baseline_frame_id,
                        after_captured_ns=commanded_ns,
                    )
                    detection_completed_ns = time.monotonic_ns()
                    got_new_frame = (
                        color_result.frame_id is not None
                        and color_result.frame_id > baseline_frame_id
                        and color_result.captured_ns is not None
                        and commanded_ns
                        <= color_result.captured_ns
                        <= next_scheduled_ns
                    )
                    color_success = bool(
                        got_new_frame and color_result.visible
                    )
                    if color_success:
                        status = "success"
                    elif not got_new_frame:
                        status = "camera_frame_timeout"
                    else:
                        status = "color_not_detected"

                    failure_frame = ""
                    if not color_success and args.save_failure_frames:
                        failure_frame = failure_frame_path(
                            sample_index,
                            status,
                            color_result,
                        )
                        failure_image = color_result.frame
                    row.update(
                        {
                            "color_visible": (
                                int(color_result.visible)
                                if got_new_frame
                                else ""
                            ),
                            "color_success": int(color_success),
                            "color_recognition_time_ms": (
                                (
                                    detection_completed_ns - commanded_ns
                                )
                                / 1_000_000
                                if color_success
                                else ""
                            ),
                            "camera_frame_id": (
                                color_result.frame_id
                                if color_result.frame_id is not None
                                else ""
                            ),
                            "camera_captured_ns": (
                                color_result.captured_ns
                                if color_result.captured_ns is not None
                                else ""
                            ),
                            "failure_frame": failure_frame,
                            "status": status,
                        }
                    )
            except Exception as exc:
                row["status"] = "error"
                row["error_message"] = f"{type(exc).__name__}: {exc}"
                print(f"[WARN] sample {sample_index}: {row['error_message']}")
            finally:
                row["finished_at"] = iso_now()
                result_writer.submit(row, failure_frame=failure_image)

            sample_index += 1

    except KeyboardInterrupt:
        print("\n[STOP] manual stop requested")
    finally:
        try:
            if result_writer is not None:
                result_writer.close()
            elif logger is not None:
                logger.close()
        except Exception as exc:
            print(f"[WARN] failed to finish result writer cleanly: {exc}")
        if uwb is not None:
            uwb.close()
        if camera_started:
            scanner.stop()
        if gimbal is not None:
            try:
                gimbal.move_to(0.0)
                time.sleep(gimbal.ALIGN_INTERVAL_SEC)
            finally:
                gimbal.cleanup()
        print(f"[DONE] Rx test samples recorded: {sample_index}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Chrony 동기 시작으로 Rx 짐벌의 UWB 추적과 색상 인식을 반복한다."""

import argparse
import math
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.dynamic_tracking.common import (
    ALIGNMENT_PERIOD_NS,
    ALIGNMENT_PERIOD_SEC,
    LatestUwbReceiver,
    calibration_deadline_monotonic_ns,
    calculate_alignment,
    distance_trajectory,
)
from time_sync.chrony_clock import (
    format_utc_epoch_ns,
    parse_utc_epoch_ns,
    sleep_until_monotonic_ns,
    wait_for_chrony_sync,
    wait_until_utc_ns,
)


RX_FIELDS = (
    "experiment_id",
    "node_id",
    "sample_index",
    "distance_m",
    "trajectory_mode",
    "alignment_period_s",
    "scheduled_elapsed_s",
    "command_elapsed_s",
    "previous_gimbal_ros_deg",
    "uwb_source",
    "uwb_received_monotonic_ns",
    "uwb_packet_reused",
    "uwb_calibration_sample_count",
    "uwb_calibration_bias_deg",
    "uwb_calibration_std_deg",
    "uwb_raw_azimuth_deg",
    "uwb_corrected_azimuth_deg",
    "uwb_ros_azimuth_deg",
    "correction_ros_deg",
    "target_calculated_ros_deg",
    "gimbal_command_ros_deg",
    "servo_clipped",
    "target_color",
    "color_visible",
    "color_success",
    "color_recognition_time_ms",
    "camera_frame_id",
    "camera_captured_ns",
    "failure_frame",
    "status",
    "started_at",
    "finished_at",
    "error_message",
)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Rx dynamic tracking: align from the latest UWB packet every "
            "0.2 seconds and detect color until the next alignment."
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
    parser.add_argument(
        "--distance",
        type=int,
        choices=(0, 1, 2, 3),
        required=True,
        help=(
            "distance condition: 1/2/3m fixed-radius arc, "
            "0=random moving rover"
        ),
    )
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument(
        "--start-utc",
        type=parse_utc_epoch_ns,
        required=True,
        metavar="EPOCH_SEC",
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument(
        "--uwb-calibration",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="collect a stationary UWB zero-bias calibration before start",
    )
    parser.add_argument(
        "--uwb-calibration-samples",
        type=int,
        default=100,
    )
    parser.add_argument(
        "--uwb-calibration-max-std-deg",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--uwb-calibration-max-bias-deg",
        type=float,
        default=20.0,
    )
    parser.add_argument(
        "--uwb-calibration-margin-sec",
        type=float,
        default=1.0,
        help="finish calibration this many seconds before --start-utc",
    )
    parser.add_argument("--crop-scale", type=float, default=0.6)
    parser.add_argument(
        "--camera-warmup",
        type=float,
        default=5.0,
        help="camera color stabilization before shared start (default: 5.0)",
    )
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
        "--chrony-max-correction-sec",
        type=float,
        default=0.005,
    )
    parser.add_argument("--chrony-wait-tries", type=int, default=60)
    return parser


def validate_args(parser, args):
    if not -90 <= args.initial_deg <= 90:
        parser.error("--initial-deg must be between -90 and 90")
    if not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.camera_warmup < 0:
        parser.error("--camera-warmup must be 0 or greater")
    if args.uwb_calibration_samples <= 0:
        parser.error("--uwb-calibration-samples must be greater than 0")
    if (
        not math.isfinite(args.uwb_calibration_max_std_deg)
        or args.uwb_calibration_max_std_deg <= 0
    ):
        parser.error("--uwb-calibration-max-std-deg must be greater than 0")
    if (
        not math.isfinite(args.uwb_calibration_max_bias_deg)
        or not 0 < args.uwb_calibration_max_bias_deg <= 180
    ):
        parser.error(
            "--uwb-calibration-max-bias-deg must be greater than 0 "
            "and at most 180"
        )
    if (
        not math.isfinite(args.uwb_calibration_margin_sec)
        or args.uwb_calibration_margin_sec < 0
    ):
        parser.error("--uwb-calibration-margin-sec must be 0 or greater")
    if args.color_min_area < 0 or args.color_min_component_area < 0:
        parser.error("color area thresholds must be 0 or greater")
    if args.chrony_max_correction_sec <= 0:
        parser.error("--chrony-max-correction-sec must be greater than 0")
    if args.chrony_wait_tries <= 0:
        parser.error("--chrony-wait-tries must be greater than 0")


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


class RxResultWriter:
    """다음 0.2초 정렬을 막지 않도록 CSV와 실패 영상을 비동기로 저장한다."""

    def __init__(self, logger, cv2):
        self.logger = logger
        self.cv2 = cv2
        self.queue = queue.Queue()
        self.error = None
        self.thread = threading.Thread(
            target=self._run,
            name="rx-dynamic-result-writer",
            daemon=True,
        )
        self.thread.start()

    def submit(self, row, failure_frame=None):
        self.raise_if_failed()
        self.queue.put_nowait((dict(row), failure_frame))

    def _run(self):
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                row, failure_frame = item
                relative_path = row.get("failure_frame", "")
                if relative_path and failure_frame is not None:
                    absolute_path = (
                        Path(self.logger.experiment_dir) / relative_path
                    )
                    absolute_path.parent.mkdir(parents=True, exist_ok=True)
                    if not self.cv2.imwrite(
                        str(absolute_path),
                        failure_frame,
                    ):
                        raise RuntimeError(
                            "failed to save color failure frame: "
                            f"{absolute_path}"
                        )
                self.logger.log_sample(row)
                print(
                    f"[RX] sample={row['sample_index']} "
                    f"status={row['status']} "
                    f"uwb_raw={row['uwb_raw_azimuth_deg']} "
                    f"command={row['gimbal_command_ros_deg']} "
                    f"color_success={row['color_success']}"
                )
            except Exception as exc:
                self.error = exc
                return
            finally:
                self.queue.task_done()

    def raise_if_failed(self):
        if self.error is not None:
            raise RuntimeError(
                f"background result writer failed: {self.error}"
            ) from self.error

    def close(self):
        try:
            if self.thread.is_alive():
                self.queue.put(None)
                self.thread.join()
            self.raise_if_failed()
        finally:
            self.logger.close()


def failure_frame_path(sample_index, status, result):
    if result.frame is None:
        return ""
    return str(
        Path("failed_frames")
        / f"rx_sample_{sample_index:06d}_{status}.jpg"
    )


def main(
    argv=None,
    *,
    reuse_uwb_packets=True,
    experiment_code="dynamic_tracking",
):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    import cv2
    from camera.realsense_scanner import HardwareScanner
    from gimbal.gimbal_controller_yaw import GimbalController
    from logger.result_logger import ResultLogger

    print("[SYNC] waiting for local Chrony synchronization")
    wait_for_chrony_sync(
        max_tries=args.chrony_wait_tries,
        max_correction_sec=args.chrony_max_correction_sec,
    )
    print("[SYNC] Chrony synchronization is ready")

    gimbal = None
    scanner = None
    uwb = None
    logger = None
    result_writer = None
    camera_started = False
    sample_index = 0
    uwb_bias_deg = 0.0
    calibration_result = None

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

        uwb = LatestUwbReceiver(
            args.host,
            args.port,
            reuse_latest=reuse_uwb_packets,
        )
        uwb.start()
        logger = ResultLogger(
            target_dir_name="result",
            experiment_code=experiment_code,
            experiment_id=args.experiment_id,
            node_id="rx",
            fieldnames=RX_FIELDS,
        )
        result_writer = RxResultWriter(logger, cv2)
        print(f"[LOG] {logger.csv_path}")
        if args.uwb_calibration:
            calibration_deadline_ns = calibration_deadline_monotonic_ns(
                args.start_utc,
                args.uwb_calibration_margin_sec,
            )
            print(
                "[CALIBRATION] keep Rx/Tx stationary and facing each other; "
                f"collecting {args.uwb_calibration_samples} UWB packets"
            )
            calibration_result = uwb.calibrate(
                args.uwb_calibration_samples,
                calibration_deadline_ns,
                max_std_deg=args.uwb_calibration_max_std_deg,
                max_abs_bias_deg=args.uwb_calibration_max_bias_deg,
            )
            uwb_bias_deg = calibration_result.bias_deg
            print(
                "[CALIBRATION] Rx complete: "
                f"samples={calibration_result.sample_count}, "
                f"bias={uwb_bias_deg:.3f} deg, "
                f"circular_std="
                f"{calibration_result.circular_std_deg:.3f} deg"
            )
        else:
            print("[CALIBRATION] Rx UWB calibration disabled; bias=0 deg")
        print(
            f"[WAIT] shared UTC start={format_utc_epoch_ns(args.start_utc)}"
        )
        experiment_start_ns = wait_until_utc_ns(args.start_utc)
        print(
            "[START] Rx dynamic tracking started; "
            f"UWB packet reuse={'on' if reuse_uwb_packets else 'off'}; "
            "press Ctrl+C to stop manually"
        )

        # 시작 전 패킷은 take_latest_after()의 기준시각으로 제외한다.
        while True:
            scheduled_ns = (
                experiment_start_ns + sample_index * ALIGNMENT_PERIOD_NS
            )
            next_scheduled_ns = scheduled_ns + ALIGNMENT_PERIOD_NS
            sleep_until_monotonic_ns(scheduled_ns)
            result_writer.raise_if_failed()
            started_at = iso_now()
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
                    "uwb_calibration_sample_count": (
                        calibration_result.sample_count
                        if calibration_result is not None
                        else 0
                    ),
                    "uwb_calibration_bias_deg": uwb_bias_deg,
                    "uwb_calibration_std_deg": (
                        calibration_result.circular_std_deg
                        if calibration_result is not None
                        else ""
                    ),
                    "target_color": args.target_color,
                    "color_visible": "",
                    "color_success": 0,
                    "status": "error",
                    "started_at": started_at,
                }
            )
            failure_image = None

            try:
                uwb_selection = uwb.take_latest_after(experiment_start_ns)
                if uwb_selection is None:
                    row["status"] = "uwb_unavailable"
                else:
                    uwb_sample = uwb_selection.sample
                    previous_deg = gimbal.current_degree
                    alignment = calculate_alignment(
                        previous_deg,
                        uwb_sample.raw_azimuth_deg,
                        uwb_bias_deg,
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
                                commanded_ns - experiment_start_ns
                            )
                            / 1_000_000_000,
                            "previous_gimbal_ros_deg": previous_deg,
                            "uwb_source": uwb_sample.source,
                            "uwb_received_monotonic_ns": (
                                uwb_sample.received_monotonic_ns
                            ),
                            "uwb_packet_reused": int(
                                uwb_selection.reused
                            ),
                            "uwb_raw_azimuth_deg": (
                                uwb_sample.raw_azimuth_deg
                            ),
                            "uwb_corrected_azimuth_deg": (
                                alignment.uwb_corrected_azimuth_deg
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
                result_writer.submit(
                    row,
                    failure_frame=failure_image,
                )

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
        print(f"[DONE] Rx samples recorded: {sample_index}")


if __name__ == "__main__":
    main()

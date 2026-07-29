#!/usr/bin/env python3
"""Chrony 동기 시작으로 Tx GPIO 짐벌의 UWB 추적을 반복한다."""

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


TX_FIELDS = (
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
    "status",
    "started_at",
    "finished_at",
    "error_message",
)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Tx dynamic tracking: align the GPIO yaw gimbal from the latest "
            "UWB packet every 0.2 seconds."
        )
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--yaw-pin", type=int, default=18)
    parser.add_argument("--initial-deg", type=float, default=0.0)
    parser.add_argument(
        "--prestart-gimbal-stabilization-sec",
        type=float,
        default=1.0,
        help=(
            "hold initial yaw before disabling PWM during calibration "
            "and shared-start waiting"
        ),
    )
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
    if args.yaw_pin < 0:
        parser.error("--yaw-pin must be 0 or greater")
    if (
        not math.isfinite(args.prestart_gimbal_stabilization_sec)
        or args.prestart_gimbal_stabilization_sec < 0
    ):
        parser.error(
            "--prestart-gimbal-stabilization-sec must be 0 or greater"
        )
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
    if args.chrony_max_correction_sec <= 0:
        parser.error("--chrony-max-correction-sec must be greater than 0")
    if args.chrony_wait_tries <= 0:
        parser.error("--chrony-wait-tries must be greater than 0")


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


class TxResultWriter:
    """CSV flush와 출력을 0.2초 정렬 스케줄 밖에서 처리한다."""

    def __init__(self, logger):
        self.logger = logger
        self.queue = queue.Queue()
        self.error = None
        self.thread = threading.Thread(
            target=self._run,
            name="tx-dynamic-result-writer",
            daemon=True,
        )
        self.thread.start()

    def submit(self, row):
        self.raise_if_failed()
        self.queue.put_nowait(dict(row))

    def _run(self):
        while True:
            row = self.queue.get()
            try:
                if row is None:
                    return
                self.logger.log_sample(row)
                print(
                    f"[TX] sample={row['sample_index']} "
                    f"status={row['status']} "
                    f"uwb_raw={row['uwb_raw_azimuth_deg']} "
                    f"command={row['gimbal_command_ros_deg']}"
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


def main(
    argv=None,
    *,
    reuse_uwb_packets=True,
    experiment_code="dynamic_tracking",
):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    from gimbal.gimbal_controller_yaw_gpio import GPIOGimbalController
    from logger.result_logger import ResultLogger

    print("[SYNC] waiting for local Chrony synchronization")
    wait_for_chrony_sync(
        max_tries=args.chrony_wait_tries,
        max_correction_sec=args.chrony_max_correction_sec,
    )
    print("[SYNC] Chrony synchronization is ready")

    gimbal = None
    uwb = None
    logger = None
    result_writer = None
    sample_index = 0
    uwb_bias_deg = 0.0
    calibration_result = None

    try:
        gimbal = GPIOGimbalController(yaw_pin=args.yaw_pin)
        gimbal.move_to(args.initial_deg)
        print(
            "[GIMBAL] holding initial yaw before pre-start PWM shutdown; "
            f"stabilization="
            f"{args.prestart_gimbal_stabilization_sec:g}s"
        )
        time.sleep(args.prestart_gimbal_stabilization_sec)
        gimbal.disable_control_signal()
        print(
            "[GIMBAL] pre-start PWM signal is off; it will resume with "
            "the first valid tracking command"
        )
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
            node_id="tx",
            fieldnames=TX_FIELDS,
        )
        result_writer = TxResultWriter(logger)
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
                "[CALIBRATION] Tx complete: "
                f"samples={calibration_result.sample_count}, "
                f"bias={uwb_bias_deg:.3f} deg, "
                f"circular_std="
                f"{calibration_result.circular_std_deg:.3f} deg"
            )
        else:
            print("[CALIBRATION] Tx UWB calibration disabled; bias=0 deg")
        print(
            f"[WAIT] shared UTC start={format_utc_epoch_ns(args.start_utc)}"
        )
        experiment_start_ns = wait_until_utc_ns(args.start_utc)
        print(
            "[START] Tx dynamic tracking started; "
            f"UWB packet reuse={'on' if reuse_uwb_packets else 'off'}; "
            "press Ctrl+C to stop manually"
        )

        while True:
            scheduled_ns = (
                experiment_start_ns + sample_index * ALIGNMENT_PERIOD_NS
            )
            sleep_until_monotonic_ns(scheduled_ns)
            result_writer.raise_if_failed()
            row = {field: "" for field in TX_FIELDS}
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
                    "status": "error",
                    "started_at": iso_now(),
                }
            )

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
                            "status": "success",
                        }
                    )

            except Exception as exc:
                row["status"] = "error"
                row["error_message"] = f"{type(exc).__name__}: {exc}"
                print(f"[WARN] sample {sample_index}: {row['error_message']}")
            finally:
                row["finished_at"] = iso_now()
                result_writer.submit(row)
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
        if gimbal is not None:
            try:
                gimbal.move_to(0.0)
                time.sleep(gimbal.ALIGN_INTERVAL_SEC)
            finally:
                gimbal.cleanup()
        print(f"[DONE] Tx samples recorded: {sample_index}")


if __name__ == "__main__":
    main()

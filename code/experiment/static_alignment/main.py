import argparse
import logging
import sys
import time
from datetime import datetime
from enum import Enum, auto
from pathlib import Path


CODE_DIR = Path(__file__).resolve().parents[2]
PROJECT_ROOT = CODE_DIR.parent
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment.angle_utils import (
    calculate_final_alignment_error,
    clamp_servo_command,
    estimate_tx_azimuth,
)
from experiment.static_alignment.config_loader import load_config
from experiment.static_alignment.devices import UwbReceiver
from experiment.static_alignment.result_store import ResultStore
from experiment.static_alignment.trial_plan import build_trial_plan


DEFAULT_CONFIG = Path(__file__).with_name("config.yaml")


class TrialState(Enum):
    MOVE_TO_ZERO = auto()
    WAIT_ZERO_SETTLE = auto()
    MOVE_TO_INITIAL = auto()
    WAIT_INITIAL_SETTLE = auto()
    RESET_TRIAL_STATE = auto()
    WAIT_FOR_FIRST_VALID_UWB = auto()
    COMPUTE_ALIGNMENT = auto()
    COMMAND_GIMBAL = auto()
    WAIT_FOR_QR_OR_TIMEOUT = auto()
    SAVE_RESULT = auto()
    RETURN_TO_ZERO = auto()
    COMPLETE = auto()
    FAILED = auto()


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def resolve_output_root(config):
    path = Path(config.logging["output_directory"])
    return path if path.is_absolute() else PROJECT_ROOT / path


def configure_logging(run_dir):
    logger = logging.getLogger("static_alignment")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    for handler in (
        logging.StreamHandler(),
        logging.FileHandler(Path(run_dir) / "experiment.log", encoding="utf-8"),
    ):
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    return logger


def detect_qr_until(scanner, deadline_ns):
    """Return the first QR detected in a post-command RealSense frame."""
    result = scanner.detect_until(deadline_ns)
    return result if result.detected else None


class StaticAlignmentRunner:
    def __init__(self, config, gimbal, uwb, qr, store, logger):
        self.config = config
        self.gimbal = gimbal
        self.uwb = uwb
        self.qr = qr
        self.store = store
        self.logger = logger

    def _state(self, trial, state):
        self.logger.info("trial=%s state=%s", trial.trial_id, state.name)

    def _base_result(self, trial, started_at):
        timing = self.config.timing
        row = trial.as_row()
        row.update(
            {
                "uwb_raw_azimuth_deg": "",
                "estimated_tx_azimuth_deg": "",
                "requested_gimbal_command_deg": "",
                "gimbal_command_deg": "",
                "final_alignment_error_deg": "",
                "qr_success": False,
                "qr_recognition_time_ms": "",
                "uwb_message_missing": False,
                "servo_clipped": False,
                "trial_status": "error",
                "started_at": started_at,
                "finished_at": "",
                "error_message": "",
                "qr_timeout_s": timing["qr_success_timeout_s"],
                "uwb_timeout_s": timing["uwb_receive_timeout_s"],
                "zero_settle_time_s": timing["zero_settle_time_s"],
                "initial_settle_time_s": timing["initial_settle_time_s"],
            }
        )
        return row

    def run_trial(self, trial):
        timing = self.config.timing
        servo = self.config.servo
        row = self._base_result(trial, iso_now())
        saved = False
        try:
            self._state(trial, TrialState.MOVE_TO_ZERO)
            self.gimbal.move_to(float(servo["zero_angle_deg"]))
            self._state(trial, TrialState.WAIT_ZERO_SETTLE)
            time.sleep(float(timing["zero_settle_time_s"]))

            self._state(trial, TrialState.MOVE_TO_INITIAL)
            self.gimbal.move_to(trial.rx_initial_gimbal_deg)
            self._state(trial, TrialState.WAIT_INITIAL_SETTLE)
            time.sleep(float(timing["initial_settle_time_s"]))

            self._state(trial, TrialState.RESET_TRIAL_STATE)
            self.uwb.discard_pending()

            self._state(trial, TrialState.WAIT_FOR_FIRST_VALID_UWB)
            uwb_result = self.uwb.receive_first_valid(
                float(timing["uwb_receive_timeout_s"])
            )
            if uwb_result is None:
                row["uwb_message_missing"] = True
                row["trial_status"] = "uwb_timeout"
            else:
                (distance, raw_azimuth, _elevation), first_uwb_ns, address = uwb_result
                self.logger.info(
                    "trial=%s first_uwb source=%s:%s measured_distance=%s",
                    trial.trial_id,
                    address[0],
                    address[1],
                    distance,
                )
                row["uwb_raw_azimuth_deg"] = raw_azimuth

                self._state(trial, TrialState.COMPUTE_ALIGNMENT)
                estimated = estimate_tx_azimuth(
                    trial.rx_initial_gimbal_deg, raw_azimuth
                )
                error = calculate_final_alignment_error(
                    estimated, trial.tx_ground_truth_azimuth_deg
                )
                command, clipped = clamp_servo_command(
                    estimated,
                    float(servo["min_angle_deg"]),
                    float(servo["max_angle_deg"]),
                )
                row.update(
                    {
                        "estimated_tx_azimuth_deg": estimated,
                        "requested_gimbal_command_deg": estimated,
                        "gimbal_command_deg": command,
                        "final_alignment_error_deg": error,
                        "servo_clipped": clipped,
                    }
                )

                self._state(trial, TrialState.COMMAND_GIMBAL)
                applied = self.gimbal.move_to(command)
                if abs(float(applied) - command) > 1e-9:
                    raise RuntimeError(
                        f"gimbal applied {applied} instead of requested {command}"
                    )

                self._state(trial, TrialState.WAIT_FOR_QR_OR_TIMEOUT)
                qr_deadline_ns = first_uwb_ns + int(
                    float(timing["qr_success_timeout_s"]) * 1_000_000_000
                )
                qr_result = detect_qr_until(self.qr, qr_deadline_ns)
                if qr_result is None:
                    row["trial_status"] = "qr_timeout"
                else:
                    detected_ns = qr_result.captured_ns
                    elapsed_ms = (detected_ns - first_uwb_ns) / 1_000_000
                    if detected_ns <= qr_deadline_ns:
                        row["qr_success"] = True
                        row["qr_recognition_time_ms"] = round(elapsed_ms, 3)
                        row["trial_status"] = "success"
                    else:
                        row["trial_status"] = "qr_timeout"

            self._state(trial, TrialState.SAVE_RESULT)
            row["finished_at"] = iso_now()
            self.store.append(row)
            saved = True
        except Exception as exc:
            self._state(trial, TrialState.FAILED)
            row["trial_status"] = "error"
            row["error_message"] = f"{type(exc).__name__}: {exc}"
            row["finished_at"] = iso_now()
            self.logger.exception("trial=%s failed", trial.trial_id)
            if not saved:
                self.store.append(row)
                saved = True
        finally:
            self._state(trial, TrialState.RETURN_TO_ZERO)
            try:
                self.gimbal.move_to(float(servo["zero_angle_deg"]))
            except Exception:
                self.logger.exception("trial=%s failed to return to zero", trial.trial_id)

        self._state(trial, TrialState.COMPLETE if row["trial_status"] != "error" else TrialState.FAILED)
        return row


def validate_block(config, distance, tx_azimuth):
    distances = [float(value) for value in config.experiment["distances_m"]]
    azimuths = [float(value) for value in config.experiment["tx_azimuths_deg"]]
    if float(distance) not in distances:
        raise ValueError(f"distance must be one of {distances}")
    if float(tx_azimuth) not in azimuths:
        raise ValueError(f"tx azimuth must be one of {azimuths}")


def make_run_dir(config, requested_run_dir, plan):
    if requested_run_dir:
        path = Path(requested_run_dir)
        return path if path.is_absolute() else PROJECT_ROOT / path
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return resolve_output_root(config) / f"run_{stamp}_{plan[0].block_id}"


def run_block(args, config):
    validate_block(config, args.distance, args.tx_azimuth)
    plan = build_trial_plan(config, args.distance, args.tx_azimuth)
    run_dir = make_run_dir(config, args.run_dir, plan)
    store = ResultStore(run_dir, config, plan)
    logger = configure_logging(run_dir)
    pending = [trial for trial in plan if trial.trial_id not in store.completed_trial_ids]

    print(f"Distance: {args.distance:g} m")
    print(f"Tx ground-truth azimuth: {args.tx_azimuth:g}°")
    print(f"Number of trials: {len(plan)} ({len(pending)} pending)")
    print(f"Run directory: {run_dir}")
    if input("Type START to begin the block: ").strip() != "START":
        print("Block cancelled; no hardware was moved.")
        return 1

    from gimbal.gimbal_controller_yaw import GimbalController
    from qr.realsense_scanner import HardwareScanner

    gimbal = None
    uwb = None
    qr = None
    try:
        gimbal = GimbalController(yaw_pin=args.yaw_pin)
        uwb = UwbReceiver(config.uwb["bind_host"], config.uwb["bind_port"])
        qr = HardwareScanner(
            device_index=int(config.camera["device_index"]),
            crop_scale=float(config.camera["crop_scale"]),
            live_stream=False,
        )
        if not qr.start_capture():
            raise RuntimeError(
                f"failed to start QR camera /dev/video{config.camera['device_index']}"
            )
        runner = StaticAlignmentRunner(config, gimbal, uwb, qr, store, logger)
        for number, trial in enumerate(pending, start=1):
            print(f"[{number}/{len(pending)}] {trial.trial_id}")
            runner.run_trial(trial)
    except KeyboardInterrupt:
        logger.warning("experiment interrupted by user; rerun with --run-dir to resume")
    finally:
        if gimbal is not None:
            try:
                gimbal.move_to(float(config.servo["zero_angle_deg"]))
                time.sleep(gimbal.ALIGN_INTERVAL_SEC)
            finally:
                gimbal.cleanup()
        if uwb is not None:
            uwb.close()
        if qr is not None:
            qr.stop()
    return 0


def camera_test(args, config):
    from qr.realsense_scanner import HardwareScanner

    detector = HardwareScanner(
        device_index=int(config.camera["device_index"]),
        crop_scale=float(config.camera["crop_scale"]),
        live_stream=False,
    )
    connected = detector.start_capture()
    try:
        result = None
        if connected:
            result = detect_qr_until(
                detector,
                time.monotonic_ns() + int(args.timeout * 1_000_000_000),
            )
    finally:
        detector.stop()
    if not connected:
        print("Camera stream connection failed.")
        return 1
    if result is None:
        print("Camera works, but no QR code was decoded before timeout.")
        return 2
    print("Camera and QR detection verified.")
    return 0


def show_plan(args, config):
    validate_block(config, args.distance, args.tx_azimuth)
    plan = build_trial_plan(config, args.distance, args.tx_azimuth)
    print(f"trials={len(plan)}")
    for trial in plan:
        print(f"{trial.sequence_index:03d} {trial.trial_id} initial={trial.rx_initial_gimbal_deg:g}")
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="Static UWB-to-QR alignment experiment")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="run or resume one physical block")
    run_parser.add_argument("--distance", type=float, required=True)
    run_parser.add_argument("--tx-azimuth", type=float, required=True)
    run_parser.add_argument("--yaw-pin", type=int, default=18)
    run_parser.add_argument("--run-dir", help="existing run directory to resume")

    camera_parser = subparsers.add_parser("camera-test")
    camera_parser.add_argument("--timeout", type=float, default=5.0)

    plan_parser = subparsers.add_parser("show-plan")
    plan_parser.add_argument("--distance", type=float, required=True)
    plan_parser.add_argument("--tx-azimuth", type=float, required=True)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    config = load_config(args.config)
    if args.command == "run":
        return run_block(args, config)
    if args.command == "camera-test":
        return camera_test(args, config)
    return show_plan(args, config)


if __name__ == "__main__":
    raise SystemExit(main())

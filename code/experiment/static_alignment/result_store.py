import csv
import os
import shutil
from pathlib import Path


PLAN_FIELDS = (
    "trial_id",
    "block_id",
    "sequence_index",
    "repetition_index",
    "random_seed",
    "distance_m",
    "tx_ground_truth_azimuth_deg",
    "rx_initial_gimbal_deg",
)

RESULT_FIELDS = PLAN_FIELDS + (
    "uwb_raw_azimuth_deg",
    "estimated_tx_azimuth_deg",
    "requested_gimbal_command_deg",
    "gimbal_command_deg",
    "final_alignment_error_deg",
    "qr_success",
    "qr_recognition_time_ms",
    "uwb_message_missing",
    "servo_clipped",
    "trial_status",
    "started_at",
    "finished_at",
    "error_message",
    "qr_timeout_s",
    "uwb_timeout_s",
    "zero_settle_time_s",
    "initial_settle_time_s",
)


class ResultStore:
    def __init__(self, run_dir, config, plan):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        logging = config.logging
        self.plan_path = self.run_dir / logging["trial_plan_filename"]
        self.results_path = self.run_dir / logging["summary_filename"]
        self.flush = bool(logging["flush_after_each_trial"])
        self._write_or_validate_plan(plan)
        config_copy = self.run_dir / config.source_path.name
        if not config_copy.exists():
            shutil.copy2(config.source_path, config_copy)
        self.completed_trial_ids = self._read_completed_ids()

    def _write_or_validate_plan(self, plan):
        rows = [trial.as_row() for trial in plan]
        if self.plan_path.exists():
            with self.plan_path.open(newline="", encoding="utf-8") as plan_file:
                existing = list(csv.DictReader(plan_file))
            if [row["trial_id"] for row in existing] != [row["trial_id"] for row in rows]:
                raise ValueError("existing trial plan does not match current configuration")
            return
        with self.plan_path.open("w", newline="", encoding="utf-8") as plan_file:
            writer = csv.DictWriter(plan_file, fieldnames=PLAN_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
            plan_file.flush()
            os.fsync(plan_file.fileno())

    def _read_completed_ids(self):
        if not self.results_path.exists():
            return set()
        with self.results_path.open(newline="", encoding="utf-8") as result_file:
            return {row["trial_id"] for row in csv.DictReader(result_file)}

    def append(self, row):
        trial_id = row["trial_id"]
        if trial_id in self.completed_trial_ids:
            raise ValueError(f"duplicate trial_id: {trial_id}")
        exists = self.results_path.exists()
        with self.results_path.open("a", newline="", encoding="utf-8") as result_file:
            writer = csv.DictWriter(result_file, fieldnames=RESULT_FIELDS)
            if not exists:
                writer.writeheader()
            writer.writerow({field: row.get(field, "") for field in RESULT_FIELDS})
            if self.flush:
                result_file.flush()
                os.fsync(result_file.fileno())
        self.completed_trial_ids.add(trial_id)

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class StaticAlignmentConfig:
    raw: dict
    source_path: Path

    @property
    def experiment(self):
        return self.raw["experiment"]

    @property
    def timing(self):
        return self.raw["timing"]

    @property
    def servo(self):
        return self.raw["servo"]

    @property
    def uwb(self):
        return self.raw["uwb"]

    @property
    def camera(self):
        return self.raw["camera"]

    @property
    def logging(self):
        return self.raw["logging"]


def load_config(path):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required; run: pip install -r requirements.txt"
        ) from exc

    source_path = Path(path).resolve()
    with source_path.open(encoding="utf-8") as config_file:
        raw = yaml.safe_load(config_file)
    if not isinstance(raw, dict):
        raise ValueError("configuration root must be a mapping")

    required = ("experiment", "timing", "servo", "uwb", "camera", "logging")
    missing = [key for key in required if key not in raw]
    if missing:
        raise ValueError(f"missing configuration sections: {missing}")

    experiment = raw["experiment"]
    if int(experiment["repetitions_per_angle"]) <= 0:
        raise ValueError("repetitions_per_angle must be positive")
    for key in (
        "zero_settle_time_s",
        "initial_settle_time_s",
        "uwb_receive_timeout_s",
        "qr_success_timeout_s",
    ):
        if float(raw["timing"][key]) < 0:
            raise ValueError(f"{key} must not be negative")
    if float(raw["servo"]["min_angle_deg"]) > float(raw["servo"]["max_angle_deg"]):
        raise ValueError("servo min_angle_deg must not exceed max_angle_deg")
    return StaticAlignmentConfig(raw=raw, source_path=source_path)

import random
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class TrialSpec:
    trial_id: str
    block_id: str
    sequence_index: int
    repetition_index: int
    random_seed: int
    distance_m: float
    tx_ground_truth_azimuth_deg: float
    rx_initial_gimbal_deg: float

    def as_row(self):
        return asdict(self)


def inclusive_angle_range(start, end, step):
    start, end, step = float(start), float(end), float(step)
    if step == 0 or (end - start) * step < 0:
        raise ValueError("angle step does not reach the configured end")
    values = []
    current = start
    compare = (lambda value: value <= end + 1e-9) if step > 0 else (
        lambda value: value >= end - 1e-9
    )
    while compare(current):
        values.append(round(current, 10))
        current += step
    return values


def initial_angles_for_tx(config, tx_azimuth_deg):
    side = "negative_side" if tx_azimuth_deg < 0 else "positive_side"
    values = config.experiment[side]
    return inclusive_angle_range(
        values["initial_start_deg"],
        values["initial_end_deg"],
        values["initial_step_deg"],
    )


def _number(value):
    value = float(value)
    return str(int(value)) if value.is_integer() else format(value, "g")


def build_trial_plan(config, distance_m, tx_azimuth_deg):
    experiment = config.experiment
    repetitions = int(experiment["repetitions_per_angle"])
    random_seed = int(experiment["random_seed"])
    angles = initial_angles_for_tx(config, tx_azimuth_deg)
    trial_angles = angles * repetitions
    if experiment["randomize_trial_order"]:
        random.Random(random_seed).shuffle(trial_angles)

    repetition_counts = defaultdict(int)
    distance_label = _number(distance_m)
    tx_label = _number(tx_azimuth_deg)
    block_id = f"d{distance_label}_tx{tx_label}"
    plan = []
    for sequence_index, angle in enumerate(trial_angles, start=1):
        repetition_counts[angle] += 1
        repetition_index = repetition_counts[angle]
        trial_id = (
            f"static_d{distance_label}_tx{tx_label}_idx{sequence_index:03d}"
            f"_rep{repetition_index}"
        )
        plan.append(
            TrialSpec(
                trial_id=trial_id,
                block_id=block_id,
                sequence_index=sequence_index,
                repetition_index=repetition_index,
                random_seed=random_seed,
                distance_m=float(distance_m),
                tx_ground_truth_azimuth_deg=float(tx_azimuth_deg),
                rx_initial_gimbal_deg=float(angle),
            )
        )

    expected = Counter({angle: repetitions for angle in angles})
    if Counter(trial_angles) != expected:
        raise AssertionError("generated trial plan has invalid repetitions")
    return plan

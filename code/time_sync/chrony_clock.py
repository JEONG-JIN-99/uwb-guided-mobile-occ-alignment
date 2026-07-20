import subprocess
import time
from decimal import Decimal, InvalidOperation


NANOSECONDS_PER_SECOND = 1_000_000_000


def parse_utc_epoch_ns(value):
    """Parse Unix UTC seconds without losing sub-second decimal precision."""
    try:
        seconds = Decimal(value)
    except InvalidOperation as exc:
        raise ValueError(
            "start UTC must be Unix epoch seconds, e.g. 1784532000.000"
        ) from exc

    if not seconds.is_finite() or seconds <= 0:
        raise ValueError("start UTC must be a positive finite value")
    return int(seconds * NANOSECONDS_PER_SECOND)


def format_utc_epoch_ns(utc_ns):
    seconds, nanoseconds = divmod(utc_ns, NANOSECONDS_PER_SECOND)
    return f"{seconds}.{nanoseconds:09d}"


def wait_for_chrony_sync(max_tries=60, max_correction_sec=0.005):
    """
    Wait until the local chronyd reports synchronization within the limit.

    Both Tx and Rx run this locally. Their chrony.conf files decide which
    external Linux time server they use.
    """
    command = [
        "chronyc",
        "waitsync",
        str(max_tries),
        str(max_correction_sec),
        "0",
        "1",
    ]
    try:
        subprocess.run(command, check=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            "chronyc is not installed; install the chrony package first"
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            "Chrony did not synchronize within the configured limit"
        ) from exc


def wait_until_utc_ns(start_utc_ns):
    """
    Wait for a shared UTC start and return its corresponding monotonic time.

    UTC selects the same start instant on different machines. The returned
    monotonic timestamp is then used for stable local periodic scheduling.
    """
    remaining_ns = start_utc_ns - time.time_ns()
    if remaining_ns <= 0:
        raise ValueError("start UTC is already in the past")

    while True:
        remaining_ns = start_utc_ns - time.time_ns()
        if remaining_ns <= 0:
            break

        if remaining_ns > 20_000_000:
            time.sleep((remaining_ns - 10_000_000) / NANOSECONDS_PER_SECOND)
        else:
            time.sleep(min(remaining_ns / NANOSECONDS_PER_SECOND, 0.001))

    wall_now_ns = time.time_ns()
    monotonic_now_ns = time.monotonic_ns()
    overshoot_ns = wall_now_ns - start_utc_ns
    return monotonic_now_ns - overshoot_ns


def sleep_until_monotonic_ns(target_ns):
    remaining_ns = target_ns - time.monotonic_ns()
    if remaining_ns > 0:
        time.sleep(remaining_ns / NANOSECONDS_PER_SECOND)

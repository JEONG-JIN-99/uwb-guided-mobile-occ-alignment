def normalize_angle(angle_deg):
    """Return an angle in [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def estimate_tx_azimuth(initial_gimbal_deg, uwb_relative_azimuth_deg):
    return normalize_angle(initial_gimbal_deg + uwb_relative_azimuth_deg)


def calculate_final_alignment_error(estimated_deg, ground_truth_deg):
    return normalize_angle(estimated_deg - ground_truth_deg)


def clamp_servo_command(requested_deg, min_deg, max_deg):
    if min_deg > max_deg:
        raise ValueError("servo minimum angle must not exceed maximum angle")
    applied = min(max(float(requested_deg), float(min_deg)), float(max_deg))
    return applied, applied != float(requested_deg)

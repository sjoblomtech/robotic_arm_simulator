import math

def wrap_to_minus180_180(x_deg):
    return ((x_deg + 180.0) % 360.0) - 180.0

def ease(progress: float, mode: str = "linear") -> float:
    p = min(1.0, max(0.0, progress))
    if mode == "linear":
        return p
    if mode == "cosine":
        return 0.5 - 0.5 * math.cos(math.pi * p)
    if mode == "smoothstep":
        return (3 * p**2) - (2 * p**3)
    return p    # Fallback


def interpolate_joint_space(
    start_deg: list[float],
    end_deg: list[float],
    duration_s: float = 3.0,
    fps: int = 30,
    easing: str = "linear",
    ) -> list[list[float]]:

    # Input validation
    if not start_deg or len(start_deg) != len(end_deg):
        raise ValueError("start_deg and end_deg must be non-empty and the same length")
    
    # Number of frames in motion
    N = max(1, int(round(duration_s * fps)))

    # Compute shortest-path angular deltas
    deltas = [wrap_to_minus180_180(e - s) for s, e in zip(start_deg, end_deg)]

    frames: list[list[float]] = []

    # Build trajectory
    for k in range(N + 1):
        alpha = k / N               # Progress from 0 -> 1
        s = ease(alpha, easing)     # Apply easing

        # Interpolate each joint
        frame = [s0 + s * d for s0, d in zip(start_deg, deltas)]
        frames.append(frame)
    return frames
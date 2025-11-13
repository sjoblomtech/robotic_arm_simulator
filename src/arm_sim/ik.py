import math
from typing import Tuple

# Helper to keep angles tidy (-180, 180)
def wrap_deg(x: float) -> float:
    return ((x + 180.0) % 360.0) - 180.0

def clamp_target_to_workspace(
        x: float,
        y: float,
        L1: float,
        L2: float,
) -> Tuple[float, float, bool]:
    
    r = math.hypot(x, y)
    r_min = abs(L1 - L2)
    r_max = L1 + L2

    was_clamped = False
    if r < 1e-12:
        # If target is at origin, keep orientation along +X and clamp the nearest radius
        r_new = max(r_min, min(r, r_max))
        was_clamped = (abs(r_new - r) > 1e-9)
        return (r_new, 0.0, was_clamped)
    
    r_new = min(max(r, r_min), r_max)
    was_clamped = (abs(r_new - r) > 1e-9)

    scale = r_new / r
    return (x * scale, y * scale, was_clamped)

def ik_2link_all(
        x: float,
        y: float,
        L1: float,
        L2: float,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    
    # Distance to target
    r2 = (x * x + y * y)

    # Law of cosine for elbow
    # cos θ2 = (r² - L1² - L2²) / (2 * L1 * L2)
    c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    # Clamp numeric drift
    c2 = max(-1.0, min(1.0, c2))
    th2_abs = math.acos(c2)     # Positive principle angle in [0, π]

    # Two branches for elbow: +θ2 (down) and -θ2 (up)
    th2_down = +th2_abs
    th2_up = -th2_abs

    # Shoulder via two-argument atan2 decomposition
    # θ1 = atan2(x, y) - atan2(L2 * sin(θ2), L1 + L2 * cos(θ2))
    def shoulder(theta2: float) -> float:
        k1 = L1 + L2 * math.cos(theta2)
        k2 = L2 * math.sin(theta2)
        return math.atan2(y, x) - math.atan2(k2, k1)
    
    th1_up = shoulder(th2_up)
    th1_down = shoulder(th2_down)

    # Convert to degrees and wrap
    up = (wrap_deg(math.degrees(th1_up)), wrap_deg(math.degrees(th2_up)))
    down = (wrap_deg(math.degrees(th1_down)), wrap_deg(math.degrees(th2_down)))
    return up, down

def ik_2link(
        x: float,
        y: float,
        L1: float,
        L2: float,
        prefer: str = "elbow_up",
) -> Tuple[float, float]:
    
    up, down = ik_2link_all(x, y, L1, L2)
    if prefer == "elbow_down":
        return down
    return up
    

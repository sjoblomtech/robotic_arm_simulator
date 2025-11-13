import argparse
import json
from pathlib import Path
from typing import Any, Dict, Optional
from arm_sim.planner import interpolate_joint_space
from arm_sim.visualize import animate_joint_trajectory
from arm_sim.ik import ik_2link, clamp_target_to_workspace

# Helpers
def load_scenario(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Scenario file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("Scenario json must be an object with keys")
    return data

def coalesce(*values):
    # Return first value that is not None
    for v in values:
        if v is not None:
            return v
    return None

def build_parser():
    p = argparse.ArgumentParser(description="2D robotic arm simulator")
    # Primary inputs
    p.add_argument("--links", nargs="+", type=float, help="List of link lengths")
    p.add_argument("--start", nargs="+", type=float, help="Start joint angles in degrees")
    p.add_argument("--end", nargs="+", type=float, help="End joint angles in degrees")
    # IK inputs (for 2-link arm)
    p.add_argument("--target", nargs=2, type=float, help="Cartesian target (x, y) for IK (2-link only)")
    p.add_argument("--prefer", choices=["elbow_up", "elbow_down"], default="elbow_up",
                   help="IK branch preferences")
    p.add_argument("--clamp", action="store_true", help="Clamp target to reachable workspace for IK")
    # Timing / motion
    p.add_argument("--duration", type=float, default=3.0,
                        help="Animation duration in seconds")
    p.add_argument("--fps", type=int, default=30,
                        help="Frames per second")
    p.add_argument("--easing", choices=["linear", "cosine", "smoothstep"],
                        default="linear")
    # Viz
    p.add_argument("--trail", action="store_true",
                        help="Show end-effector trail")
    p.add_argument("--save", type=str, default=None,
                        help="Outcome filename (mp4/gif). If omitted, show interactively.")
    # Scenario file
    p.add_argument("--scenario", type=str, help="Path to scenario json (CLI flags override it)")
    # Convenience demo if nothing is provided
    p.add_argument("--demo", action="store_true", help="Run a built-in demo if no scenarios/flags are provided")
    return p

def main():
    args = build_parser().parse_args()

    # Load scenario
    scenario: Dict[str, Any] = {}
    if args.scenario:
        scenario = load_scenario(Path(args.scenario))

    # Merge: CLI flags override scenario values; then fall back to demo if requested/needed
    links = coalesce(args.links,    scenario.get("links"))
    start = coalesce(args.start,   scenario.get("start"))
    end = coalesce(args.end,  scenario.get("end"))
    target = coalesce(args.target,  scenario.get("target"))
    prefer = coalesce(args.prefer,  scenario.get("prefer"), "elbow_up")
    clamp = args.clamp or bool(scenario.get("clamp", False))
    duration = coalesce(args.duration,  scenario.get("duration"), 3.0)
    fps = coalesce(args.fps,    scenario.get("fps"), 30)
    easing = coalesce(args.easing,  scenario.get("easing"), "linear")
    # Trail: CLI --trail overrides scenario (bool flags default to False if not present)
    trail = args.trail or bool(scenario.get("trail", False))
    save = coalesce(args.save,  scenario.get("save"))

    # Build in demo if nothing was provided and --demo is set
    if args.demo and (links is None or start is None or end is None):
        links = links or [7.0, 10.0]
        start = start or [35.0, 20.0]
        end = end or [10.0, 60.0]
        duration = duration or 3.0
        fps = fps or 30
        easing = easing or "cosine"
        # Trail/save remain as chosen

    # IK mode (if target is given)
    if target is not None:
        if links is None:
            raise ValueError("IK mode requires --links ( 2 lengths).")
        if len(links) != 2:
            raise ValueError("IK mode (--target) currently supports exactly 2 links.")
        
        x, y =float(target[0]), float(target[1])

        if clamp:
            x, y, was_clamped = clamp_target_to_workspace(x, y, links[0], links[1])
            if was_clamped:
                print(f"[info] Target clamped to reachable workspace: ({x:.3f}, {y:.3f})")

        # Compute IK solution for end pose (degrees)
        th1_deg, th2_deg = ik_2link(x, y, links[0], links[1], prefer=prefer)
        end = [th1_deg, th2_deg]

        # Default start if not provided
        if start is None:
            start = [0.0, 0.0]
            
    # Validation
    if links is None or start is None or end is None:
        raise ValueError("Missing inputs. Provide --scenario or flags: --links, "
        "--start, --end, (and optionally --duration, --fps, --easing, --trail, --save).")
    if len(links) != len(start) or len(links) != len(end):
        raise ValueError("Links, start and end must have the same length")
    
    # Motion planner
    frames = interpolate_joint_space(
        start,
        end,
        duration,
        fps,
        easing,
    )

    # Animate
    animate_joint_trajectory(
        links,
        frames,
        fps,
        trail,
        save,
    )

if __name__ == "__main__":
    main()
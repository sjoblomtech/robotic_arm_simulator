import argparse
from arm_sim.planner import interpolate_joint_space
from arm_sim.visualize import animate_joint_trajectory

def build_parser():
    parser = argparse.ArgumentParser(description="2D robotic arm simulator")

    parser.add_argument("--links", nargs="+", type=float, required=True,
                        help="List of link lengths")
    
    parser.add_argument("--start", nargs="+", type=float, required=True,
                        help="Start joint angles in degrees")
    
    parser.add_argument("--end", nargs="+", type=float, required=True,
                        help="End joint angles in degrees")
    
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Animation duration in seconds")
    
    parser.add_argument("--fps", type=int, default=30,
                        help="Frames per second")
    
    parser.add_argument("--easing", choices=["linear", "cosine", "smoothstep"],
                        default="linear")
    
    parser.add_argument("--trail", action="store_true",
                        help="Show end-effector trail")
    
    parser.add_argument("--save", type=str, default=None,
                        help="Outcome filename (mp4/gif). If omitted, show interactively.")
    
    return parser

def main():
    parser = build_parser()
    args = parser.parse_args()

    # Validation
    if len(args.links) != len(args.start):
        raise ValueError("Links and start angles must match in count")
    if len(args.links) != len(args.end):
        raise ValueError("Links and end angles must match in count")
    
    # Motion planner
    frames = interpolate_joint_space(
        start_deg = args.start,
        end_deg = args.end,
        duration_s = args.duration,
        fps = args.fps,
        easing = args.easing,
    )

    # Animate
    animate_joint_trajectory(
        link_lengths = args.links,
        angle_frames = frames,
        fps = args.fps,
        trail = args.trail,
        save = args.save,
    )

if __name__ == "__main__":
    main()
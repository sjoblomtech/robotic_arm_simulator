import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from arm_sim.fk import forward_kinematics

def animate_joint_trajectory(
        link_lengths: list[float], 
        angle_frames: list[list[float]], 
        fps: int = 30,
        trail: bool = False,
        save: str | None = None,
):

    # Precompute totals
    n_frames = len(angle_frames)
    if n_frames == 0:
        raise ValueError("angle_frame is empty")
    interval_ms = 1000 / fps

    # Figure and axes
    fig, ax = plt.subplots()
    ax.set_aspect("equal", adjustable = "box")
    ax.grid(True, linestyle = "--", alpha = 0.5)

    # Static view box
    max_reach = sum(link_lengths) if link_lengths else 1.0
    ax.set_xlim(-max_reach * 1.1, max_reach * 1.1)
    ax.set_ylim(-max_reach * 1.1, max_reach * 1.1)

    # Initial frame
    initial_positions = forward_kinematics(link_lengths, angle_frames[0])
    xs = [p[0] for p in initial_positions]
    ys = [p[1] for p in initial_positions]

    line, = ax.plot(xs, ys, "-o", color = "blue")

    # trail
    if trail:
        (trail_line,) = ax.plot([], [], color ="red", linewidth = 1)
        trail_x, trail_y = [], []

    # Update function
    def update(frame_idx):
        angles = angle_frames[frame_idx]
        positions = forward_kinematics(link_lengths, angles)

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]

        line.set_data(xs, ys)

        if trail:
            trail_x.append(xs[-1])
            trail_y.append(ys[-1])
            trail_line.set_data(trail_x, trail_y)
            return (line, trail_line)

        return (line,)

    # Build animation
    anim = FuncAnimation(
        fig,
        update,
        frames = n_frames,
        interval = interval_ms,
        blit = True,
    )

    # Save or show
    if save:
        anim.save(save, fps = fps) # Requires ffmpeg for MP4, Pillow for GIF
    else:
        plt.show()

    return anim
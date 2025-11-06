import math

def forward_kinematics(link_lengths, joint_angles_deg):
    # Start at base origin
    positions = [(0.0, 0.0)]
    theta = 0

    # Accumulate angle and extend each link
    for length, angle_deg in zip(link_lengths, joint_angles_deg):
        theta += math.radians(angle_deg)
        x_prev, y_prev = positions[-1]

        # Compute new joint position
        x_new = x_prev + length * math.cos(theta)
        y_new = y_prev + length * math.sin(theta)
        positions.append((x_new, y_new))

    return positions
import math

def forward_kinematics(link_lengths, joint_angles_deg):
    positions = [(0.0, 0.0)]
    theta = 0

    for length, angle_deg in zip(link_lengths, joint_angles_deg):
        theta += math.radians(angle_deg)
        x_prev, y_prev = positions[-1]
        x_new = x_prev + length * math.cos(theta)
        y_new = y_prev + length * math.sin(theta)
        positions.append((x_new, y_new))

    return positions
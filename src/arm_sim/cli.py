from arm_sim.fk import forward_kinematics
from arm_sim.plot import plot_arm

def main():
    link_lengths = [7.0, 10.0]
    joint_angles = [35.0, 20.0]

    joint_posititons = forward_kinematics(link_lengths, joint_angles)
    print("Joint positons", joint_posititons)

    plot_arm(joint_posititons, title = "Demo 2-Link Arm")

if __name__ == "__main__":
    main()
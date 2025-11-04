import matplotlib.pyplot as plt

def plot_arm(joint_positions, title = "Arm Pose"):
    xs = [p[0] for p in joint_positions]
    ys = [p[1] for p in joint_positions]

    plt.figure()
    plt.plot(xs, ys, "-o", color = "blue", linewidth = 2, markersize = 6)
    plt.scatter(xs[0], ys[0], color = "green", s = 80, label = "Base")
    plt.scatter(xs[-1], ys[-1], color = "red", s = 80, label = "End Effektor")

    plt.axis("equal")
    plt.grid(True, linestyle = "--", alpha = 0.6)
    plt.xlabel = ("X Position")
    plt.ylabel = ("Y Position")
    plt.title(title)
    plt.legend()
    plt.show()
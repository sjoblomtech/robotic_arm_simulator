# Robotic Arm Simulation with Forward and Inverse Kinematics

#### Video Demo: https://youtu.be/jLDYTC2Y7yo

#### Description:

This project is a 2D robotic arm simulator implemented in Python. The goal of 
the project is to model, visualize, and animate the motion of planar robotic arm
using both Forward Kinematics (FK) and Inverse Kinematics (IK). The simulator
allows users to define joint angles, plan smooth joint-space motion between 
configurations, and solve inverse kinematics problems interactively by selecting
target positions with the mouse.

The project was designed as an educational and engineering-oriented tool, with a
focus on clarity, correctness, and modular design. It demonstrates core robotics 
concepts such as kinematics chains, joint-space interpolation, inverse kinematics 
solution branches, and event-driven graphical user interface.


## Forward Kinematics
Forward Kinematics computes the position of the robot arm's joints and end 
effector from a given set of joint angles. In this project, the arm is modeled 
as a planar serial chain where each link rotates relative to the previous one.

The forward kinematics implementation accumulates joint angles along the chain 
and uses basic trigonometry to compute joint positions. Starting from the base 
at the origin, each link's endpoint is computed using cosine and sine of the 
cumulative angle. This produces a list of joint positions that can be plotted or 
animated.

Forward kinematics is used both for static visualization of arm poses and as the
foundation for animation and inverse kinematics verification.


## Inverse Kinematics
Inverse kinematics solves the opposite problem: determining the joint angles
required for the end effector to reach a desired Cartesian position. This project
implements a close-form inverse kinematics solution for a 2-link planar arm.

The solver supports both "elbow-up" and "elbow-down" configurations and can 
optionally clamp unreachable targets to the closest point within the arm's 
reachable workspace. This behavior mirrors real robotic systems, where commands
outside the workspace must be handled gracefully.

Inverse kinematics can be used in two ways:
- By entering a Cartesian target numerically
- By clicking directly inside the plot to set a target interactively

The inverse kinematics solution integrates seamlessly with the same motion 
planning and animation pipeline used for forward kinematics.


## Motion Planning and Animation
Motion between joint configurations is generated using joint-space interpolation.
Rather than moving the end effector directly in Cartesian space, the simulator 
interpolate joint angles over time. This reflects how many real robotic controllers
work.

The planner supports different easing functions (linear, cosine, smoothstep) to
control motion smoothness. A fixed frame rate is used to generate a sequence of
joint angle frames, which are then rendered as an animation.

This approach allows the same planning logic to be reused across:
- Command-line interface (CLI)
- Graphical user interface (GUI)
- Forward and inverse kinematics modes


# Graphical User Interface
The graphical interface is implemented using PyQt with an embedded Matplotlib
canvas. The GUI allows users to switch between Forwawrd Kinematics and Inverse 
Kinematics modes.

In Forward Kinematics mode:
- Sliders define start and end joint angles
- A play button animates the motion between poses

In Inverse Kinematics mode:
- A Cartesian target can be set by clicking in the plot
- The arm automatically solves and animates toward the target
- Elbow-up or elbow-down configurations can be selected

The GUI is event-driven and separates user interaction from kinematics and 
planning logic.


## File Structure and Design Choices
The project is structured into modular components:

- fk.py
    Implements forward kinematics calculations.

- ik.py
    Implements inverse kinematics for a 2-link planar arm, including workspace
    clamping.

- planner.py
    Generates joint-space trajectories and easing-based interpolation.

- visualize.py
    Contains aniamtion and plotting utilities shared by CLI and GUI.

- gui.py
    Implements the PyQt-based graphical user interface.

- cli.py
    Provides a command-line interface for running FK and IK animations.

- scenarios/
    Contains example JSON scenario files for reproducible demos.

Separating these concerns avoids duplicated logic and keeps mathimatical code
independent from user interface code.


## Challenges and Decisions
One of the main challanges was managing multiple modes (FK and IK) while reusing
as much as possible. This was adressed by centralizing motion planning and 
animation logic and making the GUI responsible only for collecting user input and
displaying results.

Another challange was handling unreachable inverse kinematics targets. Instead of
failing, The solver optionally clamps targets to the reachable workspace and
informs the user, which reflecs how real robotics systems behave.

Design decisions prioritized clarity and correctness over maximum performance,
making the simulator easier to understand and suitable as a learning tool.


## Future Improvements
Possible future extensions include:
- Supporting inverse kinematics for more than two links
- Adding joint limits and velocity constrains
- Implementing obstacle avoidance
- Exporting trajectories for use on real robotic hardware
- Extending the simulator to 3D kinematics


This project demonstrates the integration of robotics mathematics, motion 
planning, visualization, and graphical user interface design in a single cohesive
application.
# Robotic Arm Simulator (Python)

Planar robotic arm simulation with matplotlib animation.
- FK (forward kinematics)
- Joint-space trajectory (start>end)
- Matplotlib visualization & MP4 export

## Quick start

# Create a vitrual environment (recommended):
```bash
python3 -m venv .venv
source .venv/bin/active

# Install depenencies:
python -m pip install -r requirements.txt

# Run simulator:
python -m arm_sim.cli --demo
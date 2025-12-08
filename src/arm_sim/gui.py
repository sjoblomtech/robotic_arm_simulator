import sys
from typing import List

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QPushButton,
    QGroupBox,
    QGridLayout,
    QDoubleSpinBox,
    QCheckBox,
    QComboBox,
)
from PyQt6.QtCore import Qt, QTimer

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from arm_sim.fk import forward_kinematics
from arm_sim.planner import interpolate_joint_space
from arm_sim.ik import ik_2link, clamp_target_to_workspace

class ArmSimWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Robotic Arm Simulator (PyQt + Matplotlib, FK mode)")

        # Model parameters
        self.link_lengths: List[float] = [7.0, 10.0]
        self.fps: int = 30

        # Animation state
        self.frame: list[list[float]] = []
        self.frame_index: int = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer_tick)

        # Central layout
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout()
        central.setLayout(root_layout)

        # Matplotlib canvas
        self.fig = Figure(figsize=(5, 5))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)

        self._setup_axes()
        root_layout.addWidget(self.canvas, stretch=2)

        # Control panel
        controls = QWidget()
        controls_layout = QVBoxLayout()
        controls.setLayout(controls_layout)
        root_layout.addWidget(controls, stretch=1)

        # Start pose slider
        start_group = QGroupBox("Start pose (FK)")
        start_grid = QGridLayout()
        start_group.setLayout(start_grid)

        self.start_labels: list[QLabel] = []
        self.start_sliders: list[QSlider] = []

        for i in range(2):
            lbl = QLabel(f"Joint {i+1} start: 0°")
            sld = QSlider(Qt.Orientation.Horizontal)
            sld.setRange(-180, 180)
            sld.setSingleStep(1)
            sld.setValue(0)
            sld.valueChanged.connect(self.on_start_changed)

            self.start_labels.append(lbl)
            self.start_sliders.append(sld)

            start_grid.addWidget(lbl, i, 0)
            start_grid.addWidget(sld, i, 1)

        controls_layout.addWidget(start_group)

        # End pose sliders
        end_group = QGroupBox("End pose (FK)")
        end_grid = QGridLayout()
        end_group.setLayout(end_grid)

        self.end_labels: list[QLabel] = []
        self.end_sliders: list[QSlider] = []

        for i in range(2):
            lbl = QLabel(f"Joint {i+1} end: 0°")
            sld = QSlider(Qt.Orientation.Horizontal)
            sld.setRange(-180, 180)
            sld.setSingleStep(1)
            sld.setValue(0)
            sld.valueChanged.connect(self.on_end_changed)

            self.end_labels.append(lbl)
            self.end_sliders.append(sld)

            end_grid.addWidget(lbl, i, 0)
            end_grid.addWidget(sld, i, 1)

        controls_layout.addWidget(end_group)

        # IK target controls (for 2-link arm)
        ik_group = QGroupBox("IK target (set END pose)")
        ik_grid = QGridLayout()
        ik_group.setLayout(ik_grid)

        # Target X
        ik_grid.addWidget(QLabel("Target X:"), 0, 0)
        self.target_x_spin = QDoubleSpinBox()
        self.target_x_spin.setRange(-50.0, 50.0)
        self.target_x_spin.setSingleStep(0.1)
        self.target_x_spin.setValue(10.0)
        ik_grid.addWidget(self.target_x_spin, 0, 1)

        # Target Y
        ik_grid.addWidget(QLabel("Target Y"), 1, 0)
        self.target_y_spin = QDoubleSpinBox()
        self.target_y_spin.setRange(-50.0, 50.0)
        self.target_y_spin.setSingleStep(0.1)
        self.target_y_spin.setValue(10.0)
        ik_grid.addWidget(self.target_y_spin, 1, 1)

        # Elbow preference
        ik_grid.addWidget(QLabel("Elbow"), 2, 0)
        self.prefer_combo = QComboBox()
        self.prefer_combo.addItems(["elbow_up", "elbow_down"])
        ik_grid.addWidget(self.prefer_combo, 2, 1)

        # Clamp checkbox
        self.clamp_checkbox = QCheckBox("Clamp to wokrspace")
        self.clamp_checkbox.setChecked(True)
        ik_grid.addWidget(self.clamp_checkbox, 3, 0, 1, 2)

        # Solve IK button
        self.solve_ik_button = QPushButton("Solve IK -> End pose")
        self.solve_ik_button.clicked.connect(self.on_solve_ik_clicked)
        ik_grid.addWidget(self.solve_ik_button, 4, 0, 1, 2)

        controls_layout.addWidget(ik_group)

        # Duration slider
        duration_group = QGroupBox("Animation duration (seconds)")
        duration_layout = QVBoxLayout()
        duration_group.setLayout(duration_layout)

        self.duration_label = QLabel("Duration: 3 s")
        self.duration_slider = QSlider(Qt.Orientation.Horizontal)
        self.duration_slider.setRange(1, 10)
        self.duration_slider.setSingleStep(1)
        self.duration_slider.setValue(3)
        self.duration_slider.valueChanged.connect(self.on_duration_changed)

        duration_layout.addWidget(self.duration_label)
        duration_layout.addWidget(self.duration_slider)

        controls_layout.addWidget(duration_group)

        # Play button
        self.play_button = QPushButton("Play animation")
        self.play_button.clicked.connect(self.on_play_clicked)
        controls_layout.addWidget(self.play_button)

        controls_layout.addStretch(1)

        self._draw_pose([0.0, 0.0])

    # Helpers for GUI state

    def _setup_axes(self):
        self.ax.clear()
        self.ax.set_aspect("equal", adjustable="box")
        max_reach = sum(self.link_lengths) if self.link_lengths else 1.0
        pad = max_reach * 1.1
        self.ax.set_xlim(-pad, pad)
        self.ax.set_ylim(-pad, pad)
        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Planar 2-link arm (FK)")

    def _get_start_angles(self) -> list[float]:
        return [float(s.value()) for s in self.start_sliders]
    
    def _get_end_angles(self) -> list[float]:
        return [float(s.value()) for s in self.end_sliders]
    
    def _get_duration(self) -> float:
        return float(self.duration_slider.value())
    
    # Slider callbacks
    def on_start_changed(self):
        angles = self._get_start_angles()
        for  i, (lbl, val) in enumerate(zip(self.start_labels, angles), start=1):
            lbl.setText(f"Joint {i} start: {val:.0f}°")

    def on_end_changed(self):
        angles = self._get_end_angles()
        for i, (lbl, val) in enumerate(zip(self.end_labels, angles), start=1):
            lbl.setText(f"Joint {i} end: {val:.0f}°")
        # Show end pose preview
        self._draw_pose(angles)
    
    def on_duration_changed(self):
        dur = self._get_duration()
        self.duration_label.setText(f"Duration: {dur:.0f} s")

    # Play / Animation
    def on_play_clicked(self):
        # Compute a joint-spae trajectory and start animating.
        start = self._get_start_angles()
        end = self._get_end_angles()
        duration = self._get_duration()

        # Build frames
        self.frames = interpolate_joint_space(
            start_deg=start,
            end_deg=end,
            duration_s=duration,
            fps=self.fps,
            easing="cosine",
        )
        self.frame_index = 0

        # Disable button while animation runs
        self.play_button.setEnabled(False)

        # Start timer
        interval_ms = int(1000 / self.fps)
        self.timer.start(interval_ms)

    def on_timer_tick(self):
        if self.frame_index >= len(self.frames):
            self.timer.stop()
            self.play_button.setEnabled(True)
            return
        
        angles = self.frames[self.frame_index]
        self._draw_pose(angles)
        self.frame_index += 1

    # Drawing
    def _draw_pose(self, joint_angles_deg: list[float]):
        # Draw a single pose of the arm on the embedded canvas
        self._setup_axes()

        pts = forward_kinematics(self.link_lengths, joint_angles_deg)

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        self.ax.plot(xs, ys, "-o", color="blue", markersize=8)
        self.canvas.draw_idle()

    def on_solve_ik_clicked(self):
        # Use OL to compute end pose angles from a Cartesian target.
        if len(self.link_lengths) != 2:
            print("[IK] Currently only supports exactly 2 links.")
            return
         
        L1, L2 = self.link_lengths
        x = float(self.target_x_spin.value())
        y = float(self.target_y_spin.value())
        prefer = self.prefer_combo.currentText()
        clamp = self.clamp_checkbox.isChecked()

        # Clamp target if requested
        if clamp:
            x_clamped, y_clamped, was_clamped = clamp_target_to_workspace(x, y, L1, L2)
            if was_clamped:
                print(f"[IK] Target clamped from ({x:.2f}, {y:.2f}) "
                    f"to ({x_clamped:.2f}, {y_clamped:.2f})")
                # Update spin boxes to show clamped values
                self.target_x_spin.blockSignals(True)
                self.target_y_spin.blockSignals(True)
                self.target_x_spin.setValue(x_clamped)
                self.target_y_spin.setValue(y_clamped)
                self.target_x_spin.blockSignals(False)
                self.target_y_spin.blockSignals(False)
            x, y = x_clamped, y_clamped

        # Compute IK
        try:
            th1_deg, th2_deg = ik_2link(x, y, L1, L2, prefer=prefer)
        except ValueError as e:
            print(f"[IK] Error: {e}")
            return
        
        # Update end sliders
        self.end_sliders[0].blockSignals(True)
        self.end_sliders[1].blockSignals(True)
        self.end_sliders[0].setValue(int(round(th1_deg)))
        self.end_sliders[1].setValue(int(round(th2_deg)))
        self.end_sliders[0].blockSignals(False)
        self.end_sliders[1].blockSignals(False)

        # Update labels
        self.end_labels[0].setText(f"Joint 1 end: {th1_deg:.1f}°")
        self.end_labels[1].setText(f"Joint 2 end: {th2_deg:.1f}°")

        # Preview IK result
        self._draw_pose([th1_deg, th2_deg])

def main():
    app = QApplication(sys.argv)
    win = ArmSimWindow()
    win.resize(900, 500)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
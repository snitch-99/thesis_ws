import sys
import os
import re
import csv
import json
import math
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QDoubleSpinBox, QComboBox, QPushButton, QGroupBox,
    QFormLayout, QFileDialog, QMessageBox, QSplitter
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

# ── Paths ──
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(SCRIPT_DIR, '..', 'launch', 'drone_mapping.launch.py')
WORKSPACE_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, '..', '..'))
INSTALL_CONFIG_DIR = os.path.join(WORKSPACE_ROOT, 'install', 'drone_mapping',
                                  'share', 'drone_mapping', 'config')

WAYPOINTS_FILE = 'waypoints.csv'
CONFIG_FILE = 'trajectory_config.json'


def parse_launch_file(path):
    """Parse launch file to extract drone spawn and rock positions."""
    drone_spawn = {'x': 0.0, 'y': 0.0}
    rocks = []

    try:
        with open(path, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        return drone_spawn, rocks

    # Parse PX4_GZ_MODEL_POSE for drone spawn
    pose_match = re.search(r"PX4_GZ_MODEL_POSE.*?'([^']+)'", content)
    if pose_match:
        parts = pose_match.group(1).split(',')
        if len(parts) >= 2:
            drone_spawn['x'] = float(parts[0])
            drone_spawn['y'] = float(parts[1])

    # Parse spawn_rock nodes for rock positions
    rock_pattern = re.findall(
        r"spawn_rock\w*\s*=\s*Node\(.*?arguments\s*=\s*\[(.*?)\]",
        content, re.DOTALL
    )
    for block in rock_pattern:
        name_match = re.search(r"'-name',\s*'(\w+)'", block)
        x_match = re.search(r"'-x',\s*'([^']+)'", block)
        y_match = re.search(r"'-y',\s*'([^']+)'", block)
        if name_match and x_match and y_match:
            rocks.append({
                'name': name_match.group(1),
                'x': float(x_match.group(1)),
                'y': float(y_match.group(1))
            })

    return drone_spawn, rocks


def generate_circle(center_x, center_y, radius, height, angular_res=0.1):
    waypoints = []
    angles = np.arange(0, 2 * np.pi, angular_res)
    for theta in angles:
        x = center_x + radius * np.cos(theta)
        y = center_y + radius * np.sin(theta)
        waypoints.append((x, y, height))
    return waypoints


def generate_square(center_x, center_y, radius, height, step_size=0.5):
    waypoints = []
    corners = [
        (center_x + radius, center_y + radius),
        (center_x - radius, center_y + radius),
        (center_x - radius, center_y - radius),
        (center_x + radius, center_y - radius),
        (center_x + radius, center_y + radius),
    ]
    for i in range(len(corners) - 1):
        sx, sy = corners[i]
        ex, ey = corners[i + 1]
        dist = math.sqrt((ex - sx) ** 2 + (ey - sy) ** 2)
        num_steps = max(int(np.ceil(dist / step_size)), 1)
        xs = np.linspace(sx, ex, num_steps, endpoint=False)
        ys = np.linspace(sy, ey, num_steps, endpoint=False)
        for x, y in zip(xs, ys):
            waypoints.append((x, y, height))
    return waypoints


def reorder_from_home(waypoints, home_x, home_y, height):
    """Reorder waypoints to start closest to home and prepend home."""
    if not waypoints:
        return waypoints
    min_dist = float('inf')
    closest = 0
    for i, (wx, wy, _) in enumerate(waypoints):
        d = (wx - home_x) ** 2 + (wy - home_y) ** 2
        if d < min_dist:
            min_dist = d
            closest = i
    ordered = waypoints[closest:] + waypoints[:closest]
    ordered.insert(0, (home_x, home_y, height))
    return ordered


class TrajectoryGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Trajectory Generator')
        self.setMinimumSize(1200, 700)

        # Parse launch file
        launch_path = os.path.normpath(LAUNCH_FILE)
        self.drone_spawn, self.rocks = parse_launch_file(launch_path)

        self._build_ui()
        self._update_plot()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # ── Left panel: controls ──
        controls = QWidget()
        ctrl_layout = QVBoxLayout(controls)

        # Trajectory type
        type_group = QGroupBox('Trajectory Type')
        type_form = QFormLayout()
        self.type_combo = QComboBox()
        self.type_combo.addItems(['Circle', 'Square'])
        self.type_combo.setCurrentIndex(1)
        self.type_combo.currentIndexChanged.connect(self._update_plot)
        type_form.addRow('Type:', self.type_combo)
        type_group.setLayout(type_form)
        ctrl_layout.addWidget(type_group)

        # Parameters
        param_group = QGroupBox('Parameters')
        param_form = QFormLayout()

        self.center_x_spin = QDoubleSpinBox()
        self.center_x_spin.setRange(-100, 100)
        self.center_x_spin.setValue(5.0)
        self.center_x_spin.setSingleStep(0.5)
        self.center_x_spin.valueChanged.connect(self._update_plot)
        param_form.addRow('Center X:', self.center_x_spin)

        self.center_y_spin = QDoubleSpinBox()
        self.center_y_spin.setRange(-100, 100)
        self.center_y_spin.setValue(-5.0)
        self.center_y_spin.setSingleStep(0.5)
        self.center_y_spin.valueChanged.connect(self._update_plot)
        param_form.addRow('Center Y:', self.center_y_spin)

        self.radius_spin = QDoubleSpinBox()
        self.radius_spin.setRange(0.5, 100)
        self.radius_spin.setValue(6.0)
        self.radius_spin.setSingleStep(0.5)
        self.radius_spin.valueChanged.connect(self._update_plot)
        param_form.addRow('Radius:', self.radius_spin)

        self.height_spin = QDoubleSpinBox()
        self.height_spin.setRange(0.5, 50)
        self.height_spin.setValue(1.2)
        self.height_spin.setSingleStep(0.1)
        self.height_spin.valueChanged.connect(self._update_plot)
        param_form.addRow('Height (m):', self.height_spin)

        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.1, 5.0)
        self.step_spin.setValue(0.5)
        self.step_spin.setSingleStep(0.1)
        self.step_spin.valueChanged.connect(self._update_plot)
        param_form.addRow('Step Size (m):', self.step_spin)

        self.angular_res_spin = QDoubleSpinBox()
        self.angular_res_spin.setRange(0.01, 1.0)
        self.angular_res_spin.setValue(0.1)
        self.angular_res_spin.setSingleStep(0.01)
        self.angular_res_spin.setDecimals(3)
        self.angular_res_spin.valueChanged.connect(self._update_plot)
        param_form.addRow('Angular Res (rad):', self.angular_res_spin)

        param_group.setLayout(param_form)
        ctrl_layout.addWidget(param_group)

        # Info display
        info_group = QGroupBox('Info')
        info_layout = QVBoxLayout()
        self.info_label = QLabel()
        info_layout.addWidget(self.info_label)
        info_group.setLayout(info_layout)
        ctrl_layout.addWidget(info_group)

        # Save button
        self.save_btn = QPushButton('Save Trajectory')
        self.save_btn.setStyleSheet('font-size: 14px; padding: 10px;')
        self.save_btn.clicked.connect(self._save)
        ctrl_layout.addWidget(self.save_btn)

        ctrl_layout.addStretch()

        # ── Right panel: plots ──
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)

        # 2D plot
        self.fig_2d = Figure(figsize=(6, 4))
        self.canvas_2d = FigureCanvas(self.fig_2d)
        self.ax_2d = self.fig_2d.add_subplot(111)
        plot_layout.addWidget(QLabel('2D View (Top-Down)'))
        plot_layout.addWidget(self.canvas_2d)

        # 3D plot
        self.fig_3d = Figure(figsize=(6, 4))
        self.canvas_3d = FigureCanvas(self.fig_3d)
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        plot_layout.addWidget(QLabel('3D View'))
        plot_layout.addWidget(self.canvas_3d)

        # Splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(controls)
        splitter.addWidget(plot_widget)
        splitter.setSizes([300, 900])
        main_layout.addWidget(splitter)

    def _generate_waypoints(self):
        center_x = self.center_x_spin.value()
        center_y = self.center_y_spin.value()
        radius = self.radius_spin.value()
        height = self.height_spin.value()
        traj_type = self.type_combo.currentIndex()

        if traj_type == 0:
            wps = generate_circle(center_x, center_y, radius, height,
                                  self.angular_res_spin.value())
        else:
            wps = generate_square(center_x, center_y, radius, height,
                                  self.step_spin.value())

        home_x = self.drone_spawn['x']
        home_y = self.drone_spawn['y']
        wps = reorder_from_home(wps, home_x, home_y, height)
        return wps

    def _update_plot(self):
        wps = self._generate_waypoints()
        xs = [w[0] for w in wps]
        ys = [w[1] for w in wps]
        zs = [w[2] for w in wps]

        # Update info
        self.info_label.setText(
            f'Waypoints: {len(wps)}\n'
            f'Drone spawn: ({self.drone_spawn["x"]}, {self.drone_spawn["y"]})\n'
            f'Rocks: {len(self.rocks)}\n'
            + '\n'.join(f'  {r["name"]}: ({r["x"]}, {r["y"]})' for r in self.rocks)
        )

        # ── 2D plot ──
        self.ax_2d.clear()
        self.ax_2d.plot(xs, ys, 'b.-', markersize=3, linewidth=0.8, label='Trajectory')
        self.ax_2d.plot(xs[0], ys[0], 'gs', markersize=10, label='Start')
        self.ax_2d.plot(self.drone_spawn['x'], self.drone_spawn['y'],
                        'g^', markersize=12, label='Drone Spawn')
        center_x = self.center_x_spin.value()
        center_y = self.center_y_spin.value()
        self.ax_2d.plot(center_x, center_y, 'r+', markersize=15,
                        markeredgewidth=2, label='Center')

        for rock in self.rocks:
            self.ax_2d.plot(rock['x'], rock['y'], 'ks', markersize=8)
            self.ax_2d.annotate(rock['name'], (rock['x'], rock['y']),
                                textcoords='offset points', xytext=(5, 5), fontsize=8)

        self.ax_2d.set_xlabel('X (m)')
        self.ax_2d.set_ylabel('Y (m)')
        self.ax_2d.set_aspect('equal')
        self.ax_2d.legend(fontsize=8)
        self.ax_2d.grid(True, alpha=0.3)
        self.ax_2d.set_title('Top-Down View')
        self.fig_2d.tight_layout()
        self.canvas_2d.draw()

        # ── 3D plot ──
        self.ax_3d.clear()
        self.ax_3d.plot(xs, ys, zs, 'b.-', markersize=3, linewidth=0.8)
        self.ax_3d.scatter(xs[0], ys[0], zs[0], c='green', s=80, marker='^', label='Start')
        self.ax_3d.scatter(center_x, center_y, 0, c='red', s=80, marker='+', label='Center')

        for rock in self.rocks:
            self.ax_3d.scatter(rock['x'], rock['y'], 0, c='black', s=60, marker='s')
            self.ax_3d.text(rock['x'], rock['y'], 0, rock['name'], fontsize=7)

        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        self.ax_3d.set_title('3D View')
        self.ax_3d.legend(fontsize=8)
        self.fig_3d.tight_layout()
        self.canvas_3d.draw()

    def _save(self):
        wps = self._generate_waypoints()
        center_x = self.center_x_spin.value()
        center_y = self.center_y_spin.value()
        radius = self.radius_spin.value()
        height = self.height_spin.value()
        traj_type = self.type_combo.currentIndex()

        config = {
            'trajectory_type': traj_type,
            'center_x': center_x,
            'center_y': center_y,
            'radius': radius,
            'height': height,
            'step_size': self.step_spin.value(),
            'angular_resolution': self.angular_res_spin.value(),
            'threshold': 0.2,
            'num_waypoints': len(wps)
        }

        save_dirs = [SCRIPT_DIR]
        install_path = os.path.normpath(INSTALL_CONFIG_DIR)
        if os.path.isdir(install_path):
            save_dirs.append(install_path)
        else:
            os.makedirs(install_path, exist_ok=True)
            save_dirs.append(install_path)

        for d in save_dirs:
            # Save CSV
            csv_path = os.path.join(d, WAYPOINTS_FILE)
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'z'])
                for wp in wps:
                    writer.writerow([f'{wp[0]:.4f}', f'{wp[1]:.4f}', f'{wp[2]:.4f}'])

            # Save JSON
            json_path = os.path.join(d, CONFIG_FILE)
            with open(json_path, 'w') as f:
                json.dump(config, f, indent=2)

        QMessageBox.information(
            self, 'Saved',
            f'Saved {len(wps)} waypoints to:\n' +
            '\n'.join(os.path.join(d, WAYPOINTS_FILE) for d in save_dirs)
        )


def main():
    app = QApplication(sys.argv)
    window = TrajectoryGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

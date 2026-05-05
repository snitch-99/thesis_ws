"""
Rock Spawner GUI
----------------
1. Starts Gazebo (default world) and ros_gz_bridge on launch.
2. Parses ROCK_CONFIGS from drone_mapping.launch.py — no duplication.
3. Dropdown lists every rock; selecting one populates its editable params.
4. Spawn / Delete buttons talk directly to Gazebo via ros2 run ros_gz_sim create.
5. Survey Area canvas shows rock positions as interactive points.
6. Export button generates the updated ROCK_CONFIGS block for copy-paste.

Usage (ROS2 must be sourced):
    python3 rock_spawner_gui.py
"""

import ast
import os
import re
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

# ── Paths ──────────────────────────────────────────────────────────────────────
HOME         = os.path.expanduser('~')
LAUNCH_FILE  = os.path.join(
    HOME, 'workspaces/thesis_ws/src/drone_mapping/launch/drone_mapping.launch.py'
)
MODELS_PATH  = os.path.join(
    HOME, 'workspaces/thesis_ws/install/drone_mapping/share/drone_mapping/models/entities'
)
AGENTS_PATH  = os.path.join(
    HOME, 'workspaces/thesis_ws/install/drone_mapping/share/drone_mapping/models/agents'
)
GZ_WORLD     = os.path.join(HOME, 'PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf')

# ── Canvas constants ───────────────────────────────────────────────────────────
CANVAS_SIZE   = 340          # pixels
DOT_RADIUS    = 6            # px — normal rock dot
DOT_SEL       = 8            # px — selected rock dot
COLORS = {
    'bg':       '#1a1a2e',
    'circle':   '#4a4a8a',
    'grid':     '#2a2a4a',
    'rock':     '#e0a060',
    'terrain':  '#60a060',
    'selected': '#ff4444',
    'spawned':  '#44ff88',
    'label':    '#ffffff',
}


# ── Parse ROCK_CONFIGS from the launch file ────────────────────────────────────

def load_rock_configs(launch_file: str) -> list:
    """Extract ROCK_CONFIGS list from drone_mapping.launch.py using AST parsing."""
    with open(launch_file, 'r') as f:
        source = f.read()
    match = re.search(r'ROCK_CONFIGS\s*=\s*(\[.*?\])', source, re.DOTALL)
    if not match:
        raise ValueError('Could not find ROCK_CONFIGS in launch file.')
    block = re.sub(r'#[^\n]*', '', match.group(1))
    return ast.literal_eval(block)


# ── Gazebo / ROS helpers ───────────────────────────────────────────────────────

def start_gazebo(world_path: str) -> subprocess.Popen:
    env = os.environ.copy()
    existing = env.get('GZ_SIM_RESOURCE_PATH', '')
    parts = [p for p in [MODELS_PATH, AGENTS_PATH, existing] if p]
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join(parts)
    return subprocess.Popen(
        ['gz', 'sim', '-r', world_path],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def start_bridge() -> subprocess.Popen:
    return subprocess.Popen(
        [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _gz_control(pause: bool):
    """Pause or resume Gazebo physics to prevent crash on model removal."""
    req = 'pause: true' if pause else 'pause: false'
    subprocess.run(
        [
            'gz', 'service',
            '-s', '/world/default/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req,
        ],
        capture_output=True, timeout=5,
    )


def spawn_rock(name, model, x, y, z, R, P, Y) -> tuple[bool, str]:
    sdf = os.path.join(MODELS_PATH, model, 'model.sdf')
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', name,
        '-x', str(x), '-y', str(y), '-z', str(z),
        '-R', str(R), '-P', str(P), '-Y', str(Y),
        '-file', sdf,
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    return r.returncode == 0, (r.stdout + r.stderr).strip()


def delete_rock(name) -> tuple[bool, str]:
    """Pause physics → remove model → resume physics (avoids Gazebo crash)."""
    _gz_control(pause=True)
    try:
        cmd = [
            'gz', 'service',
            '-s', '/world/default/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'name: "{name}" type: MODEL',
        ]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    finally:
        _gz_control(pause=False)


# ── GUI ────────────────────────────────────────────────────────────────────────

class RockSpawnerGUI:

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title('Rock Spawner — Thesis Ch.3')
        self.root.resizable(False, False)

        self._gz_proc     = None
        self._bridge_proc = None
        self.spawned: dict[str, dict] = {}   # name → cfg dict

        try:
            self.rock_configs: list[dict] = load_rock_configs(LAUNCH_FILE)
        except Exception as e:
            messagebox.showerror('Load error', str(e))
            self.rock_configs = []

        self._build_ui()
        self._start_services()
        root.protocol('WM_DELETE_WINDOW', self._on_close)

    # ── UI construction ────────────────────────────────────────────────────────

    def _build_ui(self):
        # Left column: controls
        left = ttk.Frame(self.root)
        left.grid(row=0, column=0, padx=(10, 4), pady=8, sticky='n')

        # ── Rock selector ──────────────────────────────────────────────────────
        sel_frm = ttk.LabelFrame(left, text='Rock Config', padding=8)
        sel_frm.pack(fill='x', pady=(0, 4))

        ttk.Label(sel_frm, text='Select rock:').grid(row=0, column=0, sticky='w')
        self.rock_var = tk.StringVar()
        names = [c['name'] for c in self.rock_configs]
        self.rock_cb = ttk.Combobox(
            sel_frm, textvariable=self.rock_var,
            values=names, state='readonly', width=14
        )
        self.rock_cb.grid(row=0, column=1, padx=6)
        if names:
            self.rock_cb.current(0)
        self.rock_var.trace_add('write', lambda *_: self._on_rock_selected())

        # ── Pose fields ────────────────────────────────────────────────────────
        pose_frm = ttk.LabelFrame(left, text='Pose', padding=8)
        pose_frm.pack(fill='x', pady=4)

        labels = ['X', 'Y', 'Z', 'R (roll)', 'P (pitch)', 'Y (yaw)']
        keys   = ['x', 'y', 'z', 'R', 'P', 'Y']
        self.pose_vars: dict[str, tk.StringVar] = {}
        for i, (lbl, key) in enumerate(zip(labels, keys)):
            ttk.Label(pose_frm, text=f'{lbl}:').grid(
                row=i // 3, column=(i % 3) * 2, sticky='e', padx=(8, 2), pady=3)
            var = tk.StringVar()
            entry = ttk.Entry(pose_frm, textvariable=var, width=9)
            entry.grid(row=i // 3, column=(i % 3) * 2 + 1, padx=2)
            self.pose_vars[key] = var
        # Redraw canvas dot when X or Y changes
        self.pose_vars['x'].trace_add('write', lambda *_: self._update_selected_dot())
        self.pose_vars['y'].trace_add('write', lambda *_: self._update_selected_dot())

        # ── Action buttons ─────────────────────────────────────────────────────
        btn_frm = ttk.Frame(left, padding=4)
        btn_frm.pack(fill='x')
        ttk.Button(btn_frm, text='Spawn',           command=self._on_spawn).pack(side='left',  padx=4)
        ttk.Button(btn_frm, text='Delete Selected', command=self._on_delete).pack(side='left', padx=4)
        ttk.Button(btn_frm, text='Delete All',      command=self._on_delete_all).pack(side='left', padx=4)
        ttk.Button(btn_frm, text='Export',          command=self._on_export).pack(side='right', padx=4)

        # ── Spawned list ───────────────────────────────────────────────────────
        list_frm = ttk.LabelFrame(left, text='Currently Spawned', padding=6)
        list_frm.pack(fill='x', pady=4)

        cols = ('name', 'model', 'x', 'y', 'z', 'R', 'P', 'Y')
        self.tree = ttk.Treeview(list_frm, columns=cols, show='headings', height=5)
        for col in cols:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=68, anchor='center')
        self.tree.column('name',  width=82)
        self.tree.column('model', width=76)
        self.tree.pack(fill='x')

        # ── Log ────────────────────────────────────────────────────────────────
        log_frm = ttk.LabelFrame(left, text='Log', padding=4)
        log_frm.pack(fill='x', pady=(4, 0))
        self.log_box = scrolledtext.ScrolledText(log_frm, height=6, width=58, state='disabled')
        self.log_box.pack()

        # Right column: survey canvas
        right = ttk.Frame(self.root)
        right.grid(row=0, column=1, padx=(4, 10), pady=8, sticky='n')

        canvas_frm = ttk.LabelFrame(right, text='Survey Area View', padding=6)
        canvas_frm.pack()

        # Center + radius controls
        ctrl = ttk.Frame(canvas_frm)
        ctrl.pack(fill='x', pady=(0, 4))
        for col, (lbl, val) in enumerate([('Center X', '5.0'), ('Center Y', '-5.0'), ('Radius', '8.0')]):
            ttk.Label(ctrl, text=f'{lbl}:').grid(row=0, column=col * 2, sticky='e', padx=(6, 2))
            var = tk.StringVar(value=val)
            ttk.Entry(ctrl, textvariable=var, width=6).grid(row=0, column=col * 2 + 1)
            if lbl == 'Center X':
                self.view_cx = var
            elif lbl == 'Center Y':
                self.view_cy = var
            else:
                self.view_r = var
        ttk.Button(ctrl, text='Refresh', command=self._draw_canvas).grid(
            row=0, column=6, padx=(8, 0))

        self.canvas = tk.Canvas(
            canvas_frm, width=CANVAS_SIZE, height=CANVAS_SIZE,
            bg=COLORS['bg'], highlightthickness=1, highlightbackground='#4a4a8a'
        )
        self.canvas.pack()

        # Legend
        leg = ttk.Frame(canvas_frm)
        leg.pack(fill='x', pady=(4, 0))
        for color, label in [
            (COLORS['rock'],     'Rock (config)'),
            (COLORS['spawned'],  'Spawned'),
            (COLORS['terrain'],  'Terrain'),
            (COLORS['selected'], 'Selected'),
        ]:
            tk.Label(leg, bg=color, width=2).pack(side='left', padx=(6, 2))
            ttk.Label(leg, text=label).pack(side='left', padx=(0, 6))

        # Populate first rock and draw canvas
        if self.rock_configs:
            self._populate_fields(self.rock_configs[0])
        self._draw_canvas()

    # ── Canvas helpers ─────────────────────────────────────────────────────────

    def _world_to_canvas(self, wx: float, wy: float) -> tuple[float, float]:
        try:
            cx = float(self.view_cx.get())
            cy = float(self.view_cy.get())
            r  = float(self.view_r.get())
        except ValueError:
            return CANVAS_SIZE / 2, CANVAS_SIZE / 2
        scale = (CANVAS_SIZE * 0.44) / r
        px = CANVAS_SIZE / 2 - (wy - cy) * scale   # +Y is left, so flipped on canvas
        py = CANVAS_SIZE / 2 - (wx - cx) * scale   # +X is up on canvas
        return px, py

    def _draw_canvas(self):
        self.canvas.delete('all')

        # Grid lines
        try:
            cx = float(self.view_cx.get())
            cy = float(self.view_cy.get())
            r  = float(self.view_r.get())
        except ValueError:
            return

        for xi in range(int(cx - r) - 1, int(cx + r) + 2):
            p1 = self._world_to_canvas(xi, cy - r - 1)
            p2 = self._world_to_canvas(xi, cy + r + 1)
            self.canvas.create_line(*p1, *p2, fill=COLORS['grid'], width=1)
        for yi in range(int(cy - r) - 1, int(cy + r) + 2):
            p1 = self._world_to_canvas(cx - r - 1, yi)
            p2 = self._world_to_canvas(cx + r + 1, yi)
            self.canvas.create_line(*p1, *p2, fill=COLORS['grid'], width=1)

        # Survey boundary circle
        px, py = self._world_to_canvas(cx, cy)
        scale  = (CANVAS_SIZE * 0.44) / r
        pr     = r * scale
        self.canvas.create_oval(
            px - pr, py - pr, px + pr, py + pr,
            outline=COLORS['circle'], width=2, dash=(6, 4)
        )

        # Center cross
        self.canvas.create_line(px - 6, py, px + 6, py, fill=COLORS['circle'], width=2)
        self.canvas.create_line(px, py - 6, px, py + 6, fill=COLORS['circle'], width=2)

        # Axis labels
        self.canvas.create_text(CANVAS_SIZE - 6, CANVAS_SIZE / 2 + 8,
                                 text=f'Y={cy - r:.0f}', fill=COLORS['label'],
                                 anchor='e', font=('Helvetica', 8))
        self.canvas.create_text(6, CANVAS_SIZE / 2 + 8,
                                 text=f'+Y={cy + r:.0f}', fill=COLORS['label'],
                                 anchor='w', font=('Helvetica', 8))
        self.canvas.create_text(CANVAS_SIZE / 2, 6,
                                 text=f'+X={cx + r:.0f}', fill=COLORS['label'],
                                 anchor='n', font=('Helvetica', 8))
        self.canvas.create_text(CANVAS_SIZE / 2, CANVAS_SIZE - 4,
                                 text=f'X={cx - r:.0f}', fill=COLORS['label'],
                                 anchor='s', font=('Helvetica', 8))

        # Rock dots
        selected_name = self.rock_var.get()
        for cfg in self.rock_configs:
            name = cfg['name']
            rx, ry = cfg.get('x', 0.0), cfg.get('y', 0.0)

            # If this is the currently selected rock, use the live pose fields
            if name == selected_name:
                try:
                    rx = float(self.pose_vars['x'].get())
                    ry = float(self.pose_vars['y'].get())
                except ValueError:
                    pass

            px_r, py_r = self._world_to_canvas(rx, ry)

            is_selected = (name == selected_name)
            is_spawned  = (name in self.spawned)
            is_terrain  = (name == 'terrain')

            color = (COLORS['selected'] if is_selected
                     else COLORS['spawned'] if is_spawned
                     else COLORS['terrain'] if is_terrain
                     else COLORS['rock'])
            dot_r = DOT_SEL if is_selected else DOT_RADIUS

            self.canvas.create_oval(
                px_r - dot_r, py_r - dot_r,
                px_r + dot_r, py_r + dot_r,
                fill=color, outline='white', width=1,
                tags=(f'dot_{name}',)
            )
            self.canvas.create_text(
                px_r + dot_r + 3, py_r - dot_r,
                text=name, fill=COLORS['label'],
                anchor='w', font=('Helvetica', 8),
                tags=(f'lbl_{name}',)
            )

    def _update_selected_dot(self):
        """Redraw only when X/Y fields change — cheap redraw."""
        self._draw_canvas()

    # ── Event handlers ─────────────────────────────────────────────────────────

    def _on_rock_selected(self):
        name = self.rock_var.get()
        cfg  = next((c for c in self.rock_configs if c['name'] == name), None)
        if cfg:
            self._populate_fields(cfg)
        self._draw_canvas()

    def _populate_fields(self, cfg: dict):
        for key in ['x', 'y', 'z', 'R', 'P', 'Y']:
            self.pose_vars[key].set(str(cfg.get(key, 0.0)))

    def _on_spawn(self):
        name = self.rock_var.get()
        cfg  = next((c for c in self.rock_configs if c['name'] == name), None)
        if not cfg:
            return

        try:
            x = float(self.pose_vars['x'].get())
            y = float(self.pose_vars['y'].get())
            z = float(self.pose_vars['z'].get())
            R = float(self.pose_vars['R'].get())
            P = float(self.pose_vars['P'].get())
            Y = float(self.pose_vars['Y'].get())
        except ValueError:
            messagebox.showerror('Error', 'All pose fields must be numbers')
            return

        if name in self.spawned:
            messagebox.showerror('Error', f'"{name}" is already spawned. Delete it first.')
            return

        self._log(f'Spawning {name} ({cfg["model"]}) at ({x},{y},{z}) R={R} P={P} Y={Y} …')
        ok, out = spawn_rock(name, cfg['model'], x, y, z, R, P, Y)

        if ok:
            spawned_cfg = {**cfg, 'x': x, 'y': y, 'z': z, 'R': R, 'P': P, 'Y': Y}
            self.spawned[name] = spawned_cfg
            self.tree.insert('', 'end', iid=name,
                             values=(name, cfg['model'], x, y, z, R, P, Y))
            for c in self.rock_configs:
                if c['name'] == name:
                    c.update({'x': x, 'y': y, 'z': z, 'R': R, 'P': P, 'Y': Y})
            self._log(f'  ✓ {name} spawned.')
        else:
            self._log(f'  ✗ Failed:\n    {out}')

        self._draw_canvas()

    def _on_delete(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo('Info', 'Select a rock from the list first')
            return
        for name in sel:
            self._log(f'Deleting {name} (pausing physics) …')
            ok, out = delete_rock(name)
            if ok:
                self.spawned.pop(name, None)
                if self.tree.exists(name):
                    self.tree.delete(name)
                self._log(f'  ✓ {name} deleted.')
            else:
                self._log(f'  ✗ Failed:\n    {out}')
        self._draw_canvas()

    def _on_delete_all(self):
        for name in list(self.spawned):
            self._log(f'Deleting {name} …')
            delete_rock(name)
            if self.tree.exists(name):
                self.tree.delete(name)
        self.spawned.clear()
        self._log('All rocks deleted.')
        self._draw_canvas()

    def _on_export(self):
        lines = ['ROCK_CONFIGS = [']
        for i, cfg in enumerate(self.rock_configs):
            tag = 'Terrain' if cfg['name'] == 'terrain' else f'Scene {i}'
            lines.append(f"    # {tag}")
            lines.append(
                f"    {{'name': '{cfg['name']}',  'model': '{cfg['model']}',  "
                f"'x': {cfg['x']}, 'y': {cfg['y']}, 'z': {cfg['z']}, "
                f"'R': {cfg['R']}, 'P': {cfg['P']}, 'Y': {cfg['Y']}}},"
            )
        lines.append(']')
        block = '\n'.join(lines)

        win = tk.Toplevel(self.root)
        win.title('ROCK_CONFIGS — paste into drone_mapping.launch.py')
        txt = scrolledtext.ScrolledText(win, width=90, height=22)
        txt.pack(padx=8, pady=8)
        txt.insert('1.0', block)
        ttk.Button(
            win, text='Copy to clipboard',
            command=lambda: (win.clipboard_clear(), win.clipboard_append(block),
                             self._log('Copied to clipboard.'))
        ).pack(pady=4)

    # ── Services ───────────────────────────────────────────────────────────────

    def _start_services(self):
        self._log(f'Starting Gazebo: {GZ_WORLD}')
        self._gz_proc = start_gazebo(GZ_WORLD)
        self._log(f'  Gazebo launched (PID {self._gz_proc.pid}). Waiting 4s …')
        self.root.after(4000, self._delayed_bridge)

    def _delayed_bridge(self):
        self._log('Starting ros_gz_bridge (/clock) …')
        self._bridge_proc = start_bridge()
        self._log(f'  Bridge launched (PID {self._bridge_proc.pid}).')
        self._log('Ready. Select a rock and hit Spawn.')

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        self.log_box.config(state='normal')
        self.log_box.insert('end', msg + '\n')
        self.log_box.see('end')
        self.log_box.config(state='disabled')

    def _on_close(self):
        self._on_delete_all()
        if self._bridge_proc:
            self._bridge_proc.terminate()
        if self._gz_proc:
            self._gz_proc.terminate()
        self.root.destroy()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    root = tk.Tk()
    RockSpawnerGUI(root)
    root.mainloop()

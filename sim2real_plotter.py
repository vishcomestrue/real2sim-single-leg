#!/usr/bin/env python3
"""
Sim2Real Plotter
Real-time visualization of simulation vs reality for motor positions
"""

import matplotlib.pyplot as plt
from collections import deque
from datetime import datetime
import numpy as np


class Sim2RealPlotter:
    """Real-time plotter for comparing simulation and real motor positions."""

    def __init__(self, joint_names=["hipY", "knee"], window_size=500, update_interval=5):
        """
        Initialize the sim2real plotter.

        Args:
            joint_names: List of joint names (should be 2 joints)
            window_size: Number of data points to display (rolling window)
            update_interval: Update plot every N data points (for performance)
        """
        self.joint_names = joint_names
        self.window_size = window_size
        self.update_interval = update_interval
        self.update_counter = 0

        # Data storage (using deque for efficient rolling window)
        self.time_data = deque(maxlen=window_size)

        # For each joint: qpos, ctrl, motor_pos
        self.qpos_data = {name: deque(maxlen=window_size) for name in joint_names}
        self.ctrl_data = {name: deque(maxlen=window_size) for name in joint_names}
        self.motor_data = {name: deque(maxlen=window_size) for name in joint_names}

        # Start time for relative timestamps
        self.start_time = None

        # Setup the plot
        self._setup_plot()

    def _setup_plot(self):
        """Setup the matplotlib figure with 3x2 subplots."""
        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(3, 2, figsize=(14, 10))
        self.fig.suptitle('Sim2Real Motor Comparison', fontsize=16, fontweight='bold')

        # Column titles
        self.axes[0, 0].set_title(f'{self.joint_names[0]} (Hip)', fontweight='bold')
        self.axes[0, 1].set_title(f'{self.joint_names[1]} (Knee)', fontweight='bold')

        # Row 1: Position Tracking
        for i, name in enumerate(self.joint_names):
            ax = self.axes[0, i]
            ax.set_ylabel('Position (rad)', fontsize=10)
            ax.grid(True, alpha=0.3)

        # Row 2: Tracking Error (Command vs Reality)
        for i, name in enumerate(self.joint_names):
            ax = self.axes[1, i]
            ax.set_ylabel('Tracking Error (rad)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)

        # Row 3: Sim2Real Gap
        for i, name in enumerate(self.joint_names):
            ax = self.axes[2, i]
            ax.set_ylabel('Sim2Real Gap (rad)', fontsize=10)
            ax.set_xlabel('Time (s)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)

        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.001)

        # Store line objects for efficient updates
        self.lines = {}
        for i, name in enumerate(self.joint_names):
            # Row 1: Position tracking (3 lines per subplot)
            self.lines[f'{name}_qpos'], = self.axes[0, i].plot([], [], 'b--', linewidth=2, label='Sim (qpos)')
            self.lines[f'{name}_motor'], = self.axes[0, i].plot([], [], 'r-', linewidth=2, label='Reality (motor)')
            self.lines[f'{name}_ctrl'], = self.axes[0, i].plot([], [], 'g:', linewidth=1.5, alpha=0.7, label='Command (ctrl)')
            self.axes[0, i].legend(loc='upper right', fontsize=8)

            # Row 2: Tracking error
            self.lines[f'{name}_track_err'], = self.axes[1, i].plot([], [], 'purple', linewidth=1.5, label='ctrl - motor')
            self.axes[1, i].legend(loc='upper right', fontsize=8)

            # Row 3: Sim2Real gap
            self.lines[f'{name}_sim2real'], = self.axes[2, i].plot([], [], 'orange', linewidth=1.5, label='qpos - motor')
            self.axes[2, i].legend(loc='upper right', fontsize=8)

    def update(self, timestamp, qpos, ctrl, motor_pos):
        """
        Update the plot with new data.

        Args:
            timestamp: Current timestamp in seconds
            qpos: Dict {joint_name: position} or list [hip, knee] - simulated positions
            ctrl: Dict {joint_name: control} or list [hip, knee] - control commands
            motor_pos: Dict {motor_id: position} or list [hip, knee] or None - real motor positions
        """
        # Initialize start time on first update
        if self.start_time is None:
            self.start_time = timestamp

        relative_time = timestamp - self.start_time

        # Convert to dict if list
        if isinstance(qpos, (list, tuple)):
            qpos = {name: pos for name, pos in zip(self.joint_names, qpos)}
        if isinstance(ctrl, (list, tuple)):
            ctrl = {name: pos for name, pos in zip(self.joint_names, ctrl)}

        # Handle motor positions (might be None or dict with motor_ids)
        motor_dict = {}
        if motor_pos is not None:
            if isinstance(motor_pos, (list, tuple)):
                motor_dict = {name: pos for name, pos in zip(self.joint_names, motor_pos)}
            elif isinstance(motor_pos, dict):
                # Assume motor_ids [1, 2] map to joint_names
                # Convert motor_id keys to joint_name keys
                motor_ids = sorted(motor_pos.keys())
                for motor_id, name in zip(motor_ids, self.joint_names):
                    motor_dict[name] = motor_pos[motor_id]
        else:
            # No motor data, use NaN
            motor_dict = {name: float('nan') for name in self.joint_names}

        # Store data
        self.time_data.append(relative_time)
        for name in self.joint_names:
            self.qpos_data[name].append(qpos.get(name, float('nan')))
            self.ctrl_data[name].append(ctrl.get(name, float('nan')))
            self.motor_data[name].append(motor_dict.get(name, float('nan')))

        # Update plot every N points for performance
        self.update_counter += 1
        if self.update_counter >= self.update_interval:
            self.update_counter = 0
            self._redraw()

    def _redraw(self):
        """Redraw all plots with current data."""
        if len(self.time_data) == 0:
            return

        time_array = np.array(self.time_data)

        for i, name in enumerate(self.joint_names):
            qpos_array = np.array(self.qpos_data[name])
            ctrl_array = np.array(self.ctrl_data[name])
            motor_array = np.array(self.motor_data[name])

            # Row 1: Position tracking
            self.lines[f'{name}_qpos'].set_data(time_array, qpos_array)
            self.lines[f'{name}_motor'].set_data(time_array, motor_array)
            self.lines[f'{name}_ctrl'].set_data(time_array, ctrl_array)

            # Row 2: Tracking error (ctrl - motor)
            track_err = ctrl_array - motor_array
            self.lines[f'{name}_track_err'].set_data(time_array, track_err)

            # Row 3: Sim2Real gap (qpos - motor)
            sim2real = qpos_array - motor_array
            self.lines[f'{name}_sim2real'].set_data(time_array, sim2real)

            # Auto-scale axes
            for row in range(3):
                self.axes[row, i].relim()
                self.axes[row, i].autoscale_view()

        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def force_update(self):
        """Force an immediate plot update (useful for final update before save)."""
        self._redraw()

    def save(self, filename=None):
        """
        Save the current plot to a file.

        Args:
            filename: Output filename. If None, auto-generates with timestamp.
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"sim2real_comparison_{timestamp}.png"

        # Force final update
        self.force_update()

        # Reset all axes to show complete data (override any manual zoom/pan)
        for i in range(len(self.joint_names)):
            for row in range(3):
                ax = self.axes[row, i]
                # Clear autoscale flags and reset to show all data
                ax.autoscale(enable=True, axis='both')
                ax.relim()
                ax.autoscale_view()

        # Redraw with full data view
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Save with high DPI
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"\n[Sim2Real Plotter] Plot saved to: {filename}")

    def close(self):
        """Close the plot window."""
        plt.close(self.fig)


if __name__ == "__main__":
    """Test the plotter with simulated data."""
    import time
    import math

    print("Testing Sim2Real Plotter")
    print("Close the window or press Ctrl+C to stop")

    plotter = Sim2RealPlotter(joint_names=["hipY", "knee"])

    try:
        t = 0
        start = time.time()
        while True:
            current_time = time.time()

            # Simulate some data
            hip_sim = 0.5 * math.sin(t)
            knee_sim = 0.8 * math.cos(t * 0.7)

            hip_ctrl = hip_sim + 0.05 * math.sin(t * 5)  # Command with small oscillation
            knee_ctrl = knee_sim + 0.05 * math.cos(t * 5)

            # Motor follows with some lag and error
            hip_motor = hip_ctrl * 0.95 + 0.02 * math.sin(t * 3)
            knee_motor = knee_ctrl * 0.95 - 0.02 * math.cos(t * 3)

            qpos = {"hipY": hip_sim, "knee": knee_sim}
            ctrl = {"hipY": hip_ctrl, "knee": knee_ctrl}
            motor = {"hipY": hip_motor, "knee": knee_motor}

            plotter.update(current_time, qpos, ctrl, motor)

            time.sleep(0.05)  # 20 Hz
            t += 0.05

    except KeyboardInterrupt:
        print("\n\nStopping test...")

    finally:
        plotter.save()
        plotter.close()
        print("Test completed")

#!/usr/bin/env python3
"""
Motor Slider GUI Controller

Interactive GUI with sliders to control Dynamixel motors in real-time.
Uses the Fast Dynamixel Controller for efficient communication.
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import math
from fast_dynamixel_controller import FastDynamixelController


class MotorSliderGUI:
    """GUI application for controlling motors with sliders."""

    def __init__(self, port="/dev/ttyUSB0", motor_ids=[1, 2],
                 joint_names=["Hip", "Knee"], baudrate=2000000,
                 control_frequency=50):
        """
        Initialize the motor slider GUI.

        Args:
            port: Serial port path
            motor_ids: List of motor IDs
            joint_names: Names for each joint/motor
            baudrate: Communication baudrate
            control_frequency: Control loop frequency in Hz
        """
        self.port = port
        self.motor_ids = motor_ids
        self.joint_names = joint_names
        self.baudrate = baudrate
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency

        # Motor controller
        self.controller = None
        self.connected = False

        # Control state
        self.running = False
        self.control_thread = None

        # Target positions (radians) - updated by sliders
        self.target_positions = {name: 0.0 for name in joint_names}

        # Actual positions (radians) - read from motors
        self.actual_positions = {name: 0.0 for name in joint_names}

        # Statistics
        self.loop_count = 0
        self.loop_times = []  # Total cycle time (including sleep)
        self.work_times = []  # Just work time (write + read)
        self.last_stats_update = time.time()

        # Position limits (radians)
        self.position_limits = (-math.pi/2, math.pi/2)  # -90° to +90°

        # Create GUI
        self.create_gui()

    def create_gui(self):
        """Create the GUI window and widgets."""
        self.root = tk.Tk()
        self.root.title("Motor Slider Controller")
        self.root.geometry("800x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Title
        title = ttk.Label(main_frame, text="Motor Slider Controller",
                         font=("Arial", 16, "bold"))
        title.grid(row=0, column=0, columnspan=3, pady=10)

        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)

        ttk.Label(conn_frame, text=f"Port: {self.port}").grid(row=0, column=0, padx=5)
        ttk.Label(conn_frame, text=f"Baudrate: {self.baudrate}").grid(row=0, column=1, padx=5)
        ttk.Label(conn_frame, text=f"Motors: {self.motor_ids}").grid(row=0, column=2, padx=5)

        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=10)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=4, padx=5)

        # Control frequency
        freq_frame = ttk.Frame(conn_frame)
        freq_frame.grid(row=1, column=0, columnspan=5, pady=5)

        ttk.Label(freq_frame, text="Control Frequency:").grid(row=0, column=0, padx=5)
        self.freq_var = tk.StringVar(value=str(self.control_frequency))
        self.freq_spinbox = ttk.Spinbox(freq_frame, from_=10, to=200, textvariable=self.freq_var,
                                        width=10, command=self.update_frequency)
        self.freq_spinbox.grid(row=0, column=1, padx=5)
        # Bind Enter key and focus out to update frequency
        self.freq_spinbox.bind('<Return>', lambda e: self.update_frequency())
        self.freq_spinbox.bind('<FocusOut>', lambda e: self.update_frequency())
        ttk.Label(freq_frame, text="Hz").grid(row=0, column=2)

        # Motor sliders
        self.sliders = {}
        self.target_labels = {}
        self.actual_labels = {}
        self.error_labels = {}

        for i, (motor_id, joint_name) in enumerate(zip(self.motor_ids, self.joint_names)):
            # Frame for this motor
            motor_frame = ttk.LabelFrame(main_frame, text=f"{joint_name} (Motor ID {motor_id})",
                                        padding="15")
            motor_frame.grid(row=2+i, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)

            # Slider
            slider = tk.Scale(motor_frame, from_=math.degrees(self.position_limits[0]),
                            to=math.degrees(self.position_limits[1]),
                            orient=tk.HORIZONTAL, length=400, resolution=0.1,
                            command=lambda val, name=joint_name: self.on_slider_change(name, val))
            slider.set(0.0)  # Start at center
            slider.grid(row=0, column=0, columnspan=4, padx=10, pady=5)
            self.sliders[joint_name] = slider

            # Labels
            ttk.Label(motor_frame, text="Target:").grid(row=1, column=0, sticky=tk.W, padx=5)
            target_label = ttk.Label(motor_frame, text="0.000 rad (0.0°)",
                                    font=("Arial", 10, "bold"))
            target_label.grid(row=1, column=1, sticky=tk.W, padx=5)
            self.target_labels[joint_name] = target_label

            ttk.Label(motor_frame, text="Actual:").grid(row=2, column=0, sticky=tk.W, padx=5)
            actual_label = ttk.Label(motor_frame, text="0.000 rad (0.0°)")
            actual_label.grid(row=2, column=1, sticky=tk.W, padx=5)
            self.actual_labels[joint_name] = actual_label

            ttk.Label(motor_frame, text="Error:").grid(row=3, column=0, sticky=tk.W, padx=5)
            error_label = ttk.Label(motor_frame, text="0.000 rad")
            error_label.grid(row=3, column=1, sticky=tk.W, padx=5)
            self.error_labels[joint_name] = error_label

            # Zero button
            zero_btn = ttk.Button(motor_frame, text="Zero",
                                 command=lambda name=joint_name: self.zero_motor(name))
            zero_btn.grid(row=1, column=2, padx=10)

        # Statistics frame
        stats_frame = ttk.LabelFrame(main_frame, text="Statistics", padding="10")
        stats_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)

        self.freq_label = ttk.Label(stats_frame, text="Actual Frequency: 0.0 Hz")
        self.freq_label.grid(row=0, column=0, padx=10)

        self.loop_time_label = ttk.Label(stats_frame, text="Cycle Time: 0.0 ms")
        self.loop_time_label.grid(row=0, column=1, padx=10)

        self.work_time_label = ttk.Label(stats_frame, text="Work Time: 0.0 ms")
        self.work_time_label.grid(row=0, column=2, padx=10)

        self.cpu_util_label = ttk.Label(stats_frame, text="CPU Usage: 0%")
        self.cpu_util_label.grid(row=1, column=0, padx=10)

        self.loop_count_label = ttk.Label(stats_frame, text="Loops: 0")
        self.loop_count_label.grid(row=1, column=1, padx=10)

        # Console output
        console_frame = ttk.LabelFrame(main_frame, text="Console", padding="10")
        console_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)

        self.console = tk.Text(console_frame, height=8, width=80, state='disabled')
        self.console.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        scrollbar = ttk.Scrollbar(console_frame, orient=tk.VERTICAL, command=self.console.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.console['yscrollcommand'] = scrollbar.set

        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(5, weight=1)
        console_frame.columnconfigure(0, weight=1)
        console_frame.rowconfigure(0, weight=1)

    def log(self, message):
        """Log a message to the console."""
        self.console.configure(state='normal')
        self.console.insert(tk.END, message + "\n")
        self.console.see(tk.END)
        self.console.configure(state='disabled')

    def on_slider_change(self, joint_name, value):
        """Handle slider value change."""
        degrees = float(value)
        radians = math.radians(degrees)
        self.target_positions[joint_name] = radians
        self.target_labels[joint_name].config(text=f"{radians:.3f} rad ({degrees:.1f}°)")

    def zero_motor(self, joint_name):
        """Set motor target to zero position."""
        self.sliders[joint_name].set(0.0)
        self.target_positions[joint_name] = 0.0
        self.target_labels[joint_name].config(text="0.000 rad (0.0°)")

    def update_frequency(self):
        """Update control frequency from spinbox."""
        try:
            new_freq = int(self.freq_var.get())
            if 10 <= new_freq <= 200:
                self.control_frequency = new_freq
                self.control_period = 1.0 / new_freq
                self.log(f"Control frequency updated to {new_freq} Hz")
        except ValueError:
            pass

    def toggle_connection(self):
        """Connect or disconnect from motors."""
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        """Connect to motors and start control loop."""
        self.log("Connecting to motors...")

        try:
            # Create controller
            # Note: use_sync_read=False for better compatibility (sequential reads)
            self.controller = FastDynamixelController(
                port=self.port,
                motor_ids=self.motor_ids,
                joint_names=self.joint_names,
                baudrate=self.baudrate,
                use_sync_read=True  # Use sequential reads for reliability
            )

            # Connect
            if not self.controller.connect():
                self.log("Failed to connect to motors!")
                return

            self.log("Connected successfully!")

            # Calibrate at zero
            self.log("Calibrating motors at zero position...")
            if self.controller.calibrate([0.0] * len(self.motor_ids)):
                self.log("Calibration complete!")
            else:
                self.log("Calibration failed!")
                return

            # Update GUI
            self.connected = True
            self.status_label.config(text="Connected", foreground="green")
            self.connect_button.config(text="Disconnect")

            # Start control loop
            self.running = True
            self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
            self.control_thread.start()
            self.log(f"Control loop started at {self.control_frequency} Hz")

        except Exception as e:
            self.log(f"Connection error: {e}")

    def disconnect(self):
        """Disconnect from motors and stop control loop."""
        self.log("Disconnecting...")

        # Stop control loop
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=2.0)

        # Disconnect controller
        if self.controller:
            self.controller.disconnect()

        # Update GUI
        self.connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_button.config(text="Connect")
        self.log("Disconnected")

    def control_loop(self):
        """Main control loop running in separate thread."""
        self.log("Control loop running...")
        self.loop_count = 0
        self.loop_times = []
        self.work_times = []

        # Track previous loop start for accurate cycle time measurement
        previous_loop_start = None

        while self.running:
            loop_start = time.time()

            # Calculate actual cycle time (from previous iteration start to current start)
            if previous_loop_start is not None:
                cycle_time = (loop_start - previous_loop_start) * 1000  # ms
                self.loop_times.append(cycle_time)
                self.loop_count += 1

            try:
                # Get target positions from sliders
                target_list = [self.target_positions[name] for name in self.joint_names]

                # Write positions to motors
                self.controller.write_positions(target_list)

                # Read actual positions from motors
                positions = self.controller.read_positions_radians()

                if positions:
                    # Update actual positions
                    for motor_id, joint_name in zip(self.motor_ids, self.joint_names):
                        if motor_id in positions:
                            self.actual_positions[joint_name] = positions[motor_id]

                    # Update GUI (schedule on main thread)
                    self.root.after(0, self.update_position_displays)

                # Calculate work time (communication + processing time)
                loop_end = time.time()
                work_time = (loop_end - loop_start) * 1000  # ms
                self.work_times.append(work_time)

                # Update statistics every 0.5 seconds
                if time.time() - self.last_stats_update > 0.5:
                    self.root.after(0, self.update_statistics)
                    self.last_stats_update = time.time()

                # Sleep to maintain desired frequency
                elapsed = loop_end - loop_start
                sleep_time = max(0, self.control_period - elapsed)
                time.sleep(sleep_time)

                # Store this loop start for next iteration's cycle time calculation
                previous_loop_start = loop_start

            except Exception as e:
                self.log(f"Control loop error: {e}")
                time.sleep(0.1)
                # Reset timing after error
                previous_loop_start = None

        self.log("Control loop stopped")

    def update_position_displays(self):
        """Update position displays in GUI (runs on main thread)."""
        for joint_name in self.joint_names:
            actual_rad = self.actual_positions[joint_name]
            actual_deg = math.degrees(actual_rad)
            target_rad = self.target_positions[joint_name]
            error_rad = target_rad - actual_rad

            self.actual_labels[joint_name].config(
                text=f"{actual_rad:.3f} rad ({actual_deg:.1f}°)"
            )
            self.error_labels[joint_name].config(
                text=f"{error_rad:.3f} rad"
            )

    def update_statistics(self):
        """Update statistics displays (runs on main thread)."""
        if self.loop_times and self.work_times:
            # Calculate averages over last 100 samples
            avg_cycle_time = sum(self.loop_times[-100:]) / min(len(self.loop_times), 100)
            avg_work_time = sum(self.work_times[-100:]) / min(len(self.work_times), 100)

            # Actual frequency based on total cycle time (work + sleep)
            actual_freq = 1000.0 / avg_cycle_time if avg_cycle_time > 0 else 0

            # CPU utilization = (work_time / cycle_time) * 100
            cpu_usage = (avg_work_time / avg_cycle_time * 100) if avg_cycle_time > 0 else 0

            self.freq_label.config(text=f"Actual Frequency: {actual_freq:.1f} Hz")
            self.loop_time_label.config(text=f"Cycle Time: {avg_cycle_time:.2f} ms")
            self.work_time_label.config(text=f"Work Time: {avg_work_time:.2f} ms")
            self.cpu_util_label.config(text=f"CPU Usage: {cpu_usage:.1f}%")
            self.loop_count_label.config(text=f"Loops: {self.loop_count}")

    def on_closing(self):
        """Handle window close event."""
        if self.connected:
            self.disconnect()
        self.root.destroy()

    def run(self):
        """Start the GUI application."""
        self.log("Motor Slider GUI ready")
        self.log(f"Configure settings and click 'Connect' to start")
        self.root.mainloop()


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Motor Slider GUI Controller")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                       help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--motor-ids", type=int, nargs="+", default=[1, 2],
                       help="Motor IDs (default: 1 2)")
    parser.add_argument("--joint-names", type=str, nargs="+", default=["Hip", "Knee"],
                       help="Joint names (default: Hip Knee)")
    parser.add_argument("--baudrate", type=int, default=2000000,
                       help="Baudrate (default: 2000000)")
    parser.add_argument("--frequency", type=int, default=50,
                       help="Control frequency in Hz (default: 50)")

    args = parser.parse_args()

    # Create and run GUI
    app = MotorSliderGUI(
        port=args.port,
        motor_ids=args.motor_ids,
        joint_names=args.joint_names,
        baudrate=args.baudrate,
        control_frequency=args.frequency
    )
    app.run()


if __name__ == "__main__":
    main()

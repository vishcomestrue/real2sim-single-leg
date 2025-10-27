#!/usr/bin/env python3
"""
MuJoCo Test Rig Loader
Loads and visualizes the test_rig.xml model in MuJoCo
"""

import mujoco
import mujoco.viewer
import numpy as np
import os
import argparse
import time
from position_monitor import PositionMonitor
from fast_dynamixel_controller import FastDynamixelController
from sim2real_plotter import Sim2RealPlotter

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="MuJoCo Test Rig with optional Dynamixel motor sync")
    parser.add_argument("--motors", action="store_true", help="Enable real Dynamixel motor synchronization")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port for Dynamixel motors (default: /dev/ttyUSB0)")
    parser.add_argument("--motor-ids", type=int, nargs=2, default=[1, 2], help="Motor IDs for hipY and knee (default: 1 2)")
    parser.add_argument("--baudrate", type=int, default=2000000, help="Motor baudrate (default: 2000000 = 2Mbps)")
    parser.add_argument("--control-rate", type=float, default=20.0, help="Control loop rate in Hz (default: 20)")
    parser.add_argument("--plot", action="store_true", help="Enable real-time sim2real plotting")
    args = parser.parse_args()

    # Get the path to the XML file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(script_dir, "xmls", "scene.xml")

    # Check if XML file exists
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"XML file not found at: {xml_path}")

    print(f"Loading MuJoCo model from: {xml_path}")

    # Load the MuJoCo model
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print(f"Model loaded successfully!")
    print(f"Number of DOFs: {model.nv}")
    print(f"Number of actuators: {model.nu}")
    print(f"Number of bodies: {model.nbody}")

    # Get actuator IDs for control monitoring
    hip_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "hipY")
    knee_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee")

    if hip_actuator_id < 0 or knee_actuator_id < 0:
        print("Warning: Could not find hipY or knee actuators")
    else:
        print(f"Found actuators - hipY: {hip_actuator_id}, knee: {knee_actuator_id}")

    # Get joint IDs for reading qpos
    hip_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hipY")
    knee_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "knee")

    if hip_joint_id < 0 or knee_joint_id < 0:
        print("Warning: Could not find hipY or knee joints")
    else:
        print(f"Found joints - hipY: {hip_joint_id}, knee: {knee_joint_id}")

    # Initialize position monitor
    position_monitor = PositionMonitor(["hipY", "knee"])

    # Initialize plotter if enabled
    plotter = None
    if args.plot:
        print("\n[Sim2Real Plotter] Initializing real-time plotting...")
        plotter = Sim2RealPlotter(joint_names=["hipY", "knee"])
        print("[Sim2Real Plotter] Enabled")
    else:
        print("\n[Sim2Real Plotter] Disabled. Use --plot flag to enable real-time plotting.")

    # Reset to keyframe "1" if it exists
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "1")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        print("Reset to keyframe '1'")

    # Initialize Dynamixel controller if enabled
    dynamixel_controller = None
    if args.motors:
        print("\n[Motor Sync] Initializing Dynamixel motors...")
        dynamixel_controller = FastDynamixelController(
            port=args.port,
            motor_ids=args.motor_ids,
            joint_names=["hipY", "knee"],
            baudrate=args.baudrate,
            use_sync_read=True  # Use GroupSyncRead for better performance
        )
        if not dynamixel_controller.connect():
            print("[Motor Sync] Failed to connect to motors. Continuing without motor sync.")
            dynamixel_controller = None
        else:
            print("[Motor Sync] Motors connected!")

            # Calibrate motors using initial control values from the keyframe
            if hip_actuator_id >= 0 and knee_actuator_id >= 0:
                initial_hip_ctrl = data.ctrl[hip_actuator_id]
                initial_knee_ctrl = data.ctrl[knee_actuator_id]

                if not dynamixel_controller.calibrate([initial_hip_ctrl, initial_knee_ctrl]):
                    print("[Motor Sync] Calibration failed. Motor positions may not match simulation.")
    else:
        print("\n[Motor Sync] Disabled. Use --motors flag to enable real motor synchronization.")

    # Launch the interactive viewer
    print("\nLaunching MuJoCo viewer...")
    print("Controls:")
    print("  - Double-click: Select body")
    print("  - Right-click drag: Rotate view")
    print("  - Scroll: Zoom")
    print("  - Ctrl+Right-click drag: Pan")
    print("  - Space: Pause/Resume")
    print("  - Backspace: Reset simulation")
    print("  - Esc: Exit viewer")

    # Setup control timing
    control_dt = 1.0 / args.control_rate
    steps_per_control = max(1, int(control_dt / model.opt.timestep))
    last_control_time = time.time()
    print(f"\n[Control Rate] {args.control_rate} Hz ({control_dt*1000:.1f}ms period)")
    print(f"[Control Rate] Running {steps_per_control} simulation steps per control cycle")
    print(f"[MuJoCo] Simulation timestep: {model.opt.timestep*1000:.2f}ms\n")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Simulation loop
            while viewer.is_running():
                current_time = time.time()

                # Step the simulation (multiple times to match control rate)
                for _ in range(steps_per_control):
                    mujoco.mj_step(model, data)

                # Control cycle at specified rate
                if current_time - last_control_time >= control_dt:
                    if hip_actuator_id >= 0 and knee_actuator_id >= 0 and hip_joint_id >= 0 and knee_joint_id >= 0:
                        # 1. Read MuJoCo joint positions (qpos)
                        hip_qpos = data.qpos[hip_joint_id]
                        knee_qpos = data.qpos[knee_joint_id]

                        # 2. Read MuJoCo control commands (from sliders)
                        hip_ctrl = data.ctrl[hip_actuator_id]
                        knee_ctrl = data.ctrl[knee_actuator_id]

                        # 3. Read real motor positions (if motors enabled)
                        motor_positions_rad = None
                        if dynamixel_controller is not None:
                            motor_positions_rad = dynamixel_controller.read_positions_radians()

                        # 4. Write commands to motors
                        if dynamixel_controller is not None:
                            dynamixel_controller.write_positions([hip_ctrl, knee_ctrl])

                        # 5. Update position monitor (currently shows ctrl values)
                        # TODO: Update to show comparison of qpos, ctrl, and motor positions
                        position_monitor.update_positions([hip_ctrl, knee_ctrl])

                        # 6. Update plotter if enabled
                        if plotter is not None:
                            qpos = {"hipY": hip_qpos, "knee": knee_qpos}
                            ctrl = {"hipY": hip_ctrl, "knee": knee_ctrl}
                            plotter.update(current_time, qpos, ctrl, motor_positions_rad)

                    last_control_time = current_time

                # Sync the viewer
                viewer.sync()
    finally:
        # Clean up position monitor
        position_monitor.close()

        # Save and close plotter
        if plotter is not None:
            plotter.save()
            plotter.close()

        # Disconnect Dynamixel motors
        if dynamixel_controller is not None:
            dynamixel_controller.disconnect()

if __name__ == "__main__":
    main()

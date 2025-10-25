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
from position_monitor import PositionMonitor
from dynamixel_controller import DynamixelController

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="MuJoCo Test Rig with optional Dynamixel motor sync")
    parser.add_argument("--motors", action="store_true", help="Enable real Dynamixel motor synchronization")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port for Dynamixel motors (default: /dev/ttyUSB0)")
    parser.add_argument("--motor-ids", type=int, nargs=2, default=[1, 2], help="Motor IDs for hipY and knee (default: 1 2)")
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

    # Initialize position monitor
    position_monitor = PositionMonitor(["hipY", "knee"])

    # Reset to keyframe "1" if it exists
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "1")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        print("Reset to keyframe '1'")

    # Initialize Dynamixel controller if enabled
    dynamixel_controller = None
    if args.motors:
        print("\n[Motor Sync] Initializing Dynamixel motors...")
        dynamixel_controller = DynamixelController(
            port=args.port,
            motor_ids=args.motor_ids,
            joint_names=["hipY", "knee"]
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

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Simulation loop
            while viewer.is_running():
                # Step the simulation
                mujoco.mj_step(model, data)

                # Read and monitor control commands
                if hip_actuator_id >= 0 and knee_actuator_id >= 0:
                    # Read control commands (what the sliders set)
                    hip_ctrl = data.ctrl[hip_actuator_id]
                    knee_ctrl = data.ctrl[knee_actuator_id]

                    # Update position monitor
                    position_monitor.update_positions([hip_ctrl, knee_ctrl])

                    # Send positions to real motors if enabled
                    if dynamixel_controller is not None:
                        dynamixel_controller.write_positions([hip_ctrl, knee_ctrl])

                # Sync the viewer
                viewer.sync()
    finally:
        # Clean up position monitor
        position_monitor.close()

        # Disconnect Dynamixel motors
        if dynamixel_controller is not None:
            dynamixel_controller.disconnect()

if __name__ == "__main__":
    main()

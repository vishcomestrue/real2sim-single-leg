#!/usr/bin/env python3
"""
Dynamixel Controller
Controls Dynamixel MX-64 motors via Dynamixel SDK Protocol 2.0
"""

import math
from dynamixel_sdk import *


class DynamixelController:
    """Controller for Dynamixel MX-64 motors using Protocol 2.0."""

    # Control Table Addresses (MX-64 with Protocol 2.0)
    ADDR_DRIVE_MODE = 10
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    # Data Byte Length
    LEN_GOAL_POSITION = 4

    # Drive Mode values
    DRIVE_MODE_NORMAL = 0
    DRIVE_MODE_REVERSE = 1

    # Protocol and Communication Settings
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 57600

    # Motor position range
    DXL_MINIMUM_POSITION = 0
    DXL_MAXIMUM_POSITION = 4095
    DXL_CENTER_POSITION = 2048  # Corresponds to 0 radians

    # Communication result values
    COMM_SUCCESS = 0
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0

    def __init__(self, port="/dev/ttyUSB0", motor_ids=[1, 2], joint_names=["hipY", "knee"]):
        """
        Initialize Dynamixel controller.

        Args:
            port: Serial port path (e.g., "/dev/ttyUSB0")
            motor_ids: List of motor IDs [hip_motor_id, knee_motor_id]
            joint_names: List of joint names for logging
        """
        self.port = port
        self.motor_ids = motor_ids
        self.joint_names = joint_names
        self.enabled = False
        self.calibrated = False

        # Calibration offsets (Dynamixel units) for each motor
        # offset[motor_id] = current_motor_position - expected_position_from_radian
        self.position_offsets = {}

        # Initialize port and packet handlers
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize sync write for efficient multi-motor control
        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            self.ADDR_GOAL_POSITION,
            self.LEN_GOAL_POSITION
        )

        print(f"[Dynamixel Controller] Initialized for motors: {motor_ids} on {port}")

    def connect(self):
        """
        Open serial port and enable motor torque.

        Returns:
            bool: True if connection successful, False otherwise
        """
        # Open port
        if not self.port_handler.openPort():
            print(f"[Dynamixel Controller] Failed to open port {self.port}")
            return False
        print(f"[Dynamixel Controller] Opened port {self.port}")

        # Set baudrate
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            print(f"[Dynamixel Controller] Failed to set baudrate to {self.BAUDRATE}")
            return False
        print(f"[Dynamixel Controller] Set baudrate to {self.BAUDRATE}")

        # Set Drive Mode to NORMAL for all motors (must be done before enabling torque)
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                self.ADDR_DRIVE_MODE,
                self.DRIVE_MODE_NORMAL
            )

            if dxl_comm_result != COMM_SUCCESS:
                print(f"[Dynamixel Controller] Motor ID {motor_id} drive mode: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[Dynamixel Controller] Motor ID {motor_id} drive mode: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                print(f"[Dynamixel Controller] Motor ID {motor_id} drive mode set to NORMAL")

        # Enable torque for all motors
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_ENABLE
            )

            if dxl_comm_result != COMM_SUCCESS:
                print(f"[Dynamixel Controller] Motor ID {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return False
            elif dxl_error != 0:
                print(f"[Dynamixel Controller] Motor ID {motor_id}: {self.packet_handler.getRxPacketError(dxl_error)}")
                return False

            print(f"[Dynamixel Controller] Motor ID {motor_id} torque enabled")

        self.enabled = True
        print("[Dynamixel Controller] All motors connected and enabled")
        return True

    def read_positions(self):
        """
        Read current positions from all motors.

        Returns:
            dict: {motor_id: position_in_dynamixel_units} or None if error
        """
        if not self.enabled:
            return None

        positions = {}
        for motor_id in self.motor_ids:
            # Read present position (4 bytes)
            dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                motor_id,
                self.ADDR_PRESENT_POSITION
            )

            if dxl_comm_result != COMM_SUCCESS:
                print(f"[Dynamixel Controller] Failed to read position from motor {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return None
            elif dxl_error != 0:
                print(f"[Dynamixel Controller] Error reading position from motor {motor_id}: {self.packet_handler.getRxPacketError(dxl_error)}")
                return None

            positions[motor_id] = dxl_present_position

        return positions

    def dynamixel_to_radians(self, dxl_position, motor_id):
        """
        Convert Dynamixel position units to radians, with calibration offset removed.

        Args:
            dxl_position: Position in Dynamixel units (0-4095)
            motor_id: Motor ID to apply the correct calibration offset

        Returns:
            float: Position in radians
        """
        # Remove calibration offset if available
        position = dxl_position
        if self.calibrated and motor_id in self.position_offsets:
            position -= self.position_offsets[motor_id]

        # Convert Dynamixel units to radians
        # Reverse of radians_to_dynamixel_raw conversion
        radians = (position - self.DXL_CENTER_POSITION) * math.pi / 2048

        return radians

    def read_positions_radians(self):
        """
        Read current positions from all motors and return in radians.

        Returns:
            dict: {motor_id: position_in_radians} or None if error
        """
        dxl_positions = self.read_positions()
        if dxl_positions is None:
            return None

        positions_rad = {}
        for motor_id, dxl_pos in dxl_positions.items():
            positions_rad[motor_id] = self.dynamixel_to_radians(dxl_pos, motor_id)

        return positions_rad

    def calibrate(self, initial_control_values):
        """
        Calibrate motor positions based on initial control values from MuJoCo.

        This reads the current physical motor positions and calculates offsets
        so that the control values from MuJoCo map correctly to motor positions.

        Args:
            initial_control_values: List or dict of initial control positions in radians
                                   If list: [hip_ctrl, knee_ctrl]
                                   If dict: {joint_name: ctrl_value}

        Returns:
            bool: True if calibration successful, False otherwise
        """
        if not self.enabled:
            print("[Dynamixel Controller] Cannot calibrate - motors not connected")
            return False

        print("\n[Dynamixel Controller] Starting calibration...")

        # Convert list to dict if needed
        if isinstance(initial_control_values, (list, tuple)):
            ctrl_dict = {name: val for name, val in zip(self.joint_names, initial_control_values)}
        else:
            ctrl_dict = initial_control_values

        # Read current motor positions
        current_positions = self.read_positions()
        if current_positions is None:
            print("[Dynamixel Controller] Calibration failed - could not read motor positions")
            return False

        # Calculate offsets for each motor
        self.position_offsets = {}
        for motor_id, joint_name in zip(self.motor_ids, self.joint_names):
            if joint_name in ctrl_dict:
                # Expected position based on control value (without any offset)
                expected_position = self.radians_to_dynamixel_raw(ctrl_dict[joint_name])

                # Current actual position
                actual_position = current_positions[motor_id]

                # Calculate offset
                offset = actual_position - expected_position
                self.position_offsets[motor_id] = offset

                print(f"[Calibration] {joint_name} (Motor ID {motor_id}):")
                print(f"  Control value: {ctrl_dict[joint_name]:+.4f} rad")
                print(f"  Expected position: {expected_position} units")
                print(f"  Current position: {actual_position} units")
                print(f"  Offset: {offset:+d} units")

        self.calibrated = True
        print("[Dynamixel Controller] Calibration complete!\n")
        return True

    def disconnect(self):
        """Disable torque and close port."""
        if self.enabled:
            # Disable torque for all motors
            for motor_id in self.motor_ids:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler,
                    motor_id,
                    self.ADDR_TORQUE_ENABLE,
                    self.TORQUE_DISABLE
                )
            print("[Dynamixel Controller] Torque disabled for all motors")

        # Close port
        self.port_handler.closePort()
        self.enabled = False
        print("[Dynamixel Controller] Port closed")

    def radians_to_dynamixel_raw(self, radians):
        """
        Convert radians to Dynamixel position units (without calibration offset).

        Args:
            radians: Position in radians

        Returns:
            int: Position in Dynamixel units (0-4095)
        """
        # Convert radians to Dynamixel units
        # Center position (2048) = 0 radians
        # Each unit = 0.088 degrees = 0.088 * pi/180 radians
        # Note: Using NORMAL Drive Mode (CCW=Positive, CW=Negative)
        position = int((radians * 2048 / math.pi) + self.DXL_CENTER_POSITION)

        # Clamp to valid range
        position = max(self.DXL_MINIMUM_POSITION, min(self.DXL_MAXIMUM_POSITION, position))

        return position

    def radians_to_dynamixel(self, radians, motor_id):
        """
        Convert radians to Dynamixel position units with calibration offset applied.

        Args:
            radians: Position in radians
            motor_id: Motor ID to apply the correct calibration offset

        Returns:
            int: Position in Dynamixel units (0-4095) with offset applied
        """
        # Get raw position
        position = self.radians_to_dynamixel_raw(radians)

        # Apply calibration offset if available
        if self.calibrated and motor_id in self.position_offsets:
            position += self.position_offsets[motor_id]

        # Clamp to valid range after applying offset
        position = max(self.DXL_MINIMUM_POSITION, min(self.DXL_MAXIMUM_POSITION, position))

        return position

    def write_positions(self, positions):
        """
        Write positions to motors.

        Args:
            positions: List or dict of positions in radians
                      If list: [hip_pos, knee_pos]
                      If dict: {joint_name: position}
        """
        if not self.enabled:
            return

        # Convert list to dict if needed
        if isinstance(positions, (list, tuple)):
            pos_dict = {name: pos for name, pos in zip(self.joint_names, positions)}
        else:
            pos_dict = positions

        # Convert radians to Dynamixel units (with calibration offsets)
        dxl_positions = {}
        for motor_id, joint_name in zip(self.motor_ids, self.joint_names):
            if joint_name in pos_dict:
                radian_pos = pos_dict[joint_name]
                dxl_pos = self.radians_to_dynamixel(radian_pos, motor_id)
                dxl_positions[joint_name] = dxl_pos

        # Write positions using sync write for efficiency
        for motor_id, joint_name in zip(self.motor_ids, self.joint_names):
            if joint_name in dxl_positions:
                dxl_pos = dxl_positions[joint_name]

                # Allocate goal position value into byte array (4 bytes)
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(dxl_pos)),
                    DXL_HIBYTE(DXL_LOWORD(dxl_pos)),
                    DXL_LOBYTE(DXL_HIWORD(dxl_pos)),
                    DXL_HIBYTE(DXL_HIWORD(dxl_pos))
                ]

                # Add parameter to sync write
                self.group_sync_write.addParam(motor_id, param_goal_position)

        # Transmit packet
        dxl_comm_result = self.group_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[Dynamixel Controller] Sync write failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")

        # Clear sync write parameter storage
        self.group_sync_write.clearParam()


if __name__ == "__main__":
    """Test the Dynamixel controller."""
    import time

    print("Dynamixel Controller Test")
    print("=" * 50)
    print("This will test connection to motors ID 1 and 2")
    print("Make sure motors are connected and powered!")
    print("=" * 50)

    # Create controller
    controller = DynamixelController(port="/dev/ttyUSB0", motor_ids=[1, 2])

    # Try to connect
    if not controller.connect():
        print("\nFailed to connect to motors. Exiting...")
        exit(1)

    print("\nConnection successful! Calibrating with zero positions...")
    # Calibrate with zero positions for testing
    controller.calibrate([0.0, 0.0])

    print("\nTesting motor movement...")
    print("Motors will move in a sine wave pattern")
    print("Press Ctrl+C to stop\n")

    try:
        t = 0
        while True:
            # Generate test positions
            hip_pos = 0.3 * math.sin(t)
            knee_pos = 0.3 * math.cos(t * 0.7)

            # Send positions
            controller.write_positions([hip_pos, knee_pos])

            # Print positions
            hip_dxl = controller.radians_to_dynamixel(hip_pos, 1)
            knee_dxl = controller.radians_to_dynamixel(knee_pos, 2)
            print(f"\rHip: {hip_pos:+.3f} rad ({hip_dxl:4d}) | Knee: {knee_pos:+.3f} rad ({knee_dxl:4d})", end="")

            time.sleep(0.02)  # 50 Hz update rate
            t += 0.02

    except KeyboardInterrupt:
        print("\n\nStopping test...")

    finally:
        controller.disconnect()
        print("Test completed")

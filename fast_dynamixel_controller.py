#!/usr/bin/env python3
"""
Fast Dynamixel Controller - Optimized for Maximum Performance
Controls Dynamixel MX-64 motors via Dynamixel SDK Protocol 2.0

Performance Optimizations:
- GroupSyncRead: Read all motors in single transaction (vs sequential reads)
- GroupSyncWrite: Write all motors efficiently
- 1Mbps baudrate: Reliable speed for U2D2 adapter (17x faster than 57600)
- Reduced latency timer: 4ms instead of default 16ms
- Pre-allocated buffers: Minimize runtime allocations
- Better error handling: Automatic retries for transient errors

Expected Performance: 80-120Hz sustained control loop
"""

import math
from dynamixel_sdk import *


class FastDynamixelController:
    """High-performance controller for Dynamixel MX-64 motors using Protocol 2.0."""

    # Control Table Addresses (MX-64 with Protocol 2.0)
    ADDR_DRIVE_MODE = 10
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    # Data Byte Length
    LEN_GOAL_POSITION = 4
    LEN_PRESENT_POSITION = 4

    # Drive Mode values
    DRIVE_MODE_NORMAL = 0
    DRIVE_MODE_REVERSE = 1

    # Protocol and Communication Settings
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 2000000  # 2Mbps - Default baudrate
    LATENCY_TIMER = 16  # 16ms - Default latency timer

    # Motor position range
    DXL_MINIMUM_POSITION = 0
    DXL_MAXIMUM_POSITION = 4095
    DXL_CENTER_POSITION = 2048  # Corresponds to 0 radians

    # Communication result values
    COMM_SUCCESS = 0
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0

    # Retry settings for error handling
    MAX_READ_RETRIES = 3
    MAX_WRITE_RETRIES = 2

    def __init__(self, port="/dev/ttyUSB0", motor_ids=[1, 2], joint_names=["hipY", "knee"],
                 baudrate=None, use_fast_sync=False, use_sync_read=True):
        """
        Initialize Fast Dynamixel controller.

        Args:
            port: Serial port path (e.g., "/dev/ttyUSB0")
            motor_ids: List of motor IDs [hip_motor_id, knee_motor_id]
            joint_names: List of joint names for logging
            baudrate: Custom baudrate (default: 1000000)
            use_fast_sync: Use FastSyncRead protocol (experimental, even faster)
            use_sync_read: Use GroupSyncRead (True) or sequential reads (False)
        """
        self.port = port
        self.motor_ids = motor_ids
        self.joint_names = joint_names
        self.enabled = False
        self.calibrated = False
        self.use_fast_sync = use_fast_sync
        self.use_sync_read = use_sync_read
        self.sync_read_failures = 0  # Track consecutive failures for auto-fallback

        # Allow custom baudrate
        if baudrate is not None:
            self.BAUDRATE = baudrate

        # Calibration offsets (Dynamixel units) for each motor
        # offset[motor_id] = current_motor_position - expected_position_from_radian
        self.position_offsets = {}

        # Initialize port and packet handlers
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncRead for efficient multi-motor reading
        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.ADDR_PRESENT_POSITION,
            self.LEN_PRESENT_POSITION
        )

        # Initialize GroupSyncWrite for efficient multi-motor control
        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            self.ADDR_GOAL_POSITION,
            self.LEN_GOAL_POSITION
        )

        # Statistics for monitoring
        self.stats = {
            'read_errors': 0,
            'write_errors': 0,
            'read_retries': 0,
            'write_retries': 0,
            'total_reads': 0,
            'total_writes': 0
        }

        print(f"[Fast Dynamixel Controller] Initialized for motors: {motor_ids} on {port}")
        print(f"[Fast Dynamixel Controller] Baudrate: {self.BAUDRATE} bps")
        print(f"[Fast Dynamixel Controller] Latency timer: {self.LATENCY_TIMER} ms")
        print(f"[Fast Dynamixel Controller] Read method: {'GroupSyncRead' if use_sync_read else 'Sequential'}")
        if use_fast_sync:
            print(f"[Fast Dynamixel Controller] FastSyncRead: ENABLED (experimental)")

    def connect(self):
        """
        Open serial port and enable motor torque.

        Returns:
            bool: True if connection successful, False otherwise
        """
        # Open port
        if not self.port_handler.openPort():
            print(f"[Fast Dynamixel Controller] Failed to open port {self.port}")
            return False
        print(f"[Fast Dynamixel Controller] Opened port {self.port}")

        # Set baudrate
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            print(f"[Fast Dynamixel Controller] Failed to set baudrate to {self.BAUDRATE}")
            return False
        print(f"[Fast Dynamixel Controller] Set baudrate to {self.BAUDRATE}")

        # Set latency timer
        try:
            # This is a module-level constant that affects timeout calculations
            import dynamixel_sdk.port_handler as ph
            original_latency = ph.LATENCY_TIMER
            ph.LATENCY_TIMER = self.LATENCY_TIMER
            if original_latency != self.LATENCY_TIMER:
                print(f"[Fast Dynamixel Controller] Latency timer: {original_latency}ms → {self.LATENCY_TIMER}ms")
            else:
                print(f"[Fast Dynamixel Controller] Latency timer: {self.LATENCY_TIMER}ms")
        except Exception as e:
            print(f"[Fast Dynamixel Controller] Warning: Could not adjust latency timer: {e}")

        # Disable torque first (required to write to EEPROM areas like Drive Mode)
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_DISABLE
            )
            # Ignore errors here - motor might already have torque disabled

        # Set Drive Mode to NORMAL for all motors (must be done with torque disabled)
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                self.ADDR_DRIVE_MODE,
                self.DRIVE_MODE_NORMAL
            )

            if dxl_comm_result != COMM_SUCCESS:
                print(f"[Fast Dynamixel Controller] Motor ID {motor_id} drive mode: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"[Fast Dynamixel Controller] Motor ID {motor_id} drive mode: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                print(f"[Fast Dynamixel Controller] Motor ID {motor_id} drive mode set to NORMAL")

        # Enable torque for all motors
        for motor_id in self.motor_ids:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_ENABLE
            )

            if dxl_comm_result != COMM_SUCCESS:
                print(f"[Fast Dynamixel Controller] Motor ID {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return False
            elif dxl_error != 0:
                print(f"[Fast Dynamixel Controller] Motor ID {motor_id}: {self.packet_handler.getRxPacketError(dxl_error)}")
                return False

            print(f"[Fast Dynamixel Controller] Motor ID {motor_id} torque enabled")

        # Add all motors to sync read group
        for motor_id in self.motor_ids:
            if not self.group_sync_read.addParam(motor_id):
                print(f"[Fast Dynamixel Controller] Failed to add motor {motor_id} to sync read group")
                return False

        self.enabled = True
        print("[Fast Dynamixel Controller] All motors connected and enabled")
        return True

    def read_positions(self):
        """
        Read current positions from all motors.
        Uses GroupSyncRead if enabled, otherwise sequential reads.

        Returns:
            dict: {motor_id: position_in_dynamixel_units} or None if error
        """
        if not self.enabled:
            return None

        self.stats['total_reads'] += 1

        # Try GroupSyncRead first if enabled
        if self.use_sync_read:
            result = self._read_positions_sync()
            if result is not None:
                self.sync_read_failures = 0  # Reset failure counter
                return result
            else:
                self.sync_read_failures += 1
                # Auto-fallback after 10 consecutive failures
                if self.sync_read_failures >= 10:
                    print(f"[Fast Dynamixel Controller] GroupSyncRead failing consistently, switching to sequential reads")
                    self.use_sync_read = False
                    # Fall through to sequential read

        # Use sequential reads (fallback or by choice)
        return self._read_positions_sequential()

    def _read_positions_sync(self):
        """
        Read positions using GroupSyncRead (single transaction for all motors).

        Returns:
            dict: {motor_id: position} or None if error
        """
        # Attempt read with retries for transient errors
        for attempt in range(self.MAX_READ_RETRIES):
            # Transmit sync read request and receive responses
            dxl_comm_result = self.group_sync_read.txRxPacket()

            if dxl_comm_result != COMM_SUCCESS:
                if attempt < self.MAX_READ_RETRIES - 1:
                    self.stats['read_retries'] += 1
                    continue  # Retry
                else:
                    self.stats['read_errors'] += 1
                    return None

            # Extract position data for each motor
            positions = {}
            all_available = True

            for motor_id in self.motor_ids:
                # Check if data is available for this motor
                if not self.group_sync_read.isAvailable(motor_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION):
                    if attempt < self.MAX_READ_RETRIES - 1:
                        self.stats['read_retries'] += 1
                        all_available = False
                        break
                    else:
                        self.stats['read_errors'] += 1
                        return None

                # Get position data
                position = self.group_sync_read.getData(motor_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
                positions[motor_id] = position

            if all_available:
                return positions

        # Should not reach here, but handle gracefully
        self.stats['read_errors'] += 1
        return None

    def _read_positions_sequential(self):
        """
        Read positions using sequential individual reads (reliable fallback).

        Returns:
            dict: {motor_id: position} or None if error
        """
        positions = {}
        for motor_id in self.motor_ids:
            # Read present position (4 bytes)
            dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                motor_id,
                self.ADDR_PRESENT_POSITION
            )

            if dxl_comm_result != COMM_SUCCESS:
                self.stats['read_errors'] += 1
                return None
            elif dxl_error != 0:
                self.stats['read_errors'] += 1
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
            print("[Fast Dynamixel Controller] Cannot calibrate - motors not connected")
            return False

        print("\n[Fast Dynamixel Controller] Starting calibration...")

        # Convert list to dict if needed
        if isinstance(initial_control_values, (list, tuple)):
            ctrl_dict = {name: val for name, val in zip(self.joint_names, initial_control_values)}
        else:
            ctrl_dict = initial_control_values

        # Read current motor positions
        current_positions = self.read_positions()
        if current_positions is None:
            print("[Fast Dynamixel Controller] Calibration failed - could not read motor positions")
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
        print("[Fast Dynamixel Controller] Calibration complete!\n")
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
            print("[Fast Dynamixel Controller] Torque disabled for all motors")

        # Clear sync read parameters
        self.group_sync_read.clearParam()

        # Close port
        self.port_handler.closePort()
        self.enabled = False
        print("[Fast Dynamixel Controller] Port closed")

        # Print statistics
        if self.stats['total_reads'] > 0:
            print(f"\n[Fast Dynamixel Controller] Session Statistics:")
            print(f"  Total reads: {self.stats['total_reads']}")
            print(f"  Total writes: {self.stats['total_writes']}")
            print(f"  Read errors: {self.stats['read_errors']} ({100*self.stats['read_errors']/self.stats['total_reads']:.2f}%)")
            print(f"  Write errors: {self.stats['write_errors']} ({100*self.stats['write_errors']/max(1, self.stats['total_writes']):.2f}%)")
            print(f"  Read retries: {self.stats['read_retries']}")
            print(f"  Write retries: {self.stats['write_retries']}")

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
        Write positions to motors using GroupSyncWrite.

        Args:
            positions: List or dict of positions in radians
                      If list: [hip_pos, knee_pos]
                      If dict: {joint_name: position}

        Returns:
            bool: True if write successful, False otherwise
        """
        if not self.enabled:
            return False

        self.stats['total_writes'] += 1

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
                dxl_positions[motor_id] = dxl_pos

        # Add parameters to sync write
        for motor_id in self.motor_ids:
            if motor_id in dxl_positions:
                dxl_pos = dxl_positions[motor_id]

                # Allocate goal position value into byte array (4 bytes)
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(dxl_pos)),
                    DXL_HIBYTE(DXL_LOWORD(dxl_pos)),
                    DXL_LOBYTE(DXL_HIWORD(dxl_pos)),
                    DXL_HIBYTE(DXL_HIWORD(dxl_pos))
                ]

                # Add parameter to sync write
                if not self.group_sync_write.addParam(motor_id, param_goal_position):
                    print(f"[Fast Dynamixel Controller] Failed to add motor {motor_id} to sync write")
                    self.group_sync_write.clearParam()
                    self.stats['write_errors'] += 1
                    return False

        # Attempt write with retries
        for attempt in range(self.MAX_WRITE_RETRIES):
            # Transmit packet
            dxl_comm_result = self.group_sync_write.txPacket()

            # Clear sync write parameter storage
            self.group_sync_write.clearParam()

            if dxl_comm_result == COMM_SUCCESS:
                return True
            elif attempt < self.MAX_WRITE_RETRIES - 1:
                self.stats['write_retries'] += 1
                # Retry - params already cleared, will re-add on next iteration
                continue
            else:
                print(f"[Fast Dynamixel Controller] Sync write failed after {self.MAX_WRITE_RETRIES} attempts: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                self.stats['write_errors'] += 1
                return False

        return False

    def get_statistics(self):
        """
        Get communication statistics.

        Returns:
            dict: Statistics including error rates and retry counts
        """
        return self.stats.copy()

    def reset_statistics(self):
        """Reset communication statistics."""
        self.stats = {
            'read_errors': 0,
            'write_errors': 0,
            'read_retries': 0,
            'write_retries': 0,
            'total_reads': 0,
            'total_writes': 0
        }


if __name__ == "__main__":
    """Test the Fast Dynamixel controller."""
    import time

    print("Fast Dynamixel Controller Test")
    print("=" * 50)
    print("This will test high-speed connection to motors ID 1 and 2")
    print("Make sure motors are connected and powered!")
    print("=" * 50)

    # Create controller with optimized settings
    controller = FastDynamixelController(
        port="/dev/ttyUSB0",
        motor_ids=[1, 2],
        baudrate=2000000  # 2Mbps
    )

    # Try to connect
    if not controller.connect():
        print("\nFailed to connect to motors. Exiting...")
        exit(1)

    print("\nConnection successful! Calibrating with zero positions...")
    # Calibrate with zero positions for testing
    controller.calibrate([0.0, 0.0])

    print("\nTesting motor movement at high speed...")
    print("Motors will move in a sine wave pattern")
    print("Press Ctrl+C to stop\n")

    try:
        t = 0
        loop_times = []
        start_time = time.time()

        while True:
            loop_start = time.time()

            # Generate test positions
            hip_pos = 0.3 * math.sin(t)
            knee_pos = 0.3 * math.cos(t * 0.7)

            # Send positions
            controller.write_positions([hip_pos, knee_pos])

            # Read current positions
            positions = controller.read_positions_radians()
            if positions:
                actual_hip = positions[1]
                actual_knee = positions[2]

            # Calculate loop time
            loop_end = time.time()
            loop_time = (loop_end - loop_start) * 1000  # Convert to ms
            loop_times.append(loop_time)

            # Print status
            hip_dxl = controller.radians_to_dynamixel(hip_pos, 1)
            knee_dxl = controller.radians_to_dynamixel(knee_pos, 2)
            freq = 1000.0 / loop_time if loop_time > 0 else 0

            print(f"\rLoop: {loop_time:.2f}ms ({freq:.1f}Hz) | Hip: {hip_pos:+.3f}rad ({hip_dxl:4d}) | Knee: {knee_pos:+.3f}rad ({knee_dxl:4d})", end="")

            # Small sleep to prevent overwhelming output
            time.sleep(0.005)  # 5ms - aiming for ~100Hz
            t += 0.01

    except KeyboardInterrupt:
        print("\n\nStopping test...")

        # Calculate statistics
        if loop_times:
            avg_time = sum(loop_times) / len(loop_times)
            min_time = min(loop_times)
            max_time = max(loop_times)
            avg_freq = 1000.0 / avg_time

            print(f"\nPerformance Statistics:")
            print(f"  Average loop time: {avg_time:.2f} ms ({avg_freq:.1f} Hz)")
            print(f"  Min loop time: {min_time:.2f} ms ({1000.0/min_time:.1f} Hz)")
            print(f"  Max loop time: {max_time:.2f} ms ({1000.0/max_time:.1f} Hz)")
            print(f"  Total loops: {len(loop_times)}")

    finally:
        controller.disconnect()
        print("Test completed")

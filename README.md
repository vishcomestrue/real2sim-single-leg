# Real2Sim Single Leg

Real-time synchronization between MuJoCo simulation and Dynamixel MX-64 motors for robotic leg control. This project enables seamless integration between physics simulation and physical hardware with automatic calibration.

## Features

- 🎯 **Real-time Sync**: MuJoCo simulation commands instantly control physical Dynamixel motors
- 🔧 **Auto Calibration**: Automatic offset calculation ensures simulation and hardware match perfectly
- 🎮 **Interactive Control**: Use MuJoCo viewer sliders to control both simulation and real motors
- 📊 **Position Monitoring**: Real-time terminal display of joint positions
- 🔒 **Safety Features**: Position limits and clean shutdown procedures

## Hardware Requirements

- **Motors**: 2x Dynamixel MX-64 servos
- **Interface**: U2D2 or compatible USB-to-serial adapter
- **Motor IDs**: 1 (hipY joint) and 2 (knee joint)
- **Baudrate**: 57600

## Dependencies

This project requires:
- [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main/python) - Official Python SDK for Dynamixel motors
- [MuJoCo](https://mujoco.org/) - Physics simulation engine

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/vishcomestrue/real2sim-single-leg.git
cd real2sim-single-leg
```

### 2. Set Up Python Environment with uv

Create a Python 3.11 virtual environment using `uv`:

```bash
# Install uv if you don't have it
# See: https://github.com/astral-sh/uv

# Create virtual environment with Python 3.11
uv venv --python 3.11

# Activate the environment
source .venv/bin/activate  # On Linux/macOS
# or
.venv\Scripts\activate     # On Windows
```

### 3. Install DynamixelSDK

```bash
# Navigate to the DynamixelSDK directory
cd DynamixelSDK

# Install in editable mode
uv pip install -e .

# Return to project root
cd ..
```

### 4. Install MuJoCo

```bash
uv pip install mujoco
```

### 5. Configure Serial Port Permissions (Linux)

```bash
# Add your user to the dialout group
sudo usermod -a -G dialout $USER

# Or give permission to the specific port
sudo chmod 666 /dev/ttyUSB0

# Log out and back in for group changes to take effect
```

## Usage

### Simulation Only (No Hardware)

Run the MuJoCo simulation without connecting to real motors:

```bash
cd real2sim
python3 load_mujoco.py
```

**Controls:**
- Use the sliders in the MuJoCo viewer to control joint positions
- Space: Pause/Resume
- Backspace: Reset simulation
- Esc: Exit

### With Real Motors

Connect and synchronize with physical Dynamixel motors:

```bash
python3 load_mujoco.py --motors
```

On startup, the system will:
1. Load the MuJoCo simulation with initial keyframe
2. Connect to motors on `/dev/ttyUSB0`
3. **Automatically calibrate** - read current motor positions and calculate offsets
4. Sync simulation commands to real motors in real-time

**Example calibration output:**
```
[Calibration] hipY (Motor ID 1):
  Control value: +0.1080 rad
  Expected position: 2117 units
  Current position: 2250 units
  Offset: +133 units

[Calibration] knee (Motor ID 2):
  Control value: -0.0500 rad
  Expected position: 2015 units
  Current position: 1980 units
  Offset: -35 units
```

### Custom Configuration

```bash
# Use a different serial port
python3 load_mujoco.py --motors --port /dev/ttyUSB1

# Use different motor IDs
python3 load_mujoco.py --motors --motor-ids 3 4

# Combine options
python3 load_mujoco.py --motors --port /dev/ttyUSB1 --motor-ids 3 4
```

### Test Motors Only

Test motor connection with a sine wave pattern:

```bash
python3 dynamixel_controller.py
```

Press Ctrl+C to stop.

## Project Structure

```
real2sim-single-leg/
├── DynamixelSDK/              # Dynamixel SDK dependency
│   ├── src/
│   │   └── dynamixel_sdk/     # SDK Python package
│   └── setup.py
├── real2sim/                  # Main project code
│   ├── load_mujoco.py         # Main application with motor sync
│   ├── dynamixel_controller.py # Motor controller with calibration
│   ├── position_monitor.py    # Terminal position display
│   └── xmls/                  # MuJoCo model files
│       ├── scene.xml          # Main scene configuration
│       ├── test_rig.xml       # Leg model definition
│       └── meshes/            # 3D mesh files
└── README.md
```

## How It Works

### System Architecture

```
┌─────────────────┐
│  MuJoCo Viewer  │  ← User controls with sliders
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  load_mujoco.py │  ← Main sync loop
└────────┬────────┘
         │
         ├─► Position Monitor (Terminal display)
         │
         ├─► MuJoCo Simulation (data.ctrl values)
         │
         ▼
┌─────────────────┐
│ Dynamixel Ctrl  │  ← Converts radians → motor units
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Real Motors    │  ← Physical MX-64 servos
└─────────────────┘
```

### Calibration Process

1. **Load Keyframe**: MuJoCo loads the initial pose from keyframe "1"
2. **Connect Motors**: System connects to Dynamixel motors via serial port
3. **Read Positions**: Current physical motor positions are read
4. **Calculate Offsets**: 
   ```
   offset = actual_position - expected_position_from_keyframe
   ```
5. **Apply Offsets**: All future commands include these offsets
   ```
   target = convert_radians_to_units(command) + offset
   ```

This ensures the real motors perfectly match the simulation state, regardless of their initial positions.

### Position Conversion

- **MuJoCo**: Uses radians
- **Dynamixel MX-64**: Uses 0-4095 units (12-bit resolution, ~0.088° per unit)
- **Center position**: 2048 units = 0 radians
- **Formula**: 
  ```python
  dynamixel_units = int((radians * 2048 / π) + 2048) + calibration_offset
  ```

## Command-Line Options

```bash
python3 load_mujoco.py [OPTIONS]
```

| Option | Description | Default |
|--------|-------------|---------|
| `--motors` | Enable real motor synchronization | Disabled |
| `--port PORT` | Serial port path | `/dev/ttyUSB0` |
| `--motor-ids ID1 ID2` | Motor IDs for hipY and knee | `1 2` |

## Troubleshooting

### Motors Not Connecting

**Error:**
```
[Dynamixel Controller] Failed to open port /dev/ttyUSB0
```

**Solutions:**
- Check if U2D2 is connected: `ls /dev/ttyUSB*`
- Check permissions: `sudo chmod 666 /dev/ttyUSB0`
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Try different port: `--port /dev/ttyUSB1`
- Verify motors are powered on

### Large Calibration Offsets

**Normal behavior** - If motors start far from the keyframe position, you'll see large offsets. The system automatically compensates for this.

To minimize offsets:
- Manually position motors near expected pose before running
- Verify motors are in Position Control Mode
- Check motor IDs are correct

### Motors Move in Opposite Direction

The calibration system automatically handles position matching. If directions are still reversed:
- Check motor mounting orientation
- Verify Drive Mode is set to NORMAL (configured automatically)

### Permission Denied Errors

```bash
# Temporary fix
sudo chmod 666 /dev/ttyUSB0

# Permanent fix
sudo usermod -a -G dialout $USER
# Then log out and back in
```

## Technical Details

### Motor Configuration

- **Protocol**: Dynamixel Protocol 2.0
- **Operating Mode**: Position Control Mode (Mode 3)
- **Drive Mode**: NORMAL (CCW = Positive, CW = Negative)
- **Position Range**: 0-4095 units
- **Baudrate**: 57600 bps
- **Update Rate**: ~50 Hz (synchronized with MuJoCo)

### Communication

- **Write Method**: GroupSyncWrite (both motors simultaneously)
- **Latency**: Typically <20ms from slider to motor response
- **Direction**: One-way (simulation → motors)

## Development

### Running Tests

Test the Dynamixel controller independently:

```bash
python3 dynamixel_controller.py
```

This will move motors in a sine wave pattern for testing.

### Adding More Motors

To add additional motors, modify:
1. `motor_ids` parameter in `DynamixelController`
2. `joint_names` parameter to match your joints
3. Update actuator IDs in `load_mujoco.py`

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

[Add your license information here]

## Acknowledgments

- [MuJoCo](https://mujoco.org/) - Physics simulation engine
- [ROBOTIS DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) - Motor control SDK

## Contact

Vishwanath R - rvishwanath03@outlook.com

---

**Status**: ✅ Ready for use  
**Last Updated**: October 25, 2025

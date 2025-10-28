# Real2Sim Single Leg

Real-time synchronization between MuJoCo simulation and Dynamixel MX-64 motors for robotic leg control. This project enables seamless integration between physics simulation and physical hardware with automatic calibration.

## Features

- 🎯 **Real-time Sync**: MuJoCo simulation commands instantly control physical Dynamixel motors
- 🔧 **Auto Calibration**: Automatic offset calculation ensures simulation and hardware match perfectly
- 🎮 **Interactive Control**: Use MuJoCo viewer sliders to control both simulation and real motors
- 🎚️ **Motor Slider GUI**: Standalone GUI with sliders for direct motor control at configurable frequencies (10-200 Hz)
- ⚡ **Fast Controller**: Optimized controller with GroupSyncRead achieving 50-120 Hz control rates (3-7x faster)
- 📊 **Real-time Plotting**: Live visualization of simulation vs. real motor positions
- 🔒 **Safety Features**: Position limits, auto-fallback mechanisms, and clean shutdown procedures
- 🔄 **Auto-Fallback**: Intelligent switching between GroupSyncRead and sequential reads for reliability

## Hardware Requirements

- **Motors**: 2x Dynamixel MX-64 servos
- **Interface**: U2D2 or compatible USB-to-serial adapter (U2D2 recommended for best performance)
- **Motor IDs**: 1 (hipY joint) and 2 (knee joint) - configurable
- **Baudrate**: 2000000 (2Mbps) - default, configurable (57600 to 4Mbps)
- **Performance**: 50-120 Hz control loop with FastDynamixelController

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

### With Real Motors (MuJoCo Integration)

Connect and synchronize with physical Dynamixel motors:

```bash
# Basic motor sync
python3 load_mujoco.py --motors

# With custom baudrate and control rate
python3 load_mujoco.py --motors --baudrate 2000000 --control-rate 50

# With real-time plotting
python3 load_mujoco.py --motors --baudrate 2000000 --control-rate 50 --plot
```

On startup, the system will:
1. Load the MuJoCo simulation with initial keyframe
2. Connect to motors on `/dev/ttyUSB0` using FastDynamixelController
3. **Automatically calibrate** - read current motor positions and calculate offsets
4. Sync simulation commands to real motors in real-time
5. (Optional) Display real-time plotting of sim vs. real positions

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

### Motor Slider GUI (Direct Control)

Control motors directly without MuJoCo simulation using an interactive GUI:

```bash
# Basic usage (2Mbps, 50Hz)
python3 motor_slider_gui.py

# Custom baudrate and frequency
python3 motor_slider_gui.py --baudrate 2000000 --frequency 100

# Different motor IDs and custom names
python3 motor_slider_gui.py --motor-ids 3 4 --joint-names "Shoulder" "Elbow"
```

**GUI Features:**
- ✅ Real-time sliders for each motor (-90° to +90°)
- ✅ Live position feedback (target, actual, error)
- ✅ Adjustable control frequency (10-200 Hz)
- ✅ Statistics display (actual frequency, cycle time, CPU usage)
- ✅ Zero buttons for quick reset
- ✅ Non-blocking threaded control loop


### Custom Configuration

```bash
# Use a different serial port
python3 load_mujoco.py --motors --port /dev/ttyUSB1

# Use different motor IDs
python3 load_mujoco.py --motors --motor-ids 3 4

# Custom baudrate and control rate
python3 load_mujoco.py --motors --baudrate 1000000 --control-rate 100

# Combine all options with plotting
python3 load_mujoco.py --motors --port /dev/ttyUSB1 --motor-ids 3 4 --baudrate 2000000 --control-rate 50 --plot
```

### Test Fast Controller

**Test fast controller** with sine wave pattern (if script supports standalone execution):

```bash
python3 fast_dynamixel_controller.py
```

This will demonstrate the optimized controller performance with real-time statistics.

## Project Structure

```
real2sim-single-leg/
├── xmls/                          # MuJoCo scene and model files
│   ├── meshes/                    # 3D mesh files (.stl, .obj)
│   ├── scene.xml                  # Main scene configuration
│   └── test_rig.xml               # Robot leg model definition
│
├── load_mujoco.py                 # MuJoCo simulation with real motor sync
├── motor_slider_gui.py            # Interactive GUI for direct motor control
├── fast_dynamixel_controller.py   # Optimized motor controller (50-120 Hz)
├── dynamixel_controller.py        # Synchronous motor controller (legacy)
├── sim2real_plotter.py            # Real-time sim vs. real position plotting
│
├── LICENSE                        # License information
└── README.md                      # Project documentation (this file)
```

## How It Works

### System Architecture

**Option 1: MuJoCo Integration (load_mujoco.py)**
```
┌─────────────────┐
│  MuJoCo Viewer  │  ← User controls with sliders
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  load_mujoco.py │  ← Main sync loop (non-blocking)
└────────┬────────┘
         │
         ├─► MuJoCo Simulation (data.ctrl, data.qpos)
         │
         ├─► Sim2Real Plotter (optional, --plot flag)
         │
         ▼
┌─────────────────────┐
│ FastDynamixelCtrl   │  ← GroupSyncRead/Write, auto-fallback
└────────┬────────────┘
         │
         ▼
┌─────────────────┐
│  Real Motors    │  ← Physical MX-64 servos (50-120 Hz)
└─────────────────┘
```

**Option 2: Direct GUI Control (motor_slider_gui.py)**
```
┌─────────────────┐
│   Tkinter GUI   │  ← User controls with sliders
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Control Thread  │  ← Threaded loop with time.sleep
└────────┬────────┘
         │
         ▼
┌─────────────────────┐
│ FastDynamixelCtrl   │  ← GroupSyncRead/Write (or sequential)
└────────┬────────────┘
         │
         ▼
┌─────────────────┐
│  Real Motors    │  ← Physical MX-64 servos (10-200 Hz)
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

### load_mujoco.py (MuJoCo Integration)

```bash
python3 load_mujoco.py [OPTIONS]
```

| Option | Description | Default |
|--------|-------------|---------|
| `--motors` | Enable real motor synchronization | Disabled |
| `--port PORT` | Serial port path | `/dev/ttyUSB0` |
| `--motor-ids ID1 ID2` | Motor IDs for hipY and knee | `1 2` |
| `--baudrate RATE` | Motor communication baudrate | `2000000` (2Mbps) |
| `--control-rate HZ` | Control loop rate in Hz | `20` |
| `--plot` | Enable real-time sim2real plotting | Disabled |

### motor_slider_gui.py (Direct Control)

```bash
python3 motor_slider_gui.py [OPTIONS]
```

| Option | Description | Default |
|--------|-------------|---------|
| `--port PORT` | Serial port path | `/dev/ttyUSB0` |
| `--motor-ids ID1 ID2 ...` | Motor IDs | `1 2` |
| `--joint-names NAME1 NAME2 ...` | Joint display names | `Hip Knee` |
| `--baudrate RATE` | Motor communication baudrate | `2000000` (2Mbps) |
| `--frequency HZ` | Control frequency (10-200) | `50` |

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

### GroupSyncRead Failures / Packet Errors

**Symptoms:**
```
[Fast Dynamixel Controller] Sync read failed: [TxRxResult] Incorrect status packet!
```

**Auto-Fix**: Controller automatically switches to sequential reads after 10 failures.

**Manual Solutions:**
```bash
# Force sequential reads from start (more compatible)
python3 motor_slider_gui.py --baudrate 2000000  # Uses auto-fallback

# Or try lower baudrate
python3 motor_slider_gui.py --baudrate 1000000
```

**Common causes**: Hardware incompatibility, poor cable quality, or electrical noise.

### Low Control Frequency (<50 Hz)

**Check:**
1. Baudrate matches motor configuration (verify with Dynamixel Wizard)
2. USB cable quality (use <2m, high quality)
3. No other programs using the port
4. Controller is using FastDynamixelController (not legacy)

**Solutions:**
```bash
# Increase baudrate if motors support it
python3 load_mujoco.py --motors --baudrate 2000000

# Try the motor slider GUI to test control frequency
python3 motor_slider_gui.py --baudrate 2000000 --frequency 100
```

## Technical Details

### Motor Configuration

- **Protocol**: Dynamixel Protocol 2.0
- **Operating Mode**: Position Control Mode (Mode 3)
- **Drive Mode**: NORMAL (CCW = Positive, CW = Negative) - automatically configured
- **Position Range**: 0-4095 units (12-bit resolution)
- **Baudrate**: 2000000 bps (2Mbps) default - configurable from 57600 to 4Mbps
- **Update Rate**: 50-120 Hz with FastDynamixelController

### Communication (FastDynamixelController)

- **Read Method**: GroupSyncRead (all motors in single transaction) with auto-fallback to sequential reads
- **Write Method**: GroupSyncWrite (all motors simultaneously)
- **Auto-Fallback**: Switches to sequential reads after 10 consecutive GroupSyncRead failures
- **Latency**: <10ms read+write cycle at 2Mbps
- **Error Handling**: Automatic retries (3x read, 2x write)
- **Statistics Tracking**: Read/write errors, retries, success rates

### Performance Comparison

| Controller | Baudrate | Read Method | Control Rate | Notes |
|-----------|----------|-------------|--------------|-------|
| Original | 57600 | Sequential | 15-20 Hz | Legacy |
| Fast | 2Mbps | GroupSync | 80-120 Hz | Best performance |
| Fast | 2Mbps | Sequential | 50-80 Hz | More compatible |
| Fast | 1Mbps | Sequential | 40-60 Hz | Lower speed hardware |

## Development

### Running Tests

**Test fast controller** with sine wave pattern:
```bash
python3 fast_dynamixel_controller.py
```

**Test motor slider GUI**:
```bash
python3 motor_slider_gui.py
```

**Test MuJoCo integration**:
```bash
python3 load_mujoco.py --motors --baudrate 2000000
```

### Adding More Motors

To add additional motors:
1. Update `motor_ids` parameter: `--motor-ids 1 2 3 4`
2. Update `joint_names` parameter: `--joint-names "Hip" "Knee" "Ankle" "Toe"`
3. For MuJoCo integration, update actuator/joint IDs in `load_mujoco.py`

**Example with 4 motors:**
```bash
python3 motor_slider_gui.py --motor-ids 1 2 3 4 --joint-names "Hip" "Knee" "Ankle" "Toe"
```

### Changing Motor Baudrate

**Recommended method**: Use Dynamixel Wizard (official ROBOTIS tool)

1. Download from https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/
2. Connect to motor
3. Change baudrate in the control table
4. Save to EEPROM

**Important**: Ensure your code's baudrate parameter matches the motor's configured baudrate.

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

**Status**: ✅ Production Ready
**Performance**: 50-120 Hz control rate with FastDynamixelController
**Last Updated**: October 28, 2025

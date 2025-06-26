# SteadyWin BLDC Motor RS485 Driver

A comprehensive Python library for communicating with SteadyWin BLDC motors over RS485 using a custom protocol. This project provides both a core library for integration and an interactive testing system for development and debugging.

## Features

### Core Library (`steadywin_rs485.py`)
- **Complete Command Coverage**: 19 implemented commands (0x00-0x2F)
- **Robust Protocol**: CRC16 validation, proper framing, error handling
- **Real-time Data**: Live motor status, position, speed, current monitoring
- **Control Modes**: Current, velocity, and position control
- **Parameter Management**: Read/write user and control parameters
- **Safety Features**: Input validation, fault handling, timeout protection
- **Performance Optimized**: Efficient communication with minimal delays

### Interactive Test System (`steadywin_rs485_test.py`)
- **User-Friendly Menu**: 23 organized test options with descriptions
- **Auto-Discovery**: Automatic motor address detection
- **Connection Caching**: Remembers last successful connection settings
- **Safety Confirmations**: Protects against dangerous operations
- **Real-time Monitoring**: Live motor state display during operations
- **Debug Support**: Configurable logging and manual packet testing

## Installation

### Prerequisites
- Python 3.7+
- Conda (recommended) or pip
- RS485 USB adapter
- SteadyWin BLDC motor controller

### Environment Setup
```bash
# Create conda environment
conda env create -f environment.yml
conda activate steadywin-bldc

# Or install manually
pip install pyserial logging
```

### Hardware Setup
1. Connect RS485 adapter to your computer
2. Wire adapter to motor controller RS485 terminals
3. Power on motor controller
4. Note the serial port (e.g., `/dev/ttyUSB0`, `COM3`)

## Quick Start

### Basic Usage
```python
from steadywin_rs485 import SteadyWinRS485

# Initialize motor connection
motor = SteadyWinRS485('/dev/ttyUSB0', 115200, device_addr=0x01)

# Connect and read system info
if motor.connect():
    system_info = motor.read_system_info()
    realtime_data = motor.read_realtime_info()
    
    # Control motor
    motor.velocity_control(100.0, 50.0)  # 100 RPM, 50 RPM/s acceleration
    motor.disconnect()
```

### Interactive Testing
```bash
python steadywin_rs485_test.py
```

The interactive system provides:
- Automatic connection management
- Step-by-step command testing
- Real-time motor state monitoring
- Safety confirmations for dangerous operations

## Supported Commands

| Command | Hex  | Function | Description |
|---------|------|----------|-------------|
| Restart Slave | 0x00 | `restart_slave()` | Restarts motor controller |
| Read System Info | 0x0A | `read_system_info()` | Hardware/software version data |
| Read Realtime Info | 0x0B | `read_realtime_info()` | Live motor data |
| Clear Faults | 0x0F | `clear_faults()` | Clears fault conditions |
| Read User Parameters | 0x10 | `read_user_parameters()` | Reads configuration block |
| Write User Parameters | 0x11 | `write_user_parameters()` | Writes motor settings |
| Read Control Parameters | 0x14 | `read_control_parameters()` | Reads PID parameters |
| Write Control Parameters | 0x15 | `write_control_parameters()` | Writes control parameters (temporary) |
| Write & Save Control Parameters | 0x16 | `write_control_parameters_and_save()` | Writes control parameters (permanent) |
| Set Zero Position | 0x1D | `set_zero_position()` | Sets current position as zero |
| Calibrate Encoder | 0x1E | `calibrate_encoder()` | Performs encoder calibration |
| Factory Reset | 0x1F | `factory_reset()` | Restores factory defaults |
| Q-axis Current Control | 0x20 | `q_axis_current_control()` | Current control mode |
| Velocity Control | 0x21 | `velocity_control()` | Speed control mode |
| Absolute Position Control | 0x22 | `absolute_position_control()` | Absolute positioning |
| Relative Position Control | 0x23 | `relative_position_control()` | Relative positioning |
| Return Home | 0x24 | `return_home()` | Returns to home position |
| Brake Control | 0x2E | `brake_control()` | Brake on/off/status |
| Disable Motor | 0x2F | `disable_motor()` | Disables motor operation |

## Usage Examples

### Reading Motor Status
```python
# Get comprehensive motor state
response = motor.read_realtime_info()
if response and 'parsed_data' in response:
    data = response['parsed_data']
    print(f"Position: {data['abs_angle_deg']:.2f}°")
    print(f"Speed: {data['speed_rpm']:.2f} RPM")
    print(f"Current: {data['current_a']:.3f} A")
    print(f"Temperature: {data['work_temp_c']}°C")
```

### Motor Control
```python
# Current control (torque mode)
motor.q_axis_current_control(1000, 100)  # 1000mA, 100mA/s slope

# Velocity control
motor.velocity_control(300.0, 100.0)  # 300 RPM, 100 RPM/s acceleration

# Position control
motor.absolute_position_control(motor.degrees_to_counts(90))  # Move to 90°
motor.relative_position_control(motor.degrees_to_counts(45))  # Move +45°

# Return to home
motor.return_home()
```

### Parameter Management
```python
# Read current parameters
user_params = motor.read_user_parameters()
control_params = motor.read_control_parameters()

# Modify and write parameters
new_user_params = bytearray(16)  # 16-byte parameter block
new_user_params[0:4] = struct.pack('<I', 5000)  # Max speed: 50.00 RPM
motor.write_user_parameters(new_user_params)
```

### Safety Operations
```python
# Clear any faults
motor.clear_faults()

# Emergency stop
motor.disable_motor()

# Factory reset (use with caution!)
motor.factory_reset(confirm_reset=True)
```

## Protocol Details

### Communication
- **Interface**: RS485 serial communication
- **Framing**: Custom packet format with CRC16 validation
- **Byte Order**: Little-endian for multi-byte values
- **Error Handling**: Timeout, CRC validation, response verification

### Data Units
- **Position**: Counts (16384 counts = 360°)
- **Speed**: RPM (protocol uses 0.01 RPM units)
- **Current**: mA (1:1 protocol conversion)
- **Temperature**: °C
- **Voltage**: 0.1V units

### Helper Functions
```python
# Unit conversions
degrees = motor.counts_to_degrees(16384)  # Returns 360.0
counts = motor.degrees_to_counts(180)     # Returns 8192
```

## Development

### Project Structure
```
steadywin_driver/
├── steadywin_rs485.py          # Core library
├── steadywin_rs485_test.py     # Interactive test system
├── environment.yml             # Conda environment
├── steadywin_connection.json   # Connection cache (auto-generated)
└── README.md                   # This file
```

### Architecture
- **Core Library**: Optimized for performance and reliability
- **Helper Methods**: DRY principle implementation for maintainability
- **Error Handling**: Comprehensive validation and recovery
- **Logging**: Configurable debug output
- **Testing**: Interactive system with safety features

### Code Quality
- **Performance Optimized**: Minimal delays, efficient operations
- **Maintainable**: Helper methods eliminate code duplication
- **Safe**: Input validation and safety confirmations
- **Documented**: Comprehensive docstrings and comments

## Safety Considerations

### Important Warnings
- **High Current**: Current control can generate significant torque
- **High Speed**: Velocity control can cause rapid motor movement
- **Factory Reset**: Permanently erases custom settings
- **Encoder Calibration**: Motor will rotate during process

### Best Practices
1. **Start with low values** when testing control commands
2. **Use safety confirmations** in the interactive system
3. **Monitor motor temperature** during operation
4. **Keep emergency stop accessible**
5. **Understand parameter changes** before writing

## Troubleshooting

### Common Issues

**Connection Failed**
- Check serial port and baud rate
- Verify RS485 wiring
- Ensure motor controller is powered
- Try auto-discovery feature

**Command Timeout**
- Check device address
- Verify motor is responsive
- Increase timeout if needed
- Check for communication errors

**Invalid Response**
- Verify CRC validation
- Check for electrical interference
- Ensure proper grounding
- Try different baud rates

**Motor Not Responding**
- Check motor power supply
- Verify fault status
- Try clearing faults
- Consider factory reset

### Debug Mode
Enable detailed logging for troubleshooting:
```python
from steadywin_rs485 import setup_logging
setup_logging(level='DEBUG')
```

## License

This project is provided as-is for educational and development purposes. Please ensure safe operation and proper testing before production use.

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing patterns
- All functions are tested
- Documentation is updated
- Safety considerations are maintained

## Support

For issues and questions:
1. Check this README and code documentation
2. Test with the interactive system
3. Enable debug logging for detailed information
4. Verify hardware setup and connections 
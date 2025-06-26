#!/usr/bin/env python3
"""
SteadyWin RS485 Motor Interactive Test Script

This script provides testing functionality for SteadyWin BLDC motor commands
with automatic connection management and caching.
"""

import os
import sys
import time
import json
import logging
import struct
from steadywin_rs485 import SteadyWinRS485, setup_logging

class SteadyWinRS485Tester:
    """Interactive testing class for SteadyWin RS485 motors"""
    
    def __init__(self):
        self.motor = None
        self.port = None
        self.baudrate = 115200
        self.device_addr = None
        self.connected = False
        self.cache_file = "steadywin_connection.json"
        
    def clear_screen(self):
        """Clear the terminal screen"""
        os.system('clear' if os.name == 'posix' else 'cls')
    
    def print_header(self):
        """Print the main header"""
        # self.clear_screen()
        print("=" * 60)
        print("    SteadyWin RS485 Motor Interactive Test System")
        print("=" * 60)
        if self.motor and self.connected:
            print(f"Status: ✓ Connected to {self.port} (Address: 0x{self.device_addr:02X})")
        else:
            print("Status: ✗ Not connected")
        print("=" * 60)
    
    def get_user_input(self, prompt, input_type=str, default=None):
        """Get validated user input"""
        while True:
            try:
                if default is not None:
                    user_input = input(f"{prompt} [default: {default}]: ").strip()
                    if not user_input:
                        return default
                else:
                    user_input = input(f"{prompt}: ").strip()
                
                if input_type == int:
                    if user_input.startswith('0x') or user_input.startswith('0X'):
                        return int(user_input, 16)
                    else:
                        return int(user_input)
                elif input_type == str:
                    return user_input
                else:
                    return input_type(user_input)
            except (ValueError, KeyboardInterrupt) as e:
                if isinstance(e, KeyboardInterrupt):
                    print("\nExiting...")
                    sys.exit(0)
                print("Invalid input. Please try again.")
    
    def check_conda_environment(self):
        """Check if we're in the correct conda environment"""
        conda_env = os.environ.get('CONDA_DEFAULT_ENV')
        if conda_env != 'steadywin-bldc':
            print("⚠ Warning: Not in 'steadywin-bldc' conda environment")
            print(f"Current environment: {conda_env}")
            print("\nTo activate the correct environment, run:")
            print("conda activate steadywin-bldc")
            print("\nContinuing anyway...")
            input("Press Enter to continue...")
    
    def save_connection_cache(self):
        """Save connection parameters to cache file"""
        try:
            cache_data = {
                'port': self.port,
                'baudrate': self.baudrate,
                'device_addr': self.device_addr
            }
            with open(self.cache_file, 'w') as f:
                json.dump(cache_data, f, indent=2)
            print(f"✓ Connection settings saved to {self.cache_file}")
        except Exception as e:
            print(f"⚠ Failed to save connection cache: {e}")
    
    def load_connection_cache(self):
        """Load connection parameters from cache file"""
        try:
            if os.path.exists(self.cache_file):
                with open(self.cache_file, 'r') as f:
                    cache_data = json.load(f)
                return cache_data.get('port'), cache_data.get('baudrate'), cache_data.get('device_addr')
            return None, None, None
        except Exception as e:
            print(f"⚠ Failed to load connection cache: {e}")
            return None, None, None
    
    def clear_connection_cache(self):
        """Clear the connection cache file"""
        try:
            if os.path.exists(self.cache_file):
                os.remove(self.cache_file)
                print(f"✓ Connection cache cleared")
        except Exception as e:
            print(f"⚠ Failed to clear connection cache: {e}")
    
    def auto_connect(self):
        """Auto-connect using cached connection or setup new connection"""
        # Try to load cached connection
        cached_port, cached_baudrate, cached_addr = self.load_connection_cache()
        
        if cached_port and cached_baudrate and cached_addr is not None:
            print(f"Found cached connection: {cached_port} @ {cached_baudrate} baud, address 0x{cached_addr:02X}")
            print("Attempting to connect...")
            
            # Try cached connection
            if self._try_connection(cached_port, cached_baudrate, cached_addr):
                print("✓ Successfully connected using cached settings!")
                return True
            else:
                print("✗ Cached connection failed, setting up new connection...")
                self.clear_connection_cache()
        
        # No cache or cached connection failed, setup new connection
        return self.setup_new_connection()
    
    def _try_connection(self, port, baudrate, device_addr):
        """Try to establish connection with given parameters"""
        try:
            self.port = port
            self.baudrate = baudrate
            self.device_addr = device_addr
            
            # Create motor instance
            self.motor = SteadyWinRS485(port, baudrate, device_addr=device_addr)
            
            # Test connection
            if self.motor.connect():
                # Test communication with command 0x0A
                response = self.motor.read_system_info()
                if response is not None:
                    self.connected = True
                    return True
                else:
                    self.motor.disconnect()
                    self.motor = None
            return False
        except Exception:
            if self.motor:
                self.motor.disconnect()
                self.motor = None
            return False
    
    def _test_communication(self, motor_instance):
        """
        Test communication with a motor instance
        
        Args:
            motor_instance: SteadyWinRS485 instance to test
            
        Returns:
            bool: True if communication successful, False otherwise
        """
        try:
            # Test communication using command 0x0A (read system info)
            response = motor_instance.read_system_info()
            return response is not None
        except Exception:
            return False
    
    def setup_new_connection(self):
        """Setup new connection parameters"""
        # self.clear_screen()
        print("=" * 60)
        print("    Connection Setup")
        print("=" * 60)
        
        # Get serial port
        print("Enter the serial port for your motor:")
        print("Examples: /dev/ttyUSB0, /dev/ttyACM0, COM3, COM4")
        port = self.get_user_input("Serial Port")
        
        # Get baud rate
        print(f"\nCommon baud rates: 9600, 57600, 115200, 230400")
        baudrate = self.get_user_input("Baud Rate", int, 115200)
        
        # Ask if user wants auto-discovery or manual address
        print(f"\nDevice Address Options:")
        print("1. Auto-discover (recommended)")
        print("2. Manual entry")
        choice = self.get_user_input("Choose option (1 or 2)", int, 1)
        
        if choice == 1:
            device_addr = self.auto_discover_motor(port, baudrate)
        else:
            print("Common addresses: 0x01, 0x00, 0x02")
            device_addr = self.get_user_input("Device Address (hex)", int, 0x01)
        
        if device_addr is None:
            print("\n✗ Motor discovery failed!")
            input("Press Enter to continue...")
            return False
        
        # Test the connection
        print(f"\nTesting connection to motor at 0x{device_addr:02X}...")
        if self._try_connection(port, baudrate, device_addr):
            print("✓ Connection successful!")
            # Save to cache
            self.save_connection_cache()
            # input("\nPress Enter to continue...")
            return True
        else:
            print("✗ Connection failed!")
            input("Press Enter to continue...")
            return False
    
    def auto_discover_motor(self, port, baudrate):
        """Auto-discover motor address"""
        print("\n=== Auto-discovering motor address ===")
        test_addresses = [0x01, 0x02, 0x03, 0x05, 0x10, 0x20]
        
        for addr in test_addresses:
            print(f"Testing address 0x{addr:02X}...", end=" ")
            
            temp_motor = SteadyWinRS485(port, baudrate, device_addr=addr)
            try:
                if temp_motor.connect():
                    response = temp_motor.read_system_info()
                    if response is not None:
                        print("✓ FOUND!")
                        temp_motor.disconnect()
                        return addr
                    temp_motor.disconnect()
                print("✗")
            except Exception:
                print("✗")
            
            time.sleep(0.1)
        
        return None
    
    def test_restart_slave(self):
        """Test Command 0x00 - Restart Slave"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x00 (Restart Slave) ===\n")
        
        print("Sending restart command to motor...")
        success = self.motor.restart_slave()
        
        if success:
            print("✓ Restart command sent successfully!")
            print("  Note: Command 0x00 does not expect a response")
        else:
            print("✗ Failed to send restart command!")
        
        input("\nPress Enter to continue...")
    
    def test_read_system_info(self):
        """Test Command 0x0A - Read System Information"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x0A (Read System Info) ===\n")
        
        print("Reading system information from motor...")
        response = self.motor.read_system_info()
        
        if response:
            print("✓ Successfully read system information!")
            print(f"\nResponse Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
                
                # According to protocol manual, this should contain:
                # Boot, software, hardware, RS485 protocol and CAN protocol version
                print(f"\nPayload Analysis:")
                payload = response['payload']
                if len(payload) >= 16:  # Expected minimum size for version info
                    print(f"  Boot Version: {payload[0:2].hex().upper()}")
                    print(f"  Application Version: {payload[2:4].hex().upper()}")
                    print(f"  Hardware Version: {payload[4:6].hex().upper()}")
                    print(f"  Driver Version: {payload[6:8].hex().upper()}")
                    print(f"  RS485 Protocol: {payload[8]}")
                    print(f"  RS485-Modbus Protocol: {payload[9]}")
                    print(f"  CAN Protocol: {payload[10]}")
                    print(f"  CAN-CanOpen Protocol: {payload[11]}")
                    if len(payload) >= 26:
                        print(f"  UID: {payload[15:26].hex(' ').upper()}")
                else:
                    print(f"  Payload too short for detailed analysis")
            else:
                print("  No payload data received")
        else:
            print("✗ Failed to read system information!")
            print("  Device may not be responding or address is incorrect")
        
        input("\nPress Enter to continue...")
    
    def test_read_realtime_info(self):
        """Test Command 0x0B - Read Real-time Information"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x0B (Read Real-time Info) ===\n")
        
        print("Reading real-time information from motor...")
        response = self.motor.read_realtime_info()
        
        if response:
            print("✓ Successfully read real-time information!")
            print(f"\nResponse Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
                
                # Display parsed data if available
                if 'parsed_data' in response:
                    parsed = response['parsed_data']
                    print(f"\nParsed Real-time Data:")
                    print(f"- Absolute Angle: {parsed['abs_angle_deg']:.2f}° (raw: {parsed['abs_angle_raw']})")
                    print(f"- Multi-turn Angle: {parsed['multi_angle_deg']:.2f}° (raw: {parsed['multi_angle_raw']})")
                    print(f"- Motor Speed: {parsed['speed_rpm']:.2f} RPM (raw: {parsed['speed_raw']})")
                    print(f"- Q-axis Current: {parsed['current_a']:.3f} A (raw: {parsed['current_raw']})")
                    print(f"- Bus Voltage: {parsed['voltage_v']:.2f} V (raw: {parsed['voltage_raw']})")
                    print(f"- Bus Current: {parsed['bus_current_a']:.2f} A (raw: {parsed['bus_current_raw']})")
                    print(f"- Working Temperature: {parsed['work_temp_c']}°C")
                    
                    # Decode operating status
                    op_status = parsed['operating_status']
                    status_names = {
                        0: "Shutdown",
                        1: "Voltage Control", 
                        2: "Current Control",
                        3: "Speed Control",
                        4: "Position Control"
                    }
                    op_name = status_names.get(op_status, f"Unknown({op_status})")
                    print(f"- Operating Status: {op_name} (0x{op_status:02X})")
                    
                    # Decode motor status bits
                    motor_status = parsed['motor_status']
                    print(f"- Motor Status: 0x{motor_status:02X}", end="")
                    if motor_status == 0:
                        print(" (Off)")
                    else:
                        status_bits = []
                        if motor_status & 0x01: status_bits.append("Voltage Ctrl")
                        if motor_status & 0x02: status_bits.append("Voltage Ctrl") 
                        if motor_status & 0x04: status_bits.append("Speed Ctrl")
                        if motor_status & 0x08: status_bits.append("Encoder Fail")
                        if motor_status & 0x40: status_bits.append("HW Fail")
                        if motor_status & 0x80: status_bits.append("SW Fail")
                        print(f" ({', '.join(status_bits) if status_bits else 'On'})")
                    
                    # Decode fault code
                    fault = parsed['fault_code']
                    print(f"- Fault Code: 0x{fault:02X}", end="")
                    if fault == 0:
                        print(" (No Fault)")
                    else:
                        print(f" (Fault Present)")
                else:
                    print(f"\nRaw payload parsing failed, showing hex data only")
            else:
                print("  No payload data received")
        else:
            print("✗ Failed to read real-time information!")
            print("  Device may not be responding or address is incorrect")
        
        input("\nPress Enter to continue...")
    
    def test_clear_faults(self):
        """Test Command 0x0F - Clear Faults"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x0F (Clear Faults) ===\n")
        
        # First, read current fault status
        print("Reading current fault status...")
        realtime_response = self.motor.read_realtime_info()
        
        current_fault = None
        if realtime_response and 'parsed_data' in realtime_response:
            current_fault = realtime_response['parsed_data']['fault_code']
            print(f"Current fault code: 0x{current_fault:02X}", end="")
            if current_fault == 0:
                print(" (No Fault)")
            else:
                print(" (Fault Present)")
        else:
            print("⚠ Could not read current fault status")
        
        print(f"\nSending clear faults command...")
        response = self.motor.clear_faults()
        
        if response:
            print("✓ Clear faults command executed successfully!")
            print(f"\nResponse Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
            else:
                print("  No payload data in response")
            
            # Verify fault clearing by reading status again
            print(f"\nVerifying fault status after clear command...")
            time.sleep(0.1)  # Small delay
            verify_response = self.motor.read_realtime_info()
            
            if verify_response and 'parsed_data' in verify_response:
                new_fault = verify_response['parsed_data']['fault_code']
                print(f"New fault code: 0x{new_fault:02X}", end="")
                if new_fault == 0:
                    print(" (No Fault)")
                    if current_fault is not None and current_fault != 0:
                        print("✓ Faults successfully cleared!")
                    else:
                        print("✓ No faults were present")
                else:
                    print(" (Fault Still Present)")
                    if current_fault is not None and new_fault == current_fault:
                        print("⚠ Fault code unchanged - some faults may require specific conditions to clear")
                    else:
                        print("⚠ Fault code changed but not cleared")
            else:
                print("⚠ Could not verify fault status after clear command")
                
        else:
            print("✗ Failed to execute clear faults command!")
            print("  Device may not be responding or command not supported")
        
        input("\nPress Enter to continue...")
    
    def test_read_user_parameters(self):
        """Test Command 0x10 - Read User Parameters"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x10 (Read User Parameters) ===\n")
        
        print("Reading user parameters from motor...")
        response = self.motor.read_user_parameters()
        
        if response:
            print("✓ Successfully read user parameters!")
            print(f"\nResponse Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
                
                # Display parsed data if available
                if 'parsed_data' in response:
                    parsed = response['parsed_data']
                    print(f"\nParsed User Parameters:")
                    
                    # Angle offsets
                    print(f"  Electrical Angle Offset: {parsed['elec_angle_offset_deg']:.2f}° (raw: {parsed['elec_angle_offset_raw']})")
                    print(f"  Mechanical Angle Offset: {parsed['mech_angle_offset_deg']:.2f}° (raw: {parsed['mech_angle_offset_raw']})")
                    
                    # Current offsets
                    print(f"  U Phase Current Offset: {parsed['u_current_offset']}")
                    print(f"  V Phase Current Offset: {parsed['v_current_offset']}")
                    print(f"  W Phase Current Offset: {parsed['w_current_offset']}")
                    
                    # Encoder settings
                    print(f"  Encoder Model: {parsed['encoder_model']} (code: {parsed['encoder_model_code']})")
                    print(f"  Change Encoder Direction: {'Yes' if parsed['change_encoder_dir'] != 0 else 'No'} ({parsed['change_encoder_dir']})")
                    print(f"  Save Second Encoder: {'Yes' if parsed['save_second_encoder'] != 0 else 'No'} ({parsed['save_second_encoder']})")
                    
                    # Filter and communication settings
                    print(f"  Speed Filter Coefficient: {parsed['speed_filter_value']:.2f} (raw: {parsed['speed_filter_coeff']})")
                    print(f"  Device Address: 0x{parsed['device_address']:02X} ({parsed['device_address']})")
                    print(f"  RS485 Baud Rate: {parsed['rs485_baud']} (code: {parsed['rs485_baud_code']})")
                    print(f"  CAN Baud Rate: {parsed['can_baud']} (code: {parsed['can_baud_code']})")
                    print(f"  Protocol: {parsed['canopen_protocol']} (code: {parsed['save_canopen']})")
                    
                    # Limits and safety settings
                    print(f"  Maximum Bus Voltage: {parsed['max_bus_voltage']:.2f}V (raw: {parsed['max_bus_voltage_raw']})")
                    print(f"  Maximum Bus Current: {parsed['max_bus_current']:.2f}A (raw: {parsed['max_bus_current_raw']})")
                    print(f"  Maximum Temperature: {parsed['max_temperature']}°C")
                    
                    # Fault timing settings
                    print(f"  Fault Duration (Power On): {parsed['fault_duration_power_on']}s")
                    print(f"  Fault Duration (Field 2): {parsed['fault_duration_2']}s")
                    print(f"  Fault Continuous Duration: {parsed['fault_continuous_duration']}s")
                    
                    # Provide hexdump-style output for reference
                    payload = response['payload']
                    print(f"\nDetailed Hex Dump:")
                    for i in range(0, len(payload), 16):
                        chunk = payload[i:i+16]
                        hex_str = ' '.join(f'{b:02X}' for b in chunk)
                        ascii_str = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in chunk)
                        print(f"  {i:04X}: {hex_str:<48} |{ascii_str}|")
                else:
                    print(f"\n⚠ Raw payload parsing failed, showing hex data only")
                    payload = response['payload']
                    if len(payload) > 0:
                        print(f"  Raw hex data: {payload.hex(' ').upper()}")
            else:
                print("  No payload data received")
        else:
            print("✗ Failed to read user parameters!")
            print("  Device may not be responding or command not supported")
        
        input("\nPress Enter to continue...")
    
    def test_read_control_parameters(self):
        """Test Command 0x14 - Read Control Parameters (PID and Output Limits)"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        self.print_header()
        print("=== Test Command 0x14 (Read Control Parameters) ===\n")
        print("Reading control parameters from motor...")
        response = self.motor.read_control_parameters()
        if response:
            print("✓ Successfully read control parameters!")
            print(f"\nResponse Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
                if 'parsed_data' in response:
                    parsed = response['parsed_data']
                    print("\nParsed Control Parameters:")
                    print(f"  Position Loop Kp         : {parsed['position_kp']:.4f}")
                    print(f"  Position Loop Ki         : {parsed['position_ki']:.4f}")
                    print(f"  Position Output Limit    : {parsed['position_output_limit_rpm']:.2f} RPM (raw: {parsed['position_output_limit_raw']})")
                    print(f"  Speed Loop Kp            : {parsed['speed_kp']:.4f}")
                    print(f"  Speed Loop Ki            : {parsed['speed_ki']:.4f}")
                    print(f"  Speed Output Limit       : {parsed['speed_output_limit_a']:.3f} A (raw: {parsed['speed_output_limit_raw']})")
                else:
                    print("\n⚠ Raw payload parsing failed, showing hex data only")
            else:
                print("  No payload data received")
        else:
            print("✗ Failed to read control parameters!")
            print("  Device may not be responding or command not supported")
        input("\nPress Enter to continue...")
    
    def test_write_user_parameters(self):
        """Test Command 0x11 - Write User Parameters (Interactive)"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x11 (Write User Parameters) - Interactive ===\n")
        
        print("This test allows you to interactively modify motor parameters.")
        print("⚠ WARNING: This will modify actual motor parameters!")
        
        # Read current parameters
        print("📖 Reading current user parameters...")
        current_response = self.motor.read_user_parameters()
        
        if not current_response or 'parsed_data' not in current_response:
            print("✗ Failed to read current parameters. Cannot proceed safely.")
            input("Press Enter to continue...")
            return
        
        current_params = current_response['parsed_data']
        
        while True:
            self.print_header()
            print("=== Parameter Selection Menu ===\n")
            
            # Display current values and parameter options
            print("📋 Available parameters to modify:")
            print(f"  1. Encoder Model           : {current_params['encoder_model']} (code: {current_params['encoder_model_code']})")
            print(f"  2. Encoder Direction       : {'Reversed' if current_params['change_encoder_dir'] else 'Normal'}")
            print(f"  3. Second Encoder Save     : {'Enabled' if current_params['save_second_encoder'] else 'Disabled'}")
            print(f"  4. Speed Filter            : {current_params['speed_filter_value']:.2f} (coeff: {current_params['speed_filter_coeff']})")
            print(f"  5. Device Address          : 0x{current_params['device_address']:02X}")
            print(f"  6. RS485 Baud Rate         : {current_params['rs485_baud']} baud")
            print(f"  7. CAN Baud Rate           : {current_params['can_baud']}")
            print(f"  8. CAN Protocol            : {current_params['canopen_protocol']}")
            print(f"  9. Max Bus Voltage         : {current_params['max_bus_voltage']:.2f}V")
            print(f" 10. Fault Duration Power-On : {current_params['fault_duration_power_on']}s")
            print(f" 11. Max Bus Current         : {current_params['max_bus_current']:.2f}A")
            print(f" 12. Fault Duration 2        : {current_params['fault_duration_2']}")
            print(f" 13. Max Temperature         : {current_params['max_temperature']}°C")
            print(f" 14. Fault Continuous Duration: {current_params['fault_continuous_duration']}")
            print(f" -1. Back to Main Menu")
            
            try:
                choice = self.get_user_input("\nSelect parameter to modify (1-14 or -1 to go back)", int)
                
                if choice == -1:
                    break
                elif 1 <= choice <= 14:
                    if self._modify_parameter_interactive(choice, current_params):
                        # Re-read parameters to get updated values
                        print("📖 Reading current user parameters...")
                        updated_response = self.motor.read_user_parameters()
                        if updated_response and 'parsed_data' in updated_response:
                            current_params = updated_response['parsed_data']
                        
                    # input("\nPress Enter to continue...")
                else:
                    print("Invalid choice. Please select 1-14 or -1 to go back.")
                    input("Press Enter to continue...")
                    
            except KeyboardInterrupt:
                print("\nOperation cancelled.")
                break
        
        print(f"\n✓ Write user parameters test completed!")
        # input("\nPress Enter to continue...")

    def _modify_parameter_interactive(self, param_choice, current_params):
        """Interactive parameter modification helper using individual wrapper functions"""
        import time
        
        param_info = {
            1: {
                'name': 'Encoder Model',
                'current_key': 'encoder_model_code',
                'current_display': lambda p: f"{p['encoder_model']} (code: {p['encoder_model_code']})",
                'options': {
                    0: 'AS504x_SPI', 1: 'Ma7xx_SPI', 2: 'MT6835_SPI',
                    3: 'TL5012_SSI', 4: 'AS504x_PWM', 5: 'Tamgawa'
                },
                'function': lambda value: self.motor.change_encoder_model(value),
                'input_type': 'model_code'
            },
            2: {
                'name': 'Encoder Direction',
                'current_key': 'change_encoder_dir',
                'current_display': lambda p: 'Reversed' if p['change_encoder_dir'] else 'Normal',
                'options': {0: 'Normal', 1: 'Reversed'},
                'function': lambda value: self.motor.change_encoder_direction(reverse_direction=bool(value)),
                'input_type': 'boolean'
            },
            3: {
                'name': 'Second Encoder Save',
                'current_key': 'save_second_encoder',
                'current_display': lambda p: 'Enabled' if p['save_second_encoder'] else 'Disabled',
                'options': {0: 'Disabled', 1: 'Enabled'},
                'function': lambda value: self.motor.change_second_encoder_save(enable_save=bool(value)),
                'input_type': 'boolean'
            },
            4: {
                'name': 'Speed Filter',
                'current_key': 'speed_filter_coeff',
                'current_display': lambda p: f"{p['speed_filter_value']:.2f} (coeff: {p['speed_filter_coeff']})",
                'range': (1, 100),
                'unit': 'coefficient (1-100)',
                'function': lambda value: self.motor.change_speed_filter(value),
                'input_type': 'coefficient'
            },
            5: {
                'name': 'Device Address',
                'current_key': 'device_address',
                'current_display': lambda p: f"0x{p['device_address']:02X}",
                'range': (0x01, 0xFE),
                'unit': 'hex (0x01-0xFE)',
                'function': lambda value: self.motor.change_device_address(value),
                'input_type': 'hex_address',
                'warning': 'WARNING: This will change the RS485 communication address!'
            },
            6: {
                'name': 'RS485 Baud Rate',
                'current_key': 'rs485_baud_code',
                'current_display': lambda p: f"{p['rs485_baud']} baud",
                'options': {
                    0: '921600', 1: '460800', 2: '115200', 3: '57600',
                    4: '38400', 5: '19200', 6: '9600'
                },
                'function': lambda value: self.motor.change_rs485_baudrate(int(value)),
                'input_type': 'baud_rate',
                'warning': 'WARNING: This will change the RS485 baud rate!'
            },
            7: {
                'name': 'CAN Baud Rate',
                'current_key': 'can_baud_code',
                'current_display': lambda p: p['can_baud'],
                'options': {0: '1M', 1: '500K', 2: '250K', 3: '125K', 4: '100K'},
                'function': lambda value: self.motor.change_can_baudrate(value),
                'input_type': 'can_baud_string'
            },
            8: {
                'name': 'CAN Protocol',
                'current_key': 'save_canopen',
                'current_display': lambda p: p['canopen_protocol'],
                'options': {0: 'CAN Custom', 1: 'CanOpen'},
                'function': lambda value: self.motor.change_canopen_protocol(use_canopen=bool(value)),
                'input_type': 'boolean'
            },
            9: {
                'name': 'Max Bus Voltage',
                'current_key': 'max_bus_voltage_raw',
                'current_display': lambda p: f"{p['max_bus_voltage']:.2f}V",
                'range': (0.0, 655.35),
                'unit': 'volts (0.0-655.35V)',
                'function': lambda value: self.motor.change_max_bus_voltage(value),
                'input_type': 'voltage'
            },
            10: {
                'name': 'Fault Duration Power-On',
                'current_key': 'fault_duration_power_on',
                'current_display': lambda p: f"{p['fault_duration_power_on']}s",
                'range': (0, 255),
                'unit': 'seconds (0-255)',
                'function': lambda value: self.motor.change_fault_duration_power_on(value),
                'input_type': 'integer'
            },
            11: {
                'name': 'Max Bus Current',
                'current_key': 'max_bus_current_raw',
                'current_display': lambda p: f"{p['max_bus_current']:.2f}A",
                'range': (0.0, 655.35),
                'unit': 'amperes (0.0-655.35A)',
                'function': lambda value: self.motor.change_max_bus_current(value),
                'input_type': 'current'
            },
            12: {
                'name': 'Fault Duration 2',
                'current_key': 'fault_duration_2',
                'current_display': lambda p: str(p['fault_duration_2']),
                'range': (0, 255),
                'unit': 'value (0-255)',
                'function': lambda value: self.motor.change_fault_duration_2(value),
                'input_type': 'integer'
            },
            13: {
                'name': 'Max Temperature',
                'current_key': 'max_temperature',
                'current_display': lambda p: f"{p['max_temperature']}°C",
                'range': (0, 255),
                'unit': 'Celsius (0-255)',
                'function': lambda value: self.motor.change_max_temperature(value),
                'input_type': 'integer'
            },
            14: {
                'name': 'Fault Continuous Duration',
                'current_key': 'fault_continuous_duration',
                'current_display': lambda p: str(p['fault_continuous_duration']),
                'range': (0, 255),
                'unit': 'value (0-255)',
                'function': lambda value: self.motor.change_fault_continuous_duration(value),
                'input_type': 'integer'
            }
        }
        
        if param_choice not in param_info:
            print("Invalid parameter choice.")
            return False
        
        info = param_info[param_choice]
        current_value = current_params[info['current_key']]
        
        print(f"\n=== Modifying {info['name']} ===")
        print(f"Current value: {info['current_display'](current_params)}")
        
        if 'warning' in info:
            print(f"\n⚠️  {info['warning']}")
        
        # Initialize variables
        option_code = None
        new_value = None
        
        # Get input value based on parameter type
        if 'options' in info:
            print(f"\nAvailable options:")
            for code, name in info['options'].items():
                marker = " ← Current" if code == current_value else ""
                print(f"  {code}: {name}{marker}")
            print(f" -1: Back to parameter selection menu")
            
            try:
                user_input = input(f"\nEnter new value (0-{max(info['options'].keys())}, or '-1' to go back): ").strip()
                if user_input == '-1':
                    return False
                
                option_code = int(user_input)
                if option_code not in info['options']:
                    print(f"Invalid option. Must be one of: {list(info['options'].keys())}")
                    return False
                
                # Convert option to appropriate input for wrapper function
                if info['input_type'] == 'model_code':
                    new_value = option_code  # Use code directly
                elif info['input_type'] == 'boolean':
                    new_value = option_code  # Will be converted to bool by lambda
                elif info['input_type'] == 'baud_rate':
                    new_value = info['options'][option_code]  # Use the string value
                elif info['input_type'] == 'can_baud_string':
                    new_value = info['options'][option_code]  # Use the string value
                else:
                    new_value = option_code
                    
            except ValueError:
                print("Invalid input.")
                return False
                
        elif 'range' in info:
            min_val, max_val = info['range']
            print(f"\nValid range: {min_val}-{max_val} ({info['unit']})")
            
            try:
                user_input = input(f"Enter new value ({min_val}-{max_val}), or '-1' to go back: ").strip()
                if user_input == '-1':
                    return False
                
                # Convert input based on type
                if info['input_type'] in ['voltage', 'current']:
                    new_value = float(user_input)
                    if not min_val <= new_value <= max_val:
                        print(f"Value out of range. Must be {min_val}-{max_val}")
                        return False
                elif info['input_type'] == 'hex_address':
                    try:
                        if user_input.startswith('0x') or user_input.startswith('0X'):
                            new_value = int(user_input, 16)
                        else:
                            new_value = int(user_input)  # Assume hex even without 0x
                        if not min_val <= new_value <= max_val:
                            print(f"Address out of range. Must be 0x{min_val:02X}-0x{max_val:02X}")
                            return False
                    except ValueError:
                        print("Invalid hexadecimal input.")
                        return False
                else:  # integer type
                    new_value = int(user_input)
                    if not min_val <= new_value <= max_val:
                        print(f"Value out of range. Must be {min_val}-{max_val}")
                        return False
                        
            except ValueError:
                print("Invalid input.")
                return False
        else:
            print("No modification options available for this parameter.")
            return False
        
        # Confirm the change
        old_display = info['current_display'](current_params)
        
        if 'options' in info and option_code is not None:
            new_display = info['options'][option_code]
        else:
            if info['input_type'] == 'hex_address':
                new_display = f"0x{new_value:02X}"
            elif info['input_type'] == 'voltage':
                new_display = f"{new_value:.2f}V"
            elif info['input_type'] == 'current':
                new_display = f"{new_value:.2f}A"
            else:
                new_display = str(new_value)
        
        print(f"\nConfirm parameter change:")
        print(f"  Parameter: {info['name']}")
        print(f"  Old value: {old_display}")
        print(f"  New value: {new_display}")
        
        confirm = input(f"\nProceed with change? (y/n): ").strip().lower()
        if confirm != 'y':
            print("Change cancelled.")
            return False
        
        # Call the appropriate wrapper function
        print(f"\n📝 Writing parameter change using wrapper function...")
        
        try:
            write_response = info['function'](new_value)
            
            if write_response:
                print("✓ Parameter change executed successfully!")
                
                # Verify the change
                print(f"🔍 Verifying parameter change...")
                time.sleep(0.2)
                
                verify_response = self.motor.read_user_parameters()
                if verify_response and 'parsed_data' in verify_response:
                    verify_params = verify_response['parsed_data']
                    print(f"✓ Parameter change verified!")
                    print(f"  New value: {info['current_display'](verify_params)}")
                    print("Parameter change is now active.")
                else:
                    print("⚠ Could not verify parameter change")
                    
                return True
            else:
                print("✗ Failed to write parameter!")
                return False
                
        except Exception as e:
            print(f"✗ Error calling wrapper function: {e}")
            return False
    

    def test_manual_packet(self):
        """Test manual packet construction and transmission"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Manual Packet Construction and Test ===\n")
        
        print("This function allows you to manually construct and send packets.")
        print("Enter the packet data (without CRC) and the system will calculate CRC automatically.\n")
        
        print("Packet format: Header + PacketID + DeviceAddr + Command + DataLength + Data")
        print("Example formats:")
        print("  Read realtime info:  AE 01 01 0B 00")
        print("  Custom command: AE 01 01 [CMD] [LEN] [DATA...]")
        print()
        
        while True:
            try:
                # Get packet data from user
                packet_input = input("Enter packet hex data (without CRC, or 'q' to quit): ").strip()
                
                if packet_input.lower() == 'q':
                    break
                
                if not packet_input:
                    print("Please enter packet data or 'q' to quit.\n")
                    continue
                
                # Parse hex input
                try:
                    # Remove any extra spaces and convert to bytes
                    hex_parts = packet_input.replace(',', ' ').split()
                    packet_data = bytes([int(part, 16) for part in hex_parts])
                except ValueError as e:
                    print(f"✗ Invalid hex format: {e}")
                    print("Please enter hex values separated by spaces (e.g., AE 01 01 10 00)\n")
                    continue
                
                if len(packet_data) < 5:
                    print("✗ Packet too short. Minimum 5 bytes: Header + PacketID + DeviceAddr + Command + DataLength")
                    continue
                
                # Validate basic packet structure
                header = packet_data[0]
                packet_id = packet_data[1]
                device_addr = packet_data[2]
                command = packet_data[3]
                data_length = packet_data[4]
                
                print(f"\nParsed packet:")
                print(f"  Header: 0x{header:02X}")
                print(f"  Packet ID: 0x{packet_id:02X}")
                print(f"  Device Address: 0x{device_addr:02X}")
                print(f"  Command: 0x{command:02X}")
                print(f"  Data Length: {data_length}")
                
                # Validate data length
                expected_total_length = 5 + data_length  # Header + PacketID + DeviceAddr + Command + DataLength + Data
                if len(packet_data) != expected_total_length:
                    print(f"✗ Data length mismatch. Expected {expected_total_length} bytes total, got {len(packet_data)}")
                    print(f"  Expected: 5 header bytes + {data_length} data bytes = {expected_total_length}")
                    continue
                
                if data_length > 0:
                    data_payload = packet_data[5:]
                    print(f"  Data ({data_length} bytes): {data_payload.hex(' ').upper()}")
                else:
                    data_payload = b''
                    print(f"  No data payload")
                
                # Calculate CRC16
                crc = self.motor.crc16_func(packet_data)
                crc_bytes = struct.pack('<H', crc)  # Little-endian CRC
                
                # Complete packet with CRC
                complete_packet = packet_data + crc_bytes
                
                print(f"\nComplete packet with CRC:")
                print(f"  Packet: {complete_packet.hex(' ').upper()}")
                print(f"  CRC16: 0x{crc:04X} (bytes: {crc_bytes.hex(' ').upper()})")
                
                # Ask for confirmation
                send_confirm = input(f"\nSend this packet? (y/n): ").strip().lower()
                
                if send_confirm == 'y':
                    print(f"\nSending packet...")
                    
                    # Determine if we expect a response based on command
                    expect_response = command not in [0x00]  # Only restart (0x00) doesn't expect response
                    
                    try:
                        # Send raw packet
                        self.motor.serial_conn.write(complete_packet)
                        print(f"✓ Packet sent: {complete_packet.hex(' ').upper()}")
                        
                        if expect_response:
                            print(f"Waiting for response...")
                            
                            # Read response with timeout
                            import time
                            start_time = time.time()
                            response_data = b''
                            
                            while time.time() - start_time < self.motor.timeout:
                                if self.motor.serial_conn.in_waiting > 0:
                                    chunk = self.motor.serial_conn.read(self.motor.serial_conn.in_waiting)
                                    response_data += chunk
                                    
                                    # Check if we have a complete response
                                    if len(response_data) >= 7:  # Minimum response length
                                        # Try to parse the response
                                        try:
                                            if response_data[0] == 0xAC:  # Response header
                                                expected_length = 7 + response_data[4]  # Header + fields + data + CRC
                                                if len(response_data) >= expected_length:
                                                    break
                                        except IndexError:
                                            pass
                                
                                time.sleep(0.01)  # Small delay
                            
                            if response_data:
                                print(f"✓ Response received ({len(response_data)} bytes):")
                                print(f"  Raw: {response_data.hex(' ').upper()}")
                                
                                # Try to parse response
                                if len(response_data) >= 7 and response_data[0] == 0xAC:
                                    resp_packet_id = response_data[1]
                                    resp_device_addr = response_data[2]
                                    resp_command = response_data[3]
                                    resp_data_length = response_data[4]
                                    
                                    print(f"  Parsed response:")
                                    print(f"    Header: 0x{response_data[0]:02X}")
                                    print(f"    Packet ID: 0x{resp_packet_id:02X}")
                                    print(f"    Device Address: 0x{resp_device_addr:02X}")
                                    print(f"    Command: 0x{resp_command:02X}")
                                    print(f"    Data Length: {resp_data_length}")
                                    
                                    if resp_data_length > 0 and len(response_data) >= 5 + resp_data_length:
                                        resp_data = response_data[5:5+resp_data_length]
                                        print(f"    Data: {resp_data.hex(' ').upper()}")
                                    
                                    # Verify CRC
                                    if len(response_data) >= 5 + resp_data_length + 2:
                                        response_without_crc = response_data[:-2]
                                        received_crc = struct.unpack('<H', response_data[-2:])[0]
                                        calculated_crc = self.motor.crc16_func(response_without_crc)
                                        
                                        if received_crc == calculated_crc:
                                            print(f"    CRC: 0x{received_crc:04X} ✓ Valid")
                                        else:
                                            print(f"    CRC: 0x{received_crc:04X} ✗ Invalid (expected 0x{calculated_crc:04X})")
                                else:
                                    print(f"  Unable to parse response (invalid format)")
                            else:
                                print(f"✗ No response received within timeout")
                        else:
                            print(f"✓ Command sent (no response expected)")
                    
                    except Exception as e:
                        print(f"✗ Error sending packet: {e}")
                
                print()  # Empty line for readability
                
            except KeyboardInterrupt:
                print("\nOperation cancelled.")
                break
            except Exception as e:
                print(f"✗ Unexpected error: {e}")
        
        print("\nManual packet test completed.")
        input("Press Enter to continue...")
    
    def reset_connection(self):
        """Reset connection settings"""
        self.print_header()
        print("=== Reset Connection ===\n")
        
        if self.connected:
            print("Disconnecting current connection...")
            self.motor.disconnect()
            self.connected = False
        
        print("Clearing cached connection settings...")
        self.clear_connection_cache()
        
        print("Setting up new connection...")
        success = self.setup_new_connection()
        
        if not success:
            print("Failed to setup new connection.")
        
        input("Press Enter to continue...")
    
    def toggle_debug_logging(self):
        """Toggle debug logging"""
        self.print_header()
        print("=== Logging Configuration ===\n")
        
        current_level = logging.getLogger().getEffectiveLevel()
        print(f"Current logging level: {logging.getLevelName(current_level)}")
        
        print("\nSelect logging level:")
        print("1. INFO (default)")
        print("2. DEBUG (verbose)")
        print("3. WARNING (minimal)")
        
        choice = self.get_user_input("Select option (1-3)", int, 1)
        
        if choice == 1:
            setup_logging(logging.INFO)
            print("✓ Logging set to INFO level")
        elif choice == 2:
            setup_logging(logging.DEBUG)
            print("✓ Logging set to DEBUG level (verbose)")
        elif choice == 3:
            setup_logging(logging.WARNING)
            print("✓ Logging set to WARNING level (minimal)")
        
        input("\nPress Enter to continue...")
    
    def show_main_menu(self):
        """Display main menu and get user choice"""
        self.print_header()
        print("Available Commands:\n")
        print("1. Test Command 0x00 (Restart Slave)")
        print("2. Test Command 0x0A (Read System Info)")
        print("3. Test Command 0x0B (Read Real-time Info)")
        print("4. Test Command 0x0F (Clear Faults)")
        print("5. Test Command 0x10 (Read User Parameters)")
        print("6. Test Command 0x11 (Write User Parameters)")
        print("7. Test Command 0x14 (Read Control Parameters)")
        print("8. Test Command 0x15 (Write Control Parameters)")
        print("9. Test Command 0x16 (Write and Save Control Parameters)")
        print("10. Test Command 0x1D (Set Zero Position)")
        print("11. Test Command 0x1E (Calibrate Encoder)")
        print("12. Test Manual Packet")
        print("13. Reset Connection")
        print("14. Enable/Disable Debug Logging")
        print("15. Test Command 0x1F (Factory Reset)")
        print("16. Exit")
        print()
        choice = self.get_user_input("Select option (1-16)", int)
        return choice
    
    def run(self):
        """Main program loop"""
        setup_logging(logging.INFO)
        
        # Check conda environment
        self.check_conda_environment()
        
        print("\n" + "=" * 60)
        print("    SteadyWin RS485 Motor Test System")
        print("=" * 60)
        
        # Auto-connect on startup
        print("Connecting to motor...")
        if not self.auto_connect():
            print("Failed to establish connection. Exiting...")
            return
        
        print(f"✓ Connected to motor at {self.port} (Address: 0x{self.device_addr:02X})")
        input("Press Enter to continue to main menu...")
        
        try:
            while True:
                choice = self.show_main_menu()
                
                if choice == 1:
                    self.test_restart_slave()
                elif choice == 2:
                    self.test_read_system_info()
                elif choice == 3:
                    self.test_read_realtime_info()
                elif choice == 4:
                    self.test_clear_faults()
                elif choice == 5:
                    self.test_read_user_parameters()
                elif choice == 6:
                    self.test_write_user_parameters()
                elif choice == 7:
                    self.test_read_control_parameters()
                elif choice == 8:
                    self.test_write_control_parameters()
                elif choice == 9:
                    self.test_write_control_parameters_and_save()
                elif choice == 10:
                    self.test_set_zero_position()
                elif choice == 11:
                    self.test_calibrate_encoder()
                elif choice == 12:
                    self.test_manual_packet()
                elif choice == 13:
                    self.reset_connection()
                elif choice == 14:
                    self.toggle_debug_logging()
                elif choice == 15:
                    self.test_factory_reset()
                elif choice == 16:
                    break
                else:
                    print("Invalid choice. Please select 1-16.")
                    input("Press Enter to continue...")
        
        except KeyboardInterrupt:
            print("\n\nExiting...")
        
        finally:
            if self.motor and self.connected:
                self.motor.disconnect()
            print("Goodbye!")

    def test_write_control_parameters(self):
        """Test Command 0x15 - Write Control Parameters (Interactive)"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        self.print_header()
        print("=== Test Command 0x15 (Write Control Parameters) - Interactive ===\n")
        print("This test allows you to interactively modify control (PID/output) parameters.")
        print("⚠ WARNING: This will modify actual control parameters (temporarily)!")
        
        # Read current control parameters
        print("📖 Reading current control parameters...")
        current_response = self.motor.read_control_parameters()
        
        if not current_response or 'parsed_data' not in current_response:
            print("✗ Failed to read current control parameters. Cannot proceed safely.")
            input("Press Enter to continue...")
            return
        
        current_params = current_response['parsed_data']
        
        param_info = [
            {'key': 'position_kp', 'label': 'Position Loop Kp', 'type': float},
            {'key': 'position_ki', 'label': 'Position Loop Ki', 'type': float},
            {'key': 'position_output_limit_rpm', 'label': 'Position Output Limit (RPM)', 'type': float},
            {'key': 'speed_kp', 'label': 'Speed Loop Kp', 'type': float},
            {'key': 'speed_ki', 'label': 'Speed Loop Ki', 'type': float},
            {'key': 'speed_output_limit_a', 'label': 'Speed Output Limit (A)', 'type': float},
        ]
        
        while True:
            self.print_header()
            print("=== Control Parameter Modification Menu (TEMPORARY) ===\n")
            
            for idx, info in enumerate(param_info, 1):
                val = current_params[info['key']]
                print(f"  {idx}. {info['label']:<28}: {val}")
            print(" -1. Back to Main Menu")
            
            try:
                choice = self.get_user_input("\nSelect parameter to modify (1-6 or -1 to go back)", int)
                
                if choice == -1:
                    break
                elif 1 <= choice <= 6:
                    info = param_info[choice-1]
                    current_val = current_params[info['key']]
                    new_val = self.get_user_input(f"Enter new value for {info['label']}", info['type'], current_val)
                    
                    # Confirm
                    print(f"\nConfirm TEMPORARY parameter change:")
                    print(f"  Parameter: {info['label']}")
                    print(f"  Old value: {current_val}")
                    print(f"  New value: {new_val}")
                    print(f"  ℹ This change is temporary (lost on motor restart)")
                    confirm = input("\nProceed with temporary change? (y/n): ").strip().lower()
                    
                    if confirm != 'y':
                        print("Change cancelled.")
                        continue
                    
                    # Build new parameter dict for write
                    write_dict = {
                        'position_kp': current_params['position_kp'],
                        'position_ki': current_params['position_ki'],
                        'position_output_limit_raw': int(round(current_params['position_output_limit_rpm'] / 0.01)),
                        'speed_kp': current_params['speed_kp'],
                        'speed_ki': current_params['speed_ki'],
                        'speed_output_limit_raw': int(round(current_params['speed_output_limit_a'] / 0.001)),
                    }
                    
                    # Update the selected parameter
                    if info['key'] == 'position_output_limit_rpm':
                        write_dict['position_output_limit_raw'] = int(round(float(new_val) / 0.01))
                    elif info['key'] == 'speed_output_limit_a':
                        write_dict['speed_output_limit_raw'] = int(round(float(new_val) / 0.001))
                    else:
                        write_dict[info['key']] = float(new_val)
                    
                    print("\n📝 Writing control parameter change (temporary)...")
                    write_response = self.motor.write_control_parameters(parameters_dict=write_dict)
                    
                    if write_response:
                        print("✓ Control parameter change executed successfully!")
                        print("🔍 Verifying parameter change...")
                        import time
                        time.sleep(0.2)
                        verify_response = self.motor.read_control_parameters()
                        if verify_response and 'parsed_data' in verify_response:
                            verify_params = verify_response['parsed_data']
                            print("✓ Parameter change verified!")
                            print(f"  New value: {verify_params[info['key']]}")
                            current_params = verify_params
                        else:
                            print("⚠ Could not verify parameter change")
                    else:
                        print("✗ Failed to write control parameter!")
                else:
                    print("Invalid choice. Please select 1-6 or -1 to go back.")
                    input("Press Enter to continue...")
                    
            except KeyboardInterrupt:
                print("\nOperation cancelled.")
                break
        
        print(f"\n✓ Write control parameters test completed!")

    def test_write_control_parameters_and_save(self):
        """Test Command 0x16 - Write and Save Control Parameters (Interactive)"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        self.print_header()
        print("=== Test Command 0x16 (Write and Save Control Parameters) - Interactive ===\n")
        print("This test allows you to interactively modify control (PID/output) parameters.")
        print("⚠ WARNING: This will PERMANENTLY modify and save control parameters!")
        
        # Read current control parameters
        print("📖 Reading current control parameters...")
        current_response = self.motor.read_control_parameters()
        
        if not current_response or 'parsed_data' not in current_response:
            print("✗ Failed to read current control parameters. Cannot proceed safely.")
            input("Press Enter to continue...")
            return
        
        current_params = current_response['parsed_data']
        
        param_info = [
            {'key': 'position_kp', 'label': 'Position Loop Kp', 'type': float},
            {'key': 'position_ki', 'label': 'Position Loop Ki', 'type': float},
            {'key': 'position_output_limit_rpm', 'label': 'Position Output Limit (RPM)', 'type': float},
            {'key': 'speed_kp', 'label': 'Speed Loop Kp', 'type': float},
            {'key': 'speed_ki', 'label': 'Speed Loop Ki', 'type': float},
            {'key': 'speed_output_limit_a', 'label': 'Speed Output Limit (A)', 'type': float},
        ]
        
        while True:
            self.print_header()
            print("=== Control Parameter Modification Menu (SAVE TO MEMORY) ===\n")
            
            for idx, info in enumerate(param_info, 1):
                val = current_params[info['key']]
                print(f"  {idx}. {info['label']:<28}: {val}")
            print(" -1. Back to Main Menu")
            
            try:
                choice = self.get_user_input("\nSelect parameter to modify (1-6 or -1 to go back)", int)
                
                if choice == -1:
                    break
                elif 1 <= choice <= 6:
                    info = param_info[choice-1]
                    current_val = current_params[info['key']]
                    new_val = self.get_user_input(f"Enter new value for {info['label']}", info['type'], current_val)
                    
                    # Confirm
                    print(f"\nConfirm PERMANENT parameter change:")
                    print(f"  Parameter: {info['label']}")
                    print(f"  Old value: {current_val}")
                    print(f"  New value: {new_val}")
                    print(f"  ⚠ This change will be SAVED to non-volatile memory!")
                    confirm = input("\nProceed with PERMANENT change? (y/n): ").strip().lower()
                    
                    if confirm != 'y':
                        print("Change cancelled.")
                        continue
                    
                    # Build new parameter dict for write
                    write_dict = {
                        'position_kp': current_params['position_kp'],
                        'position_ki': current_params['position_ki'],
                        'position_output_limit_raw': int(round(current_params['position_output_limit_rpm'] / 0.01)),
                        'speed_kp': current_params['speed_kp'],
                        'speed_ki': current_params['speed_ki'],
                        'speed_output_limit_raw': int(round(current_params['speed_output_limit_a'] / 0.001)),
                    }
                    
                    # Update the selected parameter
                    if info['key'] == 'position_output_limit_rpm':
                        write_dict['position_output_limit_raw'] = int(round(float(new_val) / 0.01))
                    elif info['key'] == 'speed_output_limit_a':
                        write_dict['speed_output_limit_raw'] = int(round(float(new_val) / 0.001))
                    else:
                        write_dict[info['key']] = float(new_val)
                    
                    print("\n📝 Writing and saving control parameter change...")
                    write_response = self.motor.write_control_parameters_and_save(parameters_dict=write_dict)
                    
                    if write_response:
                        print("✓ Control parameter change executed and saved successfully!")
                        print("🔍 Verifying parameter change...")
                        import time
                        time.sleep(0.2)
                        verify_response = self.motor.read_control_parameters()
                        if verify_response and 'parsed_data' in verify_response:
                            verify_params = verify_response['parsed_data']
                            print("✓ Parameter change verified!")
                            print(f"  New value: {verify_params[info['key']]}")
                            current_params = verify_params
                        else:
                            print("⚠ Could not verify parameter change")
                    else:
                        print("✗ Failed to write and save control parameter!")
                else:
                    print("Invalid choice. Please select 1-6 or -1 to go back.")
                    input("Press Enter to continue...")
                    
            except KeyboardInterrupt:
                print("\nOperation cancelled.")
                break
        
        print(f"\n✓ Write and save control parameters test completed!")

    def test_set_zero_position(self):
        """Test Command 0x1D - Set Zero Position"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x1D (Set Zero Position) ===\n")
        
        print("This command sets the current motor position as the zero/origin reference point.")
        print("⚠ WARNING: This will change the motor's position reference!")
        
        # Read current position first
        print("📖 Reading current motor position...")
        realtime_response = self.motor.read_realtime_info()
        
        if realtime_response and 'parsed_data' in realtime_response:
            current_data = realtime_response['parsed_data']
            print(f"✓ Current position information:")
            print(f"  Absolute Angle: {current_data['abs_angle_deg']:.2f}°")
            print(f"  Multi-turn Angle: {current_data['multi_angle_deg']:.2f}°")
        else:
            print("⚠ Could not read current position")
        
        # Confirm action
        print(f"\nConfirm zero position setting:")
        print(f"  This will set the current position as the new zero reference")
        print(f"  ⚠ Position measurements will be relative to this new zero point")
        confirm = input("\nProceed with setting zero position? (y/n): ").strip().lower()
        
        if confirm != 'y':
            print("Operation cancelled.")
            input("\nPress Enter to continue...")
            return
        
        print("\n🎯 Setting current position as zero...")
        response = self.motor.set_zero_position()
        
        if response:
            print("✓ Zero position set successfully!")
            print(f"\nResponse Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
            else:
                print("  No payload data received")
            
            # Read position again to verify
            print("\n🔍 Verifying new zero position...")
            import time
            time.sleep(0.2)
            verify_response = self.motor.read_realtime_info()
            
            if verify_response and 'parsed_data' in verify_response:
                verify_data = verify_response['parsed_data']
                print("✓ Position after zero setting:")
                print(f"  Absolute Angle: {verify_data['abs_angle_deg']:.2f}°")
                print(f"  Multi-turn Angle: {verify_data['multi_angle_deg']:.2f}°")
            else:
                print("⚠ Could not verify new position")
                
        else:
            print("✗ Failed to set zero position!")
            print("  Device may not be responding or command not supported")
        
        input("\nPress Enter to continue...")

    def test_calibrate_encoder(self):
        """Test Command 0x1E - Calibrate Encoder"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x1E (Calibrate Encoder) ===\n")
        
        print("This command performs encoder calibration.")
        print("⚠ Ensure the motor is in a safe state and can rotate freely.")
        
        # Read current system state first
        print("\n📖 Reading current motor status...")
        realtime_response = self.motor.read_realtime_info()
        
        if realtime_response and 'parsed_data' in realtime_response:
            current_data = realtime_response['parsed_data']
            print(f"✓ Current motor status:")
            print(f"  Absolute Angle: {current_data['abs_angle_deg']:.2f}°")
            print(f"  Multi-turn Angle: {current_data['multi_angle_deg']:.2f}°")
            print(f"  Motor Speed: {current_data['speed_rpm']:.2f} RPM")
            print(f"  Operating Status: {current_data['operating_status']} (0=Shutdown, 1=Voltage, 2=Current, 3=Speed, 4=Position)")
            print(f"  Motor Status: 0x{current_data['motor_status']:02X}")
            print(f"  Fault Code: 0x{current_data['fault_code']:02X}")
        else:
            print("⚠ Could not read current motor status")
        
        # Single confirmation
        confirm = input("\nProceed with encoder calibration? (y/n): ").strip().lower()
        if confirm != 'y':
            print("Calibration cancelled.")
            input("\nPress Enter to continue...")
            return
        
        print("\n🔧 Starting encoder calibration...")
        print("Note: Motor may rotate during calibration process...")
        print("📊 Monitoring calibration progress (this may take 60+ seconds)...")
        
        response = self.motor.calibrate_encoder(timeout_seconds=90, poll_interval=1.0)
        
        if response:
            print("✓ Encoder calibration process completed!")
            print(f"\nCalibration Details:")
            
            if 'calibration_duration' in response:
                duration = response['calibration_duration']
                print(f"  Duration: {duration:.1f} seconds")
            
            if 'calibration_timeout' in response and response['calibration_timeout']:
                print("  ⚠ Calibration timed out!")
            elif 'calibration_completed' in response and response['calibration_completed']:
                print("  ✓ Calibration completed successfully")
                if 'final_motor_status' in response:
                    print(f"  Final Motor Status: 0x{response['final_motor_status']:02X}")
                if 'final_operating_status' in response:
                    print(f"  Final Operating Status: {response['final_operating_status']}")
            
            print(f"\nCommand Response Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
            else:
                print("  No payload data received")
            
            # Read final status
            print("\n🔍 Reading final motor status...")
            import time
            time.sleep(0.2)
            verify_response = self.motor.read_realtime_info()
            
            if verify_response and 'parsed_data' in verify_response:
                verify_data = verify_response['parsed_data']
                print("✓ Final motor status:")
                print(f"  Absolute Angle: {verify_data['abs_angle_deg']:.2f}°")
                print(f"  Multi-turn Angle: {verify_data['multi_angle_deg']:.2f}°")
                print(f"  Motor Speed: {verify_data['speed_rpm']:.2f} RPM")
                print(f"  Operating Status: {verify_data['operating_status']}")
                print(f"  Motor Status: 0x{verify_data['motor_status']:02X}")
                print(f"  Fault Code: 0x{verify_data['fault_code']:02X}")
                
                if verify_data['fault_code'] == 0:
                    print("✓ No faults detected")
                else:
                    print("⚠ Fault detected after calibration!")
            else:
                print("⚠ Could not read final motor status")
                
        else:
            print("✗ Failed to perform encoder calibration!")
            print("  Device may not be responding or calibration command failed")
        
        input("\nPress Enter to continue...")

    def test_factory_reset(self):
        """Test Command 0x1F - Factory Reset"""
        if not self.connected:
            print("✗ Not connected to motor.")
            input("Press Enter to continue...")
            return
        
        self.print_header()
        print("=== Test Command 0x1F (Factory Reset) ===\n")
        
        print("🔴 DANGER: This command will restore most motor parameters to factory defaults!")
        print("⚠ Most custom settings will be permanently lost, including:")
        print("  • User parameters (speeds, acceleration, etc.)")
        print("  • Control parameters (PID settings, output limits)")
        print("  • Other configurable settings")
        print("\n✓ The following parameters will NOT be changed:")
        print("  • Device address")
        print("  • Encoder calibration parameters")
        print("  • Motor hardware parameters")
        
        # Read current parameters to show what will be lost
        print("\n📖 Reading current motor configuration...")
        
        # Read user parameters
        user_response = self.motor.read_user_parameters()
        if user_response and 'parsed_data' in user_response:
            user_data = user_response['parsed_data']
            print(f"✓ Current user parameters:")
            print(f"  Encoder Model: {user_data['encoder_model']}")
            print(f"  Device Address: 0x{user_data['device_address']:02X}")
            print(f"  RS485 Baud Rate: {user_data['rs485_baud']}")
            print(f"  CAN Protocol: {user_data['canopen_protocol']}")
        else:
            print("⚠ Could not read current user parameters")
        
        # Read control parameters
        control_response = self.motor.read_control_parameters()
        if control_response and 'parsed_data' in control_response:
            control_data = control_response['parsed_data']
            print(f"✓ Current control parameters:")
            print(f"  Position Kp: {control_data['position_kp']}")
            print(f"  Speed Kp: {control_data['speed_kp']}")
        else:
            print("⚠ Could not read current control parameters")
        
        # Multiple confirmations for safety
        print(f"\n🚨 FACTORY RESET CONFIRMATION:")
        print(f"  This action CANNOT be undone!")
        print(f"  You will need to reconfigure most settings after reset")
        print(f"  (Device address, encoder calibration, and hardware params preserved)")
        
        confirm1 = input("\nDo you understand that most settings will be lost? (yes/no): ").strip().lower()
        if confirm1 != 'yes':
            print("Operation cancelled.")
            input("\nPress Enter to continue...")
            return
        
        confirm2 = input("\nAre you absolutely sure you want to factory reset? (YES/no): ").strip()
        if confirm2 != 'YES':
            print("Operation cancelled.")
            input("\nPress Enter to continue...")
            return
        
        # Final confirmation
        print(f"\n⚠ FINAL WARNING:")
        print(f"  This will permanently reset most motor parameters to factory defaults")
        print(f"  Type 'RESET' to proceed or anything else to cancel")
        
        final_confirm = input("\nType 'RESET' to confirm: ").strip()
        if final_confirm != 'RESET':
            print("Factory reset cancelled.")
            input("\nPress Enter to continue...")
            return
        
        print("\n🔧 Performing factory reset...")
        print("⚠ Do not power off the motor during this operation!")
        
        response = self.motor.factory_reset()
        
        if response:
            print("✓ Factory reset completed successfully!")
            print(f"\nResponse Details:")
            print(f"  Packet ID: 0x{response['packet_id']:02X}")
            print(f"  Device Address: 0x{response['device_addr']:02X}")
            print(f"  Command: 0x{response['command']:02X}")
            print(f"  Payload Length: {len(response['payload'])} bytes")
            
            if response['payload']:
                print(f"  Raw Payload: {response['payload'].hex(' ').upper()}")
            else:
                print("  No payload data received")
            
            print(f"\n🔄 Motor has been reset to factory defaults!")
            print(f"📝 You will need to reconfigure user and control parameters for your application")
            print(f"✓ Device address, encoder calibration, and hardware params are preserved")
            
            # Read status after reset to verify
            print("\n🔍 Verifying factory reset...")
            import time
            time.sleep(1.0)  # Give time for reset to complete
            
            # Try to read user parameters to see defaults
            verify_response = self.motor.read_user_parameters()
            if verify_response and 'parsed_data' in verify_response:
                verify_data = verify_response['parsed_data']
                print("✓ Factory default user parameters:")
                print(f"  Encoder Model: {verify_data['encoder_model']}")
                print(f"  Device Address: 0x{verify_data['device_address']:02X}")
                print(f"  RS485 Baud Rate: {verify_data['rs485_baud']}")
                print(f"  CAN Protocol: {verify_data['canopen_protocol']}")
            else:
                print("⚠ Could not read parameters after reset")
                
        else:
            print("✗ Failed to perform factory reset!")
            print("  Device may not be responding or command not supported")
        
        input("\nPress Enter to continue...")


def main():
    """Main function"""
    print("SteadyWin RS485 Motor Interactive Test System")
    print("Initializing...")
    
    # Start interactive tester
    tester = SteadyWinRS485Tester()
    tester.run()


if __name__ == "__main__":
    main() 
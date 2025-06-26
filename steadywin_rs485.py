"""
SteadyWin BLDC Motor RS485 Protocol Library

This library implements the RS485 communication protocol for SteadyWin BLDC motors
based on the official protocol specification document.

Protocol Packet Structure (from manual):
- Header: 0xAE (1 byte) - Command packet header
- Packet ID: 0x00-0xFF (1 byte) - Incremental packet ID
- Device Address: 0x00-0xFF (1 byte) - Motor device address  
- Command: 0x00-0xFF (1 byte) - Command code
- Data Length: 0x00-0xFF (1 byte) - Payload data length
- Data: 0-255 bytes - Command payload data
- CRC16: 2 bytes (Little Endian) - Modbus CRC16 checksum

Response Packet Structure:
- Header: 0xAC (1 byte) - Response packet header
- Packet ID: Same as command (1 byte)
- Device Address: Same as command (1 byte)
- Command: Same as command (1 byte)
- Data Length: 0x00-0xFF (1 byte) - Response payload length
- Data: 0-255 bytes - Response payload data
- CRC16: 2 bytes (Little Endian) - Modbus CRC16 checksum
"""

import serial
import struct
import time
import logging
import crcmod.predefined

class SteadyWinRS485:
    """SteadyWin BLDC Motor RS485 Communication Class"""
    
    # Protocol Constants
    CMD_HEADER = 0xAE      # Command packet header
    RESP_HEADER = 0xAC     # Response packet header
    MAX_PAYLOAD_SIZE = 255 # Maximum payload size
    MIN_PACKET_SIZE = 7    # Minimum packet size (header + id + addr + cmd + len + crc16)
    
    def __init__(self, port, baudrate=115200, timeout=1.0, device_addr=0x01):
        """
        Initialize SteadyWin Motor communication
        
        Args:
            port (str): Serial port device path (e.g., '/dev/ttyUSB0')
            baudrate (int): Communication baud rate (default: 115200)
            timeout (float): Communication timeout in seconds (default: 1.0)
            device_addr (int): Motor device address (default: 0x01)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.device_addr = device_addr
        self.serial_conn = None
        self.packet_id = 0
        
        # Initialize CRC16 function (Modbus CRC16)
        self.crc16_func = crcmod.predefined.mkPredefinedCrcFun('modbus')
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        
    def connect(self):
        """
        Establish serial connection to the motor
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE
            )
            
            # Clear any existing data in buffers
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.logger.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.logger.info("Disconnected from motor")
    
    def _build_packet(self, command, payload=b''):
        """
        Build command packet according to protocol specification
        
        Args:
            command (int): Command code (0x00-0xFF)
            payload (bytes): Command payload data
            
        Returns:
            bytes: Complete packet with header, command, payload, and CRC16
        """
        if len(payload) > self.MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {len(payload)} > {self.MAX_PAYLOAD_SIZE}")
        
        # Increment packet ID (0-255, wrapping)
        self.packet_id = (self.packet_id + 1) % 256
        
        # Build packet without CRC
        packet_data = struct.pack('<BBBBB', 
                                 self.CMD_HEADER,    # Header
                                 self.packet_id,     # Packet ID
                                 self.device_addr,   # Device Address
                                 command,            # Command
                                 len(payload))       # Data Length
        packet_data += payload
        
        # Calculate and append CRC16
        crc16 = self.crc16_func(packet_data)
        packet_data += struct.pack('<H', crc16)
        
        return packet_data
    
    def _parse_response(self, raw_data):
        """
        Parse response packet and validate structure
        
        Args:
            raw_data (bytes): Raw response data from serial port
            
        Returns:
            dict: Parsed response data or None if invalid
        """
        if len(raw_data) < self.MIN_PACKET_SIZE:
            self.logger.error(f"Response too short: {len(raw_data)} bytes")
            return None
        
        # Parse header
        header = raw_data[0]
        if header != self.RESP_HEADER:
            self.logger.error(f"Invalid response header: 0x{header:02X} (expected 0x{self.RESP_HEADER:02X})")
            return None
        
        # Parse packet fields
        packet_id = raw_data[1]
        device_addr = raw_data[2]
        command = raw_data[3]
        data_length = raw_data[4]
        
        # Check if we have enough data
        expected_size = 5 + data_length + 2  # header + fields + payload + crc16
        if len(raw_data) < expected_size:
            self.logger.error(f"Incomplete response: {len(raw_data)} < {expected_size} bytes")
            return None
        
        # Extract payload and CRC
        payload = raw_data[5:5+data_length]
        crc_received = struct.unpack('<H', raw_data[5+data_length:5+data_length+2])[0]
        
        # Verify CRC16
        crc_calc = self.crc16_func(raw_data[:5+data_length])
        if crc_calc != crc_received:
            self.logger.error(f"CRC mismatch: calculated=0x{crc_calc:04X}, received=0x{crc_received:04X}")
            return None
        
        return {
            'packet_id': packet_id,
            'device_addr': device_addr,
            'command': command,
            'payload': payload
        }
    
    def send_command(self, command, payload=b'', expect_response=True):
        """
        Send command to motor and optionally wait for response
        
        Args:
            command (int): Command code
            payload (bytes): Command payload
            expect_response (bool): Whether to wait for response
            
        Returns:
            dict: Response data if expect_response=True, True if expect_response=False, None on error
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            self.logger.error("Not connected to motor")
            return None
        
        try:
            # Build and send packet
            packet = self._build_packet(command, payload)
            self.logger.debug(f"TX: {packet.hex(' ').upper()}")
            
            self.serial_conn.write(packet)
            
            if not expect_response:
                return True
            
            # Wait for response
            time.sleep(0.01)  # Small delay for response
            
            # Read response with timeout
            response_data = self.serial_conn.read(1024)  # Read up to 1KB
            
            if not response_data:
                self.logger.error("No response received")
                return None
            
            self.logger.debug(f"RX: {response_data.hex(' ').upper()}")
            
            # Parse and return response
            return self._parse_response(response_data)
            
        except Exception as e:
            self.logger.error(f"Error sending command 0x{command:02X}: {e}")
            return None
    
    def restart_slave(self):
        """
        Restart slave device (Command 0x00)
        
        This command restarts/resets the slave device. 
        Command 0x00 typically does not return a response.
        No payload is required for this command.
        
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        self.logger.info("Restarting slave device")
        
        # Command 0x00 doesn't expect a response
        result = self.send_command(0x00, b'', expect_response=False)
        
        if result:
            self.logger.info("✓ Slave restart command sent")
            return True
        else:
            self.logger.error("✗ Failed to send slave restart command")
            return False
    
    def read_system_info(self):
        """
        Read system information (Command 0x0A)
        
        This command reads Boot, software, hardware, RS485 protocol and CAN protocol 
        version information from the motor.
        No payload is required for this command.
        
        Returns:
            dict: Response data containing system info, or None on error
        """
        self.logger.info("Reading system information")
        
        response = self.send_command(0x0A, b'', expect_response=True)
        
        if response:
            self.logger.info("✓ System information read successfully")
            return response
        else:
            self.logger.error("✗ Failed to read system information")
            return None
    
    def _parse_realtime_data(self, payload):
        """
        Parse real-time data payload according to protocol specification
        
        Args:
            payload (bytes): Raw payload data from command 0x0B response
            
        Returns:
            dict: Parsed real-time data or None if invalid
        """
        if len(payload) < 22:  # Minimum expected size
            self.logger.error(f"Real-time payload too short: {len(payload)} bytes (expected >= 22)")
            return None
        
        try:
            # Parse according to protocol specification
            # All multi-byte values are in little-endian format
            offset = 0
            
            # Absolute angle value (2 bytes) - unit: 360°/16384
            abs_angle_raw = struct.unpack('<H', payload[offset:offset+2])[0]
            abs_angle_deg = abs_angle_raw * (360.0 / 16384.0)
            offset += 2
            
            # Multi-turn absolute angle value (4 bytes) - unit: 360°/16384  
            multi_angle_raw = struct.unpack('<I', payload[offset:offset+4])[0]
            multi_angle_deg = multi_angle_raw * (360.0 / 16384.0)
            offset += 4
            
            # Motor speed (4 bytes) - unit: 0.01 RPM
            speed_raw = struct.unpack('<i', payload[offset:offset+4])[0]  # signed
            speed_rpm = speed_raw * 0.01
            offset += 4
            
            # Q-axis current (4 bytes) - unit: 0.001A
            current_raw = struct.unpack('<i', payload[offset:offset+4])[0]  # signed
            current_a = current_raw * 0.001
            offset += 4
            
            # Bus voltage (2 bytes) - unit: 0.01V
            voltage_raw = struct.unpack('<H', payload[offset:offset+2])[0]
            voltage_v = voltage_raw * 0.01
            offset += 2
            
            # Bus current (2 bytes) - unit: 0.01A
            bus_current_raw = struct.unpack('<H', payload[offset:offset+2])[0]
            bus_current_a = bus_current_raw * 0.01
            offset += 2
            
            # Working temperature (1 byte) - unit: °C
            work_temp_c = payload[offset]
            offset += 1
            
            # Operating status (1 byte)
            # Bit meanings from protocol:
            # 0: Shutdown state   1: Voltage control
            # 2: Current control  3: Speed control  
            # 4: Position control
            operating_status = payload[offset]
            offset += 1
            
            # Motor status (1 byte)
            # 0: Off  Other: On
            # Bit[0]: Voltage control  Bit[1]: Voltage control
            # Bit[2]: Speed control    Bit[3]: Encoder failure  
            # Bit[6]: Hardware failure Bit[7]: Software failure
            motor_status = payload[offset]
            offset += 1
            
            # Fault code (1 byte)
            # Bxxx represents fault digit position, Bit0 represents fault existence
            fault_code = payload[offset]
            
            return {
                'abs_angle_raw': abs_angle_raw,
                'abs_angle_deg': abs_angle_deg,
                'multi_angle_raw': multi_angle_raw, 
                'multi_angle_deg': multi_angle_deg,
                'speed_raw': speed_raw,
                'speed_rpm': speed_rpm,
                'current_raw': current_raw,
                'current_a': current_a,
                'voltage_raw': voltage_raw,
                'voltage_v': voltage_v,
                'bus_current_raw': bus_current_raw,
                'bus_current_a': bus_current_a,
                'work_temp_c': work_temp_c,
                'operating_status': operating_status,
                'motor_status': motor_status,
                'fault_code': fault_code
            }
            
        except Exception as e:
            self.logger.error(f"Error parsing real-time data: {e}")
            return None

    def read_realtime_info(self):
        """
        Read real-time information (Command 0x0B)
        
        This command reads real-time motor status information including:
        - Absolute angle value (2 bytes)
        - Multi-turn absolute angle value (4 bytes) 
        - Motor speed (4 bytes)
        - Q-axis current (4 bytes)
        - Bus voltage (2 bytes)
        - Motor temperature (2 bytes)
        - Working temperature (1 byte)
        - Operating status (1 byte)
        - Motor status (1 byte)
        - Fault code (1 byte)
        No payload is required for this command.
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        self.logger.info("Reading real-time information")
        
        response = self.send_command(0x0B, b'', expect_response=True)
        
        if response:
            self.logger.info("✓ Real-time information read successfully")
            
            # Parse the payload data
            parsed_data = self._parse_realtime_data(response['payload'])
            if parsed_data:
                # Return both raw response and parsed data
                response['parsed_data'] = parsed_data
                return response
            else:
                self.logger.error("✗ Failed to parse real-time data payload")
                return response  # Return raw response even if parsing fails
        else:
            self.logger.error("✗ Failed to read real-time information")
            return None

    def clear_faults(self):
        """
        Clear faults (Command 0x0F)
        
        This command clears any existing faults in the motor system.
        No payload is required for this command.
        
        Returns:
            dict: Response data from motor, or None on error
        """
        self.logger.info("Clearing motor faults")
        
        response = self.send_command(0x0F, b'', expect_response=True)
        
        if response:
            self.logger.info("✓ Clear faults command executed successfully")
            return response
        else:
            self.logger.error("✗ Failed to clear faults")
            return None

    def _parse_user_parameters(self, payload):
        """
        Parse user parameters payload according to protocol specification
        
        Args:
            payload (bytes): Raw payload data from command 0x10 response
            
        Returns:
            dict: Parsed user parameters or None if invalid
        """
        if len(payload) < 26:  # Expected size
            self.logger.error(f"User parameters payload too short: {len(payload)} bytes (expected 26)")
            return None
        
        try:
            # Parse according to protocol specification (all multi-byte values are little-endian)
            offset = 0
            
            # Electrical angle offset (2 bytes) - unit: angle value/360/16384
            elec_angle_offset_raw = struct.unpack('<H', payload[offset:offset+2])[0]
            elec_angle_offset_deg = elec_angle_offset_raw * (360.0 / 16384.0)
            offset += 2
            
            # Mechanical angle offset (2 bytes) - unit: angle value/360/16384  
            mech_angle_offset_raw = struct.unpack('<H', payload[offset:offset+2])[0]
            mech_angle_offset_deg = mech_angle_offset_raw * (360.0 / 16384.0)
            offset += 2
            
            # U phase current offset (2 bytes)
            u_current_offset = struct.unpack('<H', payload[offset:offset+2])[0]
            offset += 2
            
            # V phase current offset (2 bytes)
            v_current_offset = struct.unpack('<H', payload[offset:offset+2])[0]
            offset += 2
            
            # W phase current offset (2 bytes)
            w_current_offset = struct.unpack('<H', payload[offset:offset+2])[0]
            offset += 2
            
            # Encoder model (1 byte)
            encoder_model_code = payload[offset]
            encoder_models = {
                0: "AS504x_SPI",
                1: "Ma7xx_SPI", 
                2: "MT6835_SPI",
                3: "TL5012_SSI",
                4: "AS504x_PWM",
                5: "Tamgawa"
            }
            encoder_model = encoder_models.get(encoder_model_code, f"Unknown({encoder_model_code})")
            offset += 1
            
            # Change encoder direction (1 byte)
            change_encoder_dir = payload[offset]
            offset += 1
            
            # Save second encoder (1 byte)
            save_second_encoder = payload[offset]
            offset += 1
            
            # Speed filter coefficient (1 byte) - range 1-100, represents 0.01-1
            speed_filter_coeff = payload[offset]
            speed_filter_value = speed_filter_coeff * 0.01
            offset += 1
            
            # Device address (1 byte)
            device_address = payload[offset]
            offset += 1
            
            # RS485 baud rate (1 byte)
            rs485_baud_code = payload[offset]
            rs485_bauds = {
                0: 921600,
                1: 460800, 
                2: 115200,
                3: 57600,
                4: 38400,
                5: 19200,
                6: 9600
            }
            rs485_baud = rs485_bauds.get(rs485_baud_code, f"Unknown({rs485_baud_code})")
            offset += 1
            
            # CAN baud rate (1 byte)
            can_baud_code = payload[offset]
            can_bauds = {
                0: "1M",
                1: "500K",
                2: "250K", 
                3: "125K",
                4: "100K"
            }
            can_baud = can_bauds.get(can_baud_code, f"Unknown({can_baud_code})")
            offset += 1
            
            # Save CanOpen (1 byte)
            save_canopen = payload[offset]
            canopen_protocol = "CanOpen" if save_canopen != 0 else "CAN Custom"
            offset += 1
            
            # Maximum bus voltage (2 bytes) - unit: 0.01V
            max_bus_voltage_raw = struct.unpack('<H', payload[offset:offset+2])[0]
            max_bus_voltage = max_bus_voltage_raw * 0.01
            offset += 2
            
            # Fault duration after power on (1 byte) - unit: seconds
            fault_duration_power_on = payload[offset]
            offset += 1
            
            # Maximum bus current (2 bytes) - unit: 0.01A
            max_bus_current_raw = struct.unpack('<H', payload[offset:offset+2])[0]
            max_bus_current = max_bus_current_raw * 0.01
            offset += 2
            
            # Another fault duration field (1 byte) - unit: seconds
            fault_duration_2 = payload[offset]
            offset += 1
            
            # Maximum temperature (1 byte) - unit: °C
            max_temperature = payload[offset]
            offset += 1
            
            # Fault continuous duration (1 byte) - unit: seconds
            fault_continuous_duration = payload[offset]
            
            return {
                'elec_angle_offset_raw': elec_angle_offset_raw,
                'elec_angle_offset_deg': elec_angle_offset_deg,
                'mech_angle_offset_raw': mech_angle_offset_raw,
                'mech_angle_offset_deg': mech_angle_offset_deg,
                'u_current_offset': u_current_offset,
                'v_current_offset': v_current_offset,
                'w_current_offset': w_current_offset,
                'encoder_model_code': encoder_model_code,
                'encoder_model': encoder_model,
                'change_encoder_dir': change_encoder_dir,
                'save_second_encoder': save_second_encoder,
                'speed_filter_coeff': speed_filter_coeff,
                'speed_filter_value': speed_filter_value,
                'device_address': device_address,
                'rs485_baud_code': rs485_baud_code,
                'rs485_baud': rs485_baud,
                'can_baud_code': can_baud_code,
                'can_baud': can_baud,
                'save_canopen': save_canopen,
                'canopen_protocol': canopen_protocol,
                'max_bus_voltage_raw': max_bus_voltage_raw,
                'max_bus_voltage': max_bus_voltage,
                'fault_duration_power_on': fault_duration_power_on,
                'max_bus_current_raw': max_bus_current_raw,
                'max_bus_current': max_bus_current,
                'fault_duration_2': fault_duration_2,
                'max_temperature': max_temperature,
                'fault_continuous_duration': fault_continuous_duration
            }
            
        except Exception as e:
            self.logger.error(f"Error parsing user parameters: {e}")
            return None

    def read_user_parameters(self):
        """
        Read user parameters (Command 0x10)
        
        This command reads user-configurable parameters from the motor.
        These parameters typically include user-defined settings and configurations.
        No payload is required for this command.
        
        Returns:
            dict: Response data containing user parameters, or None on error
        """
        self.logger.info("Reading user parameters")
        
        response = self.send_command(0x10, b'', expect_response=True)
        
        if response:
            self.logger.info("✓ User parameters read successfully")
            
            # Parse the payload data
            parsed_data = self._parse_user_parameters(response['payload'])
            if parsed_data:
                # Return both raw response and parsed data
                response['parsed_data'] = parsed_data
                return response
            else:
                self.logger.error("✗ Failed to parse user parameters payload")
                return response  # Return raw response even if parsing fails
        else:
            self.logger.error("✗ Failed to read user parameters")
            return None

    def write_user_parameters(self, parameters_dict=None, raw_payload=None):
        """
        Write user parameters (Command 0x11)
        
        This command writes user-configurable parameters to the motor.
        You can provide either a parameters dictionary or raw payload bytes.
        
        Args:
            parameters_dict (dict): Dictionary with parameter values to write
            raw_payload (bytes): Raw 26-byte payload to write directly
            
        Returns:
            dict: Response data from motor, or None on error
        """
        if parameters_dict is None and raw_payload is None:
            self.logger.error("Either parameters_dict or raw_payload must be provided")
            return None
            
        if raw_payload is not None:
            if len(raw_payload) != 16:
                self.logger.error(f"Raw payload must be exactly 16 bytes, got {len(raw_payload)}")
                return None
            payload = raw_payload
        else:
            # Build payload from parameters dictionary
            payload = self._build_user_parameters_payload(parameters_dict)
            if payload is None:
                return None
        
        self.logger.info("Writing user parameters")
        
        response = self.send_command(0x11, payload, expect_response=True)
        
        if response:
            self.logger.info("✓ User parameters written successfully")
            # Parse the response like read_user_parameters does
            parsed_data = self._parse_user_parameters(response['payload'])
            response['parsed_data'] = parsed_data
            return response
        else:
            self.logger.error("✗ Failed to write user parameters")
            return None

    def _build_user_parameters_payload(self, params):
        """
        Build user parameters payload from dictionary
        
        Args:
            params (dict): Dictionary with parameter values
            
        Returns:
            bytes: 16-byte payload or None if error
        """
        try:
            # Build payload with current values first
            current_response = self.read_user_parameters()
            if current_response and 'parsed_data' in current_response:
                current_params = current_response['parsed_data']
                
                # Write command uses 16 bytes, not 26 like read command
                payload = bytearray(16)
                
                # Set current values for 16-byte write format
                # Write payload starts from encoder_model_code (byte 10 in read) to fault_continuous_duration (byte 25 in read)
                payload[0] = current_params['encoder_model_code']                    # Byte 0 in write = Byte 10 in read
                payload[1] = current_params['change_encoder_dir']                    # Byte 1 in write = Byte 11 in read
                payload[2] = current_params['save_second_encoder']                   # Byte 2 in write = Byte 12 in read
                payload[3] = current_params['speed_filter_coeff']                    # Byte 3 in write = Byte 13 in read
                payload[4] = current_params['device_address']                        # Byte 4 in write = Byte 14 in read
                payload[5] = current_params['rs485_baud_code']                       # Byte 5 in write = Byte 15 in read
                payload[6] = current_params['can_baud_code']                         # Byte 6 in write = Byte 16 in read
                payload[7] = current_params['save_canopen']                          # Byte 7 in write = Byte 17 in read
                payload[8:10] = struct.pack('<H', current_params['max_bus_voltage_raw'])  # Bytes 8-9 in write = Bytes 18-19 in read
                payload[10] = current_params['fault_duration_power_on']              # Byte 10 in write = Byte 20 in read
                payload[11:13] = struct.pack('<H', current_params['max_bus_current_raw'])  # Bytes 11-12 in write = Bytes 21-22 in read
                payload[13] = current_params['fault_duration_2']                     # Byte 13 in write = Byte 23 in read
                payload[14] = current_params['max_temperature']                      # Byte 14 in write = Byte 24 in read
                payload[15] = current_params['fault_continuous_duration']            # Byte 15 in write = Byte 25 in read
                
            else:
                # Fallback to zeros if can't read current parameters
                self.logger.warning("Could not read current parameters, starting with zeros")
                payload = bytearray(16)
            
            # Update parameters in the corrected write payload format
            # Note: Write payload corresponds to bytes 10-25 of the read payload
            
            # Encoder model (1 byte) - Byte 0 in write
            if 'encoder_model_code' in params:
                payload[0] = params['encoder_model_code'] & 0xFF
            
            # Change encoder direction (1 byte) - Byte 1 in write
            if 'change_encoder_dir' in params:
                payload[1] = 1 if params['change_encoder_dir'] else 0
            
            # Save second encoder (1 byte) - Byte 2 in write
            if 'save_second_encoder' in params:
                payload[2] = 1 if params['save_second_encoder'] else 0
            
            # Speed filter coefficient (1 byte) - Byte 3 in write
            if 'speed_filter_value' in params:
                coeff = int(params['speed_filter_value'] * 100)
                payload[3] = max(1, min(100, coeff))
            elif 'speed_filter_coeff' in params:
                payload[3] = max(1, min(100, params['speed_filter_coeff']))
            
            # Device address (1 byte) - Byte 4 in write
            if 'device_address' in params:
                payload[4] = params['device_address'] & 0xFF
            
            # RS485 baud rate (1 byte) - Byte 5 in write
            if 'rs485_baud_code' in params:
                payload[5] = params['rs485_baud_code'] & 0xFF
            elif 'rs485_baud' in params:
                baud_map = {921600: 0, 460800: 1, 115200: 2, 57600: 3, 38400: 4, 19200: 5, 9600: 6}
                payload[5] = baud_map.get(params['rs485_baud'], 2)  # Default to 115200
            
            # CAN baud rate (1 byte) - Byte 6 in write
            if 'can_baud_code' in params:
                payload[6] = params['can_baud_code'] & 0xFF
            elif 'can_baud' in params:
                baud_map = {"1M": 0, "500K": 1, "250K": 2, "125K": 3, "100K": 4}
                payload[6] = baud_map.get(params['can_baud'], 0)  # Default to 1M
            
            # Save CanOpen (1 byte) - Byte 7 in write
            if 'save_canopen' in params:
                payload[7] = 1 if params['save_canopen'] else 0
            elif 'canopen_protocol' in params:
                payload[7] = 1 if params['canopen_protocol'] == "CanOpen" else 0
            
            # Maximum bus voltage (2 bytes) - Bytes 8-9 in write
            if 'max_bus_voltage' in params:
                raw_value = int(params['max_bus_voltage'] * 100)
                payload[8:10] = struct.pack('<H', raw_value & 0xFFFF)
            elif 'max_bus_voltage_raw' in params:
                payload[8:10] = struct.pack('<H', params['max_bus_voltage_raw'] & 0xFFFF)
            
            # Fault duration after power on (1 byte) - Byte 10 in write
            if 'fault_duration_power_on' in params:
                payload[10] = params['fault_duration_power_on'] & 0xFF
            
            # Maximum bus current (2 bytes) - Bytes 11-12 in write
            if 'max_bus_current' in params:
                raw_value = int(params['max_bus_current'] * 100)
                payload[11:13] = struct.pack('<H', raw_value & 0xFFFF)
            elif 'max_bus_current_raw' in params:
                payload[11:13] = struct.pack('<H', params['max_bus_current_raw'] & 0xFFFF)
            
            # Fault duration 2 (1 byte) - Byte 13 in write
            if 'fault_duration_2' in params:
                payload[13] = params['fault_duration_2'] & 0xFF
            
            # Maximum temperature (1 byte) - Byte 14 in write
            if 'max_temperature' in params:
                payload[14] = params['max_temperature'] & 0xFF
            
            # Fault continuous duration (1 byte) - Byte 15 in write
            if 'fault_continuous_duration' in params:
                payload[15] = params['fault_continuous_duration'] & 0xFF
            
            return bytes(payload)
            
        except Exception as e:
            self.logger.error(f"Error building user parameters payload: {e}")
            return None

    def change_encoder_model(self, encoder_model):
        """
        Change encoder model setting
        
        Args:
            encoder_model (int or str): Encoder model code (0-5) or model name
                0: AS504x_SPI, 1: Ma7xx_SPI, 2: MT6835_SPI,
                3: TL5012_SSI, 4: AS504x_PWM, 5: Tamgawa
                
        Returns:
            dict: Response data from motor, or None on error
        """
        if isinstance(encoder_model, str):
            model_map = {
                'AS504x_SPI': 0, 'Ma7xx_SPI': 1, 'MT6835_SPI': 2,
                'TL5012_SSI': 3, 'AS504x_PWM': 4, 'Tamgawa': 5
            }
            encoder_code = model_map.get(encoder_model)
            if encoder_code is None:
                self.logger.error(f"Invalid encoder model name: {encoder_model}")
                return None
        else:
            encoder_code = int(encoder_model)
            if not 0 <= encoder_code <= 5:
                self.logger.error(f"Encoder model code must be 0-5, got {encoder_code}")
                return None
        
        self.logger.info(f"Changing encoder model to: {encoder_code}")
        return self.write_user_parameters({'encoder_model_code': encoder_code})

    def change_encoder_direction(self, reverse_direction=True):
        """
        Change encoder direction setting
        
        Args:
            reverse_direction (bool): True to reverse encoder direction, False for normal
                
        Returns:
            dict: Response data from motor, or None on error
        """
        direction = 1 if reverse_direction else 0
        self.logger.info(f"Setting encoder direction: {'Reversed' if reverse_direction else 'Normal'}")
        return self.write_user_parameters({'change_encoder_dir': direction})

    def change_second_encoder_save(self, enable_save=True):
        """
        Change second encoder save setting
        
        Args:
            enable_save (bool): True to enable saving second encoder, False to disable
                
        Returns:
            dict: Response data from motor, or None on error
        """
        save_setting = 1 if enable_save else 0
        self.logger.info(f"Setting second encoder save: {'Enabled' if enable_save else 'Disabled'}")
        return self.write_user_parameters({'save_second_encoder': save_setting})

    def change_speed_filter(self, filter_value):
        """
        Change speed filter coefficient
        
        Args:
            filter_value (float or int): Speed filter value (0.01-1.00 as float) or 
                                       coefficient (1-100 as int)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        if isinstance(filter_value, float):
            if not 0.01 <= filter_value <= 1.0:
                self.logger.error(f"Speed filter value must be 0.01-1.0, got {filter_value}")
                return None
            coeff = int(filter_value * 100)
        else:
            coeff = int(filter_value)
            if not 1 <= coeff <= 100:
                self.logger.error(f"Speed filter coefficient must be 1-100, got {coeff}")
                return None
        
        self.logger.info(f"Setting speed filter coefficient to: {coeff} ({coeff * 0.01:.2f})")
        return self.write_user_parameters({'speed_filter_coeff': coeff})

    def change_device_address(self, new_address):
        """
        Change device address (WARNING: This will change communication address!)
        
        Args:
            new_address (int): New device address (0x01-0xFE)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        addr = int(new_address)
        if not 0x01 <= addr <= 0xFE:
            self.logger.error(f"Device address must be 0x01-0xFE, got 0x{addr:02X}")
            return None
        
        self.logger.warning(f"⚠️  CHANGING DEVICE ADDRESS from 0x{self.device_addr:02X} to 0x{addr:02X}")
        self.logger.warning("⚠️  You will need to reconnect with the new address!")
        
        response = self.write_user_parameters({'device_address': addr})
        
        # if response:
        #     self.logger.info(f"✓ Device address changed. Update connection to use address 0x{addr:02X}")
        #     # Update internal address for future communications
        #     self.device_addr = addr
        
        return response

    def change_rs485_baudrate(self, baudrate):
        """
        Change RS485 baud rate
        
        Args:
            baudrate (int): Baud rate (921600, 460800, 115200, 57600, 38400, 19200, 9600)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        baud_map = {921600: 0, 460800: 1, 115200: 2, 57600: 3, 38400: 4, 19200: 5, 9600: 6}
        baud_code = baud_map.get(baudrate)
        
        if baud_code is None:
            valid_rates = list(baud_map.keys())
            self.logger.error(f"Invalid baud rate. Valid rates: {valid_rates}")
            return None
        
        self.logger.warning(f"⚠️  CHANGING RS485 BAUD RATE from {self.baudrate} to {baudrate}")
        self.logger.warning("⚠️  You will need to reconnect with the new baud rate!")
        
        response = self.write_user_parameters({'rs485_baud_code': baud_code})
        
        if response:
            self.logger.info(f"✓ RS485 baud rate changed. Update connection to use {baudrate} baud")
            # Update internal baud rate for future communications
            self.baudrate = baudrate
        
        return response

    def change_can_baudrate(self, can_baudrate):
        """
        Change CAN baud rate
        
        Args:
            can_baudrate (str): CAN baud rate ("1M", "500K", "250K", "125K", "100K")
                
        Returns:
            dict: Response data from motor, or None on error
        """
        baud_map = {"1M": 0, "500K": 1, "250K": 2, "125K": 3, "100K": 4}
        baud_code = baud_map.get(can_baudrate)
        
        if baud_code is None:
            valid_rates = list(baud_map.keys())
            self.logger.error(f"Invalid CAN baud rate. Valid rates: {valid_rates}")
            return None
        
        self.logger.info(f"Setting CAN baud rate to: {can_baudrate}")
        return self.write_user_parameters({'can_baud_code': baud_code})

    def change_canopen_protocol(self, use_canopen=True):
        """
        Change CAN protocol setting (CanOpen vs Custom)
        
        Args:
            use_canopen (bool): True for CanOpen protocol, False for Custom protocol
                
        Returns:
            dict: Response data from motor, or None on error
        """
        protocol_setting = 1 if use_canopen else 0
        protocol_name = "CanOpen" if use_canopen else "Custom"
        self.logger.info(f"Setting CAN protocol to: {protocol_name}")
        return self.write_user_parameters({'save_canopen': protocol_setting})

    def change_max_bus_voltage(self, max_voltage):
        """
        Change maximum bus voltage limit
        
        Args:
            max_voltage (float): Maximum voltage in volts (0.01V resolution)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        voltage = float(max_voltage)
        if voltage < 0 or voltage > 655.35:  # 16-bit value * 0.01
            self.logger.error(f"Max voltage must be 0-655.35V, got {voltage}V")
            return None
        
        raw_value = int(voltage * 100)
        self.logger.info(f"Setting maximum bus voltage to: {voltage:.2f}V")
        return self.write_user_parameters({'max_bus_voltage_raw': raw_value})

    def change_fault_duration_power_on(self, duration_seconds):
        """
        Change fault duration after power on
        
        Args:
            duration_seconds (int): Duration in seconds (0-255)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        duration = int(duration_seconds)
        if not 0 <= duration <= 255:
            self.logger.error(f"Fault duration must be 0-255 seconds, got {duration}")
            return None
        
        self.logger.info(f"Setting power-on fault duration to: {duration} seconds")
        return self.write_user_parameters({'fault_duration_power_on': duration})

    def change_max_bus_current(self, max_current):
        """
        Change maximum bus current limit
        
        Args:
            max_current (float): Maximum current in amperes (0.01A resolution)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        current = float(max_current)
        if current < 0 or current > 655.35:  # 16-bit value * 0.01
            self.logger.error(f"Max current must be 0-655.35A, got {current}A")
            return None
        
        raw_value = int(current * 100)
        self.logger.info(f"Setting maximum bus current to: {current:.2f}A")
        return self.write_user_parameters({'max_bus_current_raw': raw_value})

    def change_fault_duration_2(self, duration):
        """
        Change fault duration 2 setting
        
        Args:
            duration (int): Duration value (0-255)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        dur = int(duration)
        if not 0 <= dur <= 255:
            self.logger.error(f"Fault duration 2 must be 0-255, got {dur}")
            return None
        
        self.logger.info(f"Setting fault duration 2 to: {dur}")
        return self.write_user_parameters({'fault_duration_2': dur})

    def change_max_temperature(self, max_temp_celsius):
        """
        Change maximum operating temperature
        
        Args:
            max_temp_celsius (int): Maximum temperature in Celsius (0-255)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        temp = int(max_temp_celsius)
        if not 0 <= temp <= 255:
            self.logger.error(f"Max temperature must be 0-255°C, got {temp}°C")
            return None
        
        self.logger.info(f"Setting maximum temperature to: {temp}°C")
        return self.write_user_parameters({'max_temperature': temp})

    def change_fault_continuous_duration(self, duration):
        """
        Change fault continuous duration setting
        
        Args:
            duration (int): Duration value (0-255)
                
        Returns:
            dict: Response data from motor, or None on error
        """
        dur = int(duration)
        if not 0 <= dur <= 255:
            self.logger.error(f"Fault continuous duration must be 0-255, got {dur}")
            return None
        
        self.logger.info(f"Setting fault continuous duration to: {dur}")
        return self.write_user_parameters({'fault_continuous_duration': dur})

    def _parse_control_parameters(self, payload):
        """
        Parse control parameters payload from command 0x14 response
        Args:
            payload (bytes): Raw payload data from command 0x14 response
        Returns:
            dict: Parsed control parameters or None if invalid
        """
        if len(payload) < 0x18:  # 24 bytes expected
            self.logger.error(f"Control parameters payload too short: {len(payload)} bytes (expected 24)")
            return None
        try:
            # All values are little-endian
            offset = 0
            pos_kp = struct.unpack('<f', payload[offset:offset+4])[0]
            offset += 4
            pos_ki = struct.unpack('<f', payload[offset:offset+4])[0]
            offset += 4
            pos_output_limit = struct.unpack('<I', payload[offset:offset+4])[0]  # 0.01rpm
            offset += 4
            spd_kp = struct.unpack('<f', payload[offset:offset+4])[0]
            offset += 4
            spd_ki = struct.unpack('<f', payload[offset:offset+4])[0]
            offset += 4
            spd_output_limit = struct.unpack('<I', payload[offset:offset+4])[0]  # 0.001A
            # No offset increment needed; done
            return {
                'position_kp': pos_kp,
                'position_ki': pos_ki,
                'position_output_limit_raw': pos_output_limit,
                'position_output_limit_rpm': pos_output_limit * 0.01,
                'speed_kp': spd_kp,
                'speed_ki': spd_ki,
                'speed_output_limit_raw': spd_output_limit,
                'speed_output_limit_a': spd_output_limit * 0.001
            }
        except Exception as e:
            self.logger.error(f"Error parsing control parameters: {e}")
            return None

    def read_control_parameters(self):
        """
        Read control parameters (Command 0x14)
        This command reads the PID and output limit parameters for position and speed loops.
        No payload is required for this command.
        Returns:
            dict: Response data containing control parameters, or None on error
        """
        self.logger.info("Reading control parameters (PID and output limits)")
        response = self.send_command(0x14, b'', expect_response=True)
        if response:
            self.logger.info("✓ Control parameters read successfully")
            parsed_data = self._parse_control_parameters(response['payload'])
            if parsed_data:
                response['parsed_data'] = parsed_data
                return response
            else:
                self.logger.error("✗ Failed to parse control parameters payload")
                return response
        else:
            self.logger.error("✗ Failed to read control parameters")
            return None

    def _build_control_parameters_payload(self, params):
        """
        Build control parameters payload from dictionary for command 0x15
        Args:
            params (dict): Dictionary with control parameter values
        Returns:
            bytes: 24-byte payload or None if error
        """
        try:
            payload = bytearray(24)
            # Position loop Kp (float32)
            payload[0:4] = struct.pack('<f', float(params.get('position_kp', 0.0)))
            # Position loop Ki (float32)
            payload[4:8] = struct.pack('<f', float(params.get('position_ki', 0.0)))
            # Position output limit (uint32, 0.01rpm)
            payload[8:12] = struct.pack('<I', int(params.get('position_output_limit_raw', 0)))
            # Speed loop Kp (float32)
            payload[12:16] = struct.pack('<f', float(params.get('speed_kp', 0.0)))
            # Speed loop Ki (float32)
            payload[16:20] = struct.pack('<f', float(params.get('speed_ki', 0.0)))
            # Speed output limit (uint32, 0.001A)
            payload[20:24] = struct.pack('<I', int(params.get('speed_output_limit_raw', 0)))
            return bytes(payload)
        except Exception as e:
            self.logger.error(f"Error building control parameters payload: {e}")
            return None

    def write_control_parameters(self, parameters_dict=None, raw_payload=None):
        """
        Write control parameters (Command 0x15)
        This command writes the PID and output limit parameters for position and speed loops.
        Args:
            parameters_dict (dict): Dictionary with control parameter values
            raw_payload (bytes): Raw 24-byte payload to write directly
        Returns:
            dict: Response data from motor, or None on error
        """
        if parameters_dict is None and raw_payload is None:
            self.logger.error("Either parameters_dict or raw_payload must be provided")
            return None
        if raw_payload is not None:
            if len(raw_payload) != 24:
                self.logger.error(f"Raw payload must be exactly 24 bytes, got {len(raw_payload)}")
                return None
            payload = raw_payload
        else:
            payload = self._build_control_parameters_payload(parameters_dict)
            if payload is None:
                return None
        self.logger.info("Writing control parameters (PID and output limits)")
        response = self.send_command(0x15, payload, expect_response=True)
        if response:
            self.logger.info("✓ Control parameters written successfully")
            parsed_data = self._parse_control_parameters(response['payload'])
            response['parsed_data'] = parsed_data
            return response
        else:
            self.logger.error("✗ Failed to write control parameters")
            return None

    def write_control_parameters_and_save(self, parameters_dict=None, raw_payload=None):
        """
        Write and save control parameters (Command 0x16)
        
        This command writes the PID and output limit parameters for position and speed loops
        and saves them to non-volatile memory (unlike 0x15 which is temporary).
        
        Args:
            parameters_dict (dict): Dictionary with control parameter values
            raw_payload (bytes): Raw 24-byte payload to write directly
            
        Returns:
            dict: Response data from motor, or None on error
        """
        if parameters_dict is None and raw_payload is None:
            self.logger.error("Either parameters_dict or raw_payload must be provided")
            return None
            
        if raw_payload is not None:
            if len(raw_payload) != 24:
                self.logger.error(f"Raw payload must be exactly 24 bytes, got {len(raw_payload)}")
                return None
            payload = raw_payload
        else:
            payload = self._build_control_parameters_payload(parameters_dict)
            if payload is None:
                return None
        
        self.logger.info("Writing and saving control parameters (PID and output limits)")
        
        response = self.send_command(0x16, payload, expect_response=True)
        
        if response:
            self.logger.info("✓ Control parameters written and saved successfully")
            parsed_data = self._parse_control_parameters(response['payload'])
            response['parsed_data'] = parsed_data
            return response
        else:
            self.logger.error("✗ Failed to write and save control parameters")
            return None


def setup_logging(level=logging.INFO):
    """Setup logging configuration"""
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    ) 
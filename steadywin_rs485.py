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
            
            # Read response with timeout (no delay needed - timeout handles timing)
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

    def _send_control_command(self, command, payload, operation_description):
        """
        Helper method for control commands that return realtime data
        
        Args:
            command (int): Command code
            payload (bytes): Command payload
            operation_description (str): Description for logging
            
        Returns:
            dict: Response with parsed_data, or None on error
        """
        response = self.send_command(command, payload, expect_response=True)
        
        if response:
            # Parse the response using realtime data parser
            parsed_data = self._parse_realtime_data(response['payload'])
            if parsed_data:
                response['parsed_data'] = parsed_data
            return response
        else:
            self.logger.error(f"✗ Failed to execute {operation_description}")
            return None

    def _send_simple_command(self, command, operation_description, expect_response=True):
        """
        Helper method for simple commands with no payload
        
        Args:
            command (int): Command code
            operation_description (str): Description for logging
            expect_response (bool): Whether to expect a response
            
        Returns:
            dict/bool: Response data or success boolean, None on error
        """
        response = self.send_command(command, b'', expect_response=expect_response)
        
        if response is None:
            self.logger.error(f"✗ Failed to execute {operation_description}")
            return None
        elif expect_response and isinstance(response, dict):
            # Try to parse realtime data if payload is long enough
            if len(response['payload']) >= 22:
                parsed_data = self._parse_realtime_data(response['payload'])
                if parsed_data:
                    response['parsed_data'] = parsed_data
            return response
        else:
            return response

    def _send_parameter_read_command(self, command, operation_description, parser_func):
        """
        Helper method for parameter reading commands that need custom parsing
        
        Args:
            command (int): Command code
            operation_description (str): Description for logging
            parser_func: Function to parse the response payload
            
        Returns:
            dict: Response with parsed_data, or None on error
        """
        response = self.send_command(command, b'', expect_response=True)
        
        if response:
            # Parse the payload data using the provided parser
            parsed_data = parser_func(response['payload'])
            if parsed_data:
                response['parsed_data'] = parsed_data
            return response
        else:
            self.logger.error(f"✗ Failed to {operation_description}")
            return None

    def _send_parameter_write_command(self, command, payload, operation_description, parser_func):
        """
        Helper method for parameter writing commands that need custom parsing
        
        Args:
            command (int): Command code
            payload (bytes): Command payload
            operation_description (str): Description for logging
            parser_func: Function to parse the response payload
            
        Returns:
            dict: Response with parsed_data, or None on error
        """
        response = self.send_command(command, payload, expect_response=True)
        
        if response:
            # Parse the response using the provided parser
            parsed_data = parser_func(response['payload'])
            if parsed_data:
                response['parsed_data'] = parsed_data
            return response
        else:
            self.logger.error(f"✗ Failed to {operation_description}")
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
        result = self._send_simple_command(0x00, "slave restart", expect_response=False)
        return result is not None
    
    def read_system_info(self):
        """
        Read system information (Command 0x0A)
        
        This command reads Boot, software, hardware, RS485 protocol and CAN protocol 
        version information from the motor.
        No payload is required for this command.
        
        Returns:
            dict: Response data containing system info, or None on error
        """
        return self._send_simple_command(0x0A, "read system information")
    
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
        return self._send_simple_command(0x0B, "read real-time information")

    def clear_faults(self):
        """
        Clear faults (Command 0x0F)
        
        This command clears any existing faults in the motor system.
        No payload is required for this command.
        
        Returns:
            dict: Response data from motor, or None on error
        """
        self.logger.info("Clearing motor faults")
        return self._send_simple_command(0x0F, "clear faults")

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
        return self._send_parameter_read_command(0x10, "read user parameters", self._parse_user_parameters)

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
        return self._send_parameter_write_command(0x11, payload, "write user parameters", self._parse_user_parameters)

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
        return self._send_parameter_read_command(0x14, "read control parameters", self._parse_control_parameters)

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
        return self._send_parameter_write_command(0x15, payload, "write control parameters", self._parse_control_parameters)

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
        return self._send_parameter_write_command(0x16, payload, "write and save control parameters", self._parse_control_parameters)

    def set_zero_position(self):
        """
        Set current position as zero/origin (Command 0x1D)
        
        This command sets the current motor position as the zero reference point.
        No payload is required for this command.
        
        Returns:
            dict: Response data from motor, or None on error
        """
        self.logger.info("Setting current position as zero/origin")
        return self._send_simple_command(0x1D, "set zero position")

    def calibrate_encoder(self, timeout_seconds=90, poll_interval=1.0, monitor_progress=True):
        """
        Calibrate encoder (Command 0x1E)
        
        This command performs encoder calibration. The calibration process may cause
        the motor to rotate and typically transitions the motor to voltage control mode.
        The function monitors motor status to detect when calibration is complete.
        
        ⚠ WARNING: Motor may rotate during calibration. Ensure the motor shaft can
        rotate freely and is in a safe mechanical position before performing calibration.
        
        Args:
            timeout_seconds (float): Maximum time to wait for calibration completion (default: 90)
            poll_interval (float): Time between status checks in seconds (default: 1.0)
            monitor_progress (bool): Whether to monitor and wait for calibration completion (default: True)
        
        Returns:
            dict: Response data from motor with calibration details, or None on error
        """
        import time
        
        self.logger.info("Starting encoder calibration")
        
        # Send the calibration command
        response = self.send_command(0x1E, b'', expect_response=True)
        
        if not response:
            self.logger.error("✗ Failed to send encoder calibration command")
            return None
        
        self.logger.info("✓ Encoder calibration command sent successfully")
        
        if not monitor_progress:
            # Return immediately without monitoring
            return response
        
        # Monitor calibration progress by checking motor status
        self.logger.info("📊 Monitoring calibration progress...")
        start_time = time.time()
        calibration_started = False
        
        while True:
            elapsed = time.time() - start_time
            
            # Check timeout
            if elapsed > timeout_seconds:
                self.logger.error(f"✗ Calibration timeout after {elapsed:.1f} seconds")
                response['calibration_timeout'] = True
                response['calibration_duration'] = elapsed
                return response
            
            # Read current motor status
            status_response = self.read_realtime_info()
            if not status_response or 'parsed_data' not in status_response:
                self.logger.warning("⚠ Could not read motor status during calibration")
                time.sleep(poll_interval)
                continue
            
            motor_data = status_response['parsed_data']
            motor_status = motor_data['motor_status']
            operating_status = motor_data['operating_status']
            
            # Check if calibration is in progress (motor_status = 0x01, operating_status = 1)
            if motor_status == 0x01 and operating_status == 1:
                if not calibration_started:
                    calibration_started = True
                    self.logger.info("🔧 Calibration in progress (Motor Status: 0x01, Voltage Control)")
                else:
                    self.logger.debug(f"📊 Calibration ongoing... ({elapsed:.1f}s elapsed)")
            
            # Check if calibration completed (motor status changed from 0x01 OR operating status changed from 1)
            elif calibration_started and (motor_status != 0x01 or operating_status != 1):
                calibration_duration = time.time() - start_time
                self.logger.info(f"✓ Encoder calibration completed successfully ({calibration_duration:.1f}s)")
                self.logger.info(f"📊 Final motor status: 0x{motor_status:02X}, Operating status: {operating_status}")
                
                # Add calibration details to response
                response['calibration_completed'] = True
                response['calibration_duration'] = calibration_duration
                response['final_motor_status'] = motor_status
                response['final_operating_status'] = operating_status
                return response
            
            # Handle case where calibration might be very long - provide periodic feedback
            elif calibration_started and elapsed > 30 and int(elapsed) % 10 == 0:
                self.logger.info(f"📊 Calibration still in progress... ({elapsed:.0f}s elapsed, Status: 0x{motor_status:02X})")
            
            # Also check if motor hasn't started calibration after 10 seconds
            elif not calibration_started and elapsed > 10:
                self.logger.warning(f"⚠ Calibration may not have started - motor status: 0x{motor_status:02X}, operating status: {operating_status}")
                # Don't return here, continue monitoring
            
            # Wait before next check
            time.sleep(poll_interval)
        
        # Should never reach here due to timeout check, but just in case
        return response

    def factory_reset(self):
        """
        Factory reset - Restore parameters to default values (Command 0x1F)
        
        This command restores most motor parameters to their factory default values.
        
        ⚠ WARNING: This will reset user parameters and control parameters to defaults!
        
        Note: The following parameters are NOT changed by factory reset:
        - Device address
        - Encoder calibration parameters
        - Motor hardware parameters
        
        Returns:
            dict: Response data from motor, or None on error
        """
        self.logger.info("Performing factory reset - restoring parameters to default values")
        result = self._send_simple_command(0x1F, "factory reset")
        if result:
            self.logger.warning("⚠ All parameters have been reset to factory defaults")
        return result

    def q_axis_current_control(self, target_current_ma, current_slope_ma_per_s=0):
        """
        Q-axis current control (Command 0x20)
        
        This command sets the target Q-axis current and current slope for motor control.
        The motor will attempt to reach the target current using the specified slope.
        The response contains the same real-time information as read_realtime_info().
        
        Args:
            target_current_ma (int): Target Q-axis current in milliamps (mA)
                                   Positive = one direction, negative = opposite direction
                                   Force = target current × torque constant
            current_slope_ma_per_s (int): Current slope in mA/s (default: 0 = maximum slope)
                                        Controls how fast the current changes
                                        0 means use maximum possible slope
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        # Validate inputs
        if not isinstance(target_current_ma, int):
            self.logger.error("Target current must be an integer (milliamps)")
            return None
            
        if not isinstance(current_slope_ma_per_s, int) or current_slope_ma_per_s < 0:
            self.logger.error("Current slope must be a non-negative integer")
            return None
        
        # Build 8-byte payload
        try:
            payload = bytearray(8)
            
            # Target Q-axis current (4 bytes signed) - unit: 0.001A
            # Convert mA to the protocol unit (0.001A = 1mA)
            target_current_protocol = target_current_ma
            payload[0:4] = struct.pack('<i', target_current_protocol)
            
            # Q-axis current slope (4 bytes unsigned) - unit: 0.001A/s
            # Convert mA/s to the protocol unit (0.001A/s = 1mA/s)  
            current_slope_protocol = current_slope_ma_per_s
            payload[4:8] = struct.pack('<I', current_slope_protocol)
            
        except Exception as e:
            self.logger.error(f"Error building Q-axis current control payload: {e}")
            return None
        
        self.logger.info(f"Setting Q-axis current: {target_current_ma}mA, slope: {current_slope_ma_per_s}mA/s")
        return self._send_control_command(0x20, bytes(payload), "Q-axis current control")

    def velocity_control(self, target_speed_rpm, acceleration_rpm_per_s=0):
        """
        Velocity control (Command 0x21)
        
        This command sets the target motor speed and acceleration for velocity control.
        The motor will attempt to reach the target speed using the specified acceleration.
        The response contains the same real-time information as read_realtime_info().
        
        Args:
            target_speed_rpm (float): Target motor speed in RPM
                                    Positive = forward direction, negative = reverse direction
            acceleration_rpm_per_s (float): Acceleration in RPM/s (default: 0 = maximum acceleration)
                                          Controls how fast the speed changes
                                          0 means use maximum possible acceleration
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        # Validate inputs
        if not isinstance(target_speed_rpm, (int, float)):
            self.logger.error("Target speed must be a number (RPM)")
            return None
            
        if not isinstance(acceleration_rpm_per_s, (int, float)) or acceleration_rpm_per_s < 0:
            self.logger.error("Acceleration must be a non-negative number")
            return None
        
        # Build 8-byte payload
        try:
            payload = bytearray(8)
            
            # Target speed (4 bytes signed) - unit: 0.01 RPM
            # Convert RPM to the protocol unit (0.01 RPM)
            target_speed_protocol = int(round(target_speed_rpm * 100))
            payload[0:4] = struct.pack('<i', target_speed_protocol)
            
            # Acceleration (4 bytes unsigned) - unit: 0.01 RPM/s
            # Convert RPM/s to the protocol unit (0.01 RPM/s)
            acceleration_protocol = int(round(acceleration_rpm_per_s * 100))
            payload[4:8] = struct.pack('<I', acceleration_protocol)
            
        except Exception as e:
            self.logger.error(f"Error building velocity control payload: {e}")
            return None
        
        self.logger.info(f"Setting target speed: {target_speed_rpm:.2f} RPM, acceleration: {acceleration_rpm_per_s:.2f} RPM/s")
        return self._send_control_command(0x21, bytes(payload), "velocity control")

    def absolute_position_control(self, target_position_counts):
        """
        Absolute position control (Command 0x22)
        
        This command sets the target absolute position for the motor.
        The motor will move to the specified absolute position.
        The response contains the same real-time information as read_realtime_info().
        
        Args:
            target_position_counts (int): Target absolute position in counts
                                        Unit: Count (rotation = value/16384 Count)
                                        Positive/negative values supported
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        # Validate inputs
        if not isinstance(target_position_counts, int):
            self.logger.error("Target position must be an integer (counts)")
            return None
        
        # Build 4-byte payload
        try:
            payload = bytearray(4)
            
            # Absolute position (4 bytes signed) - unit: Count
            payload[0:4] = struct.pack('<i', target_position_counts)
            
        except Exception as e:
            self.logger.error(f"Error building absolute position control payload: {e}")
            return None
        
        # Convert counts to degrees for logging
        target_degrees = target_position_counts * 360.0 / 16384
        
        self.logger.info(f"Setting absolute position: {target_position_counts} counts ({target_degrees:.2f}°)")
        return self._send_control_command(0x22, bytes(payload), "absolute position control")

    def relative_position_control(self, relative_position_counts):
        """
        Relative position control (Command 0x23)
        
        This command moves the motor by a relative amount from the current position.
        The motor will move by the specified relative distance.
        The response contains the same real-time information as read_realtime_info().
        
        Args:
            relative_position_counts (int): Relative position change in counts
                                          Unit: Count (rotation = value/16384 Count)
                                          Positive = forward, negative = backward
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        # Validate inputs
        if not isinstance(relative_position_counts, int):
            self.logger.error("Relative position must be an integer (counts)")
            return None
        
        # Build 4-byte payload
        try:
            payload = bytearray(4)
            
            # Relative position (4 bytes signed) - unit: Count
            payload[0:4] = struct.pack('<i', relative_position_counts)
            
        except Exception as e:
            self.logger.error(f"Error building relative position control payload: {e}")
            return None
        
        # Convert counts to degrees for logging
        relative_degrees = relative_position_counts * 360.0 / 16384
        
        self.logger.info(f"Moving relative position: {relative_position_counts} counts ({relative_degrees:.2f}°)")
        return self._send_control_command(0x23, bytes(payload), "relative position control")

    def return_home(self):
        """
        Return to home position (Command 0x24)
        
        This command returns the motor to its home position.
        No payload is required for this command.
        The response contains the same real-time information as read_realtime_info().
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        self.logger.info("Returning motor to home position")
        return self._send_simple_command(0x24, "return home")

    def brake_control(self, operation):
        """
        Brake control (Command 0x2E)
        
        This command controls the motor brake operation.
        The response contains the same real-time information as read_realtime_info().
        
        Args:
            operation (int or str): Brake operation
                                  0 or "off": Switch brake off
                                  1 or "on": Switch brake on  
                                  255 or "read": Read brake status
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        # Convert string operations to integers
        if isinstance(operation, str):
            operation_map = {"off": 0, "on": 1, "read": 255}
            if operation.lower() not in operation_map:
                self.logger.error("Operation must be 'off', 'on', 'read', or integer 0/1/255")
                return None
            operation_code = operation_map[operation.lower()]
        elif isinstance(operation, int):
            if operation not in [0, 1, 255]:
                self.logger.error("Operation must be 0 (off), 1 (on), or 255 (read)")
                return None
            operation_code = operation
        else:
            self.logger.error("Operation must be integer or string")
            return None
        
        # Build 1-byte payload
        try:
            payload = bytearray(1)
            payload[0] = operation_code
            
        except Exception as e:
            self.logger.error(f"Error building brake control payload: {e}")
            return None
        
        operation_names = {0: "off", 1: "on", 255: "read status"}
        self.logger.info(f"Brake control: {operation_names.get(operation_code, operation_code)}")
        
        response = self.send_command(0x2E, bytes(payload), expect_response=True)
        
        if response:
            self.logger.info("✓ Brake control command executed successfully")
            
            # Try to parse the response if it's the expected realtime data format
            if len(response['payload']) >= 22:
                parsed_data = self._parse_realtime_data(response['payload'])
                if parsed_data:
                    # Return both raw response and parsed data
                    response['parsed_data'] = parsed_data
                    return response
                else:
                    self.logger.error("✗ Failed to parse brake control response")
                    return response  # Return raw response even if parsing fails
            else:
                # Short response - just return the operation result
                if len(response['payload']) >= 1:
                    brake_status = response['payload'][0]
                    status_names = {0: "off", 1: "on", 255: "status read"}
                    self.logger.info(f"Brake status: {status_names.get(brake_status, brake_status)}")
                return response
        else:
            self.logger.error("✗ Failed to execute brake control command")
            return None

    def disable_motor(self):
        """
        Disable motor (Command 0x2F)
        
        This command disables the motor operation.
        No payload is required for this command.
        The response contains the same real-time information as read_realtime_info().
        
        Returns:
            dict: Response data containing real-time info, or None on error
        """
        self.logger.info("Disabling motor")
        return self._send_simple_command(0x2F, "motor disable")

    @staticmethod
    def counts_to_degrees(counts):
        """
        Convert position counts to degrees
        
        Args:
            counts (int): Position in counts
            
        Returns:
            float: Position in degrees
        """
        return counts * 360.0 / 16384

    @staticmethod
    def degrees_to_counts(degrees):
        """
        Convert degrees to position counts
        
        Args:
            degrees (float): Position in degrees
            
        Returns:
            int: Position in counts
        """
        return int(round(degrees * 16384 / 360.0))


def setup_logging(level=logging.INFO):
    """Setup logging configuration"""
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    ) 
#!/usr/bin/env python3
# filepath: /home/joseph/project/nuc980/ble-scanner/ble_scanner_simple.py

import sys
import time
import traceback

# Simplified BLE scanner without pc_ble_driver_py dependency
# Uses direct calls to match your C implementation

class SimpleBLEScanner:
    def __init__(self, serial_port, target_name, send_data, service_uuid, write_uuid, response_uuid):
        self.serial_port = serial_port
        self.target_name = target_name.lower()
        self.send_data = send_data.encode('utf-8') if isinstance(send_data, str) else send_data
        self.service_uuid_str = service_uuid
        self.write_uuid_str = write_uuid
        self.response_uuid_str = response_uuid
        
        print(f"Target device: '{self.target_name}'")
        print(f"Send data: {self.send_data}")
        print(f"Service UUID: {service_uuid}")
        print(f"Write UUID: {write_uuid}")
        print(f"Response UUID: {response_uuid}")

    def parse_uuid128(self, uuid_str):
        """Parse UUID string to byte array (Nordic little-endian format)"""
        # Remove dashes and convert to bytes
        uuid_hex = uuid_str.replace('-', '')
        if len(uuid_hex) != 32:
            raise ValueError(f"Invalid UUID length: {uuid_str}")
        
        # Convert hex string to bytes and reverse for Nordic little-endian
        uuid_bytes = bytes.fromhex(uuid_hex)
        return uuid_bytes[::-1]  # Reverse byte order

    def run(self):
        """Run the scanner by calling the C binary"""
        import subprocess
        
        try:
            # Call your working C binary
            cmd = [
                '/data/ble_scanner',
                self.serial_port,
                self.target_name,
                self.send_data.decode('utf-8'),
                self.service_uuid_str,
                self.write_uuid_str,
                self.response_uuid_str
            ]
            
            print("Executing:", ' '.join(cmd))
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            
            print("STDOUT:")
            print(result.stdout)
            
            if result.stderr:
                print("STDERR:")
                print(result.stderr)
                
            return result.returncode == 0
            
        except subprocess.TimeoutExpired:
            print("Scanner timed out")
            return False
        except Exception as e:
            print(f"Error running scanner: {e}")
            return False


def main():
    if len(sys.argv) < 7:
        print("Usage: python3 ble_scanner_simple.py <serial_port> <target_name> <send_data> <service_uuid> <write_uuid> <response_uuid>")
        print("Example: python3 ble_scanner_simple.py /dev/ttyACM0 es110 'R/EBCAECAwQFBg8P' 81DFD14F-BFAA-40C8-BF69-41D8F7213D7A F21B0295-CD04-482F-B6D2-5D4CE5B2A9E1 A07CC6A7-C0D6-4AC8-ADD4-8D692F3E9803")
        return 1
    
    serial_port = sys.argv[1]
    target_name = sys.argv[2]
    send_data = sys.argv[3]
    service_uuid = sys.argv[4]
    write_uuid = sys.argv[5]
    response_uuid = sys.argv[6]
    
    # Create and run scanner
    scanner = SimpleBLEScanner(serial_port, target_name, send_data, service_uuid, write_uuid, response_uuid)
    success = scanner.run()
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
#!/usr/bin/env python3
# filepath: /home/joseph/project/nuc980/ble_scanner_py/ble_scanner.py

import sys
import time
import logging
import base64
from pc_ble_driver_py.ble_driver import *
from pc_ble_driver_py.ble_adapter import BLEAdapter

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class BLEScanner:
    def __init__(self, serial_port, target_name, send_data, service_uuid, write_uuid, response_uuid):
        self.serial_port = serial_port
        self.target_name = target_name.lower()
        self.send_data = send_data.encode('utf-8') if isinstance(send_data, str) else send_data
        self.service_uuid = self.parse_uuid128(service_uuid)
        self.write_uuid = self.parse_uuid128(write_uuid)
        self.response_uuid = self.parse_uuid128(response_uuid)
        
        # BLE state
        self.adapter = None
        self.conn_handle = None
        self.write_handle = None
        self.response_handle = None
        self.scanning = False
        self.connected = False
        
        # UUID types (will be set after registration)
        self.service_uuid_type = None
        self.write_uuid_type = None
        self.response_uuid_type = None
        
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

    def setup_adapter(self):
        """Initialize BLE adapter"""
        try:
            # Create adapter
            self.adapter = BLEAdapter(
                serial_port=self.serial_port,
                baud_rate=1000000
            )
            
            # Set event handlers
            self.adapter.observer_register(self.on_ble_event)
            
            # Open adapter
            self.adapter.open()
            print("Adapter opened successfully")
            
            # Enable BLE stack
            self.adapter.enable()
            print("BLE stack enabled")
            
            return True
            
        except Exception as e:
            print(f"Failed to setup adapter: {e}")
            return False

    def register_custom_uuids(self):
        """Register custom 128-bit UUIDs with the SoftDevice"""
        print("\n--- UUID REGISTRATION ---")
        
        try:
            # Create base UUIDs with 16-bit placeholders at bytes 12-13
            service_base = bytearray(self.service_uuid)
            service_base[12:14] = b'\x00\x00'
            
            write_base = bytearray(self.write_uuid)
            write_base[12:14] = b'\x00\x00'
            
            response_base = bytearray(self.response_uuid)
            response_base[12:14] = b'\x00\x00'
            
            # Register UUIDs
            service_uuid_base = BLEUUIDBase(list(service_base))
            self.service_uuid_type = self.adapter.ble_vs_uuid_add(service_uuid_base)
            service_16bit = (self.service_uuid[13] << 8) | self.service_uuid[12]
            print(f"Service UUID -> Type: {self.service_uuid_type} (16-bit: 0x{service_16bit:04X})")
            
            write_uuid_base = BLEUUIDBase(list(write_base))
            self.write_uuid_type = self.adapter.ble_vs_uuid_add(write_uuid_base)
            write_16bit = (self.write_uuid[13] << 8) | self.write_uuid[12]
            print(f"Write UUID -> Type: {self.write_uuid_type} (16-bit: 0x{write_16bit:04X})")
            
            response_uuid_base = BLEUUIDBase(list(response_base))
            self.response_uuid_type = self.adapter.ble_vs_uuid_add(response_uuid_base)
            response_16bit = (self.response_uuid[13] << 8) | self.response_uuid[12]
            print(f"Response UUID -> Type: {self.response_uuid_type} (16-bit: 0x{response_16bit:04X})")
            
            return True
            
        except Exception as e:
            print(f"Failed to register UUIDs: {e}")
            return False

    def start_scan(self):
        """Start BLE scanning"""
        try:
            scan_params = BLEGapScanParams(
                interval_ms=100,
                window_ms=50,
                timeout_s=30
            )
            
            print("\n--- STARTING SCAN ---")
            self.adapter.ble_gap_scan_start(scan_params)
            self.scanning = True
            print("Scanning for 30 seconds...")
            
            return True
            
        except Exception as e:
            print(f"Failed to start scan: {e}")
            return False

    def connect_to_device(self, peer_addr):
        """Connect to target device"""
        try:
            conn_params = BLEGapConnParams(
                min_conn_interval_ms=50,
                max_conn_interval_ms=70,
                slave_latency=0,
                conn_sup_timeout_ms=4000
            )
            
            print(f"Connecting to device...")
            self.adapter.ble_gap_connect(peer_addr, conn_params)
            return True
            
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def discover_services(self):
        """Discover primary services"""
        try:
            print("\n--- STARTING SERVICE DISCOVERY ---")
            self.adapter.ble_gattc_primary_services_discover(self.conn_handle)
            return True
            
        except Exception as e:
            print(f"Failed to discover services: {e}")
            return False

    def discover_characteristics(self, service):
        """Discover characteristics in a service"""
        try:
            handle_range = BLEGattcHandleRange(
                start_handle=service.handle_range.start_handle,
                end_handle=service.handle_range.end_handle
            )
            
            self.adapter.ble_gattc_characteristics_discover(
                self.conn_handle, 
                handle_range
            )
            return True
            
        except Exception as e:
            print(f"Failed to discover characteristics: {e}")
            return False

    def send_data(self):
        """Send data to write characteristic"""
        if self.write_handle and self.send_data:
            try:
                print(f"Sending data: {self.send_data}")
                
                write_params = BLEGattcWriteParams(
                    write_op=BLEGattWriteOperation.write_req,
                    flags=0,
                    handle=self.write_handle,
                    data=list(self.send_data),
                    offset=0
                )
                
                self.adapter.ble_gattc_write(self.conn_handle, write_params)
                return True
                
            except Exception as e:
                print(f"Failed to send data: {e}")
                return False
        else:
            print("Write handle not found or no data to send")
            return False

    def on_ble_event(self, ble_driver, event):
        """Handle BLE events"""
        if event['evt_id'] == BLEEvtID.gap_evt_adv_report:
            self.on_adv_report(event)
            
        elif event['evt_id'] == BLEEvtID.gap_evt_connected:
            self.on_connected(event)
            
        elif event['evt_id'] == BLEEvtID.gattc_evt_prim_srvc_disc_rsp:
            self.on_service_discovery(event)
            
        elif event['evt_id'] == BLEEvtID.gattc_evt_char_disc_rsp:
            self.on_characteristic_discovery(event)
            
        elif event['evt_id'] == BLEEvtID.gattc_evt_write_rsp:
            self.on_write_response(event)
            
        elif event['evt_id'] == BLEEvtID.gattc_evt_hvx:
            self.on_notification(event)
            
        elif event['evt_id'] == BLEEvtID.gap_evt_timeout:
            self.on_timeout(event)

    def on_adv_report(self, event):
        """Handle advertisement report"""
        adv_report = event['evt']['gap_evt']['params']['adv_report']
        
        # Extract device name from advertisement data
        device_name = self.extract_device_name(adv_report.get('data', []))
        
        if device_name:
            device_name_lower = device_name.lower()
            print(f"Found device: '{device_name_lower}' vs target '{self.target_name}'")
            
            if device_name_lower == self.target_name:
                print(f"Target device '{self.target_name}' found, connecting...")
                
                # Stop scanning
                try:
                    self.adapter.ble_gap_scan_stop()
                    self.scanning = False
                except:
                    pass
                
                # Connect to device
                peer_addr = adv_report['peer_addr']
                self.connect_to_device(peer_addr)

    def extract_device_name(self, adv_data):
        """Extract device name from advertisement data"""
        i = 0
        while i < len(adv_data):
            if i + 1 >= len(adv_data):
                break
                
            length = adv_data[i]
            if length == 0 or i + 1 + length > len(adv_data):
                break
                
            ad_type = adv_data[i + 1]
            
            # Complete Local Name (0x09) or Shortened Local Name (0x08)
            if ad_type == 0x09 or ad_type == 0x08:
                name_data = adv_data[i + 2:i + 1 + length]
                try:
                    return bytes(name_data).decode('utf-8').strip()
                except:
                    return None
                    
            i += 1 + length
            
        return None

    def on_connected(self, event):
        """Handle connection established"""
        self.conn_handle = event['evt']['gap_evt']['conn_handle']
        self.connected = True
        
        print(f"\n=== CONNECTED TO DEVICE ===")
        print(f"Connection Handle: 0x{self.conn_handle:04X}")
        
        # Register custom UUIDs
        if self.register_custom_uuids():
            # Start service discovery
            self.discover_services()

    def on_service_discovery(self, event):
        """Handle service discovery response"""
        services = event['evt']['gattc_evt']['params']['prim_srvc_disc_rsp']['services']
        
        target_service = None
        custom_services = []
        
        for i, service in enumerate(services):
            uuid_type = service['uuid']['type']
            uuid_value = service['uuid']['uuid']
            start_handle = service['handle_range']['start_handle']
            end_handle = service['handle_range']['end_handle']
            
            print(f"Service {i}: Type={uuid_type}, UUID=0x{uuid_value:04X}, Handles=0x{start_handle:04X}-0x{end_handle:04X}", end="")
            
            # Check if this is our target service
            if uuid_type == self.service_uuid_type:
                print(" ← TARGET SERVICE")
                target_service = service
            else:
                print("")
            
            # Store non-standard services
            if uuid_value not in [0x1800, 0x1801]:  # Skip Generic Access and Generic Attribute
                custom_services.append(service)
        
        print(f"\n--- FOUND {len(custom_services)} CUSTOM SERVICES ---")
        
        if target_service:
            print(f"\n--- DISCOVERING CHARACTERISTICS IN TARGET SERVICE ---")
            print(f"Target Service: Type={target_service['uuid']['type']}, UUID=0x{target_service['uuid']['uuid']:04X}, "
                  f"Handles=0x{target_service['handle_range']['start_handle']:04X}-0x{target_service['handle_range']['end_handle']:04X}")
            
            self.discover_characteristics(target_service)
        else:
            print("❌ Target service not found!")

    def on_characteristic_discovery(self, event):
        """Handle characteristic discovery response"""
        chars = event['evt']['gattc_evt']['params']['char_disc_rsp']['chars']
        
        print(f"Target Service Characteristics ({len(chars)} found):")
        
        for i, char in enumerate(chars):
            handle = char['handle_value']
            uuid_type = char['uuid']['type']
            uuid_value = char['uuid']['uuid']
            properties = char['char_props']
            
            print(f"  Char {i}: Handle=0x{handle:04X}, Type={uuid_type}, UUID=0x{uuid_value:04X}", end="")
            
            # Check for matches
            if uuid_type == self.write_uuid_type:
                self.write_handle = handle
                print(" ← WRITE CHAR", end="")
                
            if uuid_type == self.response_uuid_type:
                self.response_handle = handle
                print(" ← RESPONSE CHAR", end="")
            
            # Show properties
            props = []
            if properties.get('read'): props.append('R')
            if properties.get('write'): props.append('W')
            if properties.get('write_wo_resp'): props.append('w')
            if properties.get('notify'): props.append('N')
            if properties.get('indicate'): props.append('I')
            
            print(f" [{''.join(props)}]")
        
        print(f"\n=== DISCOVERY COMPLETE ===")
        print(f"Found Handles: Write=0x{self.write_handle or 0:04X}, Response=0x{self.response_handle or 0:04X}")
        
        # Send data if write handle found
        self.send_data()

    def on_write_response(self, event):
        """Handle write response"""
        print("Write complete!")

    def on_notification(self, event):
        """Handle notifications"""
        hvx = event['evt']['gattc_evt']['params']['hvx']
        handle = hvx['handle']
        data = hvx['data']
        
        print(f"Notification received from handle: 0x{handle:04X}")
        
        if handle == self.response_handle:
            print("Notification from response characteristic:")
            print(f"  Hex: {' '.join(f'{b:02X}' for b in data)}")
            print(f"  ASCII: {''.join(chr(b) if 32 <= b <= 126 else '.' for b in data)}")

    def on_timeout(self, event):
        """Handle timeout events"""
        timeout_src = event['evt']['gap_evt']['params']['timeout']['src']
        if timeout_src == BLEGapTimeoutSrc.scan:
            print("Scan timed out")
            self.scanning = False

    def run(self):
        """Main execution loop"""
        try:
            # Setup adapter
            if not self.setup_adapter():
                return False
            
            # Start scanning
            if not self.start_scan():
                return False
            
            # Wait for connection and operations
            scan_count = 0
            while self.scanning and scan_count < 30:
                time.sleep(1)
                scan_count += 1
                if not self.connected:
                    print(f"Scanning... {scan_count}/30s")
            
            # Keep connected for a bit to handle any responses
            if self.connected:
                print("Waiting for responses...")
                time.sleep(5)
            
            return True
            
        except KeyboardInterrupt:
            print("\nInterrupted by user")
            return False
            
        except Exception as e:
            print(f"Error during execution: {e}")
            return False
            
        finally:
            # Cleanup
            if self.adapter:
                try:
                    if self.scanning:
                        self.adapter.ble_gap_scan_stop()
                    if self.connected:
                        self.adapter.ble_gap_disconnect(self.conn_handle)
                    self.adapter.close()
                except:
                    pass
                    
            print("Application terminated")


def main():
    if len(sys.argv) < 7:
        print("Usage: python3 ble_scanner.py <serial_port> <target_name> <send_data> <service_uuid> <write_uuid> <response_uuid>")
        print("Example: python3 ble_scanner.py /dev/ttyACM0 VAO19L-A1 'R/EBCAECAwQFBg8P' 81DFD14F-BFAA-40C8-BF69-41D8F7213D7A F21B0295-CD04-482F-B6D2-5D4CE5B2A9E1 A07CC6A7-C0D6-4AC8-ADD4-8D692F3E9803")
        return 1
    
    serial_port = sys.argv[1]
    target_name = sys.argv[2]
    send_data = sys.argv[3]
    service_uuid = sys.argv[4]
    write_uuid = sys.argv[5]
    response_uuid = sys.argv[6]
    
    # Create and run scanner
    scanner = BLEScanner(serial_port, target_name, send_data, service_uuid, write_uuid, response_uuid)
    success = scanner.run()
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
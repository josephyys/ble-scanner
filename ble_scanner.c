#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include <time.h>  
#include "sd_rpc.h"
#include "ble.h"
#include "ble_gattc.h"
#include "ble_types.h"
#include <termios.h>
#include <fcntl.h>

// Add base64 decoding (simple version)
// #include <openssl/bio.h>
// #include <openssl/evp.h>

#define MAX_DATA_LEN 128

// Globals for UUIDs and data
uint8_t g_service_uuid[16] = {0};
uint8_t g_write_uuid[16] = {0};
uint8_t g_response_uuid[16] = {0};
uint8_t g_send_data[MAX_DATA_LEN] = {0};
size_t g_send_data_len = 0;
static bool g_verbose_debug= false;
static bool g_decode_base64 = true; // Default to decoding base64
static uint16_t m_cccd_handle = 0;
static bool g_auto_send_enabled = true;
static bool g_is_cccd_enabled = false;
static bool g_connected = false;
static bool g_discovery_complete = false;
static bool g_command_sent = false;
static uint16_t m_conn_handle = 0xFFFF;

// Scan parameters
ble_gap_scan_params_t m_scan_param = {
    .active = 1,            // Active scanning (will send scan requests)
    .use_whitelist = 0,     // Don't use whitelist
    .interval = 0x00A0,     // Scan interval (100ms = 160 * 0.625ms)
    .window = 0x0050,       // Scan window (50ms = 80 * 0.625ms)
    .timeout = 0,           // No timeout, scan continuously
};

// Function declarations
void ble_evt_handler(adapter_t* adapter, ble_evt_t* p_ble_evt);
void log_handler(adapter_t* adapter, sd_rpc_log_severity_t severity, const char* msg);
void status_handler(adapter_t* adapter, sd_rpc_app_status_t status, const char* msg);
void print_adv_report(const ble_gap_evt_adv_report_t* p_adv_report);
void trim_trailing_whitespace(char* str);
void print_ble_address(const uint8_t* addr);
void register_uuid_in_registry(uint8_t uuid_type, const char* description, const uint8_t* uuid_bytes);
void print_uuid_by_type(uint8_t uuid_type);

// Add these includes if not present
#include "ble_gattc.h"
#include "ble_gap.h"


// Serial port configuration
#define SERIAL_PORT     "/dev/ttyACM0"
#define BAUD_RATE       1000000  // 1M baud

// Add global for characteristic handle (set after discovery)
static uint16_t m_char_handle = 0, m_response_handle = 0;

// Store multiple services to check - MOVE THIS BEFORE ble_evt_handler
static int current_service_index = 0;
static ble_gattc_service_t discovered_services[10];
static int total_services = 0;

// Global variables
adapter_t* m_adapter = NULL;
bool m_scan_active = false;


// Add a global for the target name
char g_target_name[32] = "Target";


// Add this structure near your other globals
typedef struct {
    ble_gap_addr_t addr;
    char name[32];
    bool found;
    int8_t rssi;
} stored_device_t;

static stored_device_t g_target_device = {0};

// Add these near your other global variables
uint16_t potential_cccd_handles[4] = {0}; // Will store potential handles
int num_cccd_handles = 0;                 // Number of handles to try
int current_cccd_index = 0;               // Current handle being tried
bool cccd_discovery_complete = false;     // Flag to track if we've tried all handles

// UUID type variables for registration
static uint8_t g_service_uuid_type = 0;
static uint8_t g_write_uuid_type = 0;
static uint8_t g_response_uuid_type = 0;
static bool g_operations_complete = false;
static time_t g_notification_start_time = 0;
static bool g_waiting_for_notification = false;
static bool g_notification_received = false;

// UUID registry for debugging
struct uuid_registry_entry {
    uint8_t uuid_type;
    const char* description;
    const uint8_t* uuid_bytes;
} uuid_registry[10];
static int uuid_registry_count = 0;

// Add this near your other global variables
//static uint16_t m_service_end_handle = 0;
int base64_decode(const char *in, uint8_t *out, int out_size);


// Function to connect to stored device
void connect_to_stored_device() {
    if (!g_target_device.found) {
        printf("No target device stored!\n");
        return;
    }
    
    printf("\n=== CONNECTING TO STORED DEVICE ===\n");
    printf("Connecting to: %s\n", g_target_device.name);
    
    // Reset connection flags
    g_connected = false;
    g_discovery_complete = false;
    g_command_sent = false;
    
    ble_gap_conn_params_t conn_params = {
        .min_conn_interval = 0x0028,
        .max_conn_interval = 0x0038,
        .slave_latency = 0,
        .conn_sup_timeout = 400
    };
    
    uint32_t err = sd_ble_gap_connect(m_adapter, &g_target_device.addr, &m_scan_param, &conn_params, 0);
    if (err != NRF_SUCCESS) {
        printf("Connect failed: %u\n", err);
    }
}

// Function to disconnect
void disconnect_device() {
    if (m_conn_handle != 0xFFFF) {
        printf("\n=== DISCONNECTING DEVICE ===\n");
        uint32_t err = sd_ble_gap_disconnect(m_adapter, m_conn_handle, 0x13);  // Remote user terminated connection
        if (err != NRF_SUCCESS) {
            printf("Disconnect failed: %u\n", err);
        }
    }
}
// Simplified command send function (no CCCD re-enabling)
void send_command_only() {
    if (m_char_handle != 0 && g_send_data_len > 0) {
        printf("\n=== SENDING COMMAND ===\n");
        
        uint8_t data_to_send[MAX_DATA_LEN];
        size_t data_len = 0;
        
        if (g_decode_base64) {
            data_len = base64_decode((char*)g_send_data, data_to_send, MAX_DATA_LEN);
            printf("Sending decoded data (%zu bytes): ", data_len);
            for (int i = 0; i < data_len; i++) {
                printf("%02X ", data_to_send[i]);
            }
            printf("\n");
        } else {
            data_len = g_send_data_len;
            memcpy(data_to_send, g_send_data, data_len);
            printf("Sending raw data: %s\n", g_send_data);
        }
        
        ble_gattc_write_params_t write_params = {
            .write_op = BLE_GATT_OP_WRITE_CMD,
            .flags = 0,
            .handle = m_char_handle,
            .offset = 0,
            .len = data_len,
            .p_value = data_to_send
        };
        
        sd_ble_gattc_write(m_adapter, m_conn_handle, &write_params);
        g_command_sent = true;
        
        // Start timeout for response
        g_notification_start_time = time(NULL);
        g_waiting_for_notification = true;
        g_notification_received = false;
    }
}

// Helper: parse 128-bit UUID string to bytes (Nordic little-endian format)
bool parse_uuid128(const char* uuid_str, uint8_t* uuid_bin) {
    // UUID format: 81DFD14F-BFAA-40C8-BF69-41D8F7213D7A
    unsigned int uuid_bytes[16];
    int n = sscanf(uuid_str,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        &uuid_bytes[0], &uuid_bytes[1], &uuid_bytes[2], &uuid_bytes[3],
        &uuid_bytes[4], &uuid_bytes[5],
        &uuid_bytes[6], &uuid_bytes[7],
        &uuid_bytes[8], &uuid_bytes[9],
        &uuid_bytes[10], &uuid_bytes[11], &uuid_bytes[12], &uuid_bytes[13], &uuid_bytes[14], &uuid_bytes[15]);
    
    if (n != 16) return false;
    
    // Convert to Nordic's little-endian format
    // Reverse the entire UUID byte array
    for (int i = 0; i < 16; ++i) {
        uuid_bin[15-i] = (uint8_t)uuid_bytes[i];
    }
    return true;
}


// Function to make stdin non-blocking
void set_stdin_nonblocking() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// Function to check if a key was pressed (non-blocking)
int kbhit() {
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}


// Add this function before your main()
void try_next_cccd_handle(adapter_t* adapter, uint16_t conn_handle) {
    if (current_cccd_index < num_cccd_handles) {
        m_cccd_handle = potential_cccd_handles[current_cccd_index++];
        printf("Trying to enable notifications on 0xC6A7 via CCCD at handle: 0x%04X (%d of %d)\n", 
               m_cccd_handle, current_cccd_index, num_cccd_handles);
        
        uint8_t notify_enable[] = {0x01, 0x00};
        ble_gattc_write_params_t cccd_write = {
            .write_op = BLE_GATT_OP_WRITE_REQ,
            .flags = 0,
            .handle = m_cccd_handle,
            .offset = 0,
            .len = sizeof(notify_enable),
            .p_value = notify_enable
        };
        
        uint32_t err = sd_ble_gattc_write(adapter, conn_handle, &cccd_write);
        if (err != NRF_SUCCESS) {
            printf("Failed to write to handle 0x%04X, error: %u\n", m_cccd_handle, err);
            try_next_cccd_handle(adapter, conn_handle);
        } else {
            printf("CCCD write request sent to handle 0x%04X\n", m_cccd_handle);
        }
    } else {
        printf("Tried all %d potential CCCD handles without success\n", num_cccd_handles);
        cccd_discovery_complete = true;
        write_to_characteristic(adapter, conn_handle);
    }
}

// Add this function to write to the write characteristic
void write_to_characteristic(adapter_t* adapter, uint16_t conn_handle) {
    // First re-enable notifications if needed
    if (m_cccd_handle != 0 && m_response_handle != 0) {
        printf("Re-enabling notifications before sending command...\n");
        uint8_t notify_enable[] = {0x01, 0x00};
        ble_gattc_write_params_t cccd_write = {
            .write_op = BLE_GATT_OP_WRITE_REQ,
            .flags = 0,
            .handle = m_cccd_handle,
            .offset = 0,
            .len = sizeof(notify_enable),
            .p_value = notify_enable
        };
        // sd_ble_gattc_write(adapter, conn_handle, &cccd_write);
        // usleep(200000); // 100ms
        // Small delay to allow notification to take effect
        // g_is_cccd_enabled = false;
        // while(1){
        //     usleep(100000); // 100ms
        //     if(g_is_cccd_enabled)
        //         break;

        // }
    }

    // Now send the actual command
    if (m_char_handle != 0 && g_send_data_len > 0) {
        printf("\n=== WRITING TO CHARACTERISTIC 0x0295 (Handle: 0x%04X) ===\n", m_char_handle);
        
        uint8_t data_to_send[MAX_DATA_LEN];
        size_t data_len = 0;
        
        if (g_decode_base64) {
            data_len = base64_decode((char*)g_send_data, data_to_send, MAX_DATA_LEN);
            printf("Sending decoded base64 data (%zu bytes): ", data_len);
            for (int i = 0; i < data_len; i++) {
                printf("%02X ", data_to_send[i]);
            }
            printf("\n");
        } else {
            data_len = g_send_data_len;
            memcpy(data_to_send, g_send_data, data_len);
            printf("Sending raw data: %s\n", g_send_data);
        }
        
        ble_gattc_write_params_t write_params = {
            .write_op = BLE_GATT_OP_WRITE_CMD,  // Write without response
            .flags = 0,
            .handle = m_char_handle,
            .offset = 0,
            .len = data_len,
            .flags = 0,
            .handle = m_char_handle,
            .p_value = data_to_send
        };

        sd_ble_gattc_write(adapter, conn_handle, &write_params);
        printf("Data written to characteristic, now waiting for notifications...\n");
        
        // Set up notification monitoring
        g_notification_start_time = time(NULL);
        g_waiting_for_notification = true;
        g_notification_received = false;
    }
}

void to_lowercase(char* str) {
    for (; *str; ++str) *str = tolower(*str);
}



// Add these function implementations before the main() function:

/**
 * Print advertisement report
 */
void print_adv_report(const ble_gap_evt_adv_report_t* p_adv_report)
{
    printf("Device: ");
    print_ble_address(p_adv_report->peer_addr.addr);
    printf(", RSSI: %d", p_adv_report->rssi);
    
    // Try to extract device name from advertisement data
    const uint8_t* p_data = p_adv_report->data;
    uint16_t data_len = p_adv_report->dlen;
    uint16_t pos = 0;
    
    while (pos < data_len) {
        uint8_t field_len = p_data[pos];
        if (pos + 1 + field_len > data_len) break;
        
        uint8_t field_type = p_data[pos + 1];
        if (field_type == 0x09 || field_type == 0x08) { // Complete or shortened local name
            uint8_t name_len = field_len - 1;
            printf(", Name: '");
            for (int i = 0; i < name_len && i < 31; i++) {
                printf("%c", p_data[pos + 2 + i]);
            }
            printf("'");
            break;
        }
        pos += 1 + field_len;
    }
    printf("\n");
}

/**
 * Remove trailing whitespace from string
 */
void trim_trailing_whitespace(char* str)
{
    if (!str) return;
    
    int len = strlen(str);
    while (len > 0 && isspace((unsigned char)str[len - 1])) {
        str[--len] = '\0';
    }
}

// 1. In your adv_report handler, save the address of the device you want to connect to
void ble_evt_handler(adapter_t* adapter, ble_evt_t* p_ble_evt) {
    uint16_t evt_id = p_ble_evt->header.evt_id;
    switch (evt_id) {
       
        case BLE_GAP_EVT_ADV_REPORT: {
            const ble_gap_evt_adv_report_t* p_adv_report = &p_ble_evt->evt.gap_evt.params.adv_report;
            
            // Add NULL checks and bounds checking
            if (p_adv_report->data == NULL || p_adv_report->dlen == 0) {
                printf("Warning: Empty advertisement data\n");
                break;
            }
            
            // Connect to the first device found with the specified name
            const uint8_t* p_data = p_adv_report->data;
            uint16_t data_len = p_adv_report->dlen;
            uint16_t pos = 0;
            char device_name[32] = {0};
            bool found_name = false;
                        


            while (pos < data_len) {
                uint8_t field_len = p_data[pos];
                if (pos + 1 + field_len > data_len) break;
                uint8_t field_type = p_data[pos + 1];
                if (field_type == 0x09 && field_len > 1) {
                    uint8_t name_len = field_len - 1;
                    if (name_len > 30) name_len = 30;
                    memcpy(device_name, &p_data[pos + 2], name_len);
                    device_name[name_len] = '\0';
                    trim_trailing_whitespace(device_name);
                    to_lowercase(device_name);

                    found_name = true;
                    printf("found name '%s' '%s'\n", device_name, g_target_name);
                    break;
                }
                pos += 1 + field_len;
            }
            if (found_name && strcmp(device_name, g_target_name) == 0) {
                printf("Target device '%s' found, storing device info...\n", g_target_name);
                
                // Store device information instead of connecting immediately
                g_target_device.addr = p_adv_report->peer_addr;
                strncpy(g_target_device.name, device_name, sizeof(g_target_device.name) - 1);
                g_target_device.found = true;
                g_target_device.rssi = p_adv_report->rssi;
                
                printf("Device stored: %s, RSSI: %d\n", g_target_device.name, g_target_device.rssi);
                
                // Stop scanning and exit scan loop
                sd_ble_gap_scan_stop(adapter);
                m_scan_active = false;
                g_operations_complete = true;  // This will exit the scan loop
            }
            break;
        }
        case BLE_GAP_EVT_CONNECTED: {
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    g_connected = true;
    printf("\n=== CONNECTED ===\n");
    printf("Connection Handle: 0x%04x\n", m_conn_handle);
    
    // Register custom 128-bit UUIDs with Nordic SoftDevice
    uint32_t err;
    
    printf("\n--- UUID REGISTRATION ---\n");
    
    // Service UUID
    ble_uuid128_t service_base_uuid;
    memcpy(service_base_uuid.uuid128, g_service_uuid, 16);
    service_base_uuid.uuid128[12] = 0x00;
    service_base_uuid.uuid128[13] = 0x00;
    
    err = sd_ble_uuid_vs_add(m_adapter, &service_base_uuid, &g_service_uuid_type);
    if (err != NRF_SUCCESS) {
        printf("Failed to add service UUID, error: %u\n", err);
    }
    printf("Service UUID -> Type: %u (16-bit: 0x%02X%02X)\n", 
           g_service_uuid_type, g_service_uuid[13], g_service_uuid[12]);
    
    // Write UUID
    ble_uuid128_t write_base_uuid;
    memcpy(write_base_uuid.uuid128, g_write_uuid, 16);
    write_base_uuid.uuid128[12] = 0x00;
    write_base_uuid.uuid128[13] = 0x00;
    
    err = sd_ble_uuid_vs_add(m_adapter, &write_base_uuid, &g_write_uuid_type);
    printf("Write UUID -> Type: %u (16-bit: 0x%02X%02X)\n", 
           g_write_uuid_type, g_write_uuid[13], g_write_uuid[12]);
    
    // Response UUID
    ble_uuid128_t response_base_uuid;
    memcpy(response_base_uuid.uuid128, g_response_uuid, 16);
    response_base_uuid.uuid128[12] = 0x00;
    response_base_uuid.uuid128[13] = 0x00;
    
    err = sd_ble_uuid_vs_add(m_adapter, &response_base_uuid, &g_response_uuid_type);
    printf("Response UUID -> Type: %u (16-bit: 0x%02X%02X)\n", 
           g_response_uuid_type, g_response_uuid[13], g_response_uuid[12]);

    printf("\n--- STARTING SERVICE DISCOVERY ---\n");
    sd_ble_gattc_primary_services_discover(m_adapter, m_conn_handle, 0x0001, NULL);
    break;
}
        // After service discovery, discover characteristics:
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP: {
            const ble_gattc_evt_prim_srvc_disc_rsp_t* p_srvc_disc_rsp = 
                &(p_ble_evt->evt.gattc_evt.params.prim_srvc_disc_rsp);
            
            // Look for target service in the response
            bool target_service_found = false;
            ble_gattc_service_t target_service;
            
            for (int i = 0; i < p_srvc_disc_rsp->count; i++) {
                printf("Service %d: Type=%u, UUID=0x%04X, Handles=0x%04X-0x%04X", 
                    i,
                    p_srvc_disc_rsp->services[i].uuid.type,
                    p_srvc_disc_rsp->services[i].uuid.uuid,
                    p_srvc_disc_rsp->services[i].handle_range.start_handle,
                    p_srvc_disc_rsp->services[i].handle_range.end_handle);
                
                // Mark if it's our target service
                if (p_srvc_disc_rsp->services[i].uuid.type == g_service_uuid_type) {
                    printf(" ← TARGET SERVICE");
                    target_service = p_srvc_disc_rsp->services[i];
                    target_service_found = true;
                }
                printf("\n");
                
                // Also store this service in our array for debugging purposes
                if (total_services < 10) {
                    discovered_services[total_services++] = p_srvc_disc_rsp->services[i];
                }
            }
            
            // If target service found, stop service discovery and start characteristic discovery
            if (target_service_found) {
                printf("\n--- TARGET SERVICE FOUND - STOPPING SERVICE DISCOVERY ---\n");
                printf("Target Service: Type=%u, UUID=0x%04X, Handles=0x%04X-0x%04X\n",
                       target_service.uuid.type,
                       target_service.uuid.uuid,
                       target_service.handle_range.start_handle,
                       target_service.handle_range.end_handle);
                
                // Start characteristic discovery for the target service immediately
                ble_gattc_handle_range_t handle_range = {
                    .start_handle = target_service.handle_range.start_handle,
                    .end_handle = target_service.handle_range.end_handle
                };
                
                // Set current_service_index to 0 (the target service will be at index 0)
                current_service_index = 0;
                
                // Only store the target service in our service array
                discovered_services[0] = target_service;
                total_services = 1;
                
                printf("\n--- DISCOVERING CHARACTERISTICS IN TARGET SERVICE ---\n");
                sd_ble_gattc_characteristics_discover(adapter, m_conn_handle, &handle_range);
                return; // Skip the rest of the function
            }
            
            // If we didn't find the target service yet, continue service discovery
            if (!target_service_found && p_srvc_disc_rsp->count > 0) {
                uint16_t last_handle = p_srvc_disc_rsp->services[p_srvc_disc_rsp->count-1].handle_range.end_handle;
                if (last_handle < 0xFFFF) {
                    sd_ble_gattc_primary_services_discover(adapter, m_conn_handle, last_handle + 1, NULL);
                    return; // Continue discovery
                }
            }
            
            // If we reached this point, service discovery is complete but target not found
            if (!target_service_found) {
                printf("\n--- SERVICE DISCOVERY COMPLETE, TARGET SERVICE NOT FOUND ---\n");
            }
            break;
}
                
        case BLE_GATTC_EVT_DESC_DISC_RSP: {
            const ble_gattc_evt_desc_disc_rsp_t* p_desc_disc_rsp = 
                &p_ble_evt->evt.gattc_evt.params.desc_disc_rsp;
            
            printf("Found %d descriptors\n", p_desc_disc_rsp->count);
            
            // Look for CCCD (UUID 0x2902)
            for (int i = 0; i < p_desc_disc_rsp->count; i++) {
                const ble_gattc_desc_t* p_desc = &p_desc_disc_rsp->descs[i];
                
                printf("  Descriptor %d: Handle=0x%04X, UUID=0x%04X\n", 
                    i, p_desc->handle, p_desc->uuid.uuid);
                
                // Check if this is a CCCD (Client Characteristic Configuration Descriptor)
                if (p_desc->uuid.uuid == 0x2902) {
                    m_cccd_handle = p_desc->handle;
                    printf("  Found CCCD at handle 0x%04X\n", m_cccd_handle);
                    
                    // Enable notifications here
                    uint8_t notify_enable[] = {0x01, 0x00}; // Enable notifications
                    ble_gattc_write_params_t cccd_write = {
                        .write_op = BLE_GATT_OP_WRITE_REQ,
                        .flags = 0,
                        .handle = m_cccd_handle,
                        .offset = 0,
                        .len = sizeof(notify_enable),
                        .p_value = notify_enable
                    };
                    
                    uint32_t err = sd_ble_gattc_write(adapter, m_conn_handle, &cccd_write);
                    if (err != NRF_SUCCESS) {
                        printf("Failed to enable notifications, error: %u\n", err);
                    } else {
                        printf("Notification enable request sent, waiting for confirmation...\n");
                    }
                }
            }
            break;
        }
        // After characteristic discovery, save handle and send data:
        case BLE_GATTC_EVT_CHAR_DISC_RSP: {
            const ble_gattc_evt_char_disc_rsp_t* p_char_disc_rsp = &(p_ble_evt->evt.gattc_evt.params.char_disc_rsp);
            
            printf("Service %d Characteristics (%d found):\n", current_service_index, p_char_disc_rsp->count);
            
            // Process characteristics
            for (int i = 0; i < p_char_disc_rsp->count; i++) {
                const ble_gattc_char_t* p_char = &(p_char_disc_rsp->chars[i]);
                
                printf("  Char %d: Handle=0x%04X, Type=%u, UUID=0x%04X", 
                       i, p_char->handle_value, p_char->uuid.type, p_char->uuid.uuid);
                
                // Check for matches
                bool is_write = (p_char->uuid.type == g_write_uuid_type);
                bool is_response = (p_char->uuid.type == g_response_uuid_type);
                
                if (is_write) {
                    m_char_handle = p_char->handle_value;
                    printf(" ← WRITE CHAR");
                }
                if (is_response) {
                    m_response_handle = p_char->handle_value;
                    printf(" ← RESPONSE CHAR");
                }
                
                // Show properties
                printf(" [");
                if (p_char->char_props.read) printf("R");
                if (p_char->char_props.write) printf("W");
                if (p_char->char_props.write_wo_resp) printf("w");
                if (p_char->char_props.notify) printf("N");
                if (p_char->char_props.indicate) printf("I");
                printf("]\n");
            }
            
            // Continue or move to next service
            bool continue_current_service = false;
            if (p_char_disc_rsp->count > 0) {
                uint16_t last_handle = p_char_disc_rsp->chars[p_char_disc_rsp->count-1].handle_decl;
                uint16_t service_end_handle = discovered_services[current_service_index].handle_range.end_handle;
                
                if (last_handle < service_end_handle) {
                    ble_gattc_handle_range_t next_range = {
                        .start_handle = last_handle + 1,
                        .end_handle = service_end_handle
                    };
                    sd_ble_gattc_characteristics_discover(adapter, m_conn_handle, &next_range);
                    continue_current_service = true;
                }
            }
            
            if (!continue_current_service) {
                // MOVED: Enable notifications for response characteristic here
                // After discovering characteristics and finding your notification characteristic:
// In your BLE_GATTC_EVT_CHAR_DISC_RSP handler, after finding the response handle:
if (m_response_handle != 0) {
    // Set up potential CCCD handles to try
    num_cccd_handles = 0;
    
    // Standard location (handle + 1)
    potential_cccd_handles[num_cccd_handles++] = m_response_handle + 1;
    
    // Some devices have it at handle + 2
    potential_cccd_handles[num_cccd_handles++] = m_response_handle + 2;
    
    // Try handle - 1 (rare but possible)
    if (m_response_handle > 1) {
        potential_cccd_handles[num_cccd_handles++] = m_response_handle - 1;
    }
    
    // Add the service end handle too (some devices have unusual layouts)
    potential_cccd_handles[num_cccd_handles++] = 
        discovered_services[current_service_index].handle_range.end_handle;
    
    // Start with the first potential handle
    current_cccd_index = 0;
    cccd_discovery_complete = false;
    
    // Try the first handle
    printf("Testing multiple potential CCCD handles...\n");
    try_next_cccd_handle(adapter, m_conn_handle);
}                
// if (m_response_handle != 0) {
//     printf("Discovering descriptors for response characteristic...\n");
    
//     // Set handle range for descriptor discovery
//     ble_gattc_handle_range_t desc_range = {
//         .start_handle = m_response_handle + 1,
//         .end_handle = discovered_services[current_service_index].handle_range.end_handle
//     };
    
//     // Discover descriptors
//     uint32_t err = sd_ble_gattc_descriptors_discover(adapter, m_conn_handle, &desc_range);
//     if (err != NRF_SUCCESS) {
//         printf("Failed to start descriptor discovery, error: %u\n", err);
//     }
// }
                // if (m_response_handle != 0) {
                //     printf("Enabling notifications on response characteristic...\n");
                //     m_cccd_handle = m_response_handle + 1;  // Save the CCCD handle
                //     printf("CCCD Handle: 0x%04X\n", m_cccd_handle);

                //     // Write to CCCD (Client Characteristic Configuration Descriptor)
                //     uint8_t notify_enable[] = {0x01, 0x00}; // Enable notifications

                //     ble_gattc_write_params_t cccd_write = {
                //         .write_op = BLE_GATT_OP_WRITE_REQ, // Write with response
                //         .flags = 0,
                //         .handle = m_cccd_handle,  // CCCD handle
                //         .offset = 0,
                //         .len = sizeof(notify_enable),
                //         .p_value = notify_enable
                //     };

                //     uint32_t err = sd_ble_gattc_write(adapter, m_conn_handle, &cccd_write);
                //     if (err != NRF_SUCCESS) {
                //         printf("Failed to enable notifications, error: %u\n", err);
                //     } else {
                //         printf("Notification enable request sent, waiting for confirmation...\n");
                //     }
                // }
                
                // Only continue to the next service if we're still processing services
                if (current_service_index < total_services) {
                    current_service_index++;
                    if (current_service_index < total_services) {
                        printf("\n--- DISCOVERING CHARACTERISTICS IN SERVICE %d ---\n", current_service_index);
                        
                        ble_gattc_handle_range_t handle_range = {
                            .start_handle = discovered_services[current_service_index].handle_range.start_handle,
                            .end_handle = discovered_services[current_service_index].handle_range.end_handle
                        };
                        sd_ble_gattc_characteristics_discover(adapter, m_conn_handle, &handle_range);
                    } else {
                        printf("\n=== DISCOVERY COMPLETE ===\n");
                        printf("Found Handles: Write=0x%04X, Response=0x%04X\n", m_char_handle, m_response_handle);
                        
                        // Send data if found and not already sent
                        if (m_char_handle != 0 && g_send_data_len > 0) {
                            printf("Sending data %s to write characteristic...\n", g_send_data);
                            ble_gattc_write_params_t write_params = {
                                .write_op = BLE_GATT_OP_WRITE_CMD,  // Write without response
                                .flags = 0,
                                .handle = m_char_handle,
                                .offset = 0,
                                .len = g_send_data_len,
                                .p_value = g_send_data
                            };
                            sd_ble_gattc_write(adapter, m_conn_handle, &write_params);
                        }
                    }
                }
            }
            break;
        }
        // case 0x0011: // BLE_GATTC_EVT_WRITE_CMD_RSP
        // {
        //     printf("Write command response received (0x0011)\n");
        //     const uint16_t handle = p_ble_evt->evt.gattc_evt.params.write_rsp.handle;  // <-- CHANGED HERE
        //     printf("Write confirmed for handle: 0x%04X\n", handle);
            
        //     // If this is our command characteristic, just acknowledge
        //     if (handle == m_char_handle) {
        //         printf("Command write confirmed\n");
        //     }
        //     break;
        // }
        // Optionally, handle write response:
        case BLE_GATTC_EVT_WRITE_RSP: {

            const uint16_t handle = p_ble_evt->evt.gattc_evt.params.write_rsp.handle;
            g_is_cccd_enabled = true;
            printf("Write response received for handle: 0x%04X\n", handle);
            
            if (handle == m_cccd_handle) {
                printf("Successfully enabled notifications on 0xC6A7 via CCCD at handle 0x%04X\n", m_cccd_handle);
                
                // Only auto-send the first time, not for subsequent CCCD re-enables
                if (g_auto_send_enabled) {
                    write_to_characteristic(adapter, m_conn_handle);
                    g_auto_send_enabled = false; // Disable auto-send after first command
                } else {
                    printf("CCCD re-enabled, ready for manual command sending\n");
                }
            }
            break;
        }
        // Add this case to handle BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE (0x003C)
        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE: {
            printf("Write command transmission complete\n");
            break;
        }
        // Add this case to handle BLE_GATTC_EVT_NOTIFICATION (0x001F)
        // case BLE_GATTC_EVT_NOTIFICATION: {
        //     // This is the event code 0x001F
        //     const ble_gattc_evt_hvx_t* p_notification = &p_ble_evt->evt.gattc_evt.params.hvx;
            
        //     printf("\n!!! NOTIFICATION RECEIVED (event: 0x001F, length: %d) !!!\n", p_notification->len);
        //     printf("Notification from handle: 0x%04X (expecting: 0xC6A7)\n", 
        //            p_notification->handle, m_response_handle);
            
        //     // Dump raw data in hex format
        //     printf("RAW DATA (hex): ");
        //     for (int i = 0; i < p_notification->len; i++) {
        //         printf("%02X ", p_notification->data[i]);
        //     }
        //     printf("\n");
            
        //     // Also print as ASCII where possible
        //     printf("ASCII DATA: ");
        //     for (int i = 0; i < p_notification->len; i++) {
        //         if (isprint(p_notification->data[i])) {
        //             printf("%c", p_notification->data[i]);
        //         } else {
        //             printf(".");
        //         }
        //     }
        //     printf("\n");
            
        //     g_notification_received = true;
        //     break;
        // }
        // Add this special case handler to directly match the notification event ID
        case 0x001F: // HVX/notification handler
    {
        const ble_gattc_evt_hvx_t* p_hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
        
        printf("\n!!! NOTIFICATION RECEIVED !!!\n");
        printf("Notification from handle: 0x%04X (our response char: 0x%04X)\n", 
            p_hvx->handle, m_response_handle);
        
        // Always print notification data
        printf("Data (%d bytes): ", p_hvx->len);
        for (int i = 0; i < p_hvx->len; i++) {
            printf("%02X ", p_hvx->data[i]);
        }
        printf("\n");
        
        // ASCII representation
        printf("ASCII: ");
        for (int i = 0; i < p_hvx->len; i++) {
            if (isprint(p_hvx->data[i])) {
                printf("%c", p_hvx->data[i]);
            } else {
                printf(".");
            }
        }
        printf("\n");
        
        // Mark received if it's from our response characteristic
        if (p_hvx->handle == m_response_handle) {
            printf("✓ Response received from 0xC6A7, disconnecting...\n");
            g_notification_received = true;
            
            // Auto-disconnect after receiving response (like your mobile app)
            usleep(500000);  // Brief delay (500ms)
            disconnect_device();
        } else {
            printf("⚠ This is from a different characteristic (not 0xC6A7)\n");
        }
        break;
    }
            // Handle notification events            
        case BLE_GATTC_EVT_HVX: {
            const ble_gattc_evt_hvx_t* p_hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
            printf("\n!!! NOTIFICATION RECEIVED (length: %d) !!!\n", p_hvx->len);
            printf("Notification from handle: 0x%04X (expecting: 0x%04X)\n", 
                   p_hvx->handle, m_response_handle);
            
            if (p_hvx->handle == m_response_handle) {
                printf("Response data (%d bytes): ", p_hvx->len);
                for (int i = 0; i < p_hvx->len; i++) {
                    printf("%02X ", p_hvx->data[i]);
                }
                printf("\n");
                
                // Also try to print as ASCII if it might be text
                printf("ASCII: ");
                for (int i = 0; i < p_hvx->len; i++) {
                    if (isprint(p_hvx->data[i])) {
                        printf("%c", p_hvx->data[i]);
                    } else {
                        printf(".");
                    }
                }
                printf("\n");

                // Add this line to set the notification received flag
                g_notification_received = true;
    
                // Don't exit here - wait for possible additional notifications
            }
            break;
        }
        case BLE_GATTC_EVT_READ_RSP: {
            // Read response received
            printf("Read response received!\n");
            // Parse p_ble_evt->evt.gattc_evt.params.read_rsp.data
            break;
        }
        case BLE_GAP_EVT_TIMEOUT:
        {
            const ble_gap_evt_timeout_t* p_timeout = &(p_ble_evt->evt.gap_evt.params.timeout);
            if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                printf("Scan timed out\n");
                m_scan_active = false;
            }
            break;
        }
        case BLE_GAP_EVT_DISCONNECTED: {
    printf("\n=== DISCONNECTED ===\n");
    
    // Reset all connection-related variables
    m_conn_handle = 0xFFFF;
    g_connected = false;
    g_discovery_complete = false;
    g_command_sent = false;
    g_waiting_for_notification = false;
    g_notification_received = false;
    g_auto_send_enabled = true;  // Re-enable for next connection
    
    // Reset handles
    m_char_handle = 0;
    m_response_handle = 0;
    m_cccd_handle = 0;
    
    // Reset discovery state
    current_service_index = 0;
    total_services = 0;
    cccd_discovery_complete = false;
    current_cccd_index = 0;
    num_cccd_handles = 0;
    
    printf("Ready for next command\n");
    break;
}
        default:
            printf("Unhandled BLE event: 0x%04X\n", evt_id);
            break;
    }
}

/**
 * Print BLE MAC address
 */
void print_ble_address(const uint8_t* addr)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

/**
 * Log handler
 */
void log_handler(adapter_t* adapter, sd_rpc_log_severity_t severity, const char* msg)
{
    switch (severity)
    {
        case SD_RPC_LOG_ERROR:
            printf("ERROR: %s\n", msg);
            break;
        case SD_RPC_LOG_WARNING:
            printf("WARNING: %s\n", msg);
            break;
        case SD_RPC_LOG_INFO:
            printf("INFO: %s\n", msg);
            break;
        case SD_RPC_LOG_DEBUG:
            printf("DEBUG: %s\n", msg);
            break;
        case SD_RPC_LOG_TRACE:
            printf("TRACE: %s\n", msg);
            break;
        case SD_RPC_LOG_FATAL:
            printf("FATAL: %s\n", msg);
            break;
        default:
            printf("LOG: %s\n", msg);
            break;
    }
}

/**
 * Status handler
 */
void status_handler(adapter_t* adapter, sd_rpc_app_status_t status, const char* msg)
{
    printf("STATUS: %s\n", msg);
}

/**
 * Simple Base64 decoder table
 */
static const unsigned char base64_table[256] = {
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 62, 64, 62, 64, 63,
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 64, 64, 64, 64, 64,
    64,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 64, 64, 64, 64, 63,
    64, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
};

/**
 * Base64 decoder
 */
int base64_decode(const char *in, uint8_t *out, int out_size) {
    int i, j = 0, k;
    uint32_t v;
    size_t len = strlen(in);
    
    // Handle padding
    if (len > 0 && in[len-1] == '=') len--;
    if (len > 0 && in[len-1] == '=') len--;
    
    for (i = 0; i < len; i += 4) {
        // Get values for each group of four base 64 characters
        v = 0;
        for (k = 0; k < 4 && i+k < len; k++) {
            v = (v << 6) + base64_table[(unsigned char)in[i+k]];
        }
        
        // Three bytes will be produced
        if (j < out_size) out[j++] = (v >> 16) & 0xFF;
        if (j < out_size && i+1 < len) out[j++] = (v >> 8) & 0xFF;
        if (j < out_size && i+2 < len) out[j++] = v & 0xFF;
    }
    
    return j;
}

// Helper function to add to registry
void register_uuid_in_registry(uint8_t uuid_type, const char* description, const uint8_t* uuid_bytes) {
    if (uuid_registry_count < 10) {
        uuid_registry[uuid_registry_count].uuid_type = uuid_type;
        uuid_registry[uuid_registry_count].description = description;
        uuid_registry[uuid_registry_count].uuid_bytes = uuid_bytes;
        uuid_registry_count++;
    }
}

// Helper function to print UUID by type
void print_uuid_by_type(uint8_t uuid_type) {
    for (int i = 0; i < uuid_registry_count; i++) {
        if (uuid_registry[i].uuid_type == uuid_type) {
            printf("%s (", uuid_registry[i].description);
            for (int j = 0; j < 16; j++) {
                printf("%02X", uuid_registry[i].uuid_bytes[j]);
                if (j == 3 || j == 5 || j == 7 || j == 9) printf("-");
            }
            printf(")");
            return;
        }
    }
    printf("Unknown UUID type %u", uuid_type);
}

/**
 * Main function
 */
int main(int argc, char* argv[])
{
    uint32_t error_code;
    char* serial_port = SERIAL_PORT;
    uint32_t baud_rate = BAUD_RATE;
    
    // Usage: ./ble_scanner [serial_port] [target_name] [send_data] [service_uuid] [write_uuid] [response_uuid]
    if (argc > 1) {
        serial_port = argv[1];
    }
    if (argc > 2) {
        strncpy(g_target_name, argv[2], sizeof(g_target_name) - 1);
        g_target_name[sizeof(g_target_name) - 1] = '\0';
        trim_trailing_whitespace(g_target_name);
        to_lowercase(g_target_name);
    }
    if (argc > 3) {
        strncpy((char*)g_send_data, argv[3], MAX_DATA_LEN - 1);
        g_send_data[MAX_DATA_LEN - 1] = '\0';
        g_send_data_len = strlen((char*)g_send_data);
    }
    if (argc > 4) {
        if (!parse_uuid128(argv[4], g_service_uuid)) {
            printf("Invalid service UUID: %s\n", argv[4]);
            return 1;
        }
        
    }
    if (argc > 5) {
        if (!parse_uuid128(argv[5], g_write_uuid)) {
            printf("Invalid write UUID: %s\n", argv[5]);
            return 1;
        }
    }
    if (argc > 6) {
        if (!parse_uuid128(argv[6], g_response_uuid)) {
            printf("Invalid response UUID: %s\n", argv[6]);
            return 1;
        }
    }
    // In main(), add verbose flag support:
    if (argc > 7 && strcmp(argv[7], "-v") == 0) {
        g_verbose_debug = true;
        printf("Verbose debug mode enabled\n");
    }

    // Add this after the verbose flag check
    if (argc > 8 && strcmp(argv[8], "-d") == 0) {
        g_decode_base64 = true;
        printf("Base64 decoding enabled\n");
    }

    // Add after parsing UUIDs from command line:
    printf("Debug - Target UUIDs:\n");
    printf("Service UUID: %02X%02X...%02X%02X\n", 
        g_service_uuid[0], g_service_uuid[1], g_service_uuid[14], g_service_uuid[15]);
    printf("Write UUID: %02X%02X...%02X%02X\n", 
        g_write_uuid[0], g_write_uuid[1], g_write_uuid[14], g_write_uuid[15]);
    printf("Response UUID: %02X%02X...%02X%02X\n", 
        g_response_uuid[0], g_response_uuid[1], g_response_uuid[14], g_response_uuid[15]);

    printf("Opening connection to %s at %d baud\n", serial_port, baud_rate);
    printf("Looking for device with name: '%s'\n", g_target_name);

    // Check if serial port exists
    if (access(serial_port, F_OK) == -1)
    {
        printf("ERROR: Serial port %s does not exist\n", serial_port);
        return 1;
    }
    
    // Setup the configuration for physical transport layer
    physical_layer_t* phy = sd_rpc_physical_layer_create_uart(serial_port, baud_rate, 
                                                            SD_RPC_FLOW_CONTROL_NONE, 
                                                            SD_RPC_PARITY_NONE);
    if (phy == NULL)
    {
        printf("Failed to create physical layer\n");
        return 1;
    }
    
    // Setup the data link layer
    data_link_layer_t* data_link = sd_rpc_data_link_layer_create_bt_three_wire(phy, 250);
    if (data_link == NULL)
    {
        printf("Failed to create data link layer\n");
        // No sd_rpc_physical_layer_free, sd_rpc_data_link_layer_free, or sd_rpc_transport_layer_free in v5 API
        // Resources are managed by the adapter and deleted with sd_rpc_adapter_delete
        return 1;
    }
    
    // Setup the transport layer
    transport_layer_t* transport = sd_rpc_transport_layer_create(data_link, 1500);
    if (transport == NULL)
    {
        printf("Failed to create transport layer\n");
        // sd_rpc_data_link_layer_free(data_link);
        return 1;
    }
    
    // Create the adapter
    m_adapter = sd_rpc_adapter_create(transport);
    if (m_adapter == NULL)
    {
        printf("Failed to create adapter\n");
        //sd_rpc_transport_layer_free(transport);
        return 1;
    }
    
    // Open the adapter (fix handler order and types)
    error_code = sd_rpc_open(m_adapter, status_handler, ble_evt_handler, log_handler);
    if (error_code != NRF_SUCCESS)
    {
        printf("Failed to open adapter, error: %d\n", error_code);
        sd_rpc_adapter_delete(m_adapter);
        return 1;
    }
    
    // Enable the BLE stack
    // sd_ble_enable expects only (adapter_t *adapter, uint32_t *p_app_ram_base)
    // Pass NULL for p_app_ram_base if not needed
    error_code = sd_ble_enable(m_adapter, NULL);
    if (error_code != NRF_SUCCESS)
    {
        printf("Failed to enable BLE stack, error: %d\n", error_code);
        sd_rpc_close(m_adapter);
        sd_rpc_adapter_delete(m_adapter);
        return 1;
    }
    
    // Get version of the SoftDevice API
    ble_version_t version;
    error_code = sd_ble_version_get(m_adapter, &version);
    if (error_code == NRF_SUCCESS)
    {
        printf("SoftDevice version: %d.%d (company: %d)\n", 
       version.version_number, 
       version.subversion_number,
       version.company_id);
        
    }
    
    // Set log handler to NULL to reduce debug noise
    sd_rpc_log_handler_severity_filter_set(m_adapter, SD_RPC_LOG_ERROR);
    
    // Start scanning    
    printf("Starting scan...\n");
    error_code = sd_ble_gap_scan_start(m_adapter, &m_scan_param);
    if (error_code != NRF_SUCCESS) {
        printf("Failed to start scanning, error: %d\n", error_code);
        sd_rpc_close(m_adapter);
        sd_rpc_adapter_delete(m_adapter);
        return 1;
    }

    // Add this line to set the scan as active
    m_scan_active = true;  // <-- ADD THIS LINE

    printf("Scanning for up to 30 seconds...\n");
    int scan_count = 0;
    while (m_scan_active && scan_count < 30 && !g_operations_complete)
    {
        sleep(1);
        scan_count++;
        printf("Scanning... %d/30s\n", scan_count);
        fflush(stdout);
        
        // Add a check to exit if operations are complete
        if (g_operations_complete) {
            printf("Operations complete, exiting scan loop.\n");
            break;
        }
        // In your main scanning loop:
        if (g_waiting_for_notification) {
            time_t current_time = time(NULL);
            if (current_time - g_notification_start_time > 2) // Shorter timeout for CCCD testing
                {
                printf("No notification received with current CCCD handle\n");
                g_waiting_for_notification = false;
                
                // Try the next handle if we haven't received a notification
                if (!g_notification_received && !cccd_discovery_complete) {
                    try_next_cccd_handle(m_adapter, m_conn_handle);
                }
            }
        }
    }
    
    // Stop scanning
    printf("Stopping scan\n");
    error_code = sd_ble_gap_scan_stop(m_adapter);
    if (error_code != NRF_SUCCESS)
    {
        printf("Failed to stop scanning, error: %d\n", error_code);
    }

    // Enable interactive mode
    if (g_target_device.found) {
    printf("\n=== INTERACTIVE MODE ===\n");
    printf("Device stored: %s\n", g_target_device.name);
    printf("Press ENTER to send command, 'q' to quit\n");
    
    set_stdin_nonblocking();
    bool interactive_mode = true;
    
    while (interactive_mode) {
        // Handle timeouts during connection/command
        if (g_waiting_for_notification && g_connected) {  // Only check timeout if connected
            time_t current_time = time(NULL);
            if (current_time - g_notification_start_time > 10) {
                printf("Timeout - disconnecting...\n");
                disconnect_device();
                // Don't set g_waiting_for_notification = false here, let disconnect handler do it
            }
        }
        
        // Check for keypress
        if (kbhit()) {
            int ch = getchar();
            if (ch == 'q' || ch == 'Q') {
                printf("Exiting...\n");
                if (g_connected) {
                    disconnect_device();
                    sleep(1);
                }
                interactive_mode = false;
            } else if (ch == '\n') {
                if (!g_connected) {  // Only connect if not already connected
                    printf("Connecting and sending command...\n");
                    connect_to_stored_device();
                } else {
                    printf("Already connected, please wait for completion or timeout...\n");
                }
            }
        }
        
        usleep(50000);
    }
} else {
    printf("No target device found during scan!\n");
}
    
    
    // Close and clean up
    error_code = sd_rpc_close(m_adapter);
    if (error_code != NRF_SUCCESS)
    {
        printf("Failed to close adapter, error: %d\n", error_code);
    }
    
    sd_rpc_adapter_delete(m_adapter);
    printf("Application terminated\n");
    
    return 0;
}
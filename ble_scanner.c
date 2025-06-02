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
static bool g_handles_verified = false;
static uint16_t g_actual_notification_handle = 0;
// Add these global variables at the top of ble_scanner.c:
static uint8_t g_cccd_value = 0;  // Start with notifications disabled
static bool g_first_command_sent = false;
static bool g_waiting_for_user_input = false;
#define BLE_CCCD_NOTIFY                      0x01
// Add these near your other global variables:
// Update your operation mode enum:
typedef enum {
    MODE_IDLE,
    MODE_DISCOVERY_ONLY,
    MODE_SEND_COMMAND,
    MODE_MANUAL_SEND_NO_CCCD    // ← New mode: no CCCD, just send command
} operation_mode_t;

// Add these to suppress unused variable warnings:
// (void)g_first_command_sent;
// (void)g_waiting_for_user_input;
// (void)g_actual_notification_handle;
// (void)g_handles_verified;
// (void)g_is_cccd_enabled;
// (void)g_auto_send_enabled;

// Add this helper function near the top of your file:
void print_timestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    long ms = ts.tv_nsec / 1000000;
    printf("[%ld.%03ld] ", ts.tv_sec, ms);
}

// Add this macro for timestamped logging:
#define LOG_WITH_TIME(format, ...) do { \
    print_timestamp(); \
    printf(format, ##__VA_ARGS__); \
} while(0)

static operation_mode_t g_current_mode = MODE_IDLE;
static bool g_discovery_completed = false;
static time_t g_discovery_start_time = 0;
static time_t g_command_start_time = 0;

// Add this near your other global variables (around line 50):
static bool continue_discovery = false;

// Add this macro near the top of your file after includes:
#define LOG_SD_BLE_CALL(api_call, description) do { \
    printf("🔧 API CALL: %s - %s\n", #api_call, description); \
    uint32_t _err = (api_call); \
    printf("   Result: %s (0x%08X)\n", _err == NRF_SUCCESS ? "SUCCESS" : "ERROR", _err); \
    if (_err != NRF_SUCCESS) { \
        printf("   ❌ API FAILED: %s returned error 0x%08X\n", #api_call, _err); \
    } \
    _err; \
} while(0)

// Separate timeouts for different operations
#define DISCOVERY_TIMEOUT_SECONDS 15
#define COMMAND_TIMEOUT_SECONDS 10
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
void force_send_command(void);

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

// Add this function similar to main.c's hrm_cccd_set():
static uint32_t response_cccd_set(uint8_t value)
{
    ble_gattc_write_params_t write_params;
    uint8_t cccd_value[2] = {value, 0};

    printf("✅ Setting Response CCCD to: %s (0x%02X) to handle (0x%04X)\n", 
           value ? "ENABLED" : "DISABLED", value, m_cccd_handle);

    if (m_cccd_handle == 0) {
        printf("Error: No CCCD handle found\n");
        return NRF_ERROR_INVALID_STATE;
    }

    write_params.handle   = m_cccd_handle;
    write_params.len      = 2;
    write_params.p_value  = cccd_value;
    write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    write_params.offset   = 0;
    write_params.flags= 0;

    // ✅ FIX API LOGGING:
    uint32_t result;
    LOG_SD_BLE_CALL(result = sd_ble_gattc_write(m_adapter, m_conn_handle, &write_params), "CCCD Enable Write Request");

    return result;  
}
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
    
    uint32_t err;
    LOG_SD_BLE_CALL(err = sd_ble_gap_connect(m_adapter, &g_target_device.addr, &m_scan_param, &conn_params, 0), "GAP Connect Request");
    if (err != NRF_SUCCESS) {
        printf("Connect failed: %u\n", err);
    }
}

// Add this function to discover descriptors properly
static uint32_t descr_discovery_start()
{
    if (m_response_handle == 0) {
        printf("No response characteristic handle found\n");
        return NRF_ERROR_INVALID_STATE;
    }

    printf("Discovering descriptors for response characteristic 0x%04X\n", m_response_handle);
    
    ble_gattc_handle_range_t handle_range;
    handle_range.start_handle = m_response_handle + 1;  // ✅ START AFTER the characteristic
    handle_range.end_handle = discovered_services[0].handle_range.end_handle;
    
    printf("Descriptor discovery range: 0x%04X to 0x%04X\n", 
           handle_range.start_handle, handle_range.end_handle);
    
    return sd_ble_gattc_descriptors_discover(m_adapter, m_conn_handle, &handle_range);
}

// Function to disconnect
void disconnect_device() {
    if (m_conn_handle != 0xFFFF) {
        printf("\n=== DISCONNECTING DEVICE ===\n");
        uint32_t err;
LOG_SD_BLE_CALL(err = sd_ble_gap_disconnect(m_adapter, m_conn_handle, 0x13), "Disconnect Request");
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
            .write_op = BLE_GATT_OP_WRITE_REQ, //BLE_GATT_OP_WRITE_CMD,
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
        
        printf("\n=== CCCD ATTEMPT %d of %d ===\n", current_cccd_index, num_cccd_handles);
        printf("Response Char Handle: 0x%04X (expecting notifications from here)\n", m_response_handle);
        printf("Trying CCCD Handle: 0x%04X (writing notification enable here)\n", m_cccd_handle);
        printf("Expected relationship: CCCD at 0x%04X should control notifications from 0x%04X\n", 
               m_cccd_handle, m_response_handle);
        
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
            printf("CCCD write FAILED to handle 0x%04X, error: %u\n", m_cccd_handle, err);
            try_next_cccd_handle(adapter, conn_handle);
        } else {
            printf("CCCD write SUCCESS to handle 0x%04X, waiting for confirmation...\n", m_cccd_handle);
        }
    } else {
        printf("\n=== ALL CCCD HANDLES EXHAUSTED ===\n");
        printf("Tried %d potential CCCD handles without success\n", num_cccd_handles);
        cccd_discovery_complete = true;
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
            .write_op =BLE_GATT_OP_WRITE_REQ,// BLE_GATT_OP_WRITE_CMD,  // Write without response
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
    
    // ✅ ADD TIMESTAMP TO EVERY EVENT
    print_timestamp();
    printf("🎯 EVENT: 0x%04X ", evt_id);
    
    // Add human-readable event names
    switch(evt_id) {
        case 0x0010: printf("(BLE_GAP_EVT_CONNECTED)\n"); break;
        case 0x0011: printf("(BLE_GAP_EVT_DISCONNECTED)\n"); break;
        case 0x001F: printf("(UNKNOWN/HVX/TIMEOUT?)\n"); break;
        case 0x0023: printf("(UNKNOWN)\n"); break;
        case 0x0032: printf("(BLE_GATTC_EVT_CHAR_DISC_RSP)\n"); break;
        case 0x0033: printf("(BLE_GATTC_EVT_DESC_DISC_RSP)\n"); break;
        case 0x0038: printf("(BLE_GATTC_EVT_WRITE_RSP)\n"); break;
        case 0x003C: printf("(BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE)\n"); break;
        case 0x0055: printf("(UNKNOWN)\n"); break;
        default: printf("(UNKNOWN)\n"); break;
    }

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
            printf("Operation Mode: %s\n", 
                g_current_mode == MODE_DISCOVERY_ONLY ? "DISCOVERY ONLY" :
                g_current_mode == MODE_SEND_COMMAND ? "SEND COMMAND" :
                g_current_mode == MODE_MANUAL_SEND_NO_CCCD ? "MANUAL SEND (NO CCCD)" : "IDLE");
            
            // Check for existing handles
            if (m_char_handle != 0 && m_response_handle != 0 && m_cccd_handle != 0) {
                printf("🎯 USING PREVIOUSLY DISCOVERED HANDLES:\n");
                printf("   Write Handle: 0x%04X\n", m_char_handle);
                printf("   Response Handle: 0x%04X\n", m_response_handle);
                printf("   CCCD Handle: 0x%04X\n", m_cccd_handle);
                
                if (g_current_mode == MODE_DISCOVERY_ONLY) {
                    printf("Discovery mode: Handles already known, disconnecting immediately\n");
                    disconnect_device();
                    return;
                } else if (g_current_mode == MODE_SEND_COMMAND) {
                    printf("🚀 Send mode: SKIPPING ALL DISCOVERY - going straight to command\n");
                    g_command_start_time = time(NULL);
                    
                    printf("Enabling CCCD and sending command directly...\n");
                    uint32_t err = response_cccd_set(BLE_CCCD_NOTIFY);
                    if (err != NRF_SUCCESS) {
                        printf("❌ Failed to enable CCCD, error: %u\n", err);
                    }
                    return;
                } else if (g_current_mode == MODE_MANUAL_SEND_NO_CCCD) {
                    printf("🔧 MANUAL SEND (NO CCCD): SKIPPING CCCD - sending command directly\n");
                    g_command_start_time = time(NULL);
                    
                    // Skip CCCD entirely - go straight to command
                    printf("⚠️ SKIPPING CCCD ENABLE - sending command without notifications setup\n");
                    force_send_command();
                    return;
                }
            }
    // ✅ ONLY DO DISCOVERY IF HANDLES ARE NOT KNOWN
    printf("⚠️ Handles not known, proceeding with discovery...\n");
    
    if (g_current_mode == MODE_DISCOVERY_ONLY) {
        printf("🔍 Starting DISCOVERY ONLY mode\n");
        g_discovery_start_time = time(NULL);
    } else if (g_current_mode == MODE_SEND_COMMAND) {
        printf("🚀 Starting SEND COMMAND mode with discovery\n");
        g_command_start_time = time(NULL);
    }
    
    // UUID REGISTRATION AND DISCOVERY CODE (only when needed)
    printf("\n=== UUID REGISTRATION ===\n");
    
    // Register service UUID
    // ble_uuid128_t service_uuid = { {0}, BLE_UUID_TYPE_128 };
    
// ✅ CORRECT - Use BLE_UUID_TYPE_VENDOR_BEGIN or BLE_UUID_TYPE_UNKNOWN:
ble_uuid128_t service_uuid = { {0}, BLE_UUID_TYPE_VENDOR_BEGIN };
// OR
// ble_uuid128_t service_uuid = { {0}, BLE_UUID_TYPE_UNKNOWN };
    memcpy(service_uuid.uuid128, g_service_uuid, 16);
    uint32_t err = sd_ble_uuid_vs_add(m_adapter, &service_uuid, &g_service_uuid_type);
    if (err == NRF_SUCCESS) {
        printf("✅ Service UUID registered as type %u\n", g_service_uuid_type);
        register_uuid_in_registry(g_service_uuid_type, "Service UUID", g_service_uuid);
    } else {
        printf("❌ Failed to register service UUID, error: %u\n", err);
    }
    
    // Register write UUID  
    ble_uuid128_t write_uuid = { {0}, BLE_UUID_TYPE_VENDOR_BEGIN };
    memcpy(write_uuid.uuid128, g_write_uuid, 16);
    err = sd_ble_uuid_vs_add(m_adapter, &write_uuid, &g_write_uuid_type);
    if (err == NRF_SUCCESS) {
        printf("✅ Write UUID registered as type %u\n", g_write_uuid_type);
        register_uuid_in_registry(g_write_uuid_type, "Write UUID", g_write_uuid);
    } else {
        printf("❌ Failed to register write UUID, error: %u\n", err);
    }
    
    // Register response UUID
    ble_uuid128_t response_uuid = { {0}, BLE_UUID_TYPE_VENDOR_BEGIN };
    memcpy(response_uuid.uuid128, g_response_uuid, 16);
    err = sd_ble_uuid_vs_add(m_adapter, &response_uuid, &g_response_uuid_type);
    if (err == NRF_SUCCESS) {
        printf("✅ Response UUID registered as type %u\n", g_response_uuid_type);
        register_uuid_in_registry(g_response_uuid_type, "Response UUID", g_response_uuid);
    } else {
        printf("❌ Failed to register response UUID, error: %u\n", err);
    }
    
    printf("\n--- STARTING SERVICE DISCOVERY ---\n");
    sd_ble_gattc_primary_services_discover(m_adapter, m_conn_handle, 0x0001, NULL);

    printf("\n=== UUID REGISTRATION DEBUG ===\n");
    printf("Service UUID Type: %u\n", g_service_uuid_type);
    printf("Write UUID Type: %u\n", g_write_uuid_type);  
    printf("Response UUID Type: %u\n", g_response_uuid_type);
    printf("Service UUID bytes: ");
    for (int i = 0; i < 16; i++) printf("%02X ", g_service_uuid[i]);
    printf("\nWrite UUID bytes: ");
    for (int i = 0; i < 16; i++) printf("%02X ", g_write_uuid[i]);
    printf("\nResponse UUID bytes: ");
    for (int i = 0; i < 16; i++) printf("%02X ", g_response_uuid[i]);
    printf("\n");

    break;
}
        // After service discovery, discover characteristics:
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP: {
            const ble_gattc_evt_prim_srvc_disc_rsp_t* p_srvc_disc_rsp = 
                &(p_ble_evt->evt.gattc_evt.params.prim_srvc_disc_rsp);
            
            printf("\n=== SERVICE DISCOVERY RESPONSE ===\n");
            
            // Look for target service ONLY
            bool target_service_found = false;
            ble_gattc_service_t target_service;
            
            for (int i = 0; i < p_srvc_disc_rsp->count; i++) {
                printf("Service %d: Type=%u, UUID=0x%04X, Handles=0x%04X-0x%04X", 
                    i,
                    p_srvc_disc_rsp->services[i].uuid.type,
                    p_srvc_disc_rsp->services[i].uuid.uuid,
                    p_srvc_disc_rsp->services[i].handle_range.start_handle,
                    p_srvc_disc_rsp->services[i].handle_range.end_handle);
                
                // Check if this is our target service
                if (p_srvc_disc_rsp->services[i].uuid.type == g_service_uuid_type) {
                    printf(" ← TARGET SERVICE FOUND! ✓");
                    target_service = p_srvc_disc_rsp->services[i];
                    target_service_found = true;
                }
                printf("\n");
            }
            
            // EXPANDED DISCOVERY: Instead of just target service, discover ALL characteristics
            if (target_service_found) {
                printf("\n=== EXPANDING DISCOVERY TO FULL HANDLE TABLE ===\n");
                printf("Target service range: 0x%04X-0x%04X\n",
                       target_service.handle_range.start_handle,
                       target_service.handle_range.end_handle);
                
                // Store the target service
                discovered_services[0] = target_service;
                total_services = 1;
                current_service_index = 0;
                
                // DISCOVER ALL CHARACTERISTICS in the full handle table
                printf("🔍 DISCOVERING ALL CHARACTERISTICS (0x0001-0xFFFF)\n");
                printf("This will find the real notification characteristic!\n");
                
                ble_gattc_handle_range_t full_range = {
                    .start_handle = 0x0001,
                    .end_handle = 0xFFFF
                };
                
                uint32_t err = sd_ble_gattc_characteristics_discover(adapter, m_conn_handle, &full_range);
                if (err != NRF_SUCCESS) {
                    printf("ERROR: Full characteristic discovery failed with error: %u\n", err);
                } else {
                    printf("✅ Full characteristic discovery started!\n");
                }
                return;
            }
            
            // Continue service discovery if target not found yet
            if (!target_service_found && p_srvc_disc_rsp->count > 0) {
                uint16_t last_handle = p_srvc_disc_rsp->services[p_srvc_disc_rsp->count-1].handle_range.end_handle;
                if (last_handle < 0xFFFF) {
                    printf("Continuing service discovery from handle 0x%04X...\n", last_handle + 1);
                    sd_ble_gattc_primary_services_discover(adapter, m_conn_handle, last_handle + 1, NULL);
                }
            }
            break;
}
                
        case BLE_GATTC_EVT_DESC_DISC_RSP: {
            const ble_gattc_evt_desc_disc_rsp_t* p_desc_disc_rsp = 
                &p_ble_evt->evt.gattc_evt.params.desc_disc_rsp;
            
            printf("Received descriptor discovery response, descriptor count: %d\n", p_desc_disc_rsp->count);
            printf("Looking for CCCD (UUID 0x2902)...\n");
            
            bool cccd_found = false;
            
            for (int i = 0; i < p_desc_disc_rsp->count; i++) {
                printf("Descriptor %d: handle=0x%04X, UUID=0x%04X\n", 
                       i, p_desc_disc_rsp->descs[i].handle, p_desc_disc_rsp->descs[i].uuid.uuid);
               
                if (p_desc_disc_rsp->descs[i].uuid.uuid == 0x2902) { // BLE_UUID_CCCD
                    m_cccd_handle = p_desc_disc_rsp->descs[i].handle;
                    printf("✅ FOUND REAL CCCD at handle 0x%04X\n", m_cccd_handle);
                    cccd_found = true;

                    // Toggle CCCD value like main.c
                    g_cccd_value ^= BLE_CCCD_NOTIFY;  // Flip between 0x01 and 0x00
                    
                    printf("\n=== TOGGLING CCCD ===\n");
                    printf("Previous state: %s\n", (g_cccd_value ^ BLE_CCCD_NOTIFY) ? "ENABLED" : "DISABLED");
                    printf("New state: %s\n", g_cccd_value ? "ENABLED" : "DISABLED");
                    
                    // Set the CCCD
                    uint32_t err = response_cccd_set(g_cccd_value);
                    if (err != NRF_SUCCESS) {
                        printf("❌ Failed to set CCCD, error: %u\n", err);
                    }                    
                    
                    // // Enable notifications using the REAL CCCD handle
                    // uint8_t notify_enable[] = {0x01, 0x00};
                    // ble_gattc_write_params_t cccd_write = {
                    //     .write_op = BLE_GATT_OP_WRITE_REQ,
                    //     .flags = 0,
                    //     .handle = m_cccd_handle,
                    //     .offset = 0,
                    //     .len = sizeof(notify_enable),
                    //     .p_value = notify_enable
                    // };
            
                    // uint32_t err = sd_ble_gattc_write(m_adapter, m_conn_handle, &cccd_write);
                    // if (err != NRF_SUCCESS) {
                    //     printf("Failed to enable notifications, error: %u\n", err);
                    // } else {
                    //     printf("Notification enable request sent to REAL CCCD 0x%04X\n", m_cccd_handle);
                    // }
                    break; // Found CCCD, no need to continue
                }
            }
            
            if (cccd_found) {
                printf("✅ FOUND REAL CCCD at handle 0x%04X\n", m_cccd_handle);
                g_discovery_completed = true;
                
                // Handle based on operation mode
                if (g_current_mode == MODE_DISCOVERY_ONLY) {
                    printf("\n🎉 DISCOVERY ONLY COMPLETE!\n");
                    printf("Discovered handles:\n");
                    printf("  - Write Handle: 0x%04X\n", m_char_handle);
                    printf("  - Response Handle: 0x%04X\n", m_response_handle);
                    printf("  - CCCD Handle: 0x%04X\n", m_cccd_handle);
                    printf("Disconnecting (discovery only mode)...\n");
                    disconnect_device();
                    return;
                } else if (g_current_mode == MODE_SEND_COMMAND) {
                    printf("\n🚀 DISCOVERY COMPLETE, PROCEEDING TO COMMAND\n");
                    // Enable CCCD and send command
                    uint32_t err = response_cccd_set(BLE_CCCD_NOTIFY);
                    if (err != NRF_SUCCESS) {
                        printf("❌ Failed to set CCCD, error: %u\n", err);
                    }
                }
            } else {
                // Handle CCCD not found based on mode
                if (g_current_mode == MODE_DISCOVERY_ONLY) {
                    printf("❌ DISCOVERY FAILED: No CCCD found\n");
                    disconnect_device();
                } else if (g_current_mode == MODE_SEND_COMMAND) {
                    printf("⚠️ No CCCD found, attempting command anyway\n");
                    force_send_command();
                }
            }
            break;
        }
        // After characteristic discovery, save handle and send data:
        case BLE_GATTC_EVT_CHAR_DISC_RSP: {
            const ble_gattc_evt_char_disc_rsp_t* p_char_disc_rsp = &(p_ble_evt->evt.gattc_evt.params.char_disc_rsp);
            
            printf("\n=== FULL CHARACTERISTIC DISCOVERY RESULTS ===\n");
            printf("Found %d characteristics in this batch:\n", p_char_disc_rsp->count);
            
            // Process ALL characteristics and look for notification-capable ones
            for (int i = 0; i < p_char_disc_rsp->count; i++) {
                const ble_gattc_char_t* p_char = &(p_char_disc_rsp->chars[i]);
                
                printf("\n--- CHAR %d: Handle 0x%04X ---\n", i, p_char->handle_value);
                printf("Declaration: 0x%04X, Value: 0x%04X\n", p_char->handle_decl, p_char->handle_value);
                printf("UUID Type: %u, 16-bit UUID: 0x%04X\n", p_char->uuid.type, p_char->uuid.uuid);
                
                // Check properties
                printf("Properties: ");
                if (p_char->char_props.read) printf("READ ");
                if (p_char->char_props.write) printf("WRITE ");
                if (p_char->char_props.write_wo_resp) printf("WRITE_NO_RESP ");
                if (p_char->char_props.notify) printf("NOTIFY ");
                if (p_char->char_props.indicate) printf("INDICATE ");
                printf("\n");
                
                // ✅ FIX: Only assign if UUID type matches AND handle not already assigned
                if (p_char->uuid.type == g_write_uuid_type && m_char_handle == 0) {
                    m_char_handle = p_char->handle_value;
                    printf("✅ WRITE CHARACTERISTIC: Handle 0x%04X (UUID Type %u)\n", 
                           m_char_handle, p_char->uuid.type);
                }
                
                if (p_char->uuid.type == g_response_uuid_type && m_response_handle == 0) {
                    m_response_handle = p_char->handle_value;
                    printf("✅ RESPONSE CHARACTERISTIC: Handle 0x%04X (UUID Type %u)\n", 
                           m_response_handle, p_char->uuid.type);
                }
                
                // Debug: Show what we're looking for
                printf("Looking for: Write UUID Type %u, Response UUID Type %u\n", 
                       g_write_uuid_type, g_response_uuid_type);
            }
            
            // ✅ ADD THIS MISSING CONTINUATION LOGIC:
            // Reset continue_discovery flag
            continue_discovery = false;
            
            if (p_char_disc_rsp->count > 0) {
                uint16_t last_value_handle = 0;
                for (int i = 0; i < p_char_disc_rsp->count; i++) {
                    if (p_char_disc_rsp->chars[i].handle_value > last_value_handle) {
                        last_value_handle = p_char_disc_rsp->chars[i].handle_value;
                    }
                }
                
                if (last_value_handle < 0xFFFF) {
                    printf("\nContinuing full discovery from 0x%04X...\n", last_value_handle + 1);
                    ble_gattc_handle_range_t next_range = {
                        .start_handle = last_value_handle + 1,
                        .end_handle = 0xFFFF
                    };
                    sd_ble_gattc_characteristics_discover(adapter, m_conn_handle, &next_range);
                    continue_discovery = true;
                }
            }
            
            // Discovery complete
          // Replace the existing if (!continue_discovery) block around line 755-800

// Discovery complete
if (!continue_discovery) {
    printf("\n=== CHARACTERISTIC DISCOVERY COMPLETE ===\n");
    printf("Write Handle: %s (0x%04X)\n", m_char_handle ? "FOUND" : "NOT FOUND", m_char_handle);
    printf("Response Handle: %s (0x%04X)\n", m_response_handle ? "FOUND" : "NOT FOUND", m_response_handle);
    
    if (m_response_handle != 0) {
        // Start descriptor discovery to find the REAL CCCD
        printf("Starting descriptor discovery for response characteristic...\n");
        uint32_t err = descr_discovery_start();
        if (err != NRF_SUCCESS) {
            printf("❌ ERROR: Failed to start descriptor discovery, error: %u\n", err);
        }
    } else {
        printf("❌ ERROR: Response characteristic not found!\n");
        
        // Handle based on mode
        if (g_current_mode == MODE_DISCOVERY_ONLY) {
            printf("Discovery mode: Disconnecting after failed discovery\n");
            disconnect_device();
        } else if (g_current_mode == MODE_SEND_COMMAND) {
            printf("Command mode: Attempting command without notifications\n");
            if (m_char_handle != 0) {
                force_send_command();
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
            printf("Write response received for handle: 0x%04X\n", handle);
            
            if (handle == m_cccd_handle) {
                printf("✅ CCCD %s successfully!\n", g_cccd_value ? "ENABLED" : "DISABLED");
                
                // ✅ ADD THIS CRITICAL FIX: Call tx_process after CCCD write response
                print_timestamp();
                // printf("🔧 Calling sd_ble_gattc_tx_process to flush TX queue...\n");
                // uint32_t tx_err = sd_ble_gattc_tx_process(m_adapter, m_conn_handle);
                // if (tx_err == NRF_SUCCESS) {
                //     printf("✅ TX process completed successfully\n");
                // } else {
                //     printf("⚠️ TX process failed: 0x%08X\n", tx_err);
                // }
                
                // Only send command in SEND_COMMAND mode
                if (g_current_mode == MODE_SEND_COMMAND) {
                    printf("CCCD enabled, sending command...\n");
                    force_send_command();
                } else {
                    printf("Discovery mode: CCCD enabled but not sending command\n");
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
            // Add this special case handler to directly match the notification event ID
    //     case 0x001F: // HVX/notification handler
    // {
    //     const ble_gattc_evt_hvx_t* p_hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
        
    //     printf("\n!!! NOTIFICATION RECEIVED !!!\n");
    //     printf("From handle: 0x%04X, Expected: 0x%04X\n", p_hvx->handle, m_response_handle);
        
    //     if (p_hvx->handle == m_response_handle) {
    //         printf("✅ CORRECT RESPONSE NOTIFICATION!\n");
            
    //         printf("Data (%d bytes): ", p_hvx->len);
    //         for (int i = 0; i < p_hvx->len; i++) {
    //             printf("%02X ", p_hvx->data[i]);
    //         }
    //         printf("\n");
            
    //         g_notification_received = true;
    //         disconnect_device();
    //     } else {
    //         printf("📝 Unexpected notification from handle 0x%04X\n", p_hvx->handle);
    //     }
    //     break;
    // }
            // Handle notification events  BLE_GATTC_EVT_HVX  0x001F          
        case 0x0039: {  // BLE_GATTC_EVT_HVX - The REAL notification event!
            print_timestamp();
            printf("!!! REAL NOTIFICATION EVENT 0x0039 (BLE_GATTC_EVT_HVX) !!!\n");
            
            const ble_gattc_evt_hvx_t* p_hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
            
            print_timestamp();
            printf("Notification Handle: 0x%04X, Length: %d bytes\n", p_hvx->handle, p_hvx->len);
            print_timestamp();
            printf("Type: %s\n", p_hvx->type == BLE_GATT_HVX_NOTIFICATION ? "NOTIFICATION" : "INDICATION");
            
            if (p_hvx->handle == 0x00A0 || p_hvx->handle == m_response_handle) {
                print_timestamp();
                printf("🎉 REAL NOTIFICATION RECEIVED from handle 0x%04X!\n", p_hvx->handle);
                
                print_timestamp();
                printf("Response data (%d bytes):\n", p_hvx->len);
                for (int i = 0; i < p_hvx->len; i++) {
                    printf("%02X ", p_hvx->data[i]);
                    if ((i + 1) % 16 == 0) printf("\n");
                }
                printf("\n");
                
                g_notification_received = true;
                print_timestamp();
                printf("🎉 COMMAND SUCCESSFUL - Real notification received!\n");
                disconnect_device();
            } else {
                print_timestamp();
                printf("📝 Notification from unexpected handle: 0x%04X\n", p_hvx->handle);
            }
            break;
        }
        // Keep 0x001F as GAP event handler:
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
    printf("Mode was: %s\n", 
           g_current_mode == MODE_DISCOVERY_ONLY ? "DISCOVERY ONLY" :
           g_current_mode == MODE_SEND_COMMAND ? "SEND COMMAND" : "IDLE");
    
    // Reset connection-related variables
    m_conn_handle = 0xFFFF;
    g_connected = false;
    g_waiting_for_notification = false;
    g_notification_received = false;
    
    // Reset mode
    g_current_mode = MODE_IDLE;
    
    // ✅ ALWAYS KEEP HANDLES - Never clear them once discovered
    if (g_discovery_completed && m_char_handle != 0 && m_response_handle != 0 && m_cccd_handle != 0) {
        printf("✅ Discovery was successful, KEEPING handles for next operation:\n");
        printf("   Write Handle: 0x%04X\n", m_char_handle);
        printf("   Response Handle: 0x%04X\n", m_response_handle);
        printf("   CCCD Handle: 0x%04X\n", m_cccd_handle);
    } else {
        printf("Discovery was incomplete, clearing handles\n");
        m_char_handle = 0;
        m_response_handle = 0;
        m_cccd_handle = 0;
        g_discovery_completed = false;  // Reset discovery flag
    }
    
    printf("Ready for next command ('d' for discovery, 's' for send command)\n");
    break;
}
        // Add this case in your ble_evt_handler switch statement, BEFORE the default case:
        case 0x001F: {
    print_timestamp();
    printf("!!! GAP EVENT 0x001F (Connection Parameter Update or similar) !!!\n");
    print_timestamp();
    printf("This is a GAP layer event, not related to notifications\n");
    // Don't disconnect - this is not an error
    break;
}

//         case 0x001F: {
//     printf("\n!!! EVENT 0x001F RECEIVED (TIMEOUT) !!!\n");
    
//     // Try to cast as notification data first
//     const ble_gattc_evt_hvx_t* p_hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
    
//     printf("Handle: 0x%04X, Length: %d bytes\n", p_hvx->handle, p_hvx->len);
//     printf("Data: ");
//     for (int i = 0; i < p_hvx->len && i < 20; i++) {
//         printf("%02X ", p_hvx->data[i]);
//     }
//     printf("\n");
    
//     // Check if this matches expected response handle
//     if (p_hvx->handle == m_response_handle || p_hvx->handle == 0x00A0) {
//         printf("✅ This appears to be response data from 0x%04X!\n", p_hvx->handle);
//         g_notification_received = true;
//         disconnect_device();
//     } else {
//         printf("⚠️ This appears to be a timeout event, not notification data\n");
//     }
//     break;
// }
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

// Add this function to print detailed UUID information
void print_detailed_uuid_info(const char* label, const uint8_t* uuid_128, uint8_t uuid_type) {
    printf("=== %s ===\n", label);
    printf("128-bit UUID: ");
    for (int i = 0; i < 16; i++) {
        printf("%02X", uuid_128[i]);
        if (i == 3 || i == 5 || i == 7 || i == 9) printf("-");
    }
    printf("\n");
    printf("Nordic format: ");
    for (int i = 0; i < 16; i++) {
        printf("%02X ", uuid_128[i]);
    }
    printf("\n");
    printf("16-bit part: 0x%02X%02X (bytes 12-13)\n", uuid_128[13], uuid_128[12]);
    printf("Registered UUID Type: %u\n", uuid_type);
    printf("Expected 16-bit UUID for comparison: 0x%02X%02X\n", uuid_128[13], uuid_128[12]);
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
    
    // Add this right after parsing UUIDs in main()
    printf("\n=== UUID PARSING VERIFICATION ===\n");
    printf("Expo Response UUID: a07cc6a7-c0d6-4ac8-add4-8d692f3e9803\n");
    printf("Parsed Response UUID (Nordic format): ");
    for (int i = 0; i < 16; i++) {
        printf("%02X ", g_response_uuid[i]);
    }
    printf("\n");
    printf("Parsed Response UUID (Standard format): ");
    for (int i = 15; i >= 0; i--) {
        printf("%02x", g_response_uuid[i]);
        if (i == 12 || i == 10 || i == 8 || i == 6) printf("-");
    }
    printf("\n");
    printf("16-bit part for comparison: 0x%02X%02X\n", g_response_uuid[13], g_response_uuid[12]);

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
            if (current_time - g_notification_start_time > 30) // Shorter timeout for CCCD testing
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
    printf("Commands:\n");
    printf("  'd' - Discovery only (find handles)\n");
    printf("  's' - Send command (with CCCD enable)\n");
    printf("  'm' - Manual send (NO CCCD, direct command only)\n");  // ← New option
    printf("  'q' - Quit\n");
    
    set_stdin_nonblocking();
    bool interactive_mode = true;
    
    while (interactive_mode) {
        //sd_rpc_process_events(m_adapter);
        //sd_rpc_conn_continue(m_adapter);
        // Handle discovery timeout
        if (g_current_mode == MODE_DISCOVERY_ONLY && g_connected) {
            time_t current_time = time(NULL);
            if (current_time - g_discovery_start_time > DISCOVERY_TIMEOUT_SECONDS) {
                printf("Discovery timeout (%d seconds) - disconnecting...\n", DISCOVERY_TIMEOUT_SECONDS);
                disconnect_device();
            }
        }
        
        // Handle command timeout
        if (g_current_mode == MODE_SEND_COMMAND && g_connected && g_waiting_for_notification) {
            time_t current_time = time(NULL);
            time_t elapsed = current_time - g_command_start_time;
            
            if (elapsed > COMMAND_TIMEOUT_SECONDS) {
                print_timestamp();
                printf("Command timeout (%ld seconds elapsed) - disconnecting...\n", elapsed);
                disconnect_device();
            }
            
            // Print periodic status
            static time_t last_status_time = 0;
            if (current_time - last_status_time >= 2) {  // Every 2 seconds
                print_timestamp();
                printf("Waiting for response... (%ld/%d seconds)\n", elapsed, COMMAND_TIMEOUT_SECONDS);
                last_status_time = current_time;
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
            } else if (ch == 'd' || ch == 'D') {
                if (!g_connected) {
                    printf("🔍 Starting DISCOVERY ONLY mode...\n");
                    g_current_mode = MODE_DISCOVERY_ONLY;
                    connect_to_stored_device();
                } else {
                    printf("Already connected, please wait or press 'q' to quit\n");
                }
            } else if (ch == 's' || ch == 'S') {
                if (!g_connected) {
                    printf("🚀 Starting SEND COMMAND mode...\n");
                    g_current_mode = MODE_SEND_COMMAND;
                    connect_to_stored_device();
                } else {
                    printf("Already connected, please wait or press 'q' to quit\n");
                }
            // Add manual mode handler
            } else if (ch == 'm' || ch == 'M') {
                if (!g_connected) {
                    printf("🔧 Starting MANUAL SEND (NO CCCD) mode...\n");
                    g_current_mode = MODE_MANUAL_SEND_NO_CCCD;
                    connect_to_stored_device();
                } else {
                    printf("Already connected, please wait or press 'q' to quit\n");
                }
            }
        }
        
        usleep(50000);
    }
}
else {
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

void force_send_command() {
    print_timestamp();
    printf("🚀 FORCE SENDING COMMAND TO WRITE CHARACTERISTIC 0x%04X\n", m_char_handle);
    
    uint8_t data_to_send[MAX_DATA_LEN];
    size_t data_len = 0;
    
    if (g_decode_base64) {
        data_len = base64_decode((char*)g_send_data, data_to_send, MAX_DATA_LEN);
        printf("Decoded data (%zu bytes): ", data_len);
        for (int i = 0; i < data_len; i++) {
            printf("%02X ", data_to_send[i]);
        }
        printf("\n");
    } else {
        data_len = g_send_data_len;
        memcpy(data_to_send, g_send_data, data_len);
    }
    
    ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,//BLE_GATT_OP_WRITE_CMD,  // Write without response
        .flags = 0,
        .handle = m_char_handle,
        .offset = 0,
        .len = data_len,
        .p_value = data_to_send
    };
    
    print_timestamp();
    printf("🔧 API CALL: sd_ble_gattc_write - Command Write (No Response Expected)\n");
    uint32_t err = sd_ble_gattc_write(m_adapter, m_conn_handle, &write_params);
    print_timestamp();
    printf("   Result: %s (0x%08X)\n", err == NRF_SUCCESS ? "SUCCESS" : "ERROR", err);
    
    if (err == NRF_SUCCESS) {
        // ✅ ADD TX PROCESS AFTER COMMAND WRITE TOO
        print_timestamp();
        // printf("🔧 Calling sd_ble_gattc_tx_process after command write...\n");
        // uint32_t tx_err = sd_ble_gattc_tx_process(m_adapter, m_conn_handle);
        // if (tx_err == NRF_SUCCESS) {
        //     printf("✅ TX process completed successfully\n");
        // } else {
        //     printf("⚠️ TX process failed: 0x%08X\n", tx_err);
        // }
        
        print_timestamp();
        printf("✅ Command sent successfully! Now waiting for response...\n");
        g_notification_start_time = time(NULL);
        g_waiting_for_notification = true;
    }
}
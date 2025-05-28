#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include "sd_rpc.h"
#include "ble.h"
#include "ble_gattc.h"
#include "ble_types.h"

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

// UUID type variables for registration
static uint8_t g_service_uuid_type = 0;
static uint8_t g_write_uuid_type = 0;
static uint8_t g_response_uuid_type = 0;
static bool g_operations_complete = false;

// UUID registry for debugging
struct uuid_registry_entry {
    uint8_t uuid_type;
    const char* description;
    const uint8_t* uuid_bytes;
} uuid_registry[10];
static int uuid_registry_count = 0;

// Add this near your other global variables
//static uint16_t m_service_end_handle = 0;

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

// Helper: decode base64 to binary

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

void to_lowercase(char* str) {
    for (; *str; ++str) *str = tolower(*str);
}

// Add global variables for connection
static uint16_t m_conn_handle = 0xFFFF;
static ble_gap_addr_t m_target_addr; // Set this when you find your target device


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
            const ble_gap_evt_adv_report_t* p_adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
            // print_adv_report(p_adv_report);  // Comment this out to reduce noise
            
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
                printf("Target device '%s' found, connecting...\n", g_target_name);
                m_target_addr = p_adv_report->peer_addr;
                sd_ble_gap_scan_stop(adapter);
                ble_gap_conn_params_t conn_params = {
                    .min_conn_interval = 0x0028,
                    .max_conn_interval = 0x0038,
                    .slave_latency = 0,
                    .conn_sup_timeout = 400
                };
                uint32_t err = sd_ble_gap_connect(adapter, &m_target_addr, &m_scan_param, &conn_params, 0);
                if (err != NRF_SUCCESS) {
                    printf("Connect failed: %u\n", err);
                }
            }
            break;
        }
        case BLE_GAP_EVT_CONNECTED: {
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    printf("\n=== CONNECTED TO DEVICE ===\n");
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
                if (m_response_handle != 0) {
                    printf("Enabling notifications on response characteristic...\n");
                    printf("CCCD Handle: 0x%04X\n", m_response_handle + 1);

                    // Write to CCCD (Client Characteristic Configuration Descriptor)
                    uint8_t notify_enable[] = {0x01, 0x00}; // Enable notifications

                    ble_gattc_write_params_t cccd_write = {
                        .write_op = BLE_GATT_OP_WRITE_REQ, // Write with response
                        .flags = 0,
                        .handle = m_response_handle + 1,  // CCCD handle is typically response_handle + 1
                        .offset = 0,
                        .len = sizeof(notify_enable),
                        .p_value = notify_enable
                    };

                    uint32_t err = sd_ble_gattc_write(adapter, m_conn_handle, &cccd_write);
                    if (err != NRF_SUCCESS) {
                        printf("Failed to enable notifications, error: %u\n", err);
                    } else {
                        printf("Notifications enabled successfully\n");
                        
                        // Add this section to send the command right after enabling notifications 
                        // instead of waiting for all services to be discovered
                        if (m_char_handle != 0 && g_send_data_len > 0) {
                            printf("\n=== SENDING COMMAND DATA ===\n");
                            printf("Found Handles: Write=0x%04X, Response=0x%04X\n", m_char_handle, m_response_handle);
                            
                            uint8_t data_to_send[MAX_DATA_LEN];
                            size_t data_len = 0;
                            
                            if (g_decode_base64) {
                                // Decode base64 data
                                data_len = base64_decode((char*)g_send_data, data_to_send, MAX_DATA_LEN);
                                printf("Sending decoded base64 data (%zu bytes): ", data_len);
                                for (int i = 0; i < data_len; i++) {
                                    printf("%02X ", data_to_send[i]);
                                }
                                printf("\n");
                            } else {
                                // Use raw data
                                data_len = g_send_data_len;
                                memcpy(data_to_send, g_send_data, data_len);
                                printf("Sending raw data %s to write characteristic...\n", g_send_data);
                            }
                            
                            ble_gattc_write_params_t write_params = {
                                .write_op = BLE_GATT_OP_WRITE_CMD,  // Write without response
                                .flags = 0,
                                .handle = m_char_handle,
                                .offset = 0,
                                .len = data_len,
                                .p_value = data_to_send
                            };
                            sd_ble_gattc_write(adapter, m_conn_handle, &write_params);

                            // Skip to the end of service discovery, we're done with the target service
                            current_service_index = total_services;
                        }
                    }
                }
                
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
        // Optionally, handle write response:
        case BLE_GATTC_EVT_WRITE_RSP: {
            printf("Write complete!\n");
            
            // If we're done with all operations, set the flag to exit the program
            if (m_char_handle != 0 && m_response_handle != 0) {
                printf("All operations completed successfully!\n");
                g_operations_complete = true;
            }
            break;
        }
        case BLE_GATTC_EVT_HVX: {
            const ble_gattc_evt_hvx_t* hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
            printf("Notification received from handle: 0x%04X\n", hvx->handle);
            
            // Check if this is from our response characteristic
            if (hvx->handle == m_response_handle) {
                printf("Notification from response characteristic: ");
                // Print the notification data as hex
                for (int i = 0; i < hvx->len; i++) {
                    printf("%02X ", hvx->data[i]);
                }
                printf("\n");
                
                // Also try to print as ASCII if possible
                printf("ASCII: ");
                for (int i = 0; i < hvx->len; i++) {
                    if (isprint(hvx->data[i])) {
                        printf("%c", hvx->data[i]);
                    } else {
                        printf(".");
                    }
                }
                printf("\n");
                // Add custom logic to process the notification data
                // Example: Check if the notification matches a specific transaction ID
                // uint8_t expected_trans_id = ...; // Define your transaction ID logic
                // if (hvx->data[0] == expected_trans_id) {
                //     printf("Transaction ID matched!\n");
                // }
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

        default:
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
    // error_code = sd_ble_gap_scan_start(m_adapter, &m_scan_param, &scan_data);
    if (error_code != NRF_SUCCESS)
    {
        printf("Failed to start scanning, error: %d\n", error_code);
        sd_rpc_close(m_adapter);
        sd_rpc_adapter_delete(m_adapter);
        return 1;
    }
    
    m_scan_active = true;
    
    // Scan for up to 30 seconds
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
    }
    
    // Stop scanning
    printf("Stopping scan\n");
    error_code = sd_ble_gap_scan_stop(m_adapter);
    if (error_code != NRF_SUCCESS)
    {
        printf("Failed to stop scanning, error: %d\n", error_code);
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
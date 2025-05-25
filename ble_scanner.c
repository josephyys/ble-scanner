#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include "sd_rpc.h"
#include "ble.h"
// Add at the top:
#include <string.h>
#include "ble_gattc.h"
#include "ble_types.h"

// Serial port configuration
#define SERIAL_PORT     "/dev/ttyACM0"
#define BAUD_RATE       1000000  // 1M baud

// Add global for data to send
char g_send_data[128] = {0};


// Add global for characteristic handle (set after discovery)
static uint16_t m_char_handle = 0;

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

// Add these includes if not present
#include "ble_gattc.h"
#include "ble_gap.h"

void to_lowercase(char* str) {
    for (; *str; ++str) *str = tolower(*str);
}

// Add global variables for connection
static uint16_t m_conn_handle = 0xFFFF;
static ble_gap_addr_t m_target_addr; // Set this when you find your target device

// 1. In your adv_report handler, save the address of the device you want to connect to
void ble_evt_handler(adapter_t* adapter, ble_evt_t* p_ble_evt) {
    uint16_t evt_id = p_ble_evt->header.evt_id;
    switch (evt_id) {
       
        case BLE_GAP_EVT_ADV_REPORT: {
            const ble_gap_evt_adv_report_t* p_adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
            print_adv_report(p_adv_report);

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
            printf("Connected! Handle: 0x%04x\n", m_conn_handle);

            // Start service discovery for your target service UUID
            ble_uuid_t service_uuid = {
                .type = BLE_UUID_TYPE_BLE, // or BLE_UUID_TYPE_VENDOR_BEGIN if custom
                .uuid =  0x180D // Example: Heart Rate Service  16-bit or 128-bit UUID
            };
            sd_ble_gattc_primary_services_discover(adapter, m_conn_handle, 0x0001, &service_uuid);
            break;
        }
        // After service discovery, discover characteristics:
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP: {
            // Parse services, find your target service UUID
            // Then discover characteristics for that service
            // Example handle range (replace with values from discovery)
            ble_gattc_handle_range_t handle_range = { .start_handle = 0x000C, .end_handle = 0x0015 };
            sd_ble_gattc_characteristics_discover(adapter, m_conn_handle, &handle_range);
            break;
        }
        // After characteristic discovery, save handle and send data:
        case BLE_GATTC_EVT_CHAR_DISC_RSP: {
            // Parse for your target characteristic UUID, then:
            // Example characteristic handle (replace with value from discovery)
m_char_handle = 0x0012;

            // Prepare write parameters
            ble_gattc_write_params_t write_params = {
                .write_op = BLE_GATT_OP_WRITE_REQ,
                .flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
                .handle = m_char_handle,
                .offset = 0,
                .len = strlen(g_send_data),
                .p_value = (uint8_t*)g_send_data
            };
            uint32_t err = sd_ble_gattc_write(adapter, m_conn_handle, &write_params);
            if (err != NRF_SUCCESS) {
                printf("Write failed: %u\n", err);
            } else {
                printf("Write request sent: %s\n", g_send_data);
            }
            break;
        }
        // Optionally, handle write response:
        case BLE_GATTC_EVT_WRITE_RSP: {
            printf("Write complete!\n");
            break;
        }
        case BLE_GATTC_EVT_HVX: {
            // Notification/Indication received
            printf("Notification received!\n");
            // Parse p_ble_evt->evt.gattc_evt.params.hvx.data
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

// Helper to trim trailing whitespace
void trim_trailing_whitespace(char* str) {
    int len = strlen(str);
    while (len > 0 && (str[len-1] == ' ' || str[len-1] == '\r' || str[len-1] == '\n' || str[len-1] == '\t')) {
        str[len-1] = '\0';
        len--;
    }
}

/**
 * Parse and print advertising data
 */
void print_adv_report(const ble_gap_evt_adv_report_t* p_adv_report)
{
    // Print address and RSSI
    // printf("Device: ");
    // print_ble_address(p_adv_report->peer_addr.addr);
    // printf(", RSSI: %d", p_adv_report->rssi);
    
    // Use correct data fields for v5
    const uint8_t* p_data = p_adv_report->data;
    uint16_t data_len = p_adv_report->dlen;
    
    uint16_t pos = 0;
    char device_name[32] = {0};
    bool found_name = false;
    
    while (pos < data_len)
    {
        uint8_t field_len = p_data[pos];
        if (pos + 1 + field_len > data_len) break; // Invalid length
        
        if (field_len > 0)
        {
            uint8_t field_type = p_data[pos + 1];
            // Check for Complete Local Name (type 0x09)
            if (field_type == 0x09 && field_len > 1) {
                uint8_t name_len = field_len - 1;
                if (name_len > 30) name_len = 30;
                memcpy(device_name, &p_data[pos + 2], name_len);
                device_name[name_len] = '\0';
                trim_trailing_whitespace(device_name);
                trim_trailing_whitespace(g_target_name);
                found_name = true;
                break;
            }
        }
        pos += 1 + field_len;
    }
    if (found_name)
    {
        printf("Device: ");
        print_ble_address(p_adv_report->peer_addr.addr);
        printf(", RSSI: %d, Name: '%s'\n", p_adv_report->rssi, device_name);
    }
    // printf("\n");
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
        // case SD_RPC_LOG_INFO:
        //     printf("INFO: %s\n", msg);
        //     break;
        // default:
        //     printf("LOG: %s\n", msg);
        //     break;
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
 * Main function
 */
int main(int argc, char* argv[])
{
    uint32_t error_code;
    char* serial_port = SERIAL_PORT;
    uint32_t baud_rate = BAUD_RATE;
    
    // Allow override of serial port and target name from command line
    if (argc > 1) {
        serial_port = argv[1];
    }
    if (argc > 2) {
        strncpy(g_target_name, argv[2], sizeof(g_target_name) - 1);
        g_target_name[sizeof(g_target_name) - 1] = '\0';
                // Only once, after parsing argv:
            trim_trailing_whitespace(g_target_name);
            to_lowercase(g_target_name);

    }
    if (argc > 3) {
        strncpy(g_send_data, argv[3], sizeof(g_send_data) - 1);
        g_send_data[sizeof(g_send_data) - 1] = '\0';
    }

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
    
    // Scan for 30 seconds
    printf("Scanning for 30 seconds...\n");
    int scan_count = 0;
    while (m_scan_active && scan_count < 30)
    {
        sleep(1);
        scan_count++;
        printf("Scanning... %d/30s\n", scan_count);
        fflush(stdout);
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
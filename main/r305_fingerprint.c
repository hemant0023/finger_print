/**
 * @file r305_fingerprint.c
 * @brief Industrial-grade R305 Fingerprint Module Driver Implementation
 * @version 2.0
 * @date 2026-01-22
 */

#include "r305_fingerprint.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

static const char *TAG = "R305";

/* ==================== INTERNAL STRUCTURES ==================== */

/**
 * @brief Packet structure for protocol communication
 */
typedef struct {
    uint8_t buffer[512];
    uint16_t length;
} r305_packet_t;

/**
 * @brief Internal driver handle
 */
struct r305_handle_s {
	
    uart_port_t uart_num;
    r305_config_t config;
    r305_state_t state;
    r305_sys_params_t sys_params;
    SemaphoreHandle_t mutex;
    bool initialized;
    uint32_t last_command_time;
};

/* ==================== PACKET HELPERS ==================== */


static uint16_t calculate_checksum(const uint8_t *packet, uint16_t length) {
    uint16_t sum = 0;
    // Sum from package identifier to end of data (before checksum bytes)
    for (int i = 6; i < length - 2; i++) {
        sum += packet[i];
    }
    return sum;
}


static uint16_t build_packet(uint8_t *packet, uint32_t address, uint8_t cmd,  const uint8_t *params, uint16_t param_len) {
    // Header
    packet[0] = R305_HEADER_H;
    packet[1] = R305_HEADER_L;
    
    // Address (4 bytes, big-endian)
    packet[2] = (address >> 24) & 0xFF;
    packet[3] = (address >> 16) & 0xFF;
    packet[4] = (address >> 8) & 0xFF;
    packet[5] = address & 0xFF;
    
    
    // Package identifier
    packet[6] = R305_PKT_CMD;
    
    // Package length (instruction code + params + checksum)
    
    uint16_t pkg_len = 1 + param_len + 2;  // cmd(1) + params + checksum(2)
    packet[7] = (pkg_len >> 8) & 0xFF;
    packet[8] = pkg_len & 0xFF;
    
    // Instruction code
    packet[9] = cmd;
    
    // Parameters
    if (params && param_len > 0) {
        memcpy(&packet[10], params, param_len);
    }
    
    // Calculate and append checksum
    uint16_t total_len = 10 + param_len + 2;
    uint16_t checksum = calculate_checksum(packet, total_len);
    packet[10 + param_len] = (checksum >> 8) & 0xFF;
    packet[11 + param_len] = checksum & 0xFF;
    
    return total_len;
}

/**
 * @brief Validate received packet
 */
static bool validate_packet(const uint8_t *packet, uint16_t length){
	
    // Minimum packet size check
    if (length < 12) {
        ESP_LOGW(TAG, "Packet too short: %d bytes", length);
        return false;
    }
    
    // Header check
    if (packet[0] != R305_HEADER_H || packet[1] != R305_HEADER_L){
        ESP_LOGW(TAG, "Invalid header: 0x%02X%02X", packet[0], packet[1]);
        return false;
    }
    
    // Package identifier check (should be ACK)
    if (packet[6] != R305_PKT_ACK){
        ESP_LOGW(TAG, "Not an ACK packet: 0x%02X", packet[6]);
        return false;
    }
    
    // Checksum validation
    uint16_t received_checksum = (packet[length - 2] << 8) | packet[length - 1];
    uint16_t calculated_checksum = calculate_checksum(packet, length);
    
    if (received_checksum != calculated_checksum){
       ESP_LOGW(TAG, "Checksum mismatch: received=0x%04X, calculated=0x%04X", received_checksum, calculated_checksum);
       return false;
    }
    
    return true;
}
bool ESP_SYSTEM_LOG_DEBUG = 0;
static void dump_packet(const char *label, const uint8_t *packet, uint16_t length){
	if ( ESP_SYSTEM_LOG_DEBUG) {
    //if (esp_log_level_get(TAG) >= ESP_LOG_DEBUG) {
        printf("%s [%d bytes]: ", label, length);
        for (int i = 0; i < length; i++) {
            printf("%02X ", packet[i]);
            if ((i + 1) % 16 == 0) printf("\n                  ");
        }
        printf("\n");
    }
}

/* ==================== UART COMMUNICATION ==================== */


static r305_status_t send_packet(r305_handle_t *handle, const uint8_t *packet, uint16_t length) {
	
    dump_packet("TX", packet, length);
    
    uart_flush(handle->uart_num);
    int written = uart_write_bytes(handle->uart_num, (const char *)packet, length);
    
    if (written != length) {
        ESP_LOGE(TAG, "UART write failed: %d/%d bytes", written, length);
        return R305_STATUS_COMM_ERROR;
    }
    
    // Small delay to ensure transmission completes
    vTaskDelay(pdMS_TO_TICKS(50));
    
    return R305_STATUS_OK;
}


static r305_status_t receive_packet(r305_handle_t *handle, uint8_t *packet,  uint16_t max_length, uint16_t *received_length, uint32_t timeout_ms){
    memset(packet, 0, max_length);
    
    int length = uart_read_bytes(handle->uart_num, packet, max_length,  pdMS_TO_TICKS(timeout_ms));
    if (length <= 0) {
        ESP_LOGW(TAG, "UART read timeout or error: %d", length);
        return R305_STATUS_TIMEOUT;
    }
    
    *received_length = length;
    dump_packet("RX", packet, length);
    
    if (!validate_packet(packet, length)) {
        return R305_STATUS_COMM_ERROR;
    }
    
    return R305_STATUS_OK;
}


static r305_status_t transceive_packet(r305_handle_t *handle, const uint8_t *tx_packet, uint16_t tx_length, uint8_t *rx_packet,uint16_t rx_max_length, uint16_t *rx_length,uint32_t timeout_ms, int max_retries) {
  
    r305_status_t status = R305_STATUS_UNKNOWN_ERROR;
    
    for (int attempt = 0; attempt < max_retries; attempt++) {
        if (attempt > 0) {
            ESP_LOGW(TAG, "Retry attempt %d/%d", attempt + 1, max_retries);
            vTaskDelay(pdMS_TO_TICKS(1500));  // Wait before retry
        }
        
        // Send command
        status = send_packet(handle, tx_packet, tx_length);
        if (status != R305_STATUS_OK) {
            continue;
        }
        
        // Receive response
        status = receive_packet(handle, rx_packet, rx_max_length, rx_length, timeout_ms);
        if (status == R305_STATUS_OK) {
            return R305_STATUS_OK;
        }
    }
    
    ESP_LOGE(TAG, "Communication failed after %d attempts", max_retries);
    return status;
}

/**
 * @brief Get ACK code from response packet
 */
static r305_ack_code_t get_ack_code(const uint8_t *packet){
  
    return (r305_ack_code_t)packet[9];
}

/* ==================== COMMAND IMPLEMENTATIONS ==================== */

/**
 * @brief Generic command execution
 */
static r305_status_t execute_command(r305_handle_t *handle, uint8_t cmd, const uint8_t *params, uint16_t param_len,  uint8_t *response, uint16_t *response_len,  uint32_t timeout_ms) {
  
    uint8_t tx_packet[256];
    uint8_t rx_packet[256];
    uint16_t tx_len, rx_len;
    r305_status_t status;
    
    // Build command packet
    tx_len = build_packet(tx_packet, handle->config.device_address, cmd, params, param_len);
    
    // Transceive
    status = transceive_packet(handle, tx_packet, tx_len, rx_packet, sizeof(rx_packet), &rx_len, timeout_ms, R305_RETRY_COMM);
    if (status != R305_STATUS_OK) {
        return status;
    }
    
    // Copy response if requested
    if (response && response_len) {
        *response_len = rx_len;
        memcpy(response, rx_packet, rx_len);
    }
    
    // Check ACK code
    r305_ack_code_t ack = get_ack_code(rx_packet);
    if(ack != R305_OK){
       // ESP_LOGI(TAG, "Command 0x%02X failed: %s (0x%02X)", cmd, r305_ack_to_string(ack), ack);
        
        // Map ACK code to status
        switch (ack) {
            case R305_ERR_NO_FINGER:        return R305_STATUS_NO_FINGER;
            case R305_ERR_NO_MATCH:         return R305_STATUS_MATCH_FAIL;
            case R305_ERR_NOT_FOUND:        return R305_STATUS_NOT_FOUND;
            case R305_ERR_PACKET:           return R305_STATUS_COMM_ERROR;
            case R305_ERR_ENROLL_FAIL:      return R305_STATUS_ENROLL_FAIL;
            case R305_ERR_COMBINE_FAIL:     return R305_STATUS_ENROLL_FAIL;
            case R305_ERR_GEN_CHAR_DISORDER:
            case R305_ERR_GEN_CHAR_SMALL:
            case R305_ERR_NO_VALID_IMAGE:   return R305_STATUS_IMAGE_FAIL;
            default:                        return R305_STATUS_UNKNOWN_ERROR;
        }
    }
    
    return R305_STATUS_OK;
}

/* ==================== LOW-LEVEL OPERATIONS ==================== */

/**
 * @brief Capture fingerprint image
 */
static r305_status_t capture_image(r305_handle_t *handle){
 
 
   // ESP_LOGI(TAG, "Capturing fingerprint image...");
    return execute_command(handle, R305_CMD_GEN_IMAGE, NULL, 0, NULL, NULL,   R305_IMAGE_CAPTURE_MS);
}

/**
 * @brief Convert image to character file
 */
static r305_status_t image_to_char(r305_handle_t *handle, uint8_t buffer_id){
  
    ESP_LOGD(TAG, "Converting image to character file (buffer : [%d])...", buffer_id);
    uint8_t params[1] = { buffer_id };
    return execute_command(handle, R305_CMD_IMG_2_TZ, params, 1, NULL, NULL, R305_CMD_TIMEOUT_MS);
}

/**
 * @brief Generate template from two character files
 */
static r305_status_t generate_template(r305_handle_t *handle){
   
    ESP_LOGD(TAG, "Generating template from character files...");
    return execute_command(handle, R305_CMD_REG_MODEL, NULL, 0, NULL, NULL, R305_CMD_TIMEOUT_MS);
}

/**
 * @brief Store template to flash
 */
static r305_status_t store_template(r305_handle_t *handle, uint8_t buffer_id,  uint16_t template_id){
    
    ESP_LOGD(TAG, "Storing template to ID %d...", template_id);
    uint8_t params[3] = {
        buffer_id,
        (template_id >> 8) & 0xFF,
        template_id & 0xFF
    };
    
    return execute_command(handle, R305_CMD_STORE, params, 3, NULL, NULL, R305_CMD_TIMEOUT_MS);
    
}


static r305_status_t load_template(r305_handle_t *handle, uint8_t buffer_id,uint16_t template_id){
	
    ESP_LOGD(TAG, "Loading template ID %d to buffer %d...", template_id, buffer_id);
    uint8_t params[3] = {
        buffer_id,
        (template_id >> 8) & 0xFF,
        template_id & 0xFF
    };
    return execute_command(handle, R305_CMD_LOAD_CHAR, params, 3, NULL, NULL,  R305_CMD_TIMEOUT_MS);
}

/**
 * @brief Search fingerprint library
 */
static r305_status_t search_library(r305_handle_t *handle, uint8_t buffer_id,uint16_t start_page, uint16_t page_count,r305_search_result_t *result) {
	
    ESP_LOGI(TAG, "Searching fingerprint database...");
    uint8_t params[5] = {
        buffer_id,
        (start_page >> 8) & 0xFF,
        start_page & 0xFF,
        (page_count >> 8) & 0xFF,
        page_count & 0xFF
    };
    
    uint8_t response[32];
    uint16_t response_len;
    
    r305_status_t status = execute_command(handle, R305_CMD_SEARCH, params, 5, response, &response_len,   R305_SEARCH_TIMEOUT_MS);
    if (status == R305_STATUS_OK && result) {
        result->found = true;
        result->template_id = (response[10] << 8) | response[11];
        result->confidence = (response[12] << 8) | response[13];
        
       // ESP_LOGI(TAG, "Match found: ID=%d, Confidence=%d", result->template_id, result->confidence);
        
    } else if (status == R305_STATUS_NOT_FOUND && result){
        result->found = false;
        result->template_id = 0;
        result->confidence = 0;
    }
    
    return status;
}

/**
 * @brief Match two templates
 */
static r305_status_t match_templates(r305_handle_t *handle, uint16_t *confidence){
	
	ESP_LOGD(TAG, "Matching templates in buffer 1 and 2...");
    
    uint8_t response[32];
    uint16_t response_len;
    
    r305_status_t status = execute_command(handle, R305_CMD_MATCH, NULL, 0,response, &response_len, R305_CMD_TIMEOUT_MS);
    if (status == R305_STATUS_OK && confidence) {
        *confidence = (response[10] << 8) | response[11];
        ESP_LOGD(TAG, "Match confidence: %d", *confidence);
    }
    
    return status;
}

/* ==================== HELPER FUNCTIONS ==================== */

/**
 * @brief Fire event callback if registered
 */
static void fire_event(r305_handle_t *handle, r305_event_t *event) {
  
    if (handle->config.event_callback) {
        handle->config.event_callback(event, handle->config.callback_user_data);
    }
}

/**
 * @brief Wait for finger with timeout
 */
static r305_status_t wait_for_finger_internal(r305_handle_t *handle,  uint32_t timeout_ms) {
   
    uint32_t start_time = xTaskGetTickCount();
    uint32_t elapsed_ms = 0;
    
  //  ESP_LOGI(TAG, "Waiting for finger placement...");
    
    while (elapsed_ms < timeout_ms) {
		
        r305_status_t status = capture_image(handle);
        if (status == R305_STATUS_OK){
            ESP_LOGI(TAG, "Finger detected");
            r305_event_t event = { .type = R305_EVENT_FINGER_DETECTED };
            fire_event(handle, &event);
            return R305_STATUS_OK;
        }
        
        if (status != R305_STATUS_NO_FINGER){
           //  ESP_LOGI(TAG, "Finger not detected....");
            return status;
        }
        
        vTaskDelay(pdMS_TO_TICKS(R305_FINGER_DETECT_MS ));
        elapsed_ms = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
    }
    
    ESP_LOGW(TAG, "Finger detection timeout");
    return R305_STATUS_TIMEOUT;
}

/**
 * @brief Wait for finger removal
 */
static r305_status_t wait_for_removal_internal(r305_handle_t *handle,  uint32_t timeout_ms) {
  
    uint32_t start_time = xTaskGetTickCount();
    uint32_t elapsed_ms = 0;
    
    ESP_LOGI(TAG, "Waiting for finger removal...");
    
    while (elapsed_ms < timeout_ms) {
        r305_status_t status = capture_image(handle);
         if(status == R305_STATUS_NO_FINGER){
            ESP_LOGI(TAG, "Finger removed successful on enroll");
            r305_event_t event = { .type = R305_EVENT_FINGER_REMOVED };
            fire_event(handle, &event);
            return R305_STATUS_OK;
        }
        
        vTaskDelay(pdMS_TO_TICKS(R305_FINGER_DETECT_MS));
        elapsed_ms = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
    }
    
    ESP_LOGW(TAG, "Finger removal timeout");
    return R305_STATUS_TIMEOUT;
}

/* ==================== PUBLIC API IMPLEMENTATION ==================== */

r305_status_t r305_init(const r305_config_t *config, r305_handle_t **handle) {
	
    if (!config || !handle) {
        return R305_STATUS_INVALID_PARAM;
    }
    
    ESP_LOGI(TAG, "Initializing R305 fingerprint module...");
    
    // Allocate handle
    r305_handle_t *h = calloc(1, sizeof(r305_handle_t));
    if (!h) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return R305_STATUS_COMM_ERROR;
    }
    
    // Copy configuration
    memcpy(&h->config, config, sizeof(r305_config_t));
    h->uart_num = R305_UART_NUM;
    h->state = R305_STATE_INITIALIZING;
    
    // Create mutex
    h->mutex = xSemaphoreCreateMutex();
    if (!h->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(h);
        return R305_STATUS_COMM_ERROR;
    }
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity =    UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t err = uart_param_config(h->uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(h->mutex);
        free(h);
        return R305_STATUS_COMM_ERROR;
    }
    
    err = uart_set_pin(h->uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(h->mutex);
        free(h);
        return R305_STATUS_COMM_ERROR;
    }
    
    err = uart_driver_install(h->uart_num, R305_UART_BUF_SIZE * 2,   R305_UART_BUF_SIZE * 2, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(h->mutex);
        free(h);
        return R305_STATUS_COMM_ERROR;
    }
    
    // Wait for module power-on initialization
    vTaskDelay(pdMS_TO_TICKS(R305_POWER_ON_DELAY_MS));
    
    h->initialized = true;
    h->state = R305_STATE_IDLE;
    *handle = h;
    
    ESP_LOGI(TAG, "R305 initialization complete");
    
    r305_event_t event = { .type = R305_EVENT_INITIALIZED };
    fire_event(h, &event);
    
    return R305_STATUS_OK;
}

r305_status_t r305_deinit(r305_handle_t *handle){
   
    if (!handle){
        return R305_STATUS_INVALID_PARAM;
    }
    
    ESP_LOGI(TAG, "Deinitializing R305...");
   
    uart_driver_delete(handle->uart_num);
    vSemaphoreDelete(handle->mutex);
    free(handle);
    
    return R305_STATUS_OK;
}

r305_status_t r305_handshake(r305_handle_t *handle) {
   
    if (!handle || !handle->initialized) {
        return R305_STATUS_NOT_INITIALIZED;
    }
    
    ESP_LOGI(TAG, "Performing handshake...");
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    
    uint8_t params[1] = { 0x00 };  // Port control: check status
    r305_status_t status = execute_command(handle, R305_CMD_PORT_CONTROL, params, 1, NULL, NULL,  R305_CMD_TIMEOUT_MS);
    
    xSemaphoreGive(handle->mutex);
    
    if (status == R305_STATUS_OK) {
        ESP_LOGI(TAG, "Handshake successful");
    } else {
        ESP_LOGE(TAG, "Handshake failed");
    }
    
    return status;
}

r305_status_t r305_read_sys_params(r305_handle_t *handle, r305_sys_params_t *params) {
   
    if (!handle || !handle->initialized || !params) {
        return R305_STATUS_INVALID_PARAM;
    }
    
    ESP_LOGI(TAG, "Reading system parameters...");
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    
    uint8_t response[64];
    uint16_t response_len;
    
    r305_status_t status = execute_command(handle, R305_CMD_READ_SYS_PARA, NULL, 0, response, &response_len, R305_CMD_TIMEOUT_MS);
    
    if (status == R305_STATUS_OK) {
        params->status_register = (response[10] << 8) | response[11];
        params->system_id = (response[12] << 8) | response[13];
        params->library_size = (response[14] << 8) | response[15];
        params->security_level = (response[16] << 8) | response[17];
        params->device_address = (response[18] << 24) | (response[19] << 16) |  (response[20] << 8) | response[21];
        params->packet_size_code = (response[22] << 8) | response[23];
        
        // Convert packet size code to bytes
        switch (params->packet_size_code) {
            case 0: params->packet_size_bytes = 32; break;
            case 1: params->packet_size_bytes = 64; break;
            case 2: params->packet_size_bytes = 128; break;
            case 3: params->packet_size_bytes = 256; break;
            default: params->packet_size_bytes = 128; break;
        }
        
        uint16_t baud_multiplier = (response[24] << 8) | response[25];
        params->baud_rate = baud_multiplier * 9600;
        
        // Cache in handle
        memcpy(&handle->sys_params, params, sizeof(r305_sys_params_t));
        
      /*  ESP_LOGI(TAG, "=== System Parameters ===");
        ESP_LOGI(TAG, "System ID: 0x%04X", params->system_id);
        ESP_LOGI(TAG, "Library Size: %d", params->library_size);
        ESP_LOGI(TAG, "Security Level: %d", params->security_level);
        ESP_LOGI(TAG, "Device Address: 0x%08lX", params->device_address);
        ESP_LOGI(TAG, "Packet Size: %d bytes", params->packet_size_bytes);
        ESP_LOGI(TAG, "Baud Rate: %lu bps", params->baud_rate);
        ESP_LOGI(TAG, "========================");*/
    }
    
    xSemaphoreGive(handle->mutex);
    
    return status;
}

r305_status_t r305_read_info_page(r305_handle_t *handle, uint8_t *info_buf,size_t buf_size, size_t *bytes_read) {
    
    if (!handle || !handle->initialized || !info_buf || !bytes_read) {
        return R305_STATUS_INVALID_PARAM;
    }
    
    ESP_LOGI(TAG, "Reading information page...");
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    
    uint8_t response[256];
    uint16_t response_len;
    
    r305_status_t status = execute_command(handle, R305_CMD_READ_INF_PAGE,  NULL, 0, response, &response_len,  R305_CMD_TIMEOUT_MS);
    if (status == R305_STATUS_OK) {
        uint16_t payload_len = ((response[7] << 8) | response[8]) - 2;
        *bytes_read = (payload_len < buf_size) ? payload_len : buf_size;
        
        memcpy(info_buf, &response[10], *bytes_read);
        
       // ESP_LOGI(TAG, "Info page read: %d bytes", *bytes_read);
    }
    
    xSemaphoreGive(handle->mutex);
    
    return status;
}

r305_status_t r305_get_template_count(r305_handle_t *handle, uint16_t *count) {
    
    if(!handle || !handle->initialized || !count) {
        return R305_STATUS_INVALID_PARAM;
    }
    
    ESP_LOGI(TAG, "Getting template count...");
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    
    uint8_t response[32];
    uint16_t response_len;
    
    r305_status_t status = execute_command(handle, R305_CMD_TEMPLATE_NUM, NULL, 0, response, &response_len,   R305_CMD_TIMEOUT_MS);
    if (status == R305_STATUS_OK) {
        *count = (response[10] << 8) | response[11];
       // ESP_LOGI(TAG, "Template count: %d", *count);
    }
    
    xSemaphoreGive(handle->mutex);
    
    return status;
}

r305_status_t r305_enroll_finger(r305_handle_t *handle, uint16_t template_id, uint32_t timeout_ms) {
  
    if (!handle || !handle->initialized) {
        return R305_STATUS_NOT_INITIALIZED;
    }
    
    if (template_id > R305_MAX_TEMPLATE_ID) {
        ESP_LOGE(TAG, "Invalid template ID: %d", template_id);
        return R305_STATUS_INVALID_ID;
    }
    
    ESP_LOGI(TAG, "========== ENROLLMENT START ==========");
    ESP_LOGI(TAG, "Template ID: %d", template_id);
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->state = R305_STATE_ENROLLING;
    
    r305_status_t final_status = R305_STATUS_ENROLL_FAIL;
   
    uint32_t enroll_timeout = (timeout_ms > 0) ? timeout_ms : R305_ENROLL_TIMEOUT_MS;
    
    for (int attempt = 0; attempt < R305_MAX_ENROLL_ATTEMPTS; attempt++) {
    
     if (attempt > 0){
            ESP_LOGW(TAG, "Enrollment re-attempt %d/%d", attempt + 1, R305_MAX_ENROLL_ATTEMPTS);
        }
        
        // === STEP 1: First finger placement ===
        r305_enroll_progress_t progress = {
            .state = R305_ENROLL_WAIT_FINGER_1,
            .template_id = template_id,
            .attempt = attempt + 1,
            .max_attempts = R305_MAX_ENROLL_ATTEMPTS
        };
        
        r305_event_t event = { 
            .type = R305_EVENT_ENROLL_PROGRESS,
            .data.enroll_progress = progress
        };
        fire_event(handle, &event);
        
        ESP_LOGI(TAG, "Step 1: Place finger on sensor...");
        r305_status_t status = wait_for_finger_internal(handle, enroll_timeout);
        if (status != R305_STATUS_OK) {
            ESP_LOGE(TAG, "Failed to detect first finger");
            continue;
        }
        
        // Convert first image
        progress.state = R305_ENROLL_PROCESS_1;
        event.data.enroll_progress = progress;
        fire_event(handle, &event);
        
        status = image_to_char(handle, R305_BUFFER_1);
        if (status != R305_STATUS_OK) {
            ESP_LOGE(TAG, "Failed to process first image");
            continue;
        }ESP_LOGI(TAG, "First image processed successfully");
        
        // === STEP 2: Finger removal ===
        progress.state = R305_ENROLL_REMOVE_FINGER;
        event.data.enroll_progress = progress;
        fire_event(handle, &event);
        
        ESP_LOGI(TAG, "Step 2: Remove finger from sensor...");
       
        vTaskDelay(pdMS_TO_TICKS(100));  // Brief pause
        
        status = wait_for_removal_internal(handle, R305_FINGER_REMOVE_WAIT_MS);
        if (status != R305_STATUS_OK) {
            ESP_LOGW(TAG, "Finger removal timeout, continuing anyway...");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));  // Pause before second scan
        
        // === STEP 3: Second finger placement ===
        progress.state = R305_ENROLL_WAIT_FINGER_2;
        event.data.enroll_progress = progress;
        fire_event(handle, &event);
        
        ESP_LOGI(TAG, "Step 3: Place same finger again...");
        status = wait_for_finger_internal(handle, enroll_timeout);
        if (status != R305_STATUS_OK) {
            ESP_LOGE(TAG, "Failed to detect second finger");
            continue;
        }
        
        // Convert second image
        progress.state = R305_ENROLL_PROCESS_2;
        event.data.enroll_progress = progress;
        fire_event(handle, &event);
        
        status = image_to_char(handle, R305_BUFFER_2);
        if (status != R305_STATUS_OK) {
            ESP_LOGE(TAG, "Failed to process second image");
            continue;
        }ESP_LOGI(TAG, "Second image processed successfully");
        
  
        /// match pending
        
        
      
        // === STEP 4: Generate template ===
        progress.state = R305_ENROLL_MERGE;
        event.data.enroll_progress = progress;
        fire_event(handle, &event);
        
        ESP_LOGI(TAG, "Step 4: Generating template...");
      
        status = generate_template(handle);
        if (status != R305_STATUS_OK){
            ESP_LOGE(TAG, "Template generation failed (fingerprints don't match)");
            continue;
        }
        
        // === STEP 5: Store template ===
        progress.state = R305_ENROLL_STORE;
        event.data.enroll_progress = progress;
        fire_event(handle, &event);
        
        ESP_LOGI(TAG, "Step 5: Storing template to ID %d...", template_id);
        status = store_template(handle, R305_BUFFER_1, template_id);
        if (status != R305_STATUS_OK) {
            ESP_LOGE(TAG, "Failed to store template");
            continue;
        }
        
        // Success!
        ESP_LOGI(TAG, "========== ENROLLMENT SUCCESS ==========");
        ESP_LOGI(TAG, "Template ID %d enrolled successfully", template_id);
        
        event.type = R305_EVENT_ENROLL_SUCCESS;
        event.data.template_id = template_id;
        fire_event(handle, &event);
        
        final_status = R305_STATUS_OK;
        break;
    }
    
    if (final_status != R305_STATUS_OK) {
        ESP_LOGE(TAG, "========== ENROLLMENT FAILED ==========");
        ESP_LOGE(TAG, "Failed after %d attempts", R305_MAX_ENROLL_ATTEMPTS);
        
        r305_event_t event = { 
            .type = R305_EVENT_ENROLL_FAILED,
            .data.error_code = final_status
        };
        fire_event(handle, &event);
    }
    
    handle->state = R305_STATE_IDLE;
    xSemaphoreGive(handle->mutex);
    
    return final_status;
}

r305_status_t r305_search_finger(r305_handle_t *handle, r305_search_result_t *result, uint32_t timeout_ms) {
  
    if (!handle || !handle->initialized || !result) {
        return R305_STATUS_INVALID_PARAM;
    }
    
    ESP_LOGI(TAG, "========== FINGERPRINT SEARCH Place finger on sensor... ==========");
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->state = R305_STATE_SEARCHING;
    
    uint32_t search_timeout = (timeout_ms > 0) ? timeout_ms : R305_SEARCH_TIMEOUT_MS;
    
   
    r305_status_t status = wait_for_finger_internal(handle, search_timeout);
    if (status != R305_STATUS_OK) {
        ESP_LOGW(TAG, "No finger detected AFTER search_timeout");
        handle->state = R305_STATE_IDLE;
        xSemaphoreGive(handle->mutex);
        return status;
    }
    
    // Convert image to character file
    status = image_to_char(handle, R305_BUFFER_1);
    if (status != R305_STATUS_OK){
        ESP_LOGE(TAG, "Failed to process fingerprint image");
        handle->state = R305_STATE_IDLE;
        xSemaphoreGive(handle->mutex);
        return status;
    }
    
    // Search library
    status = search_library(handle, R305_BUFFER_1, 0, R305_MAX_TEMPLATES, result);
    if (status == R305_STATUS_OK) {
       
        ESP_LOGI(TAG, "========== MATCH FOUND ==========: %d",result->found);
        ESP_LOGI(TAG, "Template ID: %d", result->template_id);
        ESP_LOGI(TAG, "Confidence: %d (%s)", result->confidence, r305_confidence_to_string(result->confidence));
        
        r305_event_t event = {
            .type = R305_EVENT_SEARCH_SUCCESS,
            .data.search_result = *result,
           // .data.template_id = result->template_id
        };
        fire_event(handle, &event);
    } else {
       // ESP_LOGI(TAG, "========== NO MATCH FOUND ==========");
        
        r305_event_t event = {
            .type = R305_EVENT_SEARCH_FAILED,
            .data.error_code = status
        };
        fire_event(handle, &event);
    }
    
    handle->state = R305_STATE_IDLE;
    xSemaphoreGive(handle->mutex);
    
    return status;
}

r305_status_t r305_match_finger(r305_handle_t *handle, uint16_t template_id, uint16_t *confidence, uint32_t timeout_ms ){
    
    if (!handle || !handle->initialized || !confidence){
        return R305_STATUS_INVALID_PARAM;
    }
    
    if (template_id > R305_MAX_TEMPLATE_ID) {
        return R305_STATUS_INVALID_ID;
    }
    
    ESP_LOGI(TAG, "========== 1:1 MATCH ==========");
    ESP_LOGI(TAG, "Matching against template ID: %d", template_id);
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->state = R305_STATE_MATCHING;
    
    uint32_t match_timeout = (timeout_ms > 0) ? timeout_ms : R305_SEARCH_TIMEOUT_MS;
    
    // Load template to buffer 2
    r305_status_t status = load_template(handle, R305_BUFFER_2, template_id);
    if (status != R305_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to load template %d", template_id);
        handle->state = R305_STATE_IDLE;
        xSemaphoreGive(handle->mutex);
        return status;
    }
    
    // Wait for finger
    ESP_LOGI(TAG, "Place finger on sensor...");
    status = wait_for_finger_internal(handle, match_timeout);
    if (status != R305_STATUS_OK) {
        ESP_LOGE(TAG, "No finger detected");
        handle->state = R305_STATE_IDLE;
        xSemaphoreGive(handle->mutex);
        return status;
    }
    
    // Convert image
    status = image_to_char(handle, R305_BUFFER_1);
    if (status != R305_STATUS_OK) {
        ESP_LOGE(TAG, "Failed to process fingerprint image");
        handle->state = R305_STATE_IDLE;
        xSemaphoreGive(handle->mutex);
        return status;
    }
    
    // Match templates
    status = match_templates(handle, confidence);
    
    if (status == R305_STATUS_OK) {
        ESP_LOGI(TAG, "========== MATCH SUCCESS ==========");
        ESP_LOGI(TAG, "Confidence: %d (%s)", *confidence,
                 r305_confidence_to_string(*confidence));
        
        r305_event_t event = {
            .type = R305_EVENT_MATCH_SUCCESS,
            .data.template_id = template_id
        };
        fire_event(handle, &event);
    } else {
        ESP_LOGI(TAG, "========== MATCH FAILED ==========");
        
        r305_event_t event = {
            .type = R305_EVENT_MATCH_FAILED,
            .data.error_code = status
        };
        fire_event(handle, &event);
    }
    
    handle->state = R305_STATE_IDLE;
    xSemaphoreGive(handle->mutex);
    
    return status;
}

r305_status_t r305_delete_template(r305_handle_t *handle, uint16_t template_id) {
   
    return r305_delete_templates(handle, template_id, 1);
}

r305_status_t r305_delete_templates(r305_handle_t *handle, uint16_t start_id, uint16_t count){
   
    if (!handle || !handle->initialized) {
        return R305_STATUS_NOT_INITIALIZED;
    }
    
    if (start_id > R305_MAX_TEMPLATE_ID || count == 0) {
        return R305_STATUS_INVALID_PARAM;
    }
    
    ESP_LOGI(TAG, "Deleting templates %d to %d...", start_id, start_id + count - 1);
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->state = R305_STATE_DELETING;
    
    uint8_t params[4] = {
        (start_id >> 8) & 0xFF,
        start_id & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF
    };
    
    r305_status_t status = execute_command(handle, R305_CMD_DELETE_CHAR, params, 4, NULL, NULL, R305_CMD_TIMEOUT_MS);
    if(status == R305_STATUS_OK) {
      //  ESP_LOGI(TAG, "Templates deleted successfully");
    } else {
        ESP_LOGE(TAG, "Failed to delete templates");
    }
    
    handle->state = R305_STATE_IDLE;
    xSemaphoreGive(handle->mutex);
    
    return status;
}

r305_status_t r305_clear_database(r305_handle_t *handle) {
  
    if (!handle || !handle->initialized) {
        return R305_STATUS_NOT_INITIALIZED;
    }
    
    ESP_LOGW(TAG, "Clearing entire fingerprint database...");
    
    xSemaphoreTake(handle->mutex, portMAX_DELAY);
    handle->state = R305_STATE_DELETING;
    
    r305_status_t status = execute_command(handle, R305_CMD_EMPTY, NULL, 0, NULL, NULL, R305_CMD_TIMEOUT_MS * 2);
    
    if (status == R305_STATUS_OK) {
      //  ESP_LOGI(TAG, "Database cleared successfully");
    } else {
        ESP_LOGE(TAG, "Failed to clear database");
    }
    
    handle->state = R305_STATE_IDLE;
    xSemaphoreGive(handle->mutex);
    
    return status;
}

r305_status_t r305_is_finger_present(r305_handle_t *handle, bool *present){
    
    if (!handle || !handle->initialized || !present) {
        return R305_STATUS_INVALID_PARAM;
    }
    
    r305_status_t status = capture_image(handle);
    *present = (status == R305_STATUS_OK);
    
    return R305_STATUS_OK;
}

r305_status_t r305_wait_for_finger(r305_handle_t *handle, uint32_t timeout_ms) {
  
    if (!handle || !handle->initialized) {
        return R305_STATUS_NOT_INITIALIZED;
    }
    
    return wait_for_finger_internal(handle, timeout_ms);
}

r305_status_t r305_wait_finger_removed(r305_handle_t *handle, uint32_t timeout_ms) {
  
    if (!handle || !handle->initialized) {
        return R305_STATUS_NOT_INITIALIZED;
    }
    
    return wait_for_removal_internal(handle, timeout_ms);
}

r305_state_t r305_get_state(r305_handle_t *handle) {
   
    if (!handle) {
        return R305_STATE_UNINITIALIZED;
    }
    return handle->state;
}

/* ==================== STRING UTILITIES ==================== */

const char* r305_status_to_string(r305_status_t status){
    
    switch (status) {
        case R305_STATUS_OK:                return "Success";
        case R305_STATUS_TIMEOUT:           return "Timeout";
        case R305_STATUS_COMM_ERROR:        return "Communication error";
        case R305_STATUS_INVALID_PARAM:     return "Invalid parameter";
        case R305_STATUS_NOT_INITIALIZED:   return "Not initialized";
        case R305_STATUS_BUSY:              return "Module busy";
        case R305_STATUS_INVALID_ID:        return "Invalid template ID";
        case R305_STATUS_DATABASE_FULL:     return "Database full";
        case R305_STATUS_NO_FINGER:         return "No finger detected";
        case R305_STATUS_IMAGE_FAIL:        return "Image capture failed";
        case R305_STATUS_MATCH_FAIL:        return "Fingerprint doesn't match";
        case R305_STATUS_NOT_FOUND:         return "Fingerprint not found";
        case R305_STATUS_ENROLL_FAIL:       return "Enrollment failed";
        case R305_STATUS_DELETE_FAIL:       return "Deletion failed";
        default:                            return "Unknown error";
    }
}

const char* r305_ack_to_string(r305_ack_code_t ack_code){
  
    switch (ack_code){
        case R305_OK:                       return "Success";
        case R305_ERR_PACKET:               return "Packet error";
        case R305_ERR_NO_FINGER:            return "No finger";
        case R305_ERR_ENROLL_FAIL:          return "Enrollment failed";
        case R305_ERR_GEN_CHAR_DISORDER:    return "Image too disorderly";
        case R305_ERR_GEN_CHAR_SMALL:       return "Image too small";
        case R305_ERR_NO_MATCH:             return "No match";
        case R305_ERR_NOT_FOUND:            return "Not found";
        case R305_ERR_COMBINE_FAIL:         return "Combine failed";
        case R305_ERR_ADDR_BEYOND:          return "Address out of range";
        case R305_ERR_READ_TEMPLATE:        return "Template read error";
        case R305_ERR_UPLOAD_TEMPLATE:      return "Template upload error";
        case R305_ERR_RECV_DATA_PACKAGE:    return "Data package receive error";
        case R305_ERR_UPLOAD_IMAGE:         return "Image upload error";
        case R305_ERR_DELETE_TEMPLATE:      return "Template delete error";
        case R305_ERR_CLEAR_LIBRARY:        return "Library clear error";
        case R305_ERR_WRONG_PASSWORD:       return "Wrong password";
        case R305_ERR_NO_VALID_IMAGE:       return "No valid image";
        case R305_ERR_WRITE_FLASH:          return "Flash write error";
        case R305_ERR_NO_DEFINITION:        return "Undefined error";
        case R305_ERR_INVALID_REG:          return "Invalid register";
        case R305_ERR_INCORRECT_CONFIG:     return "Incorrect configuration";
        case R305_ERR_WRONG_NOTEPAD_PAGE:   return "Wrong notepad page";
        case R305_ERR_PORT_OPERATION:       return "Port operation error";
        default:                            return "Unknown ACK code";
    
    }
}

void r305_parse_status_register(uint16_t status_reg, r305_status_bits_t *bits){
    
    if (!bits) return;
    
    bits->busy = (status_reg & 0x01) != 0;
    bits->finger_matched = (status_reg & 0x02) != 0;
    bits->password_verified = (status_reg & 0x04) != 0;
    bits->image_buffer_valid = (status_reg & 0x08) != 0;

}

const char* r305_confidence_to_string(uint16_t confidence) {
    if (confidence >= R305_CONF_THRESHOLD_HIGH) {
        return "Excellent";
    } else if (confidence >= R305_CONF_THRESHOLD_MEDIUM) {
        return "Good";
    } else if (confidence >= R305_CONF_THRESHOLD_LOW) {
        return "Fair";
    } else {
        return "Poor";
    }
}
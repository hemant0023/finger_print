/**
 * @file r305_fingerprint.h
 * @brief Industrial-grade R305 Fingerprint Module Driver for ESP32
 * @version 2.0
 * @date 2026-01-22
 * 
 * Complete fingerprint authentication system with:
 * - Comprehensive error handling
 * - State machine management
 * - Retry mechanisms
 * - Timeout handling
 * - Event-driven architecture
 */

#ifndef R305_FINGERPRINT_H

#define R305_FINGERPRINT_H


#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ==================== CONFIGURATION ==================== */

#define R305_UART_NUM               UART_NUM_2
#define R305_TX_PIN                 GPIO_NUM_16
#define R305_RX_PIN                 GPIO_NUM_17
#define R305_UART_BAUD_RATE         57600
#define R305_UART_BUF_SIZE          1024

/* Timing Configuration */
#define R305_POWER_ON_DELAY_MS      500     // Module initialization time
#define R305_CMD_TIMEOUT_MS         1000    // Standard command timeout
#define R305_SEARCH_TIMEOUT_MS      2000    // Search operation timeout
#define R305_ENROLL_TIMEOUT_MS      4000    // Enrollment timeout
#define R305_IMAGE_CAPTURE_MS       500     // Image capture timeout
#define R305_FINGER_DETECT_MS       100     // Finger detection poll interval
#define R305_FINGER_REMOVE_WAIT_MS  5000    // Wait time between enrollment scans

/* Retry Configuration */
#define R305_RETRY_COMM             3       // Communication retries
#define R305_RETRY_IMAGE_CAPTURE    5       // Image capture retries
#define R305_RETRY_ENROLL_STEP      3       // Per-step enrollment retries
#define R305_MAX_ENROLL_ATTEMPTS    3       // Total enrollment attempts

/* Capacity Configuration */
#define R305_MAX_TEMPLATES          1000    // Maximum fingerprints
#define R305_MIN_TEMPLATE_ID        0
#define R305_MAX_TEMPLATE_ID        (R305_MAX_TEMPLATES - 1)

/* Quality Thresholds */
#define R305_CONF_THRESHOLD_LOW     40      // Minimum acceptable confidence
#define R305_CONF_THRESHOLD_MEDIUM  60      // Good confidence
#define R305_CONF_THRESHOLD_HIGH    80      // Excellent confidence

/* ==================== PROTOCOL DEFINITIONS ==================== */

/* Packet Structure */
#define R305_HEADER_H               0xEF
#define R305_HEADER_L               0x01
#define R305_PKT_CMD                0x01
#define R305_PKT_DATA               0x02
#define R305_PKT_ACK                0x07
#define R305_PKT_END_DATA           0x08

/* Default Address */
#define R305_DEFAULT_ADDR           0xFFFFFFFF

/* Instruction Codes */
#define R305_CMD_GEN_IMAGE          0x01    // Capture fingerprint image
#define R305_CMD_IMG_2_TZ           0x02    // Generate character file from image
#define R305_CMD_MATCH              0x03    // Match two character files
#define R305_CMD_SEARCH             0x04    // Search fingerprint library
#define R305_CMD_REG_MODEL          0x05    // Generate template from char files
#define R305_CMD_STORE              0x06    // Store template to flash
#define R305_CMD_LOAD_CHAR          0x07    // Load template from flash
#define R305_CMD_UP_CHAR            0x08    // Upload character/template
#define R305_CMD_DOWN_CHAR          0x09    // Download character/template
#define R305_CMD_UP_IMAGE           0x0A    // Upload image
#define R305_CMD_DOWN_IMAGE         0x0B    // Download image
#define R305_CMD_DELETE_CHAR        0x0C    // Delete templates
#define R305_CMD_EMPTY              0x0D    // Clear database
#define R305_CMD_SET_SYS_PARA       0x0E    // Set system parameter
#define R305_CMD_READ_SYS_PARA      0x0F    // Read system parameters
#define R305_CMD_SET_PWD            0x12    // Set password
#define R305_CMD_VFY_PWD            0x13    // Verify password
#define R305_CMD_GET_RANDOM         0x14    // Get random number
#define R305_CMD_SET_ADDR           0x15    // Set module address
#define R305_CMD_READ_INF_PAGE      0x16    // Read information page
#define R305_CMD_PORT_CONTROL       0x17    // Control port on/off
#define R305_CMD_WRITE_NOTEPAD      0x18    // Write to notepad
#define R305_CMD_READ_NOTEPAD       0x19    // Read from notepad
#define R305_CMD_TEMPLATE_NUM       0x1D    // Get template count

/* Buffer IDs */
#define R305_BUFFER_1               0x01
#define R305_BUFFER_2               0x02

/* ==================== ACKNOWLEDGEMENT CODES ==================== */

typedef enum {
    R305_OK                     = 0x00,  // Command execution complete
    R305_ERR_PACKET             = 0x01,  // Error receiving data package
    R305_ERR_NO_FINGER          = 0x02,  // No finger on sensor
    R305_ERR_ENROLL_FAIL        = 0x03,  // Failed to enroll finger
    R305_ERR_GEN_CHAR_DISORDER  = 0x06,  // Too disorderly fingerprint image
    R305_ERR_GEN_CHAR_SMALL     = 0x07,  // Too small fingerprint or lack of character points
    R305_ERR_NO_MATCH           = 0x08,  // Fingers don't match
    R305_ERR_NOT_FOUND          = 0x09,  // Failed to find matching finger
    R305_ERR_COMBINE_FAIL       = 0x0A,  // Failed to combine character files
    R305_ERR_ADDR_BEYOND        = 0x0B,  // Address code beyond range
    R305_ERR_READ_TEMPLATE      = 0x0C,  // Error reading template or invalid template
    R305_ERR_UPLOAD_TEMPLATE    = 0x0D,  // Error uploading template
    R305_ERR_RECV_DATA_PACKAGE  = 0x0E,  // Module can't receive data packages
    R305_ERR_UPLOAD_IMAGE       = 0x0F,  // Error uploading image
    R305_ERR_DELETE_TEMPLATE    = 0x10,  // Failed to delete template
    R305_ERR_CLEAR_LIBRARY      = 0x11,  // Failed to clear finger library
    R305_ERR_WRONG_PASSWORD     = 0x13,  // Wrong password
    R305_ERR_NO_VALID_IMAGE     = 0x15,  // No valid primary image
    R305_ERR_WRITE_FLASH        = 0x18,  // Error writing flash
    R305_ERR_NO_DEFINITION      = 0x19,  // No definition error
    R305_ERR_INVALID_REG        = 0x1A,  // Invalid register number
    R305_ERR_INCORRECT_CONFIG   = 0x1B,  // Incorrect register configuration
    R305_ERR_WRONG_NOTEPAD_PAGE = 0x1C,  // Wrong notepad page number
    R305_ERR_PORT_OPERATION     = 0x1D,  // Failed to operate communication port
} r305_ack_code_t;

/* ==================== DRIVER STATUS CODES ==================== */

typedef enum {
    R305_STATUS_OK              = 0,     // Success
    R305_STATUS_TIMEOUT         = -1,    // Operation timeout
    R305_STATUS_COMM_ERROR      = -2,    // Communication error
    R305_STATUS_INVALID_PARAM   = -3,    // Invalid parameter
    R305_STATUS_NOT_INITIALIZED = -4,    // Driver not initialized
    R305_STATUS_BUSY            = -5,    // Module busy
    R305_STATUS_INVALID_ID      = -6,    // Invalid template ID
    R305_STATUS_DATABASE_FULL   = -7,    // Database full
    R305_STATUS_NO_FINGER       = -8,    // No finger detected
    R305_STATUS_IMAGE_FAIL      = -9,    // Image capture failed
    R305_STATUS_MATCH_FAIL      = -10,   // Fingerprint doesn't match
    R305_STATUS_NOT_FOUND       = -11,   // Fingerprint not found
    R305_STATUS_ENROLL_FAIL     = -12,   // Enrollment failed
    R305_STATUS_DELETE_FAIL     = -13,   // Deletion failed
    R305_STATUS_UNKNOWN_ERROR   = -99,   // Unknown error
} r305_status_t;

/* ==================== SYSTEM STATES ==================== */

typedef enum {
    R305_STATE_UNINITIALIZED,
    R305_STATE_INITIALIZING,
    R305_STATE_IDLE,
    R305_STATE_ENROLLING,
    R305_STATE_MATCHING,
    R305_STATE_SEARCHING,
    R305_STATE_DELETING,
    R305_STATE_ERROR,
} r305_state_t;

/* ==================== ENROLLMENT STATES ==================== */

typedef enum {
    R305_ENROLL_WAIT_FINGER_1,      // Waiting for first finger placement
    R305_ENROLL_CAPTURE_1,          // Capturing first image
    R305_ENROLL_PROCESS_1,          // Processing first image
    R305_ENROLL_REMOVE_FINGER,      // Waiting for finger removal
    R305_ENROLL_WAIT_FINGER_2,      // Waiting for second finger placement
    R305_ENROLL_CAPTURE_2,          // Capturing second image
    R305_ENROLL_PROCESS_2,          // Processing second image
    R305_ENROLL_MERGE,              // Merging templates
    R305_ENROLL_STORE,              // Storing template
    R305_ENROLL_COMPLETE,           // Enrollment complete
    R305_ENROLL_FAILED,             // Enrollment failed
} r305_enroll_state_t;

/* ==================== SECURITY LEVELS ==================== */

typedef enum {
    R305_SECURITY_LEVEL_1 = 1,      // Lowest (Highest FAR, Lowest FRR)
    R305_SECURITY_LEVEL_2 = 2,
    R305_SECURITY_LEVEL_3 = 3,
    R305_SECURITY_LEVEL_4 = 4,
    R305_SECURITY_LEVEL_5 = 5,      // Highest (Lowest FAR, Highest FRR)
} r305_security_level_t;

/* ==================== PACKET SIZE CODES ==================== */

typedef enum {
    R305_PACKET_SIZE_32  = 0,       // 32 bytes
    R305_PACKET_SIZE_64  = 1,       // 64 bytes
    R305_PACKET_SIZE_128 = 2,       // 128 bytes
    R305_PACKET_SIZE_256 = 3,       // 256 bytes
} r305_packet_size_t;

/* ==================== DATA STRUCTURES ==================== */

/**
 * @brief System parameters structure
 */
typedef struct {
    uint16_t status_register;       // System status register
    uint16_t system_id;             // System identifier (0x0009)
    uint16_t library_size;          // Fingerprint library capacity
    uint16_t security_level;        // Security level (1-5)
    uint32_t device_address;        // 32-bit device address
    uint16_t packet_size_code;      // Data packet size code (0-3)
    uint16_t packet_size_bytes;     // Actual packet size in bytes
    uint32_t baud_rate;             // Baud rate in bps
} r305_sys_params_t;

/**
 * @brief Status register bits
 */
typedef struct {
    bool busy;                      // System executing commands
    bool finger_matched;            // Matching finger found
    bool password_verified;         // Password verified
    bool image_buffer_valid;        // Image buffer contains valid image
} r305_status_bits_t;

/**
 * @brief Search result structure
 */
typedef struct {
    bool found;                     // Match found
    uint16_t template_id;           // Matched template ID
    uint16_t confidence;            // Match confidence score (0-65535)
} r305_search_result_t;

/**
 * @brief Enrollment progress callback data
 */
typedef struct {
    r305_enroll_state_t state;      // Current enrollment state
    uint16_t template_id;           // Template ID being enrolled
    uint8_t attempt;                // Current attempt number
    uint8_t max_attempts;           // Maximum attempts allowed
} r305_enroll_progress_t;

/**
 * @brief Event types
 */
typedef enum {
    R305_EVENT_INITIALIZED,
    R305_EVENT_FINGER_DETECTED,
    R305_EVENT_FINGER_REMOVED,
    R305_EVENT_IMAGE_CAPTURED,
    R305_EVENT_ENROLL_PROGRESS,
    R305_EVENT_ENROLL_SUCCESS,
    R305_EVENT_ENROLL_FAILED,
    R305_EVENT_MATCH_SUCCESS,
    R305_EVENT_MATCH_FAILED,
    R305_EVENT_SEARCH_SUCCESS,
    R305_EVENT_SEARCH_FAILED,
    R305_EVENT_ERROR,
} r305_event_type_t;

/**
 * @brief Event structure
 */
typedef struct {
    r305_event_type_t type;
    union {
        r305_enroll_progress_t enroll_progress;
        r305_search_result_t search_result;
        r305_status_t error_code;
        uint16_t template_id;
    } data;
} r305_event_t;

/**
 * @brief Event callback function type
 */
typedef void (*r305_event_callback_t)(r305_event_t *event, void *user_data);

/**
 * @brief Configuration structure
 */
typedef struct {
	
    gpio_num_t tx_pin;              // UART TX pin
    gpio_num_t rx_pin;              // UART RX pin
    uint32_t baud_rate;             // UART baud rate
    r305_security_level_t security_level;  // Matching security level
    uint32_t device_address;        // Module address
    r305_event_callback_t event_callback;  // Event callback function
    void *callback_user_data;       // User data for callback
    
} r305_config_t;

/**
 * @brief Driver handle structure (opaque)
 */
typedef struct r305_handle_s r305_handle_t;

/* ==================== PUBLIC API ==================== */

/**
 * @brief Initialize R305 fingerprint module
 * 
 * @param config Configuration structure
 * @param handle Pointer to store driver handle
 * @return r305_status_t Status code
 */
r305_status_t r305_init(const r305_config_t *config, r305_handle_t **handle);

/**
 * @brief Deinitialize and cleanup
 * 
 * @param handle Driver handle
 * @return r305_status_t Status code
 */
r305_status_t r305_deinit(r305_handle_t *handle);

/**
 * @brief Verify module communication (handshake)
 * 
 * @param handle Driver handle
 * @return r305_status_t Status code
 */
r305_status_t r305_handshake(r305_handle_t *handle);

/**
 * @brief Read system parameters
 * 
 * @param handle Driver handle
 * @param params Pointer to store parameters
 * @return r305_status_t Status code
 */
r305_status_t r305_read_sys_params(r305_handle_t *handle, r305_sys_params_t *params);

/**
 * @brief Read module information page
 * 
 * @param handle Driver handle
 * @param info_buf Buffer to store info (min 256 bytes)
 * @param buf_size Buffer size
 * @param bytes_read Pointer to store bytes read
 * @return r305_status_t Status code
 */
r305_status_t r305_read_info_page(r305_handle_t *handle, uint8_t *info_buf,  size_t buf_size, size_t *bytes_read);

/**
 * @brief Get enrolled template count
 * 
 * @param handle Driver handle
 * @param count Pointer to store count
 * @return r305_status_t Status code
 */
r305_status_t r305_get_template_count(r305_handle_t *handle, uint16_t *count);

/**
 * @brief Enroll new fingerprint
 * 
 * @param handle Driver handle
 * @param template_id Template ID to assign (0-999)
 * @param timeout_ms Maximum time to wait for enrollment (0 = default)
 * @return r305_status_t Status code
 */
r305_status_t r305_enroll_finger(r305_handle_t *handle, uint16_t template_id,   uint32_t timeout_ms);

/**
 * @brief Search for matching fingerprint in database
 * 
 * @param handle Driver handle
 * @param result Pointer to store search result
 * @param timeout_ms Finger detection timeout (0 = default)
 * @return r305_status_t Status code
 */
r305_status_t r305_search_finger(r305_handle_t *handle, r305_search_result_t *result,   uint32_t timeout_ms);

/**
 * @brief Match finger against specific template
 * 
 * @param handle Driver handle
 * @param template_id Template ID to match against
 * @param confidence Pointer to store confidence score
 * @param timeout_ms Finger detection timeout (0 = default)
 * @return r305_status_t Status code
 */
r305_status_t r305_match_finger(r305_handle_t *handle, uint16_t template_id,  uint16_t *confidence, uint32_t timeout_ms);

/**
 * @brief Delete specific template
 * 
 * @param handle Driver handle
 * @param template_id Template ID to delete
 * @return r305_status_t Status code
 */
r305_status_t r305_delete_template(r305_handle_t *handle, uint16_t template_id);

/**
 * @brief Delete range of templates
 * 
 * @param handle Driver handle
 * @param start_id Starting template ID
 * @param count Number of templates to delete
 * @return r305_status_t Status code
 */
r305_status_t r305_delete_templates(r305_handle_t *handle, uint16_t start_id, uint16_t count);

/**
 * @brief Clear entire fingerprint database
 * 
 * @param handle Driver handle
 * @return r305_status_t Status code
 */
r305_status_t r305_clear_database(r305_handle_t *handle);

/**
 * @brief Check if finger is present on sensor
 * 
 * @param handle Driver handle
 * @param present Pointer to store result
 * @return r305_status_t Status code
 */
r305_status_t r305_is_finger_present(r305_handle_t *handle, bool *present);

/**
 * @brief Wait for finger placement
 * 
 * @param handle Driver handle
 * @param timeout_ms Timeout in milliseconds
 * @return r305_status_t Status code
 */
r305_status_t r305_wait_for_finger(r305_handle_t *handle, uint32_t timeout_ms);

/**
 * @brief Wait for finger removal
 * 
 * @param handle Driver handle
 * @param timeout_ms Timeout in milliseconds
 * @return r305_status_t Status code
 */
r305_status_t r305_wait_finger_removed(r305_handle_t *handle, uint32_t timeout_ms);

/**
 * @brief Get current driver state
 * 
 * @param handle Driver handle
 * @return r305_state_t Current state
 */
r305_state_t r305_get_state(r305_handle_t *handle);

/**
 * @brief Get status code description string
 * 
 * @param status Status code
 * @return const char* Description string
 */
const char* r305_status_to_string(r305_status_t status);

/**
 * @brief Get ACK code description string
 * 
 * @param ack_code ACK code
 * @return const char* Description string
 */
const char* r305_ack_to_string(r305_ack_code_t ack_code);

/**
 * @brief Parse status register bits
 * 
 * @param status_reg Status register value
 * @param bits Pointer to store parsed bits
 */
void r305_parse_status_register(uint16_t status_reg, r305_status_bits_t *bits);

/**
 * @brief Get confidence quality description
 * 
 * @param confidence Confidence score
 * @return const char* Quality description
 */
const char* r305_confidence_to_string(uint16_t confidence);

#ifdef __cplusplus
}
#endif

#endif /* R305_FINGERPRINT_H */
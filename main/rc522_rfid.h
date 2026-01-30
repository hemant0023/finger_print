/**
 * @file rc522_rfid.h
 * @brief RC522 RFID Module Driver for ESP32
 * @version 1.0
 * @date 2026-01-27
 * 
 * Driver for MFRC522 RFID reader module
 * Supports ISO/IEC 14443A cards (MIFARE)
 */

#ifndef RC522_RFID_H
#define RC522_RFID_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== CONFIGURATION ==================== */

#define RC522_SPI_HOST          SPI2_HOST
#define RC522_PIN_MISO          GPIO_NUM_19
#define RC522_PIN_MOSI          GPIO_NUM_23
#define RC522_PIN_SCK           GPIO_NUM_18
#define RC522_PIN_CS            GPIO_NUM_5
#define RC522_PIN_RST           GPIO_NUM_4

#define RC522_SPI_CLOCK_HZ      1000000  // 1 MHz

/* Card UID maximum length */
#define RC522_MAX_UID_LEN       10

/* ==================== STATUS CODES ==================== */

typedef enum {
    RC522_OK = 0,
    RC522_ERR_TIMEOUT,
    RC522_ERR_NO_CARD,
    RC522_ERR_COLLISION,
    RC522_ERR_CRC,
    RC522_ERR_PROTOCOL,
    RC522_ERR_COMM,
    RC522_ERR_INVALID_PARAM,
    RC522_ERR_NOT_INITIALIZED,
} rc522_status_t;

/* ==================== DATA STRUCTURES ==================== */

/**
 * @brief Card UID structure
 */
typedef struct {
    uint8_t data[RC522_MAX_UID_LEN];
    uint8_t length;
     uint8_t size;
     char uid[11];
    uint8_t sak;  // Select acknowledge
} rc522_uid_t;

/**
 * @brief Card type
 */
typedef enum {
    RC522_CARD_UNKNOWN = 0,
    RC522_CARD_MIFARE_MINI,
    RC522_CARD_MIFARE_1K,
    RC522_CARD_MIFARE_4K,
    RC522_CARD_MIFARE_UL,
    RC522_CARD_MIFARE_PLUS,
    RC522_CARD_MIFARE_DESFIRE,
} rc522_card_type_t;

/**
 * @brief RFID configuration
 */
typedef struct {
    gpio_num_t miso_pin;
    gpio_num_t mosi_pin;
    gpio_num_t sck_pin;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
    uint32_t spi_clock_hz;
} rc522_config_t;

/**
 * @brief RFID handle
 */
typedef struct rc522_handle_s rc522_handle_t;

/* ==================== PUBLIC API ==================== */

/**
 * @brief Initialize RC522 module
 */
rc522_status_t rc522_init(const rc522_config_t *config, rc522_handle_t **handle);

/**
 * @brief Deinitialize RC522 module
 */
rc522_status_t rc522_deinit(rc522_handle_t *handle);

/**
 * @brief Check if card is present
 */
rc522_status_t rc522_is_card_present(rc522_handle_t *handle, bool *present);

/**
 * @brief Read card UID
 */
rc522_status_t rc522_read_card_uid(rc522_handle_t *handle, rc522_uid_t *uid);

/**
 * @brief Get card type
 */
rc522_card_type_t rc522_get_card_type(rc522_handle_t *handle, const rc522_uid_t *uid);

/**
 * @brief Get card type string
 */
const char* rc522_card_type_to_string(rc522_card_type_t type);

/**
 * @brief Convert UID to string
 */
void rc522_uid_to_string(const rc522_uid_t *uid, char *str, size_t str_len);

/**
 * @brief Compare two UIDs
 */
bool rc522_uid_compare(const rc522_uid_t *uid1, const rc522_uid_t *uid2);

#ifdef __cplusplus
}
#endif

#endif /* RC522_RFID_H */
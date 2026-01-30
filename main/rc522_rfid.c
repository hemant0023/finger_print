
#include "rc522_rfid.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RC522";

/* MFRC522 Commands */
#define RC522_CMD_IDLE          0x00
#define RC522_CMD_CALCCRC       0x03
#define RC522_CMD_TRANSMIT      0x04
#define RC522_CMD_RECEIVE       0x08
#define RC522_CMD_TRANSCEIVE    0x0C
#define RC522_CMD_AUTHENT       0x0E
#define RC522_CMD_SOFTRESET     0x0F

/* MFRC522 Registers */
#define RC522_REG_COMMAND       0x01
#define RC522_REG_COMIEN        0x02
#define RC522_REG_COMMIRQ       0x04
#define RC522_REG_ERROR         0x06
#define RC522_REG_STATUS2       0x08
#define RC522_REG_FIFODATA      0x09
#define RC522_REG_FIFOLEVEL     0x0A
#define RC522_REG_CONTROL       0x0C
#define RC522_REG_BITFRAMING    0x0D
#define RC522_REG_MODE          0x11
#define RC522_REG_TXCONTROL     0x14
#define RC522_REG_TXAUTO        0x15
#define RC522_REG_VERSION       0x37

/* PICC Commands */
#define PICC_CMD_REQIDL         0x26
#define PICC_CMD_REQALL         0x52
#define PICC_CMD_ANTICOLL       0x93
#define PICC_CMD_SELECTTAG      0x93
#define PICC_CMD_READ           0x30

/* ==================== INTERNAL STRUCTURES ==================== */

struct rc522_handle_s {
    spi_device_handle_t spi;
    gpio_num_t rst_pin;
    bool initialized;
};

/* ==================== SPI COMMUNICATION ==================== */

static esp_err_t rc522_write_reg(rc522_handle_t *handle, uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = { (reg << 1) & 0x7E, value };
    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(handle->spi, &trans);
}

static esp_err_t rc522_read_reg(rc522_handle_t *handle, uint8_t reg, uint8_t *value) {
    uint8_t tx_data[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx_data[2];
    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    esp_err_t ret = spi_device_transmit(handle->spi, &trans);
    if (ret == ESP_OK) {
        *value = rx_data[1];
    }
    return ret;
}

static void rc522_set_bit_mask(rc522_handle_t *handle, uint8_t reg, uint8_t mask) {
    uint8_t tmp;
    rc522_read_reg(handle, reg, &tmp);
    rc522_write_reg(handle, reg, tmp | mask);
}

static void rc522_clear_bit_mask(rc522_handle_t *handle, uint8_t reg, uint8_t mask) {
    uint8_t tmp;
    rc522_read_reg(handle, reg, &tmp);
    rc522_write_reg(handle, reg, tmp & (~mask));
}

/* ==================== INITIALIZATION ==================== */

rc522_status_t rc522_init(const rc522_config_t *config, rc522_handle_t **handle) {
    if (!config || !handle) {
        return RC522_ERR_INVALID_PARAM;
    }

    rc522_handle_t *h = calloc(1, sizeof(rc522_handle_t));
    if (!h) {
        return RC522_ERR_COMM;
    }

    h->rst_pin = config->rst_pin;

    // Configure SPI
    spi_bus_config_t bus_cfg = {
        .miso_io_num = config->miso_pin,
        .mosi_io_num = config->mosi_pin,
        .sclk_io_num = config->sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->spi_clock_hz,
        .mode = 0,
        .spics_io_num = config->cs_pin,
        .queue_size = 7,
    };

    esp_err_t ret = spi_bus_initialize(RC522_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        free(h);
        return RC522_ERR_COMM;
    }

    ret = spi_bus_add_device(RC522_SPI_HOST, &dev_cfg, &h->spi);
    if (ret != ESP_OK) {
        spi_bus_free(RC522_SPI_HOST);
        free(h);
        return RC522_ERR_COMM;
    }

    // Configure reset pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << h->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    // Reset
    gpio_set_level(h->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(h->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Soft reset
    rc522_write_reg(h, RC522_REG_COMMAND, RC522_CMD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Configure
    rc522_write_reg(h, RC522_REG_MODE, 0x3D);
    rc522_write_reg(h, RC522_REG_TXAUTO, 0x40);
    rc522_write_reg(h, RC522_REG_TXCONTROL, 0x83);

    // Turn on antenna
    rc522_set_bit_mask(h, RC522_REG_TXCONTROL, 0x03);

    // Verify version
    uint8_t version;
    rc522_read_reg(h, RC522_REG_VERSION, &version);
    ESP_LOGI(TAG, "RC522 Version: 0x%02X", version);

    h->initialized = true;
    *handle = h;

    ESP_LOGI(TAG, "RC522 initialized successfully");
    return RC522_OK;
}

rc522_status_t rc522_deinit(rc522_handle_t *handle) {
    if (!handle) {
        return RC522_ERR_INVALID_PARAM;
    }

    spi_bus_remove_device(handle->spi);
    spi_bus_free(RC522_SPI_HOST);
    free(handle);

    return RC522_OK;
}

/* ==================== CARD OPERATIONS ==================== */

rc522_status_t rc522_is_card_present(rc522_handle_t *handle, bool *present) {
    if (!handle || !handle->initialized || !present) {
        return RC522_ERR_NOT_INITIALIZED;
    }

    rc522_write_reg(handle, RC522_REG_BITFRAMING, 0x07);

    uint8_t req_data[1] = { PICC_CMD_REQIDL };
    uint8_t back_data[2];
    uint8_t back_len = 0;

    rc522_write_reg(handle, RC522_REG_COMMIRQ, 0x7F);
    rc522_clear_bit_mask(handle, RC522_REG_FIFOLEVEL, 0x80);
    rc522_write_reg(handle, RC522_REG_FIFODATA, req_data[0]);
    rc522_write_reg(handle, RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    rc522_set_bit_mask(handle, RC522_REG_BITFRAMING, 0x80);

    // Wait for completion
    for (int i = 0; i < 2000; i++) {
        uint8_t n;
        rc522_read_reg(handle, RC522_REG_COMMIRQ, &n);
        if (n & 0x30) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    rc522_clear_bit_mask(handle, RC522_REG_BITFRAMING, 0x80);

    uint8_t irq;
    rc522_read_reg(handle, RC522_REG_COMMIRQ, &irq);

    *present = (irq & 0x20) ? true : false;
    return RC522_OK;
}

rc522_status_t rc522_read_card_uid(rc522_handle_t *handle, rc522_uid_t *uid) {
    if (!handle || !handle->initialized || !uid) {
        return RC522_ERR_NOT_INITIALIZED;
    }

    memset(uid, 0, sizeof(rc522_uid_t));

    // Request
    rc522_write_reg(handle, RC522_REG_BITFRAMING, 0x07);
    uint8_t req_data[1] = { PICC_CMD_REQALL };
    
    rc522_write_reg(handle, RC522_REG_COMMIRQ, 0x7F);
    rc522_clear_bit_mask(handle, RC522_REG_FIFOLEVEL, 0x80);
    rc522_write_reg(handle, RC522_REG_FIFODATA, req_data[0]);
    rc522_write_reg(handle, RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    rc522_set_bit_mask(handle, RC522_REG_BITFRAMING, 0x80);

    vTaskDelay(pdMS_TO_TICKS(10));
    rc522_clear_bit_mask(handle, RC522_REG_BITFRAMING, 0x80);

    // Anti-collision
    uint8_t anticoll_data[2] = { PICC_CMD_ANTICOLL, 0x20 };
    
    rc522_write_reg(handle, RC522_REG_COMMIRQ, 0x7F);
    rc522_clear_bit_mask(handle, RC522_REG_FIFOLEVEL, 0x80);
    
    for (int i = 0; i < 2; i++) {
        rc522_write_reg(handle, RC522_REG_FIFODATA, anticoll_data[i]);
    }
    
    rc522_write_reg(handle, RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
    rc522_set_bit_mask(handle, RC522_REG_BITFRAMING, 0x80);

    vTaskDelay(pdMS_TO_TICKS(10));
    rc522_clear_bit_mask(handle, RC522_REG_BITFRAMING, 0x80);

    // Read FIFO
    uint8_t fifo_level;
    rc522_read_reg(handle, RC522_REG_FIFOLEVEL, &fifo_level);

    if (fifo_level >= 5) {
        for (int i = 0; i < 4; i++) {
            rc522_read_reg(handle, RC522_REG_FIFODATA, &uid->data[i]);
        }
        uid->length = 4;
        
        ESP_LOGI(TAG, "Card detected: UID = %02X:%02X:%02X:%02X",
                 uid->data[0], uid->data[1], uid->data[2], uid->data[3]);
        
        return RC522_OK;
    }

    return RC522_ERR_NO_CARD;
}

rc522_card_type_t rc522_get_card_type(rc522_handle_t *handle, const rc522_uid_t *uid) {
    if (uid->sak & 0x04) {
        return RC522_CARD_MIFARE_UL;
    } else if (uid->sak & 0x08) {
        return RC522_CARD_MIFARE_1K;
    } else if (uid->sak & 0x18) {
        return RC522_CARD_MIFARE_4K;
    }
    return RC522_CARD_UNKNOWN;
}

const char* rc522_card_type_to_string(rc522_card_type_t type) {
    switch (type) {
        case RC522_CARD_MIFARE_MINI:    return "MIFARE Mini";
        case RC522_CARD_MIFARE_1K:      return "MIFARE 1K";
        case RC522_CARD_MIFARE_4K:      return "MIFARE 4K";
        case RC522_CARD_MIFARE_UL:      return "MIFARE Ultralight";
        case RC522_CARD_MIFARE_PLUS:    return "MIFARE Plus";
        case RC522_CARD_MIFARE_DESFIRE: return "MIFARE DESFire";
        default:                        return "Unknown";
    }
}

void rc522_uid_to_string(const rc522_uid_t *uid, char *str, size_t str_len) {
    if (!uid || !str || str_len < (uid->length * 3)) {
        return;
    }

    int pos = 0;
    for (int i = 0; i < uid->length; i++) {
        pos += snprintf(str + pos, str_len - pos, "%02X", uid->data[i]);
        if (i < uid->length - 1) {
            str[pos++] = ':';
        }
    }
}

bool rc522_uid_compare(const rc522_uid_t *uid1, const rc522_uid_t *uid2) {
    if (!uid1 || !uid2 || uid1->length != uid2->length) {
        return false;
    }
    return memcmp(uid1->data, uid2->data, uid1->length) == 0;
}




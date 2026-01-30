/**
 * @file buzzer.c
 * @brief Buzzer Control Implementation
 */

#include "buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
static const char *TAG = "BUZZER";

esp_err_t buzzer_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        gpio_set_level(BUZZER_PIN, 0);
        ESP_LOGI(TAG, "Buzzer initialized on GPIO %d", BUZZER_PIN);
    }
    
    return ret;
}

esp_err_t buzzer_deinit(void) {
    gpio_reset_pin(BUZZER_PIN);
    return ESP_OK;
}

void buzzer_on(void) {
    gpio_set_level(BUZZER_PIN, 1);
}

void buzzer_off(void) {
    gpio_set_level(BUZZER_PIN, 0);
}

void buzzer_beep(uint32_t duration_ms) {
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_off();
}

void buzzer_play_pattern(buzzer_pattern_t pattern) {
    switch (pattern) {
        case BUZZER_PATTERN_SUCCESS:
            // Single short beep
            buzzer_beep(100);
            break;
            
        case BUZZER_PATTERN_ERROR:
            // Two short beeps
            buzzer_beep(100);
            vTaskDelay(pdMS_TO_TICKS(100));
            buzzer_beep(100);
            break;
            
        case BUZZER_PATTERN_WARNING:
            // Long beep
            buzzer_beep(500);
            break;
            
        case BUZZER_PATTERN_ENROLL_START:
            // Three short beeps
            for (int i = 0; i < 3; i++) {
                buzzer_beep(80);
                vTaskDelay(pdMS_TO_TICKS(80));
            }
            break;
            
        case BUZZER_PATTERN_ENROLL_STEP:
            // Single medium beep
            buzzer_beep(150);
            break;
            
        case BUZZER_PATTERN_ENROLL_COMPLETE:
            // Ascending pattern
            buzzer_beep(100);
            vTaskDelay(pdMS_TO_TICKS(50));
            buzzer_beep(100);
            vTaskDelay(pdMS_TO_TICKS(50));
            buzzer_beep(200);
            break;
            
        case BUZZER_PATTERN_STARTUP:
            // Power-on melody
            buzzer_beep(80);
            vTaskDelay(pdMS_TO_TICKS(50));
            buzzer_beep(80);
            vTaskDelay(pdMS_TO_TICKS(50));
            buzzer_beep(120);
            break;
            
        case BUZZER_PATTERN_SHUTDOWN:
            // Power-off melody
            buzzer_beep(120);
            vTaskDelay(pdMS_TO_TICKS(50));
            buzzer_beep(80);
            vTaskDelay(pdMS_TO_TICKS(50));
            buzzer_beep(80);
            break;
            
        default:
            buzzer_beep(100);
            break;
    }
}

void buzzer_play_custom(const uint16_t *pattern, size_t pattern_len) {
    for (size_t i = 0; i < pattern_len; i++) {
        if (pattern[i] > 0) {
            buzzer_beep(pattern[i]);
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}





/**
 * @file buzzer.h
 * @brief Buzzer Control Module for Attendance System
 * @version 1.0
 * @date 2026-01-27
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== CONFIGURATION ==================== */

#define BUZZER_PIN              GPIO_NUM_25

/* ==================== BUZZER PATTERNS ==================== */

typedef enum {
    BUZZER_PATTERN_SUCCESS,         // Short beep (successful scan)
    BUZZER_PATTERN_ERROR,           // Two short beeps (error/denied)
    BUZZER_PATTERN_WARNING,         // Long beep (warning)
    BUZZER_PATTERN_ENROLL_START,    // Three short beeps (enrollment start)
    BUZZER_PATTERN_ENROLL_STEP,     // Single beep (enrollment step)
    BUZZER_PATTERN_ENROLL_COMPLETE, // Ascending tones (enrollment complete)
    BUZZER_PATTERN_STARTUP,         // Power-on melody
    BUZZER_PATTERN_SHUTDOWN,        // Power-off melody
} buzzer_pattern_t;

/* ==================== PUBLIC API ==================== */

/**
 * @brief Initialize buzzer
 */
esp_err_t buzzer_init(void);

/**
 * @brief Deinitialize buzzer
 */
esp_err_t buzzer_deinit(void);

/**
 * @brief Turn buzzer on
 */
void buzzer_on(void);

/**
 * @brief Turn buzzer off
 */
void buzzer_off(void);

/**
 * @brief Beep for specified duration
 */
void buzzer_beep(uint32_t duration_ms);

/**
 * @brief Play predefined pattern
 */
void buzzer_play_pattern(buzzer_pattern_t pattern);

/**
 * @brief Play custom pattern (non-blocking)
 */
void buzzer_play_custom(const uint16_t *pattern, size_t pattern_len);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
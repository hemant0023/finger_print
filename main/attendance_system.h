

#ifndef ATTENDANCE_SYSTEM_H
#define ATTENDANCE_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"
#include "rc522_rfid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== CONFIGURATION ==================== */


#define MAX_USERS 200
#define MAX_SESSIONS 50                  // Was 100
#define MAX_NAME_LEN 32
#define MAX_EMAIL_LEN 64
#define MAX_ROLL_NUMBER_LEN 16
#define MAX_DEPARTMENT_LEN 32
#define MAX_SUBJECT_LEN 20
#define MAX_SESSION_ID_LEN 16
#define MAX_ATTENDANCE_RECORDS 500

#define CLASSROOM_ID                "ECE-301"
#define CLASSROOM_NAME              "ECE-301-A"
#define DEPARTMENT_NAME             "PIET ECE"

/* ==================== USER TYPES ==================== */

typedef enum {
    USER_TYPE_STUDENT = 0,
    USER_TYPE_FACULTY = 1,
    USER_TYPE_ADMIN = 2,
    USER_TYPE_GUEST = 3,
} user_type_t;

/* ==================== USER STATUS ==================== */

typedef enum {
    USER_STATUS_ACTIVE = 0,
    USER_STATUS_INACTIVE = 1,
    USER_STATUS_SUSPENDED = 2,
    USER_STATUS_GRADUATED = 3,
} user_status_t;

/* ==================== ATTENDANCE STATUS ==================== */

typedef enum {
    ATTENDANCE_PRESENT = 0,
    ATTENDANCE_ABSENT =  1,
    ATTENDANCE_LATE =    2,
    ATTENDANCE_LEAVE =    3,
    USER_NOT_FOUND,
    ATTENDANCE_DONE,
    ATTENDANCE_FAILED,
    SESSION_INACTIVE,
    SESSION_ACTIVE,
    ALREAY_MARKED,
    MEMORY_FULL,
    SYSTEM_ERROR
   
} attendance_status_t;

/* ==================== AUTHENTICATION MODE ==================== */

typedef enum {
    AUTH_METHOD_FINGERPRINT = 0,
    AUTH_METHOD_RFID = 1,
    AUTH_METHOD_BOTH = 2
} auth_method_t;
/* ==================== DATA STRUCTURES ==================== */

/**
 * @brief User record structure
 */
typedef struct {
    int16_t user_id;
    char name[MAX_NAME_LEN];
    char email[MAX_EMAIL_LEN];
    user_type_t type;
    user_status_t status;
    
    // Student specific
    char roll_number[MAX_ROLL_NUMBER_LEN];
    uint8_t semester;
    char department[MAX_DEPARTMENT_LEN];
    char department_code[15];
    char classroom_code[16];
    char classroom_name[MAX_DEPARTMENT_LEN];
    
    // Authentication
    uint16_t fingerprint_id;
    bool fingerprint_enrolled;
   
    //rc522_uid_t rfid_uid;
    
    
    bool rfid_enrolled;
    uint8_t rfid_uid[10];
    uint8_t rfid_uid_len;
    
    // Statistics
    uint32_t total_attendance;
    uint32_t total_classes;
    time_t created_at;
    time_t last_seen;
} user_record_t;

/**
 * @brief Session information
 */
typedef struct {
	
    char session_id[MAX_SESSION_ID_LEN];
    char subject_name[MAX_SUBJECT_LEN];
    char faculty_name[MAX_NAME_LEN];
    char department[MAX_DEPARTMENT_LEN];
    
    bool   active;
    time_t start_time;
    time_t end_time;
    
    uint16_t total_present;
    uint16_t total_absent;
} session_info_t;

/**
 * @brief Attendance record structure
 */
 
typedef struct {
	
	 uint16_t user_id;
     uint32_t record_id;
    time_t timestamp;
    attendance_status_t status;
    auth_method_t auth_method;
    char session_id[MAX_SESSION_ID_LEN];
    
} attendance_record_t;




/**
 * @brief System statistics
 */
typedef struct {
	
    uint16_t total_users;
    uint16_t total_students;
    uint16_t total_faculty;
 
    uint32_t total_attendance_records;
    uint32_t today_attendance_count;
  
    uint32_t successful_scans;
    uint32_t failed_scans;
     time_t system_start_time;
     time_t last_updated;
} system_stats_t;


/**
 * @brief System configuration
 */
typedef struct {
    char classroom_id[16];
    char classroom_name[64];
    char department[MAX_DEPARTMENT_LEN];
    auth_method_t default_auth_mode;
    bool auto_logout_enabled;
    uint32_t auto_logout_timeout;
    bool rfid_enabled;
    bool fingerprint_enabled;
    bool buzzer_enabled;
    bool lcd_backlight_enabled;
} system_config_t;
/*
 ==================== CALLBACK TYPES ==================== 

typedef void (*attendance_callback_t)(user_record_t *user, attendance_status_t status);
typedef void (*enrollment_callback_t)(user_record_t *user, bool success);

 ==================== PUBLIC API ==================== 

*
 * @brief Initialize attendance system
 
esp_err_t attendance_system_init(void);

*
 * @brief Start attendance session
 
esp_err_t attendance_start_session(const char *subject, const char *faculty);

*
 * @brief Stop attendance session
 
esp_err_t attendance_stop_session(void);

*
 * @brief Mark attendance (auto-detect method)
 

esp_err_t attendance_mark(uint16_t user_id, attendance_status_t status, auth_method_t method);
*
 * @brief Add new user
 
esp_err_t attendance_add_user(user_record_t *user);

*
 * @brief Update user information
 
esp_err_t attendance_update_user(uint16_t user_id, user_record_t *user);

*
 * @brief Delete user
 
esp_err_t attendance_delete_user(uint16_t user_id);

*
 * @brief Get user by ID
 
esp_err_t attendance_get_user(uint16_t user_id, user_record_t *user);

*
 * @brief Find user by fingerprint ID
 
esp_err_t attendance_find_user_by_fingerprint(uint16_t fp_id, user_record_t *user);

*
 * @brief Find user by RFID UID
 
esp_err_t attendance_find_user_by_rfid(rc522_uid_t *uid, user_record_t *user);

*
 * @brief List all users
 
esp_err_t attendance_list_users(user_record_t *users, uint16_t max_count, 
                                uint16_t *actual_count);

*
 * @brief Get attendance records for user
 
esp_err_t attendance_get_user_records(uint16_t user_id, 
                                      attendance_record_t *records,
                                      uint16_t max_count, 
                                      uint16_t *actual_count);

*
 * @brief Get today's attendance
 
esp_err_t attendance_get_today_records(attendance_record_t *records,
                                       uint16_t max_count,
                                       uint16_t *actual_count);

*
 * @brief Get system statistics
 
esp_err_t attendance_get_statistics(system_stats_t *stats);

*
 * @brief Get system configuration
 
esp_err_t attendance_get_config(system_config_t *config);

*
 * @brief Update system configuration
 
esp_err_t attendance_update_config(system_config_t *config);

*
 * @brief Export attendance data (CSV format)
 
esp_err_t attendance_export_csv(char *buffer, size_t buffer_size, 
                                time_t start_date, time_t end_date);

*
 * @brief Clear all attendance records
 
esp_err_t attendance_clear_records(void);

*
 * @brief Backup data to file
 
esp_err_t attendance_backup_data(const char *filename);

*
 * @brief Restore data from file
 
esp_err_t attendance_restore_data(const char *filename);

*
 * @brief Register attendance callback
 
void attendance_register_callback(attendance_callback_t callback);

 ==================== HELPER FUNCTIONS ==================== 

*
 * @brief User type to string
 
const char* user_type_to_string(user_type_t type);

*
 * @brief User status to string
 
const char* user_status_to_string(user_status_t status);

*
 * @brief Attendance status to string
 
const char* attendance_status_to_string(attendance_status_t status);

*
 * @brief Calculate attendance percentage
 
float attendance_calculate_percentage(user_record_t *user);

*
 * @brief Check if user is late
 
bool attendance_is_late(time_t entry_time, time_t session_start);*/

#ifdef __cplusplus
}
#endif

#endif /* ATTENDANCE_SYSTEM_H */
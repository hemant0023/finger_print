
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"


#include "r305_fingerprint.h"
#include "rc522_rfid.h"
#include "LCD_DISPLAY.h"
#include "buzzer.h"
#include "attendance_system.h"

static const char *TAG = "ATTENDANCE";

/* ==================== GLOBAL HANDLES ==================== */

static r305_handle_t *fp_handle = NULL;
static rc522_handle_t *rfid_handle = NULL;


static bool system_ready = false;
static session_info_t current_session = {0};
static system_stats_t system_stats = {0};

/* Simple user database (in production, use NVS) */
static user_record_t user_database[MAX_USERS];
static attendance_record_t attendance_records[MAX_ATTENDANCE_RECORDS];
static uint16_t total_users = 0;
static uint32_t total_records = 0;

/* ==================== LCD DISPLAY FUNCTIONS ==================== */

void lcd_show_welcome(void) {
    clear_lcd();
    lcd_set_cursor(0, 0);
    lcd_write("  ATTENDANCE  ");
    lcd_set_cursor(1, 0);
    lcd_write("    SYSTEM    ");
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void lcd_show_ready(void) {
    clear_lcd();
    lcd_set_cursor(0, 0);
    lcd_write("System Ready");
    lcd_set_cursor(1, 0);
    lcd_write("Scan ID/Finger");
}

void lcd_show_scanning(void) {
    clear_lcd();
    lcd_set_cursor(0, 0);
    lcd_write("  Scanning...");
    lcd_set_cursor(1, 0);
    lcd_write("Please Wait");
}

void lcd_show_user(user_record_t *user, attendance_status_t status) {
    clear_lcd();
    lcd_set_cursor(0, 0);
    
    char line1[17];
    snprintf(line1, sizeof(line1), "%.15s", user->name);
    lcd_write(line1);
    
    lcd_set_cursor(1, 0);
    
    char line2[17];
    if (status == ATTENDANCE_PRESENT) {
        snprintf(line2, sizeof(line2), "Present %02d:%02d", 
                 (int)(time(NULL) % 86400) / 3600,
                 (int)(time(NULL) % 3600) / 60);
    } else {
        snprintf(line2, sizeof(line2), "Status: %s", 
                 attendance_status_to_string(status));
    }
    lcd_write(line2);
}

void lcd_show_error(const char *message) {
    clear_lcd();
    lcd_set_cursor(0, 0);
    lcd_write("   ERROR!   ");
    lcd_set_cursor(1, 0);
    
    char line[17];
    snprintf(line, sizeof(line), "%.15s", message);
    lcd_write(line);
}

void lcd_show_enrollment(const char *name, int step) {
    clear_lcd();
    lcd_set_cursor(0, 0);
    
    char line1[17];
    snprintf(line1, sizeof(line1), "Enroll: %.8s", name);
    lcd_write(line1);
    
    lcd_set_cursor(1, 0);
    
    char line2[17];
    snprintf(line2, sizeof(line2), "Step %d/2", step);
    lcd_write(line2);
}

void lcd_show_stats(void) {
    clear_lcd();
    lcd_set_cursor(0, 0);
    
    char line1[17];
    snprintf(line1, sizeof(line1), "Users: %d", system_stats.total_users);
    lcd_write(line1);
    
    lcd_set_cursor(1, 0);
    
    char line2[17];
    snprintf(line2, sizeof(line2), "Today: %lu", system_stats.today_attendance_count);
    lcd_write(line2);
}

/* ==================== DATABASE FUNCTIONS ==================== */

void init_user_database(void) {
    memset(user_database, 0, sizeof(user_database));
    memset(attendance_records, 0, sizeof(attendance_records));
    total_users = 0;
    total_records = 0;
    
    ESP_LOGI(TAG, "User database initialized");
}

int find_user_by_fingerprint(uint16_t fp_id) {
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].fingerprint_enrolled && 
            user_database[i].fingerprint_id == fp_id &&
            user_database[i].status == USER_STATUS_ACTIVE) {
            return i;
        }
    }
    return -1;
}

int find_user_by_rfid(rc522_uid_t *uid) {
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].rfid_enrolled && 
            rc522_uid_compare(&user_database[i].rfid_uid, uid) &&
            user_database[i].status == USER_STATUS_ACTIVE) {
            return i;
        }
    }
    return -1;
}

bool add_user(user_record_t *user) {
    if (total_users >= MAX_USERS) {
        ESP_LOGE(TAG, "User database full!");
        return false;
    }
    
    user->user_id = total_users;
    user->created_at = time(NULL);
    user->total_attendance = 0;
    user->total_classes = 0;
    
    memcpy(&user_database[total_users], user, sizeof(user_record_t));
    total_users++;
    system_stats.total_users = total_users;
    
    if (user->type == USER_TYPE_STUDENT) {
        system_stats.total_students++;
    } else if (user->type == USER_TYPE_FACULTY) {
        system_stats.total_faculty++;
    }
    
    ESP_LOGI(TAG, "User added: ID=%d, Name=%s, Type=%s", 
             user->user_id, user->name, user_type_to_string(user->type));
    
    return true;
}

bool mark_attendance(uint16_t user_id, auth_mode_t method) {
    if (user_id >= total_users) {
        return false;
    }
    
    if (total_records >= MAX_ATTENDANCE_RECORDS) {
        ESP_LOGE(TAG, "Attendance records full!");
        return false;
    }
    
    // Check if already marked today
    time_t now = time(NULL);
    struct tm *timeinfo = localtime(&now);
    time_t today_start = mktime(&(struct tm){
        .tm_year = timeinfo->tm_year,
        .tm_mon = timeinfo->tm_mon,
        .tm_mday = timeinfo->tm_mday,
        .tm_hour = 0, .tm_min = 0, .tm_sec = 0
    });
    
    for (uint32_t i = 0; i < total_records; i++) {
        if (attendance_records[i].user_id == user_id && 
            attendance_records[i].timestamp >= today_start) {
            ESP_LOGW(TAG, "User %d already marked attendance today", user_id);
            return false;
        }
    }
    
    // Mark attendance
    attendance_record_t *record = &attendance_records[total_records];
    record->record_id = total_records;
    record->user_id = user_id;
    record->timestamp = now;
    record->auth_method = method;
    
    // Determine if late
    if (current_session.active && 
        now > (current_session.start_time + 15 * 60)) {  // 15 min grace
        record->status = ATTENDANCE_LATE;
    } else {
        record->status = ATTENDANCE_PRESENT;
    }
    
    strncpy(record->session_id, current_session.session_id, 
            sizeof(record->session_id));
    
    total_records++;
    system_stats.total_attendance_records = total_records;
    system_stats.today_attendance_count++;
    system_stats.successful_scans++;
    
    // Update user statistics
    user_database[user_id].total_attendance++;
    user_database[user_id].total_classes++;
    user_database[user_id].last_seen = now;
    
    if (current_session.active) {
        current_session.total_present++;
    }
    
    ESP_LOGI(TAG, "Attendance marked: User=%d, Status=%s, Method=%d",
             user_id, attendance_status_to_string(record->status), method);
    
    return true;
}

/* ==================== FINGERPRINT HANDLER ==================== */

void fingerprint_event_handler(r305_event_t *event, void *user_data) {
    switch (event->type) {
        case R305_EVENT_SEARCH_SUCCESS: {
            r305_search_result_t *result = &event->data.search_result;
            
            int user_idx = find_user_by_fingerprint(result->template_id);
            if (user_idx >= 0) {
                user_record_t *user = &user_database[user_idx];
                
                if (mark_attendance(user->user_id, AUTH_MODE_FINGERPRINT)) {
                    lcd_show_user(user, ATTENDANCE_PRESENT);
                    buzzer_play_pattern(BUZZER_PATTERN_SUCCESS);
                    
                    ESP_LOGI(TAG, "✓ ACCESS GRANTED - %s (FP)", user->name);
                } else {
                    lcd_show_error("Already marked");
                    buzzer_play_pattern(BUZZER_PATTERN_ERROR);
                }
            } else {
                lcd_show_error("User not found");
                buzzer_play_pattern(BUZZER_PATTERN_ERROR);
                system_stats.failed_scans++;
            }
            
            vTaskDelay(pdMS_TO_TICKS(3000));
            lcd_show_ready();
            break;
        }
        
        case R305_EVENT_SEARCH_FAILED:
            lcd_show_error("Not recognized");
            buzzer_play_pattern(BUZZER_PATTERN_ERROR);
            system_stats.failed_scans++;
            vTaskDelay(pdMS_TO_TICKS(2000));
            lcd_show_ready();
            break;
            
        case R305_EVENT_ENROLL_PROGRESS: {
            r305_enroll_progress_t *progress = &event->data.enroll_progress;
            int step = (progress->state <= R305_ENROLL_PROCESS_1) ? 1 : 2;
            lcd_show_enrollment("User", step);
            buzzer_play_pattern(BUZZER_PATTERN_ENROLL_STEP);
            break;
        }
        
        default:
            break;
    }
}

/* ==================== RFID SCAN TASK ==================== */

void rfid_scan_task(void *pvParameters) {
    rc522_uid_t last_uid = {0};
    time_t last_scan_time = 0;
    
    while (1) {
        if (system_ready && current_session.active) {
            bool card_present = false;
            rc522_is_card_present(rfid_handle, &card_present);
            
            if (card_present) {
                rc522_uid_t uid;
                if (rc522_read_card_uid(rfid_handle, &uid) == RC522_OK) {
                    time_t now = time(NULL);
                    
                    // Debounce - prevent double scans
                    if (!rc522_uid_compare(&uid, &last_uid) || 
                        (now - last_scan_time) > 3) {
                        
                        memcpy(&last_uid, &uid, sizeof(rc522_uid_t));
                        last_scan_time = now;
                        
                        lcd_show_scanning();
                        
                        int user_idx = find_user_by_rfid(&uid);
                        if (user_idx >= 0) {
                            user_record_t *user = &user_database[user_idx];
                            
                            if (mark_attendance(user->user_id, AUTH_MODE_RFID)) {
                                lcd_show_user(user, ATTENDANCE_PRESENT);
                                buzzer_play_pattern(BUZZER_PATTERN_SUCCESS);
                                
                                ESP_LOGI(TAG, "✓ ACCESS GRANTED - %s (RFID)", 
                                         user->name);
                            } else {
                                lcd_show_error("Already marked");
                                buzzer_play_pattern(BUZZER_PATTERN_ERROR);
                            }
                        } else {
                            lcd_show_error("Card not found");
                            buzzer_play_pattern(BUZZER_PATTERN_ERROR);
                            system_stats.failed_scans++;
                        }
                        
                        vTaskDelay(pdMS_TO_TICKS(3000));
                        lcd_show_ready();
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ==================== FINGERPRINT SCAN TASK ==================== */

void fingerprint_scan_task(void *pvParameters) {
    while (1) {
        if (system_ready && current_session.active) {
            r305_search_result_t result;
            r305_status_t status = r305_search_finger(fp_handle, &result, 5000);
            
            if (status != R305_STATUS_TIMEOUT) {
                // Event handler takes care of feedback
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/* ==================== WEB SERVER - HTML PAGES ==================== */

const char* html_header = 
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:Arial;margin:20px;background:#f0f0f0}"
    ".container{max-width:1200px;margin:auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}"
    "h1{color:#333;border-bottom:3px solid #007bff;padding-bottom:10px}"
    "h2{color:#555}"
    ".nav{margin-bottom:20px}"
    ".nav a{text-decoration:none;padding:10px 20px;margin:5px;background:#007bff;color:white;border-radius:5px;display:inline-block}"
    ".nav a:hover{background:#0056b3}"
    ".card{background:#f9f9f9;padding:15px;margin:10px 0;border-left:4px solid #007bff;border-radius:5px}"
    ".stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin:20px 0}"
    ".stat-box{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;padding:20px;border-radius:10px;text-align:center}"
    ".stat-box h3{margin:0;font-size:2em}"
    ".stat-box p{margin:5px 0}"
    "table{width:100%;border-collapse:collapse;margin:20px 0}"
    "th,td{padding:12px;text-align:left;border-bottom:1px solid #ddd}"
    "th{background:#007bff;color:white}"
    "tr:hover{background:#f5f5f5}"
    "input,select{width:100%;padding:10px;margin:5px 0;border:1px solid #ddd;border-radius:5px;box-sizing:border-box}"
    "button{padding:12px 30px;background:#28a745;color:white;border:none;border-radius:5px;cursor:pointer;font-size:1em}"
    "button:hover{background:#218838}"
    ".btn-danger{background:#dc3545}"
    ".btn-danger:hover{background:#c82333}"
    ".success{color:#28a745;font-weight:bold}"
    ".error{color:#dc3545;font-weight:bold}"
    ".present{color:#28a745}"
    ".absent{color:#dc3545}"
    ".late{color:#ff9800}"
    "</style></head><body><div class='container'>";

const char* html_footer = "</div></body></html>";

/* ==================== WEB SERVER HANDLERS ==================== */


  

/* API: Start Session */
static esp_err_t api_session_start(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content));
    
    if (ret > 0) {
        // Parse form data (simplified)
        // In production, use proper URL decoding
        current_session.active = true;
        current_session.start_time = time(NULL);
        snprintf(current_session.subject_name, sizeof(current_session.subject_name), 
                 "Computer Networks");
        snprintf(current_session.faculty_name, sizeof(current_session.faculty_name), 
                 "Prof. Smith");
        snprintf(current_session.session_id, sizeof(current_session.session_id), 
                 "SES%lld", time(NULL));
        current_session.total_present = 0;
        current_session.total_absent = 0;
        
        lcd_show_ready();
        buzzer_play_pattern(BUZZER_PATTERN_STARTUP);
        
        ESP_LOGI(TAG, "Session started: %s - %s", 
                 current_session.subject_name, current_session.faculty_name);
    }
    
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/session");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* API: Stop Session */
static esp_err_t api_session_stop(httpd_req_t *req) {
    current_session.active = false;
    current_session.end_time = time(NULL);
    
    lcd_show_stats();
    buzzer_play_pattern(BUZZER_PATTERN_SHUTDOWN);
    
    ESP_LOGI(TAG, "Session stopped. Present: %d", 
             current_session.total_present);
    
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/session");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ==================== WEB SERVER INITIALIZATION ==================== */



/* ==================== WIFI INITIALIZATION ==================== */

void wifi_init_softap(void) {
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "AttendanceSystem",
            .ssid_len = strlen("AttendanceSystem"),
            .password = "admin1234",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP started: SSID=AttendanceSystem, IP=192.168.4.1");
}

/* ==================== SYSTEM INITIALIZATION ==================== */

bool initialize_attendance_system(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  CLASSROOM ATTENDANCE SYSTEM");
    ESP_LOGI(TAG, "  Department: %s", DEPARTMENT_NAME);
    ESP_LOGI(TAG, "  Classroom: %s", CLASSROOM_NAME);
    ESP_LOGI(TAG, "========================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize LCD
    ESP_LOGI(TAG, "Initializing LCD...");
    LCD_INITILIZED();
    lcd_show_welcome();
    
    // Initialize Buzzer
  //  ESP_LOGI(TAG, "Initializing Buzzer...");
   // buzzer_init();
   // buzzer_play_pattern(BUZZER_PATTERN_STARTUP);
    
    // Initialize Fingerprint
    ESP_LOGI(TAG, "Initializing Fingerprint Scanner...");
    r305_config_t fp_config = {
        .tx_pin = GPIO_NUM_16,
        .rx_pin = GPIO_NUM_17,
        .baud_rate = 57600,
        .security_level = R305_SECURITY_LEVEL_3,
        .device_address = R305_DEFAULT_ADDR,
        .event_callback = fingerprint_event_handler,
        .callback_user_data = NULL
    };
    
    if (r305_init(&fp_config, &fp_handle) != R305_STATUS_OK) {
        ESP_LOGE(TAG, "Fingerprint init failed!");
        lcd_show_error("FP Init Fail");
        return false;
    }
    
    if (r305_handshake(fp_handle) != R305_STATUS_OK) {
        ESP_LOGE(TAG, "Fingerprint handshake failed!");
        lcd_show_error("FP Not Found");
        return false;
    }
    
    // Initialize RFID
    ESP_LOGI(TAG, "Initializing RFID Reader...");
    rc522_config_t rfid_config = {
        .miso_pin = RC522_PIN_MISO,
        .mosi_pin = RC522_PIN_MOSI,
        .sck_pin = RC522_PIN_SCK,
        .cs_pin = RC522_PIN_CS,
        .rst_pin = RC522_PIN_RST,
        .spi_clock_hz = RC522_SPI_CLOCK_HZ
    };
    
    if (rc522_init(&rfid_config, &rfid_handle) != RC522_OK) {
        ESP_LOGE(TAG, "RFID init failed!");
        lcd_show_error("RFID Init Fail");
        return false;
    }
    
    // Initialize database
    init_user_database();
    
    // Add sample users for demonstration
    user_record_t sample_user1 = {
        .type = USER_TYPE_STUDENT,
        .status = USER_STATUS_ACTIVE,
        .semester = 6,
        .fingerprint_enrolled = false,
        .rfid_enrolled = false
    };
    strncpy(sample_user1.name, "John Doe", MAX_NAME_LEN);
    strncpy(sample_user1.roll_number, "CS2021001", MAX_ROLL_NO_LEN);
    strncpy(sample_user1.department, "Computer Science", MAX_DEPARTMENT_LEN);
    strncpy(sample_user1.email, "john@university.edu", MAX_EMAIL_LEN);
    add_user(&sample_user1);
    
    user_record_t sample_user2 = {
        .type = USER_TYPE_FACULTY,
        .status = USER_STATUS_ACTIVE,
        .fingerprint_enrolled = false,
        .rfid_enrolled = false
    };
    strncpy(sample_user2.name, "Prof. Smith", MAX_NAME_LEN);
    strncpy(sample_user2.department, "Computer Science", MAX_DEPARTMENT_LEN);
    strncpy(sample_user2.email, "smith@university.edu", MAX_EMAIL_LEN);
    add_user(&sample_user2);
    
    // Initialize WiFi and Web Server
    ESP_LOGI(TAG, "Initializing Network...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_softap();
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
   // start_webserver();
    
    // Initialize statistics
    system_stats.system_start_time = time(NULL);
    
    lcd_show_ready();
    buzzer_play_pattern(BUZZER_PATTERN_SUCCESS);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "System Ready!");
    ESP_LOGI(TAG, "WiFi: AttendanceSystem / admin1234");
    ESP_LOGI(TAG, "Web Interface: http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");
    
    system_ready = true;
    return true;
}

/* ==================== HELPER FUNCTION IMPLEMENTATIONS ==================== */

const char* user_type_to_string(user_type_t type) {
    switch (type) {
        case USER_TYPE_STUDENT: return "Student";
        case USER_TYPE_FACULTY: return "Faculty";
        case USER_TYPE_ADMIN: return "Admin";
        case USER_TYPE_GUEST: return "Guest";
        default: return "Unknown";
    }
}

const char* user_status_to_string(user_status_t status) {
    switch (status) {
        case USER_STATUS_ACTIVE: return "Active";
        case USER_STATUS_INACTIVE: return "Inactive";
        case USER_STATUS_SUSPENDED: return "Suspended";
        case USER_STATUS_GRADUATED: return "Graduated";
        default: return "Unknown";
    }
}

const char* attendance_status_to_string(attendance_status_t status) {
    switch (status) {
        case ATTENDANCE_PRESENT: return "Present";
        case ATTENDANCE_ABSENT: return "Absent";
        case ATTENDANCE_LATE: return "Late";
        case ATTENDANCE_LEAVE: return "Leave";
        default: return "Unknown";
    }
}

/* ==================== MAIN APPLICATION ==================== */
/*
void app_main(void) {
    ESP_LOGI(TAG, "Starting Classroom Attendance System...");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    if (!initialize_attendance_system()) {
        ESP_LOGE(TAG, "System initialization failed!");
        return;
    }
    
    // Create scanning tasks
    xTaskCreate(rfid_scan_task, "rfid_scan", 4096, NULL, 5, NULL);
    xTaskCreate(fingerprint_scan_task, "fp_scan", 4096, NULL, 5, NULL);
    
    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Periodic statistics update
        if (system_ready && !current_session.active) {
            lcd_show_stats();
            vTaskDelay(pdMS_TO_TICKS(5000));
            lcd_show_ready();
        }
    }
}*/




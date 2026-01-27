
 
 
#include "esp_event.h"
#include <esp_log.h>
#include "esp_system.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "r305_fingerprint.h"

static const char *TAG = "MAIN";

/* ==================== GLOBAL STATE ==================== */

static r305_handle_t *fp_handle = NULL;
static uint16_t current_user_count = 0;
static bool system_ready = false;

/* ==================== USER MANAGEMENT ==================== */

#define MAX_USERNAME_LEN 32

typedef struct {
    uint16_t template_id;
    char username[MAX_USERNAME_LEN];
    bool enrolled;
} user_record_t;

// Simple user database (in production, use NVS or external database)
static user_record_t user_database[R305_MAX_TEMPLATES];

/**
 * @brief Initialize user database
 */
static void init_user_database(void) {
	
    memset(user_database, 0, sizeof(user_database));
    for (int i = 0; i < R305_MAX_TEMPLATES; i++) {
        user_database[i].template_id = i;
    }
    ESP_LOGI(TAG, "User database initialized");
}

/**
 * @brief Find next available template ID
 */
static int find_next_free_id(void){
	ESP_LOGW(TAG, "current_user_count = %d ",current_user_count);
    for (int i = 0; i < R305_MAX_TEMPLATES; i++){
        if (!user_database[i].enrolled) {// && i == current_user_count + 1
            return i;
        }
    }
    return -1;  // Database full
}

/**
 * @brief Add user to database
 */
static bool add_user(uint16_t template_id, const char *username) {
 
    if (template_id >= R305_MAX_TEMPLATES) {
        return false;
    }
    
    user_database[template_id].enrolled = true;
    strncpy(user_database[template_id].username, username, MAX_USERNAME_LEN - 1);
    user_database[template_id].username[MAX_USERNAME_LEN - 1] = '\0';
    current_user_count++;
    
    ESP_LOGI(TAG, "User added: ID=%d, Name=%s", template_id, username);
    return true;
}

/**
 * @brief Get user info by template ID
 */
static user_record_t* get_user(uint16_t template_id) {
  
    if (template_id >= R305_MAX_TEMPLATES || !user_database[template_id].enrolled) {
        return NULL;
    }
    
    return &user_database[template_id];
}

/**
 * @brief Remove user from database
 */
static bool remove_user(uint16_t template_id){
	
    if (template_id >= R305_MAX_TEMPLATES || !user_database[template_id].enrolled) {
        return false;
    }
    
    ESP_LOGI(TAG, "User removed: ID=%d, Name=%s", template_id, user_database[template_id].username);
    memset(&user_database[template_id], 0, sizeof(user_record_t));
    user_database[template_id].template_id = template_id;
    current_user_count--;
    
    return true;
}

/* ==================== EVENT CALLBACK ==================== */

/**
 * @brief Event callback handler
 */
static void fingerprint_event_callback(r305_event_t *event, void *user_data) {
   
    switch (event->type){
		
        case R305_EVENT_INITIALIZED:
            ESP_LOGI(TAG, ">>> EVENT: System initialized");
            break;
            
        case R305_EVENT_FINGER_DETECTED:
            ESP_LOGI(TAG, ">>> EVENT: Finger detected");
            break;
            
        case R305_EVENT_FINGER_REMOVED:
            ESP_LOGI(TAG, ">>> EVENT: Finger removed");
            break;
            
        case R305_EVENT_IMAGE_CAPTURED:
            ESP_LOGI(TAG, ">>> EVENT: Image captured");
            break;
            
        case R305_EVENT_ENROLL_PROGRESS: {
            r305_enroll_progress_t *progress = &event->data.enroll_progress;
            const char *state_str = "";
            
            switch (progress->state) {
                case R305_ENROLL_WAIT_FINGER_1:
                    state_str = "Waiting for first finger placement";
                    break;
                case R305_ENROLL_PROCESS_1:
                    state_str = "Processing first image";
                    break;
                case R305_ENROLL_REMOVE_FINGER:
                    state_str = "Remove finger";
                    break;
                case R305_ENROLL_WAIT_FINGER_2:
                    state_str = "Waiting for second finger placement";
                    break;
                case R305_ENROLL_PROCESS_2:
                    state_str = "Processing second image";
                    break;
                case R305_ENROLL_MERGE:
                    state_str = "Merging templates";
                    break;
                case R305_ENROLL_STORE:
                    state_str = "Storing template";
                    break;
                default:
                    break;
            }
            
            ESP_LOGI(TAG, ">>> ENROLLMENT PROGRESS: %s (Attempt %d/%d)",
                     state_str, progress->attempt, progress->max_attempts);
            break;
        }
        
        case R305_EVENT_ENROLL_SUCCESS:
            ESP_LOGI(TAG, ">>> EVENT: Enrollment successful - ID %d",
                     event->data.template_id);
            break;
            
        case R305_EVENT_ENROLL_FAILED:
            ESP_LOGE(TAG, ">>> EVENT: Enrollment failed - %s",
                     r305_status_to_string(event->data.error_code));
            break;
            
        case R305_EVENT_MATCH_SUCCESS:
            ESP_LOGI(TAG, ">>> EVENT: Match successful - ID %d",
                     event->data.template_id);
            break;
            
        case R305_EVENT_MATCH_FAILED:
            ESP_LOGW(TAG, ">>> EVENT: Match failed");
            break;
            
        case R305_EVENT_SEARCH_SUCCESS: {
            r305_search_result_t *result = &event->data.search_result;
            ESP_LOGI(TAG, ">>> EVENT: Search successful - ID %d, Confidence %d",
                     result->template_id, result->confidence);
            break;
        }
        
        case R305_EVENT_SEARCH_FAILED:
            ESP_LOGW(TAG, ">>> EVENT: Search failed - no match found");
            break;
            
        case R305_EVENT_ERROR:
            ESP_LOGE(TAG, ">>> EVENT: Error - %s",
            r305_status_to_string(event->data.error_code));
            break;
            
        default:
            break;
    }
}

/* ==================== SYSTEM OPERATIONS ==================== */

/**
 * @brief Initialize fingerprint system
 */
static bool initialize_system(void) {
	
    // Configure driver
    r305_config_t config = {
		
        .tx_pin = GPIO_NUM_16,
        .rx_pin = GPIO_NUM_17,
        .baud_rate = 9600,
        .security_level = R305_SECURITY_LEVEL_3,
        .device_address = R305_DEFAULT_ADDR,
        .event_callback = fingerprint_event_callback,
        .callback_user_data = NULL
    };
    
   
    ESP_LOGI(TAG, "Initializing R305 driver...");
    r305_status_t status = r305_init(&config, &fp_handle);
    if (status != R305_STATUS_OK){
        ESP_LOGE(TAG, "Driver initialization failed: %s", r305_status_to_string(status));
        return false;
    }
    
    // Verify communication
    ESP_LOGI(TAG, "Verifying module communication...");
    
    status = r305_handshake(fp_handle);
    if (status != R305_STATUS_OK){
        ESP_LOGE(TAG, "Handshake failed: %s", r305_status_to_string(status));
        return false;
    }
    
    ESP_LOGI(TAG, "Module communication verified!");
    
    // Read system parameters
    r305_sys_params_t params;
    status = r305_read_sys_params(fp_handle, &params);
    if (status == R305_STATUS_OK) {
		  ESP_LOGI(TAG, "=== System Parameters ===");
        ESP_LOGI(TAG, "System ID: 0x%04X", params.system_id);
        ESP_LOGI(TAG, "Library Size: %d", params.library_size);
        ESP_LOGI(TAG, "Security Level: %d", params.security_level);
        ESP_LOGI(TAG, "Device Address: 0x%08lX", params.device_address);
        ESP_LOGI(TAG, "Packet Size: %d bytes", params.packet_size_bytes);
        ESP_LOGI(TAG, "Baud Rate: %lu bps", params.baud_rate);
        ESP_LOGI(TAG, "========================");
    }
    
  /*  // Read module info
    uint8_t info_buf[256];
    size_t info_len;
    status = r305_read_info_page(fp_handle, info_buf, sizeof(info_buf), &info_len);
    if (status == R305_STATUS_OK) {
        ESP_LOGI(TAG, "Module info: %.*s", (int)info_len, info_buf);
    }*/
    
    // Get current template count
    uint16_t template_count;
    status = r305_get_template_count(fp_handle, &template_count);
    if (status == R305_STATUS_OK) {
        ESP_LOGI(TAG, "Enrolled fingerprints: %d", template_count);
        current_user_count = template_count;
    }
    
    // Initialize user database
    init_user_database();
    ESP_LOGI(TAG, "System initialization complete!");
  
    system_ready = true;
    return true;
}

/**
 * @brief Enroll new user
 */
static void enroll_new_user(void) {
  
    if (!system_ready) {
        ESP_LOGE(TAG, "System not ready");
        return;
    }
    
    // Find next available ID
    int template_id = find_next_free_id();
    if (template_id < 0) {
        ESP_LOGE(TAG, "Database full! Cannot enroll more users.");
        return;
    }
    
    // Get username (in production, get from user input)
    char username[MAX_USERNAME_LEN];
    snprintf(username, sizeof(username), "User_%d", template_id);
    
    ESP_LOGI(TAG, "\n");
   // ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "     NEW USER ENROLLMENT.....       ");
   // ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, " Template ID: %-5d                  ", template_id);
    ESP_LOGI(TAG, " Username:    %-20s                 ", username);
   // ESP_LOGI(TAG, "====================================");
    
    // Start enrollment
    r305_status_t status = r305_enroll_finger(fp_handle, template_id, 10000);
    if (status == R305_STATUS_OK) {
        add_user(template_id, username);
        
        ESP_LOGI(TAG, "\n");
        ESP_LOGI(TAG, "====================================");
        ESP_LOGI(TAG, "   ✓ ENROLLMENT SUCCESSFUL!         ");
        ESP_LOGI(TAG, "====================================");
        ESP_LOGI(TAG, " User '%s' enrolled                 ", username);
        ESP_LOGI(TAG, " Template ID: %d                    ", template_id);
        ESP_LOGI(TAG, " Total users: %d                    ", current_user_count);
        ESP_LOGI(TAG, "====================================");
    } else {
        ESP_LOGE(TAG, "\n");
        ESP_LOGE(TAG, "====================================");
        ESP_LOGE(TAG, "   ✗ ENROLLMENT FAILED              ");
        ESP_LOGE(TAG, "====================================");
        ESP_LOGE(TAG, " Error: %-27s                       ", r305_status_to_string(status));
        ESP_LOGE(TAG, "====================================");
    }
}

/**
 * @brief Verify fingerprint (1:N search)
 */
static void verify_fingerprint(void) {
  
    if (!system_ready) {
        ESP_LOGE(TAG, "System not ready");
        return;
    }
    
    if (current_user_count == 0) {
        ESP_LOGE(TAG, "No users enrolled yet!");
        return;
    }
    
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "   FINGERPRINT VERIFICATION         ");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, " Searching database...              ");
    ESP_LOGI(TAG, " Total users: %-5d                  ", current_user_count);
    ESP_LOGI(TAG, "====================================");
    
    r305_search_result_t result;
    r305_status_t status = r305_search_finger(fp_handle, &result, 10000);
    if (status == R305_STATUS_OK){
	    ESP_LOGI(TAG, "\n");
        ESP_LOGI(TAG, "====================================");
        ESP_LOGI(TAG, "   ✓ ACCESS GRANTED                 ");
        ESP_LOGI(TAG, "====================================");
        
        user_record_t *user = get_user(result.template_id);
        if(user){
        ESP_LOGI(TAG, " User: %-20s                        ", user->username);
       }else{
        ESP_LOGE(TAG, " User NOT FOUND                     " );  }
        
        
        ESP_LOGI(TAG, " Template ID: %-5d                  ", result.template_id);
        ESP_LOGI(TAG, " Confidence:  %-5d (%s)%-6s         ",result.confidence,r305_confidence_to_string(result.confidence), "");
        ESP_LOGI(TAG, "====================================");
        
        // In production: unlock door, log access, etc.
        
    } else if (status == R305_STATUS_NOT_FOUND) {
        ESP_LOGW(TAG, "\n");
        ESP_LOGW(TAG, "====================================");
        ESP_LOGW(TAG, "   ✗ ACCESS DENIED                  ");
        ESP_LOGW(TAG, "====================================");
        ESP_LOGW(TAG, " Fingerprint not recognized         ");
        ESP_LOGW(TAG, "====================================");
        
    } else {
        ESP_LOGE(TAG, "\n");
        ESP_LOGE(TAG, "====================================");
        ESP_LOGE(TAG, "   ✗ VERIFICATION ERROR             ");
        ESP_LOGE(TAG, "====================================");
        ESP_LOGE(TAG, " Error: %-27s                       ", r305_status_to_string(status));
        ESP_LOGE(TAG, "====================================");
    }
}

/**
 * @brief Match specific user (1:1 match)
 */
static void match_specific_user(uint16_t template_id) {
    if (!system_ready) {
        ESP_LOGE(TAG, "System not ready");
        return;
    }
    
    user_record_t *user = get_user(template_id);
    if (!user) {
        ESP_LOGE(TAG, "User ID %d not found", template_id);
        return;
    }
    
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔====================================╗");
    ESP_LOGI(TAG, "║   1:1 FINGERPRINT MATCH            ║");
    ESP_LOGI(TAG, "╠====================================╣");
    ESP_LOGI(TAG, "║ User:        %-20s ║", user->username);
    ESP_LOGI(TAG, "║ Template ID: %-5d                 ║", template_id);
    ESP_LOGI(TAG, "╚====================================╝");
    
    uint16_t confidence;
    r305_status_t status = r305_match_finger(fp_handle, template_id,  &confidence, 10000);
    
    if (status == R305_STATUS_OK) {
        ESP_LOGI(TAG, "\n");
        ESP_LOGI(TAG, "╔====================================╗");
        ESP_LOGI(TAG, "║   ✓ MATCH SUCCESSFUL               ║");
        ESP_LOGI(TAG, "╠====================================╣");
        ESP_LOGI(TAG, "║ Confidence: %-5d (%s)%-7s ║",
                 confidence,
                 r305_confidence_to_string(confidence), "");
        ESP_LOGI(TAG, "╚====================================╝");
    } else {
        ESP_LOGW(TAG, "\n");
        ESP_LOGW(TAG, "╔====================================╗");
        ESP_LOGW(TAG, "║   ✗ MATCH FAILED                   ║");
        ESP_LOGW(TAG, "╠====================================╣");
        ESP_LOGW(TAG, "║ Fingerprint does not match         ║");
        ESP_LOGW(TAG, "╚====================================╝");
    }
}

/**
 * @brief Delete user
 */
static void delete_user(uint16_t template_id) {
   
    if (!system_ready) {
        ESP_LOGE(TAG, "System not ready");
        return;
    }
    
    user_record_t *user = get_user(template_id);
    if (!user) {
        ESP_LOGE(TAG, "User ID %d not found", template_id);
        return;
    }
    
    ESP_LOGI(TAG, "Deleting user: %s (ID %d)", user->username, template_id);
    
    r305_status_t status = r305_delete_template(fp_handle, template_id);
    if (status == R305_STATUS_OK) {
        remove_user(template_id);
        ESP_LOGI(TAG, "User deleted successfully");
    } else {
        ESP_LOGE(TAG, "Failed to delete user: %s", r305_status_to_string(status));
    }
}

/**
 * @brief Clear all users
 */
static void clear_all_users(void){
    
    if (!system_ready){
        ESP_LOGE(TAG, "System not ready");
        return;
    }
    
    ESP_LOGW(TAG, "Clearing entire fingerprint database...");
    r305_status_t status = r305_clear_database(fp_handle);
    if(status == R305_STATUS_OK){
        init_user_database();
        current_user_count = 0;
        ESP_LOGI(TAG, "Database cleared successfully");
    } else {
        ESP_LOGE(TAG, "Failed to clear database: %s", r305_status_to_string(status));
    }
}

/**
 * @brief List all enrolled users
 */
static void list_users(void) {
  
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔================================================╗");
    ESP_LOGI(TAG, "║          ENROLLED USERS LIST                   ║");
    ESP_LOGI(TAG, "╠================================================╣");
    ESP_LOGI(TAG, "║ Total enrolled: %-5d                           ║", current_user_count);
    ESP_LOGI(TAG, "╠================================================╣");
   
    if (current_user_count == 0) {
        ESP_LOGI(TAG, "║ No users enrolled                              ║");
    } else {
        for (int i = 0; i < R305_MAX_TEMPLATES && current_user_count > 0; i++) {
            if (user_database[i].enrolled){
                ESP_LOGI(TAG, "║ ID: %-3d | %-35s ║",user_database[i].template_id,user_database[i].username);
            }
        }
    }
    
   ESP_LOGI(TAG, "╚================================================╝");
    
}

/* ==================== DEMO MODES ==================== */

/**
 * @brief Continuous authentication mode
 */
static void continuous_auth_mode(void) {
   
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔====================================╗");
    ESP_LOGI(TAG, "║   CONTINUOUS AUTHENTICATION MODE   ║");
    ESP_LOGI(TAG, "╠====================================╣");
    ESP_LOGI(TAG, "║ Place finger to verify identity    ║");
    ESP_LOGI(TAG, "║ System will continuously monitor   ║");
    ESP_LOGI(TAG, "╚====================================╝");
    
    while (true) {
        verify_fingerprint();
        vTaskDelay(pdMS_TO_TICKS(2000));  // Wait before next scan
    }
}

/**
 * @brief Demo sequence
 */
static void run_demo_sequence(void) {
    
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "    STARTING DEMO SEQUENCE");
    ESP_LOGI(TAG, "========================================");
    
    // Clear database
    ESP_LOGI(TAG, "\n[1] Clearing database...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    clear_all_users();
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Enroll 3 users
    	  ESP_LOGI(TAG, "\n Enrolling 3 users...");
    
    for (int i = 0; i < 3; i++) {
		
	ESP_LOGI(TAG, "[%d] Enroll users...",i + 1);
	
        vTaskDelay(pdMS_TO_TICKS(2000));
        enroll_new_user();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    
    // List users
    ESP_LOGI(TAG, "\n[3] Listing enrolled users...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    list_users();
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Continuous verification
    ESP_LOGI(TAG, "\n[4] Starting continuous verification...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    continuous_auth_mode();
}

/* ==================== MAIN APPLICATION ==================== */

void app_main(void) {

    vTaskDelay(pdMS_TO_TICKS(2000));
   
    if (!initialize_system()) {
        ESP_LOGE(TAG, "System initialization failed!");
        ESP_LOGE(TAG, "Please check connections and restart");
        return;
    }
    
    
    
    // Mode 1: Full automated demo
    run_demo_sequence();
    
    // Mode 2: Single enrollment
    // enroll_new_user();
    
    // Mode 3: Continuous verification
    // continuous_auth_mode();
    
 
    /*
  
    while (true) {
		
        ESP_LOGI(TAG, "\n=== MENU ===");
        ESP_LOGI(TAG, "1. Enroll new user");
        ESP_LOGI(TAG, "2. Verify fingerprint");
        ESP_LOGI(TAG, "3. Match specific user (ID 0)");
        ESP_LOGI(TAG, "4. List users");
        ESP_LOGI(TAG, "5. Delete user (ID 0)");
        ESP_LOGI(TAG, "6. Clear database");
        ESP_LOGI(TAG, "============");
        
        // In production, get user input here
        // For now, cycle through options
        static int option = 1;
        
        switch (option) {
			
            case 1: enroll_new_user(); break;
            case 2: verify_fingerprint(); break;
            case 3: match_specific_user(0); break;
            case 4: list_users(); break;
            case 5: delete_user(0); break;
            case 6: clear_all_users(); break;
        }
        
        option = (option % 6) + 1;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    */
}
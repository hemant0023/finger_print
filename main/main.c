



#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
//#include "arch/sys_arch.h"

//#include "freertos/idf_additions.h"
#include "stdbool.h"
#include "stdint.h"
#include "unistd.h"
#include <math.h>
#include <inttypes.h>
///ESP32
#include <esp_err.h>
#include "esp_event.h"
#include <esp_log.h>
#include "esp_system.h"
//#include <sys/param.h>
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
//#include "esp_heap_trace.h"

//wifi//////
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
//#include "esp_wifi_default.h"
//#include "esp_wifi_types_generic.h"

//#include "lwip/err.h"
//#include "lwip/sockets.h"
//#include "lwip/sys.h"
//#include "lwip/netdb.h"
//#include "lwip/dns.h"



static const char *WIFI_TAG = "WIFI";  
wifi_mode_t WIFI_MODE;
wifi_ap_record_t ap_info;
void wifi_init_sta(void);
void wifi_init_ap();
void wifi_init_ap_sta(void);
void WIFI_FALLBACK(char* MODE);

 bool WIFI_MODE_APSTA_STA_ENABLE = true;

uint16_t  STA_RETRY_COUNT = 0;


#define ESP_WIFI_SSID_STA_DEFAULT                 "SENSEWELL_INSTRUMENT"
#define ESP_WIFI_PASS_STA_DEFAULT                 "12345678"
bool ESP_WIFI_PASS_AP_ENABLE =true;
#define ESP_WIFI_SSID_AP_DEFAULT                  "DCN_WDL"
#define ESP_WIFI_PASS_AP_DEFAULT                  "HEMANT@WDL"

//server
esp_err_t ESP_ERR_RESULT;
esp_err_t err;

#define HTTP_SERVER_TAG "HTTP_SERVER"
//#include "http_parser.h"
#include <esp_http_server.h>
#include "esp_http_client.h"
#include "esp_tls_crypto.h"
//#include "esp_crt_bundle.h"
///TLS_CRYPTO
#include "esp_tls.h"
#include "cJSON.h"
#include <esp_https_server.h>
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#define HTTP_CLIENT_TAG "DCN_HTTP_CLIENT_REQUEST"
bool HTTP_CLIENT_RESPOND_DATA_FLAG =  false;
#define COMPRESS_GZIP_FILE 0
#define SPIFF_MAX_NVS_OPEN_READ_RETRIES 3
#define FILE_BUFFER_SIZE 1024 * 8
httpd_handle_t server = NULL;
httpd_handle_t HTTP_server = NULL;
extern const unsigned  char servercert_start[] asm("_binary_servercert_pem_start");
extern const unsigned  char servercert_end[]   asm("_binary_servercert_pem_end");
extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
extern const unsigned char prvtkey_pem_end[]   asm("_binary_prvtkey_pem_end");
extern const unsigned  char sbc_cert_start[]   asm("_binary_sbc_cert_pem_start");
extern const unsigned  char sbc_cert_end[]     asm("_binary_sbc_cert_pem_end");

void   STOP_START_HTTP_SERVER(httpd_handle_t *server, bool STOP_SERVER, bool RESTART_SERVER);
char * http_auth_basic(const char *username, const char *password);
void    REQUEST_HEADER( httpd_req_t *req);
esp_err_t   serve_file(httpd_req_t *req, const char *path, const char *type, bool COMPRESS_GZIP);
int  PERFORM_HTTP_REQUEST(uint8_t HTTP_REQUEST_MAX_RETRIES,const char *url,  esp_http_client_method_t http_method , const char *post_data, const char * content_type, const char **headers, int num_headers);
void SEND_DCN_INFO_SBC(const char SBC_IP[16]);
void HTTP_disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void HTTP_connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
#define     MAX(a,b) ((a) > (b) ? (a) : (b))
#define     MIN(a,b) ((a) < (b) ? (a) : (b))
esp_err_t   FETCH_HTTP_POST_CONTENT(httpd_req_t *req, char **content_out);
static char *HTTP_POST_CONTENT = NULL;


///freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

///uart///////
#include "driver/uart.h"
///GPIO///////
#include "driver/gpio.h"
#define ESP_INTR_FLAG_DEFAULT 0
//I2C
#include "driver/i2c.h"
#include <driver/i2c_master.h>
bool  I2C_INIT_FLAG = true;
#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22

#define I2C_MASTER_FREQ_HZ 100000  // 100kHz
#define I2C_PORT I2C_NUM_0

//#define TCP_OVERSIZE  1
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master does not need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0



///NVS
#include <nvs_flash.h>
#include <nvs.h>
//NVS_HANDLE
nvs_handle_t JSON_BODY_NVS_HANDLE;
nvs_handle_t NVS_SOFT_SETTING_HANDLE;
#define NVS_MAX_RETRIES  3
char error_msg[40];
char SYSTEM_ERROR_MSG[40];


//RTC
#include <time.h>
#include <sys/time.h>
#define RTC_TIME_TAG  "RTC TIME"
//#include "lwip/apps/sntp.h"
esp_err_t  sbc_response_DATE_TIME(const char *response);
cJSON * DCN_INFORMATION_GET_HANDLER();
void format_datetime(const char *datetime_str);
void set_time(int year, int month, int date, int hour, int min, int sec);
time_t DATE_TIME_TO_SEC(const char *datetime_str);
time_t   CURRENT_TIME= 0;
char  TIME_buffer[12];
char  DATE_buffer[30];
char  DATE_TIME_STR[45];
bool mcp79410_RTC_INIT_FLAG = false;
#define MCP79410_ADDR 0x6F 
#define MAX_RETRIES 3  // Number of retry attempts

//MDNS 
//#include "mdns.h"
#define SBC_HOST_NAME           "sensewell"
#define SBC_INSTANCE_NAME       "SBC-server"
#define SERVICE_TYPE            "_server"
#define SERVICE_PROTOCOL        "_tcp"
#define IP_ADDR_STR_MAX_LEN      16
#define MDNS_TAG                 "MDNS"
bool  MDNS_SEARCH_FLAG =         false;
bool  STATION_IP_FLAG  =         false;
static bool mdns_initialized =   false;
#define MDNS_MONITOR_INTERVAL_MS 10000  
#define MAX_MDNS_RETRIES 3 
char Final_IP[40];
const char* TAG = "--";
void start_mdns_service();


///SD CARD  (fatfs32)
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <dirent.h>
#include "esp_spiffs.h"
#define MOUNT_POINT "/sdcard"
esp_err_t STORE_JSON_BODY_TO_NVS(const char *nvs_namespace, const char *key_length, const char *key_content, int32_t content_length, const char *content, httpd_req_t *req);
esp_err_t READ_JSON_BODY_FROM_NVS(const char *nvs_namespace, const char *key_length, const char *key_content, char * *JSON_BODY_NVS_CONTENT, httpd_req_t *req);

/////MDNS 
enum{
	
    WIFI_STARTED,
    IP_GET,
    FIND_SBC,
    SEND_ESP_DATA,
    CONNECT_MQTT,
    MQTT_CONNECTED,
    AP_STA_WIFI_FALLBACK,
    SEND_MQTT_DATA,
  
};



///new SENSWELL kit
#define UART_NUM   UART_NUM_2

#define TXD_PIN    GPIO_NUM_17
#define RXD_PIN    GPIO_NUM_16
#define DE_PIN     GPIO_NUM_18
#define RE_PIN     GPIO_NUM_5
//HSPI 20000KHZ
#define PIN_NUM_MISO  GPIO_NUM_27
#define PIN_NUM_MOSI  GPIO_NUM_13
#define PIN_NUM_CLK   GPIO_NUM_14
#define PIN_NUM_CS    GPIO_NUM_26


/// INPUT/OUTPUT
#define OUT_PIN_1     GPIO_NUM_25 //#INPUT_OUTPUT
#define OUT_PIN_2     GPIO_NUM_33 //#INPUT_OUTPUT
#define OUT_PIN_3     GPIO_NUM_32 //#INPUT_OUTPUT

// Input pin ONLY 
    // GPIOs 34 to 39 are GPIs – input only pins. These pins don’t have internal pull-up or pull-down resistors. 
   // They can’t be used as outputs, so use these pins only as inputs:
#define IN_PIN_1      GPIO_NUM_35  //#INPUT_ONLY
#define IN_PIN_2      GPIO_NUM_34   //#INPUT_ONLY
#define IN_PIN_3      GPIO_NUM_36    //#INPUT_ONLY
#define IN_PIN_4      GPIO_NUM_39    //#INPUT_ONLY


#define OUT_PIN_4     GPIO_NUM_4
#define OUT_PIN_5     GPIO_NUM_15 //#LOW DURING FLASHING BOOT ( strapped GPIOs)
#define OUT_PIN_6     GPIO_NUM_2  //#LOW DURING FLASHING BOOT ( strapped GPIOs)
#define OUT_PIN_7     GPIO_NUM_19
//#define OUT_PIN_8     GPIO_NUM_23

//#define OUT_PIN_9    GPIO_NUM_0 //#LOW DURING FLASHING BOOT ( strapped GPIOs)
//#define OUT_PIN_10    GPIO_NUM_12  //#boot fails if pulled high, strapping pin( strapped GPIOs)




#define EVENT_GROUP_TAG  "EVENT_GROUP"
ESP_EVENT_DEFINE_BASE(MAIN_EVENT_BASE);

TaskHandle_t WATCHDOG_TASK_HANDLE = NULL;
TaskHandle_t MQTT_TASK_HANDLE = NULL;




/////SBC_INFORMATION 
#define IP_ADDR_STR_MAX_LEN 16
#define SBC_IP_HEAD "http://"
#define SBC_IP_TAIL ":5000/api/addesp"
char SBC_IP[IP_ADDR_STR_MAX_LEN];


//// PASSWORD
#define SV_AUTH_PASSWORD "SENSEWELL@WDL"

////MQTT
#include "mqtt_client.h"
esp_mqtt_client_handle_t client = NULL;
bool MQTT_STOP_FLAG = false;
bool MQTT_INNITIALIZED_FLAG = false;
bool MQTT_CONNECTED_FLAG = false;
int16_t MQTT_MESSAGE_ID = 0;
#define BROKER_URL_MAX_LEN 30
#define MAX_RECONNECT_ATTEMPTS 5 
#define MQTT_PORT 1883
#define MQTT_TLS_PORT 8883
uint8_t  reconnect_retry = 0;
char BROKER_URL[BROKER_URL_MAX_LEN];
char  MQTT_TOPIC_KEY[50];   
void INITAILZE_MQTT();
void MQTT_RESPONSE_SEND(const char* response_data);
uint16_t PUBLISH_MQTT_DATA(const char *topic, const char *data,uint8_t QOS);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);






//////////SD CARD
#define  MAX_SD_CARD_RETRY_COUNT   3  
#define  MAX_SD_WRITE_READ_RETRIES 2 
#define SD_CARD_FILE_RETRY_MS    200
#define SD_CARD_RETRY_DELAY_MS   500  // Delay between retries in milliseconds
static const char *SD_TAG = "SD_CARD_INIT";
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
sdmmc_card_t *card = NULL;
bool SD_CARD_INITALIZE_SUCCESS_FLAG = false;
bool FILE_WRITING_READING_FLAG =  false;
esp_err_t LOG__FILE_FOLDER_SDCARD(uint8_t SLAVE_POS ,const char *DCN_MAIN_FOLDER,const  char *SLAVE_MAIN_FOLDER, const char *SUB_FOLDER_NAME, const char *FILE_NAME, const char *CREATE_DATE_TIME, const char *company_make, const char *make_model);
void LOG_CSV_REPORT_DATA_IN_SDCARD(uint8_t SLAVE_POS, time_t REPORT_TIME,time_t CURRENT_TIME_REPORT , bool force_write);
esp_err_t delete_directory(const char *path);
esp_err_t rename_file(const char *old_path,const char *old_name, const char *new_name);
 time_t CURRENT_TIME_REPORT= 0;
bool REPORT_TRANSMISSION_STATUS_FLAG = false;
bool REPORT_LOG_STARTING_FLAG = false;






uint8_t  COMMUNICATION_ERROR =  0;
char     MODBUS_ERROR_CHECK_STR[64];
char     PID_ERROR_CHECK_STR[64];

bool     RECIEVE_VALID_DATA_FLAG    = false;
bool     RESPONSE_SUCCESS_FLAG      = false;
bool     COMMUNICATION_EXECUTE_FLAG = false;
bool     SETTING_TRANSMISSION_STATUS_FLAG = true;
bool     MQTT_REPORT_TASK_FLAG =  true;
bool     INITIAL_SETTING_READ  = false;

uint8_t  PID_SETTING_READ_SUCCESS = 0;
uint8_t  SLAVE_DEVICE_COUNT =   0;
uint8_t   REGISTER_FUNCTION        =  0x00;
uint8_t   REGISTER_QUANTITY        =  0X00;
uint8_t   CHANNEL_ADDRESS_MSB      =  0X00;
uint8_t   CHANNEL_ADDRESS_LSB      =  0X00;
uint16_t  WRITE_ADDRESS_REGISTER   =  0X0000;
uint16_t  CHANNEL_STARTING_ADDRESS =  0X0000;
uint8_t   SETTING_PARAMETER_POSITION = 0;
uint8_t   DATA_LENGTH=0;
uint8_t  READ_FUNCTION_STATUS= 0;

bool     SETTING_PARAMETER_READ_MEMORY_INITIALIZED_FLAG = true;
uint32_t   SLAVE_DEVICE_SCAN_COUNT =0;
bool     INITIAL_READ_MODEL_EXECUTION_DONE_FLAG[20]= { false };
bool     SLAVE_INITIAL_READ_FLAG[20] = {1};
bool     SLAVE_INITIAL_WRITE_FLAG[20] = {true};
#define  MAX_CHANNELS_ALLOWED   100
#define MAX_SLAVE_DEVICE_ALLOWED 100
 char  SLAVE_FOLDER_NAME[40] ="TEST_DCN";
 char DCN_FOLDER_NAME[40] ;

//function 
bool START_SV_POINT_TASK_STATUS = false;
#define EXTRACT_AND_SCALE(high, low,scale) (((high) << 8) | (low))

///DEBUGGING FUNCTION 
#define REPORT_DEBUG 0
#define UART_DEBUG 0
#define PRINT_DEBUG  0
#define SYSTEM_DEBUG 1 
#define MQTT_DEBUG 0
#define PROFILE_DEBUG 0
esp_event_loop_handle_t main_event_handle;
void print_task_info(TaskHandle_t taskHandle, const char *taskName) ;
void HEAP_STACK_MEMORY_PRINT(char* FUNCTION_NAME);




#include "attendance_system.h"
#include "r305_fingerprint.h"
#include "rc522_rfid.h"
#include "LCD_DISPLAY.h"
#include "buzzer.h"



#define NVS_NAMESPACE "attendance"
#define NVS_KEY_USERS "users"
#define NVS_KEY_ATTENDANCE "attendance"
#define NVS_KEY_SESSIONS "sessions"
#define NVS_KEY_CONFIG "config"
#define NVS_KEY_STATS "stats"


/* System status flags */
typedef struct {
    bool nvs_ready;
    bool wifi_ready;
    bool lcd_ready;
    bool IDLE_DISPLAY_PRINT;
    bool buzzer_ready;
    bool database_ready;
    bool fingerprint_ready;
    bool rfid_ready;
    bool webserver_ready;
} system_status_t;

static system_status_t system_status = {0};
/* ==================== GLOBAL HANDLES ==================== */

static r305_handle_t *fp_handle = NULL;
static rc522_handle_t *rfid_handle = NULL;


static bool system_ready = false;

// Global arrays (loaded from NVS at startup)
static user_record_t user_database[MAX_USERS];
static attendance_record_t attendance_records[MAX_ATTENDANCE_RECORDS];
static session_info_t session_history[MAX_SESSIONS];
static session_info_t current_session = {0};
static system_stats_t system_stats = {0};

// Counters
static uint16_t total_users = 0;
static uint32_t total_records = 0;
static uint16_t total_sessions = 0;
uint16_t CURRENT_R305_TEMPLATE_COUNT =0;

esp_err_t api_fingerprint_enroll_handler(httpd_req_t *req);
esp_err_t api_rfid_enroll_handler(httpd_req_t *req) ;
esp_err_t api_fingerprint_clear_database_handler(httpd_req_t *req);
esp_err_t fingerprint_delete_template_id(uint16_t template_id); 



esp_err_t api_reset_clear_database_handler(httpd_req_t *req);
esp_err_t api_clear_attendance_handler(httpd_req_t *req);
   



esp_err_t  api_system_data_handler(httpd_req_t *req) ;
esp_err_t  api_user_add_handler(httpd_req_t *req) ; 
esp_err_t  api_user_update_handler(httpd_req_t *req) ;
esp_err_t  api_user_delete_handler(httpd_req_t *req) ;
esp_err_t  api_session_start_handler(httpd_req_t *req) ;
esp_err_t  api_session_stop_handler(httpd_req_t *req) ;



 void print_task_info(TaskHandle_t taskHandle, const char *taskName) {
	 
	 if(taskHandle == NULL){
		 return;
	 }
	 
    eTaskState state = eTaskGetState(taskHandle);
    

    ESP_LOGI("TASK INFO","Task: %s | State: ", taskName);

    switch (state) {
        case eRunning:
            ESP_LOGI("TASK INFO","RUNNING\n");
            break;
        case eReady:
            ESP_LOGI("TASK INFO","READY\n");
            break;
        case eBlocked:
            ESP_LOGI("TASK INFO","BLOCKED (Waiting for Event/Semaphore/Delay)\n");
            break;
        case eSuspended:
            ESP_LOGI("TASK INFO","SUSPENDED (Manually Paused)\n");
            break;
        case eDeleted:
            ESP_LOGI("TASK INFO","DELETED (Not Cleaned Up Yet)\n");
            break;
        default:
            ESP_LOGI("TASK INFO","UNKNOWN STATE\n");
            break;
    }

}

#define MIN_FREE_HEAP 30000  // Minimum 30KB free heap required

static bool check_heap_available(size_t required_size, const char *operation) {
    size_t free_heap = esp_get_free_heap_size();
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    
    if (free_heap < MIN_FREE_HEAP || largest_block < required_size) {
    //   ESP_LOGE(TAG, "Insufficient heap for %s: free=%llu, largest=%llu, required=%llu",operation, free_heap, largest_block, required_size);
     ESP_LOGE(TAG, "Insufficient heap for %s",operation);
        return false;
    }
    
  //  ESP_LOGD(TAG, "Heap OK for %s: free=%llu, largest=%llu, required=%llu",operation, free_heap, largest_block, required_size);
  ESP_LOGD(TAG, "Heap OK for %s: ",operation);
    return true;
}


void HEAP_STACK_MEMORY_PRINT(char* FUNCTION_NAME){
	
	 if(PRINT_DEBUG){
	    ESP_LOGW("FREE STACK", "%s  STACK: %u\n", FUNCTION_NAME ,uxTaskGetStackHighWaterMark(NULL));
        ESP_LOGW("FREE HEAP","%s    HEAP: %lu\n", FUNCTION_NAME , esp_get_free_heap_size());
        ESP_LOGW("HEAP BLOCK", "Largest free block: %u bytes",  heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
        ESP_LOGW("LOWEST HEAP", "Lowest ever free heap: %u bytes",  heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
     
        multi_heap_info_t info;
        memset(&info, 0, sizeof(info)); // Zero the structure first (optional if you memset inside get_info)
        heap_caps_get_info(&info, MALLOC_CAP_DEFAULT);
        ESP_LOGW("HEAP BLOCK","TOTAL Free: %u bytes \n Largest block: %u bytes \n   MIN free %u bytes\n",info.total_free_bytes, info.largest_free_block,info.minimum_free_bytes);
        ESP_LOGW("HEAP BLOCK","TOTAL BLOCK: %u  \n  Allocated blocks: %u  \n Free blocks: %u  Bytes \n in use: %u\n",info.total_blocks ,info.allocated_blocks,info.free_blocks,info.total_allocated_bytes);
        

   
  //  heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);

     if(!heap_caps_check_integrity(MALLOC_CAP_DEFAULT, true)) {
        printf("Heap corruption detected! \n");
      }else{
        printf("Heap is OK \n");
        }
        
        if(!heap_caps_check_integrity_all( true)) {
        printf("ALL CAPACITY FLAG CHECK Heap corruption detected! \n");
      }else{
        printf("ALL CAPACITY FLAG CHECK Heap is OK \n");
        }
        
       
       // heap_caps_dump(MALLOC_CAP_DEFAULT);
        
       // heap_caps_dump_all();
        
        
        
        }
        
}


 
union{
	

    float    val_float;
    uint32_t val_long;
    uint16_t val_int[2];
    uint8_t  val_char[4];

} mem_val;


typedef struct {

    char AUTOMATION_LOCAL_CREATE_DATE_TIME[30];
    char AUTOMATION_LOCAL_CREATE_DATE[15];
    char AUTOMATION_LOCAL_CREATE_TIME[15];
    
    char AUTOMATION_DEVICE_NAME[32]; 
    char AUTOMATION_PROTOCOL[15];
   
    int16_t AUTOMATION_STORAGE_COUNT;
    int16_t AUTOMATION_STORAGE_RATE;
    uint16_t AUTOMATION_SCAN_RATE;
    
     
    char AUTOMATION_BAUD_RATE[10];
    char AUTOMATION_PARITY[10];
    char AUTOMATION_STOP_BITS[5];
    char AUTOMATION_DATA_BITS[5];
    
    char    SLAVE_FOLDER_NAME[32];
    char    DCN_FOLDER_NAME[32];
    
    char  DCN_RESET_REASON_STR[20];
    
}DEVICE_LOGGER_STRUCT;
 
DEVICE_LOGGER_STRUCT DEVICE_LOGGER;


struct NVS_STORAGE_STRUCT{

    char ESP32_MAC_ADDRESS_STR[20];
	char SERIAL_NUMBER_STRING [20];
    char DEVICE_NAME_NVS_STR  [32];
    char DCN_IP_ADDRESS[20];
    char DCN_APP_VERSION[6];
    char DCN_HAR_VERSION[6];
};

struct NVS_STORAGE_STRUCT  NVS_STORAGE;



/*esp_err_t i2c_master_init(){
	
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE("I2C_INIT", "I2C configuration failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE("I2C_INIT", "I2C driver installation failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("I2C_INIT", "I2C initialized successfully");
    }
    return err;
}*/



esp_err_t mcp79410_write(uint8_t reg, uint8_t value){
	
    uint8_t data[2] = {reg, value};
    esp_err_t err = i2c_master_write_to_device(I2C_PORT, MCP79410_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
    
    if (err != ESP_OK) {
        ESP_LOGE(RTC_TIME_TAG, "Failed to write to MCP79410 (Reg: 0x%02X): %s", reg, esp_err_to_name(err));
    }
    return err;
}


esp_err_t mcp79410_read(uint8_t reg, uint8_t *data, size_t len){
	
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, MCP79410_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
    
    if (err != ESP_OK) {
        ESP_LOGE(RTC_TIME_TAG, "Failed to read from MCP79410 (Reg: 0x%02X): %s", reg, esp_err_to_name(err));
    }
    return err;
}


esp_err_t mcp79410_enable_oscillator() {
	
	if(I2C_INIT_FLAG == false){
        ESP_LOGE("mcp79410_read_time", "I2C Initialized failed");
         return ESP_FAIL;
   }
   
    uint8_t sec;
    
    err = mcp79410_read(0x00, &sec, 1);
    
    if (err != ESP_OK) {
        ESP_LOGE(RTC_TIME_TAG, "Error reading seconds register for oscillator enable");
        return err;
    }

    if (!(sec & 0x80)) {  // If oscillator is off, enable it
        err = mcp79410_write(0x00, sec | 0x80);
        if (err == ESP_OK) {
            ESP_LOGI(RTC_TIME_TAG, "RTC Oscillator Enabled");
        }
    } else {
        ESP_LOGI(RTC_TIME_TAG, "RTC Oscillator Already Enabled");
    }
    return err;
}


uint8_t bcdToDec(uint8_t value)
{
     return ((value/16) * 10+(value%16));
}


uint8_t int_to_bcd(uint8_t value)
{
	return (((value/10) * 16) + (value%10));
}

esp_err_t mcp79410_set_time(int second, int minute, int hour, int day, int date, int month, int year){
	
    if(I2C_INIT_FLAG == false){
        ESP_LOGE("mcp79410_read_time", "I2C Initialized failed");
         return ESP_FAIL;
   }

    // Convert integer values to BCD
    uint8_t bcd_second = int_to_bcd(second) | 0x80; // Start oscillator (bit 7)
    uint8_t bcd_minute = int_to_bcd(minute) & 0x7F;
    uint8_t bcd_hour = int_to_bcd(hour) & 0x3F; //12 HR// PM
    uint8_t bcd_day = (int_to_bcd(day) & 0x07) | 0x08; // Vbat enable bit
    uint8_t bcd_date = int_to_bcd(date) & 0x3F;
    uint8_t bcd_month = int_to_bcd(month) & 0x1F;
    uint8_t bcd_year = int_to_bcd(year); // Convert to 2-digit format

    // Write each register using mcp79410_write()
    if ((err = mcp79410_write(0x00, bcd_second)) != ESP_OK) return err;
    if ((err = mcp79410_write(0x01, bcd_minute)) != ESP_OK) return err;
    if ((err = mcp79410_write(0x02, bcd_hour)) != ESP_OK) return err;
    if ((err = mcp79410_write(0x03, bcd_day)) != ESP_OK) return err;
    if ((err = mcp79410_write(0x04, bcd_date)) != ESP_OK) return err;
    if ((err = mcp79410_write(0x05, bcd_month)) != ESP_OK) return err;
    if ((err = mcp79410_write(0x06, bcd_year)) != ESP_OK) return err;

    ESP_LOGI(RTC_TIME_TAG, "RTC time successfully set.");
    return ESP_OK;
    
    
}


esp_err_t mcp79410_read_time(){
	
	if(I2C_INIT_FLAG == false){
        ESP_LOGE("mcp79410_read_time", "I2C Initialized failed");
         return ESP_FAIL;
   }
	
	
    uint8_t time_data[7];

   
    err = mcp79410_read(0x00, time_data, 7);
    if (err != ESP_OK) {
        ESP_LOGE(RTC_TIME_TAG, "Failed to read time from MCP79410");
        return err;
    }

    // Convert BCD to decimal and apply bit masking where needed
    uint8_t sec   = bcdToDec(time_data[0] & 0x7F);  // Mask oscillator bit
    uint8_t min   = bcdToDec(time_data[1] & 0x7F);
    uint8_t hour  = bcdToDec(time_data[2] & 0x3F);
    uint8_t day   = bcdToDec(time_data[3] & 0x07);
    uint8_t date  = bcdToDec(time_data[4] & 0x3F);
    uint8_t month = bcdToDec(time_data[5] & 0x1F);
    int  year  = 2000 + bcdToDec(time_data[6]); // Convert to four-digit year

    // Log the retrieved time
    ESP_LOGI(RTC_TIME_TAG, "Time Set: %02d:%02d:%02d, day:%d Date: %02d/%02d/%04d", hour, min, sec, day,date, month, year);
     //printf("DAY: %02d\n", day);
    
    set_time( year,  month,  date,  hour,  min,  sec);
    return ESP_OK;
    
    
}
 void MCP79410_RTC_IC_INIT(){


     if(I2C_INIT_FLAG == false){
	    ESP_LOGE("MCP79410_RTC_IC_INIT", "I2C Initialization failed. Exiting...");
	    return;
	}
      vTaskDelay(pdMS_TO_TICKS(5));
      
    if (mcp79410_enable_oscillator() != ESP_OK) {
        ESP_LOGE(RTC_TIME_TAG, "RTC Oscillator Enable failed. Exiting...");
       
    }
     vTaskDelay(pdMS_TO_TICKS(5));
 
    int retry_count = 0;
    while (retry_count < MAX_RETRIES){
        err = mcp79410_read_time();
        if (err == ESP_OK) {
            ESP_LOGI(RTC_TIME_TAG, "RTC time read successfully.");
            mcp79410_RTC_INIT_FLAG = true;
            return;  // Exit function if successful
        }
        ESP_LOGE(RTC_TIME_TAG, "RTC time read failed. Retrying... (%d/%d)", retry_count + 1, MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(300));  // Wait before retrying
        retry_count++;
    }

    // If all retries failed, set the RTC flag to false
    ESP_LOGE(RTC_TIME_TAG, "RTC time read failed after %d retries. Exiting...", MAX_RETRIES);
    mcp79410_RTC_INIT_FLAG = false;
    
}
 
 
 void set_time(int year, int month, int date, int hour, int min, int sec){

	struct tm timeinfo = {
		
        .tm_year = year - 1900, // tm_year is the number of years since 1900
        .tm_mon  = month - 1,   // tm_mon is 0-based (0 = January)
        .tm_mday = date,
        .tm_hour = hour,
        .tm_min  = min,
        .tm_sec  = sec,
        
    };

	time_t now = mktime(&timeinfo); // Convert struct tm to time_t
    if (now == -1) {
            //printf("mktime failed\n");
           return ;
       }

    struct timeval now_tv = { .tv_sec = now, .tv_usec = 0 };
    settimeofday(&now_tv, NULL);    // Set the system time

    ESP_LOGI(RTC_TIME_TAG, "Time manually set to: %s", asctime(&timeinfo));

 
     snprintf(TIME_buffer, sizeof(TIME_buffer),"%02d:%02d:%02d",timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
     snprintf(DATE_buffer,sizeof(DATE_buffer),"%d-%02d-%02d",timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
     sprintf(DATE_TIME_STR,"%s-%s",DATE_buffer,TIME_buffer);
     
       //printf("SYSTEM FORMATED TIME : %s \n",TIME_buffer);
       //printf("SYSTEM FORMATED DATE :%s  \n", DATE_buffer);

     
      /* //printf("SYSTEM FORMATED TIME =%02d:%02d:%02d \n",timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
       //printf("SYSTEM FORMATED DATE =%02d-%02d-%02d \n",  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
      */
      
}
 time_t DATE_TIME_TO_SEC(const char *datetime_str){
	
    int year, month, date, hour, minute, second;
    sscanf(datetime_str, "%d-%d-%dT%d:%d:%d", &year, &month, &date, &hour, &minute, &second);

    struct tm t = {0};
    t.tm_year = year - 1900; // tm_year is years since 1900
    t.tm_mon  = month - 1;   // tm_mon is 0-based (0 = Jan)
    t.tm_mday = date;
    t.tm_hour = hour;
    t.tm_min  = minute;
    t.tm_sec  = second;

    // convert to time_t (epoch seconds, local time)
    time_t epoch_seconds = mktime(&t);
     if (epoch_seconds== -1) {
          ESP_LOGW("DATE_TIME_TO_SEC","DATE_TIME_TO_SEC FAILED %s",datetime_str);
           return 0;
       }

    /*struct timeval now_tv = { .tv_sec = epoch_seconds, .tv_usec = 0 };
    settimeofday(&now_tv, NULL);    // Set the system time
*/
    ESP_LOGI(RTC_TIME_TAG, "Time manually set to: %s", asctime(&t));

 
    printf("DATE_TIME_TO_SEC Epoch Seconds: %llu\n",epoch_seconds);
    printf("DATE_TIME_TO_SEC Formatted DateTime: %02d/%02d/%02d %02d:%02d:%02d\n",date, month, year, hour, minute, second);
    printf("DATE_TIME_TO_SEC UPDATED Time: %02d:%02d:%02d \n",t.tm_hour, t.tm_min, t.tm_sec);
    printf("DATE_TIME_TO_SECUPDATED Date: %02d-%02d-%d \n",t.tm_mon + 1,t.tm_mday,t.tm_year + 1900);
  

    return epoch_seconds;
}

char* seconds_to_hms_STR(time_t seconds, size_t str_size){
	
	 static char hms_str[10];
   
    if (str_size < 9) return NULL;
    struct tm timeinfo;
    localtime_r(&seconds, &timeinfo);  // Convert time_t to broken-down local time
    snprintf(hms_str, str_size, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return hms_str;
    
}


void format_datetime(const char *datetime_str){

 int year, month, date, hour, minute, second;
 sscanf(datetime_str, "%d-%d-%dT%d:%d:%d", &year, &month, &date, &hour, &minute, &second);

  ESP_LOGW(RTC_TIME_TAG,"Formatted DateTime: %02d/%02d/%02d %02d:%02d:%02d\n",date ,month,year,hour,minute,second);

  if(mcp79410_RTC_INIT_FLAG == true){   	 
  err=mcp79410_set_time( second, minute,  hour, 1, date , month, year - 2000);
  if (err == ESP_OK) {
	  vTaskDelay(pdMS_TO_TICKS(100));
         mcp79410_read_time();
         
    }else if(err != ESP_OK){
		
        ESP_LOGE(RTC_TIME_TAG, "RTC Time Set failed. Exiting...");
      
    }
    
  }else{
	  
	  set_time( year,  month,  date, hour,minute , 0);
        //printf("date - time :%s--%s \n",DATE_buffer,TIME_buffer);
       
       }
	
}

 
void time_update_task(void *pvParameters){
	
   
    struct tm timeinfo;
    while (1){
		
        time(&CURRENT_TIME);
        localtime_r(&CURRENT_TIME, &timeinfo);
         snprintf(TIME_buffer, sizeof(TIME_buffer),"%02d:%02d:%02d",timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
         snprintf(DATE_buffer,sizeof(DATE_buffer), "%d/%02d/%02d",timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,timeinfo.tm_mday);
         snprintf(DATE_TIME_STR,sizeof(DATE_TIME_STR),"%s-%s",DATE_buffer,TIME_buffer);
          //printf("TIME_buffer :%s \n",TIME_buffer);
          //printf("DATE_buffer :%s \n ",DATE_buffer);
 
  /*
  if(PRINT_DEBUG){
	  
    if(MODBUS_TASK_HANDLE){
    print_task_info(MODBUS_TASK_HANDLE, "SELECT_CONNECTION_TYPE");
    }
    
    if(PIDProfileTaskHandle){
    print_task_info(PIDProfileTaskHandle, "PIDProfileTaskHandle");
    }
    
    if(MQTT_TASK_HANDLE){
    print_task_info(MQTT_TASK_HANDLE, "MQTT_TASK_HANDLE");
    }
   }//PRINT_DEBUG*/
   
   
   uint64_t free_heap_size =  esp_get_free_heap_size();
   ESP_LOGW(TAG, "FREE HEAP: %llu bytes \n",free_heap_size);
   
     if (free_heap_size < 30000){
      
      ESP_LOGE(TAG, "Heap critically low! less 30000 ESP32 FREE heap : %llu bytes \n",free_heap_size);
       
       if(free_heap_size < 10000){
		 ESP_LOGE(TAG, "Heap critically low! REBOOTING DEVICE : %llu bytes \n",free_heap_size);
       esp_restart();
       }
    }
     
      vTaskDelay(pdMS_TO_TICKS(300));
    }
}





struct WIFI_CREDIENTIAL_STRCT{
    
    char AP_SSID_AP_MODE[32];
	char AP_PASSWORD_AP_MODE[32];
	
	char AP_SSID_STA_MODE[32];
	char AP_PASSWORD_STA_MODE[32];
	
    char MAC_ADDRESS_STR[20];
    char ESP_SERIAL_NUMBER_STR[20];
   
     char HOST_NAME_STR[20];
     char STA_IP_STR[20];
     char AP_IP_STR[20];
     char AP_LOCAL_IP_STR[20];
     
	
	int16_t STA_AUTO_SWITCHING_SCAN_RATE;
	bool STA_AUTO_SWITCHING;
	char STATIC_IP[20];
	char GATEWAY_IP[20];
	bool STATIC_DYNAMIC_IP;
	char RSSI_EVA_STR[20];
	int RSSI_DBI;
	int RSSI_PER;
	uint8_t CHANNEL_AP;
	uint8_t MAX_STA_CONN_AP;
	uint8_t MAX_AP_LIST;
	uint8_t MAX_CONN_RETRY;
	
};

struct WIFI_CREDIENTIAL_STRCT WIFI_CREDENTIAL;



int rssi_to_percent(int rssi) {
    // Typical range: -100 (very weak) to -50 (excellent)
    int percent = 2 * (rssi + 100); // Map -100 to 0%, -50 to 100%
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}

// Evaluate signal quality as text
const char* evaluate_rssi(int rssi) {
	
    if (rssi >= -45)    
     return "Excellent";
    else if (rssi >= -60) 
    return "Very Good";
    else if (rssi >= -65)
     return "Good";
    else if (rssi >= -75) 
    return "Fair";
    else if (rssi >= -80) 
    return "Poor";
    else return "Very Poor / Unusable";
}


static void wifi_event_handler_sta(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){

       const char*  TAG = "WIFI EVENT STA";
   
	  
	if ( event_base == WIFI_EVENT){


        if (event_id == WIFI_EVENT_STA_START){
			  STA_RETRY_COUNT = 0; 
			  ESP_LOGI(TAG, "Wi-Fi STA Started");
			  esp_wifi_connect();
			
        }else if(event_id == WIFI_EVENT_STA_DISCONNECTED){
			
			 ESP_LOGW(TAG, "Wi-Fi STA Disconnected!");
			 wifi_event_sta_disconnected_t *AP = (wifi_event_sta_disconnected_t *) event_data;
             ESP_LOGI(TAG, "Disconnected from SSID: %.*s", AP->ssid_len, (char *)AP->ssid);
             ESP_LOGI(TAG, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x",AP->bssid[0], AP->bssid[1], AP->bssid[2],AP->bssid[3], AP->bssid[4], AP->bssid[5]);
             ESP_LOGI(TAG, "Reason: %d", AP->reason);
             ESP_LOGI(TAG, "RSSI before disconnection: %d", AP->rssi);
       
          
      
			 if (MQTT_INNITIALIZED_FLAG && client != NULL && !MQTT_STOP_FLAG){
	             
	             ESP_LOGW(TAG, "FOURCE ESP MQTT STOP");
                 esp_mqtt_client_stop(client);
                 esp_mqtt_client_unregister_event( client, ESP_EVENT_ANY_ID,mqtt_event_handler);
                 MQTT_STOP_FLAG =  true;
                 MQTT_CONNECTED_FLAG  = false;
              
              }
       
                 if (STA_RETRY_COUNT < WIFI_CREDENTIAL.MAX_CONN_RETRY){
		              
		               esp_wifi_connect();
		               STA_RETRY_COUNT++;
		               ESP_LOGI(TAG, "Retry to connect to  AP [%d/%d] ",STA_RETRY_COUNT,WIFI_CREDENTIAL.MAX_CONN_RETRY);
                    
                   }else if( STA_RETRY_COUNT >= WIFI_CREDENTIAL.MAX_CONN_RETRY){ 
					     
					       STA_RETRY_COUNT = 0;
				           ESP_LOGE(TAG," WIFI STATION FAILED TO CONNECT ENTERING IN ACCESS POINT MODE \n");
				           esp_wifi_get_mode(&WIFI_MODE); 
				           
				           if(WIFI_MODE != WIFI_MODE_APSTA ){
							 esp_event_post_to(main_event_handle,MAIN_EVENT_BASE,AP_STA_WIFI_FALLBACK,NULL,0,portMAX_DELAY);
                           }
                           
                            
		    	  }
            
            
       
        }else if(WIFI_EVENT_STA_CONNECTED){
			
		ESP_LOGI(TAG, "Connected to AP");
		  
		STA_RETRY_COUNT = 0; 
		
		     
		}else if(WIFI_EVENT_STA_STOP){
			
	     ESP_LOGI(TAG, "Wi-Fi STA Stopped");
	     
		}else if(WIFI_EVENT_STA_WPS_ER_SUCCESS){
			
        ESP_LOGI(TAG, "WPS succeeded, connecting to saved AP");
     
     
        }else if( WIFI_EVENT_STA_WPS_ER_FAILED){
        ESP_LOGW(TAG, "WPS failed, retry or fallback");
      

        }else if(WIFI_EVENT_STA_WPS_ER_TIMEOUT){
        
        ESP_LOGW(TAG, "WPS timeout");
        

        }else if( WIFI_EVENT_STA_WPS_ER_PIN){
		
        /*wifi_event_sta_wps_er_pin_t* pin = (wifi_event_sta_wps_er_pin_t*) event_data;
        ESP_LOGI(TAG, "WPS_PIN = "PINSTR, PIN2STR(pin->pin_code));*/
        }
        
        
      
        
   }else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
		
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "WIFI STATION IP: " IPSTR, IP2STR(&event->ip_info.ip));
        snprintf(WIFI_CREDENTIAL.STA_IP_STR, sizeof(WIFI_CREDENTIAL.STA_IP_STR), IPSTR, IP2STR(&event->ip_info.ip));
        strncpy(NVS_STORAGE.DCN_IP_ADDRESS,WIFI_CREDENTIAL.STA_IP_STR ,sizeof(NVS_STORAGE.DCN_IP_ADDRESS));
        STA_RETRY_COUNT = 0; 
        
         err = esp_wifi_sta_get_rssi(&WIFI_CREDENTIAL.RSSI_DBI);
         if(err == ESP_OK){
		     WIFI_CREDENTIAL.RSSI_PER  = rssi_to_percent(WIFI_CREDENTIAL.RSSI_DBI);
              strncpy(WIFI_CREDENTIAL.RSSI_EVA_STR , evaluate_rssi(WIFI_CREDENTIAL.RSSI_DBI),sizeof(WIFI_CREDENTIAL.RSSI_EVA_STR));
              ESP_LOGI("STA_WIFI_EVENT", "RSSI: %d dBm | Signal: %s | Strength: %d%%", WIFI_CREDENTIAL.RSSI_DBI,WIFI_CREDENTIAL.RSSI_EVA_STR, WIFI_CREDENTIAL.RSSI_PER );
              }else{
	 		     ESP_LOGE("STA_WIFI_EVENT", "Failed to get RSSI: %s", esp_err_to_name(err));
                  }
               
       if(MQTT_INNITIALIZED_FLAG && client != NULL && MQTT_STOP_FLAG){    
             esp_mqtt_client_start(client);
             esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);  
             MQTT_STOP_FLAG = false;
             MQTT_CONNECTED_FLAG  = true;
           
        }else{  
        esp_event_post_to(main_event_handle,MAIN_EVENT_BASE,IP_GET,NULL,0,portMAX_DELAY);
             }
       }
    
}


static void wifi_event_handler_ap(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data){

 const char*  TAG = "WIFI EVENT AP";
 
if (event_base == WIFI_EVENT){
	
    switch (event_id) {
		
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "AP Started");
        
            break;

        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(TAG, "AP Stopped");
           
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
            ESP_LOGI(TAG, "Station connected: "MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
            ESP_LOGI(TAG, "Station disconnected: "MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
            break;
        }

          default:
            break;
    }
   
   } else if (event_base == IP_EVENT){
	   
	    switch (event_id) {
        case IP_EVENT_AP_STAIPASSIGNED: {
            ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*) event_data;
           
              snprintf(WIFI_CREDENTIAL.AP_LOCAL_IP_STR, sizeof(WIFI_CREDENTIAL.AP_LOCAL_IP_STR),"192.168.4.1");
              snprintf(WIFI_CREDENTIAL.AP_IP_STR, sizeof(WIFI_CREDENTIAL.AP_IP_STR), IPSTR, IP2STR(&event->ip));
             ESP_LOGI(TAG, "Station assigned IP: %s" ,WIFI_CREDENTIAL.AP_IP_STR);
            break;
        }

        default:
            break;
    }
	   
	   }
}


void WIFI_FALLBACK(char* MODE){
	 

	    esp_wifi_get_mode(&WIFI_MODE);
	    ESP_LOGI("WIFI_FALLBACK", "WIFI_FALLBACK current MODE ");
	
  
      if(WIFI_MODE == WIFI_MODE_STA){ 
      ESP_LOGI("WIFI_FALLBACK", "CURRENT MODE STA mode");
      
          esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &HTTP_connect_handler);
          esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &HTTP_disconnect_handler);
	   
          esp_wifi_disconnect();  // Only required for STA mode
          esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_sta);
          esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta);
       
       }else if(WIFI_MODE == WIFI_MODE_AP ){
		  ESP_LOGI("WIFI_FALLBACK", "CURRENT MODE AP mode");
		
          esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_START, &HTTP_connect_handler);
          esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_STOP, &HTTP_disconnect_handler);
             
           esp_event_handler_unregister(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler_ap);
           esp_event_handler_unregister(IP_EVENT,   ESP_EVENT_ANY_ID, &wifi_event_handler_ap);
    
       }else if(WIFI_MODE == WIFI_MODE_APSTA ){
		   ESP_LOGI("WIFI_FALLBACK", "CURRENT MODE AP+STA mode");
		
		  esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_START, &HTTP_connect_handler);
          esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_STOP, &HTTP_disconnect_handler);
        
           if(WIFI_MODE_APSTA_STA_ENABLE){
		   esp_wifi_disconnect();  // Only required for STA mode
		   esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_sta);
           esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta);
           }
         
           esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,&wifi_event_handler_ap);
           esp_event_handler_unregister(IP_EVENT,   ESP_EVENT_ANY_ID, &wifi_event_handler_ap);
	  }
		

		 //STOP_START_HTTP_SERVER(&server,true, false);             
    
          esp_wifi_stop();
          vTaskDelay(100 / portTICK_PERIOD_MS);
          esp_wifi_deinit();  
          
             
          
          esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
          if(sta_netif){
          ESP_LOGI("WIFI_FALLBACK", "Destroying existing STA netif");
          esp_netif_destroy_default_wifi(sta_netif);
        
          }

        
          esp_netif_t* AP_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
          if(AP_netif){
             ESP_LOGI("WIFI_FALLBACK", "Destroying existing AP netif");
             esp_netif_destroy_default_wifi(AP_netif);
           }
        
          
            
 
             if(strcmp(MODE,"WIFI_AP_STA")==0){
             
              ESP_LOGI("WIFI_FALLBACK", "STARTING AP+STA mode");
             
               esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &HTTP_connect_handler, &server);
               esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &HTTP_disconnect_handler, &server);
                wifi_init_ap_sta();
                
              }else if(strcmp(MODE,"WIFI_STA")==0){
				   
			   ESP_LOGI("WIFI_FALLBACK", "STARTING STA mode");
			   esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &HTTP_connect_handler, &server);
               esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &HTTP_disconnect_handler, &server);
			   wifi_init_sta();
				     
			  }else if(strcmp(MODE,"WIFI_AP")==0){
				  
				ESP_LOGI("WIFI_FALLBACK", "STARTING AP mode");
				
				esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &HTTP_connect_handler, &server);
                esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &HTTP_disconnect_handler, &server);
                wifi_init_ap();
			  
			  }else if(strcmp(MODE,"ESP_RESTART")==0){
              
                  esp_restart();
                  SETTING_TRANSMISSION_STATUS_FLAG = true;
                  return;
                  
               }else{
				   
				    ESP_LOGE("WIFI_FALLBACK", "NOTHING MATCH INVALID MODE: %s", MODE);
				    SETTING_TRANSMISSION_STATUS_FLAG = true;
				     return;
			      
	            }
	
	esp_wifi_get_mode(&WIFI_MODE);
	
    SETTING_TRANSMISSION_STATUS_FLAG = true;           
	  return;
                      
}

 cJSON * CREATE_WIFI_CREDENTIAL_JSON(){
	
    cJSON *root = cJSON_CreateObject();
    if (!root){
        ESP_LOGE("WIFI_JSON", "Failed to create JSON root");
        return NULL;
    }

    // Add all string members
     cJSON *wifi_config = cJSON_CreateObject();
    if (!wifi_config) {
        cJSON_Delete(root);
        ESP_LOGE("WIFI_JSON", "Failed to create WIFI_CONFIGURATION object");
        return NULL;
    }
    
    
    cJSON_AddNumberToObject(wifi_config, "RSSI_DBI", WIFI_CREDENTIAL.RSSI_DBI);
    cJSON_AddNumberToObject(wifi_config, "RSSI_PER", WIFI_CREDENTIAL.RSSI_PER);
    cJSON_AddStringToObject(wifi_config, "AP_SSID_AP_MODE", WIFI_CREDENTIAL.AP_SSID_AP_MODE);
    cJSON_AddStringToObject(wifi_config, "AP_PASSWORD_AP_MODE", WIFI_CREDENTIAL.AP_PASSWORD_AP_MODE);
    cJSON_AddStringToObject(wifi_config, "AP_SSID_STA_MODE", WIFI_CREDENTIAL.AP_SSID_STA_MODE);
    cJSON_AddStringToObject(wifi_config, "AP_PASSWORD_STA_MODE", WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE);
    cJSON_AddStringToObject(wifi_config, "MAC_ADDRESS_STR", WIFI_CREDENTIAL.MAC_ADDRESS_STR);
    cJSON_AddStringToObject(wifi_config, "ESP_SERIAL_NUMBER_STR", WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR);
    cJSON_AddStringToObject(wifi_config, "STA_IP_STR", WIFI_CREDENTIAL.STA_IP_STR);
    cJSON_AddStringToObject(wifi_config, "AP_IP_STR", WIFI_CREDENTIAL.AP_LOCAL_IP_STR);
    cJSON_AddStringToObject(wifi_config, "HOST_NAME_STR", WIFI_CREDENTIAL.HOST_NAME_STR);
   
   cJSON *wifi_setting = cJSON_CreateObject();
    if(!wifi_setting){
        cJSON_Delete(root);
        ESP_LOGE("WIFI_JSON", "Failed to create WIFI_SETTING object");
        return NULL;
    }
   
        char IP_TYPE_STR[20];
        if(WIFI_CREDENTIAL.STATIC_DYNAMIC_IP == true){
		 strncpy(IP_TYPE_STR, "STATIC", sizeof(IP_TYPE_STR));
	    }else if(WIFI_CREDENTIAL.STATIC_DYNAMIC_IP == false){
		strncpy(IP_TYPE_STR, "DYNAMIC", sizeof(IP_TYPE_STR));
	    }
    
    cJSON_AddStringToObject(  wifi_setting, "STATIC_DYNAMIC_IP", IP_TYPE_STR);
    cJSON_AddStringToObject(wifi_setting, "STATIC_IP", WIFI_CREDENTIAL.STATIC_IP);
    cJSON_AddStringToObject(wifi_setting, "GATEWAY_IP", WIFI_CREDENTIAL.GATEWAY_IP);
    cJSON_AddStringToObject(wifi_setting, "RSSI_EVA_STR", WIFI_CREDENTIAL.RSSI_EVA_STR);
    
        
        if(WIFI_CREDENTIAL.STA_AUTO_SWITCHING == true){
		 strncpy(IP_TYPE_STR, "ENABLE", sizeof(IP_TYPE_STR));
	    }else if(WIFI_CREDENTIAL.STA_AUTO_SWITCHING == false){
		strncpy(IP_TYPE_STR, "DISABLE", sizeof(IP_TYPE_STR));
	    }
    
   cJSON_AddStringToObject( wifi_setting, "STA_AUTO_SWITCHING", IP_TYPE_STR);
    
   cJSON_AddNumberToObject(wifi_setting, "STA_AUTO_SWITCHING_SCAN_RATE", WIFI_CREDENTIAL.STA_AUTO_SWITCHING_SCAN_RATE);
   cJSON_AddNumberToObject(wifi_setting, "CHANNEL_AP", WIFI_CREDENTIAL.CHANNEL_AP);
   cJSON_AddNumberToObject(wifi_setting, "MAX_STA_CONN_AP", WIFI_CREDENTIAL.MAX_STA_CONN_AP);
   cJSON_AddNumberToObject(wifi_setting, "MAX_AP_LIST", WIFI_CREDENTIAL.MAX_AP_LIST);
   cJSON_AddNumberToObject(wifi_setting, "MAX_CONN_RETRY", WIFI_CREDENTIAL.MAX_CONN_RETRY);

   cJSON_AddItemToObject(wifi_config, "WIFI_SETTING", wifi_setting);
     
     cJSON_AddItemToObject(root, "WIFI_CONFIGURATION", wifi_config);
    // Generate JSON string
   
    return  root ;  // Caller must free() this
}


esp_err_t WIFI_CONFIGURATION_PARSE_NVS(const char *content , uint16_t content_LEN , bool NVS_SAVE){
	
	  cJSON *parsed_json = cJSON_Parse(content);
      if(parsed_json == NULL){
            ESP_LOGE(WIFI_TAG, "Failed to parse JSON");
            strncpy(SYSTEM_ERROR_MSG,  "Invalid JSON",sizeof(SYSTEM_ERROR_MSG));
            return ESP_FAIL;
        }
        
  
        cJSON *wifi_config = cJSON_GetObjectItem(parsed_json, "WIFI_CONFIGURATION");
        if (!cJSON_IsObject(wifi_config)){
            ESP_LOGE(WIFI_TAG, "Missing WIFI_CONFIGURATION object");
            cJSON_Delete(parsed_json);
            strncpy(SYSTEM_ERROR_MSG, "Invalid JSON structure",sizeof(SYSTEM_ERROR_MSG));
             return ESP_FAIL;
        }
        
         cJSON *temp;
         char AP_SSID_STA_MODE[32];
         char AP_PASSWORD_STA_MODE[32];
         bool IP_TYPE = false;
         char GATEWAY_IP[20];
         char STATIC_IP[20];
         
         
        
        temp = cJSON_GetObjectItem(wifi_config, "SSID");
        if (temp && cJSON_IsString(temp)){
            strncpy(AP_SSID_STA_MODE, temp->valuestring, sizeof(AP_SSID_STA_MODE) - 1);
            AP_SSID_STA_MODE[sizeof(AP_SSID_STA_MODE) - 1] = '\0';
        
        }else{
			//strncpy(AP_SSID_STA_MODE,WIFI_CREDENTIAL.AP_SSID_STA_MODE, sizeof(WIFI_CREDENTIAL.AP_SSID_STA_MODE));
            ESP_LOGE(WIFI_TAG, "Invalid or missing AP_SSID_STA_MODE");
            cJSON_Delete(parsed_json);
            strncpy(SYSTEM_ERROR_MSG, "missing AP_SSID_STA_MODE",sizeof(SYSTEM_ERROR_MSG));
            return ESP_FAIL;
          }

        temp = cJSON_GetObjectItem(wifi_config, "PASSWORD");
        if (temp && cJSON_IsString(temp)){
            strncpy(AP_PASSWORD_STA_MODE, temp->valuestring, sizeof(AP_PASSWORD_STA_MODE) - 1);
            AP_PASSWORD_STA_MODE[sizeof(AP_PASSWORD_STA_MODE) - 1] = '\0';
        
        }else{
			//strncpy(AP_PASSWORD_STA_MODE,WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE,sizeof(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE));
            ESP_LOGE(WIFI_TAG, "Invalid or missing AP_PASSWORD_STA_MODE");
            cJSON_Delete(parsed_json);
            strncpy(SYSTEM_ERROR_MSG, "missing AP_PASSWORD_STA_MODE",sizeof(SYSTEM_ERROR_MSG));
            return ESP_FAIL;
            }
        
       
        
        
         cJSON *wifi_setting_obj = cJSON_GetObjectItem(wifi_config, "WIFI_SETTING");
         if(cJSON_IsObject(wifi_setting_obj)){
          
          temp = cJSON_GetObjectItem(wifi_setting_obj, "STATIC_DYNAMIC_IP");
          if(temp && cJSON_IsString(temp)){
			  
            if(strcmp(temp->valuestring, "STATIC") == 0){
		 	IP_TYPE = true;
			//WIFI_CREDENTIAL.STATIC_DYNAMIC_IP = true;
	     	}else if(strcmp(temp->valuestring,"DYNAMIC") == 0){
			//WIFI_CREDENTIAL.STATIC_DYNAMIC_IP = false;
			IP_TYPE = false;
	        }
          
        
        }else{
             ESP_LOGE(WIFI_TAG, "Invalid or missing IP_TYPE");
             strncpy(SYSTEM_ERROR_MSG,  "Invalid or missing SSID",sizeof(SYSTEM_ERROR_MSG));
            }
        
           
        
          temp = cJSON_GetObjectItem(wifi_setting_obj, "STATIC_IP");
         if (temp && cJSON_IsString(temp)) {
            strncpy(STATIC_IP, temp->valuestring, sizeof(STATIC_IP) - 1);
            STATIC_IP[sizeof(STATIC_IP) - 1] = '\0';
        } else {
            ESP_LOGE(WIFI_TAG, "Invalid or missing STATIC_IP");
            strncpy(SYSTEM_ERROR_MSG, "Invalid or missing SSID",sizeof(SYSTEM_ERROR_MSG));
           // cJSON_Delete(parsed_json);
          //  return ESP_FAIL;
        }

        temp = cJSON_GetObjectItem(wifi_setting_obj, "GATEWAY_IP");
        if (temp && cJSON_IsString(temp)) {
            strncpy(GATEWAY_IP, temp->valuestring, sizeof(GATEWAY_IP) - 1);
            GATEWAY_IP[sizeof(GATEWAY_IP) - 1] = '\0';
        } else {
            ESP_LOGE(WIFI_TAG, "Invalid or missing GATEWAY_IP");
            strncpy(SYSTEM_ERROR_MSG, "Invalid or missing PASSWORD",sizeof(SYSTEM_ERROR_MSG));
           // cJSON_Delete(parsed_json);
          //  return ESP_FAIL;
        }
        
        
        
        temp = cJSON_GetObjectItem(wifi_setting_obj, "MAX_CONN_RETRY");
        if (temp && cJSON_IsNumber(temp)){
            WIFI_CREDENTIAL.MAX_CONN_RETRY =  temp->valueint;
        }else{
			ESP_LOGE(WIFI_TAG, "Invalid or missing STA_RETRY_COUNT");
            strncpy(SYSTEM_ERROR_MSG, "Invalid or missing PASSWORD",sizeof(SYSTEM_ERROR_MSG));
           // cJSON_Delete(parsed_json);
          //  return ESP_FAIL;
            }
        
        
        
        
          temp = cJSON_GetObjectItem(wifi_setting_obj, "STA_AUTO_SWITCHING");
        if (temp && cJSON_IsString(temp)){
			char AUTO_SWITCHING[20];
            strncpy(AUTO_SWITCHING, temp->valuestring, sizeof(AUTO_SWITCHING) - 1);
            AUTO_SWITCHING[sizeof(AUTO_SWITCHING) - 1] = '\0';
            
            if(strcmp(AUTO_SWITCHING,"ENABLE") == 0){
			 
			WIFI_CREDENTIAL.STA_AUTO_SWITCHING= true;
			
	     	}else if(strcmp(AUTO_SWITCHING,"DISABLE") == 0){
			
		    WIFI_CREDENTIAL.STA_AUTO_SWITCHING= false;
	       
	        }
	        
        }else{
			
            ESP_LOGE(WIFI_TAG, "Invalid or missing STA_AUTO_SWITCHING");
            strncpy(SYSTEM_ERROR_MSG, "Invalid or missing STA_AUTO_SWITCHING",sizeof(SYSTEM_ERROR_MSG));
            //cJSON_Delete(parsed_json);
            //return ESP_FAIL;
            }
        
        
          temp = cJSON_GetObjectItem(wifi_setting_obj, "STA_AUTO_SWITCHING_SCAN_RATE");
         if (temp ){
			 if (cJSON_IsNumber(temp)){
             WIFI_CREDENTIAL.STA_AUTO_SWITCHING_SCAN_RATE  =  temp->valueint;
            
             }else if (cJSON_IsString(temp)){ 
             WIFI_CREDENTIAL.STA_AUTO_SWITCHING_SCAN_RATE  =  atoi(temp->valuestring);
        
                }
        
        }
        
        }else{
			  ESP_LOGW("WIFI_CONFIGURATION_HANDLER", "Missing WIFI_SETTING object");
			  strcpy(GATEWAY_IP,WIFI_CREDENTIAL.GATEWAY_IP); 
			  strcpy(STATIC_IP,WIFI_CREDENTIAL.STATIC_IP);
			  IP_TYPE  = WIFI_CREDENTIAL.STATIC_DYNAMIC_IP;
			  strncpy(SYSTEM_ERROR_MSG, "Missing WIFI_SETTING object",sizeof(SYSTEM_ERROR_MSG));
			 
             }
        
         ESP_LOGI(WIFI_TAG, "ACCESS POINT WiFi with new SSID: %s, Password: %s", AP_SSID_STA_MODE, AP_PASSWORD_STA_MODE);

          if(parsed_json){
           cJSON_Delete(parsed_json);  // Free the parsed JSON object
         }
          if(strlen(AP_SSID_STA_MODE) < 4 || strlen(AP_PASSWORD_STA_MODE ) < 4 ){
		  
            strncpy(SYSTEM_ERROR_MSG, "missing AP NAME OR PASSWORD_STA_MODE",sizeof(SYSTEM_ERROR_MSG));
            return ESP_FAIL;
		  }
   
   
     if(strcmp(WIFI_CREDENTIAL.AP_SSID_STA_MODE,AP_SSID_STA_MODE) != 0  ||  strcmp(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE,AP_PASSWORD_STA_MODE) !=0 ||WIFI_CREDENTIAL.STATIC_DYNAMIC_IP != IP_TYPE||  strcmp(WIFI_CREDENTIAL.STATIC_IP,STATIC_IP) !=0 ||strcmp(WIFI_CREDENTIAL.GATEWAY_IP,GATEWAY_IP) != 0){
       
           strncpy(WIFI_CREDENTIAL.AP_SSID_STA_MODE,AP_SSID_STA_MODE,  sizeof(WIFI_CREDENTIAL.AP_SSID_STA_MODE));
           strncpy(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE,AP_PASSWORD_STA_MODE,sizeof(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE));
       
           strcpy(WIFI_CREDENTIAL.GATEWAY_IP,GATEWAY_IP); 
		   strcpy(WIFI_CREDENTIAL.STATIC_IP,STATIC_IP);
		   WIFI_CREDENTIAL.STATIC_DYNAMIC_IP = IP_TYPE;
       
       
      if(NVS_SAVE){  
			  
         esp_err_t err = STORE_JSON_BODY_TO_NVS("WIFI_CONFIG_NVS", "LEN_NVS", "CONTENT_NVS",content_LEN, content, NULL);   
         if(err == ESP_OK){
			 strncpy(SYSTEM_ERROR_MSG,"WIFI CONFIGURATION SUCCESSFULL",sizeof(SYSTEM_ERROR_MSG)); 
			  ESP_LOGW("WIFI_CONFIGURATION_HANDLER", "WIFI CONFIGURATION NVS SAVE SUCCESSFULLY");
			 
			  WIFI_FALLBACK("WIFI_STA");     
			
			  ESP_LOGW("WIFI_CONFIGURATION_HANDLER", "WIFI CONFIGURATIOMN DATA MATHING WIFI_FALLBACK ");
			  return ESP_OK;
        
          }else{
			 
			ESP_LOGW("WIFI_CONFIGURATION_HANDLER", "WIFI CONFIGURATION NVS SAVE FAILED");
            strncpy(SYSTEM_ERROR_MSG,"WIFI CONFIGURATION FAILED TO SAVE",sizeof(SYSTEM_ERROR_MSG));
		     return ESP_FAIL;
           } 
           
         }
          
       /*nvs_handle_t WIFI_NVS;
        esp_err_t err = nvs_open("WIFI_CONFIG_NVS", NVS_READWRITE, &WIFI_NVS);
        if(err != ESP_OK){
            ESP_LOGE(WIFI_TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS open failed");
            return ESP_FAIL;
         }
      }else if(err == ESP_ERR_NVS_NOT_FOUND){
        
        strcpy(SYSTEM_ERROR_MSG,"WIFI CONFIGURATION SUCCESSFULL"); 
        
        nvs_set_str(WIFI_NVS,"AP_SSID", AP_SSID_STA_MODE);
        nvs_set_str(WIFI_NVS,"AP_PASSWORD", AP_PASSWORD_STA_MODE);
        nvs_commit( WIFI_NVS);
        nvs_close( WIFI_NVS);
        
        
        */
  
       
                  
        }else{
			strncpy(SYSTEM_ERROR_MSG, "WIFI CONFIGURATION ALREADY EXIT SAME",sizeof(SYSTEM_ERROR_MSG));
			ESP_LOGW("WIFI_CONFIGURATION_HANDLER", "WIFI CONFIGURATIOMN ALREADY EXITS SAME");
			return ESP_OK;
		  
		
		    }
		    
    
	return ESP_OK;
}



esp_err_t WIFI_CONFIGURATION_HANDLER(httpd_req_t *req){

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
    

    ESP_LOGI(WIFI_TAG, "WIFI_CONFIGURATION_HANDLER request received");

    if(req->method == HTTP_GET){
    
       ESP_LOGI("DEVICE_LOGGER_DATA"," GET WIFI_CONFIGURATION_HANDLER received");
    
      cJSON *root = CREATE_WIFI_CREDENTIAL_JSON();
      if(!root){
        ESP_LOGE("WIFI_CONFIGURATION_HANDLER", "Failed to create JSON object.");
         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "WIFI CONFIGURATION JSON ERROR");
         return ESP_FAIL;
     }

   
    char *DATA = cJSON_PrintUnformatted(root); 
    if (!DATA ){
        ESP_LOGE("REPORT_LOG_DATA", "Failed to serialize JSON data.");
        if(root){
        cJSON_Delete(root);
        }
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "WIFI CONGIF READ ERROR");
        return ESP_FAIL;
    }
    
      printf("WIFI CONFIG DATA :%s \n",DATA );
      httpd_resp_set_type(req, "application/json");
      httpd_resp_send(req, DATA, HTTPD_RESP_USE_STRLEN);
      
       if(root){
        cJSON_Delete(root);
        }
        
       if(DATA != NULL){
          free(DATA);
          DATA= NULL;  
         }
 
    }else if(req->method == HTTP_POST){
			
        char *content = malloc(req->content_len + 1);  // Allocate memory for content

        if (content == NULL){
            ESP_LOGE(WIFI_TAG, "Failed to allocate memory for content");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid CONTENT");
            return ESP_FAIL;
        }

        int err, remaining = req->content_len;

           while (remaining > 0){
            err = httpd_req_recv(req, content + (req->content_len - remaining), MIN(remaining, req->content_len));
            if (err < 0){
                if (err == HTTPD_SOCK_ERR_TIMEOUT){
                    continue;
                }
                ESP_LOGE(WIFI_TAG, "Failed to receive POST data");
                   if(content != NULL) {
        	           free(content);
        	           content = NULL; 
        	          }  // Free allocated memory in case of error
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive POST data");
                return ESP_FAIL;
            }
            remaining -= err;
        }

        content[req->content_len] = '\0';  // Null-terminate the string

     printf("WIFI WIFI_CONFIGURATION_PARSE  DATA :%s \n",content );
     
     httpd_resp_send(req, "WIFI CONFIGURATION SUCCESSFUL ", HTTPD_RESP_USE_STRLEN);
     
       err = WIFI_CONFIGURATION_PARSE_NVS(content,req->content_len ,true);
       if(err == ESP_OK){
			  httpd_resp_send(req, "WIFI CONFIGURATION SUCCESSFUL ", HTTPD_RESP_USE_STRLEN); 
		  
       }else{
			
             httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "WIFI CONFIGURATION JSON ERROR");
		   
           } 
        
        
        if(content != NULL){
            free(content);
            content= NULL;  // Prevent accidental reuse
        }
      
    }else{
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method Not Allowed");
        return ESP_FAIL;
    }
    
    

    return ESP_OK;
}



void nvs_read_wifi(){
	
 
    char *JSON_BODY_NVS_CONTENT = NULL;
    esp_err_t err = READ_JSON_BODY_FROM_NVS("WIFI_CONFIG_NVS", "LEN_NVS", "CONTENT_NVS", &JSON_BODY_NVS_CONTENT, NULL);
    if(err != ESP_OK){
       ESP_LOGE("nvs_read_wifi", "Failed to read WIFI_CONFIG_NVS  JSON body from NVS");
   
      }else if(err == ESP_ERR_NVS_NOT_FOUND){
        
        ESP_LOGE("nvs_read_wifi", "Failed to read WIFI_CONFIG_NVS NVS NOT FOUND");
          
    }else if(err == ESP_OK && JSON_BODY_NVS_CONTENT != NULL){
		  
      printf("WIFI_CONFIGURATION_CONTENT_NVS:%s\n",JSON_BODY_NVS_CONTENT);  
      WIFI_CONFIGURATION_PARSE_NVS(JSON_BODY_NVS_CONTENT,0,false);
	      
    }
    
      if(JSON_BODY_NVS_CONTENT != NULL){
       free(JSON_BODY_NVS_CONTENT);
       JSON_BODY_NVS_CONTENT = NULL;  // Prevent accidental reuse
     } 
   
        
 printf("strlen(WIFI_CREDENTIAL.AP_SSID_STA_MODE)==%d SSID:%s \n ",strlen(WIFI_CREDENTIAL.AP_SSID_STA_MODE),WIFI_CREDENTIAL.AP_SSID_STA_MODE);
 printf("strlen(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE)==%d PASSWORD :%s\n",strlen(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE),WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE);

    
}

wifi_ap_record_t *wifi_scan(uint16_t *found_ap_count){
	
    wifi_scan_config_t scan_config = {
		
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
       .scan_type = WIFI_SCAN_TYPE_PASSIVE,
       // .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        //.scan_time.passive = 200,
        .scan_time.active.min = 150,
        .scan_time.active.max = 200,
    
    };

    ESP_LOGI("WIFI_SCAN_TAG", "Starting Wi-Fi scan......");
    err = esp_wifi_scan_start(&scan_config, true);
    if(err != ESP_OK){ 
	  ESP_LOGE("WIFI_SCAN_TAG", "Scan start failed: %s", esp_err_to_name(err));
        return NULL;
     }

    uint16_t ap_count = 0;
    err = esp_wifi_scan_get_ap_num(&ap_count);
    if (err != ESP_OK){
        ESP_LOGE("WIFI_SCAN", "Failed to get AP count: %s", esp_err_to_name(err));
        return NULL;
    }ESP_LOGI("WIFI_SCAN", "Number of Access Points found: %d", ap_count);

    if (ap_count == 0){
		ESP_LOGE("WIFI_SCAN", "ERROR Number of Access Points found: %d", ap_count);
		return NULL;
      }
     
    if(ap_count > WIFI_CREDENTIAL.MAX_AP_LIST){
        ap_count = WIFI_CREDENTIAL.MAX_AP_LIST;
    }
    
    *found_ap_count = ap_count;  // Return the number of APs found
    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_records == NULL) {
        ESP_LOGE("WIFI_SCAN_TAG", "Memory allocation failed");
        return NULL;
    }


  if(esp_wifi_scan_get_ap_records(&ap_count, ap_records) != ESP_OK){
    ESP_LOGE("WIFI_SCAN_TAG", "Failed to get AP records");
      if (ap_records){
         free(ap_records);
         ap_records = NULL;
      }
    return NULL;
}
           //  for(int i = 0; i < ap_count; i++){		
          // printf("WIFI_NETWORK SSID: %s | RSSI: %d dBm | CHANNEL: %d \n", ap_records[i].ssid, ap_records[i].rssi, ap_records[i].primary);
         //  }

   
     return ap_records;
     
}

esp_err_t WIFI_AVAILABLE_NETWORK_HANDLER(httpd_req_t *req){
	
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    if(req->method == HTTP_GET){
		uint16_t ap_count = 0;
		
        wifi_ap_record_t *ap_records = wifi_scan(&ap_count);
       if(ap_records == NULL && ap_count > 0 ){
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NETWORK FAILED");
            return ESP_FAIL;
        }

        cJSON *json_root = cJSON_CreateObject();
        cJSON *json_array = cJSON_CreateArray();

        for (int i = 0; i < ap_count; i++){
			
            cJSON *ap_json = cJSON_CreateObject();
            cJSON_AddStringToObject(ap_json, "SSID", (char *)ap_records[i].ssid);
            char RSSI_EVA_STR[20];
             uint16_t  RSSI_PER  = rssi_to_percent(ap_records[i].rssi);
              strncpy(RSSI_EVA_STR , evaluate_rssi(ap_records[i].rssi),sizeof(RSSI_EVA_STR));
               
            
            cJSON_AddNumberToObject(ap_json, "RSSI", ap_records[i].rssi);
            cJSON_AddNumberToObject(ap_json, "RSSI_PER", RSSI_PER);
            cJSON_AddStringToObject(ap_json, "RSSI_STR", RSSI_EVA_STR);
            
            cJSON_AddNumberToObject(ap_json, "CHANNEL", ap_records[i].primary);
            cJSON_AddItemToArray(json_array, ap_json);
        }
        
        cJSON_AddNumberToObject(json_root, "NETWORK_COUNT", ap_count);
        cJSON_AddItemToObject(json_root, "AVAILABLE_WIFI_NETWORKS", json_array);

        char *json_response = cJSON_PrintUnformatted(json_root);
        if(json_response){
        ESP_LOGI("WIFI_SCAN_TAG", "Scanned APs JSON: %s", json_response);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_response, strlen(json_response));
        }else{
	    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NETWORK FAILED");
		}
          
        esp_wifi_clear_ap_list();
        
        if(ap_records){ 
        free(ap_records); 
        ap_records = NULL;
        }
      
        if(json_root){
        cJSON_Delete(json_root);
        }
        
        if(json_response){
        free(json_response);
        json_response = NULL;
        }
        
       
        
    }else{
		
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method Not Allowed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void STA_SSID_SCAN_TASK(void *pvParameters){
	
    uint16_t ap_count = 0;

    while (1) {
        wifi_ap_record_t *ap_list = wifi_scan(&ap_count);

        if (ap_list != NULL && ap_count > 0) {
            for (int i = 0; i < ap_count; i++) {
                if (strncmp((const char *)ap_list[i].ssid, WIFI_CREDENTIAL.AP_SSID_STA_MODE, sizeof(WIFI_CREDENTIAL.AP_SSID_STA_MODE)) == 0) {
                    ESP_LOGI("STA_SSID_SCAN_TASK", "Matched known SSID: %s | RSSI: %d | Channel: %d",ap_list[i].ssid, ap_list[i].rssi, ap_list[i].primary);
                    
                    esp_wifi_clear_ap_list();
                    if (ap_list){
                    free(ap_list);
                    ap_list = NULL;
                    }
                   
                   
                    if(WIFI_MODE == WIFI_MODE_APSTA){
                        ESP_LOGI("STA_SSID_SCAN_TASK", "Switching from AP+STA to STA mode...");
                        vTaskDelay(pdMS_TO_TICKS(50));  // Short wait before switching
                        WIFI_FALLBACK("WIFI_STA");
                    } else{
                        ESP_LOGI("STA_SSID_SCAN_TASK", "Already in STA mode. No fallback needed.");
                    }

                    vTaskDelete(NULL);  // End task after successful switch
                    return;
                }
            }
                  esp_wifi_clear_ap_list();
                   if (ap_list){
                    free(ap_list);
                    ap_list = NULL;
                    }
        } else {
            ESP_LOGW("STA_SSID_SCAN_TASK", "No access points found during scan.");
        }

        ESP_LOGW("STA_SSID_SCAN_TASK", "No known SSID (%s) found in this scan cycle.", WIFI_CREDENTIAL.AP_SSID_STA_MODE);

        // Wait before next scan
        vTaskDelay(pdMS_TO_TICKS(WIFI_CREDENTIAL.STA_AUTO_SWITCHING_SCAN_RATE));
    }

    // Task safety cleanup
    vTaskDelete(NULL);
}


void wifi_init_sta(void){

   //printf("WIFI STATION MODE\n ");
  //esp_netif_init();
 // esp_event_loop_create_default();	
	
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
     if (sta_netif == NULL) {
        ESP_LOGW("WIFI", "Failed to create default Wi-Fi AP netif");
       } 
  
    if(WIFI_CREDENTIAL.STATIC_DYNAMIC_IP && sta_netif != NULL){
    esp_netif_ip_info_t ip_info;
       memset(&ip_info, 0, sizeof(ip_info));
    ip_info.ip.addr = esp_ip4addr_aton(WIFI_CREDENTIAL.STATIC_IP);     // Static IP
    ip_info.gw.addr = esp_ip4addr_aton(WIFI_CREDENTIAL.GATEWAY_IP);      // Router gateway
    ip_info.netmask.addr = esp_ip4addr_aton("255.255.255.0");
   
     if (esp_netif_dhcpc_stop(sta_netif) == ESP_OK){
            if (esp_netif_set_ip_info(sta_netif, &ip_info) != ESP_OK){
                ESP_LOGE(WIFI_TAG, "esp_netif_set_ip_info failed");
            } else {
                ESP_LOGI(WIFI_TAG, "STATIC_IP CONGFIGUARED :%s GATEWAY_IP :%s",WIFI_CREDENTIAL.STATIC_IP, WIFI_CREDENTIAL.GATEWAY_IP);
            }
        } else {
            ESP_LOGW(WIFI_TAG, "esp_netif_dhcpc_stop failed or was not running");
        }
   
    

    }
 
 
   esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    &wifi_event_handler_sta, NULL);
   esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta, NULL);
  
  //esp_netif_set_hostname(sta_netif, WIFI_CREDENTIAL.HOST_NAME_STR);
    wifi_config_t wifi_config = {
        .sta = {
			//.failure_retry_cnt
	
            .ssid =     ESP_WIFI_SSID_STA_DEFAULT,
            .password = ESP_WIFI_PASS_STA_DEFAULT,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
           // .scan_method = WIFI_FAST_SCAN,
           
            .pmf_cfg = {
                .capable = false,
              
                .required = false
               
            },
        },
    };

  if (  WIFI_CREDENTIAL.AP_SSID_STA_MODE[0] != '\0'
   && strlen(WIFI_CREDENTIAL.AP_SSID_STA_MODE) >= 4 
   && strcmp(WIFI_CREDENTIAL.AP_SSID_STA_MODE,"NOT_CONFIGUARED") != 0
   && WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE[0] != '\0'
   && strlen(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE) >= 4 
   && strcmp(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE,"NOT_CONFIGUARED") != 0
   ){
	
   strncpy((char *)wifi_config.sta.ssid,WIFI_CREDENTIAL.AP_SSID_STA_MODE, sizeof(WIFI_CREDENTIAL.AP_SSID_STA_MODE) - 1);
   strncpy((char *)wifi_config.sta.password,WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE, sizeof(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE) - 1);

 }else{ESP_LOGE("WIFI_STA_MODE", "NVS SSID AND PASSWORD ERROR %s / %s ",WIFI_CREDENTIAL.AP_SSID_STA_MODE,WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE);}
 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
   
    esp_wifi_set_mode(WIFI_MODE_STA);
     esp_wifi_set_max_tx_power(78);  // Set maximum TX power (78 = 19.5 dBm)
    esp_wifi_set_ps(WIFI_PS_NONE);  // Disable Wi-Fi power saving mode for max performance
    
    //esp_wifi_set_storage(WIFI_STORAGE_FLASH);  //WIFI_STORAGE_RAM	StORE Wi-Fi configuration  (Lost after restart or power off)  WIFI_STORAGE_FLASH	Saves Wi-Fi configuration in NVS Flash (Persistent after restart).
   // esp_wifi_set_storage( WIFI_STORAGE_RAM); 
   
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(WIFI_TAG, "wifi_init_STATION finished. SSID: %s password: %s",wifi_config.sta.ssid,wifi_config.sta.password);
    
 }

void wifi_init_ap(){
	
	
	  //  esp_netif_init();
	//  esp_event_loop_create_default();
	  
	  
	  esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
     if (ap_netif == NULL) {
        ESP_LOGW("WIFI", "Failed to create default Wi-Fi AP netif");
       // return ;
       } 	  
	
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

   // esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler_ap,NULL,NULL);

   esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler_ap,NULL);
   esp_event_handler_register(IP_EVENT,   ESP_EVENT_ANY_ID, &wifi_event_handler_ap, NULL);
 

    wifi_config_t wifi_config_ap = {
        .ap = {
            .ssid = ESP_WIFI_SSID_AP_DEFAULT,
            .ssid_len = strlen(ESP_WIFI_SSID_AP_DEFAULT),
            .channel = WIFI_CREDENTIAL.CHANNEL_AP ,
            .password = ESP_WIFI_PASS_AP_DEFAULT,
            .max_connection = WIFI_CREDENTIAL.MAX_STA_CONN_AP,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK,
			.pmf_cfg = {                            // Protected Management Frames 
            .capable = false,  // Enable PMF support
            .required = false // PMF is not mandatory
            }

        },
        
           };
           
           
if (strlen(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR) >= 12 && strcmp(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR,"NOT_CONFIGURARED") !=0) {
	
	char ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR[50];
    snprintf(ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR, sizeof(ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR),"DCN-%s", WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR);
    strncpy((char *)wifi_config_ap.ap.ssid, ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR, sizeof(wifi_config_ap.ap.ssid));
    strncpy((char *)wifi_config_ap.ap.password, ESP_WIFI_PASS_AP_DEFAULT, sizeof(wifi_config_ap.ap.password));
   
}else{
	
    strncpy((char *)wifi_config_ap.ap.ssid, ESP_WIFI_SSID_AP_DEFAULT, sizeof(wifi_config_ap.ap.ssid));
    strncpy((char *)wifi_config_ap.ap.password, ESP_WIFI_PASS_AP_DEFAULT, sizeof(wifi_config_ap.ap.password));
}

wifi_config_ap.ap.ssid[sizeof(wifi_config_ap.ap.ssid) - 1] = '\0';
wifi_config_ap.ap.password[sizeof(wifi_config_ap.ap.password) - 1] = '\0';
wifi_config_ap.ap.ssid_len = strlen((char *)wifi_config_ap.ap.ssid);  
    
   if(ESP_WIFI_PASS_AP_ENABLE == false || wifi_config_ap.ap.ssid_len == 0){
    wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
   }else{
    wifi_config_ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
   }
   
   
           esp_wifi_set_ps(WIFI_PS_NONE); 
           esp_wifi_set_mode(WIFI_MODE_AP);
           esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap);
           esp_wifi_start();
    
           ESP_LOGI(WIFI_TAG, "SOFT AP INIT finished. SSID:%s password:%s",ESP_WIFI_SSID_AP_DEFAULT,ESP_WIFI_PASS_AP_DEFAULT);
           
}


esp_netif_t * WIFI_APSTA_MODE_STA_CONFIG(void){


      esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
     if (sta_netif == NULL) {
        ESP_LOGW("WIFI_APSTA_MODE_STA_CONFIG", "Failed to create default Wi-Fi AP netif");
       } 
  
    if(WIFI_CREDENTIAL.STATIC_DYNAMIC_IP && sta_netif != NULL){
    esp_netif_ip_info_t ip_info;
       memset(&ip_info, 0, sizeof(ip_info));
    ip_info.ip.addr = esp_ip4addr_aton(WIFI_CREDENTIAL.STATIC_IP);     // Static IP
    ip_info.gw.addr = esp_ip4addr_aton(WIFI_CREDENTIAL.GATEWAY_IP);      // Router gateway
    ip_info.netmask.addr = esp_ip4addr_aton("255.255.255.0");
   
     if (esp_netif_dhcpc_stop(sta_netif) == ESP_OK){
            if (esp_netif_set_ip_info(sta_netif, &ip_info) != ESP_OK){
                ESP_LOGE("WIFI_APSTA_MODE_STA_CONFIG", "esp_netif_set_ip_info failed");
            } else {
                ESP_LOGI("WIFI_APSTA_MODE_STA_CONFIG", "STATIC_IP CONGFIGUARED :%s GATEWAY_IP :%s",WIFI_CREDENTIAL.STATIC_IP, WIFI_CREDENTIAL.GATEWAY_IP);
            }
        } else {
            ESP_LOGW("WIFI_APSTA_MODE_STA_CONFIG", "esp_netif_dhcpc_stop failed or was not running");
        }
   
    //esp_netif_dhcps_start(ap_netif);     // Restart DHCP server NOT NEEDED FOR STATIC 

    }
     
    
 
   esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,    &wifi_event_handler_sta, NULL);
   esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, &wifi_event_handler_sta, NULL);
  
  //esp_netif_set_hostname(sta_netif, WIFI_CREDENTIAL.HOST_NAME_STR);
    wifi_config_t wifi_config = {
        .sta = {
			//.failure_retry_cnt
	
            .ssid =    ESP_WIFI_SSID_STA_DEFAULT,
            .password = ESP_WIFI_PASS_STA_DEFAULT,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
           // .scan_method = WIFI_FAST_SCAN,
           
            .pmf_cfg = {
                .capable = false,
              
                .required = false
               
            },
        },
    };

if (  WIFI_CREDENTIAL.AP_SSID_STA_MODE[0] != '\0'
   && strlen(WIFI_CREDENTIAL.AP_SSID_STA_MODE) >= 4 
   && strcmp(WIFI_CREDENTIAL.AP_SSID_STA_MODE,"NOT_CONFIGUARED") != 0
   && WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE[0] != '\0'
   && strlen(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE) >= 4 
   && strcmp(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE,"NOT_CONFIGUARED") != 0
   ){
	
   strncpy((char *)wifi_config.sta.ssid,WIFI_CREDENTIAL.AP_SSID_STA_MODE, sizeof(WIFI_CREDENTIAL.AP_SSID_STA_MODE) - 1);
   strncpy((char *)wifi_config.sta.password,WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE, sizeof(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE) - 1);

 }else{
	 
   ESP_LOGE("WIFI_APSTA_MODE_STA_CONFIG", "NVS SSID AND PASSWORD ERROR FALLBACK DEFAULT %s / %s ",WIFI_CREDENTIAL.AP_SSID_STA_MODE,WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE);
    strlcpy((char *)wifi_config.sta.ssid, ESP_WIFI_SSID_STA_DEFAULT, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, ESP_WIFI_PASS_STA_DEFAULT, sizeof(wifi_config.sta.password));
    
  
}
	
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  
  esp_wifi_set_max_tx_power(78);  // Set maximum TX power (78 = 19.5 dBm)
  ESP_LOGI("WIFI_APSTA_MODE_STA_CONFIG", "STA WIFI STARTING AP SEARCH... SSID: %s password: %s",wifi_config.sta.ssid,wifi_config.sta.password);
   return sta_netif;
    
 }
 
 esp_netif_t * WIFI_APSTA_MODE_AP_CONFIG(void){
	 
	  esp_netif_t *esp_ap_netif = esp_netif_create_default_wifi_ap();
    if (esp_ap_netif == NULL){
    ESP_LOGW("WIFI", "Failed to create default Wi-Fi AP netif");
    
   } 
  
    esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler_ap,NULL);
    esp_event_handler_register(IP_EVENT,   ESP_EVENT_ANY_ID, &wifi_event_handler_ap, NULL);
  
 
 
    wifi_config_t wifi_config_ap = {
           .ap = {
			    
               .ssid = ESP_WIFI_SSID_AP_DEFAULT,
               .ssid_len = 0,
               .password = ESP_WIFI_PASS_AP_DEFAULT,
               .channel = WIFI_CREDENTIAL.CHANNEL_AP , //6//1//11    
               .ssid_hidden= 0,
               .max_connection = WIFI_CREDENTIAL.MAX_STA_CONN_AP,
   			   .authmode = WIFI_AUTH_WPA2_PSK,
               .beacon_interval= 50, //100 
               .pairwise_cipher = WIFI_CIPHER_TYPE_CCMP ,// Use AES-CCMP encryption
               .csa_count = 5 ,//Channel Switch Announcement (CSA)  //4-10 (Medium)	Balanced approach: Allows clients time to prepare
           
             // .sae_pwe_h2e
            //.ftm_responder
            //.dtim_period
               .pmf_cfg = {      // Protected Management Frames   If true, ESP32 supports PMF, but does not force clients to use it.If true, PMF is required, and clients must support PMF to connect.
               .capable = false,  // Enable PMF support
               .required = false // PMF is optional for clients
            
            }
            
           },

   
    };
    

    
if (strlen(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR) >= 12 && strcmp(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR,"NOT_CONFIGUARED") != 0){
	
	char ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR[50];
    snprintf(ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR, sizeof(ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR),"DCN-%s", WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR);
    strncpy((char *)wifi_config_ap.ap.ssid, ESP_WIFI_SSID_AP_WIFI_CREDENTIALS_DEFAULT_STR, sizeof(wifi_config_ap.ap.ssid));
    strncpy((char *)wifi_config_ap.ap.password, ESP_WIFI_PASS_AP_DEFAULT, sizeof(wifi_config_ap.ap.password));
   
}else{
	
    strncpy((char *)wifi_config_ap.ap.ssid, ESP_WIFI_SSID_AP_DEFAULT, sizeof(wifi_config_ap.ap.ssid));
    strncpy((char *)wifi_config_ap.ap.password,ESP_WIFI_PASS_AP_DEFAULT, sizeof(wifi_config_ap.ap.password));
     
     }

wifi_config_ap.ap.ssid[sizeof(wifi_config_ap.ap.ssid) - 1] = '\0';
wifi_config_ap.ap.password[sizeof(wifi_config_ap.ap.password) - 1] = '\0';
wifi_config_ap.ap.ssid_len = strlen((char *)wifi_config_ap.ap.ssid);  

  ESP_LOGW(WIFI_TAG, "wifi_config_ap.ap.ssid_len : %d",wifi_config_ap.ap.ssid_len);   

  ESP_LOGI(WIFI_TAG, "WIFI INIT AP_STA MODE finished. SSID:%s password:%s",wifi_config_ap.ap.ssid,wifi_config_ap.ap.password);
  
   
    if (ESP_WIFI_PASS_AP_ENABLE == false || wifi_config_ap.ap.ssid_len == 0){
    wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
   }else{
    wifi_config_ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
   }
   
     esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap);
	
	  return esp_ap_netif;
	  
}

void wifi_init_ap_sta(void){
	
   
   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   esp_wifi_init(&cfg);
  
    wifi_country_t country_config = {
    .cc = "IN",  // Country code for India //US 1-11
    .schan = 1,  // Start from channel 1
    .nchan = 13, // India allows channels 1-13
    .policy = WIFI_COUNTRY_POLICY_MANUAL // Use a fixed country setting

};esp_wifi_set_country(&country_config); // Set country configuration
    
     esp_wifi_set_ps(WIFI_PS_NONE);  
     esp_wifi_set_mode(WIFI_MODE_APSTA);
   
   
     ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_AP_CONFIGURING......");
   // esp_netif_t *esp_netif_ap = WIFI_APSTA_MODE_AP_CONFIG();
      WIFI_APSTA_MODE_AP_CONFIG();
    
    if(WIFI_MODE_APSTA_STA_ENABLE){
    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA_CONFIGURING......");
    esp_netif_t *esp_netif_sta = WIFI_APSTA_MODE_STA_CONFIG();
    esp_netif_set_default_netif(esp_netif_sta);  
     }
   
    esp_wifi_start();
    esp_wifi_get_mode(&WIFI_MODE); 		


  /* if(esp_netif_napt_enable(esp_netif_ap) != ESP_OK){
       ESP_LOGE(WIFI_TAG, "NAPT not enabled on AP interface");
   }*/
 
   //if(WIFI_MODE_APSTA_STA_ENABLE == false && WIFI_CREDENTIAL.STA_AUTO_SWITCHING  && strlen(WIFI_CREDENTIAL.AP_SSID_STA_MODE) > 4 && strcmp(WIFI_CREDENTIAL.AP_SSID_STA_MODE,"NOT_CONFIGURARED") !=0){
	  //   esp_wifi_get_mode(&WIFI_MODE); 
     //    xTaskCreate(STA_SSID_SCAN_TASK, "STA_SSID_SCAN_TASK", 4096, NULL, 3, NULL);
 
   // }   
   
   
}


typedef struct{
	
	
    char utc_offset[10];     // UTC offset string
     char timezone[50];       // Timezone string
    int day_of_week;         // Day of the week (integer)
    int day_of_year;
    int year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t secounds;
    uint8_t milliSeconds;      // Day of the year (integer)
    char datetime[50];       // Datetime string
    char utc_datetime[50];   // UTC Datetime string
    long unixtime;           // Unix time (long)
    int week_number;         // Week number (integer)
    char abbreviation[10]; 
    
    bool dst;                // Daylight saving time flag (boolean)
    
} time_info_t;

  time_info_t http_time_info;
  
void TIME_INFO_json_response(const char *json_data, time_info_t *time_info) {
	
    // Parse the JSON data
    
     //printf("json_data  %s \n",json_data);
    
    cJSON *root = cJSON_Parse(json_data);
    
    if (root == NULL) {
        ESP_LOGE(RTC_TIME_TAG, "Error parsing JSON data");
        return;
    }

    // Extract "utc_offset"
    cJSON *utc_offset = cJSON_GetObjectItem(root, "utc_offset");
    if (utc_offset != NULL) {
        strncpy(time_info->utc_offset, utc_offset->valuestring, sizeof(time_info->utc_offset) - 1);
    }

    // Extract "timezone"
    cJSON *timezone = cJSON_GetObjectItem(root, "timezone");
    if (timezone != NULL) {
        strncpy(time_info->timezone, timezone->valuestring, sizeof(time_info->timezone) - 1);
    }

    // Extract "day_of_week"
    cJSON *day_of_week = cJSON_GetObjectItem(root, "dayofweek");
    if (day_of_week != NULL) {
        time_info->day_of_week = day_of_week->valueint;
    }

    // Extract "day_of_year"
    cJSON *day_of_year = cJSON_GetObjectItem(root, "day_of_year");
    if (day_of_year != NULL) {
        time_info->day_of_year = day_of_year->valueint;
    }
    
      cJSON *hour  = cJSON_GetObjectItem(root, "hour");
    if (day_of_week != NULL) {
        time_info->hour = hour ->valueint;
    }
       
      cJSON *minute  = cJSON_GetObjectItem(root, "minute");
    if (day_of_week != NULL) {
        time_info->minute = minute->valueint;
    }
    
    cJSON *secounds  = cJSON_GetObjectItem(root, "secounds");
    if (day_of_week != NULL) {
        time_info->secounds = secounds->valueint;
    }


    // Extract "datetime"
    cJSON *datetime = cJSON_GetObjectItem(root, "datetime");
    if (datetime != NULL) {
        strncpy(time_info->datetime, datetime->valuestring, sizeof(time_info->datetime) - 1);
    }

 

    cJSON_Delete(root);
    
      ESP_LOGI(RTC_TIME_TAG, "UTC Offset: %s", time_info->utc_offset);
      ESP_LOGI(RTC_TIME_TAG, "Timezone: %s", time_info->timezone);
      ESP_LOGI(RTC_TIME_TAG, "Day of Week: %d", time_info->day_of_week);
      ESP_LOGI(RTC_TIME_TAG, "Day of Year: %d", time_info->day_of_year);
      ESP_LOGI(RTC_TIME_TAG, "Datetime: %s", time_info->datetime);
      ESP_LOGI(RTC_TIME_TAG, "UTC Datetime: %s", time_info->utc_datetime);
      ESP_LOGI(RTC_TIME_TAG, "Unix Time: %ld", time_info->unixtime);
      ESP_LOGI(RTC_TIME_TAG, "Week Number: %d", time_info->week_number);
      ESP_LOGI(RTC_TIME_TAG, "Timezone Abbreviation: %s", time_info->abbreviation);
      ESP_LOGI(RTC_TIME_TAG, "DST: %s", time_info->dst ? "true" : "false");
}




void get_time_via_http(void){
	
	int  REQUEST_STATUS= 0;
	esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    
     if (netif) {
        esp_netif_get_ip_info(netif, &ip_info);
        ESP_LOGI("NETWORK", "IP Address: " IPSTR, IP2STR(&ip_info.ip));
        HTTP_CLIENT_RESPOND_DATA_FLAG =  true;
       REQUEST_STATUS = PERFORM_HTTP_REQUEST(3,"http://timeapi.io/api/Time/current/zone?timeZone=Asia/Kolkata",HTTP_METHOD_GET,NULL,NULL, NULL,0);
    
 
    }else {
		
        ESP_LOGE("NETWORK", "Failed to get network  IP interface");
    }
    
   if(REQUEST_STATUS == 200){
	   
     format_datetime(http_time_info.datetime);  
     
   }else{
	   
   //format_datetime(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE_TIME);
   
    }
 
 }


esp_err_t STORE_JSON_BODY_TO_NVS(const char *nvs_namespace, const char *key_length, const char *key_content, int32_t content_length, const char *content, httpd_req_t *req) {
 
   
 uint8_t retry_count = 0;
 
    if (strlen(nvs_namespace) > 15 || strlen(key_length) > 15 || strlen(key_content) > 15) {
        strcpy(error_msg, "NVS key or namespace too long");
        ESP_LOGE("STORE_JSON_NVS", "%s", error_msg);
        return ESP_FAIL;
    }

    if (content == NULL) {
        strcpy(error_msg, "Missing content");
        ESP_LOGE("STORE_JSON_NVS", "%s", error_msg);
        return ESP_FAIL;
    }

retry_nvs_open:
    err = nvs_open(nvs_namespace, NVS_READWRITE, &JSON_BODY_NVS_HANDLE);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(300));
            goto retry_nvs_open;
        }
        strcpy(error_msg,esp_err_to_name(err));
        ESP_LOGE("STORE_JSON_NVS", "%s: %s", error_msg, esp_err_to_name(err));
        return ESP_FAIL;
    }

    retry_count = 0;

retry_set_length:
    err = nvs_set_i32(JSON_BODY_NVS_HANDLE, key_length, content_length);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
             vTaskDelay(pdMS_TO_TICKS(300));
            goto retry_set_length;
        }
        strcpy(error_msg, "Failed to store length");
        ESP_LOGE("STORE_JSON_NVS", "%s: %s", error_msg, esp_err_to_name(err));
        nvs_close(JSON_BODY_NVS_HANDLE);
        return ESP_FAIL;
    }

    retry_count = 0;

retry_set_content:

    err = nvs_set_str(JSON_BODY_NVS_HANDLE, key_content, content);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(300));
            goto retry_set_content;
        }
        strcpy(error_msg, "Failed to store content");
        ESP_LOGE("STORE_JSON_NVS", "%s: %s", error_msg, esp_err_to_name(err));
        nvs_close(JSON_BODY_NVS_HANDLE);
        return ESP_FAIL;
    }

    retry_count = 0;

retry_commit:

    err = nvs_commit(JSON_BODY_NVS_HANDLE);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
             vTaskDelay(pdMS_TO_TICKS(300));
            goto retry_commit;
        }
        strcpy(error_msg, "Failed to commit");
        ESP_LOGE("STORE_JSON_NVS", "%s: %s", error_msg, esp_err_to_name(err));
        nvs_close(JSON_BODY_NVS_HANDLE);
        return ESP_FAIL;
    }

    nvs_close(JSON_BODY_NVS_HANDLE);
    ESP_LOGI("STORE_JSON_NVS", "Content stored successfully in NVS");
    return ESP_OK;
}

esp_err_t STORE_BLOB_TO_NVS(const char *nvs_namespace, const char *key, const void *data, size_t length) {
    
    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle;
    uint8_t retry_count = 0;

    if (!nvs_namespace || !key || !data || length == 0) {
        strcpy(error_msg, "Invalid input arguments");
        ESP_LOGE("STORE_BLOB_NVS", "%s", error_msg);
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(nvs_namespace) > 15 || strlen(key) > 15) {
        strcpy(error_msg, "NVS key or namespace too long");
        ESP_LOGE("STORE_BLOB_NVS", "%s", error_msg);
        return ESP_FAIL;
    }

retry_nvs_open:
    err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(300));
            goto retry_nvs_open;
        }
        ESP_LOGE("STORE_BLOB_NVS", "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    retry_count = 0;

retry_set_blob:
    err = nvs_set_blob(handle, key, data, length);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(300));
            goto retry_set_blob;
        }
        ESP_LOGE("STORE_BLOB_NVS", "Failed to set blob: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    retry_count = 0;

retry_commit:
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(300));
            goto retry_commit;
        }
        ESP_LOGE("STORE_BLOB_NVS", "Failed to commit: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);
    ESP_LOGI("STORE_BLOB_NVS", "Blob stored successfully (key=%s, len=%d)", key, length);
    return ESP_OK;
}

esp_err_t READ_JSON_BODY_FROM_NVS(const char *nvs_namespace, const char *key_length, const char *key_content, char* *JSON_BODY_NVS_CONTENT, httpd_req_t *req){
   
    uint8_t retry_count = 0;
   
  
    if (strlen(nvs_namespace) > 15 || strlen(key_length) > 15 || strlen(key_content) > 15){
        strcpy(error_msg, "NVS key or namespace too long");
       ESP_LOGE("READ_JSON_NVS", "%s-%s-%s error %s", error_msg,nvs_namespace,key_content,esp_err_to_name(err));
        return ESP_FAIL;
      }
       
    

   
    retry_nvs_open:
    err = nvs_open(nvs_namespace, NVS_READONLY, &JSON_BODY_NVS_HANDLE);
    if (err != ESP_OK) {
        if (++retry_count <= NVS_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(200));
            goto retry_nvs_open;
        }
        strcpy(error_msg, "Failed to open NVS");
        ESP_LOGE("READ_JSON_NVS", "%s-%s-%s error %s", error_msg,nvs_namespace,key_content,esp_err_to_name(err));
        return ESP_FAIL;
    }

   int32_t content_length = 0;
   retry_count = 0;

 retry_nvs_get_length:

    err = nvs_get_i32(JSON_BODY_NVS_HANDLE, key_length, &content_length);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strcpy(error_msg, "Length key not found ");
        ESP_LOGE("READ_JSON_NVS", "%s-%s-%s error %s", error_msg,nvs_namespace,key_content,esp_err_to_name(err));
        nvs_close(JSON_BODY_NVS_HANDLE);
        return err;
        
    }else if(err != ESP_OK){
        if (++retry_count <= NVS_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(200));
            goto retry_nvs_get_length;
        }
        
        strcpy(error_msg, "Failed to read length");
        ESP_LOGE("READ_JSON_NVS", "%s-%s-%s error %s", error_msg,nvs_namespace,key_content,esp_err_to_name(err));
        nvs_close(JSON_BODY_NVS_HANDLE);
        return ESP_FAIL;
    }


     size_t size = content_length + 1;
    *JSON_BODY_NVS_CONTENT = malloc(size);
    if (*JSON_BODY_NVS_CONTENT == NULL){
		
        strcpy(error_msg, "Memory allocation failed");
        ESP_LOGE("READ_JSON_NVS", "%s-%s-%s error %s", error_msg,nvs_namespace,key_content,esp_err_to_name(err));
        nvs_close(JSON_BODY_NVS_HANDLE);
        return ESP_FAIL;
    }


 retry_count = 0;

  retry_nvs_get_content:
  
    err = nvs_get_str(JSON_BODY_NVS_HANDLE, key_content, *JSON_BODY_NVS_CONTENT, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strcpy(error_msg, "Content key not found");
        ESP_LOGE("READ_JSON_NVS", "%s-%s-%s error %s", error_msg,nvs_namespace,key_content,esp_err_to_name(err));
        if (*JSON_BODY_NVS_CONTENT) {
            free(*JSON_BODY_NVS_CONTENT);
            *JSON_BODY_NVS_CONTENT = NULL;
        }
        nvs_close(JSON_BODY_NVS_HANDLE);
        return err;
    } else if (err != ESP_OK){
        if (++retry_count <= NVS_MAX_RETRIES){
            vTaskDelay(pdMS_TO_TICKS(200));
            goto retry_nvs_get_content;
        }
        
        strcpy(error_msg, "Failed to read content");
       ESP_LOGE("READ_JSON_NVS", "%s-%s-%s error %s", error_msg,nvs_namespace,key_content,esp_err_to_name(err));
        if (*JSON_BODY_NVS_CONTENT){
            free(*JSON_BODY_NVS_CONTENT);
            *JSON_BODY_NVS_CONTENT = NULL;
        }
        
        nvs_close(JSON_BODY_NVS_HANDLE);
        return ESP_FAIL;
    }

    nvs_close(JSON_BODY_NVS_HANDLE);
    ESP_LOGI("READ_JSON_NVS", "Successfully read content from NVS");
    return ESP_OK;
}



 
 esp_err_t SYSTEM_DATE_TIME_HANDLER(httpd_req_t *req){
	 
	 
    ESP_LOGI("DEVICE_LOGGER_DATA", "SYSTEM_DATE_TIME_HANDLER request received");
    REQUEST_HEADER(req);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    if (req->method != HTTP_POST) {
        ESP_LOGE("DEVICE_LOGGER_DATA", "Invalid HTTP method");
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method Not Allowed");
        return ESP_FAIL;
    }

    char *content = malloc(req->content_len + 1);
    if (!content) {
        ESP_LOGE("DEVICE_LOGGER_DATA", "Failed to allocate memory for content");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory Allocation Failed");
        return ESP_ERR_NO_MEM;
    }

    int err, remaining = req->content_len;
    char *ptr = content;

    while (remaining > 0) {
        err = httpd_req_recv(req, ptr, remaining);
        if (err < 0) {
            if (err == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE("DEVICE_LOGGER_DATA", "Failed to receive POST data");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to Receive Data");
            if(content!=NULL){
            free(content);
            content = NULL;
            }
            return ESP_FAIL;
        }
        remaining -= err;
        ptr += err;
    }

    content[req->content_len] = '\0';
    ESP_LOGI("SYSTEM_DATE_TIME_HANDLER", "Received Data: %s", content);

    // Parse JSON
    
    cJSON *parsed_json = cJSON_Parse(content);
    
          if(content!=NULL){
            free(content);
            content = NULL;
            }
            
    if (!parsed_json) {
        ESP_LOGE("DEVICE_LOGGER_DATA", "Error parsing JSON");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON Format");
        return ESP_FAIL;
    }

    // Extract SYSTEM_DATE_TIME object
    cJSON *device_config = cJSON_GetObjectItem(parsed_json, "SYSTEM_DATE_TIME");
    if (!device_config) {
        ESP_LOGE("DEVICE_LOGGER_DATA", "SYSTEM_DATE_TIME not found in JSON");
        cJSON_Delete(parsed_json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SYSTEM_DATE_TIME Missing");
        return ESP_FAIL;
    }

    // Extract CREATE_DATE
    cJSON *temp = cJSON_GetObjectItem(device_config, "CREATE_DATE");
    if (temp && cJSON_IsString(temp)) {
        strncpy(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE, temp->valuestring, 15 - 1);
    } else {
        strncpy(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE, "00/00/2025", 15 - 1);
    }

    // Extract CREATE_TIME
    temp = cJSON_GetObjectItem(device_config, "CREATE_TIME");
    if (temp && cJSON_IsString(temp)) {
        strncpy(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_TIME, temp->valuestring, 15 - 1);
    } else {
        strncpy(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_TIME, "00:00:00", 15 - 1);
    }

    // Extract DATE_TIME
    temp = cJSON_GetObjectItem(device_config, "DATE_TIME");
    if (temp && cJSON_IsString(temp)) {
		
        strncpy(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE_TIME, temp->valuestring, 30 - 1);
        format_datetime(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE_TIME);
        
    } else {
        strncpy(DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE_TIME, "00/00/2025T00:00:00", 30 - 1);
    }
    if(parsed_json){
    cJSON_Delete(parsed_json); // Free JSON object
    }
    // Log extracted data
    if(PRINT_DEBUG ){
    ESP_LOGI("DEVICE_LOGGER_DATA", "DATE: %s", DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE);
    ESP_LOGI("DEVICE_LOGGER_DATA", "TIME: %s", DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_TIME);
    ESP_LOGI("DEVICE_LOGGER_DATA", "DATE_TIME: %s", DEVICE_LOGGER.AUTOMATION_LOCAL_CREATE_DATE_TIME);
    }
    
    httpd_resp_send(req, "DATE & TIME UPDATED", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}


typedef struct {

    char ADDRESS_KEY[32];
    char PARAMETER_NAME[25];
    char UPDATED_VALUE[32];
    char STATUS[20];
    char REASON[60];
    
}STATUS_STRUCT;

typedef struct {
	
char SLAVE_NAME[32];
uint8_t RESULT_COUNT;

STATUS_STRUCT *STATUS_RESULT;

}RESPOND_STATUS_STRUCT;

char *RESPOND_STATUS_STATUS_CONTENT= NULL;

void RESPOND_STATUS_WRITE_RESULTS_TO_JSON2(int SLAVE_COUNT, RESPOND_STATUS_STRUCT *results){
    
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
         //printf("Error creating JSON root object\n");
        return;
    }
    
    cJSON *SLAVE_obj = cJSON_CreateObject();
    if (SLAVE_obj == NULL) {
         //printf("Error creating JSON SLAVE_obj\n");
        cJSON_Delete(root);
        return;
    }
    
    for (int SLAVE_POS = 0; SLAVE_POS < SLAVE_COUNT; SLAVE_POS++) {
        cJSON *status_obj = cJSON_CreateObject();
        if (status_obj == NULL) {
             //printf("Error creating JSON status object\n");
            continue;
        }
        
        if (results[SLAVE_POS].STATUS_RESULT == NULL) {
             //printf("Error: STATUS_RESULT is NULL for slave %s\n", results[SLAVE_POS].SLAVE_NAME);
            cJSON_Delete(status_obj);
            continue;
        }

        for (int i = 0; i < results[SLAVE_POS].RESULT_COUNT; i++) {
            cJSON *json_entry = cJSON_CreateObject();
            if (json_entry == NULL) {
                 //printf("Error creating JSON object for result %d\n", i);
                continue;
            }

            STATUS_STRUCT *status = &results[SLAVE_POS].STATUS_RESULT[i];

            cJSON_AddStringToObject(json_entry, "PARAMETER_NAME", status->PARAMETER_NAME);
            cJSON_AddStringToObject(json_entry, "UPDATED_VALUE", status->UPDATED_VALUE);
            cJSON_AddStringToObject(json_entry, "STATUS", status->STATUS);
            cJSON_AddStringToObject(json_entry, "REASON", status->REASON);

            if (status->ADDRESS_KEY[0] != '\0') {
                cJSON_AddItemToObject(status_obj, status->ADDRESS_KEY, json_entry);
            } else {
                 //printf("Warning: Empty ADDRESS_KEY for slave %s\n", results[SLAVE_POS].SLAVE_NAME);
                cJSON_Delete(json_entry);
            }
        }

        // Check if status_obj has any data before adding
        if (cJSON_GetArraySize(status_obj) > 0) {
            cJSON_AddItemToObject(SLAVE_obj, results[SLAVE_POS].SLAVE_NAME, status_obj);
        } else {
            cJSON_Delete(status_obj);
        }
    }
   
    cJSON_AddItemToObject(root, DEVICE_LOGGER.AUTOMATION_DEVICE_NAME, SLAVE_obj);
    
   char *json_string = cJSON_PrintUnformatted(root);
     if(root){
     cJSON_Delete(root);  
      }
    if (json_string == NULL){
        //strcpy(RESPOND_STATUS_STATUS_CONTENT,"DCN MEMORY FAILED");
        return;
    }
    
    uint16_t json_string_len = strlen(json_string);
    if(RESPOND_STATUS_STATUS_CONTENT) {
        free(RESPOND_STATUS_STATUS_CONTENT);
        RESPOND_STATUS_STATUS_CONTENT = NULL;
    }

    RESPOND_STATUS_STATUS_CONTENT = (char*)malloc(json_string_len + 1);
    
    if(RESPOND_STATUS_STATUS_CONTENT == NULL){
	ESP_LOGE("RESPOND_STATUS_STATUS_CONTENT","Memory allocation failed for RESPOND_STATUS_STATUS_CONTENT\n");
    if(json_string){
    free(json_string);
    json_string = NULL;
    } 
        return;
    }

   if(RESPOND_STATUS_STATUS_CONTENT){
       memset(RESPOND_STATUS_STATUS_CONTENT, 0, json_string_len + 2);
       strcpy(RESPOND_STATUS_STATUS_CONTENT, json_string);
       RESPOND_STATUS_STATUS_CONTENT[json_string_len] = '\n';
       RESPOND_STATUS_STATUS_CONTENT[json_string_len + 1] = '\0';
        }
  
   if(json_string){
    free(json_string);
    json_string = NULL;
    } 
}


void RESPOND_STATUS_WRITE_RESULTS_TO_JSON(int SLAVE_COUNT, RESPOND_STATUS_STRUCT *results){
    
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
         //printf("Error creating JSON root object\n");
        return;
    }
    
    cJSON *SLAVE_obj = cJSON_CreateObject();
    if (SLAVE_obj == NULL) {
         //printf("Error creating JSON SLAVE_obj\n");
        cJSON_Delete(root);
        return;
    }
    
    for (int SLAVE_POS = 0; SLAVE_POS < SLAVE_COUNT; SLAVE_POS++) {
        cJSON *status_obj = cJSON_CreateObject();
        if (status_obj == NULL) {
             //printf("Error creating JSON status object for SLAVE %d\n", SLAVE_POS);
            continue;
        }
        
        if (results[SLAVE_POS].STATUS_RESULT == NULL) {
             //printf("Error: STATUS_RESULT is NULL for slave %s\n", results[SLAVE_POS].SLAVE_NAME);
            cJSON_Delete(status_obj);
            continue;
        }

        bool has_valid_entries = false;  // Track if JSON contains valid data

        for (int i = 0; i < results[SLAVE_POS].RESULT_COUNT; i++) {
			
            STATUS_STRUCT *status = &results[SLAVE_POS].STATUS_RESULT[i];

            if (status == NULL) {
                 //printf("Error: STATUS_STRUCT is NULL for result %d\n", i);
                continue;
            }

            cJSON *json_entry = cJSON_CreateObject();
            if (json_entry == NULL) {
                 //printf("Error creating JSON object for result %d\n", i);
                continue;
            }
            
            cJSON_AddStringToObject(json_entry, "PARAMETER_NAME", strlen(status->PARAMETER_NAME)>3 ? status->PARAMETER_NAME : "N/A");
            cJSON_AddStringToObject(json_entry, "UPDATED_VALUE", strlen(status->UPDATED_VALUE)>3 ? status->UPDATED_VALUE : "N/A");
            cJSON_AddStringToObject(json_entry, "STATUS", strlen(status->STATUS)>3 ? status->STATUS : "N/A");
            cJSON_AddStringToObject(json_entry, "REASON", strlen(status->REASON)>3 ? status->REASON : "N/A");


            if (status->ADDRESS_KEY[0] != '\0') {
                cJSON_AddItemToObject(status_obj, status->ADDRESS_KEY, json_entry);
                has_valid_entries = true;  // Mark that we added valid data
            } else {
                 //printf("Warning: Empty ADDRESS_KEY for slave %s\n", results[SLAVE_POS].SLAVE_NAME);
                cJSON_Delete(json_entry);
            }
        }

        // Add to SLAVE_obj only if it contains valid data
        if (has_valid_entries) {
            cJSON_AddItemToObject(SLAVE_obj, results[SLAVE_POS].SLAVE_NAME, status_obj);
        } else {
            cJSON_Delete(status_obj);
        }
    }
   
    cJSON_AddItemToObject(root, DEVICE_LOGGER.AUTOMATION_DEVICE_NAME, SLAVE_obj);
    
    char *json_string = cJSON_PrintUnformatted(root);
     if(root){
     cJSON_Delete(root);  
      }
    if (json_string == NULL){
        //strcpy(RESPOND_STATUS_STATUS_CONTENT,"DCN MEMORY FAILED");
        return;
    }
    
    uint16_t json_string_len = strlen(json_string);
    if(RESPOND_STATUS_STATUS_CONTENT) {
        free(RESPOND_STATUS_STATUS_CONTENT);
        RESPOND_STATUS_STATUS_CONTENT = NULL;
    }

    RESPOND_STATUS_STATUS_CONTENT = (char*)malloc(json_string_len + 2);
    
    if(RESPOND_STATUS_STATUS_CONTENT == NULL){
	ESP_LOGE("RESPOND_STATUS_STATUS_CONTENT","Memory allocation failed for RESPOND_STATUS_STATUS_CONTENT\n");
    if(json_string){
    free(json_string);
    json_string = NULL;
    } 
        return;
    }
    
    if(RESPOND_STATUS_STATUS_CONTENT){
       memset(RESPOND_STATUS_STATUS_CONTENT, 0, json_string_len + 2);
       strcpy(RESPOND_STATUS_STATUS_CONTENT, json_string);
       RESPOND_STATUS_STATUS_CONTENT[json_string_len] = '\n';
       RESPOND_STATUS_STATUS_CONTENT[json_string_len + 1] = '\0';
        }
   
  
   if(json_string){
    free(json_string);
    json_string = NULL;
    } 
   
}



cJSON * DCN_INFORMATION_GET_HANDLER(){
	

    cJSON *root = cJSON_CreateObject();
    cJSON *dcn_logger = cJSON_CreateObject();

    // Add DCN parameters
    cJSON_AddStringToObject(dcn_logger, "DCN_SERIAL_NUMBER", NVS_STORAGE.SERIAL_NUMBER_STRING);
    cJSON_AddStringToObject(dcn_logger, "DCN_MAC_ADDRESS",  NVS_STORAGE.ESP32_MAC_ADDRESS_STR);
    cJSON_AddStringToObject(dcn_logger, "DCN_IP_ADDRESS", NVS_STORAGE.DCN_IP_ADDRESS); 
    cJSON_AddStringToObject(dcn_logger, "DCN_NAME", DEVICE_LOGGER.AUTOMATION_DEVICE_NAME);

    cJSON_AddStringToObject(dcn_logger, "ESP_version", IDF_VER);
    cJSON_AddStringToObject(dcn_logger, "DCN_version", "V11.1");
     //esp_chip_info_t chip_info;
    //esp_chip_info(&chip_info);
    //cJSON_AddNumberToObject(dcn_logger, "cores", chip_info.cores);
    cJSON_AddNumberToObject(dcn_logger, "SLAVE_COUNT", SLAVE_DEVICE_COUNT);

    // Create SLAVE_DEVICES object
    cJSON *slave_devices = cJSON_CreateObject();

    for (int i = 0; i < SLAVE_DEVICE_COUNT; i++) {
        cJSON *slave_no = cJSON_CreateObject();
        
        //cJSON_AddStringToObject(slave_no, "SLAVE_NAME", SLAVE_DEVICE[i].SLAVE_NAME);
      //  cJSON_AddNumberToObject(slave_no, "COMMUNICATION_ID", SLAVE_DEVICE[i].SLAVE_ID);
      //  cJSON_AddNumberToObject(slave_no, "SLAVE_NO", i + 1);
       // cJSON_AddStringToObject(slave_no, "SLAVE_MAKE", SLAVE_DEVICE[i].COMPANY_MAKE);
       // cJSON_AddStringToObject(slave_no, "SLAVE_MODEL", SLAVE_DEVICE[i].MAKE_MODEL);
        
        char slave_key[16];
        snprintf(slave_key, sizeof(slave_key), "SLAVE_%d", i + 1);
        cJSON_AddItemToObject(slave_devices, slave_key, slave_no);
    }

    // Attach slave_devices to dcn_logger
    cJSON_AddItemToObject(dcn_logger, "SLAVE_DEVICES", slave_devices);
    
    // Attach dcn_logger to root
    cJSON_AddItemToObject(root, "DCN_LOGGER_DATA", dcn_logger);


       return root;
}


esp_err_t SERIAL_NUMBETR_CONFIGURATION_HANDLER(httpd_req_t *req){

	   httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
	   httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "*");
	   httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

        if (req->method == HTTP_GET){
          
           cJSON *esp_info = cJSON_CreateObject();
           cJSON *esp_information = cJSON_CreateObject();
                   
		     cJSON_AddStringToObject(esp_information, "DCN_IP_ADDRESS", NVS_STORAGE.DCN_IP_ADDRESS); 
             cJSON_AddStringToObject(esp_information, "DCN_SERIAL_NUMBER", NVS_STORAGE.SERIAL_NUMBER_STRING);
             cJSON_AddStringToObject(esp_information, "ESP_version", IDF_VER);
             cJSON_AddStringToObject(esp_information, "DCN_version", "V11.1");
		     cJSON_AddStringToObject(esp_information, "WIFI_SSID", strlen(WIFI_CREDENTIAL.AP_SSID_STA_MODE)> 0 ?WIFI_CREDENTIAL.AP_SSID_STA_MODE: "NOT CONFIGURED");
		     cJSON_AddStringToObject(esp_information, "WIFI_PASSWORD", strlen(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE)> 0 ?WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE: "NOT CONFIGURED");
             cJSON_AddStringToObject(esp_information, "MAC_ADDRESS", strlen(NVS_STORAGE.ESP32_MAC_ADDRESS_STR)> 0 ? NVS_STORAGE.ESP32_MAC_ADDRESS_STR: "NOT CONFIGURED");
             cJSON_AddStringToObject(esp_information, "SERIAL_NUMBER", strlen(NVS_STORAGE.SERIAL_NUMBER_STRING) > 0 ?  NVS_STORAGE.SERIAL_NUMBER_STRING : "");
             cJSON_AddStringToObject(esp_information, "DEVICE_LOGGER_NAME",strlen(DEVICE_LOGGER.AUTOMATION_DEVICE_NAME) > 3 ? DEVICE_LOGGER.AUTOMATION_DEVICE_NAME : "" );
             
            cJSON_AddItemToObject(esp_info, "ESP_INFORMATION", esp_information);

          
             char *json_response = cJSON_PrintUnformatted(esp_info);
             if(json_response){
			 printf("ESP_INFORMATION JSON = %s",json_response);
             httpd_resp_set_type(req, "application/json");
             httpd_resp_send(req, json_response, strlen(json_response));
             }else{ 
				 httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SEVER FAILED");
			   } 
			       
               if(esp_info){
              cJSON_Delete(esp_info);
              }
              
             if(json_response){
            free(json_response);
             json_response = NULL;
             }  
             
    }else if(req->method == HTTP_POST){

        	   //printf("SERIAL_NUMBER_STRING  request receive \n");
        	  char *content = malloc(req->content_len + 1);  // Allocate memory for content


        	                      if (content == NULL){
        	   		  			     ESP_LOGE(HTTP_SERVER_TAG, "Failed to allocate memory for content");
        	   		  			     httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Request");
        	   		  			     return ESP_FAIL;
        	   		  			 }

        	   		  			 int err, remaining = req->content_len;

        	   		  			 while (remaining > 0) {
        	   		  			     err = httpd_req_recv(req, content + (req->content_len - remaining), MIN(remaining, req->content_len));
        	   		  			     if (err < 0) {
        	   		  			         if (err == HTTPD_SOCK_ERR_TIMEOUT) {
        	   		  			             continue;
        	   		  			         }
        	   		  			         ESP_LOGE(HTTP_SERVER_TAG, "Failed to receive POST data");
        	   		  			           if (content != NULL) {
        	   		  			              free(content);
        	   		  			               content = NULL; 
        	   		  			             } // Free allocated memory in case of error
        	   		  			         return ESP_FAIL;
        	   		  			     }
        	   		  			     remaining -= err;
        	   		  			 }

        	    content[req->content_len] = '\0';  // Null-terminate the string
        	     printf("content= %s \n",content);

        		if ( strlen(NVS_STORAGE.SERIAL_NUMBER_STRING) >= 12 ) {
        	         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, " SERIAL NUMBER ALREADY CONFIGURED");
                     return ESP_FAIL;
        	      }


                 cJSON *json = cJSON_Parse(content);
                 if (content != NULL) {
        	   		 free(content);
        	   		 content = NULL; 
        	   	} 
                 if (json == NULL) {
                     httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
                     return ESP_FAIL;
                 }

                 cJSON *esp_info = cJSON_GetObjectItem(json, "ESP_INFORMATION");
                 if (!cJSON_IsObject(esp_info)) {
                     httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ESP_INFORMATION object");
                     cJSON_Delete(json);
                     return ESP_FAIL;
                 }

              
               cJSON *serial_number = cJSON_GetObjectItem(esp_info, "SERIAL_NUMBER");

                 if (!cJSON_IsString(serial_number)) {   //!cJSON_IsString(mac_address) ||
                     httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SERIAL_NUMBER INVALID");
                     cJSON_Delete(json);
                     return ESP_FAIL;
                 }
                 
                char SERIAL_NUMBER_STRING[20] ;
                strncpy(SERIAL_NUMBER_STRING, serial_number->valuestring, sizeof(SERIAL_NUMBER_STRING));
               
                   if(json){
                   cJSON_Delete(json);
                   }
                   
        
         if(strlen(SERIAL_NUMBER_STRING) >= 12 && strlen(SERIAL_NUMBER_STRING) <= 15 ){
					 
			       strncpy(NVS_STORAGE.SERIAL_NUMBER_STRING, SERIAL_NUMBER_STRING, sizeof(NVS_STORAGE.SERIAL_NUMBER_STRING) - 1);
                   NVS_STORAGE.SERIAL_NUMBER_STRING[sizeof(NVS_STORAGE.SERIAL_NUMBER_STRING) - 1] = '\0';
                   ESP_LOGW("SERIAL_NUMBER_CONFIGURATION_HANDLER",": %s \n", NVS_STORAGE.SERIAL_NUMBER_STRING);

                    esp_err_t err = STORE_JSON_BODY_TO_NVS("NVS_STR_STRUCT", "LEN_NVS", "SERIAL_NUMBER",strlen(NVS_STORAGE.SERIAL_NUMBER_STRING), NVS_STORAGE.SERIAL_NUMBER_STRING, NULL);   
                    if(err == ESP_OK){
				    httpd_resp_send(req, "SERIAL NUMBER CONFIGURED SYSTEM REBOOTING....", HTTPD_RESP_USE_STRLEN);		
			        strncpy(SYSTEM_ERROR_MSG,"SERIAL_NUMBER CONFIGURATION SUCCESSFULL",sizeof(SYSTEM_ERROR_MSG)); 
			        ESP_LOGW("SERIAL_NUMBER_CONFIGURATION_HANDLER", "SERIAL_NUMBER CONFIGURATION NVS SAVE SUCCESSFULLY ESP_RESTART");
			        vTaskDelay(2000 / portTICK_PERIOD_MS);
			        WIFI_FALLBACK("ESP_RESTART");     
			      
                   }else{
			      
			        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SERIAL_NUMBER_FAILED_TO_STORE");
			        ESP_LOGW("SERIAL_NUMBER_CONFIGURATION_HANDLER", "SERIAL_NUMBER CONFIGURATION NVS SAVE FAILED");
                    strncpy(SYSTEM_ERROR_MSG,"SERIAL_NUMBER FAILED TO SAVE NVS",sizeof(SYSTEM_ERROR_MSG));
		            return ESP_FAIL;
                   } 


          }else{
                	   
                	  ESP_LOGE("SERIAL_NUMBER_CONFIGURATION_HANDLER", "SERIAL_NUMBER_EXCEED_LENGTH");
                	  httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SERIAL_NUMBER_EXCEED_LENGTH");
                	  return ESP_FAIL;
                }

             }else{
				 
                 httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "METHOD NOT ALLOWED");
                 return ESP_FAIL;
               }
             
             

             return ESP_OK;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Get MIME type from file extension
 */
static const char* get_mime_type(const char *filename) {
    if (strstr(filename, ".html")) return "text/html";
    if (strstr(filename, ".css")) return "text/css";
    if (strstr(filename, ".js")) return "application/javascript";
    if (strstr(filename, ".json")) return "application/json";
    if (strstr(filename, ".png")) return "image/png";
    if (strstr(filename, ".jpg") || strstr(filename, ".jpeg")) return "image/jpeg";
    if (strstr(filename, ".ico")) return "image/x-icon";
    if (strstr(filename, ".svg")) return "image/svg+xml";
    return "text/plain";
}


esp_err_t serve_file(httpd_req_t *req, const char *path, const char *type, bool COMPRESS_GZIP) {
	
	struct sockaddr_in6 client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sock = httpd_req_to_sockfd(req);
    getpeername(sock, (struct sockaddr *)&client_addr, &addr_len);

   char ip_str[INET6_ADDRSTRLEN];
   inet_ntop(AF_INET6, &client_addr.sin6_addr, ip_str, sizeof(ip_str));
   ESP_LOGI(TAG, "REQUEST Client IP: %d", sock);
   ESP_LOGE(TAG, "REQUEST Client IP: %s", ip_str);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
  httpd_resp_set_hdr(req, "Connection", "keep-alive");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");
  httpd_resp_set_hdr(req, "Expires", "0");
   
   SETTING_TRANSMISSION_STATUS_FLAG = false;
   MQTT_REPORT_TASK_FLAG = false;
   vTaskDelay(50/ portTICK_PERIOD_MS);
    
    uint8_t retry_count = 0;
    file_restart:
    
    if (retry_count >= SPIFF_MAX_NVS_OPEN_READ_RETRIES){
         ESP_LOGE("SERVE HTTP FILE"," Max retries reached. Aborting operation");
          SETTING_TRANSMISSION_STATUS_FLAG = true;
          MQTT_REPORT_TASK_FLAG = true;
          return ESP_FAIL;
    }
    
   FILE *file = fopen(path, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", path);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        retry_count++;
        vTaskDelay(200/ portTICK_PERIOD_MS);
        goto file_restart;
           
    }
    
  retry_count =0;
    restart:
    
    if (retry_count >= SPIFF_MAX_NVS_OPEN_READ_RETRIES){
		
         ESP_LOGE("SERVE HTTP FILE"," Max retries reached. Aborting operation.\n");
          SETTING_TRANSMISSION_STATUS_FLAG = true;
          MQTT_REPORT_TASK_FLAG = true;
         return ESP_FAIL;
    }

       char * buffer = malloc(FILE_BUFFER_SIZE);
         
        if (buffer == NULL){
			ESP_LOGE(TAG, "Failed to allocate memory for file buffer");
            fclose(file);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
            retry_count++;
            vTaskDelay(200/ portTICK_PERIOD_MS);
            goto restart;
           
        }
        
     // Enable caching for static files
       httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000");
  
       
       httpd_resp_set_type(req, type); //Set the response content type (e.g., text/html, application/json, etc.)
       httpd_resp_set_hdr(req, "Transfer-Encoding", "chunked");
      
       if (COMPRESS_GZIP){
		  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
      
       }else{
		    fseek(file, 0, SEEK_END);
            size_t file_size = ftell(file);
            rewind(file);
            char content_len_str[16];
            snprintf(content_len_str, sizeof(content_len_str), "%zu", file_size);
            httpd_resp_set_hdr(req, "Content-Length", content_len_str);
            }
      
      
    size_t len;
    while ((len = fread(buffer, 1, FILE_BUFFER_SIZE, file)) > 0){ 
		
        if(httpd_resp_send_chunk(req, buffer, len) != ESP_OK){
            ESP_LOGE(TAG, "Error sending file chunk");
             fclose(file);
              if(buffer !=NULL){
                 free(buffer);
                 buffer = NULL;
                 }
            httpd_resp_send_404(req);
            httpd_resp_sendstr_chunk(req, NULL);  // End response
            SETTING_TRANSMISSION_STATUS_FLAG = true;
            MQTT_REPORT_TASK_FLAG = true;
            return ESP_FAIL;
        }
        
      memset(buffer,0,FILE_BUFFER_SIZE);
    }
    
    httpd_resp_sendstr_chunk(req, NULL);  // End the HTTP response
    // Clean up
    if(buffer !=NULL){
    free(buffer);
    buffer = NULL;
    }
    
    fclose(file);
    ESP_LOGI(TAG, "File %s served successfully", path);
    SETTING_TRANSMISSION_STATUS_FLAG = true;
     MQTT_REPORT_TASK_FLAG = true;
    return ESP_OK;
}


esp_err_t SERVE_SERIAL_NUMBER_WIFI_CRENDENTIALS_CONFIGURATION_PAGE_HANDLER(httpd_req_t *req){

 ESP_LOGI("SERVE HTTP FILE handler"," SERVE_SERIAL_NUMBER_WIFI_CRENDENTIALS_CONFIGURATION_PAGE_HANDLER .\n");
    
	if(strlen(NVS_STORAGE.SERIAL_NUMBER_STRING)  < 12 || strcmp(NVS_STORAGE.SERIAL_NUMBER_STRING,"NOT_CONFIGUARED") == 0){
		
	    serve_file(req, "/storage/SERIAL_NUMBER.html", "text/html",false);
	
	}else{
		
	    if(COMPRESS_GZIP_FILE){
	   
	    // serve_file(req, "/storage/index.html.gz", "text/html",false);
		serve_file(req, "/storage/index.html", "text/html",false);
        
        }else{
			
         serve_file(req, "/storage/index.html", "text/html",false);
        }
        
        }
	    
	    
	return ESP_OK;
}

esp_err_t serve_WIFI_CONFIG_html(httpd_req_t *req){
	
	
     if(strlen(NVS_STORAGE.SERIAL_NUMBER_STRING) >= 12 && strcmp(NVS_STORAGE.SERIAL_NUMBER_STRING,"NOT_CONFIGUARED")!=0){
		 
	  serve_file(req, "/storage/WIFI_CREDENTIALS.html", "text/html",false);
      }
      
  	return ESP_OK;
  	
}




esp_err_t serve_INDEX_html(httpd_req_t *req) {
	
	ESP_LOGI("SERVE HTTP FILE handler"," serve_INDEX_html\n");
	
      if(strlen(NVS_STORAGE.SERIAL_NUMBER_STRING) >= 12 && strcmp(NVS_STORAGE.SERIAL_NUMBER_STRING,"NOT_CONFIGUARED")!=0){
	
      if(COMPRESS_GZIP_FILE){
		//serve_file(req, "/storage/index.html.gz", "text/html",true);
         serve_file(req, "/storage/index.html", "text/html",false);
       }else{
		  serve_file(req, "/storage/index.html", "text/html",false);
       }
 
      }
      
  	return ESP_OK;
}

esp_err_t serve_css(httpd_req_t *req){
	
	ESP_LOGI("SERVE HTTP FILE handler"," serve_css\n");
	
	 if (COMPRESS_GZIP_FILE){
		  serve_file(req, "/storage/styles.gz", "text/css",true);
		   //serve_file(req, "/storage/style.css.gz", "text/css",true);
		
     }else{
		 
         serve_file(req, "/storage/style.css", "text/css",false);
         
          }

	return ESP_OK;
}

esp_err_t serve_js(httpd_req_t *req) {
	
		ESP_LOGI("SERVE HTTP FILE handler"," serve_js\n");

	
	 if (COMPRESS_GZIP_FILE){
		 serve_file(req, "/storage/scripts.gz", "application/javascript",true);
         // serve_file(req, "/storage/script.js.gz", "application/javascript",true);
     
         }else{
			  serve_file(req, "/storage/script.js", "application/javascript",false);
         
            }


	return ESP_OK;
}

esp_err_t serve_svg(httpd_req_t *req, const char *path){
	
	return serve_file(req, path, "image/svg+xml",false);
}

esp_err_t serve_react_svg(httpd_req_t *req){

	return serve_svg(req, "/storage/react.svg");
	
}

esp_err_t serve_vite_svg(httpd_req_t *req) {

    return serve_svg(req, "/storage/vite.svg");
}


esp_err_t logo_handle(httpd_req_t *req) {


	//REQUEST_HEADER(req);
    return serve_file(req, "/storage/SENSEWELL_LOGO.svg","image/svg+xml",false);
}

esp_err_t JPG_LOGO_handle(httpd_req_t *req) {


    return serve_file(req, "/storage/SENSEWELL_LOGO.jpg","image/jpg",false);
}


esp_err_t TICK_logo_handle(httpd_req_t *req) {


    return serve_file(req, "/storage/tick.png","image/png",false);
}

esp_err_t file_icon_logo_handle(httpd_req_t *req) {

    return serve_file(req, "/storage/fileIcon.png","image/png",false);
}

esp_err_t folder_icon_logo_handle(httpd_req_t *req) {

    return serve_file(req, "/storage/folder.png","image/png",false);
}

esp_err_t ERROR_logo_handle(httpd_req_t *req) {

    return serve_file(req, "/storage/error.png","image/png",false);
}

esp_err_t favicon_get_handler(httpd_req_t *req) {

 return serve_file(req, "/storage/favicon.ico","image/x-icon",false);
 
}


//#define  CONFIG_EXAMPLE_SPIFFS_CHECK_ON_START

void INIT_SPIFFS(){

    esp_vfs_spiffs_conf_t config = {
        .base_path = "/storage",
        .partition_label = NULL,
        .max_files = 20,
        .format_if_mount_failed = false
        
    };

    esp_err_t err =  esp_vfs_spiffs_register(&config);
    if(err != ESP_OK) {
            if (err == ESP_FAIL) {
                ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
            } else if (err == ESP_ERR_NOT_FOUND) {
                ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
            } else {
                ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(err));
            }

        }



#ifdef CONFIG_EXAMPLE_SPIFFS_CHECK_ON_START

    ESP_LOGI("SPIFFS", "Performing SPIFFS_check().");
    err = esp_spiffs_check(NULL);
    if (err != ESP_OK) {
        ESP_LOGE("SPIFFS", "SPIFFS_check() failed (%s)", esp_err_to_name(err));
        return;
    } else {
        ESP_LOGI("SPIFFS", "SPIFFS_check() successful");
    }
    
#endif
    if(PRINT_DEBUG){
    
        size_t total = 0, used = 0;
        err = esp_spiffs_info(NULL, &total, &used);
       if (err != ESP_OK){
			
            ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(err));
            esp_spiffs_format(NULL);
            
        }else {
             ESP_LOGI("SPIFFS","SPIFFS = Partition size: total:   %d/%d \n", used,total);
               }

       if (used > total){
		   
            ESP_LOGI("SPIFFS", "Number of used bytes cannot be larger than total. Performing SPIFFS_check().\n");
            err = esp_spiffs_check(NULL);

        if (err != ESP_OK){
			
                ESP_LOGE("SPIFFS", "SPIFFS_check() failed (%s)", esp_err_to_name(err));
                  return;
              } else {
                  ESP_LOGI("SPIFFS", "SPIFFS_check() successful\n");
              }
          }
    }
/*

	     FILE* f;

	     f= fopen("/storage/WIFI_CREDENTIALS.html","r");
	       if(f == NULL)
	        {
	            //printf("WIFI_CRENDENTIALS_CONFIGURATION File Not Found \n");
	        }else{
	               //printf("WIFI_CRENDENTIALS_CONFIGURATION  File Found \n ");
	          }

	    fclose(f);

        f= fopen("/storage/SERIAL_NUMBER.html","r");
        if(f == NULL)
                 {
                      //printf("SERIAL_NUMBER.html File Not Found \n");
                   }else
                   {
                        //printf("SERIAL_NUMBER_CONFIGURATION.html  File Found \n ");
                   }

         fclose(f);

        f= fopen("/storage/index.html","r");
        if(f == NULL)
           {
              //printf("Index File Not Found \n");
           }else
           {
                //printf("Index.html File Found\n ");
           }

           fclose(f);



           f = fopen("/storage/style.css","r");
           if(f == NULL)
              {
                   //printf("style.css File Not Found \n");
              }
           else
           {
              //printf("style.css File Found *****\n ");
            }
            fclose(f);


             f = fopen("/storage/SENSEWELL_LOGO.jpg","r");
             if(f == NULL)
              {
               //printf("SENSEWELL_LOGO.jpg Not Found \n");
              }else
              {
               //printf("SENSEWELL_LOGO jpg File Found \n ");
              }
              fclose(f);


              f = fopen("/storage/script.js","r");
              if(f == NULL)
              {
               //printf("script JS File Not Found \n");
              }
              else
              {
               //printf("script JS File Found *****\n ");
              }


              f = fopen("/storage/FILETEXT.txt","r");
              if(f == NULL)
              {
               //printf("FILETEST. Not Found \n");
              }
              else
              {
               //printf("FILETEST.File Found \n ");
              }
              fclose(f);*/


}

 

esp_err_t cors_handle(httpd_req_t *req){


          httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
          httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "*");
          httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");



         httpd_resp_send(req, NULL, 0); // No content response

   return ESP_OK;
}



  #define HTTPD_401        "401 UNAUTHORIZED"
 #define BASIC_AUTH_USERNAME "SENSEWELL"
 #define BASIC_AUTH_PASSWORD "SENSEWELL@WDL"
 
#define CONFIG_EXAMPLE_BASIC_AUTH 1
#if CONFIG_EXAMPLE_BASIC_AUTH

typedef struct {
    char*    username;
    char*     password;
} basic_auth_info_t;

          


 char * http_auth_basic(const char *username, const char *password){
	  
	  
    size_t out;
    char *user_info = NULL;
    char *digest = NULL;
    size_t n = 0;
    
    int rc = asprintf(&user_info, "%s:%s", username, password);
    if (rc < 0) {
        ESP_LOGE(HTTP_SERVER_TAG, "asprintf() returned: %d", rc);
        return NULL;
    }

    if (!user_info) {
        ESP_LOGE(HTTP_SERVER_TAG, "No enough memory for user information");
        return NULL;
    }
    
    esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char *)user_info, strlen(user_info));

    /* 6: The length of the "Basic " string
     * n: Number of bytes for a base64 encode format
     * 1: Number of bytes for a reserved which be used to fill zero
    */
    digest = calloc(1, 6 + n + 1);
    if (digest) {
        strcpy(digest, "Basic ");
        esp_crypto_base64_encode((unsigned char *)digest + 6, n, &out, (const unsigned char *)user_info, strlen(user_info));
    }
    if(user_info!=NULL){
    free(user_info);
    user_info=NULL;
    }
    return digest;
}




 esp_err_t basic_auth_get_handler(httpd_req_t *req){
	
	
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");
	
	REQUEST_HEADER( req);
	
	 char *auth_credentials = NULL;
     char *auth_header = NULL;
     size_t auth_header_len = 0;
    
    
    basic_auth_info_t *basic_auth_info = req->user_ctx;

    auth_header_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
    if (auth_header_len > 1) {
        auth_header = calloc(1, auth_header_len);
        if (!auth_header) {
            ESP_LOGE(HTTP_SERVER_TAG, "No enough memory for basic authorization");
            return ESP_ERR_NO_MEM;
        }

      
    
     if (httpd_req_get_hdr_value_str(req, "Authorization", auth_header, auth_header_len) == ESP_OK) {
         //printf("Authorization Header: %s\n", auth_header);

        // Parse and validate the header value (Basic, Bearer, etc.)
        if (strncmp(auth_header, "Basic ", 6) == 0) {
			auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
             //printf("auth_credentials: %s \n",auth_credentials);
      
        if (!auth_credentials) {
            ESP_LOGE(HTTP_SERVER_TAG, "No enough memory for basic authorization credentials");
            free(auth_header);
            return ESP_ERR_NO_MEM;
        }
            //printf("Received Basic Auth: %s\n", auth_header + 6);  // Decode Base64 or process credentials
       
           
         
        } else if (strncmp(auth_header, "Bearer ", 7) == 0) {
            // Process the Bearer token
              //printf("Received Bearer Token: %s\n", auth_header + 7);
             
        } else {
				
             //printf("Unsupported Authorization Type\n");
               
               }
               
       } else {
		   
              //printf("Authorization Header Not Found\n");
             
              }
   
        
        if (strncmp(auth_credentials, auth_header, auth_header_len)){
            ESP_LOGE(HTTP_SERVER_TAG, "Not authenticated");
            httpd_resp_set_status(req, HTTPD_401);
            
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
            httpd_resp_send(req, NULL, 0);
        } else {
            ESP_LOGI(HTTP_SERVER_TAG, "Authenticated!");
            char *basic_auth_resp = NULL;
            httpd_resp_set_status(req, HTTPD_200);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            
            int rc = asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", basic_auth_info->username);
            if (rc < 0) {
                ESP_LOGE(HTTP_SERVER_TAG, "asprintf() returned: %d", rc);
                free(auth_credentials);
                free(auth_header);
                return ESP_FAIL;
            }
            if (!basic_auth_resp) {
                ESP_LOGE(HTTP_SERVER_TAG, "No enough memory for basic authorization response");
                free(auth_credentials);
                free(auth_header);
                return ESP_ERR_NO_MEM;
            }
            httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
            free(basic_auth_resp);
        }
        
        free(auth_credentials);
        free(auth_header);
    } else {
        ESP_LOGE(HTTP_SERVER_TAG, "No auth header received");
        httpd_resp_set_status(req, HTTPD_401);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
        httpd_resp_send(req, NULL, 0);
    }

    return ESP_OK;
}


 httpd_uri_t basic_auth = {
     .uri       = "/LOGIN",
     .method    = HTTP_GET,
    .handler   = basic_auth_get_handler,
};


  httpd_uri_t basic_auth_CORS = {
    .uri       = "/LOGIN",
    .method    = HTTP_OPTIONS,
    .handler   = cors_handle,
    .user_ctx  = NULL
               
};



 void httpd_register_basic_auth(httpd_handle_t server){
	
	
  basic_auth_info_t *basic_auth_info = calloc(1, sizeof(basic_auth_info_t));
    
    if(basic_auth_info){
		
        basic_auth_info->username = BASIC_AUTH_USERNAME;
        basic_auth_info->password = BASIC_AUTH_PASSWORD;
        httpd_register_uri_handler(server, &basic_auth_CORS);
        basic_auth.user_ctx = basic_auth_info;
        httpd_register_uri_handler(server, &basic_auth);
      //  httpd_register_uri_handler(server, &GET_basic_auth);
   
       
    }
}

#endif





typedef struct {
	
    char *response;
    int   response_len;
    int   buffer_size;
    
} http_response_t;


 http_response_t *http_res = NULL; // Make it persistent

esp_err_t http_event_handler(esp_http_client_event_t *event){
	
	
#define INITIAL_BUFFER_SIZE 2048  

    switch (event->event_id){
        
        case HTTP_EVENT_ERROR:
            ESP_LOGE(HTTP_CLIENT_TAG, "HTTP_EVENT_ERROR");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGW(HTTP_CLIENT_TAG, "HTTP_EVENT_ON_CONNECTED");
           
            if (HTTP_CLIENT_RESPOND_DATA_FLAG && !http_res ){
			    http_res = (http_response_t *) malloc(sizeof(http_response_t));
                if(!http_res){
                    ESP_LOGE(HTTP_CLIENT_TAG, "Failed to allocate memory for http_res");
                    return ESP_FAIL;
                } 
                
                http_res->response_len = 0;
                http_res->buffer_size = INITIAL_BUFFER_SIZE;
                http_res->response = (char *) malloc(http_res->buffer_size);
               
                if(!http_res->response){
			       ESP_LOGE(HTTP_CLIENT_TAG, "http_res->response");
                   if(http_res){
                   free(http_res);
                   http_res = NULL;}
                   return ESP_FAIL;
                }
            }
            
            break;

        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(HTTP_CLIENT_TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", event->header_key, event->header_value);
            break;

        case HTTP_EVENT_ON_DATA:
        
           if (!HTTP_CLIENT_RESPOND_DATA_FLAG){
			    ESP_LOGI(HTTP_CLIENT_TAG, "STORAGE HTTP_CLIENT_RESPOND_DATA_FLAG IS FALSE");
			    return ESP_FAIL;
		   }
		   
            if ( !http_res || !event->data){
                ESP_LOGE(HTTP_CLIENT_TAG, "SYSTEM DATA http_res buffer NULL");
                return ESP_FAIL;
            }
            
          
         
            if (http_res->response && http_res->response_len + event->data_len + 1 >= http_res->buffer_size && http_res->response_len + event->data_len + 10 < esp_get_free_heap_size()  ){
            
                http_res->buffer_size = http_res->response_len + event->data_len + 2;
                ESP_LOGW(HTTP_CLIENT_TAG, "CLIENT REALLOCATED BUFFER size [%d] REALLOCATED SIZE [%d]  EVENT DATA SIZE [%d]",http_res->buffer_size,http_res->response_len + event->data_len + 10 , event->data_len);
                
                char *new_buf = (char *)realloc(http_res->response, http_res->buffer_size);
                if(!new_buf){
                    ESP_LOGE(HTTP_CLIENT_TAG, "Failed to expand response buffer [%d] :%s ",event->data_len,(char*)event->data);
			      if(http_res){
					if(http_res->response){
                    free(http_res->response);}
                    free(http_res);
                    http_res = NULL;
                    }
                    return ESP_FAIL;
                }

                // ESP_LOGW(HTTP_CLIENT_TAG, "CLIENT RESPONSE BUFFER MEMORY REALLOCATED  [%d]:%s",event->data_len ,(char*)event->data);
                http_res->response = new_buf;
            }

            // Append received data
            memcpy(http_res->response + http_res->response_len, event->data, event->data_len);
            http_res->response_len += event->data_len;
            http_res->response[http_res->response_len] = '\0';  // Ensure null termination
            break;
            
        case HTTP_EVENT_ON_FINISH:
           ESP_LOGI(HTTP_CLIENT_TAG, "HTTP_EVENT_ON_FINISH");
            if (http_res && http_res->response){
                ESP_LOGI(HTTP_CLIENT_TAG, "HTTP CLIENT-SERVER Response: %s", http_res->response);
              }
            
            break;

        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGE(HTTP_CLIENT_TAG, "HTTP_client_EVENT_DISCONNECTED");
            
                 /* if(http_res){
					  if(http_res->response){
                    free(http_res->response);}
                    free(http_res);
                    http_res = NULL;
                    }*/

            break;

        default:
            ESP_LOGE(HTTP_CLIENT_TAG, "HTTP_EVENT_DEFAULT");
       }
    return ESP_OK;
}


/*
typedef enum {
	
    HTTP_METHOD_GET,     // GET request (retrieve data)
    HTTP_METHOD_POST,    // POST request (send data)
    HTTP_METHOD_PUT,     // PUT request (update data)
    HTTP_METHOD_PATCH,   // PATCH request (partially update data)
    HTTP_METHOD_DELETE,  // DELETE request (delete data)
    HTTP_METHOD_HEAD,    // HEAD request (retrieve headers only)
    HTTP_METHOD_NOTIFY,  // NOTIFY (used in event-based systems)
    HTTP_METHOD_SUBSCRIBE,  // SUBSCRIBE (for event-based updates)
    HTTP_METHOD_UNSUBSCRIBE,  // UNSUBSCRIBE (stop receiving events)
    HTTP_METHOD_OPTIONS, // OPTIONS request (query server capabilities)
    HTTP_METHOD_COPY,    // COPY request (not commonly used)
    HTTP_METHOD_MOVE,    // MOVE request (not commonly used)
    HTTP_METHOD_LOCK,    // LOCK request (for WebDAV)
    HTTP_METHOD_UNLOCK,  // UNLOCK request (for WebDAV)
    HTTP_METHOD_PROPFIND, // PROPFIND (WebDAV method)
    HTTP_METHOD_PROPPATCH,// PROPPATCH (WebDAV method)
    HTTP_METHOD_MKCOL,   // MKCOL (make collection in WebDAV)
    HTTP_METHOD_MAX      // Maximum enum value (internal use)
    
} esp_http_client_method_t;*/


int PERFORM_HTTP_REQUEST(uint8_t HTTP_REQUEST_MAX_RETRIES,const char *url,  esp_http_client_method_t http_method , const char *post_data, const char * content_type, const char **headers, int num_headers) {
	
	int status_code=0;
	uint8_t retry_count = 0;
	char *header_value = NULL;
	  
	/* #define BUFFER_SIZE 4096
   char *local_response_buffer = (char *) malloc(BUFFER_SIZE * sizeof(char));
   
 if (local_response_buffer == NULL) {
        ESP_LOGE(HTTP_CLIENT_TAG, "Failed to allocate memory for response buffer");
        return 0;
    }*/


    esp_http_client_config_t config = {
       .url = url,
     //.path
     //.query
      //.port
       .method =        http_method,
       .event_handler = http_event_handler,
    // .transport_type
    // .user_data =     local_response_buffer,
       .timeout_ms = 10000,
       .disable_auto_redirect = false,
       .use_global_ca_store = false,
       .skip_cert_common_name_check = true,
       .keep_alive_enable = true,
     
      
       
    };

    esp_http_client_handle_t HTTP_client = esp_http_client_init(&config);
    if (HTTP_client == NULL) {
    ESP_LOGE("HTTP", "Client handle is NULL! Aborting HTTP request.");
    return ESP_FAIL;
  }
  
  if(headers != NULL && num_headers > 0) {
	  
    for (int i = 0; i < num_headers; i += 2) {
        if (headers[i] && headers[i + 1]) {
            esp_http_client_set_header(HTTP_client, headers[i], headers[i + 1]);
        } else {
            ESP_LOGE(HTTP_CLIENT_TAG, "Invalid header key-value pair at index %d", i);
        }
    }
    
   }else{
      // ESP_LOGI(HTTP_CLIENT_TAG, "No headers provided, proceeding without custom headers.");
     }
   


    if ((http_method == HTTP_METHOD_POST || http_method== HTTP_METHOD_PUT)){
		
      if (content_type != NULL){
			esp_http_client_set_header(HTTP_client, "Content-Type", content_type);
        }
        
       if (post_data && post_data != NULL){
             esp_http_client_set_post_field(HTTP_client, post_data, strlen(post_data));
             }
    }

    while (retry_count < HTTP_REQUEST_MAX_RETRIES){
       
       ESP_LOGW(HTTP_CLIENT_TAG, "SENDING HTTP CLIENT REQUEST TO URL:%s",url);
       err = esp_http_client_perform(HTTP_client);
      if (err == ESP_OK) {
            status_code = esp_http_client_get_status_code(HTTP_client);
           ESP_LOGI(HTTP_CLIENT_TAG, "HTTP STATUS CODE  = %d, CONTENT LEN  = %"PRIu64,status_code,esp_http_client_get_content_length(HTTP_client));           
            
          if (status_code == 200 || status_code == 201){
			 err = esp_http_client_get_header(HTTP_client, "Content-Type", &header_value);
                 if(err == ESP_OK && header_value != NULL) {
					   ESP_LOGI(HTTP_CLIENT_TAG, "Content-Type: %s", header_value);
              }else{
               ESP_LOGE(HTTP_CLIENT_TAG, "Content-Type Header not found IN HTTP CLIENT RESPOND");
                  }
           
               if (HTTP_client != NULL) { esp_http_client_cleanup(HTTP_client) ;}
             return status_code;
             
       }else if (status_code == 301 || status_code == 302){
              char *location=NULL;
              esp_http_client_get_header(HTTP_client, "Location", &location);

           if (strlen(location) > 0){
              ESP_LOGI(HTTP_CLIENT_TAG, "Redirecting to: %s", location);
                if (HTTP_client != NULL) { esp_http_client_cleanup(HTTP_client) ;}
              HTTP_CLIENT_RESPOND_DATA_FLAG = true;
              return PERFORM_HTTP_REQUEST(HTTP_REQUEST_MAX_RETRIES, location, http_method, NULL, NULL,NULL, 0);
              }
              
        }else{
                ESP_LOGE(HTTP_CLIENT_TAG, "DCN UNEXPECTED RESPONSE, ATTEMPTING RETRY[%d/%d] : Status: %d",retry_count +1,HTTP_REQUEST_MAX_RETRIES,  status_code);
             }
       
       
        }else{
            ESP_LOGE(HTTP_CLIENT_TAG, "DCN HTTP CLIENT REQUEST FAILED: ATTEMPTING RETRY [%d/%d] ERROR REASON :%s",retry_count +1 ,HTTP_REQUEST_MAX_RETRIES, esp_err_to_name(err));
          }
                   
    
       retry_count++;
       if (HTTP_REQUEST_MAX_RETRIES > 1) {   vTaskDelay(2000 / portTICK_PERIOD_MS); }
    
    }

    if (retry_count == HTTP_REQUEST_MAX_RETRIES){
        ESP_LOGE(HTTP_CLIENT_TAG,"DCN HTTP CLIENT REQUEST FAILED MAX RETRIES REACH FAILED GIVE UP");
 
   } else{
	
        //printf("HTTP CLIENT RESPONSE BODY: %s\n",local_response_buffer);
       
        }
        
        
     /* if(local_response_buffer){
         free(local_response_buffer);
         local_response_buffer = NULL;
    }*/
   if (HTTP_client != NULL) { esp_http_client_cleanup(HTTP_client) ;}
    return status_code;
}




/*int PERFORM_HTTP_REQUEST(uint8_t HTTP_REQUEST_MAX_RETRIES,const char *url,  esp_http_client_method_t http_method , const char *post_data, const char * content_type, const char **headers, int num_headers) {
	
	
	int status_code = 0;
	int retry_count = 0;
	char *header_value = NULL;
	  
	 #define BUFFER_SIZE 4096
   char *local_response_buffer = (char *) malloc(BUFFER_SIZE * sizeof(char));
   
 if (local_response_buffer == NULL) {
        ESP_LOGE(HTTP_CLIENT_TAG, "Failed to allocate memory for response buffer");
        return 0;
    }

 esp_http_client_config_t config = {
        .url = url, 
      //.host
      //.path
      //.query
      //.port
      .cert_pem = NULL,
       //.transport_type
       //.auth_type
        //.cert_pem =        (char *)servercert_start, // GitHub's SSL root CA
       // .cert_len =        servercert_start - servercert_start,
        
        .client_cert_pem = (char *)servercert_start,
        .client_cert_len = servercert_end - servercert_start, 
        .client_key_pem =  (char *)prvtkey_pem_start,
        .client_key_len =  prvtkey_pem_end - prvtkey_pem_start,
       
        .max_redirection_count =5,
        .max_authorization_retries =5,
        .method =   http_method,
        .event_handler = http_event_handler,
        .timeout_ms = 10000,
        .disable_auto_redirect = false,
        //.is_async
        
       // .common_name
       // .use_global_ca_store = false,
          .skip_cert_common_name_check = true,
          .keep_alive_enable = true,
        
        
    };
    
    esp_http_client_config_t config = {
       .url = url,
     //.path
     //.query
      //.port
       .method =        http_method,
       .event_handler = http_event_handler,
    // .transport_type
    // .user_data =     local_response_buffer,
       .timeout_ms = 10000,
       .disable_auto_redirect = false,
       .use_global_ca_store = false,
       .skip_cert_common_name_check = true,
       .keep_alive_enable = true,
     
    
    };



    esp_http_client_handle_t HTTP_client = esp_http_client_init(&config);
    if (HTTP_client == NULL) {
    ESP_LOGE("HTTP", "Client handle is NULL! Aborting HTTP request.");
    return ESP_FAIL;
  }
  
  if(headers != NULL && num_headers > 0) {
    for (int i = 0; i < num_headers; i += 2) {
        if (headers[i] && headers[i + 1]) {
            esp_http_client_set_header(HTTP_client, headers[i], headers[i + 1]);
        } else {
            ESP_LOGE(HTTP_CLIENT_TAG, "Invalid header key-value pair at index %d", i);
        }
    }
    
   }else{
      // ESP_LOGI(HTTP_CLIENT_TAG, "No headers provided, proceeding without custom headers.");
     }
   


    if ((http_method == HTTP_METHOD_POST || http_method== HTTP_METHOD_PUT)){
		
      if (content_type != NULL){
			esp_http_client_set_header(HTTP_client, "Content-Type", content_type);
        }
        
       if (post_data && post_data != NULL){
             esp_http_client_set_post_field(HTTP_client, post_data, strlen(post_data));
             }
    }

    while (retry_count < HTTP_REQUEST_MAX_RETRIES){
       
       ESP_LOGW(HTTP_CLIENT_TAG, "SENDING HTTP CLIENT REQUEST TO URL:%s",url);
       err = esp_http_client_perform(HTTP_client);
      if (err == ESP_OK) {
		  
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status_code = esp_http_client_get_status_code(HTTP_client);
           ESP_LOGI(HTTP_CLIENT_TAG, "HTTP Status = %d, content_length = %"PRIu64,status_code,esp_http_client_get_content_length(HTTP_client));           
            
          if (status_code == 200 || status_code == 201){
			 err = esp_http_client_get_header(HTTP_client, "Content-Type", &header_value);
                 if(err == ESP_OK && header_value != NULL) {
					   ESP_LOGI(HTTP_CLIENT_TAG, "Content-Type: %s", header_value);
              }else{
               ESP_LOGE(HTTP_CLIENT_TAG, "Content-Type Header not found IN HTTP CLIENT RESPOND");
                  }
           
             esp_http_client_cleanup(HTTP_client);
             return status_code;
             
       }else if (status_code == 301 || status_code == 302){
              char *location=NULL;
              esp_http_client_get_header(HTTP_client, "Location", &location);

           if (strlen(location) > 0){
              ESP_LOGI(HTTP_CLIENT_TAG, "Redirecting to: %s", location);
              esp_http_client_cleanup(HTTP_client);
              HTTP_CLIENT_RESPOND_DATA_FLAG = true;
              return PERFORM_HTTP_REQUEST(HTTP_REQUEST_MAX_RETRIES, location, http_method, NULL, NULL,NULL, 0);
              }
              
        }else{
                ESP_LOGE(HTTP_CLIENT_TAG, "Unexpected response, retrying = [%d/%d] : Status: %d",retry_count +1,HTTP_REQUEST_MAX_RETRIES,  status_code);
             }
       
       
        }else{
            ESP_LOGE(HTTP_CLIENT_TAG, "HTTP CLIENT REQUEST FAILED:  retrying = [%d/%d] ERROR REASON :%s",retry_count +1 ,HTTP_REQUEST_MAX_RETRIES, esp_err_to_name(err));
          }
                   
         
       retry_count++;
       vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay between retries (2 second)
    }

    if (retry_count == HTTP_REQUEST_MAX_RETRIES){
        ESP_LOGE(HTTP_CLIENT_TAG,"HTTP CLIENT REQUEST FAILED MAX RETRIES REACH FAILED GIVE UP");
 
   } else{
	
        //printf("HTTP CLIENT RESPONSE BODY: %s\n",local_response_buffer);
       
        }
        
        
      if(local_response_buffer){
         free(local_response_buffer);
         local_response_buffer = NULL;
    }
    esp_http_client_cleanup(HTTP_client);
    return status_code;
}*/



void REQUEST_HEADER( httpd_req_t *req){
   
   if(SYSTEM_DEBUG){
	  // req->content_len
	  // req->uri
	   //req->method
	   //req->handle
	   // req->user_ctx
	   
    ESP_LOGI( "HTTP_SERVER_HEADER", "ESP SERVER_HEADER.......");
    struct sockaddr_in6 client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sock = httpd_req_to_sockfd(req);
    getpeername(sock, (struct sockaddr *)&client_addr, &addr_len);

   char ip_str[INET6_ADDRSTRLEN];
   inet_ntop(AF_INET6, &client_addr.sin6_addr, ip_str, sizeof(ip_str));
   ESP_LOGI("REQUEST_HEADER", "REQUEST Client SOCKET ID: %d", sock);
   ESP_LOGI("REQUEST_HEADER", "REQUEST content_len: %d",  req->content_len);
   ESP_LOGI("REQUEST_HEADER", "REQUEST method: %d", req->method);
  //ESP_LOGI("REQUEST_HEADER", "REQUEST user_ctx: %s", req->user_ctx);
   ESP_LOGI("REQUEST_HEADER", "REQUEST URI: %s", req->uri);


	char header_val[150];
	
    size_t header_val_size = sizeof(header_val);
    
    if (httpd_req_get_hdr_value_str(req, "Content-Type", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Content-Type: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Content-Length", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Content-Length: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "User-Agent", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "User-Agent: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Host", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Host: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Accept", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Accept: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Accept-Encoding", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Accept-Encoding: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Accept-Language", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Accept-Language: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Cache-Control", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Cache-Control: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Connection", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Connection: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Upgrade", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Upgrade: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Origin", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Origin: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Authorization", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Authorization: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Access-Control-Allow-Origin", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Access-Control-Allow-Origin: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Access-Control-Allow-Methods", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Access-Control-Allow-Methods: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Access-Control-Allow-Headers", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Access-Control-Allow-Headers: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Sec-WebSocket-Key", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Sec-WebSocket-Key: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Sec-WebSocket-Version", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Sec-WebSocket-Version: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Sec-WebSocket-Extensions", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Sec-WebSocket-Extensions: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Referer", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Referer: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "If-Modified-Since", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "If-Modified-Since: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "If-None-Match", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "If-None-Match: %s\n", header_val);
    }
    if (httpd_req_get_hdr_value_str(req, "Cookie", header_val, header_val_size) == ESP_OK) {
        ESP_LOGI( "HTTP_SERVER_HEADER", "Cookie: %s\n", header_val);
    }

}

}

void http_server_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){

 #define HTTPS_SERVER_TAG "DCN_HTTPS_EVENT_SERVER"
 #define EVENT_SERVER_TAG "DCN_HTTP_EVENT_SERVER"

   if(event_base == ESP_HTTP_SERVER_EVENT){

	
    switch (event_id){
		
        case HTTP_SERVER_EVENT_ERROR:
            ESP_LOGE(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_ERROR \n");
            
            break;
        case HTTP_SERVER_EVENT_START:
            ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_START \n");
            break;
        case HTTP_SERVER_EVENT_ON_CONNECTED:
            ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_ON_CONNECTED \n");
            break;
        case HTTP_SERVER_EVENT_ON_HEADER:
        //    ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_ON_HEADER\n");
            break;
        case HTTP_SERVER_EVENT_HEADERS_SENT:
            ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_HEADERS_SENT\n");
            break;
        case HTTP_SERVER_EVENT_ON_DATA:
     //      ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_ON_DATA   \n");
         break;
       case HTTP_SERVER_EVENT_SENT_DATA:
          // ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_SENT_DATA  \n");
           break;
        case HTTP_SERVER_EVENT_DISCONNECTED:
            ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_DISCONNECTED\n");
            break;
        case HTTP_SERVER_EVENT_STOP:
            ESP_LOGI(EVENT_SERVER_TAG, "HTTP_SERVER_EVENT_STOP \n");
            break;
        default:
            ESP_LOGI(EVENT_SERVER_TAG, "Unknown event ID: %lu\n", event_id);
            break;
    }
    
 
 
  }else if(event_base == ESP_HTTPS_SERVER_EVENT){
  
  switch (event_id){
		
		
		
        case HTTPS_SERVER_EVENT_ERROR:
        
            ESP_LOGE(HTTPS_SERVER_TAG, "HTTP_SERVER_EVENT_ERROR \n");
          
           esp_https_server_last_error_t *last_error = (esp_tls_last_error_t *) event_data;
            ESP_LOGE(HTTPS_SERVER_TAG, "ERROR last_error NAME = %s, last_tls_err CODE = %d, tls_FLAG= %d", esp_err_to_name(last_error->last_error), last_error->esp_tls_error_code, last_error->esp_tls_flags);
            
            ESP_LOGI(HTTPS_SERVER_TAG, "free heap at error: %u", xPortGetFreeHeapSize());
            char buf[128];
            mbedtls_strerror(last_error->esp_tls_error_code, buf, sizeof(buf));
            ESP_LOGI(HTTPS_SERVER_TAG, "mbedtls_strerror: %s", buf);

           
            break;
           case HTTPS_SERVER_EVENT_START:
            ESP_LOGI(HTTPS_SERVER_TAG, "HTTPS_SERVER_EVENT_START\n");
            break;
           case HTTPS_SERVER_EVENT_ON_CONNECTED:
            ESP_LOGI(HTTPS_SERVER_TAG, "HTTPS_SERVER_EVENT_ON_CONNECTED\n");
            break;
            case HTTPS_SERVER_EVENT_ON_DATA:
         //   ESP_LOGI(HTTPS_SERVER_TAG, "HTTPS_SERVER_EVENT_ON_DATA\n");
            break;
           case HTTPS_SERVER_EVENT_SENT_DATA:
          // ESP_LOGI(HTTPS_SERVER_TAG, "HTTPS_SERVER_EVENT_SENT_DATA\n");
           break;
           case HTTPS_SERVER_EVENT_DISCONNECTED:
            ESP_LOGI(HTTPS_SERVER_TAG, "HTTPS_SERVER_EVENT_DISCONNECTED\n");
            break;
           case HTTPS_SERVER_EVENT_STOP:
            ESP_LOGI(HTTPS_SERVER_TAG, "HTTPS_SERVER_EVENT_STOP\n");
            break;
           default:
            ESP_LOGI(HTTPS_SERVER_TAG, "Unknown event ID: %lu\n", event_id);
            break;
    }
  
  
  
  }else if (event_base == ESP_HTTPS_OTA_EVENT){
        switch (event_id) {
            case ESP_HTTPS_OTA_START:
                ESP_LOGI(EVENT_SERVER_TAG, "OTA started");
                break;
            case ESP_HTTPS_OTA_CONNECTED:
                ESP_LOGI(EVENT_SERVER_TAG, "Connected to server");
                break;
            case ESP_HTTPS_OTA_GET_IMG_DESC:
                ESP_LOGI(EVENT_SERVER_TAG, "Reading Image Description");
                break;
            case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
                ESP_LOGI(EVENT_SERVER_TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
                break;
            case ESP_HTTPS_OTA_DECRYPT_CB:
                ESP_LOGI(EVENT_SERVER_TAG, "Callback to decrypt function");
                break;
            case ESP_HTTPS_OTA_WRITE_FLASH:
                ESP_LOGD(EVENT_SERVER_TAG, "Writing to flash: %d written", *(int *)event_data);
                break;
            case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
                ESP_LOGI(EVENT_SERVER_TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
                break;
            case ESP_HTTPS_OTA_FINISH:
                ESP_LOGI(EVENT_SERVER_TAG, "OTA finish");
                break;
            case ESP_HTTPS_OTA_ABORT:
                ESP_LOGI(EVENT_SERVER_TAG, "OTA abort");
                break;
              default:
            ESP_LOGI(EVENT_SERVER_TAG, "Unknown event ID: %lu\n", event_id);
            break;   
        }
   
   
    }else{ ESP_LOGE("HTTP_OTA_EVENT_HANDLER", "Unknown EVENT BASE : %s, event_id=%lu",event_base, event_id);}
 
  
  
  
}
void print_server_cert_info(const unsigned char *cert_pem, size_t cert_len){
	
char err_buf[100];
printf("----- Server Certificate Size :%d bytes \n", cert_len);
 mbedtls_x509_crt cert;
 mbedtls_x509_crt_init(&cert);
 
    int ret = mbedtls_x509_crt_parse(&cert, cert_pem, cert_len);
    if (ret == 0) {
		const size_t buf_size = 3024;
       char *buf = calloc(buf_size, sizeof(char));
        if (buf == NULL) {
         ESP_LOGE(TAG, "Out of memory - Callback execution failed!");
         return;
       }
  
        mbedtls_x509_crt_info((char *) buf, buf_size - 1, "DCN SERVER  ", &cert);
        printf("ESP SEVER Certificate Info:\n%s\n", buf);
        if (buf != NULL) {
         free(buf);
         buf = NULL;
        }
    
    }else{
	
        mbedtls_strerror(ret, err_buf, sizeof(err_buf));
        printf("Failed to parse certificate. Error: %s\n", err_buf);
    }
   
    mbedtls_x509_crt_free(&cert);
}



 void print_peer_cert_info( mbedtls_ssl_context *ssl){
  
  
    const mbedtls_x509_crt *cert;
    #define  buf_size  3024
    
    char *buf = calloc(buf_size, sizeof(char));
    if (buf == NULL) {
        ESP_LOGE(TAG, "Out of memory - Callback execution failed!");
        return;
    }

    cert = mbedtls_ssl_get_peer_cert(ssl);
    if (cert != NULL) {
        mbedtls_x509_crt_info( buf, buf_size - 1, "PEER CLIENT   ", cert);
        ESP_LOGI(TAG, "Peer certificate info:\n%s", buf);
    } else {
        ESP_LOGW(TAG, "Could not obtain the peer certificate!");
    }

    if (buf != NULL){
         free(buf);
         buf = NULL;
        }
        
        
}

int sockfd = -1;
mbedtls_ssl_context *ssl_ctx = NULL;

void https_server_user_callback(esp_https_server_user_cb_arg_t *user_cb){
 
 if (!user_cb) return;

    ESP_LOGW(" ", "===========================User callback invoked===================!");
    
    
 HEAP_STACK_MEMORY_PRINT("https_server_user_callback");      


  esp_tls_conn_state_t state;
   err = esp_tls_get_conn_state(user_cb->tls, &state);
if (err == ESP_OK){
    switch (state) {
        case ESP_TLS_INIT:
            ESP_LOGI("HTTPS User STATE ", "TLS State: INIT");
            break;
        case ESP_TLS_CONNECTING:
            ESP_LOGI("HTTPS User STATE ", "TLS State: CONNECTING");
            break;
        case ESP_TLS_HANDSHAKE:
            ESP_LOGI("HTTPS User STATE ", "TLS State: HANDSHAKE");
            break;
        case ESP_TLS_DONE:
            ESP_LOGI("HTTPS User STATE ", "TLS State: DONE (Connection Established)");
            break;
        case ESP_TLS_FAIL:
            ESP_LOGI("HTTPS User STATE ", "TLS State: FAIL");
            // Handle failure or retry
            break;
        default:
            ESP_LOGI("HTTPS UserSTATE ", "TLS State: Unknown");
            break;
    }
}else{
    ESP_LOGE("HTTPS UserSTATE ", "Failed to get TLS connection state");
     }
     
     

    switch(user_cb->user_cb_state){
		
    case HTTPD_SSL_USER_CB_SESS_CREATE:
           ESP_LOGW("HTTPD_SSL_USER_CB_SESS_CREATE", "SESSION USER CALL BACK Session CREATE ");

            // Logging the socket FD
              ESP_LOGI("HTTPS User callback ", "esp_tls_get_bytes_avail: %d",  esp_tls_get_bytes_avail(user_cb->tls));
            
              
            err = esp_tls_get_conn_sockfd(user_cb->tls, &sockfd);
            if(err != ESP_OK){
                ESP_LOGE("HTTPS User callback ", "Error in obtaining the sockfd from tls context");
                break;
            } ESP_LOGI("HTTPS User callback ", "TLS CREATE Socket FD: %d", sockfd);
            
          
            ssl_ctx = (mbedtls_ssl_context *) esp_tls_get_ssl_context(user_cb->tls);
            if (ssl_ctx) {
                 
                 ESP_LOGI("HTTPS User callback ", "Current Ciphersuite: %s", mbedtls_ssl_get_ciphersuite(ssl_ctx));
                 ESP_LOGI("HTTPS User callback ", "Current Ciphersuite: %d", mbedtls_ssl_get_ciphersuite_id_from_ssl(ssl_ctx));
                 ESP_LOGI("HTTPS User callback ", "ciphersuite_name: %s",   mbedtls_ssl_get_ciphersuite_name(mbedtls_ssl_get_ciphersuite_id_from_ssl(ssl_ctx)));
                 ESP_LOGI("HTTPS User callback ", "TLS version: %s", mbedtls_ssl_get_version( ssl_ctx));
              //CRASH      // ESP_LOGI("HTTPS User callback ", "alpn_protocol: %s",   mbedtls_ssl_get_alpn_protocol(ssl_ctx));
                ESP_LOGI("HTTPS User callback ", "max_in_record_payload: %d", mbedtls_ssl_get_max_in_record_payload(ssl_ctx));
                ESP_LOGI("HTTPS User callback ", "mbedtls_ssl_get_bytes_avail: %d", mbedtls_ssl_get_bytes_avail(ssl_ctx));
                ESP_LOGI("HTTPS User callback ", "mbedtls_ssl_get_user_data_n: %d", mbedtls_ssl_get_user_data_n(ssl_ctx));
                
               // ESP_LOGI("HTTPS User callback ", "mbedtls_ssl_get_user_data_n: %d", mbedtls_ssl_get_user_data_p(ssl_ctx));
               
            
                
         //CRASH     //  ESP_LOGI("HTTPS User callback ", "mbedtls_ssl_get_hostname: %s",mbedtls_ssl_get_hostname(ssl_ctx));
                     print_peer_cert_info(ssl_ctx);
            
 
                
                // size_t name_len=0 ;
                //  mbedtls_ssl_get_hs_sni(ssl_ctx, &name_len);
               //  uint8_t verify_result = mbedtls_ssl_get_verify_result(ssl_ctx);
               
         
         
        /* mbedtls_ssl_session *session = NULL;
          session = malloc(sizeof(mbedtls_ssl_session));
          if(session != NULL){
          mbedtls_ssl_session_init(session);
          mbedtls_ssl_get_session(ssl_ctx, session);
        
       ESP_LOGI("HTTPS User callback ", "Cipher suite: %d", session->private_ciphersuite);
       ESP_LOGI("HTTPS User callback ", "Ticket creation_time : %llu sec", session->private_ticket_creation_time);
       ESP_LOGI("HTTPS User callback ", "Ticket lifetime: %lu sec", session->private_ticket_lifetime);
       ESP_LOGI("HTTPS User callback ", "Session ticket length: %d", session->private_ticket_len); 
       ESP_LOGI("HTTPS User callback ","Session Master Key %s", session->private_master);
 
       ESP_LOGI("HTTPS User callback ","private_verify_result  %lu", session->private_verify_result);
       ESP_LOGI("HTTPS User callback ","private_endpoint  %d", session->private_endpoint);
        
      //ESP_LOGI("HTTPS User callback ","Session Master Key %s", session->private_peer_cert);
     // ESP_LOGI("HTTPS User callback ","Session Master Key %s", session->private_tls_version);
             
         mbedtls_ssl_session_free(session);
         if(session != NULL){
			ESP_LOGI("HTTPS User callback ", "FREEING mbedtls_ssl_session");
			free(session);
            session =NULL;
            }
      
      }else{
        ESP_LOGE("HTTPS User callback ", "Failed to allocate memory for mbedtls_ssl_session");
           } */
          
   
    
   }//SSL
    
    break;
    
    case HTTPD_SSL_USER_CB_SESS_CLOSE:
    
         ESP_LOGW("HTTPD_SSL_USER_CB_SESS_CLOSE", "HTTPD_SSL_USER_CB_SESS_CLOSE At session close");
         
             err = esp_tls_get_conn_sockfd(user_cb->tls, &sockfd);
             if(err == ESP_OK){  
               ESP_LOGI("HTTPS User callback ", "TLS CLOSE Socket FD: %d", sockfd);
             }  
           
             if(ssl_ctx){  mbedtls_ssl_close_notify(ssl_ctx);  }
             
              HEAP_STACK_MEMORY_PRINT(" HTTPD_SSL_USER_CB_SESS_CLOSE"); 
              
            break;
         
        default:
            ESP_LOGE("HTTPS User callback ", "Illegal state!");
            return;
    }
    
   
}



typedef struct{
	
    bool auth_done;
    char username[32];
    uint32_t request_count;
    TickType_t last_activity;
    uint32_t total_bytes_sent;
    uint32_t total_bytes_received;
    char client_ip[16];
    uint32_t failed_auth_count;
    bool is_api_client;
    char jwt_token[128];
    char temp_buffer[256];
    uint8_t compression_enabled;
    uint16_t keepalive_counter;
    bool     download_in_progress;
} session_data_t;


/* void http_socket_ctx_func(void *ctx){
	
    free(ctx); // Could be more complex if we malloc inside struct
}*/





/* void key_export_callback(void *p_expkey, const unsigned char *ms, const unsigned char *kb, size_t kb_len, const unsigned char client_random[32], const unsigned char server_random[32], mbedtls_tls_prf_types tls_prf){
    
    printf("=================Master secret:========= ");
    for (int i = 0; i < 48; i++) printf("%02x", ms[i]);
    printf("\n");
}
*/


esp_err_t FETCH_HTTP_POST_CONTENT(httpd_req_t *req, char **content_out)
{
	#define TAG_FETCH "FETCH_HTTP_POST_CONTENT"
	 
    if (!req || !content_out) {
        ESP_LOGE(TAG_FETCH, "Invalid argument");
        return ESP_FAIL;
    }

    size_t content_len = req->content_len;

    if (content_len == 0) {
        ESP_LOGW(TAG_FETCH, "No content received");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    // Free any previously allocated buffer
    if (*content_out) {
        free(*content_out);
        *content_out = NULL;
    }

    // Allocate new buffer (+1 for '\0')
    *content_out = (char *)calloc(content_len + 1, 1);
    if (!*content_out) {
        ESP_LOGE(TAG_FETCH, "Failed to allocate %d bytes", content_len);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    size_t received = 0;
    while (received < content_len) {
        int ret = httpd_req_recv(req, *content_out + received, content_len - received);
        if (ret < 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                ESP_LOGW(TAG_FETCH, "Socket timeout, retrying...");
                continue;
            }
            ESP_LOGE(TAG_FETCH, "Receive failed: %d", ret);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
            free(*content_out);
            *content_out = NULL;
            return ESP_FAIL;
            
        } else if (ret == 0) {
            ESP_LOGE(TAG_FETCH, "Connection closed by peer");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Connection closed");
            free(*content_out);
            *content_out = NULL;
            return ESP_FAIL;
        }
        received += ret;
    }

    (*content_out)[content_len] = '\0';
    ESP_LOGI(TAG_FETCH, "Received %d bytes POST content", content_len);

    return ESP_OK;
}

void safe_destroy_connection(int sockfd){
	

   int clients[10]; // max 10 clients
   size_t client_count = 10;
   HEAP_STACK_MEMORY_PRINT("safe_destroy_connection"); 
   if(server != NULL){
    esp_err_t ret = httpd_get_client_list( server , &client_count, clients);
    if(ret == ESP_OK){
     ESP_LOGI("safe_destroy_connection","Connected clients: %d\n", client_count);
     for (int i = 0; i < client_count; i++){
     ESP_LOGI("safe_destroy_connection","ACTIVE CLIENT CLOSSING SESSION FD: %d\n", clients[i]);
     httpd_sess_trigger_close(server, clients[i]);
    }
   
  }else{
    ESP_LOGE("safe_destroy_connection", "Failed to get client list\n");
       }
   }
   
     
}

esp_err_t http_socket_open_fd(httpd_handle_t hd, int sock){
	
    ESP_LOGW("wss_open_fd", "========New client connected %d===================", sock);
    HEAP_STACK_MEMORY_PRINT(" http_socket_open_fd"); 
  
     char ip_str[INET6_ADDRSTRLEN]; 
     struct sockaddr_in6 client_addr;
     socklen_t addr_len = sizeof(client_addr);

   getpeername(sock, (struct sockaddr *)&client_addr, &addr_len);
   inet_ntop(AF_INET6, &client_addr.sin6_addr, ip_str, sizeof(ip_str));
   ESP_LOGI("wss_open_fd", "REQUEST Client SOCKET ID: %d", sock);
   ESP_LOGI("wss_open_fd", "REQUEST Client IP: %s", ip_str);
 

 
   int clients[10]; // max 10 clients
   size_t client_count = 10;


    esp_err_t ret = httpd_get_client_list( hd , &client_count, clients);
    if(ret == ESP_OK){
     ESP_LOGI("wss_open_fd","Connected clients: %d", client_count);
    for (int i = 0; i < client_count; i++){
     ESP_LOGI("wss_open_fd","ACTIVE CLIENT FD: %d", clients[i]);
    }
    
  }else{
    ESP_LOGE("wss_open_fd", "Failed to get client list\n");
       }
    
/*  if(httpd_ws_get_fd_info(hd, sock) == HTTPD_WS_CLIENT_WEBSOCKET){
		 
   ESP_LOGI("wss_open_fd", "WEBSOCKET Active client (fd=%d) -> WEBSOCKET CONNECTION", sock);
 //  wss_keep_alive_t KEEP_ALIVE = httpd_get_global_user_ctx(hd);
 //  wss_keep_alive_add_client(KEEP_ALIVE, sock);
 }else{
	 ESP_LOGI("wss_open_fd", "WEBSOCKET Active client (fd=%d) ==TYPE [%d] -> HTTP CONNECTION", sock,httpd_ws_get_fd_info(hd, sock));
 }*/
    
    return   ESP_OK;
}


void http_socket_close_fd(httpd_handle_t hd, int sockfd){
	
    ESP_LOGW("wss_close_fd", "======== CLOSE Client DISSSCOUNT:  %d========", sockfd);

   char ip_str[INET6_ADDRSTRLEN]; 
   struct sockaddr_in6 client_addr;
   socklen_t addr_len = sizeof(client_addr);
   getpeername(sockfd, (struct sockaddr *)&client_addr, &addr_len);
   inet_ntop(AF_INET6, &client_addr.sin6_addr, ip_str, sizeof(ip_str));
   ESP_LOGI("wss_close_fd", "CLOSE Client SOCKET ID: %d", sockfd);
   ESP_LOGI("wss_close_fd", "CLOSE Client  IP: %s", ip_str);
   
   int clients[10]; // max 10 clients
   size_t client_count = 10;

    esp_err_t ret = httpd_get_client_list( hd , &client_count, clients);
    if(ret == ESP_OK){
     ESP_LOGI("wss_close_fd","Connected clients: %d", client_count);
    for (int i = 0; i < client_count; i++){
     ESP_LOGI("wss_close_fd","ACTIVE CLIENT[%d] FD: %d",i,clients[i]);
    }
     }else{
	  
    ESP_LOGE("wss_close_fd", "Failed to get client list\n");
       }
       
       
/*      remove_ws_client(sockfd);
   if(httpd_ws_get_fd_info(hd, sockfd) == HTTPD_WS_CLIENT_WEBSOCKET){
     
      ESP_LOGI("wss_close_fd", "CLOSING client (fd=%d) -> WEBSOCKET CONNECTION", sockfd);
      //wss_keep_alive_t KEEP_ALIVE = httpd_get_global_user_ctx(hd);
     // wss_keep_alive_remove_client(KEEP_ALIVE, sockfd);
  
  }else{
	 ESP_LOGI("wss_close_fd", "WEBSOCKET CLOSE client (fd=%d) -> HTTP CONNECTION", sockfd);
       }*/
       
   HEAP_STACK_MEMORY_PRINT("before close(sockfd)"); 
    close(sockfd);
    HEAP_STACK_MEMORY_PRINT("after close(sockfd)"); 
      
}	
esp_err_t redirect_get_handler(httpd_req_t *req){
	
    ESP_LOGW(TAG, "========================================redirect_get_handler URI=====================");
    char host[64];
    char location[2088];
    REQUEST_HEADER(req);
      int sockfd = httpd_req_to_sockfd(req);
    // Get "Host" header from HTTP request
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) == ESP_OK){
		
        sprintf(location, "https://%s%s",host, req->uri);
    } else{
        // Fallback if Host header not found
        sprintf(location,  "https://%s%s",NVS_STORAGE.DCN_IP_ADDRESS, req->uri);
        ESP_LOGE(TAG, "REQUEST HOST HEADER NOT FOUND: %s", host);
    }
    
    ESP_LOGI(TAG, "REQUEST HOST: %s", host);
    ESP_LOGI(TAG, "URL URI: %s", req->uri);
    ESP_LOGI(TAG, "URLREQUEST CONNTENT LEN: %d", req->content_len);
    ESP_LOGI(TAG, "URL METHOD: %d", req->method);
    //ESP_LOGI(TAG, "URL sess_ctx %", req->sess_ctx);
   
     
     
     ESP_LOGI(TAG, "Redirecting to: %s", location);
    //httpd_set_status(301);
    httpd_resp_set_status(req, "301 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", location);
    httpd_resp_send(req, NULL, 0);  // No body
  
  // CLOSE SESSION
    httpd_sess_trigger_close(req->handle,sockfd);
    
    return ESP_OK;
    
}




esp_err_t start_http_redirect_server(void){
	
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.uri_match_fn =   httpd_uri_match_wildcard; // Enable wildcard matching
    config.task_priority  = tskIDLE_PRIORITY + 2;
    config.stack_size = 5360; // 
    config.keep_alive_enable = true;      // Enable keep-alive
    config.keep_alive_idle =    5;         // Idle time before sending keep-alive packets
    config.keep_alive_interval = 2;      // Send keep-alive every 5 seconds
    config.keep_alive_count =    3;         // Retry 3 times before closing connection
    config.recv_wait_timeout =   5;        // Lower if ESP32 gets slow
    config.send_wait_timeout =   5;        // Prevent long waits
    config.max_open_sockets   =  2;
    config.lru_purge_enable = true;
    config.core_id  = 0;
  //  config.open_fn =  http_socket_open_fd;
  //  config.close_fn = http_socket_close_fd;

    if (httpd_start(&HTTP_server, &config) == ESP_OK){
	
		  esp_event_handler_register(ESP_HTTP_SERVER_EVENT,ESP_EVENT_ANY_ID  ,http_server_event_handler, NULL);

        httpd_uri_t redirect_uri = {
            .uri      = "/*",  // Wildcard for all URIs
            .method   = HTTP_GET,
            .handler  = redirect_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(HTTP_server, &redirect_uri);
      
        return ESP_OK;
    }
    return ESP_FAIL;
}


void http_server_task(void *arg){
	

/*
typedef enum {
    HTTPD_WS_TYPE_CONTINUE   = 0x0,
    HTTPD_WS_TYPE_TEXT       = 0x1,
    HTTPD_WS_TYPE_BINARY     = 0x2,
    HTTPD_WS_TYPE_CLOSE      = 0x8,
    HTTPD_WS_TYPE_PING       = 0x9,
    HTTPD_WS_TYPE_PONG       = 0xA
} httpd_ws_type_t;

    HTTPD_WS_CLIENT_INVALID        = 0x0,
    HTTPD_WS_CLIENT_HTTP           = 0x1,
    HTTPD_WS_CLIENT_WEBSOCKET      = 0x2,
} httpd_ws_client_info_t;

#define HTTPD_200      "200 OK"                     !< HTTP Response 200 
#define HTTPD_204      "204 No Content"             !< HTTP Response 204 
#define HTTPD_207      "207 Multi-Status"           !< HTTP Response 207 
#define HTTPD_400      "400 Bad Request"            !< HTTP Response 400 
#define HTTPD_404      "404 Not Found"              !< HTTP Response 404 
#define HTTPD_408      "408 Request Timeout"        !< HTTP Response 408 
#define HTTPD_500      "500 Internal Server Error"  !< HTTP Response 500 

#define HTTPD_TYPE_JSON   "application/json"            //!< HTTP Content type JSON 
#define HTTPD_TYPE_TEXT   "text/html"                   //!< HTTP Content type text/HTML 
#define HTTPD_TYPE_OCTET  "application/octet-stream"    //!< HTTP Content type octext-stream 

//httpd_resp_send_408(httpd_req_t *r)  == httpd_resp_send_err(r, HTTPD_408_REQ_TIMEOUT, NULL);
// httpd_resp_send_500(httpd_req_t *r )= httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, NULL);
//httpd_resp_send_404(httpd_req_t *r) == httpd_resp_send_err(r, HTTPD_404_NOT_FOUND, NULL);

*/

   /*   wss_keep_alive_config_t keep_alive_config = KEEP_ALIVE_CONFIG_DEFAULT();
      keep_alive_config.max_clients= MAX_CLIENTS;
      keep_alive_config.client_not_alive_cb =   client_not_alive_cb;
      keep_alive_config.check_client_alive_cb = check_client_alive_cb;
      keep_alive_config.keep_alive_period_ms = 15000;
      keep_alive_config.not_alive_after_ms  =  25000;
 */
  //  wss_keep_alive_t keep_alive = wss_keep_alive_start(&keep_alive_config);

   
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
   
     //  conf.httpd.global_user_ctx = keep_alive;
   
    conf.servercert =     servercert_start;
    conf.servercert_len = servercert_end - servercert_start;

    conf.prvtkey_pem =     prvtkey_pem_start;
    conf.prvtkey_len =     prvtkey_pem_end - prvtkey_pem_start;
   
    //conf.cert_select_cb = my_cert_select_cb; 
   // conf.cacert_pem = NULL,                           
   // conf.cacert_len = 0,       
   
    conf.httpd.keep_alive_enable = true;        // Enable keep-alive
    conf.httpd.keep_alive_idle =    10;         // Idle time before sending keep-alive packets
    conf.httpd.keep_alive_interval = 5;         // Send keep-alive every 5 seconds
    conf.httpd.keep_alive_count =    1;         // Retry 3 times before closing connection
    conf.httpd.recv_wait_timeout =   5;         //  Lower if ESP32 gets slow
    conf.httpd.send_wait_timeout =   5;         //  Prevent long waits
              //MAX_CLIENTS;
    conf.httpd.lru_purge_enable = true;

   
  //conf.ssl_userdata = NULL,  
  //conf.httpd.global_user_ctx = keep_alive;
    conf.httpd.max_uri_handlers = 95;
    conf.httpd.max_resp_headers = 40;
    conf.httpd.task_priority  = tskIDLE_PRIORITY + 3;
    conf.httpd.stack_size =   10 * 1024; 
    conf.httpd.core_id  = 0;
    conf.httpd.open_fn =  http_socket_open_fd;
    conf.httpd.close_fn = http_socket_close_fd;
   
    conf.session_tickets = true;
    conf.port_insecure = 80;
    conf.port_secure =   443;
   // conf.transport_mode  = HTTPD_SSL_TRANSPORT_SECURE;
   conf.transport_mode  = HTTPD_SSL_TRANSPORT_INSECURE;
  
  
  // conf.use_ecdsa_peripheral = false,                
  // conf.ecdsa_key_efuse_blk = 0,                     
  // conf.use_secure_element = false,    
 
 
 // const char *alpn_protos[] =  { "h2", NULL };
  //  conf.alpn_protos = alpn_protos;  
   //const char *alpn_protos[] = { "h2", "http/1.1", NULL };    
//The NULL at the end tells the TLS stack where the list ends.
 // "h2" is the standard ALPN identifier for HTTP/2 protocol.mbeded does not support ALPN (Application-Layer Protocol Negotiation) allows 
 //  "http/1.1" = The older HTTP/1.1 protocol 
   

  if(conf.transport_mode  == HTTPD_SSL_TRANSPORT_SECURE){ 
	 conf.httpd.max_open_sockets =    4 ;
     conf.httpd.backlog_conn      = 3;
     conf.user_cb = https_server_user_callback;
      err = start_http_redirect_server();
     
	vTaskDelay(10 / portTICK_PERIOD_MS); 
	}else{
    conf.httpd.max_open_sockets =    10 ;
    conf.httpd.backlog_conn      = 5;
    err = ESP_OK;
     }
                                                                          
    if(err == ESP_OK && httpd_ssl_start(&server, &conf) == ESP_OK){
   
    if(conf.transport_mode  == HTTPD_SSL_TRANSPORT_SECURE){ 
	 esp_event_handler_register(ESP_HTTPS_SERVER_EVENT, ESP_EVENT_ANY_ID, &http_server_event_handler, NULL);
   }else{ 
	 esp_event_handler_register(ESP_HTTP_SERVER_EVENT, ESP_EVENT_ANY_ID, &http_server_event_handler, NULL);
	 }
    //   wss_keep_alive_set_user_ctx(keep_alive, server);


	 
           #if CONFIG_EXAMPLE_BASIC_AUTH
                httpd_register_basic_auth(server);
          #endif
        
 static const httpd_uri_t cors_handle_SYSTEM_DATE_TIME= {
		   
                            .uri       = "/SYSTEM_DATE_TIME",
                            .method    = HTTP_OPTIONS,
                            .handler   = cors_handle,
                            .user_ctx  = NULL
                            
 };httpd_register_uri_handler(server, &cors_handle_SYSTEM_DATE_TIME);


 static const httpd_uri_t SYSTEM_DATE_TIME_URI = {
		   
                          .uri       = "/SYSTEM_DATE_TIME",
                          .method    = HTTP_POST,
                          .handler   = SYSTEM_DATE_TIME_HANDLER,
                          .user_ctx  = NULL
 };httpd_register_uri_handler(server, &SYSTEM_DATE_TIME_URI);
       /*
          
static const httpd_uri_t cors_handle_REPORT_FILE_NAME_GET = {
		   
                            .uri       = "/REPORT_FILE_NAME_GET",
                            .method    = HTTP_OPTIONS,
                            .handler   = cors_handle,
                            .user_ctx  = NULL
                            
};httpd_register_uri_handler(server, &cors_handle_REPORT_FILE_NAME_GET);

static const httpd_uri_t REPORT_FILE_NAME_URI = {
		   
                          .uri       = "/REPORT_FILE_NAME_GET",
                          .method    = HTTP_GET,
                          .handler   = REPORT_FILE_NAME_DATA_GET_HANDLER,
                          .user_ctx  = NULL
};httpd_register_uri_handler(server, &REPORT_FILE_NAME_URI);

static const httpd_uri_t cors_handle_REPORT_FILE_NAME_DELETE = {
		   
                            .uri       = "/REPORT_FILE_NAME_DELETE",
                            .method    = HTTP_OPTIONS,
                            .handler   = cors_handle,
                            .user_ctx  = NULL
                            
};httpd_register_uri_handler(server, &cors_handle_REPORT_FILE_NAME_DELETE);


static const httpd_uri_t delete_REPORT_FILE_NAME_URI = {
		   
                          .uri       = "/REPORT_FILE_NAME_DELETE",
                          .method    = HTTP_DELETE,
                          .handler   = REPORT_FILE_NAME_DATA_GET_HANDLER,
                          .user_ctx  = NULL
};httpd_register_uri_handler(server, &delete_REPORT_FILE_NAME_URI);

static const httpd_uri_t cors_handle_REPORT_FILE_NAME_PUT = {
		   
                            .uri       = "/REPORT_FILE_NAME_PUT",
                            .method    = HTTP_OPTIONS,
                            .handler   = cors_handle,
                            .user_ctx  = NULL
                            
};httpd_register_uri_handler(server, &cors_handle_REPORT_FILE_NAME_PUT);

static const httpd_uri_t PUT_REPORT_FILE_NAME_URI = {
		   
                          .uri       = "/REPORT_FILE_NAME_PUT",
                          .method    = HTTP_PUT,
                          .handler   = REPORT_FILE_NAME_DATA_GET_HANDLER,
                          .user_ctx  = NULL
};httpd_register_uri_handler(server, &PUT_REPORT_FILE_NAME_URI);


static const httpd_uri_t POST_REPORT_FILE_NAME_URI = {
		   
                          .uri       = "/REPORT_FILE_NAME_POST",
                          .method    = HTTP_POST,
                          .handler   = REPORT_FILE_NAME_DATA_GET_HANDLER,
                          .user_ctx  = NULL
};httpd_register_uri_handler(server, &POST_REPORT_FILE_NAME_URI);
      
static const httpd_uri_t  cors_handle_REPORT_FILE_LOGS = {
		   
                            .uri       = "/REPORT_LOGS",
                            .method    = HTTP_OPTIONS,
                            .handler   = cors_handle,
                            .user_ctx  = NULL
                            
};httpd_register_uri_handler(server, &cors_handle_REPORT_FILE_LOGS);

 static const httpd_uri_t REPORT_FILE_LOGS_URI = {
		   
                          .uri       = "/REPORT_LOGS",
                          .method    = HTTP_GET,
                          .handler   = REPORT_FILE_LOGS_GET_HANDLER,
                          .user_ctx  = NULL
};httpd_register_uri_handler(server, &REPORT_FILE_LOGS_URI);
       
     static const  httpd_uri_t report_download_cors_handle = {
       
         .uri       = "/REPORT_LOG_DOWNLOAD",
         .method    = HTTP_OPTIONS,
         .handler   = cors_handle,
    	 .user_ctx  = NULL

    };httpd_register_uri_handler(server, &report_download_cors_handle);
    	  
    
   static const  httpd_uri_t report_download_uri = {
       
         .uri       = "/REPORT_LOG_DOWNLOAD",
         .method    = HTTP_GET,
         .handler   = REPORT_LOG_DOWNLOAD_HANDLER,
         .user_ctx  = NULL
  
    };httpd_register_uri_handler(server, &report_download_uri);

*/
 static const httpd_uri_t cors_handle_WIFI_CONFIGURATION = {
	 
                       .uri       = "/WIFI_CONFIGURATION",
                       .method    = HTTP_OPTIONS,
                       .handler   = cors_handle,
                       .user_ctx  = NULL
 };httpd_register_uri_handler(server, &cors_handle_WIFI_CONFIGURATION);
 
 static const httpd_uri_t WIFI_CONFIGURATION = {

    	     .uri       = "/WIFI_CONFIGURATION",
    	      .method    = HTTP_POST,
    	      .handler   = WIFI_CONFIGURATION_HANDLER,
    	      .user_ctx  = NULL
    	      
 };httpd_register_uri_handler(server, &WIFI_CONFIGURATION);
      
  static const httpd_uri_t GET_WIFI_CONFIGURATION = {

    	     .uri       = "/WIFI_CONFIGURATION",
    	      .method    = HTTP_GET,
    	      .handler   = WIFI_CONFIGURATION_HANDLER,
    	      .user_ctx  = NULL
    	      
 };httpd_register_uri_handler(server, &GET_WIFI_CONFIGURATION);       
 
  static const httpd_uri_t cors_handle_WIFI_AVAILABLE_NETWORK= {
                       .uri       = "/GET_WIFI_AVAILABLE_NETWORK",
                       .method    = HTTP_OPTIONS,
                       .handler   = cors_handle,
                       .user_ctx  = NULL
 };httpd_register_uri_handler(server, &cors_handle_WIFI_AVAILABLE_NETWORK);
 
 static const httpd_uri_t GET_WIFI_AVAILABLE_NETWORK_URI = {

    	     .uri       = "/GET_WIFI_AVAILABLE_NETWORK",
    	      .method    = HTTP_GET,
    	      .handler   = WIFI_AVAILABLE_NETWORK_HANDLER,
    	      .user_ctx  = NULL
    	      
 };httpd_register_uri_handler(server, &GET_WIFI_AVAILABLE_NETWORK_URI);
               

static const httpd_uri_t cors_handle_MAC_SERIAL_NUMBER = {
		   
                            .uri       = "/GET_MAC_SERIAL_NUMBER",
                            .method    = HTTP_OPTIONS,
                            .handler   = cors_handle,
                            .user_ctx  = NULL
 };httpd_register_uri_handler(server, &cors_handle_MAC_SERIAL_NUMBER);


  static const httpd_uri_t MAC_SERIAL_NUMBER = {
		   
                          .uri       = "/GET_MAC_SERIAL_NUMBER",
                          .method    = HTTP_GET,
                          .handler   = SERIAL_NUMBETR_CONFIGURATION_HANDLER,
                          .user_ctx  = NULL
 };httpd_register_uri_handler(server, &MAC_SERIAL_NUMBER);

 static const httpd_uri_t SERIAL_NUMBETR_CONFIGURATION = {

    	     	        .uri       = "/GET_MAC_SERIAL_NUMBER",
    	     	        .method    = HTTP_POST,
    	     	        .handler   = SERIAL_NUMBETR_CONFIGURATION_HANDLER,
    	     	        .user_ctx  = NULL

};httpd_register_uri_handler(server, &SERIAL_NUMBETR_CONFIGURATION);
    	           
    	           


static const httpd_uri_t SERIAL_NUMBETR_CONFIGURATION_PAGE_cors = {

    	              	  .uri       = "/",
    	              	  .method    = HTTP_OPTIONS,
                          .handler   =cors_handle,
    	              	  .user_ctx  = NULL

 };httpd_register_uri_handler(server, & SERIAL_NUMBETR_CONFIGURATION_PAGE_cors);


static const httpd_uri_t SERIAL_NUMBETR_CONFIGURATION_PAGE = {

    	              	  .uri       = "/",
    	              	  .method    = HTTP_GET,
    	              	  .handler   =  SERVE_SERIAL_NUMBER_WIFI_CRENDENTIALS_CONFIGURATION_PAGE_HANDLER,
    	              	  .user_ctx  = NULL

 };httpd_register_uri_handler(server, & SERIAL_NUMBETR_CONFIGURATION_PAGE);

 static const httpd_uri_t logo_uri_handle = {
	 
    	               .uri = "/SENSEWELL_LOGO.jpg",
    	               .method = HTTP_GET,
    	               .handler = JPG_LOGO_handle,
    	               .user_ctx = NULL
 };httpd_register_uri_handler(server, &logo_uri_handle );
        

if(COMPRESS_GZIP_FILE){   
	
   ESP_LOGW("http_server_task", "COMPRESS_GZIP_FILE");
	
	 static const  httpd_uri_t styles_css = {
             .uri = "/styles.gz",
             .method = HTTP_GET,
             .handler = serve_css,
             .user_ctx = NULL
   };httpd_register_uri_handler(server, &styles_css);


   static const   httpd_uri_t scripts_js = {
             .uri = "/scripts.gz",
             .method = HTTP_GET,
             .handler = serve_js,
             .user_ctx = NULL
   };httpd_register_uri_handler(server, &scripts_js);
	
	
	
}else{
	
 static const  httpd_uri_t styles_css = {
             .uri = "/style.css",
             .method = HTTP_GET,
             .handler = serve_css,
             .user_ctx = NULL
};httpd_register_uri_handler(server, &styles_css);

static const   httpd_uri_t scripts_js = {
             .uri = "/script.js",
             .method = HTTP_GET,
             .handler = serve_js,
             .user_ctx = NULL
 }; httpd_register_uri_handler(server, &scripts_js);
     
  }
  
   static const httpd_uri_t favicon_uri = {
        .uri      = "/favicon.ico",
        .method   = HTTP_GET,
        .handler  = favicon_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &favicon_uri);
    
  
 static const   httpd_uri_t fileicon_png = {
             .uri = "/fileIcon.png",
             .method = HTTP_GET,
             .handler = file_icon_logo_handle,
             .user_ctx = NULL
 }; httpd_register_uri_handler(server, &fileicon_png);//
 
  static const   httpd_uri_t folder_icon_png = {
             .uri = "/folder.png",
             .method = HTTP_GET,
             .handler = folder_icon_logo_handle,
             .user_ctx = NULL
 }; httpd_register_uri_handler(server, &folder_icon_png);///
 
  static const   httpd_uri_t tick_png = {
             .uri = "/tick.png",
             .method = HTTP_GET,
             .handler = TICK_logo_handle,
             .user_ctx = NULL
 }; httpd_register_uri_handler(server, &tick_png);
                      
   static const   httpd_uri_t error_png = {
            .uri = "/error.png",
             .method = HTTP_GET,
             .handler = ERROR_logo_handle,
             .user_ctx = NULL
   }; httpd_register_uri_handler(server, &error_png);
         
 /*
  static const httpd_uri_t cors_1CONFIG_DATA = {
      
        .uri       = "/DEVICE_DATA_LOGGER_CONFIGURATION",
        .method    = HTTP_OPTIONS,
        .handler   =cors_handle,
        .user_ctx  = NULL
 };httpd_register_uri_handler(server,&cors_1CONFIG_DATA);

  static const httpd_uri_t POST_DEVICE_CONFIG_DATA = {
        .uri       = "/DEVICE_DATA_LOGGER_CONFIGURATION",
        .method    = HTTP_POST,
        .handler   =DEVICE_LOGGER_DATA_handler,
        .user_ctx  = NULL
    };httpd_register_uri_handler(server, &POST_DEVICE_CONFIG_DATA);

  static const httpd_uri_t GET_DEVICE_CONFIG_DATA = {
        .uri       = "/DEVICE_DATA_LOGGER_CONFIGURATION",
        .method    = HTTP_GET,
        .handler   =DEVICE_LOGGER_DATA_handler,
        .user_ctx  = NULL
    };httpd_register_uri_handler(server, &GET_DEVICE_CONFIG_DATA);
*/

    static const httpd_uri_t uri_api_system_data = {
    .uri = "/api/system/data",
    .method = HTTP_GET,
    .handler = api_system_data_handler
};

static const httpd_uri_t uri_api_user_add = {
    .uri = "/api/user/add",
    .method = HTTP_POST,
    .handler = api_user_add_handler
};

static const httpd_uri_t uri_api_user_update = {
    .uri = "/api/user/update",
    .method = HTTP_POST,
    .handler = api_user_update_handler
};

static const httpd_uri_t uri_api_user_delete = {
    .uri = "/api/user/delete",
    .method = HTTP_POST,
    .handler = api_user_delete_handler
};

static const httpd_uri_t uri_api_session_start = {
    .uri = "/api/session/start",
    .method = HTTP_POST,
    .handler = api_session_start_handler
};

static const httpd_uri_t uri_api_session_stop = {
    .uri = "/api/session/stop",
    .method = HTTP_POST,
    .handler = api_session_stop_handler
};


static const httpd_uri_t uri_api_fingerprint_enroll = {
    .uri = "/api/fingerprint/enroll",
    .method = HTTP_POST,
    .handler = api_fingerprint_enroll_handler
};

static const httpd_uri_t uri_api_fingerprint_clear_database = {
    .uri = "/api/fingerprint/clear_database",
    .method = HTTP_GET,
    .handler = api_fingerprint_clear_database_handler
};

static const httpd_uri_t uri_api_rfid_enroll = {
    .uri = "/api/rfid/register",
    .method = HTTP_POST,
    .handler = api_rfid_enroll_handler
};

static const httpd_uri_t uri_api_reset_clear_database = {
    .uri = "/api/system/reset",
    .method = HTTP_GET,
    .handler = api_reset_clear_database_handler
};

static const httpd_uri_t uri_api_clear_attendance = {
    .uri = "/api/database/clear-attendance",
    .method = HTTP_GET,
    .handler = api_clear_attendance_handler
};

         httpd_register_uri_handler(server, &uri_api_fingerprint_clear_database);
         httpd_register_uri_handler(server, &uri_api_fingerprint_enroll);
         
         
         httpd_register_uri_handler(server, &uri_api_rfid_enroll);
         
         httpd_register_uri_handler(server, &uri_api_reset_clear_database);
         httpd_register_uri_handler(server, &uri_api_clear_attendance);
    
        httpd_register_uri_handler(server, &uri_api_system_data);
        httpd_register_uri_handler(server, &uri_api_user_add);
        httpd_register_uri_handler(server, &uri_api_user_update);
        httpd_register_uri_handler(server, &uri_api_user_delete);
        httpd_register_uri_handler(server, &uri_api_session_start);
        httpd_register_uri_handler(server, &uri_api_session_stop);
        
    
 }else{
         ESP_LOGE("HTTP SERVER", "Failed to start server");
         server = NULL;
        
        }

    vTaskDelete(NULL);  // Delete the task once server starts
  //  return server;
  
  }




static esp_err_t stop_webserver(httpd_handle_t server){
	
      if( HTTP_server != NULL){
		 err =  httpd_stop(HTTP_server);
		if (err == ESP_OK) {
			 HTTP_server = NULL;  
          }
		}
		
	
  //  wss_keep_alive_stop(httpd_get_global_user_ctx(server));
  esp_event_handler_unregister(ESP_HTTPS_SERVER_EVENT, ESP_EVENT_ANY_ID, &http_server_event_handler);
 
  return httpd_ssl_stop(server);
    
}

void STOP_START_HTTP_SERVER(httpd_handle_t *server, bool STOP_SERVER, bool RESTART_SERVER){
   
    
    if (STOP_SERVER && *server != NULL ){
		
        ESP_LOGW("HTTP SERVER", "HTTP SERVER STOPPing......."); 
		err = stop_webserver(*server); 
        if (err == ESP_OK) {
			 *server = NULL; 
		  ESP_LOGW("HTTP SERVER", "HTTP SERVER STOPPED"); 
         }
        
   
    }

    if(RESTART_SERVER && *server == NULL){
		
        ESP_LOGW("HTTP SERVER", "HTTP SERVER STARTING...");
          vTaskDelay(100 / portTICK_PERIOD_MS);
          if(xTaskCreatePinnedToCore(http_server_task, "http_server_task", 3192, NULL, tskIDLE_PRIORITY + 4, NULL, 0)!= pdPASS) {
          ESP_LOGE("HTTP SERVER ", "Failed to create http_server  task HTTP SERVER  FAILED TO START");

        }else{
            ESP_LOGW("HTTP SERVER", "HTTP SERVER STARTED");
           }
     
    }
    
    
}


void HTTP_disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
 
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server){
		
        ESP_LOGI("HTTP disconnect_handler", "Stopping webserver");
        
        err = stop_webserver(*server); 
        if (err == ESP_OK){
			 *server = NULL;  
			 ESP_LOGW("HTTP disconnect_handler", "HTTP SERVER STOPPED");
         }
     }

}

 void HTTP_connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	
	ESP_LOGW("HTTP connect_handler", "HTTP_connect_handler");
	
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if(*server == NULL){
        
         ESP_LOGW("HTTP connect_handler", "HTTP SERVER STARTING...");
       
          if(xTaskCreatePinnedToCore(http_server_task, "http_server_task", 3192, NULL, tskIDLE_PRIORITY + 4, NULL, 0)!= pdPASS) {
          ESP_LOGE("HTTP connect_handler ", "Failed to create http_server  task HTTP SERVER  FAILED TO START");

        }else{
            ESP_LOGW("HTTP connect_handler", "HTTP SERVER STARTED");
           }
    }
}

  /*
         
  // Set hostname
#define dcn_hostname "DCN"
#define dcn_instance_name "DCN_SERVER"
#define dcn_SERVICE_TYPE "_http"
#define dcn_SERVICE_PROTOCOL "_tcp"
#define dcn_PORT 80

void start_mdns_service(){
	
    if (mdns_initialized){
        mdns_free();  // Free previous instance if running
        mdns_initialized = false;
        ESP_LOGI(TAG, "mDNS service stopped and will be reinitialized.");
      }

    
    err = mdns_init(); // Initialize mDNS
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mDNS: %s", esp_err_to_name(err));
        return;
    }
    
   
    ESP_LOGI(TAG, "mDNS SERVICE INITIALIZED.......");
    
     esp_wifi_get_mode(&WIFI_MODE);

    if(WIFI_MODE == WIFI_MODE_STA){
        esp_netif_t *esp_netif_sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (esp_netif_sta) {
        mdns_register_netif(esp_netif_sta);
        mdns_netif_action(esp_netif_sta, MDNS_EVENT_ENABLE_IP4);
        mdns_netif_action(esp_netif_sta, MDNS_EVENT_ANNOUNCE_IP4);
    }
  }

  if (WIFI_MODE == WIFI_MODE_AP || WIFI_MODE == WIFI_MODE_APSTA) {
    esp_netif_t *esp_netif_ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (esp_netif_ap) {
        mdns_register_netif(esp_netif_ap);
        mdns_netif_action(esp_netif_ap, MDNS_EVENT_ENABLE_IP4);
        mdns_netif_action(esp_netif_ap, MDNS_EVENT_ANNOUNCE_IP4);
    }
  }


        esp_netif_t *esp_netif_sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        mdns_register_netif(esp_netif_sta);
        mdns_netif_action(esp_netif_sta, MDNS_EVENT_ENABLE_IP4);
        mdns_netif_action(esp_netif_sta, MDNS_EVENT_ANNOUNCE_IP4);  
        
      esp_netif_t *esp_netif_ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        mdns_register_netif(esp_netif_sta); // Station
        mdns_register_netif(esp_netif_ap);  // Access Point
        mdns_netif_action(esp_netif_sta, MDNS_EVENT_ENABLE_IP4);
        mdns_netif_action(esp_netif_ap,  MDNS_EVENT_ENABLE_IP4);
        mdns_netif_action(esp_netif_sta, MDNS_EVENT_ANNOUNCE_IP4);
        mdns_netif_action(esp_netif_ap,  MDNS_EVENT_ANNOUNCE_IP4);
      
        
        char *TOKEN;
        TOKEN = strtok(NVS_STORAGE.SERIAL_NUMBER_STRING, "-"); // first part (before dash)
        TOKEN = strtok(NULL, "-");  // second part (after dash)
        if (TOKEN != NULL) {
          //printf("hostname_TOKEN: %s \n",TOKEN);  // Output: 21000101
       }
       
       char dcn_hostname_TOKEN[25];
        char before[10], after[10];
        sscanf(NVS_STORAGE.SERIAL_NUMBER_STRING, "%[^-]-%s", before, after);
        sprintf(dcn_hostname_TOKEN,"DCN-%s",after);
         //printf("dcn_hostname: %s \n",dcn_hostname_TOKEN);
    
    err = mdns_hostname_set(dcn_hostname_TOKEN);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Failed to set mDNS hostname: %s", esp_err_to_name(err));
        mdns_free();
        mdns_initialized = false;
        return;
    }
    
    err = mdns_instance_name_set(dcn_instance_name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set mDNS instance name: %s", esp_err_to_name(err));
        mdns_free();
        mdns_initialized = false;
        return;
    }
   
 
    mdns_txt_item_t serviceTxtData[5] ={
        
        {"DCN_NAME", NVS_STORAGE.DEVICE_NAME_NVS_STR},
        {"DCN_SERIAL_ID", NVS_STORAGE.SERIAL_NUMBER_STRING},
        {"DCN_MAC_ADDR", NVS_STORAGE.ESP32_MAC_ADDRESS_STR},
        {"DCN_APP_VER",NVS_STORAGE.DCN_APP_VERSION},
        {"DCN_HAR_VER",NVS_STORAGE.DCN_HAR_VERSION},
    };


    err = mdns_service_add(dcn_instance_name, dcn_SERVICE_TYPE, dcn_SERVICE_PROTOCOL, dcn_PORT, serviceTxtData, 5);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Failed to add mDNS service: %s", esp_err_to_name(err));
        mdns_free();
        mdns_initialized = false;
        return;
    }
    
    
     mdns_initialized = true;
     ESP_LOGI(TAG, "mDNS service added successfully.");
   
    
}


static void query_mdns_service(const char *service_TYPE, const char *proto) {
   
//static const char * if_str[] = {"STA", "AP", "ETH", "MAX"};
static const char * ip_protocol_str[] = {"V4", "V6", "MAX"};
   

    if (!mdns_initialized){
        ESP_LOGI(MDNS_TAG, "Initializing mDNS...");
        err = mdns_init();
        if (err != ESP_OK) {
            ESP_LOGE(MDNS_TAG, "mDNS initialization failed: %s", esp_err_to_name(err));
            return;
        }
        mdns_initialized = true;
    }
    
    
    ESP_LOGI(MDNS_TAG, "QUERYING HOSTNAME:%s.local",SBC_HOST_NAME);
    ESP_LOGI(MDNS_TAG, " MDNS QUERYING  INSTANCE NAME:%s Service Type:%s || Protocol:%s.local",SBC_INSTANCE_NAME ,service_TYPE, proto);

    mdns_result_t *results = NULL;
    err =  mdns_query(SBC_INSTANCE_NAME, service_TYPE, proto, MDNS_TYPE_PTR, 2000, 20, &results);
    if (err == ESP_ERR_INVALID_STATE || err != ESP_OK ){
		  err = mdns_query_ptr(service_TYPE, proto, 2000, 20, &results);
          if (err != ESP_OK ){
               ESP_LOGE(MDNS_TAG, "MDNS query pointer failed: %s", esp_err_to_name(err));
              //mdns_free();
              //mdns_initialized = false;  // Mark as not initialized for next call
              MDNS_SEARCH_FLAG = false;
              esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, FIND_SBC, NULL, 0, portMAX_DELAY);
              return;
             }
    }
    
    
    if(!results){
        ESP_LOGI(MDNS_TAG, "NO MDNS SEARCH RESULT FOUND");
        MDNS_SEARCH_FLAG = false;
        esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, FIND_SBC, NULL, 0, portMAX_DELAY);
        return;
    }
  
    mdns_result_t *r = results;
    while (r != NULL){
		
		if(PRINT_DEBUG){
			
         ESP_LOGI(MDNS_TAG, "---- mDNS RESULT ----");
         ESP_LOGI(MDNS_TAG, "Interface:  Type: %s\n", ip_protocol_str[r->ip_protocol]);
         ESP_LOGI(MDNS_TAG, "Hostname       : %s", r->hostname ? r->hostname : "N/A");
         ESP_LOGI(MDNS_TAG, "Instance Name  : %s", r->instance_name ? r->instance_name : "N/A");
         ESP_LOGI(MDNS_TAG, "Service Type   : %s", r->service_type ? r->service_type : "N/A");
         ESP_LOGI(MDNS_TAG, "Protocol       : %s", r->proto ? r->proto : "N/A");
         ESP_LOGI(MDNS_TAG, "Port           : %u", r->port);
         ESP_LOGI(MDNS_TAG, "TIME TO LIVE :   %lu", r->ttl);
        // ESP_LOGI(MDNS_TAG, "ESP NET_IF :   %d", r->esp_netif); 
        
        }
         
      // ESP_LOGI(MDNS_TAG, "ESP NET_IF :   %d", r->addr); 
        mdns_ip_addr_t *a = r->addr;
          if (!a) {
              ESP_LOGI(MDNS_TAG, "MDNS No IP address found in results!");
              mdns_query_results_free(results);
              esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, FIND_SBC, NULL, 0, portMAX_DELAY);
               return;
             }
             
        while (a){
            if (a->addr.type == ESP_IPADDR_TYPE_V6){
                ESP_LOGI(MDNS_TAG, "IPv6 Address   : " IPV6STR, IPV62STR(a->addr.u_addr.ip6));
            } else {
                ESP_LOGI(MDNS_TAG, "IPv4 Address   : " IPSTR, IP2STR(&(a->addr.u_addr.ip4)));
               
                snprintf(SBC_IP,IP_ADDR_STR_MAX_LEN, IPSTR, IP2STR(&(a->addr.u_addr.ip4)));
            }
            a = a->next;
         }

        // TXT records
        if (r->txt_count > 0) {
            ESP_LOGI(MDNS_TAG, "TXT Records    : %d", r->txt_count);
            for (int i = 0; i < r->txt_count; i++) {
				ESP_LOGI(MDNS_TAG, "%s=%s", r->txt[i].key, r->txt[i].value);
            }
        } else {
            ESP_LOGI(MDNS_TAG, "No TXT records.");
        }

        r = r->next;
    }
    
    
    MDNS_SEARCH_FLAG = true;
    if(results){
    mdns_query_results_free(results);
    }
    
    if(r){
    mdns_query_results_free(r);
    }
    
    if(strlen(SBC_IP) > 12 ){
    esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, SEND_ESP_DATA, NULL, 0, portMAX_DELAY);
     }else{
    esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, FIND_SBC, NULL, 0, portMAX_DELAY);        
	 }




}*/


void SEND_DCN_INFO_SBC(const char SBC_IP[16]){
	
	
    #define HTTP_CLIENT "SEND_DCN_INFO_SBC"

     if (strlen(SBC_IP) < 12){
        ESP_LOGE(HTTP_CLIENT, "SBC_IP is NOT PRESENT! FAILED TO TO SEND HTTP REQUEST");
        esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, FIND_SBC, NULL, 0, portMAX_DELAY);
    }
    
          sprintf(Final_IP,"%s%s%s",SBC_IP_HEAD,SBC_IP,SBC_IP_TAIL);
         ESP_LOGW(EVENT_GROUP_TAG,"SENDING DCN INFORM TO SBC: %s",Final_IP);
       

    cJSON *root = DCN_INFORMATION_GET_HANDLER();
    if (!root){
        ESP_LOGE(HTTP_CLIENT, "Failed to create JSON");
        esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, SEND_ESP_DATA, NULL, 0, portMAX_DELAY);
    }

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);  // Now safe to delete root
    if (!payload){
        ESP_LOGE(HTTP_CLIENT, "Failed to print JSON");
        esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, SEND_ESP_DATA, NULL, 0, portMAX_DELAY);
    }

     //printf("ES32_INFORMATION_DATA:  %s\n", payload);

     HTTP_CLIENT_RESPOND_DATA_FLAG =  true;
    uint16_t  status_code = PERFORM_HTTP_REQUEST(1, Final_IP, HTTP_METHOD_POST, payload, "application/json", NULL, 0);
    if(status_code == 200){
         ESP_LOGI(HTTP_CLIENT,"DCN INFORMATION SUCCESSFULL SEND TO SBC");
         esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, CONNECT_MQTT, NULL, 0, portMAX_DELAY);
   
    }else{
		
        ESP_LOGE(HTTP_CLIENT, "DCN_SBC_INFORMATION POST DATA FAIL, status code: %d", status_code);
        esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, SEND_ESP_DATA, NULL, 0, portMAX_DELAY);
    }

   if(payload){
    free(payload);
    payload = NULL;
    }
    
}


#define MQTT_TAG "DCN_MQTT_EVENT"
void subscribe_mqtt_topic(const char *topic, int qos){
	
	
    if (client == NULL) {
        ESP_LOGE("MQTT", "MQTT client is NULL! Cannot subscribe.");
        return;
    }

    if (topic == NULL) {
        ESP_LOGE("MQTT", "Invalid topic provided for subscription!");
        return;
    }
      MQTT_MESSAGE_ID = esp_mqtt_client_subscribe_single(client, topic, qos);
  //  int MQTT_MESSAGE_ID = esp_mqtt_client_subscribe(client, topic, qos);
       if (MQTT_MESSAGE_ID > 0) {
			 
          ESP_LOGI("MQTT", "SUBCRIBE TOPIC : %s, MSG_ID =%d", topic, MQTT_MESSAGE_ID);
       
   
         }else if(MQTT_MESSAGE_ID == -1) {
			 
           ESP_LOGI(MQTT_TAG, "SUBCRIBE TOPIC FAILED|| MQTT_SKIP_PUBLISH_IF_DISCONNECTED  ");
         }else if(MQTT_MESSAGE_ID == -2) {
			 
           ESP_LOGI(MQTT_TAG, "SUBCRIBE TOPIC FAILED  full outbox");
         }else {
	
          ESP_LOGI("MQTT", "Sent publish successful, MQTT_MESSAGE_ID=%d", MQTT_MESSAGE_ID); 
        
              }
         
  
  
    if (MQTT_MESSAGE_ID < 0) {
		
        ESP_LOGE("MQTT", "MQTT subscription failed!");
    } else {
		
        
        
    }
    
}

static void mqtt_error_handler(esp_mqtt_event_handle_t event){
	
	   
	   
	    if (!event->error_handle) {
        ESP_LOGE(MQTT_TAG, "MQTT Error: No error handle available.");
        return;
    }
	   

  
         ESP_LOGE(MQTT_TAG, " \n \n LAST ERROR MQTT (%s)", strerror(event->error_handle->esp_transport_sock_errno));
      
         esp_mqtt_error_codes_t *err_handle = event->error_handle;
       
         switch (err_handle->error_type){
		   
        case MQTT_ERROR_TYPE_TCP_TRANSPORT:
        
             ESP_LOGW(MQTT_TAG, "MQTT_ERROR_TYPE_TCP_TRANSPORT : Internal error reported from MQTT broker on connection .");
        
            ESP_LOGE(MQTT_TAG, "MQTT Error: Error reported from TCP Transport Issue /esp-tls  .");
            ESP_LOGE(MQTT_TAG, "TLS Error: 0x%x", err_handle->esp_tls_last_esp_err);
            ESP_LOGE(MQTT_TAG, "TLS Stack Error: 0x%x", err_handle->esp_tls_stack_err);
            ESP_LOGE(MQTT_TAG, "TLS Cert Verify Flags: 0x%x", err_handle->esp_tls_cert_verify_flags);
            ESP_LOGE(MQTT_TAG, "Socket Errno: %d", err_handle->esp_transport_sock_errno);
            break;

        case MQTT_ERROR_TYPE_CONNECTION_REFUSED:
        
            ESP_LOGW(MQTT_TAG, "MQTT_ERROR_TYPE_CONNECTION_REFUSED : Internal error reported from MQTT broker on connection .");
            
            switch (err_handle->connect_return_code) {
				
                case MQTT_CONNECTION_REFUSE_PROTOCOL:
                    ESP_LOGE(MQTT_TAG, "  Reason: Wrong Protocol.");
                    break;
                case MQTT_CONNECTION_REFUSE_ID_REJECTED:
                    ESP_LOGE(MQTT_TAG, " Reason: ID Rejected.");
                    break;
                case MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE:
                    ESP_LOGE(MQTT_TAG, " Reason: Server Unavailable.");
                    break;
                case MQTT_CONNECTION_REFUSE_BAD_USERNAME:
                    ESP_LOGE(MQTT_TAG, "-Reason: Incorrect Username.");
                    break;
                case MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED:
                    ESP_LOGE(MQTT_TAG, " Reason: Not Authorized (Invalid Credentials).");
                    break;
                default:
                    ESP_LOGE(MQTT_TAG, " Unknown Connection Refusal Code: %d", err_handle->connect_return_code);
                    break;
            }
            break;
            
            case MQTT_ERROR_TYPE_SUBSCRIBE_FAILED:
            
                ESP_LOGE(MQTT_TAG, "MQTT_ERROR_TYPE_SUBSCRIBE_FAILED");
                
                break;
            default:
                ESP_LOGE(MQTT_TAG, "OTHER ERROR  type: %d", event->error_handle->error_type);
                
                break;
       }

       
   
}

void MQTT_SUBSCRIBE_TOPIC( const char *topic, const char *PRE_TOPIC,uint8_t QOS ){
	
	 char mqtt_topic[70];
     snprintf(mqtt_topic, sizeof(mqtt_topic),"%s/%s/%s",PRE_TOPIC,topic ,NVS_STORAGE.ESP32_MAC_ADDRESS_STR);
	
	 subscribe_mqtt_topic( mqtt_topic,QOS);
        
	
}
char mqtt_topic_sv_profile_set_key[55];
 char mqtt_topic_start_stop_button_set_key[55];
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
	
	 
     snprintf(mqtt_topic_sv_profile_set_key, sizeof(mqtt_topic_sv_profile_set_key),"sbc/sv_profile_set/%s", NVS_STORAGE.ESP32_MAC_ADDRESS_STR);	
     snprintf(mqtt_topic_start_stop_button_set_key, sizeof(mqtt_topic_start_stop_button_set_key),"sbc/sv_start_condition_set/%s", NVS_STORAGE.ESP32_MAC_ADDRESS_STR);
	
	//esp_mqtt_client_handle_t client = event->client;		
	
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
		
		
     case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGW(MQTT_TAG, "MQTT_EVENT_BEFORE_CONNECT: MQTT TRYING connecting.......");
            MQTT_CONNECTED_FLAG  = false;
            break;		
		
    case MQTT_EVENT_CONNECTED:
        MQTT_CONNECTED_FLAG =true;
         ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
         reconnect_retry = 0;
         
       
       ESP_LOGI(MQTT_TAG, "Session Present: %d", event->session_present);
       if (event->session_present) {
                ESP_LOGI(MQTT_TAG, "The broker remembers previous subscriptions. Subscriptions remain active.");
            } else {
                ESP_LOGI(MQTT_TAG, "No previous session found. A new session has been started. Subscriptions need to be re-subscribed.");
            }
            
        ESP_LOGI(MQTT_TAG, "MQTT Protocol Version: %d", event->protocol_ver);
        
        subscribe_mqtt_topic( "topic/sbc-alive",0); // NO MAC ADDRESS 
        
       
        MQTT_SUBSCRIBE_TOPIC("sv_profile_set","sbc",1 );
        MQTT_SUBSCRIBE_TOPIC("sv_start_condition_set","sbc",1 );
       
         break;
    
    case MQTT_EVENT_DISCONNECTED:
        
         MQTT_CONNECTED_FLAG  = false;
        ESP_LOGE(MQTT_TAG, "MQTT_EVENT_DISCONNECTED...........");
     
   

	if(reconnect_retry >= 5){
		
		MQTT_CONNECTED_FLAG  = false;
		esp_event_post_to(main_event_handle,MAIN_EVENT_BASE,FIND_SBC,NULL,0,portMAX_DELAY);
		 
		 
	 }else if(esp_wifi_sta_get_ap_info(&ap_info)== ESP_OK  && event->client != NULL){
     
             ESP_LOGE(MQTT_TAG, "Attempting to reconnect  (Attempt %d/%d)...", reconnect_retry + 1, MAX_RECONNECT_ATTEMPTS);
             vTaskDelay(400 / portTICK_PERIOD_MS);
             esp_err_t err = esp_mqtt_client_reconnect(client);
             if(err == ESP_OK){
                    ESP_LOGW(MQTT_TAG, "DISCONNECTED Reconnection MQTT............");
             }else{
					ESP_LOGE(MQTT_TAG, "Failed to Reconnection MQTT Error: %d", err);
                      }
                 
                 reconnect_retry++;
      }else{
		  
		   ESP_LOGE(MQTT_TAG, "FAIL TO RECONNECT WIFI DISCONNECT OR MQTT CLIENT IS NULL  %d/%d)...", reconnect_retry + 1, MAX_RECONNECT_ATTEMPTS);
		  
	       }
         
         
        break;
        
    case MQTT_EVENT_PUBLISHED:
      
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED:DATA PUBLISH SUCCESSFULLY, message ID: %d", event->msg_id);
        
    break;
        
        
    case MQTT_EVENT_DATA:
    
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
        ESP_LOGI("MQTT", "Message received on topic: %.*s ", event->topic_len, event->topic);
        ESP_LOGI(MQTT_TAG, " - QoS: %d, Retain: %d, Dup: %d", event->qos, event->retain, event->dup);
        if (event->dup) {
                ESP_LOGW(MQTT_TAG, "WARNING: This is a duplicate message! Message was resent due to a missing acknowledgment. (QoS 1 or 2 message re-delivered due to lost acknowledgment)");
            } else {
                ESP_LOGI(MQTT_TAG, "First time receiving this message");
            }
        ESP_LOGI(MQTT_TAG, "Total Data Length: %d", event->total_data_len);
        ESP_LOGI(MQTT_TAG, "Current Data Offset: %d", event->current_data_offset);   
        
        ESP_LOGI("MQTT", "Message: %.*s", event->data_len, event->data);
               
                
			    if (strncmp(event->topic, "topic/sbc-alive", event->topic_len) == 0){ 
					
					ESP_LOGE("MQTT", "Re-send mDNS DCN information REQUEST BY SBC==========================================");
                
                if (strncmp(event->data,"hello", event->data_len) == 0){ // Fix payload comparison
                    ESP_LOGE("MQTT", "SBC SAY HELLO :");
                    esp_event_post_to(main_event_handle, MAIN_EVENT_BASE, SEND_ESP_DATA, NULL, 0, portMAX_DELAY);
                  }
                
                }
			    
        break;
        
        case  MQTT_EVENT_DELETED:
  
         ESP_LOGE(MQTT_TAG, "Message dropped from outbox : %d (maybe due to retries or client stop )", esp_mqtt_client_get_outbox_size(client));
         break;
         
       case MQTT_EVENT_ERROR:
        
         ESP_LOGE(MQTT_TAG, "MQTT_EVENT_ERROR..........");
         mqtt_error_handler(event);
         break;
         
        default:
        ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
        break;
    }
    
}

#define MQTT_BUFFER_SIZE  15 * 1024
bool START_MQTT_PUBLISH_mutex  = false;
SemaphoreHandle_t MQTT_PUBLISH_mutex = NULL;


void INITAILZE_MQTT(){
	
	  if (client != NULL) {
        ESP_LOGW("MQTT", "MQTT client already initialized. Reinitializing...");
     
         err = esp_mqtt_client_stop(client);
        if (err != ESP_OK) {
            ESP_LOGE("MQTT", "Failed to stop MQTT client, error code: %d", err);
            // return;
         }
     
        err = esp_mqtt_client_destroy(client);
        if (err == ESP_OK) {
			  client = NULL; 
        }else{
			
			 ESP_LOGE("MQTT", "Failed to destroy MQTT client, error code: %d", err);
		     return;
		    }

      
    }
    
    snprintf(BROKER_URL, sizeof(BROKER_URL), "%s%s","mqtt://", SBC_IP);
    ESP_LOGI(MQTT_TAG, "Broker Url is : %s", BROKER_URL);


// Define MQTT configuration
/*  esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = NULL,                 // Default: No broker URI set
    .broker.address.port = 0,                   // Default: 0 (use protocol default)
    .broker.verification.certificate = NULL,    // Default: No TLS certificate
    .credentials.client_id = NULL,              // Default: No client ID (ESP-IDF auto-generates one)
    .credentials.username = NULL,               // Default: No username
    .credentials.authentication.password = NULL, // Default: No password
    .credentials.authentication.certificate = NULL, // Default: No client certificate
    .credentials.authentication.key = NULL,    // Default: No client private key
    //.session.protocol_ver = MQTT_PROTOCOL_V_311, // Default: MQTT 3.1.1   // MQTT_PROTOCOL_V_3_1 // MQTT_PROTOCOL_V_3_1_1 // MQTT_PROTOCOL_V_5
    .session.keepalive = 120,                   // Default: 120 seconds
    .session.disable_clean_session = false,     // Default: Clean session enabled
    .session.message_retransmit_timeout = 0,    // Default: 0 (use protocol default)
    .session.last_will.topic = NULL,            // Default: No Last Will topic
    .session.last_will.msg = NULL,              // Default: No Last Will message
    .session.last_will.msg_len = 0,             // Default: No Last Will message length
    .session.last_will.qos = 0,                 // Default: QoS 0
    .session.last_will.retain = false,          // Default: Do not retain Last Will
   
    .network.timeout_ms = 10000,                // Default: 10 seconds timeout
    .network.disable_auto_reconnect = false,    // Default: Auto-reconnect enabled
    .network.reconnect_timeout_ms = 10000,      // Default: 10 seconds reconnect timeout
   // .network.reconnect_timeout_backoff = 0,     // Default: 0 (no backoff)
   // .network.alpn_protos = NULL,                // Default: No ALPN
   // .network.use_global_ca_store = false,       // Default: Do not use global CA store
  
   // .network.reconnect_cb = NULL,               // Default: No custom reconnect callback
   // .network.connect_cb = NULL,                 // Default: No custom connect callback
   // .network.disconnect_cb = NULL,              // Default: No custom disconnect callback
   
    .buffer.size = 1024,                        // Default: 1024 bytes
    .buffer.out_size = 512,                     // Default: 512 bytes output buffer
    .task.priority = 5,                         // Default: Task priority 5
    .task.stack_size = 6144,                    // Default: Stack size 6144 bytes
  //.task.core = 0,                             // Default: Runs on core 0
 // .task.pinned = false,                       // Default: Not pinned to any core
};
*/


        esp_mqtt_client_config_t mqtt_cfg = {
			
         .broker.address.uri = BROKER_URL,
          //.broker.address.hostname,
         //.broker.address.port= 
        //.broker.address.path  = 
       //.credentials.client_id = NULL,  
      
         .broker.address.transport =  MQTT_TRANSPORT_OVER_TCP,
         .credentials.username = NULL,            
         .credentials.authentication.password = NULL, 
         
         .credentials.authentication.certificate = (const char *) servercert_start, 
         .credentials.authentication.certificate_len = servercert_end - servercert_start,
       
         .credentials.authentication.key =  (const char *)prvtkey_pem_start,    
         .credentials.authentication.key_len = prvtkey_pem_end - prvtkey_pem_start,
         
         .broker.verification.certificate =(const char *)sbc_cert_start,
         .broker.verification.certificate_len = sbc_cert_end- sbc_cert_start,
     
 
         .broker.verification.skip_cert_common_name_check = true,
          //.broker.verification.common_name
        //  .broker.verification.use_global_ca_store
        //  .broker.verification.crt_bundle_attach
     

        .network.disable_auto_reconnect = false,
        .network.refresh_connection_after_ms = 80000,
        .network.reconnect_timeout_ms= 8000,
        .network.timeout_ms = 20000, 
      //.network.transport
     
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .session.disable_keepalive = false,
        .session.keepalive = 800,
        .session.disable_clean_session = false,
        .session.message_retransmit_timeout = 120,
 
        .buffer.out_size = MQTT_BUFFER_SIZE  - 1024 ,  // Max size of a single outgoing MQTT message
        .buffer.size=  MQTT_BUFFER_SIZE , // Total buffer for send + receive
        .task.priority = tskIDLE_PRIORITY + 4,
        .task.stack_size = 5 *1024, 
        .outbox.limit = MQTT_BUFFER_SIZE , //8192, // Total memory the outbox is allowed to use
      //  .task.core = 0,
       // .task.pinned = false,   
    };
    
    
   //esp_mqtt_set_config(client, &mqtt_cfg);
    // Initialize MQTT client
    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL){
        ESP_LOGE("MQTT", "MQTT client initialization failed!");
        MQTT_INNITIALIZED_FLAG = false;
        return;
    }

    // Register event before starting the client
    esp_err_t err = esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK){
        ESP_LOGE("MQTT", "Failed to register event handler, error code: %d", err);
        MQTT_INNITIALIZED_FLAG = false;
        if(client != NULL){
        esp_mqtt_client_destroy(client);
        client = NULL;
        }
        return;
    }
    

  
    MQTT_INNITIALIZED_FLAG = true;
    err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
		MQTT_INNITIALIZED_FLAG = false;
        ESP_LOGE("MQTT", "Failed to start MQTT client, error code: %d", err);
        if (client != NULL) {
        esp_mqtt_client_destroy(client);
        client = NULL;
        }
        return;
    }
    
    MQTT_PUBLISH_mutex = xSemaphoreCreateMutex();
    if(MQTT_PUBLISH_mutex != NULL){
		   START_MQTT_PUBLISH_mutex = true;
      }else{ESP_LOGE("MQTT", "MQTT_PUBLISH_mutex Mutex creation failed");}
    
       ESP_LOGI("MQTT", "MQTT client reinitialized successfully");
    
}

void MQTT_RESPONSE_SEND(const char* response_data){
	
	 if(MQTT_CONNECTED_FLAG){
	
       sprintf( MQTT_TOPIC_KEY,"dcn-response/%s",NVS_STORAGE.ESP32_MAC_ADDRESS_STR);                  
        MQTT_MESSAGE_ID = PUBLISH_MQTT_DATA( MQTT_TOPIC_KEY,response_data,1);
       if(MQTT_MESSAGE_ID){
										
		 //printf("MQTT CONNECTED and RESPOND DATA send \n");
        }else{
											  
         //printf("MQTT NOT CONNECTED RESPOND DATA NOT send \n");
        }

}

}

uint16_t PUBLISH_MQTT_DATA(const char *topic, const char *data,uint8_t QOS){
	
	
	bool GOT_MUTEX = false;

	 if(START_MQTT_PUBLISH_mutex){
	  if (xSemaphoreTake(MQTT_PUBLISH_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW("MQTT_PUBLISED", "Could not take mutex, function skipped");
        return false;
    } else {
        GOT_MUTEX = true;  // Remember you have the mutex now
    }
  }
  
  
    if (MQTT_INNITIALIZED_FLAG != true || client == NULL){
        ESP_LOGE("MQTT_PUBLISED", "MQTT client is NULL! Cannot publish data or MQTT NOT INNITIALIZED_FLAG ");
         if(START_MQTT_PUBLISH_mutex && GOT_MUTEX) {xSemaphoreGive(MQTT_PUBLISH_mutex);}
        return 0;  // Return 0 to indicate failure
     }
    
   if ( topic == NULL || data == NULL  || strlen(data) > MQTT_BUFFER_SIZE){
       ESP_LOGE("MQTT_PUBLISED", "Invalid topic or data provided for publishing mqtt Message size exceeds buffer: %d bytes ",strlen(data));
        if(START_MQTT_PUBLISH_mutex && GOT_MUTEX) {xSemaphoreGive(MQTT_PUBLISH_mutex);}
       return 0;
   }
    
    
   
   
   if (MQTT_CONNECTED_FLAG){
	   
	   if(SYSTEM_DEBUG){	   
	   // ESP_LOGI("MQTT_PUBLISED", "BEFORE PUBLISH OUTBOX SIZE :%d ", esp_mqtt_client_get_outbox_size(client));
        }
        
       int retry_count = 0;
        #define MAX_RETRY 3

        RE_PUBLISH_MESSSAGE:

      if (++retry_count > MAX_RETRY) {
           ESP_LOGE("MQTT_PUBLISED", "Retry limit reached. MQTT publish failed.");
 
          if(START_MQTT_PUBLISH_mutex && GOT_MUTEX) {xSemaphoreGive(MQTT_PUBLISH_mutex);}
          return 0;
          }
          
    
     
      MQTT_MESSAGE_ID =  esp_mqtt_client_enqueue(client, topic, data,0, QOS, 0, true);
    //  MQTT_MESSAGE_ID = esp_mqtt_client_publish(client, topic, data_copy, 0, 1, 0);
   
       if(QOS !=0 && MQTT_MESSAGE_ID == 0){
			 
           ESP_LOGW("MQTT_PUBLISED", "MQTT MESSAGE NOT RECIEVE || QOS is %d || MQTT_MESSAGE_ID=%d",QOS, MQTT_MESSAGE_ID );
       
         }else if(MQTT_MESSAGE_ID == -1){
			 
           ESP_LOGE("MQTT_PUBLISED", "PUBLISH MQTT failed || MQTT_SKIP_PUBLISH_IF_DISCONNECTED");
       
         }else if(MQTT_MESSAGE_ID == -2){
			 
             ESP_LOGE("MQTT_PUBLISED", "failed  Outbox too full Skipping publish . :: %d",esp_mqtt_client_get_outbox_size(client));
             esp_err_t err = esp_mqtt_client_stop(client);
             if(err == ESP_OK){
				vTaskDelay(50 / portTICK_PERIOD_MS);
                esp_mqtt_client_start(client);
             }
		   if( QOS >0){ ESP_LOGW("MQTT_PUBLISED","RE_PUBLISH_MESSSAGE"); vTaskDelay(200 / portTICK_PERIOD_MS);   goto RE_PUBLISH_MESSSAGE; }
         
          }else{
			  
			if(MQTT_DEBUG){    
		    ESP_LOGI("MQTT_PUBLISED","TOPIC :%s || MQTT_MESSAGE_ID=%d ", topic, MQTT_MESSAGE_ID);
		    }
	       //  ESP_LOGI("MQTT_PUBLISED", "%s", data); 
               }
         
         
        }
     
     
    if(SYSTEM_DEBUG){    
  //  ESP_LOGI("MQTT_PUBLISED", "AFTER PUBLISH OUTBOX SIZE : %d ", esp_mqtt_client_get_outbox_size(client));
    }
    
   
     
     if(START_MQTT_PUBLISH_mutex && GOT_MUTEX) {xSemaphoreGive(MQTT_PUBLISH_mutex);}
    return MQTT_MESSAGE_ID;
    
}




uint8_t ESP_LAST_RESET_REASON(){
	
    esp_reset_reason_t reason = esp_reset_reason();
     printf("\nReset Reason Code: %d\n", reason);

    const char *reason_desc = "Unknown reset reason";  // Default description

     switch (reason) {
		 
        case ESP_RST_POWERON:
            reason_desc = "Power-on reset"; 
            strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"POWER RST",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_SW:
            reason_desc = "Software reset via esp_restart()";
            strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"SOFT RST",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_PANIC:
            reason_desc = "Exception/panic reset (WDT timeout, crash)";
             strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"PANIC",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_TASK_WDT:
            reason_desc = "Task Watchdog Timer (TWDT) timeout";
            strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"TASK WDT",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_INT_WDT:
            reason_desc = "Interrupt Watchdog Timer (IWDT) timeout";
            strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"INTR WDT",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_BROWNOUT:
            reason_desc = "Brownout detected (low voltage)";
            strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"BRWNOUT RST",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_SDIO:
            reason_desc = "Reset via SDIO";
             strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"RST SDIO",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_DEEPSLEEP:
            reason_desc = "Wake-up from deep sleep";
            strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"DEEP SLEEP",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        case ESP_RST_EXT:
            reason_desc = "External reset (RESET button pressed)";
             strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"EXT POWER",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
            break;
        default:
           strncpy(DEVICE_LOGGER.DCN_RESET_REASON_STR,"UNKNOWN",sizeof(DEVICE_LOGGER.DCN_RESET_REASON_STR));
           break;
      }

        printf("Reset Reason: %s[%s]\n", reason_desc,DEVICE_LOGGER.DCN_RESET_REASON_STR);
     
   // snprintf(reset_report_path, sizeof(reset_report_path), "%s/%sRESET_REPORT.csv",MOUNT_POINT, DEVICE_LOGGER.DCN_FOLDER_NAME);
  // uint16_t CSV_LEN = snprintf(reset_reason_str, sizeof(reset_reason_str), "%s/%s  || CODE_REASON: %d || %s \n",DATE_buffer,TIME_buffer, reason, reason_desc);
  //  WRITE_DATA_IN_FILE(reset_report_path, reset_reason_str,CSV_LEN,1000);
   return  reason;
}




void watchdog_task(void *pvParameters){

esp_err_t err = esp_task_wdt_add(NULL);
if (err == ESP_ERR_INVALID_STATE){
    ESP_LOGW("WDT_DOG_TASK", "Task already subscribed to watchdog");
} else if (err != ESP_OK) {
    ESP_LOGE("WDT_DOG_TASK", "Failed to add task to watchdog: %s", esp_err_to_name(err));
} 

/*or esp_task_wdt_delete(NULL);  // remove first (ignores if not added)
    esp_task_wdt_add(NULL);     // add again*/
 
    while (1){
		
        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 10 seconds
        //ESP_LOGW("WDT_DOG_TASK", "Watchdog: TIMER RESET TASK!\n");
        esp_task_wdt_reset(); 
         
      if(WIFI_MODE == WIFI_MODE_STA){
		  
       err = esp_wifi_sta_get_ap_info(&ap_info);
       if(err == ESP_OK){
          err = esp_wifi_sta_get_rssi(&WIFI_CREDENTIAL.RSSI_DBI);   
          if(err == ESP_OK){
			 
             WIFI_CREDENTIAL.RSSI_PER  = rssi_to_percent(WIFI_CREDENTIAL.RSSI_DBI);
             strncpy(WIFI_CREDENTIAL.RSSI_EVA_STR, evaluate_rssi(WIFI_CREDENTIAL.RSSI_DBI),sizeof(WIFI_CREDENTIAL.RSSI_EVA_STR));
           //  ESP_LOGI("WDT_DOG_TASK", "RSSI: %d dBm || %d%% || %s: ", WIFI_CREDENTIAL.RSSI_DBI,   WIFI_CREDENTIAL.RSSI_PER , WIFI_CREDENTIAL.RSSI_EVA_STR);
        
        }else{
			
            ESP_LOGW("WDT_DOG_TASK","Failed to get RSSI: %s", esp_err_to_name(err));
             
             }
       
       }else{
        ESP_LOGW("WDT_DOG_TASK","Not connected to any AP: %s", esp_err_to_name(err));
       }
    }
    
    
     /* if(PIDProfileTaskHandle){
		  
	    if (eTaskGetState(PIDProfileTaskHandle) == eSuspended){
			 ESP_LOGW("WDT_DOG_TASK", "Resuming MODBUS task");
             vTaskResume(PIDProfileTaskHandle);
          }
           
        }
           
       if(MODBUS_TASK_HANDLE){
		   
        if (eTaskGetState(MODBUS_TASK_HANDLE) == eSuspended){
            ESP_LOGW("WDT_DOG_TASK", "Resuming MODBUS task");
            vTaskResume(MODBUS_TASK_HANDLE);
             
               }
         }
         
        if(MQTT_TASK_HANDLE){
       
        if(eTaskGetState(MQTT_TASK_HANDLE) == eSuspended){
            ESP_LOGW("WDT_DOG_TASK", "Resuming MODBUS task");
            vTaskResume(MQTT_TASK_HANDLE);
             
              }
          }*/
    
    }///WHILE 
}





void print_nvs_info(){
	
if(SYSTEM_DEBUG){
	
    nvs_stats_t stats;
    esp_err_t err;

    // Get default NVS partition stats
    err = nvs_get_stats(NULL, &stats);  // NULL = default partition
    if (err == ESP_OK){
        ESP_LOGI("NVS_INFORMATION", "  NVS Stats (Default Partition):");
        ESP_LOGI("NVS_INFORMATION", "  Used entries: %d", stats.used_entries);
        ESP_LOGI("NVS_INFORMATION", "  Free entries: %d", stats.free_entries);
        ESP_LOGI("NVS_INFORMATION", "  Total entries: %d", stats.total_entries);
        ESP_LOGI("NVS_INFORMATION", "  Namespace count: %d", stats.namespace_count);
    } else {
        ESP_LOGE("NVS_INFORMATION", "Failed to get NVS stats: %s", esp_err_to_name(err));
    }
    
  nvs_iterator_t it = NULL;
  err = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
  // err = nvs_entry_find_in_handle(<nvs_handle>, NVS_TYPE_ANY, &it);

 while(err == ESP_OK) {
     nvs_entry_info_t info;
     nvs_entry_info(it, &info); // Can omit error check if parameters are guaranteed to be non-NULL
      ESP_LOGI("NVS_INFORMATION","Namespace: %s, Key: %s, Type: %d\n", info.namespace_name, info.key, info.type);
     err = nvs_entry_next(&it);
 }
 nvs_release_iterator(it);
 
     
    // You can also get flash stats (raw partition usage)
   const esp_partition_t *nvs_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_NVS,
        "nvs"
    );

    if (nvs_part) {
        ESP_LOGI("NVS_INFO", "Partition label   : %s", nvs_part->label);
      //  ESP_LOGI("NVS_INFO", "Partition address : 0x%X", nvs_part->address);
        ESP_LOGI("NVS_INFO", "Partition size    : %lu bytes", nvs_part->size);
    } else {
        ESP_LOGE("NVS_INFO", "Default NVS partition not found!");
    }
     
  }//SYSTEM DEBUG 
   
}

 void READ_NVS_INIT_STRUCT(){
	 
	        uint8_t mac[6];
		    esp_efuse_mac_get_default(mac);
		    sprintf(NVS_STORAGE.ESP32_MAC_ADDRESS_STR,"%02X:%02X:%02X:%02X:%02X:%02X",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		    ESP_LOGI("ESP_INFORMATION","MAC Address:%s \n",NVS_STORAGE.ESP32_MAC_ADDRESS_STR);
	 
	       char *JSON_BODY_NVS_CONTENT = NULL;
	        
	       err = READ_JSON_BODY_FROM_NVS("NVS_STR_STRUCT", "LEN_NVS", "SERIAL_NUMBER", &JSON_BODY_NVS_CONTENT, NULL);        
           if(err == ESP_OK && JSON_BODY_NVS_CONTENT != NULL) {
		      strncpy(NVS_STORAGE.SERIAL_NUMBER_STRING,JSON_BODY_NVS_CONTENT,sizeof(NVS_STORAGE.SERIAL_NUMBER_STRING));
		      ESP_LOGI("ESP_INFORMATION","NVS_SERIAL_NUMBER_STRING [%d]:%s \n",strlen(NVS_STORAGE.SERIAL_NUMBER_STRING),NVS_STORAGE.SERIAL_NUMBER_STRING);
          }else{
			    strncpy(NVS_STORAGE.SERIAL_NUMBER_STRING,"",sizeof(NVS_STORAGE.SERIAL_NUMBER_STRING));   
		        ESP_LOGE("NVS_STR_STRUCT", "Error (%s) reading SERIAL_NUMBER_STRING from NVS", esp_err_to_name(err));
		    }
            
          vTaskDelay(5 / portTICK_PERIOD_MS);
         
	   
      if(JSON_BODY_NVS_CONTENT){
		 free(JSON_BODY_NVS_CONTENT);
		 JSON_BODY_NVS_CONTENT = NULL;
		  }
 }


void WIFI_INIT(){
	
	err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_NO_MEM) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
      
    }
	 
	WIFI_CREDENTIAL.MAX_STA_CONN_AP =  10;
	WIFI_CREDENTIAL.MAX_CONN_RETRY =    5;
	WIFI_CREDENTIAL.STATIC_DYNAMIC_IP = false;
	strcpy(WIFI_CREDENTIAL.GATEWAY_IP,"192.168.0.1");
	strcpy(WIFI_CREDENTIAL.STATIC_IP,"192.168.0.50");
	strcpy(WIFI_CREDENTIAL.HOST_NAME_STR,"ATTENDANCE");
	
	WIFI_CREDENTIAL.RSSI_DBI = 0;
	WIFI_CREDENTIAL.RSSI_PER = 0;
	WIFI_CREDENTIAL.MAX_AP_LIST = 16;
	WIFI_CREDENTIAL.CHANNEL_AP =   1;
    WIFI_CREDENTIAL.STA_AUTO_SWITCHING = false;
    WIFI_CREDENTIAL.STA_AUTO_SWITCHING_SCAN_RATE = 8000;
    
     strncpy(WIFI_CREDENTIAL.MAC_ADDRESS_STR,NVS_STORAGE.ESP32_MAC_ADDRESS_STR,sizeof(WIFI_CREDENTIAL.MAC_ADDRESS_STR));
    if(strlen(NVS_STORAGE.SERIAL_NUMBER_STRING) >= 12 && strcmp(NVS_STORAGE.SERIAL_NUMBER_STRING,"NOT_CONFIGUARED") !=0){
		
    strcpy(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR,NVS_STORAGE.SERIAL_NUMBER_STRING);
    }else{strcpy(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR,"NOT_CONFIGUARED");}
    
     // nvs_read_wifi();
    
     esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &HTTP_connect_handler,   &server);
     esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &HTTP_disconnect_handler, &server);
 
		    WIFI_MODE_APSTA_STA_ENABLE = false;
	        wifi_init_ap_sta();  
         
	    /*if(WIFI_CREDENTIAL.STA_AUTO_SWITCHING == true && strcmp(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR,"NOT_CONFIGUARED") != 0 && strlen(WIFI_CREDENTIAL.ESP_SERIAL_NUMBER_STR) >= 12 && strlen(WIFI_CREDENTIAL.AP_SSID_STA_MODE) > 4 && strlen(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE) > 4 && strcmp(WIFI_CREDENTIAL.AP_SSID_STA_MODE,"NOT_CONFIGUARED") !=0 && strcmp(WIFI_CREDENTIAL.AP_PASSWORD_STA_MODE,"NOT_CONFIGUARED") !=0){
		     
		     ESP_LOGI(TAG, "--------------------- WIFI STATION MODE\n");
		   //  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &HTTP_connect_handler, &server);
           //  esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &HTTP_disconnect_handler, &server);
  
		    // wifi_init_sta();  
		  	    
		     
		      
     }else{
		      
		    ESP_LOGI(TAG, "---------------------WIFI ACCESS MODE MODE\n");
		    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &HTTP_connect_handler,   &server);
            esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &HTTP_disconnect_handler, &server);
 
		    WIFI_MODE_APSTA_STA_ENABLE = false;
	        wifi_init_ap_sta();
	      
          }*/
        
      esp_wifi_get_mode(&WIFI_MODE);      
            
}


void main_event_handler(void *handler_arg,esp_event_base_t event_base,int32_t event_id,void *event_data){
	
  
 
		
   switch(event_id){
		
    case WIFI_STARTED:
    
        ESP_LOGI(EVENT_GROUP_TAG,"SYSTEM STARTED");
        ESP_LOGI(EVENT_GROUP_TAG,"Connecting to WiFi.........");
        WIFI_INIT();
            vTaskDelay(50 / portTICK_PERIOD_MS); 
      //  start_mdns_service();
        break;
        
    case IP_GET:
    
        ESP_LOGI(EVENT_GROUP_TAG,"STATION IP_GET ");
        vTaskDelay(15 / portTICK_PERIOD_MS); 
        //esp_event_post_to(main_event_handle,MAIN_EVENT_BASE,FIND_SBC,NULL,0,portMAX_DELAY);
        break;
      
   /* case FIND_SBC:
    
      if (MQTT_INNITIALIZED_FLAG && client != NULL && MQTT_CONNECTED_FLAG  == false){       
	        ESP_LOGW("main_event_handler", "FOURCE ESP MQTT STOP and esp_mqtt_client_unregister_event ");
            esp_mqtt_client_stop(client);
            esp_mqtt_client_unregister_event( client, ESP_EVENT_ANY_ID,mqtt_event_handler);
            err = esp_mqtt_client_destroy(client);
            if(err == ESP_OK){
			  client = NULL; 
              }
          }
          
        vTaskDelay( 100 / portTICK_PERIOD_MS);
        ESP_LOGW(EVENT_GROUP_TAG,"Searching SBC.......");
       // query_mdns_service(SERVICE_TYPE,SERVICE_PROTOCOL);
        
        break;
        
    case SEND_ESP_DATA:
    
         REPORT_DATA_SYNC_TASK_FLAG = false;
         vTaskDelay(200  / portTICK_PERIOD_MS);
         SEND_DCN_INFO_SBC(SBC_IP);
    
        break;
        
    case CONNECT_MQTT:
    
        
        vTaskDelay( 100 / portTICK_PERIOD_MS);
        ESP_LOGI(EVENT_GROUP_TAG," INITIALIED AND Connecting to MQTT Broker");
        INITAILZE_MQTT();
        
        break;
         
     case SEND_MQTT_DATA:
    
        ESP_LOGI(EVENT_GROUP_TAG,"STAR SENDING MQTT DATA");
        MQTT_REPORT_TASK_FLAG = true;
        break;
        
      case AP_STA_WIFI_FALLBACK:
    
        ESP_LOGI(EVENT_GROUP_TAG,"WIFI_FALLBACK");
        if(WIFI_MODE != WIFI_MODE_APSTA ){
		   WIFI_FALLBACK("WIFI_AP_STA");
		   STOP_START_HTTP_SERVER(&server , true ,true);  
		}
        break;  */
   
     default:
     
        ESP_LOGI(EVENT_GROUP_TAG,"DEFAULT");
        ESP_LOGI(EVENT_GROUP_TAG, "Event loop deleted.");
       // esp_event_loop_delete(main_event_handle);
      
        break;
        
    }
}
 /* ==================== LCD DISPLAY FUNCTIONS ==================== */

void lcd_show_welcome(void) {
    clear_lcd();
     vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    lcd_write("SMART ATTENDANCE  ");
    lcd_set_cursor(1, 0);
    lcd_write("    SYSTEM    ");
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void lcd_show_ready(void){
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    lcd_write(CLASSROOM_NAME);
    lcd_set_cursor(1, 0);
    lcd_write("Scan ID/FINGER");
}

void lcd_show_device(void) {
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    lcd_write(CLASSROOM_NAME);
    lcd_set_cursor(1, 0);
    lcd_write(DEPARTMENT_NAME);
}

void lcd_show_scanning(void) {
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    lcd_write("  Scanning...");
    lcd_set_cursor(1, 0);
    lcd_write("Please Wait");
}

void lcd_show_user(user_record_t *user, attendance_status_t status) {
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    
    char line1[17];
    snprintf(line1, sizeof(line1), "%.15s", user->name);
    lcd_write(line1);
    
    lcd_set_cursor(1, 0);
    
    char line2[17];
    if (status == ATTENDANCE_PRESENT) {
        snprintf(line2, sizeof(line2), "Present %02d:%02d", (int)(time(NULL) % 86400) / 3600,(int)(time(NULL) % 3600) / 60);
    } else {
      //  snprintf(line2, sizeof(line2), "Status: %s", attendance_status_to_string(status));
    }
    lcd_write(line2);
}

void lcd_show_error(const char *message) {
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    lcd_write("   ERROR!   ");
    lcd_set_cursor(1, 0);
    
    char line[17];
    snprintf(line, sizeof(line), "%.15s", message);
    lcd_write(line);
}

void lcd_show_message(const char *message) {
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    lcd_write("   NOTIFY  ");
    lcd_set_cursor(1, 0);
    
    char line[17];
    snprintf(line, sizeof(line), "%.15s", message);
    lcd_write(line);
}

void lcd_show_enrollment(const char *name, int step) {
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_set_cursor(0, 0);
    
    char line1[17];
    snprintf(line1, sizeof(line1), "ENROLL: %.8s", name);
    lcd_write(line1);
    
    lcd_set_cursor(1, 0);
    
    char line2[17];
    snprintf(line2, sizeof(line2), "Step %d/2", step);
    lcd_write(line2);
}

void lcd_show_stats(void) {
	
    clear_lcd();
    vTaskDelay(pdMS_TO_TICKS(10));
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



/* ==================== SERIALIZATION HELPERS ==================== */

static char* serialize_users_to_json(void) {
    
    if (!check_heap_available(15240, "serialize_users")) return NULL;
    
    cJSON *root = cJSON_CreateArray();
    if (!root) return NULL;
    
    for (int i = 0; i < total_users; i++){
        cJSON *user = cJSON_CreateObject();
        if (!user){
            cJSON_Delete(root);
            return NULL;
        }
        
        cJSON_AddNumberToObject(user, "user_id", user_database[i].user_id);
        cJSON_AddStringToObject(user, "name", user_database[i].name);
        cJSON_AddStringToObject(user, "email", user_database[i].email);
        cJSON_AddStringToObject(user, "roll_number", user_database[i].roll_number);
        cJSON_AddStringToObject(user, "department", user_database[i].department);
        cJSON_AddNumberToObject(user, "type", user_database[i].type);
        cJSON_AddNumberToObject(user, "status", user_database[i].status);
        cJSON_AddNumberToObject(user, "semester", user_database[i].semester);
        cJSON_AddBoolToObject(user, "fingerprint_enrolled", user_database[i].fingerprint_enrolled);
        cJSON_AddNumberToObject(user, "fingerprint_id", user_database[i].fingerprint_id);
        cJSON_AddBoolToObject(user, "rfid_enrolled", user_database[i].rfid_enrolled);
        cJSON_AddNumberToObject(user, "total_attendance", user_database[i].total_attendance);
        cJSON_AddNumberToObject(user, "total_classes", user_database[i].total_classes);
        cJSON_AddNumberToObject(user, "created_at", (double)user_database[i].created_at);
        cJSON_AddNumberToObject(user, "last_seen", (double)user_database[i].last_seen);
        
        cJSON_AddItemToArray(root, user);
    }
    
    char *json_str = cJSON_PrintUnformatted(root);
    if (root){cJSON_Delete(root);}
    return json_str;
}

static char* serialize_attendance_to_json(void) {
    if (!check_heap_available(20480, "serialize_attendance")) return NULL;
    
    cJSON *root = cJSON_CreateArray();
    if (!root) return NULL;
    
    for (uint32_t i = 0; i < total_records; i++) {
        cJSON *record = cJSON_CreateObject();
        if (!record) {
            cJSON_Delete(root);
            return NULL;
        }
        
        cJSON_AddNumberToObject(record, "record_id", attendance_records[i].record_id);
        cJSON_AddNumberToObject(record, "user_id", attendance_records[i].user_id);
        cJSON_AddNumberToObject(record, "timestamp", (double)attendance_records[i].timestamp);
        cJSON_AddNumberToObject(record, "status", attendance_records[i].status);
        cJSON_AddNumberToObject(record, "auth_method", attendance_records[i].auth_method);
        cJSON_AddStringToObject(record, "session_id", attendance_records[i].session_id);
        
        cJSON_AddItemToArray(root, record);
    }
    
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

static char* serialize_sessions_to_json(void) {
    if (!check_heap_available(8192, "serialize_sessions")) return NULL;
    
    cJSON *root = cJSON_CreateArray();
    if (!root) return NULL;
    
    for (int i = 0; i < total_sessions; i++) {
        cJSON *session = cJSON_CreateObject();
        if (!session) {
            cJSON_Delete(root);
            return NULL;
        }
        
        cJSON_AddStringToObject(session, "session_id", session_history[i].session_id);
        cJSON_AddStringToObject(session, "subject_name", session_history[i].subject_name);
        cJSON_AddStringToObject(session, "faculty_name", session_history[i].faculty_name);
        cJSON_AddStringToObject(session, "department", session_history[i].department);
        cJSON_AddNumberToObject(session, "start_time", (double)session_history[i].start_time);
        cJSON_AddNumberToObject(session, "end_time", (double)session_history[i].end_time);
        cJSON_AddNumberToObject(session, "total_present", session_history[i].total_present);
        cJSON_AddNumberToObject(session, "total_absent", session_history[i].total_absent);
        
        cJSON_AddItemToArray(root, session);
    }
    
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

static char* serialize_stats_to_json(void) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;
    
    cJSON_AddNumberToObject(root, "total_users", system_stats.total_users);
    cJSON_AddNumberToObject(root, "total_students", system_stats.total_students);
    cJSON_AddNumberToObject(root, "total_faculty", system_stats.total_faculty);
    cJSON_AddNumberToObject(root, "today_attendance_count", system_stats.today_attendance_count);
    cJSON_AddNumberToObject(root, "successful_scans", system_stats.successful_scans);
    cJSON_AddNumberToObject(root, "failed_scans", system_stats.failed_scans);
    cJSON_AddNumberToObject(root, "last_updated", (double)system_stats.last_updated);
    
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

/* ==================== DESERIALIZATION HELPERS ==================== */

static esp_err_t parse_users_from_json(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse users JSON");
        return ESP_FAIL;
    }
    
    if (!cJSON_IsArray(root)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    
    int count = cJSON_GetArraySize(root);
    if (count > MAX_USERS) {
        ESP_LOGW(TAG, "User count %d exceeds max %d, truncating", count, MAX_USERS);
        count = MAX_USERS;
    }
    
    total_users = 0;
    for (int i = 0; i < count; i++) {
        cJSON *user = cJSON_GetArrayItem(root, i);
        if (!user) continue;
        
        cJSON *item;
        
        item = cJSON_GetObjectItem(user, "user_id");
        if (item) user_database[i].user_id = item->valueint;
        
        item = cJSON_GetObjectItem(user, "name");
        if (item && item->valuestring) {
            strncpy(user_database[i].name, item->valuestring, MAX_NAME_LEN - 1);
        }
        
        item = cJSON_GetObjectItem(user, "email");
        if (item && item->valuestring) {
            strncpy(user_database[i].email, item->valuestring, MAX_EMAIL_LEN - 1);
        }
        
        item = cJSON_GetObjectItem(user, "roll_number");
        if (item && item->valuestring) {
            strncpy(user_database[i].roll_number, item->valuestring, MAX_ROLL_NUMBER_LEN - 1);
        }
        
        
        
        item = cJSON_GetObjectItem(user, "department");
        if (item && item->valuestring) {
            strncpy(user_database[i].department, item->valuestring, MAX_DEPARTMENT_LEN - 1);
        }
        
         item = cJSON_GetObjectItem(user, "department_code");
        if (item && item->valuestring) {
            strncpy(user_database[i].department_code, item->valuestring, 15);
        }
        
         item = cJSON_GetObjectItem(user, "classroom_name");
        if (item && item->valuestring) {
            strncpy(user_database[i].classroom_name, item->valuestring, MAX_DEPARTMENT_LEN - 1);
        }
        
         item = cJSON_GetObjectItem(user, "classroom_code");
        if (item && item->valuestring) {
            strncpy(user_database[i].classroom_code, item->valuestring, 15);
        }
        
        
        
        item = cJSON_GetObjectItem(user, "type");
        if (item) user_database[i].type = item->valueint;
        
        item = cJSON_GetObjectItem(user, "status");
        if (item) user_database[i].status = item->valueint;
        
        item = cJSON_GetObjectItem(user, "semester");
        if (item) user_database[i].semester = item->valueint;
        
        item = cJSON_GetObjectItem(user, "fingerprint_enrolled");
        if (item) user_database[i].fingerprint_enrolled = cJSON_IsTrue(item);
        
        item = cJSON_GetObjectItem(user, "fingerprint_id");
        if (item) user_database[i].fingerprint_id = item->valueint;
        
        item = cJSON_GetObjectItem(user, "rfid_enrolled");
        if (item) user_database[i].rfid_enrolled = cJSON_IsTrue(item);
        
        item = cJSON_GetObjectItem(user, "total_attendance");
        if (item) user_database[i].total_attendance = item->valueint;
        
        item = cJSON_GetObjectItem(user, "total_classes");
        if (item) user_database[i].total_classes = item->valueint;
        
        item = cJSON_GetObjectItem(user, "created_at");
        if (item) user_database[i].created_at = (time_t)item->valuedouble;
        
        item = cJSON_GetObjectItem(user, "last_seen");
        if (item) user_database[i].last_seen = (time_t)item->valuedouble;
        
        total_users++;
    }
    
    if(root){ cJSON_Delete(root);}
    ESP_LOGI(TAG, "Loaded %d users from NVS", total_users);
    return ESP_OK;
}

static esp_err_t parse_attendance_from_json(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse attendance JSON");
        return ESP_FAIL;
    }
    
    if (!cJSON_IsArray(root)) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    
    int count = cJSON_GetArraySize(root);
    if (count > MAX_ATTENDANCE_RECORDS) {
        ESP_LOGW(TAG, "Record count %d exceeds max %d, truncating", count, MAX_ATTENDANCE_RECORDS);
        count = MAX_ATTENDANCE_RECORDS;
    }
    
    total_records = 0;
    for (int i = 0; i < count; i++) {
        cJSON *record = cJSON_GetArrayItem(root, i);
        if (!record) continue;
        
        cJSON *item;
        
        item = cJSON_GetObjectItem(record, "record_id");
        if (item) attendance_records[i].record_id = item->valueint;
        
        item = cJSON_GetObjectItem(record, "user_id");
        if (item) attendance_records[i].user_id = item->valueint;
        
        item = cJSON_GetObjectItem(record, "timestamp");
        if (item) attendance_records[i].timestamp = (time_t)item->valuedouble;
        
        item = cJSON_GetObjectItem(record, "status");
        if (item) attendance_records[i].status = item->valueint;
        
        item = cJSON_GetObjectItem(record, "auth_method");
        if (item) attendance_records[i].auth_method = item->valueint;
        
        item = cJSON_GetObjectItem(record, "session_id");
        if (item && item->valuestring) {
            strncpy(attendance_records[i].session_id, item->valuestring, MAX_SESSION_ID_LEN - 1);
        }
        
        total_records++;
    }
    
    if(root){ cJSON_Delete(root);}
    ESP_LOGI(TAG, "Loaded %lu attendance records from NVS", total_records);
    return ESP_OK;
}

/* ==================== NVS LOAD/SAVE FUNCTIONS ==================== */

esp_err_t attendance_load_from_nvs(void) {
    
    ESP_LOGI(TAG, "Loading attendance data from NVS...");
    
    esp_err_t err;
    char *json_content = NULL;
     vTaskDelay(pdMS_TO_TICKS(10));
    // Load users
    err = READ_JSON_BODY_FROM_NVS(NVS_NAMESPACE, "users_len", NVS_KEY_USERS, &json_content, NULL);
    if (err == ESP_OK && json_content) {
        parse_users_from_json(json_content);
        free(json_content);
        json_content = NULL;
    } else {
        ESP_LOGW(TAG, "No users found in NVS or read failed");
        total_users = 0;
    }
     vTaskDelay(pdMS_TO_TICKS(10));
    // Load attendance records
    err = READ_JSON_BODY_FROM_NVS(NVS_NAMESPACE, "attendance_len", NVS_KEY_ATTENDANCE, &json_content, NULL);
    if (err == ESP_OK && json_content) {
        parse_attendance_from_json(json_content);
        free(json_content);
        json_content = NULL;
    } else {
        ESP_LOGW(TAG, "No attendance records found in NVS");
        total_records = 0;
    }
     vTaskDelay(pdMS_TO_TICKS(10));
    // Load stats
    err = READ_JSON_BODY_FROM_NVS(NVS_NAMESPACE, "stats_len", NVS_KEY_STATS, &json_content, NULL);
    if (err == ESP_OK && json_content) {
        cJSON *root = cJSON_Parse(json_content);
    
     system_stats.total_faculty  = 0;
     system_stats.total_students = 0;
     system_stats.total_users    = 0;
        if (root) {
            cJSON *item;
            item = cJSON_GetObjectItem(root, "total_users");
            if (item) system_stats.total_users = item->valueint;
            
            item = cJSON_GetObjectItem(root, "total_students");
            if (item) system_stats.total_students = item->valueint;
            
            item = cJSON_GetObjectItem(root, "total_faculty");
            if (item) system_stats.total_faculty = item->valueint;
            
            item = cJSON_GetObjectItem(root, "today_attendance_count");
            if (item) system_stats.today_attendance_count = item->valueint;
            
            item = cJSON_GetObjectItem(root, "successful_scans");
            if (item) system_stats.successful_scans = item->valueint;
            
            item = cJSON_GetObjectItem(root, "failed_scans");
            if (item) system_stats.failed_scans = item->valueint;
            
            cJSON_Delete(root);
        }
        free(json_content);
    }
    
    ESP_LOGI(TAG, "Loaded: %d users, %lu attendance records", total_users, total_records);
    return ESP_OK;
}

esp_err_t attendance_save_to_nvs(void) {
   
    ESP_LOGI(TAG, "Saving attendance data to NVS...");  
    esp_err_t err;
    char *json_str;
     vTaskDelay(pdMS_TO_TICKS(10));
    // Save users
    json_str = serialize_users_to_json();
    if (json_str) {
        err = STORE_JSON_BODY_TO_NVS(NVS_NAMESPACE, "users_len", NVS_KEY_USERS,  strlen(json_str), json_str, NULL);
        free(json_str);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save users to NVS");
            return err;
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Save attendance
    json_str = serialize_attendance_to_json();
    if (json_str) {
        err = STORE_JSON_BODY_TO_NVS(NVS_NAMESPACE, "attendance_len", NVS_KEY_ATTENDANCE, strlen(json_str), json_str, NULL);
        free(json_str);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save attendance to NVS");
            return err;
        }
    }
    
     vTaskDelay(pdMS_TO_TICKS(10));
    // Save stats
    json_str = serialize_stats_to_json();
    if (json_str) {
        err = STORE_JSON_BODY_TO_NVS(NVS_NAMESPACE, "stats_len", NVS_KEY_STATS, strlen(json_str), json_str, NULL);
        free(json_str);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save stats to NVS");
            return err;
        }
    }
    
    ESP_LOGI(TAG, "Saved attendance data to NVS successfully");
    return ESP_OK;
}



static int find_user_index_by_id(uint16_t user_id){
  
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].user_id == user_id) {
            return i;
        }
    }
    return -1;
}

int attendance_find_by_fingerprint(uint8_t fingerprint_id){
   
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].fingerprint_enrolled && 
            user_database[i].fingerprint_id == fingerprint_id) {
            return i;
        }
    }
    return -1;
}

user_record_t* attendance_find_by_rfid(const uint8_t *uid, uint8_t uid_len){
  
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].rfid_enrolled &&
            user_database[i].rfid_uid_len == uid_len &&
            memcmp(user_database[i].rfid_uid, uid, uid_len) == 0) {
            return &user_database[i];
        }
    }
    return NULL;
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


attendance_status_t attendance_mark(uint16_t user_id, attendance_status_t status, auth_method_t method){
  
  
/*  typedef enum {
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
   
} attendance_status_t;*/
  
    if (!current_session.active){
        ESP_LOGW("attendance_mark", "No active session");
        return SESSION_INACTIVE;
    }
    
    if(total_records >= MAX_ATTENDANCE_RECORDS){
        ESP_LOGE("attendance_mark", "Attendance records full!");
        return MEMORY_FULL;
    }
    
    int user_idx = find_user_index_by_id(user_id);
    if(user_idx < 0){
        ESP_LOGE("attendance_mark", "User not found");
        return USER_NOT_FOUND;
    }
    
    
    // Check if already marked
    for (uint32_t i = 0; i < total_records; i++){
        if (attendance_records[i].user_id == user_id && strcmp(attendance_records[i].session_id, current_session.session_id) == 0) {
            ESP_LOGW("attendance_mark", "*****User already marked attendance in this session*******");
            return ALREAY_MARKED;
        }
    }
    
    // Create record
    attendance_records[total_records].record_id = total_records + 200;
    attendance_records[total_records].user_id = user_id;
    attendance_records[total_records].timestamp = time(NULL);
    attendance_records[total_records].status = status;
    attendance_records[total_records].auth_method = method;
    strncpy(attendance_records[total_records].session_id, current_session.session_id, MAX_SESSION_ID_LEN - 1);
    
    total_records++;
    
    
    // Update user stats
        user_database[user_idx].total_classes++;
    if (status == ATTENDANCE_PRESENT || status == ATTENDANCE_LATE){
        user_database[user_idx].total_attendance++;
        current_session.total_present++;
    }
    
    user_database[user_idx].last_seen = time(NULL);
    
    // Update system stats
    system_stats.today_attendance_count++;
    system_stats.successful_scans++;
    system_stats.last_updated = time(NULL);
    
    // Save to NVS (async save recommended in production)
    attendance_save_to_nvs();
    
    ESP_LOGI("attendance_mark", "Attendance marked: User %d, Status %d, Method %d", user_id, status, method);
    return ATTENDANCE_DONE;
}

 
    
int16_t add_user(user_record_t *user){
   
     ESP_LOGE(TAG, "add_user current total_users %d",total_users); 
    
    if (total_users >= MAX_USERS){
        ESP_LOGE(TAG, "User database full!");
        return -1;
    }
    
     int user_idx = -2;
    for (int i = 0; i < total_users; i++){
        if (strcmp(user_database[i].roll_number, user->roll_number) == 0 || strcmp(user_database[i].email, user->email) == 0 ){
            user_idx = i;
            break;
        }
    }
    
    if (user_idx >= 0){
		ESP_LOGE(TAG, "NEW User added: Name=%s  id : %s  email : %s EMAIL OR ID EXITS", user->name, user->roll_number,user->email);
        return -2;
    }
    
    
    user->user_id = total_users + 100;
    user->created_at = time(NULL);
    user->total_attendance = 0;
    user->total_classes = 0;
    user->status = USER_STATUS_ACTIVE;
    user->fingerprint_enrolled = false,
    user->rfid_enrolled = false,
    
    memcpy(&user_database[total_users], user, sizeof(user_record_t));
    total_users++;
  
    system_stats.total_users = total_users;
    
    if (user->type == USER_TYPE_STUDENT) {
        system_stats.total_students++;
    } else if (user->type == USER_TYPE_FACULTY) {
        system_stats.total_faculty++;
    }
    
    // Save to NVS
    attendance_save_to_nvs();
    
    ESP_LOGI(TAG, "User added: ID=%d, Name=%s, Type=%s", user->user_id, user->name, user_type_to_string(user->type));
    
    return user->user_id;
}

/**
 * @brief GET /api/system/data - Return all system data
 */
 
 esp_err_t  api_system_data_handler(httpd_req_t *req){
	 
    ESP_LOGI(TAG, "GET /api/system/data");
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    if (!check_heap_available(40960, "system_data")) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Insufficient memory");
        return ESP_FAIL;
    }
    
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON creation failed");
        return ESP_FAIL;
    }
    
    // Add users array
    cJSON *users = cJSON_CreateArray();
    for (int i = 0; i < total_users; i++) {
		
        cJSON *user = cJSON_CreateObject();
        cJSON_AddNumberToObject(user, "user_id", user_database[i].user_id);
        cJSON_AddStringToObject(user, "name", user_database[i].name);
        cJSON_AddStringToObject(user, "email", user_database[i].email);
        cJSON_AddStringToObject(user, "roll_number", user_database[i].roll_number);
        cJSON_AddStringToObject(user, "department", user_database[i].department);
        cJSON_AddNumberToObject(user, "type", user_database[i].type);
        cJSON_AddNumberToObject(user, "semester", user_database[i].semester);
        cJSON_AddBoolToObject(user, "fingerprint_enrolled", user_database[i].fingerprint_enrolled);
        cJSON_AddBoolToObject(user, "rfid_enrolled", user_database[i].rfid_enrolled);
        
        // Calculate attendance percentage
        float percentage = 0;
        if (user_database[i].total_classes > 0) {
            percentage = (user_database[i].total_attendance * 100.0f) / user_database[i].total_classes;
        }
        cJSON_AddNumberToObject(user, "attendance_percentage", percentage);
        
        cJSON_AddItemToArray(users, user);
    }
    cJSON_AddItemToObject(root, "users", users);
    
    // Add attendance records (last 50 only to save memory)
    cJSON *attendance = cJSON_CreateArray();
    uint32_t start = (total_records > 50) ? (total_records - 50) : 0;
    for (uint32_t i = start; i < total_records; i++) {
        cJSON *record = cJSON_CreateObject();
        cJSON_AddNumberToObject(record, "record_id", attendance_records[i].record_id);
        cJSON_AddNumberToObject(record, "user_id", attendance_records[i].user_id);
        cJSON_AddNumberToObject(record, "timestamp", (double)attendance_records[i].timestamp);
        cJSON_AddNumberToObject(record, "status", attendance_records[i].status);
        cJSON_AddNumberToObject(record, "auth_method", attendance_records[i].auth_method);
        cJSON_AddStringToObject(record, "session_id", attendance_records[i].session_id);
        cJSON_AddItemToArray(attendance, record);
    }
    cJSON_AddItemToObject(root, "attendance", attendance);
    
    // Add current session
    cJSON *session = cJSON_CreateObject();
    cJSON_AddBoolToObject(session, "active", current_session.active);
    if (current_session.active) {
        cJSON_AddStringToObject(session, "session_id", current_session.session_id);
        cJSON_AddStringToObject(session, "subject_name", current_session.subject_name);
        cJSON_AddStringToObject(session, "faculty_name", current_session.faculty_name);
        cJSON_AddStringToObject(session, "department", current_session.department);
        cJSON_AddNumberToObject(session, "start_time", (double)current_session.start_time);
        cJSON_AddNumberToObject(session, "total_present", current_session.total_present);
    }
    cJSON_AddItemToObject(root, "current_session", session);
    
    // Add stats
    cJSON *stats = cJSON_CreateObject();
    cJSON_AddNumberToObject(stats, "total_users", system_stats.total_users);
    cJSON_AddNumberToObject(stats, "total_students", system_stats.total_students);
    cJSON_AddNumberToObject(stats, "total_faculty", system_stats.total_faculty);
    cJSON_AddNumberToObject(stats, "today_attendance_count", system_stats.today_attendance_count);
    cJSON_AddItemToObject(root, "stats", stats);
    
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    if (!json_str) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON serialization failed");
        return ESP_FAIL;
    }
    
    httpd_resp_sendstr(req, json_str);
    free(json_str);
    
    return ESP_OK;
}
/**
 * @brief POST /api/user/add - Add new user
 */
esp_err_t api_user_add_handler(httpd_req_t *req){
	
    ESP_LOGI(TAG, "POST /api/user/add");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    if (total_users >= MAX_USERS) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "User database full");
        return ESP_FAIL;
    }
    
    // Use FETCH_HTTP_POST_CONTENT to read body
    char *content = NULL;
    esp_err_t err = FETCH_HTTP_POST_CONTENT(req, &content);
    if (err != ESP_OK || !content) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request body");
        return ESP_FAIL;
    }ESP_LOGI(TAG, "user/add COTENT :%s", content);
    
    cJSON *root = cJSON_Parse(content);
    if (!root) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    

    user_record_t new_user ;
  
    
    cJSON *item;
    item = cJSON_GetObjectItem(root, "name");
    if (item && item->valuestring) {
        strncpy(new_user.name, item->valuestring, MAX_NAME_LEN - 1);
    }
    
    item = cJSON_GetObjectItem(root, "email");
    if (item && item->valuestring) {
        strncpy(new_user.email, item->valuestring, MAX_EMAIL_LEN - 1);
    }
    
    item = cJSON_GetObjectItem(root, "roll_number");
    if (item && item->valuestring) {
        strncpy(new_user.roll_number, item->valuestring, MAX_ROLL_NUMBER_LEN - 1);
    }
    
    item = cJSON_GetObjectItem(root, "department");
    if (item && item->valuestring) {
        strncpy(new_user.department, item->valuestring, MAX_DEPARTMENT_LEN - 1);
    }
    
    item = cJSON_GetObjectItem(root, "type");
    if (item) new_user.type = item->valueint;
    
    item = cJSON_GetObjectItem(root, "semester");
    if (item) new_user.semester = item->valueint;
    
   if(root) { cJSON_Delete(root); }
    if(content) {
	  free(content);
		content = NULL; }
		

		
     cJSON *response = cJSON_CreateObject();
    
    int16_t user_id = add_user(&new_user);
   if (user_id >=0) {
           ESP_LOGI(TAG, "NEW User added: ID=%d, Name=%s", user_id, new_user.name);
           cJSON_AddBoolToObject(response, "success", true);
           cJSON_AddNumberToObject(response, "user_id", user_id);
   }else if (user_id == -2){
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "User ID or Email exits");
   }else{ 
			 ESP_LOGE(TAG, "NEW User added failed Name=%s", new_user.name);
			cJSON_AddBoolToObject(response, "failed", false);
	}

    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    
    return ESP_OK;
}

/* ==================== USER UPDATE HANDLER ==================== */

esp_err_t api_user_update_handler(httpd_req_t *req){
   
    ESP_LOGI(TAG, "POST /api/user/update");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    char *content = NULL;
    esp_err_t err = FETCH_HTTP_POST_CONTENT(req, &content);
    if (err != ESP_OK || !content) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request");
        return ESP_FAIL;
    }ESP_LOGI(TAG, "user/update COTENT :%s", content);
    
    cJSON *root = cJSON_Parse(content);
    if (!root) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *user_id_item = cJSON_GetObjectItem(root, "user_id");
    if (!user_id_item) {
        cJSON_Delete(root);
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing user_id");
        return ESP_FAIL;
    }
    
    uint16_t user_id = user_id_item->valueint;
    
    // Find user in global array
    int user_idx = -1;
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].user_id == user_id) {
            user_idx = i;
            break;
        }
    }
    
    if (user_idx < 0) {
        cJSON_Delete(root);
        free(content);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "User not found");
        return ESP_FAIL;
    }
    
    // Update fields directly in global array (no local copies)
    user_record_t *user = &user_database[user_idx];
    
    cJSON *item;
    item = cJSON_GetObjectItem(root, "name");
    if (item && item->valuestring) {
        strncpy(user->name, item->valuestring, MAX_NAME_LEN - 1);
    }
    
    item = cJSON_GetObjectItem(root, "email");
    if (item && item->valuestring) {
        strncpy(user->email, item->valuestring, MAX_EMAIL_LEN - 1);
    }
    
    item = cJSON_GetObjectItem(root, "department");
    if (item && item->valuestring) {
        strncpy(user->department, item->valuestring, MAX_DEPARTMENT_LEN - 1);
    }
    
    item = cJSON_GetObjectItem(root, "semester");
    if (item) user->semester = item->valueint;
    
    item = cJSON_GetObjectItem(root, "status");
    if (item) user->status = item->valueint;
    
    cJSON_Delete(root);
    free(content);
    
    // Save to NVS
    attendance_save_to_nvs();
    
    // Response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddNumberToObject(response, "user_id", user_id);
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    ESP_LOGI(TAG, "User updated: ID=%d", user_id);
    return ESP_OK;
}

/* ==================== USER DELETE HANDLER ==================== */

esp_err_t api_user_delete_handler(httpd_req_t *req) {
	
    ESP_LOGI(TAG, "POST /api/user/delete");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    char *content = NULL;
    esp_err_t err = FETCH_HTTP_POST_CONTENT(req, &content);
    if (err != ESP_OK || !content) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "/user/delete COTENT :%s", content);
   
    cJSON *root = cJSON_Parse(content);
    if (!root) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *user_id_item = cJSON_GetObjectItem(root, "user_id");
    if (!user_id_item) {
        cJSON_Delete(root);
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing user_id");
        return ESP_FAIL;
    }
    
    uint16_t user_id = user_id_item->valueint;
    cJSON_Delete(root);
    free(content);
    
    // Find and remove user from global array
    int user_idx = -1;
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].user_id == user_id) {
            user_idx = i;
            break;
        }
    }
    
    if (user_idx < 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "User not found");
        return ESP_FAIL;
    }
    
    
    if(user_database[user_idx].fingerprint_enrolled){
    fingerprint_delete_template_id(user_database[user_idx].fingerprint_id);
    }
   
 
    for (int i = user_idx; i < total_users - 1; i++) {
        memcpy(&user_database[i], &user_database[i + 1], sizeof(user_record_t));
    }
       total_users--;
    
    if (user_database[user_idx].type == USER_TYPE_STUDENT) {
        system_stats.total_students--;
    } else if (user_database[user_idx].type == USER_TYPE_FACULTY) {
        system_stats.total_faculty--;
    }
    // Clear last entry
    memset(&user_database[total_users], 0, sizeof(user_record_t));
    
    
    // Update stats
    system_stats.total_users = total_users;
  
    
    
    
    // Save to NVS
    attendance_save_to_nvs();
    
    // Response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    ESP_LOGI(TAG, "User deleted: ID=%d", user_id);
    return ESP_OK;
}


/* ==================== FINGERPRINT ENROLLMENT HANDLER ==================== */

esp_err_t api_fingerprint_enroll_handler(httpd_req_t *req){
	
    ESP_LOGI(TAG, "POST /api/fingerprint/enroll");
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    char *content = NULL;
    esp_err_t err = FETCH_HTTP_POST_CONTENT(req, &content);
    if (err != ESP_OK || !content) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request");
        return ESP_FAIL;
    }
    
    cJSON *root = cJSON_Parse(content);
    if (!root) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *user_id_item = cJSON_GetObjectItem(root, "user_id");
    if (!user_id_item) {
        cJSON_Delete(root);
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing user_id");
        return ESP_FAIL;
    }
    
    uint16_t user_id = user_id_item->valueint;
    cJSON_Delete(root);
    free(content);
    
       if (user_id <= 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "CREATE AND SAVE USER FIRST");
        return ESP_FAIL;
       }
    // Find user in global array
    int user_idx = -1;
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].user_id == user_id) {
            user_idx = i;
            break;
        }
    }
    
    if (user_idx < 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "User not found");
        return ESP_FAIL;
    }
    
    user_record_t *user = &user_database[user_idx];
    
    // Check if fingerprint sensor is available
    if (!fp_handle || !system_status.fingerprint_ready){
		ESP_LOGE(TAG, "Fingerprint sensor not available");
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "error", "Fingerprint sensor not available");
        
        char *resp_str = cJSON_PrintUnformatted(response);
        cJSON_Delete(response);
        httpd_resp_sendstr(req, resp_str);
        free(resp_str);
        return ESP_OK;
    }
   
    // Find next available template ID
    uint16_t template_id = CURRENT_R305_TEMPLATE_COUNT  + 1;  // Simple mapping
    system_status.IDLE_DISPLAY_PRINT = false;
  
    // Start enrollment (blocking operation)
    ESP_LOGI(TAG, "Starting fingerprint enrollment for user %d, template %d", user_id, template_id);
    
    r305_status_t status = r305_enroll_finger(fp_handle, template_id, 10000);
    bool success = (status == R305_STATUS_OK);
    if (success) {
        user->fingerprint_enrolled = true;
        user->fingerprint_id = template_id;
        attendance_save_to_nvs();
        ESP_LOGI(TAG, "Fingerprint enrolled successfully");
    } else {
        ESP_LOGE(TAG, "Fingerprint enrollment failed reason: %s", r305_status_to_string(status));
    }
    
    // Response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", success);
    if (success) {
        cJSON_AddNumberToObject(response, "fingerprint_id", template_id);
    } else {
        cJSON_AddStringToObject(response, "error", r305_status_to_string(status));
    }
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
     system_status.IDLE_DISPLAY_PRINT = true;
    return ESP_OK;
}

/* ==================== RFID ENROLLMENT HANDLER ==================== */

esp_err_t api_rfid_enroll_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "POST /api/rfid/enroll");
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    char *content = NULL;
    esp_err_t err = FETCH_HTTP_POST_CONTENT(req, &content);
    if (err != ESP_OK || !content) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request");
        return ESP_FAIL;
    }
    
    cJSON *root = cJSON_Parse(content);
    if (!root) {
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *user_id_item = cJSON_GetObjectItem(root, "user_id");
    if (!user_id_item) {
        cJSON_Delete(root);
        free(content);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing user_id");
        return ESP_FAIL;
    }
    
    uint16_t user_id = user_id_item->valueint;
    cJSON_Delete(root);
    free(content);
    
     if (user_id <= 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "CREATE AND SAVE USER FIRST");
        return ESP_FAIL;
       }
       
    // Find user
    int user_idx = -1;
    for (int i = 0; i < total_users; i++) {
        if (user_database[i].user_id == user_id) {
            user_idx = i;
            break;
        }
    }
    
    if (user_idx < 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "User not found");
        return ESP_FAIL;
    }
    
    user_record_t *user = &user_database[user_idx];
    
    // Check if RFID reader is available
    if (!rfid_handle) {
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "error", "RFID reader not available");
        
        char *resp_str = cJSON_PrintUnformatted(response);
        cJSON_Delete(response);
        httpd_resp_sendstr(req, resp_str);
        free(resp_str);
        return ESP_OK;
    }
     system_status.IDLE_DISPLAY_PRINT = false;
    ESP_LOGI(TAG, "Waiting for RFID card...");
    
    // Wait for card (blocking with timeout)
    rc522_uid_t uid;
    bool card_found = false;
    
    for (int i = 0; i < 50; i++) {  // 10 second timeout
        bool present = false;
        rc522_is_card_present(rfid_handle, &present);
        
        if (present && rc522_read_card_uid(rfid_handle, &uid) == RC522_OK) {
            card_found = true;
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    if (card_found) {
        // Save UID to user
        memcpy(user->rfid_uid, uid.uid, uid.size);
        user->rfid_uid_len = uid.size;
        user->rfid_enrolled = true;
        
        attendance_save_to_nvs();
        ESP_LOGI(TAG, "RFID card enrolled successfully");
    } else {
        ESP_LOGW(TAG, "No card detected within timeout");
    }
    
    // Response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", card_found);
    if (!card_found) {
        cJSON_AddStringToObject(response, "error", "No card detected");
    }
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
       system_status.IDLE_DISPLAY_PRINT = true;
    return ESP_OK;
}


/* ==================== SESSION MANAGEMENT ==================== */

esp_err_t attendance_start_session(const char *subject, const char *faculty, const char *department) {
   
    if (current_session.active) {
        ESP_LOGW(TAG, "Session already active");
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(&current_session, 0, sizeof(current_session));
 
    current_session.active = true;
    current_session.start_time = time(NULL);
    snprintf(current_session.session_id, MAX_SESSION_ID_LEN, "S%08lX", (unsigned long)current_session.start_time);
    strncpy(current_session.subject_name, subject, MAX_SUBJECT_LEN - 1);
    strncpy(current_session.faculty_name, faculty, MAX_NAME_LEN - 1);
    strncpy(current_session.department, department, MAX_DEPARTMENT_LEN - 1);
    
    ESP_LOGI(TAG, "Session started: %s - %s (%s)", subject, faculty, department);
    return ESP_OK;
}

/* ==================== SESSION START HANDLER ==================== */

esp_err_t api_session_start_handler(httpd_req_t *req){
	
    ESP_LOGI(TAG, "POST/api/session/start");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    char *content = NULL;
    esp_err_t err = FETCH_HTTP_POST_CONTENT(req, &content);
    if (err != ESP_OK || !content){
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read request");
        return ESP_FAIL;
    } ESP_LOGI(TAG, "session/start content :%s",content);
    
    cJSON *root = cJSON_Parse(content);
    if(!root){
          if(content ){free(content);}
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *subject = cJSON_GetObjectItem(root, "subject");
    cJSON *faculty = cJSON_GetObjectItem(root, "faculty");
    cJSON *department = cJSON_GetObjectItem(root, "department");
    
    if (!subject || !faculty || !department){
         if(root ){ cJSON_Delete(root); }
         if(content ){free(content);}
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing required fields");
        return ESP_FAIL;
    }
    
    err = attendance_start_session(subject->valuestring, faculty->valuestring, department->valuestring);
    
    
   if(root ){ cJSON_Delete(root); }
   if(content ){free(content);}
    
    // Response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", err == ESP_OK);
    if (err == ESP_OK) {
        cJSON_AddStringToObject(response, "session_id", current_session.session_id);
    }
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    return ESP_OK;
}


esp_err_t attendance_stop_session(void){
   
    if (!current_session.active) {
        ESP_LOGW(TAG, "No active session");
        return ESP_ERR_INVALID_STATE;
    }
    
    current_session.active = false;
    current_session.end_time = time(NULL);
    
    // Mark absent users
    for (int i = 0; i < total_users; i++){
		
        if (user_database[i].status != USER_STATUS_ACTIVE) continue;
        
        // Check if user marked attendance in this session
        bool marked = false;
        for (uint32_t j = 0; j < total_records; j++){
            if (attendance_records[j].user_id == user_database[i].user_id &&strcmp(attendance_records[j].session_id, current_session.session_id) == 0){
                marked = true;
                break;
            }
        }
        
        if (!marked){
            current_session.total_absent++;
        }
    }
    
    // Save session to history
    if (total_sessions < MAX_SESSIONS){
        memcpy(&session_history[total_sessions], &current_session, sizeof(session_info_t));
        total_sessions++;
    }
    

    char *json_str = serialize_sessions_to_json();
    if(json_str){
        STORE_JSON_BODY_TO_NVS(NVS_NAMESPACE, "sessions_len", NVS_KEY_SESSIONS,strlen(json_str), json_str, NULL);
        free(json_str);
    }
    
    ESP_LOGI(TAG, "Session stopped: Present=%d, Absent=%d",current_session.total_present, current_session.total_absent);
    
    return ESP_OK;
}

/* ==================== SESSION STOP HANDLER ==================== */

esp_err_t api_session_stop_handler(httpd_req_t *req) {
   
    ESP_LOGI(TAG, "POST /api/session/stop");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    esp_err_t err = attendance_stop_session();
    
    // Response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", err == ESP_OK);
    if (err == ESP_OK){
        cJSON_AddNumberToObject(response, "total_present", current_session.total_present);
        cJSON_AddNumberToObject(response, "total_absent", current_session.total_absent);
    }
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    return ESP_OK;
}




/* ==================== ATTENDANCE REPORT HANDLER ==================== */

esp_err_t api_attendance_report_handler(httpd_req_t *req){
   
    ESP_LOGI(TAG, "GET /api/attendance/report");
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    // Parse query parameters for filtering
    char department[MAX_DEPARTMENT_LEN] = {0};
    char subject[MAX_SUBJECT_LEN] = {0};
    time_t start_date = 0;
    time_t end_date = time(NULL);
    
    // Get query parameters
    char query[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[64];
        
        if (httpd_query_key_value(query, "department", param, sizeof(param)) == ESP_OK) {
            strncpy(department, param, sizeof(department) - 1);
        }
        
        if (httpd_query_key_value(query, "subject", param, sizeof(param)) == ESP_OK) {
            strncpy(subject, param, sizeof(subject) - 1);
        }
        
        if (httpd_query_key_value(query, "start_date", param, sizeof(param)) == ESP_OK) {
            start_date = atol(param);
        }
        
        if (httpd_query_key_value(query, "end_date", param, sizeof(param)) == ESP_OK) {
            end_date = atol(param);
        }
    }
    
    // Build filtered report
    cJSON *root = cJSON_CreateObject();
    cJSON *records = cJSON_CreateArray();
    
    uint32_t total_present = 0;
    uint32_t total_absent = 0;
    uint32_t total_late = 0;
    
    for (uint32_t i = 0; i < total_records; i++) {
        // Apply filters
        if (start_date > 0 && attendance_records[i].timestamp < start_date) continue;
        if (end_date > 0 && attendance_records[i].timestamp > end_date) continue;
        
        // Find user to check department
        int user_idx = -1;
        for (int j = 0; j < total_users; j++) {
            if (user_database[j].user_id == attendance_records[i].user_id) {
                user_idx = j;
                break;
            }
        }
        
        if (user_idx < 0) continue;
        
        if (department[0] != '\0' && strcmp(user_database[user_idx].department, department) != 0) continue;
        
        // Add to results
        cJSON *record = cJSON_CreateObject();
        cJSON_AddStringToObject(record, "name", user_database[user_idx].name);
        cJSON_AddStringToObject(record, "roll_number", user_database[user_idx].roll_number);
        cJSON_AddNumberToObject(record, "timestamp", (double)attendance_records[i].timestamp);
        cJSON_AddNumberToObject(record, "status", attendance_records[i].status);
        cJSON_AddStringToObject(record, "session_id", attendance_records[i].session_id);
        
        cJSON_AddItemToArray(records, record);
        
        // Update counters
        if (attendance_records[i].status == ATTENDANCE_PRESENT) total_present++;
        else if (attendance_records[i].status == ATTENDANCE_LATE) total_late++;
        else if (attendance_records[i].status == ATTENDANCE_ABSENT) total_absent++;
    }
    
    cJSON_AddItemToObject(root, "records", records);
    
    cJSON *summary = cJSON_CreateObject();
    cJSON_AddNumberToObject(summary, "total_present", total_present);
    cJSON_AddNumberToObject(summary, "total_late", total_late);
    cJSON_AddNumberToObject(summary, "total_absent", total_absent);
    cJSON_AddNumberToObject(summary, "total_records", total_present + total_late + total_absent);
    
    if (total_present + total_late + total_absent > 0) {
        float percentage = (total_present * 100.0f) / (total_present + total_late + total_absent);
        cJSON_AddNumberToObject(summary, "attendance_percentage", percentage);
    }
    
    cJSON_AddItemToObject(root, "summary", summary);
    
    char *resp_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    return ESP_OK;
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
                        int user_idx=0;
                     //   int user_idx = find_user_by_rfid(&uid);
                        if (user_idx >= 0) {
                            user_record_t *user = &user_database[user_idx];
                            
                            if(attendance_mark(user->user_id,ATTENDANCE_PRESENT, AUTH_METHOD_RFID)) {
                                lcd_show_user(user, ATTENDANCE_PRESENT);
                                buzzer_play_pattern(BUZZER_PATTERN_SUCCESS);
                                ESP_LOGI(TAG, "✓ ACCESS GRANTED - %s (RFID)", user->name);
                           
                            }else{
                                lcd_show_error("Already marked");
                                buzzer_play_pattern(BUZZER_PATTERN_ERROR);
                            }
                            
                        }else{
                            lcd_show_error("Card not found");
                            buzzer_play_pattern(BUZZER_PATTERN_ERROR);
                            system_stats.failed_scans++;
                          }
                        
                        vTaskDelay(pdMS_TO_TICKS(3000));
                      //  lcd_show_ready();
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

  

void IDLE_DISPLAY_PRINT_task(void *pvParameters) {
	
	
	  // Main loop
    while (1) {
        if (system_status.IDLE_DISPLAY_PRINT && system_status.lcd_ready ){
	
			 vTaskDelay(pdMS_TO_TICKS(2000));
        if (system_ready && !current_session.active){
             lcd_show_stats();
             vTaskDelay(pdMS_TO_TICKS(3000));
            lcd_show_device();
        }else if (system_ready && current_session.active){
			 lcd_show_ready();
		}
		
		}
		
		 vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    
}


void fingerprint_scan_task(void *pvParameters) {
    
    while (1) {
		
        if (system_ready && current_session.active && system_status.fingerprint_ready){
            r305_search_result_t result;
            r305_status_t status = r305_search_finger(fp_handle, &result, 3000);
            if (status != R305_STATUS_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(500)); // Event handler takes care of feedback
            }
        }else{
            vTaskDelay(pdMS_TO_TICKS(1000));
            }
            
    } //while 
}

 esp_err_t fingerprint_delete_template_id(uint16_t template_id) {
   
    if (!system_ready && system_status.fingerprint_ready){
        ESP_LOGE(TAG, "System not ready");
        return ESP_FAIL;
    }
    
    
    r305_status_t status = r305_delete_template(fp_handle, template_id);
    if (status == R305_STATUS_OK) {
		
   
        ESP_LOGI(TAG, "template_id User deleted successfully : %d",template_id);
    } else {
        ESP_LOGE(TAG, "Failed to delete user: %s", r305_status_to_string(status));
         return ESP_FAIL;
    }
    
     return ESP_OK;
}


 esp_err_t fingerprint_clear_all_LIBRARY(void){
    
    if (!system_ready && system_status.fingerprint_ready){
        ESP_LOGE(TAG, "System not ready");
        return ESP_FAIL;
    }
    
    ESP_LOGW(TAG, "=====Clearing entire FINGERPRINT database=======");
    r305_status_t status = r305_clear_database(fp_handle);
    if(status == R305_STATUS_OK){
        CURRENT_R305_TEMPLATE_COUNT = 0;
        
        
        ESP_LOGI(TAG, "FINGERPRINT Database cleared successfully");
    }else{
        ESP_LOGE(TAG, "FINGERPRINT Failed to clear database: %s", r305_status_to_string(status));
         return ESP_FAIL;
      }
      
     return ESP_OK;
}

esp_err_t nvs_delete_key(const char *key){
	
	
	/*#define NVS_NAMESPACE "attendance"
#define NVS_KEY_USERS "users"
#define NVS_KEY_ATTENDANCE "attendance"
#define NVS_KEY_SESSIONS "sessions"
#define NVS_KEY_CONFIG "config"
#define NVS_KEY_STATS "stats"*/

    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK){
		 ESP_LOGE("nvs_delete_key", "DATABASE NVS FAILED OPEN KEY: %s",key);
		return err;
 
	} 
    err = nvs_erase_key(handle, key);
     if (err != ESP_OK) {
	 ESP_LOGE("nvs_delete_key", "DATABASE NVS FAILED DELETE KEY: %s",key);
      return err;}
   
   ESP_LOGI("nvs_delete_key", "DATABASE NVS SUCCESS DELETE KEY: %s",key);
    nvs_commit(handle);
    nvs_close(handle);
    
    return err;
}

esp_err_t api_reset_clear_database_handler(httpd_req_t *req){
   
    ESP_LOGI(TAG, "GET /api/system/reset");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    esp_err_t err = fingerprint_clear_all_LIBRARY();
   
    vTaskDelay(pdMS_TO_TICKS(10)); 
    nvs_delete_key(NVS_KEY_ATTENDANCE); 
    vTaskDelay(pdMS_TO_TICKS(10)); 
    nvs_delete_key(NVS_KEY_SESSIONS);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    nvs_delete_key(NVS_KEY_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    nvs_delete_key(NVS_KEY_STATS);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    
    attendance_save_to_nvs();
    total_users = 0;
    total_records = 0;
    total_sessions = 0;
    CURRENT_R305_TEMPLATE_COUNT =0;
    
    attendance_save_to_nvs();
    // Response
    cJSON *response = cJSON_CreateObject();
    if (err == ESP_OK){
            cJSON_AddBoolToObject(response, "success", true);
    }else{   cJSON_AddBoolToObject(response, "success", false);}
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    return ESP_OK;
}

esp_err_t api_clear_attendance_handler(httpd_req_t *req){
   
    ESP_LOGI(TAG, "GET /api/database/clear-attendance");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    
    esp_err_t err =  nvs_delete_key(NVS_KEY_ATTENDANCE); 
    attendance_save_to_nvs();
    // Response
    cJSON *response = cJSON_CreateObject();
    if (err == ESP_OK){
            cJSON_AddBoolToObject(response, "success", true);
    }else{   cJSON_AddBoolToObject(response, "success", false);}
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    return ESP_OK;
}



esp_err_t api_fingerprint_clear_database_handler(httpd_req_t *req){
   
    ESP_LOGI(TAG, "GET /api/fingerprint/clear_database");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    
    esp_err_t err = fingerprint_clear_all_LIBRARY();
    
    // Response
    cJSON *response = cJSON_CreateObject();
    if (err == ESP_OK){
		
		 for (int i = 0; i < total_users; i++){
              user_database[i].fingerprint_enrolled = false; 
              user_database[i].fingerprint_id = 0; 
              }  attendance_save_to_nvs();
       
		
            cJSON_AddBoolToObject(response, "success", true);
    }else{   cJSON_AddBoolToObject(response, "success", false);}
    
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    
    return ESP_OK;
}


void fingerprint_event_handler(r305_event_t *event, void *user_data){
    
    
/*    typedef enum {
    R305_EVENT_INITIALIZED,
  /  R305_EVENT_FINGER_DETECTED,
   / R305_EVENT_FINGER_REMOVED,
    R305_EVENT_IMAGE_CAPTURED,
  /  R305_EVENT_ENROLL_PROGRESS,
   / R305_EVENT_ENROLL_SUCCESS,
   / R305_EVENT_ENROLL_FAILED,
  /  R305_EVENT_MATCH_SUCCESS,
  /  R305_EVENT_MATCH_FAILED,
  /  R305_EVENT_SEARCH_SUCCESS,
  /  R305_EVENT_SEARCH_FAILED,
    R305_EVENT_ERROR,
} r305_event_type_t;*/

    switch (event->type) {
     
      case R305_EVENT_FINGER_DETECTED:
           lcd_show_message("FINGER_SEARCH..");
            break;
            
        case R305_EVENT_SEARCH_SUCCESS: {
			ESP_LOGI(TAG, "[EVENT]==========R305_EVENT_SEARCH_SUCCESS==============");
            r305_search_result_t *result = &event->data.search_result;
            int user_idx = attendance_find_by_fingerprint(result->template_id);
            if(result->found && user_idx >= 0){ 
                user_record_t *user = &user_database[user_idx];
                ESP_LOGI(TAG, "USER FOUND - %s (FP)======", user->name);
             
               attendance_status_t att_result_temp = attendance_mark(user->user_id,ATTENDANCE_PRESENT, AUTH_METHOD_FINGERPRINT);
              
                if( att_result_temp == ATTENDANCE_DONE){
                    lcd_show_user(user, ATTENDANCE_PRESENT);
                    //buzzer_play_pattern(BUZZER_PATTERN_SUCCESS);
                    ESP_LOGI(TAG, "=======ACCESS GRANTED USER PRESENT - %s (FP)===========", user->email);
               
                }else if(att_result_temp == ALREAY_MARKED){
					
					ESP_LOGE(TAG, "USER DATA Already marked (FP)");
                    lcd_show_error("Already marked");
                    //buzzer_play_pattern(BUZZER_PATTERN_ERROR);
                }else if(att_result_temp == SESSION_INACTIVE){
                   ESP_LOGE(TAG, "SESSION_INACTIVE (FP)");
                    lcd_show_error("SESSION_INACTIVE");
                }else if(att_result_temp == MEMORY_FULL){
					ESP_LOGE(TAG, " MEMORY_FULL (FP)");
                    lcd_show_error("MEMORY_FULL");
					 
				}else if(att_result_temp == USER_NOT_FOUND){
					ESP_LOGE(TAG, "USER DATA USER_NOT_FOUND (FP)");
                    lcd_show_error("USER_NOT_FOUND");
			    }

   
           }else{
				
				ESP_LOGE(TAG, "USER DATA N/A  (FP)");
                lcd_show_error("USER DATA N/A");
                //buzzer_play_pattern(BUZZER_PATTERN_ERROR);
               // system_stats.failed_scans++;
                 }
         
            break;
        }
        
        case R305_EVENT_MATCH_SUCCESS:
         
            break;
            
        case R305_EVENT_MATCH_FAILED:
         
            break;    
            
         case R305_EVENT_SEARCH_FAILED:
            lcd_show_error("NOT RECOGNIZED");
            //buzzer_play_pattern(BUZZER_PATTERN_ERROR);
            system_stats.failed_scans++;
            break;     
            
            
        case R305_EVENT_ENROLL_PROGRESS: {
			
            r305_enroll_progress_t *progress = &event->data.enroll_progress;
            int step = (progress->state <= R305_ENROLL_PROCESS_1) ? 1 : 2;
            lcd_show_enrollment("user", step);
           // buzzer_play_pattern(BUZZER_PATTERN_ENROLL_STEP);
            break;
        }
        
        case R305_EVENT_FINGER_REMOVED: {
            lcd_show_message("REMOVE_FINGER");
           // buzzer_play_pattern(BUZZER_PATTERN_ENROLL_STEP);
            break;
        }
        
         case R305_EVENT_ENROLL_SUCCESS: {
			 lcd_show_message("ENROLL_SUCCESS");
			 CURRENT_R305_TEMPLATE_COUNT++;
            //lcd_show_enrollment("REMOVE FINGER", 0);
           // buzzer_play_pattern(BUZZER_PATTERN_ENROLL_STEP);
            break;
        }
        
         case R305_EVENT_ENROLL_FAILED: {
			  lcd_show_error("ENROLL_FAILED");
            //lcd_show_enrollment("REMOVE FINGER", 0);
           // buzzer_play_pattern(BUZZER_PATTERN_ENROLL_STEP);
            break;
        }
         case R305_EVENT_ERROR: {
			  lcd_show_error("FP SENSOR ERROR");
            //lcd_show_enrollment("REMOVE FINGER", 0);
           // buzzer_play_pattern(BUZZER_PATTERN_ENROLL_STEP);
            break;
        }
      
        
        default:
            break;
    }
}

static bool initialize_Fingerprint_system(void){
	
    // Configure driver
    r305_config_t config = {
		
        .tx_pin = GPIO_NUM_16,
        .rx_pin = GPIO_NUM_17,
        .baud_rate = 9600,
        .security_level = R305_SECURITY_LEVEL_3,
        .device_address = R305_DEFAULT_ADDR,
        .event_callback = fingerprint_event_handler,
        .callback_user_data = NULL
    };
    
   
    ESP_LOGI("FINGER_PRINT", "Initializing R305 driver...");
    r305_status_t status = r305_init(&config, &fp_handle);
    if (status != R305_STATUS_OK){
        ESP_LOGE("FINGER_PRINT", "Driver initialization failed: %s", r305_status_to_string(status));
        return false;
    }
    
    // Verify communication
    ESP_LOGI("FINGER_PRINT", "Verifying module communication...");
    
    status = r305_handshake(fp_handle);
    if (status != R305_STATUS_OK){
        ESP_LOGE("FINGER_PRINT", "Handshake failed: %s", r305_status_to_string(status));
        return false;
    }
    
    ESP_LOGI("FINGER_PRINT", "Module communication verified!");
    
    // Read system parameters
    r305_sys_params_t params;
    status = r305_read_sys_params(fp_handle, &params);
    if (status == R305_STATUS_OK) {
		  ESP_LOGI("FINGER_PRINT", "=== System Parameters ===");
        ESP_LOGI("FINGER_PRINT", "System ID: 0x%04X", params.system_id);
        ESP_LOGI("FINGER_PRINT", "Library Size: %d", params.library_size);
        ESP_LOGI("FINGER_PRINT", "Security Level: %d", params.security_level);
        ESP_LOGI("FINGER_PRINT", "Device Address: 0x%08lX", params.device_address);
        ESP_LOGI("FINGER_PRINT", "Packet Size: %d bytes", params.packet_size_bytes);
        ESP_LOGI("FINGER_PRINT", "Baud Rate: %lu bps", params.baud_rate);
        ESP_LOGI("FINGER_PRINT", "========================");
    }
    
  /*  // Read module info
    uint8_t info_buf[256];
    size_t info_len;
    status = r305_read_info_page(fp_handle, info_buf, sizeof(info_buf), &info_len);
    if (status == R305_STATUS_OK) {
        ESP_LOGI("FINGER_PRINT", "Module info: %.*s", (int)info_len, info_buf);
    }*/
    
    // Get current template count

    status = r305_get_template_count(fp_handle, &CURRENT_R305_TEMPLATE_COUNT);
    if (status == R305_STATUS_OK) {
        ESP_LOGI("FINGER_PRINT", "CURRENT_R305_TEMPLATE_COUNT: %d", CURRENT_R305_TEMPLATE_COUNT);
        //current_user_count = template_count;
    }
    
 
    ESP_LOGI("FINGER_PRINT", "System initialization complete!");
  
     system_status.fingerprint_ready = true;
    return true;
}


bool Initialize_RFID(){
	
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
    
     return true;
}

/* ==================== INITIALIZATION ==================== */

static esp_err_t init_attendance_database(void) {
	
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "STEP 4: Initializing Attendance Database");
    ESP_LOGI(TAG, "========================================");
    
    if (!system_status.nvs_ready) {
        ESP_LOGE(TAG, "Cannot initialize database: NVS not ready return");
        system_status.database_ready = false;
        return ESP_FAIL;
    }
   
      // Clear arrays
    memset(user_database, 0, sizeof(user_database));
    memset(attendance_records, 0, sizeof(attendance_records));
    memset(session_history, 0, sizeof(session_history));
    memset(&current_session, 0, sizeof(current_session));
    memset(&system_stats, 0, sizeof(system_stats));
    
   
  
    // Call main initialization from attendance_system_optimized.c
    esp_err_t ret = attendance_load_from_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Attendance database initialized successfully");
        ESP_LOGI(TAG, "  Total users: %d", total_users);
        ESP_LOGI(TAG, "Memory usage: Users=%u bytes, Attendance=%u bytes", sizeof(user_database), sizeof(attendance_records));
        ESP_LOGI(TAG, "  Memory usage: ~%d KB", (sizeof(user_database) + sizeof(attendance_records)) / 1024);
        
        // If no users exist, add sample users for testing
        if (total_users == 0) {
            ESP_LOGW(TAG, "No users found in NVS, adding sample users...");
            // Sample Student
            user_record_t sample_user = {
               // .user_id = 0,
                .type = USER_TYPE_STUDENT,
                .status = USER_STATUS_ACTIVE,
                .semester = 6,
                .fingerprint_enrolled = false,
                .rfid_enrolled = false,
               // .created_at = time(NULL)
            };
            
            strncpy(sample_user.name, "John Doe", MAX_NAME_LEN - 1);
            strncpy(sample_user.roll_number, "CS2021001", MAX_ROLL_NUMBER_LEN - 1);
            strncpy(sample_user.department, "Computer Science", MAX_DEPARTMENT_LEN - 1);
            strncpy(sample_user.email, "john@university.edu", MAX_EMAIL_LEN - 1);
            
            ret = add_user(&sample_user);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Sample user 1 added: %s", sample_user.name);
            }
            
            // Sample Faculty
            user_record_t faculty_user = {
                .type = USER_TYPE_FACULTY,
                .status = USER_STATUS_ACTIVE,
                .fingerprint_enrolled = false,
                .rfid_enrolled = false,
                //.created_at = time(NULL)
            };
            
            strncpy(faculty_user.name, "Prof. Smith", MAX_NAME_LEN - 1);
            strncpy(faculty_user.department, "Computer Science", MAX_DEPARTMENT_LEN - 1);
            strncpy(faculty_user.email, "smith@university.edu", MAX_EMAIL_LEN - 1);
            
            add_user(&faculty_user);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Sample user 2 added: %s", sample_user.name);
            }
            ESP_LOGI(TAG, "Sample users saved to NVS");
        }
        
        system_status.database_ready = true;
    } else {
        ESP_LOGE(TAG, "✗ Attendance database initialization failed: %s", esp_err_to_name(ret));
        system_status.database_ready = false;
    }
    
    if (system_status.lcd_ready) {
       // lcd_show_stats("Database", system_status.database_ready);
    }
    
    return ret;
}

bool initialize_attendance_system(void){
  
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  CLASSROOM ATTENDANCE SYSTEM");
    ESP_LOGI(TAG, "  Department: %s", DEPARTMENT_NAME);
    ESP_LOGI(TAG, "  Classroom: %s", CLASSROOM_NAME);
    ESP_LOGI(TAG, "========================================");
    
   //Initialize LCD
    ESP_LOGI(TAG, "Initializing LCD...");
    LCD_INITILIZED();
    system_status.lcd_ready = true;
    system_status.IDLE_DISPLAY_PRINT = true;
    lcd_show_welcome();
    
    // Initialize Buzzer
  //  ESP_LOGI(TAG, "Initializing Buzzer...");
   // buzzer_init();
   // buzzer_play_pattern(BUZZER_PATTERN_STARTUP);
    
        err = init_attendance_database();
        if(err != ESP_OK){
        ESP_LOGE(TAG, "CRITICAL: Database initialization failed!");  
        }
   
    if (!initialize_Fingerprint_system()) {
        ESP_LOGE(TAG, "System   Fingerprint Initialize  failed!");
        ESP_LOGE(TAG, "Please check connections and restart");
        return 0;
    }
    
    //Initialize_RFID();

    
 
    // Initialize statistics
    system_stats.system_start_time = time(NULL);
    

    //buzzer_play_pattern(BUZZER_PATTERN_SUCCESS);

    
    system_ready = true;
    return true;
}

/* ==================== NVS INITIALIZATION ==================== */

static esp_err_t init_nvs_flash(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "STEP 1: Initializing NVS Flash");
    ESP_LOGI(TAG, "========================================");
    
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition full or version mismatch, erasing...");
        
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(ret));
            system_status.nvs_ready = false;
            return ret;
        }
        
        ret = nvs_flash_init();
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ NVS Flash initialized successfully");
        
        // Print NVS stats
        nvs_stats_t stats;
        if (nvs_get_stats(NULL, &stats) == ESP_OK) {
            ESP_LOGI(TAG, "NVS Stats:");
            ESP_LOGI(TAG, "  Used entries: %d", stats.used_entries);
            ESP_LOGI(TAG, "  Free entries: %d", stats.free_entries);
            ESP_LOGI(TAG, "  Total entries: %d", stats.total_entries);
            ESP_LOGI(TAG, "  Namespace count: %d", stats.namespace_count);
        }
        
        system_status.nvs_ready = true;
    } else {
        ESP_LOGE(TAG, "✗ NVS Flash initialization failed: %s", esp_err_to_name(ret));
        system_status.nvs_ready = false;
    }
    
    return ret;
}

void app_main(void){
	
	 init_nvs_flash();
        
  // nvs_flash_erase();
   // print_nvs_info();
   
  ESP_LOGI(TAG, "Starting Classroom Attendance System...");
   
  snprintf(NVS_STORAGE.SERIAL_NUMBER_STRING,sizeof(NVS_STORAGE.SERIAL_NUMBER_STRING), "%s","20260228-21000567"); 
  snprintf(DCN_FOLDER_NAME,sizeof(DCN_FOLDER_NAME), "%s",NVS_STORAGE.SERIAL_NUMBER_STRING);  /// COMPULSORY AFTER  READ_NVS_INIT_STRUCT(); 
  snprintf(DEVICE_LOGGER.AUTOMATION_DEVICE_NAME,sizeof(DEVICE_LOGGER.AUTOMATION_DEVICE_NAME), "DCN-%s",NVS_STORAGE.SERIAL_NUMBER_STRING);
 
         INIT_SPIFFS(); 
         vTaskDelay(pdMS_TO_TICKS(50)); 
         WIFI_INIT();
           
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    if (!initialize_attendance_system()) {
        ESP_LOGE(TAG, "System initialization failed!");
        return;
    }
    
    // Create scanning tasks
   // xTaskCreate(rfid_scan_task, "rfid_scan", 4096, NULL, 5, NULL);
    xTaskCreate(fingerprint_scan_task, "fp_scan", 4096, NULL, 5, NULL);
    xTaskCreate(IDLE_DISPLAY_PRINT_task, "IDLE_DISPLAY_PRINT", 2096, NULL, 4, NULL);
    

}


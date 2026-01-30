#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "unistd.h"

//I2C
#include "driver/i2c.h"

#include "LCD_DISPLAY.h"

#define I2C_MASTER_SCL_IO 22   // Define GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO 21   // Define GPIO number for I2C SDA
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000  // Frequency of I2C bus

#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master does not need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0

#define I2C_LCD_ADDR 0x27

/* PCF8574 bit mapping */
#define LCD_BACKLIGHT   0x08
#define LCD_ENABLE      0x04
#define LCD_RS          0x01
static esp_err_t lcd_err;

void i2c_master_init()

{

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,  // GPIO number for SDA
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,  // GPIO number for SCL
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Function to send command to the LCD
esp_err_t lcd_send_cmd(uint8_t cmd){
	
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;  // EN=1, RS=0
    data_t[1] = data_u | 0x08;//E  // EN=0, RS=0
    data_t[2] = data_l | 0x0C;  // EN=1, RS=0
    data_t[3] = data_l | 0x08;  // EN=0, RS=0
  
    lcd_err = i2c_master_write_to_device(I2C_MASTER_NUM, I2C_LCD_ADDR, data_t, sizeof(data_t), 1000 /portTICK_PERIOD_MS);
   
     return lcd_err;

}

// Function to send data to the LCD
esp_err_t lcd_send_data(uint8_t data)

{
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;  // EN=1, RS=1
    data_t[1] = data_u | 0x09 ;  //9;  // EN=0, RS=1
    data_t[2] = data_l | 0x0D;  // EN=1, RS=1
    data_t[3] = data_l | 0x09;  // EN=0, RS=1
    lcd_err =  i2c_master_write_to_device(I2C_MASTER_NUM, I2C_LCD_ADDR, data_t, sizeof(data_t), 1000 / portTICK_PERIOD_MS);
   return lcd_err;
}

void lcd_init()
{


    // Initialization sequence as per the datasheet
    lcd_send_cmd(0x30);
    vTaskDelay(5 /  portTICK_PERIOD_MS);
  
    lcd_send_cmd(0x30);
    vTaskDelay(5 /  portTICK_PERIOD_MS);
    
    lcd_send_cmd(0x30);
    vTaskDelay(10 /  portTICK_PERIOD_MS);

    // Set to 4-bit mode
    lcd_send_cmd(0x20);
    vTaskDelay(10 /  portTICK_PERIOD_MS);

    // Function set: 2 lines, 5x8 font
    lcd_send_cmd(0x28);
    vTaskDelay(10 /  portTICK_PERIOD_MS);

    // Display on, cursor off
    lcd_send_cmd(0x0C);
    vTaskDelay(10 /  portTICK_PERIOD_MS);

    // Clear display
    lcd_send_cmd(0x01);
    vTaskDelay(10/  portTICK_PERIOD_MS);

    // Entry mode set: increment automatically, no display shift
    lcd_send_cmd(0x06);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	
    uint8_t address;

    // Calculate the DDRAM address based on the row and column
    switch (row) {
        case 0:
            address = 0x00 + col;  // First row starts at 0x00
            break;
        case 1:
            address = 0x40 + col;  // Second row starts at 0x40
            break;
        // Add more cases for additional rows if your LCD has more than 2 rows
        default:
            address = 0x00 + col;  // Default to first row
    }

    // Send command to set DDRAM address
    lcd_send_cmd(0x80 | address);  // 0x80 is the command for setting DDRAM address
}


void lcd_write(char* str)
{
	
    for (int i = 0; str[i] != '\0'; i++) {
        lcd_send_data((uint8_t)(str[i]));  // Send current character
        vTaskDelay(2 / portTICK_PERIOD_MS);  // Delay between characters (blocking)
    }

}


void clear_lcd(){

	lcd_send_cmd(0x01);
	 vTaskDelay(5 /  portTICK_PERIOD_MS);
	 }

/*esp_err_t lcd_backlight(bool enable)
{
    backlight_state = enable ? LCD_BACKLIGHT : 0x00;
    return lcd_send_cmd(0x00); // refresh
}*/



esp_err_t lcd_display_on(bool enable)
{
    return lcd_send_cmd(enable ? 0x0C : 0x08);
}

void LCD_INITILIZED(void)
{
    i2c_master_init();
    lcd_init();
    
    clear_lcd();
    
    
    // lcd_set_cursor(0, 0);
    // lcd_write("I2C LCD OK");
   
   // lcd_set_cursor(1, 0);
   // lcd_write("WEIGHT TEST");
}



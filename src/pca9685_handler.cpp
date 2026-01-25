

#define I2CDEV_NOLOCK 1

#include "esp_idf_lib_helpers.h"
#include "i2cdev.h"
#include "pca9685.h"
#include "driver/i2c.h"
#include "pca9685_handler.hpp"
#include "esp_log.h"

#include "freertos/task.h"
#include "esp_log.h"

namespace PCA9685Handler {


    void log_task_list()
{
    static char buffer[1024];
    vTaskList(buffer);
    ESP_LOGI("RTOS", "\nTask            State   Prio    Stack    Num\n%s", buffer);
}

    static const char* TAG = "PCA9685";
    static i2c_dev_t dev;
    
    void scan_i2c()
{
    ESP_LOGI(TAG, "Scanning I2C...");
    i2c_dev_t temp_dev;
    memset(&temp_dev, 0, sizeof(i2c_dev_t));
    temp_dev.port = dev.port;
    temp_dev.cfg = dev.cfg;

    for (uint8_t addr = 1; addr < 127; addr++)
    {
        temp_dev.addr = addr;
        if (i2c_dev_probe(&temp_dev, I2C_DEV_WRITE) == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
}

    void init(uint8_t i2c_sda, uint8_t i2c_scl, uint32_t freq_hz) {
        ESP_ERROR_CHECK(i2cdev_init());
        
        ESP_LOGI(TAG, "Initializing PCA9685 descriptor");
        ESP_ERROR_CHECK(pca9685_init_desc(&dev, 0x40, I2C_NUM_0, (gpio_num_t)i2c_sda, (gpio_num_t)i2c_scl));
        
        // Manual pull-up configuration in the descriptor
        dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
        dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
        dev.cfg.master.clk_speed = 400000; // Set to 400kHz for stability

        scan_i2c();
        
        ESP_LOGI(TAG, "Initializing PCA9685 hardware");
        ESP_ERROR_CHECK(pca9685_init(&dev));
        ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, freq_hz));
        
        ESP_LOGI(TAG, "PCA9685 initialized at 0x40");
        for(int i = 0; i < 16; i++){
            set_section_state(i, true);
        }
    }
    
    void set_section_state(uint8_t index, bool active) {
        if (index >= 8) return;
        index = index*2;
    
        if (active) {
            pca9685_set_pwm_value(&dev, index, 4095);  // full on
            pca9685_set_pwm_value(&dev, index+1, 4095);  // full on
        } else {
            pca9685_set_pwm_value(&dev, index, 0);     // full off
            pca9685_set_pwm_value(&dev, index+1, 0);     // full off
        }
    }
    
    } // namespace PCA9685Handler
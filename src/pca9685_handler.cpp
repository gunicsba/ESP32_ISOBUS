

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
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }

    uint8_t zero = 0x00;
    esp_err_t err = i2c_dev_write_reg(&dev, 0x00, &zero, 1);
    ESP_LOGI(TAG, "Manual write: %s", esp_err_to_name(err));

}

    void init(uint8_t i2c_sda, uint8_t i2c_scl, uint32_t freq_hz) {
        log_task_list();
        i2c_config_t i2c_conf = {};
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = (gpio_num_t)i2c_sda;
        i2c_conf.scl_io_num = (gpio_num_t)i2c_scl;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.master.clk_speed = 400000;

        ESP_LOGI(TAG, "Starting I2C");
        ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
        ESP_LOGI(TAG, "Installing driver for I2C");
//        ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
//        ESP_LOGI(TAG, "Driver already installed, skipping second install");
        ESP_LOGI(TAG, "I2C started, initalizing PCA");
        log_task_list();
    
        ESP_ERROR_CHECK(pca9685_init_desc(&dev, 0x40, I2C_NUM_0, (gpio_num_t)i2c_sda, (gpio_num_t)i2c_scl));
        ESP_LOGI(TAG, "Init desc done");
        scan_i2c();
        ESP_LOGI(TAG, "Scan done");
        ESP_ERROR_CHECK(pca9685_init(&dev));
        ESP_LOGI(TAG, "PCA Init done");
        ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, freq_hz));
        ESP_LOGI(TAG, "PCA9685 initialized at 0x40");
        for(int i = 0; i < 16; i++){
            set_section_state(i,true);
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
    
    } // namespace SectionLED
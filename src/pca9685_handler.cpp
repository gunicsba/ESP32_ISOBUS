#include "pca9685.h"
#include "driver/i2c.h"
#include "pca9685_handler.hpp"
#include "esp_log.h"

namespace PCA9685Handler {

    static const char* TAG = "PCA9685";
    static pca9685_t dev;
    
    void init(uint8_t i2c_sda, uint8_t i2c_scl, uint32_t freq_hz) {
        i2c_config_t i2c_conf = {};
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = (gpio_num_t)i2c_sda;
        i2c_conf.scl_io_num = (gpio_num_t)i2c_scl;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.master.clk_speed = 400000;
    
        ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    
        ESP_ERROR_CHECK(pca9685_init_desc(&dev, 0x40, I2C_NUM_0, i2c_sda, i2c_scl));
        ESP_ERROR_CHECK(pca9685_init(&dev));
        ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&dev, freq_hz));
    
        ESP_LOGI(TAG, "PCA9685 initialized at 0x40");
    }
    
    void set_section_state(uint8_t index, bool active) {
        if (index >= 16) return;
    
        if (active) {
            pca9685_set_pwm_value(&dev, index, 4095);  // full on
        } else {
            pca9685_set_pwm_value(&dev, index, 0);     // full off
        }
    }
    
    } // namespace SectionLED
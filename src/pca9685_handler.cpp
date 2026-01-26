
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "pca9685_handler.hpp"
#include <cstring>

// Exact Adafruit register definitions from Adafruit_PWMServoDriver.h
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_ALLCALLADR 0x05
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
#define PCA9685_ALLLED_ON_L 0xFA
#define PCA9685_ALLLED_ON_H 0xFB
#define PCA9685_ALLLED_OFF_L 0xFC
#define PCA9685_ALLLED_OFF_H 0xFD
#define PCA9685_PRESCALE 0xFE
#define PCA9685_TESTMODE 0xFF

#define MODE1_ALLCAL 0x01
#define MODE1_SUB3 0x02
#define MODE1_SUB2 0x04
#define MODE1_SUB1 0x08
#define MODE1_SLEEP 0x10
#define MODE1_AI 0x20
#define MODE1_EXTCLK 0x40
#define MODE1_RESTART 0x80

#define MODE2_OUTNE_0 0x01
#define MODE2_OUTNE_1 0x02
#define MODE2_OUTDRV 0x04
#define MODE2_OCH 0x08
#define MODE2_INVRT 0x10

#define PCA9685_I2C_ADDRESS 0x40
#define FREQUENCY_OSCILLATOR 25000000
#define PCA9685_PRESCALE_MIN 3
#define PCA9685_PRESCALE_MAX 255

#define I2C_PORT I2C_NUM_0
#define NUM_SECTIONS 16

namespace PCA9685Handler {

    static const char* TAG = "PCA9685";
    static SemaphoreHandle_t i2c_mutex = NULL;
    static bool initialized = false;
    static bool section_states[NUM_SECTIONS] = {false};
    static bool pending_states[NUM_SECTIONS] = {false};
    static uint32_t _oscillator_freq = FREQUENCY_OSCILLATOR;

    // Exact Adafruit i2c_dev->write() equivalent
    static bool i2c_dev_write(uint8_t *buffer, size_t len) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCA9685_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        for (size_t i = 0; i < len; i++) {
            i2c_master_write_byte(cmd, buffer[i], true);
        }
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
        }
        return (ret == ESP_OK);
    }

    // Exact Adafruit i2c_dev->write_then_read() equivalent
    static bool i2c_dev_write_then_read(uint8_t *write_buf, size_t write_len, 
                                         uint8_t *read_buf, size_t read_len) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCA9685_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        for (size_t i = 0; i < write_len; i++) {
            i2c_master_write_byte(cmd, write_buf[i], true);
        }
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCA9685_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
        if (read_len > 1) {
            i2c_master_read(cmd, read_buf, read_len - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, read_buf + read_len - 1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        return (ret == ESP_OK);
    }

    // Exact Adafruit write8 from line 371-374
    static void write8(uint8_t addr, uint8_t d) {
        uint8_t buffer[2] = {addr, d};
        i2c_dev_write(buffer, 2);
    }

    // Exact Adafruit read8 from line 365-369
    static uint8_t read8(uint8_t addr) {
        uint8_t buffer[1] = {addr};
        i2c_dev_write_then_read(buffer, 1, buffer, 1);
        return buffer[0];
    }

    // Exact Adafruit reset from line 90-93
    static void reset() {
        write8(PCA9685_MODE1, MODE1_RESTART);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Exact Adafruit setPWMFreq from line 144-180
    static void setPWMFreq(float freq) {
        if (freq < 1) freq = 1;
        if (freq > 3500) freq = 3500;

        float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
        if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
        if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;
        uint8_t prescale = (uint8_t)prescaleval;

        ESP_LOGI(TAG, "Prescale: %d for %.0f Hz", prescale, freq);

        uint8_t oldmode = read8(PCA9685_MODE1);
        uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
        write8(PCA9685_MODE1, newmode);           // go to sleep
        write8(PCA9685_PRESCALE, prescale);       // set the prescaler
        write8(PCA9685_MODE1, oldmode);
        vTaskDelay(pdMS_TO_TICKS(5));
        write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
    }

    // Exact Adafruit setPWM from line 235-258
    static uint8_t setPWM(uint8_t num, uint16_t on, uint16_t off) {
        uint8_t buffer[5];
        buffer[0] = PCA9685_LED0_ON_L + 4 * num;
        buffer[1] = on;
        buffer[2] = on >> 8;
        buffer[3] = off;
        buffer[4] = off >> 8;

        ESP_LOGI(TAG, "setPWM ch=%d reg=0x%02X bytes=[0x%02X,0x%02X,0x%02X,0x%02X]", 
                 num, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

        if (i2c_dev_write(buffer, 5)) {
            return 0;
        } else {
            ESP_LOGE(TAG, "FAILED ch=%d", num);
            return 1;
        }
    }

    // Exact Adafruit setPin from line 270-294
    static void setPin(uint8_t num, uint16_t val, bool invert = false) {
        if (val > 4095) val = 4095;
        if (invert) {
            if (val == 0) {
                setPWM(num, 4096, 0);
            } else if (val == 4095) {
                setPWM(num, 0, 4096);
            } else {
                setPWM(num, 0, 4095 - val);
            }
        } else {
            if (val == 4095) {
                setPWM(num, 4096, 0);
            } else if (val == 0) {
                setPWM(num, 0, 4096);
            } else {
                setPWM(num, 0, val);
            }
        }
    }

    // Exact Adafruit setOutputMode from line 189-204
    static void setOutputMode(bool totempole) {
        uint8_t oldmode = read8(PCA9685_MODE2);
        uint8_t newmode;
        if (totempole) {
            newmode = oldmode | MODE2_OUTDRV;
        } else {
            newmode = oldmode & ~MODE2_OUTDRV;
        }
        write8(PCA9685_MODE2, newmode);
    }

    // Exact Adafruit begin from line 66-85
    static bool begin(uint8_t prescale = 0) {
        reset();
        _oscillator_freq = FREQUENCY_OSCILLATOR;
        
        if (prescale) {
            // setExtClk not implemented - use internal
        }
        setPWMFreq(1000);
        return true;
    }

    void init(uint8_t i2c_sda, uint8_t i2c_scl, uint32_t freq_hz) {
        ESP_LOGI(TAG, "Adafruit exact port init SDA:%d SCL:%d", i2c_sda, i2c_scl);
        
        if (i2c_mutex == NULL) {
            i2c_mutex = xSemaphoreCreateMutex();
        }
        
        // I2C driver init
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)i2c_sda;
        conf.scl_io_num = (gpio_num_t)i2c_scl;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 100000;
        
        ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
        vTaskDelay(pdMS_TO_TICKS(10));

        // Exact Adafruit begin() sequence
        begin(0);
        
        // Set PWM frequency
        setPWMFreq(freq_hz);
        
        // Set output mode (optional, Adafruit doesn't do this in begin)
        setOutputMode(true);
        
        // Clear all channels
        ESP_LOGI(TAG, "Clearing all 16 channels");
        for (int i = 0; i < 16; i++) {
            setPin(i, 0);
        }
        
        memset(section_states, 0, sizeof(section_states));
        memset(pending_states, 0, sizeof(pending_states));
        initialized = true;
        
        // Test all 16 channels
        ESP_LOGI(TAG, "Test: channels 0-15 ON");
        for (int i = 0; i < 16; i++) {
            uint8_t result = setPWM(i, 4096, 0);
            
            // Read back ON_H register to verify write
            uint8_t reg = PCA9685_LED0_ON_L + 4 * i + 1; // ON_H
            uint8_t readback = read8(reg);
            ESP_LOGI(TAG, "Ch %d: result=%d, ON_H readback=0x%02X (expect 0x10)", i, result, readback);
            
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        for (int i = 0; i < 16; i++) {
            setPin(i, 0);
        }
        ESP_LOGI(TAG, "Init complete");
    }
    
    void set_section_state(uint8_t index, bool active) {
        if (!initialized || index >= NUM_SECTIONS) return;
        pending_states[index] = active;
    }
    
    void flush() {
        if (!initialized) return;
        
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        
        for (int i = 0; i < NUM_SECTIONS; i++) {
            if (section_states[i] != pending_states[i]) {
                setPin(i, pending_states[i] ? 4095 : 0);
                section_states[i] = pending_states[i];
            }
        }
        
        xSemaphoreGive(i2c_mutex);
    }
    
} // namespace PCA9685Handler

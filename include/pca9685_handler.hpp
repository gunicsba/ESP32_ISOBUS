#pragma once

#include <cstdint>

namespace PCA9685Handler {

/// Initializes the PCA9685 and sets up the I2C bus.
void init(uint8_t i2c_sda = 8, uint8_t i2c_scl = 18, uint32_t freq_hz = 1000);

/// Sets a section LED on or off based on working state.
/// @param index 0–15 (PCA9685 has 16 channels)
/// @param active true = on, false = off
void set_section_state(uint8_t index, bool active);

}
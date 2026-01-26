//================================================================================================
/// @file main.cpp
///
/// @brief Defines `main` for the seeder example
/// @details This example is meant to use all the major protocols in a more "complete" application.
/// @author Adrian Del Grosso
///
/// @copyright 2023 The Open-Agriculture Developers
//================================================================================================
 #include "freertos/FreeRTOS.h"
 #include "freertos/timers.h"
 //#include "objectPoolObjects.h"
 #include "seeder.hpp"
 
 #include <functional>
 #include <iostream>
 #include <memory>
 #include <driver/twai.h>
 #include "pca9685_handler.hpp"

 #include "esp_log.h"

 extern "C" void app_main()
 {
	vTaskDelay(pdMS_TO_TICKS(1000));
	ESP_LOGI("MAIN", "Application started!");
	ESP_LOGI("MAIN", "Initializing PCA9685...");
	PCA9685Handler::init(8, 18); // Explicitly passing pins for clarity
	vTaskDelay(pdMS_TO_TICKS(500));
	ESP_LOGI("MAIN", "Initializing Seeder...");
	Seeder seederExample;
	seederExample.initialize();
	ESP_LOGI("MAIN", "Seeder initialized!");
	
 
 	while (true)
 	{
 		// CAN stack runs in other threads. Do nothing forever.
 		seederExample.update();
		ESP_LOGI("MAIN", "Heartbeat...");
 		vTaskDelay(pdMS_TO_TICKS(1000));
 	}
 }
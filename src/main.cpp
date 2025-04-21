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

 extern "C" void app_main()
 {
	vTaskDelay(1500);
	Seeder seederExample;
 	seederExample.initialize();
 
 	while (true)
 	{
 		// CAN stack runs in other threads. Do nothing forever.
 		seederExample.update();
 		vTaskDelay(10);
 	}
 }
/*
 * ============================================================================
 *  espBrideIdf - Main 
 * ============================================================================
 *  2025 Leon Angele, Burger Engineering
 *  Created: 25.09.2025
 * ============================================================================
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../include/app_cycle.hpp"
#include "../include/trajectory_buffer.hpp"
#include "esp_timer.h"

#define CylclesRecord 5001 // Zyklusanzahl



// ESP32 Entry Point
extern "C" void app_main(void) {
    TrajectoryFS::setup();
    AppCycle::setup(); // Initialisierung aller Komponenten

    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1s warten bis alles stabil ist


    AppCycle::run(CylclesRecord); // Zyklische Verarbeitung (StateRequest, ML, Action)



    printf("Main loop finished, dumping trajectory buffer...\n");
    TrajectoryFS::dump_file();
    TrajectoryFS::format(); 

}

/*
 * ============================================================================
 *  espBrideIdf - Main 
 * ============================================================================
 *  2025 Leon Angele, Burger Engineering
 *  Created: 25.09.2025
 * ============================================================================
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "main_functions.hpp"

#define CycleTimeUs 1000 // 0.5ms Zykluszeit

// ESP32 Entry Point
extern "C" void app_main(void) {
    setup(); // Initialisierung aller Komponenten

    int cycle_count = 0;
    while (true) {
        uint64_t start = esp_timer_get_time();
        loop(); // Zyklische Verarbeitung (StateRequest, ML, Action)
        cycle_count++;
        if (cycle_count >= 10000) {
            print_trajectory_buffer();
            trajectory_buffer.clear();
            cycle_count = 0;

        }
        while ((esp_timer_get_time() - start) < CycleTimeUs) {
            vTaskDelay(0); // CPU-Entlastung, Task bleibt responsive
        }
    }
}

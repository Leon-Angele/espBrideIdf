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
#include "app_cycle.hpp"
#include "trajectory_buffer.hpp"
#include "esp_timer.h"

#define CycleTimeUs 1000 // 0.5ms Zykluszeit

// ESP32 Entry Point
extern "C" void app_main(void) {
    AppCycle::setup(); // Initialisierung aller Komponenten

    int cycle_count = 0;
    while (true) {
        uint64_t start = esp_timer_get_time();
        AppCycle::run(1000); // Zyklische Verarbeitung (StateRequest, ML, Action)
        cycle_count++;
        if (cycle_count >= 1000) {
            TrajectoryBuffer::print();
            TrajectoryBuffer::clear();
            cycle_count = 0;
        }
        while ((esp_timer_get_time() - start) < CycleTimeUs) {
            vTaskDelay(0); // CPU-Entlastung, Task bleibt responsive
        }
    }
}

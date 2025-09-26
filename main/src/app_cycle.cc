
// app_cycle.cc
#include "app_cycle.hpp"
#include "esp_timer.h"

#define ACTION_DELAY_US 500 // 0.5ms 
#define CYCLE_DELAY_US 5000   // 5ms


namespace AppCycle {

void setup() {
    MLInference::setup();
    spi_master_init();
    TrajectoryBuffer::clear();
}

void run(uint32_t cycles) {

    for (uint32_t i = 0; i < cycles; ++i) {
    uint64_t start_time_cycle = esp_timer_get_time();


    uint32_t slave_counter = 0;
    float sensor_value1 = 0.0f, sensor_value2 = 0.0f, sensor_value3 = 0.0f;

    esp_err_t state_result = spi_send_state_request(&slave_counter, &sensor_value1, &sensor_value2, &sensor_value3);

    if (state_result == ESP_OK) {
        uint64_t start_time = esp_timer_get_time();
        float action = MLInference::infer(sensor_value1);

        while ((esp_timer_get_time() - start_time) < ACTION_DELAY_US) {
            vTaskDelay(0);
        }
        spi_send_action_command(prep_sendFloat(action));
        TrajectoryBuffer::add(slave_counter, sensor_value1, sensor_value2, sensor_value3, action);
    }
    while ((esp_timer_get_time() - start_time_cycle) < CYCLE_DELAY_US) {
            vTaskDelay(0);
            }

    }




}//run
}// namespace AppCycle
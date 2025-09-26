#include "esp_timer.h"

#ifndef TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_MAIN_FUNCTIONS_H_
#define TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_MAIN_FUNCTIONS_H_

#include <vector>
struct TrajectoryEntry {
	uint32_t slave_counter;
	float value1;
	float value2;
	float value3;
	float action;
};
extern std::vector<TrajectoryEntry> trajectory_buffer;


// Expose a C friendly interface for main functions.
#ifdef __cplusplus
extern "C" {
#endif



void print_trajectory_buffer();

// Initializes all data needed for the example. The name is important, and needs
// to be setup() for Arduino compatibility.
void setup();

// Runs one iteration of data gathering and inference. This should be called
// repeatedly from the application code. The name needs to be loop() for Arduino
// compatibility.
// NOTE: This is now deprecated in favor of hardware timer callbacks
void loop();

// Execute TensorFlow Lite inference using sensor data
// Returns the ML inference result as float
float execute_ml_inference(float sensor_input);

#ifdef __cplusplus
}
#endif

#endif  // TENSORFLOW_LITE_MICRO_EXAMPLES_HELLO_WORLD_MAIN_FUNCTIONS_H_

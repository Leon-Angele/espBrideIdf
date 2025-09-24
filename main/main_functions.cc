/* Copyright 2020 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/


#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "main_functions.h"
#include "model.h"
#include "constants.h"
#include "output_handler.h"
#include "spi_interface.h"
#include "spi_test.h"

// Globals, used for compatibility with Arduino-style sketches.
namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
int inference_count = 0;

constexpr int kTensorArenaSize = 2000;
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

// The name of this function is important for Arduino compatibility.
void setup() {
  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported "
                "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  static tflite::MicroMutableOpResolver<1> resolver;
  if (resolver.AddFullyConnected() != kTfLiteOk) {
    return;
  }

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  // Keep track of how many inferences we have performed.
  inference_count = 0;
  
  // Run SPI self-tests first
  MicroPrintf("Running SPI self-tests...");
  if (spi_run_self_tests()) {
    MicroPrintf("SPI self-tests completed successfully");
  } else {
    MicroPrintf("SPI self-tests failed - check implementation");
  }
  
  // Initialize SPI Master interface
  esp_err_t spi_ret = spi_master_init();
  if (spi_ret != ESP_OK) {
    MicroPrintf("SPI Master initialization failed: %s", esp_err_to_name(spi_ret));
    // Continue without SPI (optional fallback)
  } else {
    MicroPrintf("SPI Master ready for full-duplex communication");
    
    // Optional: Run hardware loopback test if connections are available
    MicroPrintf("To test hardware: connect GPIO2 (MISO) to GPIO7 (MOSI)");
  }
}

// The name of this function is important for Arduino compatibility.
void loop() {
  // --- STEP 1: StateRequest → Get sensor data from slave ---
  uint32_t slave_counter = 0;
  float sensor_value1 = 0.0f, sensor_value2 = 0.0f, sensor_value3 = 0.0f;
  
  esp_err_t state_result = spi_send_state_request(&slave_counter, &sensor_value1, &sensor_value2, &sensor_value3);
  
  if (state_result == ESP_OK) {
    MicroPrintf("StateRequest OK - Counter: %lu, Sensors: [%.3f, %.3f, %.3f]",
                (unsigned long)slave_counter,
                static_cast<double>(sensor_value1),
                static_cast<double>(sensor_value2),
                static_cast<double>(sensor_value3));
    
    // --- Wait exactly 2ms as requested (StateRequest → 2ms → Action) ---
    vTaskDelay(pdMS_TO_TICKS(2));
    
    // --- STEP 2: TensorFlow Lite Inference using sensor data ---
    // Use first sensor value as ML model input
    float x = sensor_value1;
    
    // Quantize the input from floating-point to integer
    int8_t x_quantized = x / input->params.scale + input->params.zero_point;
    // Place the quantized input in the model's input tensor
    input->data.int8[0] = x_quantized;

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      MicroPrintf("Invoke failed on x: %f\n", static_cast<double>(x));
      return;
    }

    // Obtain the quantized output from model's output tensor
    int8_t y_quantized = output->data.int8[0];
    // Dequantize the output from integer to floating-point
    float y = (y_quantized - output->params.zero_point) * output->params.scale;
    
    // --- STEP 3: Action Command → Send ML inference results ---
    // Convert ML result to 32-bit action value (scale -1..1 to 0..2000)
    uint32_t action_value = static_cast<uint32_t>((y + 1.0f) * 1000.0f);
    
    esp_err_t action_result = spi_send_action_command(action_value);
    
    if (action_result == ESP_OK) {
      MicroPrintf("Action OK - ML result: %.3f → Action: 0x%08lX sent to slave",
                  static_cast<double>(y), (unsigned long)action_value);
      
      // Output the results using the original handler
      HandleOutput(x, y);
      
    } else {
      MicroPrintf("Action FAILED");
    }
    
  } else {
    MicroPrintf("StateRequest FAILED");
  }

  // Increment the inference_counter, and reset it if we have reached
  // the total number per cycle
  inference_count += 1;
  if (inference_count >= kInferencesPerCycle) {
    inference_count = 0;
    
    // Print SPI statistics every cycle
    uint32_t total_transfers, error_count;
    spi_get_stats(&total_transfers, &error_count);
    MicroPrintf("SPI Stats: %lu transfers, %lu errors",
                (unsigned long)total_transfers,
                (unsigned long)error_count);
  }
}

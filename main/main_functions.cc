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
#include "esp_timer.h"

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
  
  // SPI self-tests entfernt, da nicht mehr nötig
  
  // Initialize SPI Master interface
  esp_err_t spi_ret = spi_master_init();
  if (spi_ret != ESP_OK) {
    MicroPrintf("SPI Master initialization failed: %s", esp_err_to_name(spi_ret));
    // Continue without SPI (optional fallback)
  } else {
    MicroPrintf("SPI Master ready for full-duplex communication");
  }
}

// The name of this function is important for Arduino compatibility.
void loop() {
  // --- STEP 1: StateRequest → Get sensor data from slave ---
  uint32_t slave_counter = 0;
  float sensor_value1 = 0.0f, sensor_value2 = 0.0f, sensor_value3 = 0.0f;
  
  esp_err_t state_result = spi_send_state_request(&slave_counter, &sensor_value1, &sensor_value2, &sensor_value3);
  
  if (state_result == ESP_OK) {
    // --- ML-Inferenz während exakt 2ms Delay ---
    uint64_t start_time = esp_timer_get_time();
    float ml_result = 0.0f;

    // ML-Inferenz ausführen (z.B. mit sensor_value1 als Input)

    sensor_value1 = 3.14159/2.0f; // Beispielwert
    ml_result = execute_ml_inference(sensor_value1);
    printf("ML Result: %.3f\n", ml_result);
    
    // Falls Inferenz schneller als 2ms ist, restliche Zeit warten
    while ((esp_timer_get_time() - start_time) < 2000) {
      vTaskDelay(0); // Yield to other tasks but maintain timing precision
    }

    // --- STEP 2: Action Command → ML-Ergebnis senden ---
    uint32_t action_value = static_cast<uint32_t>((ml_result + 1.0f) * 1000.0f);


    esp_err_t action_result = spi_send_action_command(action_value);

    if (action_result != ESP_OK) {
      MicroPrintf("Action FAILED");
    }
  }

  // Increment the inference_counter, and reset it if we have reached
  // the total number per cycle
  inference_count += 1;
  if (inference_count >= kInferencesPerCycle) {
    inference_count = 0;
  }
}
// ML-Inferenz-Funktion für Busy-Wait Integration
float execute_ml_inference(float sensor_input) {
  if (input == nullptr || output == nullptr) {
    return 0.0f;
  }
  float x = sensor_input;
  
  int8_t x_quantized = x / input->params.scale + input->params.zero_point;
  input->data.int8[0] = x_quantized;
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    return 0.0f;
  }
  int8_t y_quantized = output->data.int8[0];
  float y = (y_quantized - output->params.zero_point) * output->params.scale;
  return y;
}

/*
 * ============================================================================
 *  espBrideIdf - Main Functions
 * ============================================================================
 *  2025 Leon Angele, Burger Engineering
 *  Created: 25.09.2025
 * ============================================================================
 */

/* === INCLUDES === */
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"


#include "main_functions.hpp"
#include "model.h"
#include "constants.h"

#include <vector>
#include "output_handler.h"
#include "spi_interface.h"

/* === DEFINES & CONSTANTS === */
#define ACTION_DELAY_US 0 // 0.5ms precise delay for Action command timing


/* === GLOBALS === */
// Dynamischer Trajectory Buffer (jetzt global)
std::vector<TrajectoryEntry> trajectory_buffer;

// Globale Variablen für ML und SPI
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
int inference_count = 0;
constexpr int kTensorArenaSize = 2000;
uint8_t tensor_arena[kTensorArenaSize];



// Hilfsfunktion: Trajectory Buffer printen
void print_trajectory_buffer() {
   printf("Trajectory Buffer (Size: %zu):\n", trajectory_buffer.size());
   for (size_t i = 0; i < trajectory_buffer.size(); ++i) {
      const auto& entry = trajectory_buffer[i];
      printf("[%lu, %.3f, %.3f, %.3f, %.3f]\n", entry.slave_counter, entry.value1, entry.value2, entry.value3, entry.action);
   }
}

// The name of this function is important for Arduino compatibility.
/* === INITIALISIERUNG === */
void setup() {
   // ML-Modell initialisieren
   model = tflite::GetModel(g_model);
   if (model->version() != TFLITE_SCHEMA_VERSION) return;

   static tflite::MicroMutableOpResolver<1> resolver;
   if (resolver.AddFullyConnected() != kTfLiteOk) return;

   static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
   interpreter = &static_interpreter;
   if (interpreter->AllocateTensors() != kTfLiteOk) return;

   input = interpreter->input(0);
   output = interpreter->output(0);
   inference_count = 0;

   // SPI Master initialisieren
   spi_master_init();
}

// The name of this function is important for Arduino compatibility.
/* === HAUPTZYKLUS === */
void loop() {
   // 1. StateRequest: Sensordaten vom Slave holen
   uint32_t slave_counter = 0;
   float sensor_value1 = 0.0f, sensor_value2 = 0.0f, sensor_value3 = 0.0f;
   esp_err_t state_result = spi_send_state_request(&slave_counter, &sensor_value1, &sensor_value2, &sensor_value3);
   if (state_result == ESP_OK) {
      // 2. ML-Inferenz während exakt 2ms Delay
      uint64_t start_time = esp_timer_get_time();
      float action = execute_ml_inference(sensor_value1);

      // Falls Inferenz schneller als 2ms ist, restliche Zeit warten
      while ((esp_timer_get_time() - start_time) < ACTION_DELAY_US) {
         vTaskDelay(0); // CPU-Entlastung, Task bleibt responsive
      }

      // 3. Action: ML-Ergebnis an Slave senden
      spi_send_action_command(prep_sendFloat(action)); // send float bits in Big Endian

      // 4. Trajectory Buffer: Werte speichern
      trajectory_buffer.push_back({slave_counter, sensor_value1, sensor_value2, sensor_value3, action});
   }

   // Inferenzzähler für Zyklusverwaltung
   inference_count += 1;
   if (inference_count >= kInferencesPerCycle) inference_count = 0;
}











// ML-Inferenz-Funktion für Busy-Wait Integration
/* === ML-INFERENZ === */
float execute_ml_inference(float sensor_input) {
   if (input == nullptr || output == nullptr) return 0.0f;

   // Quantisierung des Inputs
   int8_t x_quantized = sensor_input / input->params.scale + input->params.zero_point;
   input->data.int8[0] = x_quantized;

   // Inferenz ausführen
   TfLiteStatus invoke_status = interpreter->Invoke();
   if (invoke_status != kTfLiteOk) return -1.0f;

   // Dequantisierung des Outputs
   int8_t y_quantized = output->data.int8[0];
   return (y_quantized - output->params.zero_point) * output->params.scale;
}

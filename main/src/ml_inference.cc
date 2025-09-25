// ml_inference.cc
#include "ml_inference.hpp"
#include "../model.h"
#include "../constants.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input = nullptr;
    TfLiteTensor* output = nullptr;
    constexpr int kTensorArenaSize = 2000;
    uint8_t tensor_arena[kTensorArenaSize];
}

namespace MLInference {

void setup() {
    model = tflite::GetModel(g_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) return;
    static tflite::MicroMutableOpResolver<1> resolver;
    if (resolver.AddFullyConnected() != kTfLiteOk) return;
    static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;
    if (interpreter->AllocateTensors() != kTfLiteOk) return;
    input = interpreter->input(0);
    output = interpreter->output(0);
}

float infer(float sensor_input) {
    if (input == nullptr || output == nullptr) return 0.0f;
    int8_t x_quantized = sensor_input / input->params.scale + input->params.zero_point;
    input->data.int8[0] = x_quantized;
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) return -1.0f;
    int8_t y_quantized = output->data.int8[0];
    return (y_quantized - output->params.zero_point) * output->params.scale;
}

}
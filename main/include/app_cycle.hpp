// app_cycle.hpp
#pragma once
#include "trajectory_buffer.hpp"
#include "ml_inference.hpp"
#include "spi_interface.h"


namespace AppCycle {
    void setup();
    void run(uint32_t cycles);
}
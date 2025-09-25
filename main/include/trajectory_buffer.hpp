// trajectory_buffer.hpp

#pragma once
#include <cstddef>
#include <cstdint>

namespace TrajectoryBuffer {
    struct Entry {
        uint32_t slave_counter;
        float value1;
        float value2;
        float value3;
        float action;
    };

    void add(uint32_t slave_counter, float value1, float value2, float value3, float action);
    void print();
    void clear();
    size_t size();
}
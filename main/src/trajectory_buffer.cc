// trajectory_buffer.cc
#include "trajectory_buffer.hpp"
#include <vector>
#include <cstdio>

namespace {
    std::vector<TrajectoryBuffer::Entry> buffer;
}

namespace TrajectoryBuffer {

void add(uint32_t slave_counter, float value1, float value2, float value3, float action) {
    buffer.push_back({slave_counter, value1, value2, value3, action});
}

void print() {
    printf("Trajectory Buffer (Size: %zu):\n", buffer.size());
    for (size_t i = 0; i < buffer.size(); ++i) {
        const auto& entry = buffer[i];
        printf("[%lu, %.3f, %.3f, %.3f, %.3f]\n", entry.slave_counter, entry.value1, entry.value2, entry.value3, entry.action);
    }
}

void clear() {
    buffer.clear();
}

size_t size() {
    return buffer.size();
}

}
#pragma once
#include <cstddef>
#include <cstdint>

// Schwellenwert für automatisches Flash-Flush
#define TRAJECTORY_BUFFER_MAX_SIZE 1000 

//
// === Buffer Namespace ===
//
namespace TrajectoryBuffer {
    struct Entry {
        uint32_t slave_counter;
        float value1;
        float value2;
        float value3;
        float action;
    };

    void add(uint32_t slave_counter, float value1, float value2, float value3, float action);
    void flush_to_flash();
    void load_from_flash();
    void print();
    void clear();
    size_t size();
}

//
// === SPIFFS Namespace ===
//
namespace TrajectoryFS {
    // Mountet SPIFFS (einmalig beim Start aufrufen)
    void setup();

    // Löscht alle Dateien in der SPIFFS Partition
    void format();

    // Dump der SPIFFS-Datei
    void dump_file();
}

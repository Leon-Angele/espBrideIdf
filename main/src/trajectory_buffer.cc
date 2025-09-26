#include <cstring>
#include "esp_spiffs.h"
#include "esp_log.h"
#include <vector>
#include <cstdio>
#include <dirent.h>
#include <unistd.h>  // for unlink()

#include "trajectory_buffer.hpp"

namespace {
    std::vector<TrajectoryBuffer::Entry> buffer;
}

//
// === Buffer Namespace ===
//
namespace TrajectoryBuffer {

void add(uint32_t slave_counter, float value1, float value2, float value3, float action) {
    buffer.push_back({slave_counter, value1, value2, value3, action});
    if (buffer.size() >= TRAJECTORY_BUFFER_MAX_SIZE) {
        flush_to_flash();
        buffer.clear();
    }
}

void flush_to_flash() {
    FILE* f = fopen("/spiffs/trajectory_buffer.bin", "ab");
    if (f) {
        for (const auto& entry : buffer) {
            fwrite(&entry, sizeof(entry), 1, f);
        }
        fclose(f);
        ESP_LOGI("TrajectoryBuffer", "Buffer flushed to flash (%zu entries)", buffer.size());
    } else {
        ESP_LOGE("TrajectoryBuffer", "Failed to open file for writing");
    }
}

void load_from_flash() {
    FILE* f = fopen("/spiffs/trajectory_buffer.bin", "rb");
    if (f) {
        buffer.clear();
        TrajectoryBuffer::Entry entry;
        while (fread(&entry, sizeof(entry), 1, f) == 1) {
            buffer.push_back(entry);
        }
        fclose(f);
        ESP_LOGI("TrajectoryBuffer", "Buffer loaded from flash (%zu entries)", buffer.size());
    } else {
        ESP_LOGE("TrajectoryBuffer", "Failed to open file for reading");
    }
}

void print() {
    printf("Trajectory Buffer (Size: %zu):\n", buffer.size());
    for (size_t i = 0; i < buffer.size(); ++i) {
        const auto& entry = buffer[i];
        printf("[%lu, %.3f, %.3f, %.3f, %.3f]\n",
               entry.slave_counter, entry.value1, entry.value2, entry.value3, entry.action);
    }
}

void clear() {
    buffer.clear();
}

size_t size() {
    return buffer.size();
}

} // namespace TrajectoryBuffer


//
// === SPIFFS Namespace ===
//
namespace TrajectoryFS {

static const char *TAG = "TrajectoryFS";

void setup()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    // Infos zur Partition abfragen
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted, total=%d, used=%d", total, used);
    }
}

void format()
{
    const char* base_path = "/spiffs";

    DIR* dir = opendir(base_path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open SPIFFS directory for formatting");
        return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        char filepath[512];
        snprintf(filepath, sizeof(filepath), "%s/%s", base_path, entry->d_name);

        if (unlink(filepath) == 0) {
            ESP_LOGI(TAG, "Deleted: %s", filepath);
        } else {
            ESP_LOGE(TAG, "Failed to delete: %s", filepath);
        }
    }

    closedir(dir);
}
    
void dump_file() {
    const char* path = "/spiffs/trajectory_buffer.bin";
    FILE* f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGE("TrajectoryFS", "Failed to open file %s for reading", path);
        return;
    }

    TrajectoryBuffer::Entry entry;
    while (fread(&entry, sizeof(entry), 1, f) == 1) {
        printf("%lu,%.3f,%.3f,%.3f,%.3f\n",
               entry.slave_counter,
               entry.value1, entry.value2, entry.value3, entry.action);
    }
    fclose(f);
    ESP_LOGI("TrajectoryFS", "Dump of %s finished", path);
}



} // namespace TrajectoryFS

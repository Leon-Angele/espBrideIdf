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

#include "spi_test.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char* TAG = "SPI_TEST";

bool spi_run_self_tests(void) {
    ESP_LOGI(TAG, "=== Running SPI Self-Tests ===");
    
    bool all_passed = true;
    
    // Test 1: Union Layout
    ESP_LOGI(TAG, "Test 1: Union memory layout...");
    if (spi_test_union_layout()) {
        ESP_LOGI(TAG, "✓ Union layout test PASSED");
    } else {
        ESP_LOGE(TAG, "✗ Union layout test FAILED");
        all_passed = false;
    }
    
    // Test 2: Packet Validation
    ESP_LOGI(TAG, "Test 2: Packet validation...");
    if (spi_test_packet_validation()) {
        ESP_LOGI(TAG, "✓ Packet validation test PASSED");
    } else {
        ESP_LOGE(TAG, "✗ Packet validation test FAILED");
        all_passed = false;
    }
    
    // Test 3: Performance Benchmark
    ESP_LOGI(TAG, "Test 3: Performance benchmark...");
    if (spi_test_performance()) {
        ESP_LOGI(TAG, "✓ Performance test PASSED");
    } else {
        ESP_LOGE(TAG, "✗ Performance test FAILED");
        all_passed = false;
    }
    
    ESP_LOGI(TAG, "=== SPI Self-Tests %s ===",
                all_passed ? "COMPLETED SUCCESSFULLY" : "FAILED");
    
    return all_passed;
}

bool spi_test_union_layout(void) {
    spi_data_packet_t test_packet = {0};
    
    // Test 1: Size verification
    if (sizeof(spi_data_packet_t) != SPI_PACKET_SIZE_BYTES) {
        ESP_LOGE(TAG, "Size mismatch: expected %d, got %d",
                    SPI_PACKET_SIZE_BYTES, (int)sizeof(spi_data_packet_t));
        return false;
    }
    
    // Test 2: Field access vs. word access
    test_packet.fields.status_word = 0x1234;
    test_packet.fields.counter = 0xABCDEF01;
    test_packet.fields.data_1 = 3.14159f;
    test_packet.fields.data_2 = -2.71828f;
    test_packet.fields.data_3 = 1.41421f;
    
    // Verify status word
    if (test_packet.words[0] != 0x1234) {
        ESP_LOGE(TAG, "Status word mismatch: expected 0x1234, got 0x%04X",
                    test_packet.words[0]);
        return false;
    }
    
    // Verify counter (32-bit value across 2 words)
    uint32_t reconstructed_counter = ((uint32_t)test_packet.words[2] << 16) | 
                                     (uint32_t)test_packet.words[1];
    if (reconstructed_counter != 0xABCDEF01) {
        ESP_LOGE(TAG, "Counter mismatch: expected 0xABCDEF01, got 0x%08lX",
                    (unsigned long)reconstructed_counter);
        return false;
    }
    
    // Test 3: Float to words conversion
    union {
        float f;
        uint16_t words[2];
    } float_test;
    
    float_test.f = test_packet.fields.data_1;
    if (float_test.words[0] != test_packet.words[3] ||
        float_test.words[1] != test_packet.words[4]) {
        ESP_LOGE(TAG, "Float 1 word conversion failed");
        return false;
    }
    
    // Test 4: Byte array access
    uint8_t expected_status_bytes[2] = {0x34, 0x12}; // Little-endian
    if (test_packet.bytes[0] != expected_status_bytes[0] ||
        test_packet.bytes[1] != expected_status_bytes[1]) {
        ESP_LOGE(TAG, "Byte array access failed for status word");
        return false;
    }
    
    ESP_LOGI(TAG, "Union layout verification successful:");
    ESP_LOGI(TAG, "  Total size: %d bytes", (int)sizeof(spi_data_packet_t));
    ESP_LOGI(TAG, "  Status word: 0x%04X", test_packet.fields.status_word);
    ESP_LOGI(TAG, "  Counter: 0x%08lX", (unsigned long)test_packet.fields.counter);
    ESP_LOGI(TAG, "  Data values: %f, %f, %f",
                (double)test_packet.fields.data_1,
                (double)test_packet.fields.data_2,
                (double)test_packet.fields.data_3);
    
    return true;
}

bool spi_test_packet_validation(void) {
    spi_data_packet_t test_packet;
    
    // Test 1: Valid packet
    memset(&test_packet, 0, sizeof(test_packet));
    test_packet.fields.status_word = STATUS_READY | STATUS_DATA_VALID;
    test_packet.fields.counter = 123;
    test_packet.fields.data_1 = 1.0f;
    test_packet.fields.data_2 = 2.0f;
    test_packet.fields.data_3 = 3.0f;
    
    if (!spi_validate_packet(&test_packet)) {
        ESP_LOGE(TAG, "Valid packet rejected");
        return false;
    }
    
    // Test 2: Invalid status word (all zeros)
    test_packet.fields.status_word = 0x0000;
    if (spi_validate_packet(&test_packet)) {
        ESP_LOGE(TAG, "Invalid status word (0x0000) accepted");
        return false;
    }
    
    // Test 3: Invalid status word (all ones)
    test_packet.fields.status_word = 0xFFFF;
    if (spi_validate_packet(&test_packet)) {
        ESP_LOGE(TAG, "Invalid status word (0xFFFF) accepted");
        return false;
    }
    
    // Test 4: NaN values
    test_packet.fields.status_word = STATUS_READY;
    test_packet.fields.data_1 = NAN;
    if (spi_validate_packet(&test_packet)) {
        ESP_LOGE(TAG, "NaN value accepted");
        return false;
    }
    
    // Test 5: Infinity values
    test_packet.fields.data_1 = 1.0f;
    test_packet.fields.data_2 = INFINITY;
    if (spi_validate_packet(&test_packet)) {
        ESP_LOGE(TAG, "Infinity value accepted");
        return false;
    }
    
    // Test 6: NULL pointer
    if (spi_validate_packet(NULL)) {
        ESP_LOGE(TAG, "NULL pointer accepted");
        return false;
    }
    
    ESP_LOGI(TAG, "Packet validation tests successful");
    return true;
}

bool spi_test_hardware_loopback(void) {
    ESP_LOGI(TAG, "Hardware loopback test requires MISO-MOSI connection");
    ESP_LOGI(TAG, "Connect GPIO2 (MISO) to GPIO7 (MOSI) for this test");
    
    // Initialize SPI if not already done
    esp_err_t ret = spi_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Prepare test packet with known pattern
    spi_data_packet_t tx_packet = {0};
    spi_data_packet_t rx_packet = {0};
    
    tx_packet.fields.status_word = 0xABCD;
    tx_packet.fields.counter = 0x12345678;
    tx_packet.fields.data_1 = 3.14159f;
    tx_packet.fields.data_2 = -2.71828f;
    tx_packet.fields.data_3 = 1.41421f;
    
    ESP_LOGI(TAG, "Transmitting test pattern...");
    spi_dump_packet(&tx_packet, "TX");
    
    // Perform SPI transaction
    ret = spi_full_duplex_exchange(&tx_packet, &rx_packet);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Received data:");
    spi_dump_packet(&rx_packet, "RX");
    
    // In loopback mode, TX and RX should be identical
    bool loopback_ok = true;
    for (int i = 0; i < SPI_PACKET_SIZE_WORDS; i++) {
        if (tx_packet.words[i] != rx_packet.words[i]) {
            ESP_LOGE(TAG, "Loopback mismatch at word %d: TX=0x%04X, RX=0x%04X",
                        i, tx_packet.words[i], rx_packet.words[i]);
            loopback_ok = false;
        }
    }
    
    if (loopback_ok) {
        ESP_LOGI(TAG, "✓ Hardware loopback test successful");
        return true;
    } else {
        ESP_LOGE(TAG, "✗ Hardware loopback test failed - check connections");
        return false;
    }
}

bool spi_test_performance(void) {
    ESP_LOGI(TAG, "Running performance benchmark...");
    
    const int num_transactions = 100;
    spi_data_packet_t tx_packet = {0};
    spi_data_packet_t rx_packet = {0};
    
    // Prepare test packet
    tx_packet.fields.status_word = spi_build_status_word();
    tx_packet.fields.counter = 0;
    tx_packet.fields.data_1 = 1.0f;
    tx_packet.fields.data_2 = 2.0f;
    tx_packet.fields.data_3 = 3.0f;
    
    // Initialize SPI
    esp_err_t ret = spi_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed");
        return false;
    }
    
    // Measure transaction time
    int64_t start_time = esp_timer_get_time();
    
    for (int i = 0; i < num_transactions; i++) {
        tx_packet.fields.counter = i;
        ret = spi_full_duplex_exchange(&tx_packet, &rx_packet);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Transaction %d failed", i);
            return false;
        }
    }
    
    int64_t end_time = esp_timer_get_time();
    int64_t total_time_us = end_time - start_time;
    int64_t avg_time_us = total_time_us / num_transactions;
    
    // Calculate theoretical vs. actual performance
    int64_t theoretical_time_us = (SPI_PACKET_SIZE_BYTES * 8 * 1000000) / SPI_MASTER_FREQ_HZ;
    float efficiency = (float)theoretical_time_us / (float)avg_time_us * 100.0f;
    
    ESP_LOGI(TAG, "Performance Results:");
    ESP_LOGI(TAG, "  Transactions: %d", num_transactions);
    ESP_LOGI(TAG, "  Total time: %lld us", total_time_us);
    ESP_LOGI(TAG, "  Average time: %lld us per transaction", avg_time_us);
    ESP_LOGI(TAG, "  Theoretical minimum: %lld us", theoretical_time_us);
    ESP_LOGI(TAG, "  Efficiency: %.1f%%", (double)efficiency);
    ESP_LOGI(TAG, "  Throughput: %.1f transactions/sec",
                1000000.0 / (double)avg_time_us);
    
    // Check if performance is reasonable (should be within 2x of theoretical)
    if (avg_time_us > (theoretical_time_us * 2)) {
        ESP_LOGE(TAG, "Performance below expectations");
        return false;
    }
    
    return true;
}
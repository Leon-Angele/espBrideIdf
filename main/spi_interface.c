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

#include "spi_interface.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

// Static Variables
static const char* TAG = "SPI_MASTER";
static spi_device_handle_t spi_device = NULL;
static bool spi_initialized = false;

// Statistics
static uint32_t total_transfers = 0;
static uint32_t error_count = 0;
static uint8_t sequence_counter = 0;

// Private function prototypes
static void spi_print_config(void);

esp_err_t spi_master_init(void) {
    if (spi_initialized) {
        return ESP_OK;
    }

    esp_err_t ret;

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_HZ,
        .mode = 0,                          // SPI Mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .flags = 0,                         // Full-duplex mode
        .pre_cb = NULL,
        .post_cb = NULL,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 2,              // Keep CS active 2 cycles before trans
        .cs_ena_posttrans = 2              // Keep CS active 2 cycles after trans
    };

    // Initialize SPI bus
    ret = spi_bus_initialize(SPI_MASTER_HOST, &bus_cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        return ret;
    }

    // Add device to SPI bus
    ret = spi_bus_add_device(SPI_MASTER_HOST, &dev_cfg, &spi_device);
    if (ret != ESP_OK) {
        spi_bus_free(SPI_MASTER_HOST);
        return ret;
    }

    // Reset statistics
    total_transfers = 0;
    error_count = 0;
    sequence_counter = 0;
    
    spi_initialized = true;
    return ESP_OK;
}

esp_err_t spi_master_deinit(void) {
    if (!spi_initialized) {
        return ESP_OK;
    }

    esp_err_t ret;

    // Remove device from SPI bus
    ret = spi_bus_remove_device(spi_device);

    // Free SPI bus
    ret = spi_bus_free(SPI_MASTER_HOST);

    spi_device = NULL;
    spi_initialized = false;
    
    return ret;
}

esp_err_t spi_full_duplex_exchange(spi_command_packet_t* tx_packet,
                                   spi_command_packet_t* rx_packet) {
    if (!spi_initialized || spi_device == NULL) {
        error_count++;
        return ESP_ERR_INVALID_STATE;
    }

    if (tx_packet == NULL || rx_packet == NULL) {
        error_count++;
        return ESP_ERR_INVALID_ARG;
    }

    // Clear receive buffer
    memset(rx_packet, 0, sizeof(spi_command_packet_t));

    // Convert TX packet from Little-Endian to Big-Endian for SPI transmission
    spi_convert_to_big_endian(tx_packet);

    // Prepare SPI transaction
    spi_transaction_t trans = {
        .length = SPI_PACKET_SIZE_BYTES * 8,    // Length in bits
        .tx_buffer = tx_packet->words,          // Use word array for direct access
        .rx_buffer = rx_packet->words,          // Use word array for direct access
        .user = NULL
    };

    // Perform full-duplex transaction
    esp_err_t ret = spi_device_transmit(spi_device, &trans);
    
    total_transfers++;
    
    if (ret != ESP_OK) {
        error_count++;
        // Convert TX packet back to Little-Endian (restore original state)
        spi_convert_from_big_endian(tx_packet);
        return ret;
    }

    // Convert received packet from Big-Endian back to Little-Endian
    spi_convert_from_big_endian(rx_packet);
    
    // Convert TX packet back to Little-Endian (restore original state)
    spi_convert_from_big_endian(tx_packet);

    // Optional: Validate received packet
    if (!spi_validate_packet(rx_packet)) {
        error_count++;
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t spi_send_state_request(uint32_t* counter, float* value1, float* value2, float* value3) {
    if (!spi_initialized || !counter || !value1 || !value2 || !value3) {
        return ESP_FAIL;
    }

    spi_command_packet_t tx_packet, rx_packet;
    
    // Initialize TX packet: [0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
    memset(&tx_packet, 0, sizeof(tx_packet));
    tx_packet.state_request_tx.command = SPI_CMD_STATE_REQUEST;
    
    // Perform SPI exchange
    esp_err_t ret = spi_full_duplex_exchange(&tx_packet, &rx_packet);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Validate status code
    if (!spi_validate_status_code(rx_packet.state_response_rx.status)) {
        return ESP_FAIL;
    }
    
    // Manual Big-Endian interpretation (slave sends High-Word first)
    // Counter: Words 1-2 as Big-Endian uint32
    *counter = ((uint32_t)rx_packet.words[1] << 16) | rx_packet.words[2];
    
    // Float 1: Words 3-4 as Big-Endian IEEE 754
    union { uint32_t i; float f; } conv1;
    conv1.i = ((uint32_t)rx_packet.words[3] << 16) | rx_packet.words[4];
    *value1 = conv1.f;
    
    // Float 2: Words 5-6 as Big-Endian IEEE 754
    union { uint32_t i; float f; } conv2;
    conv2.i = ((uint32_t)rx_packet.words[5] << 16) | rx_packet.words[6];
    *value2 = conv2.f;
    
    // Float 3: Words 7-8 as Big-Endian IEEE 754
    union { uint32_t i; float f; } conv3;
    conv3.i = ((uint32_t)rx_packet.words[7] << 16) | rx_packet.words[8];
    *value3 = conv3.f;
    
    return ESP_OK;
}

esp_err_t spi_send_action_command(uint32_t action_value) {
    if (!spi_initialized) {
        return ESP_FAIL;
    }

    spi_command_packet_t tx_packet, rx_packet;
    
    // Initialize TX packet: [0x0002, Act_H, Act_L, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
    memset(&tx_packet, 0, sizeof(tx_packet));
    tx_packet.action_command_tx.command = SPI_CMD_ACTION;
    tx_packet.action_command_tx.action_value = action_value;
    
    // Perform SPI exchange
    esp_err_t ret = spi_full_duplex_exchange(&tx_packet, &rx_packet);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Validate status code
    if (!spi_validate_status_code(rx_packet.action_response_rx.status)) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

uint16_t spi_build_status_word(void) {
    uint16_t status = STATUS_READY | STATUS_DATA_VALID;
    
    // Add sequence number (upper 8 bits)
    status |= ((uint16_t)(++sequence_counter) << 8) & STATUS_SEQUENCE_MASK;
    
    return status;
}

bool spi_validate_status_code(uint16_t status_code) {
    switch (status_code) {
        case SSC_RL_STATUS_OK:
            return true;
        case SSC_RL_STATUS_ERROR:
            return false;
        default:
            return false;
    }
}

const char* spi_get_status_string(uint16_t status_code) {
    switch (status_code) {
        case SSC_RL_STATUS_OK:
            return "Communication OK - expected sequence";
        case SSC_RL_STATUS_ERROR:
            return "Communication Error - unexpected sequence";
        default:
            return "Unknown status code";
    }
}

bool spi_validate_packet(const spi_command_packet_t* packet) {
    if (packet == NULL) {
        return false;
    }

    // Check if status word has valid flags
    uint16_t status = packet->fields.status_word;
    
    // At minimum, should have some status bits set
    if (status == 0x0000 || status == 0xFFFF) {
        return false;
    }

    // Check for basic data integrity (floats should not be NaN or infinity)
    if (isnan(packet->fields.data_1) || isinf(packet->fields.data_1) ||
        isnan(packet->fields.data_2) || isinf(packet->fields.data_2) ||
        isnan(packet->fields.data_3) || isinf(packet->fields.data_3)) {
        return false;
    }

    return true;
}

void spi_dump_packet(const spi_command_packet_t* packet, const char* label) {
    // Removed for brevity
}

void spi_get_stats(uint32_t* total_transfers_out, uint32_t* error_count_out) {
    if (total_transfers_out != NULL) {
        *total_transfers_out = total_transfers;
    }
    if (error_count_out != NULL) {
        *error_count_out = error_count;
    }
}

void spi_reset_stats(void) {
    total_transfers = 0;
    error_count = 0;
    sequence_counter = 0;
}

// Private Functions

static void spi_print_config(void) {
    // Removed for brevity
}

void spi_dump_command_packet(const spi_command_packet_t* packet, bool is_tx, const char* label) {
    // Removed for brevity
}

void spi_convert_to_big_endian(spi_command_packet_t* packet) {
    if (!packet) return;
    
    // Convert ESP32 Little-Endian to MSB first for slave
    // Swap bytes within each 16-bit word: 0x0001 -> 0x0100
    for (int i = 0; i < SPI_PACKET_SIZE_WORDS; i++) {
        uint16_t word = packet->words[i];
        packet->words[i] = __builtin_bswap16(word);
    }
}

void spi_convert_from_big_endian(spi_command_packet_t* packet) {
    if (!packet) return;
    
    // Convert MSB first from slave back to ESP32 Little-Endian
    // Swap bytes within each 16-bit word: received MSB -> Little-Endian
    for (int i = 0; i < SPI_PACKET_SIZE_WORDS; i++) {
        uint16_t word = packet->words[i];
        packet->words[i] = __builtin_bswap16(word);
    }
}
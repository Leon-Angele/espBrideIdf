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

#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// SPI Master Configuration
#define SPI_MASTER_HOST    SPI2_HOST
#define PIN_NUM_MISO       GPIO_NUM_2
#define PIN_NUM_MOSI       GPIO_NUM_7
#define PIN_NUM_CLK        GPIO_NUM_6
#define PIN_NUM_CS         GPIO_NUM_10

#define SPI_MASTER_FREQ_HZ 1000000  // 2MHz
#define SPI_PACKET_SIZE_BYTES 18    // 9 words * 2 bytes
#define SPI_PACKET_SIZE_WORDS 9

// Command Definitions
#define SPI_CMD_STATE_REQUEST   0x0001  // Request sensor state from slave
#define SPI_CMD_ACTION          0x0002  // Send action command to slave

// SSC Status Code Definitions
#define SSC_RL_STATUS_OK        0x00A0  // Communication OK - expected sequence
#define SSC_RL_STATUS_ERROR     0x00E0  // Communication Error - unexpected sequence

// Legacy Status Bit Definitions (for compatibility)
#define STATUS_DATA_VALID    0x0001
#define STATUS_ERROR         0x0002
#define STATUS_READY         0x0004
#define STATUS_COMMAND_MASK  0x00F0
#define STATUS_SEQUENCE_MASK 0xFF00

// Union-based SPI Command Packet (Extended for Variable Layouts)
typedef union {
    // Raw 16-bit Word Array for direct SPI transfer
    uint16_t words[SPI_PACKET_SIZE_WORDS];
    
    // StateRequest TX Layout: [CMD, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
    struct {
        uint16_t command;        // Word 0: SPI_CMD_STATE_REQUEST (0x0001)
        uint16_t unused[8];      // Words 1-8: Zero padding
    } __attribute__((packed)) state_request_tx;
    
    // StateRequest RX Layout: [Status, Cnt_H, Cnt_L, V1_H, V1_L, V2_H, V2_L, V3_H, V3_L]
    struct {
        uint16_t status;         // Word 0: SSC Status Code
        uint32_t counter;        // Words 1-2: Counter (Cnt_H, Cnt_L)
        float    value1;         // Words 3-4: Float Value 1 (V1_H, V1_L)
        float    value2;         // Words 5-6: Float Value 2 (V2_H, V2_L)
        float    value3;         // Words 7-8: Float Value 3 (V3_H, V3_L)
    } __attribute__((packed)) state_response_rx;
    
    // Action Command TX Layout: [CMD, Act_H, Act_L, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
    struct {
        uint16_t command;        // Word 0: SPI_CMD_ACTION (0x0002)
        uint32_t action_value;   // Words 1-2: Action Value (Act_H, Act_L)
        uint16_t unused[6];      // Words 3-8: Zero padding
    } __attribute__((packed)) action_command_tx;
    
    // Action Command RX Layout: [Status, Act_H, Act_L, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
    struct {
        uint16_t status;         // Word 0: SSC Status Code
        uint32_t action_value;   // Words 1-2: Echo of Action Value (Act_H, Act_L)
        uint16_t unused[6];      // Words 3-8: Zero padding
    } __attribute__((packed)) action_response_rx;
    
    // Legacy structured view (for compatibility)
    struct {
        uint16_t status_word;    // Word 0: Status/Command
        uint32_t counter;        // Word 1-2: 32-bit Counter
        float    data_1;         // Word 3-4: Float 1 (IEEE 754)
        float    data_2;         // Word 5-6: Float 2 (IEEE 754)
        float    data_3;         // Word 7-8: Float 3 (IEEE 754)
    } __attribute__((packed)) fields;
    
    // Raw byte array for debugging
    uint8_t bytes[SPI_PACKET_SIZE_BYTES];
} spi_command_packet_t;

// Type alias for backward compatibility
typedef spi_command_packet_t spi_data_packet_t;

// Function Prototypes

/**
 * @brief Initialize SPI Master interface
 * 
 * Configures SPI2 as master with 1MHz clock, Mode 0
 * Sets up GPIO pins and registers device
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t spi_master_init(void);

/**
 * @brief Deinitialize SPI Master interface
 * 
 * Removes device and frees SPI bus
 * 
 * @return ESP_OK on success
 */
esp_err_t spi_master_deinit(void);

/**
 * @brief Perform full-duplex SPI data exchange
 *
 * Simultaneously sends tx_packet and receives rx_packet
 * Both packets must be valid spi_command_packet_t structures
 *
 * @param tx_packet Pointer to packet to transmit
 * @param rx_packet Pointer to buffer for received packet
 * @return ESP_OK on success, error code on failure
 */
esp_err_t spi_full_duplex_exchange(spi_command_packet_t* tx_packet,
                                   spi_command_packet_t* rx_packet);

/**
 * @brief Send StateRequest command and receive sensor data
 *
 * TX: [0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
 * RX: [Status, Cnt_H, Cnt_L, V1_H, V1_L, V2_H, V2_L, V3_H, V3_L]
 *
 * @param counter Pointer to store received counter value
 * @param value1 Pointer to store received float value 1
 * @param value2 Pointer to store received float value 2
 * @param value3 Pointer to store received float value 3
 * @return ESP_OK on success, ESP_FAIL if status indicates error
 */
esp_err_t spi_send_state_request(uint32_t* counter, float* value1, float* value2, float* value3);

/**
 * @brief Send Action command with action value
 *
 * TX: [0x0002, Act_H, Act_L, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
 * RX: [Status, Slave_Data, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000]
 *
 * Note: Slave does NOT echo the action value. RX contains status + slave response data.
 *
 * @param action_value Action value to send (typically ML inference result)
 * @return ESP_OK on success, ESP_FAIL if status indicates error
 */
esp_err_t spi_send_action_command(uint32_t action_value);

/**
 * @brief Build status word with sequence number (legacy)
 *
 * Creates status word with READY and DATA_VALID flags
 * Automatically increments sequence number
 *
 * @return 16-bit status word
 */
uint16_t spi_build_status_word(void);

/**
 * @brief Validate SSC status code from slave response
 *
 * Checks if received status indicates successful communication
 *
 * @param status_code Status code from slave (SSC_RL_STATUS_*)
 * @return true if status is OK, false if error
 */
bool spi_validate_status_code(uint16_t status_code);

/**
 * @brief Get human-readable status string
 *
 * Converts SSC status code to descriptive string
 *
 * @param status_code Status code from slave
 * @return Pointer to status description string
 */
const char* spi_get_status_string(uint16_t status_code);

/**
 * @brief Validate received SPI packet (legacy)
 *
 * Checks status word flags and basic packet integrity
 *
 * @param packet Pointer to received packet
 * @return true if packet is valid, false otherwise
 */
bool spi_validate_packet(const spi_command_packet_t* packet);

/**
 * @brief Print packet contents for debugging
 *
 * Outputs packet in both structured and hex format
 *
 * @param packet Pointer to packet to dump
 * @param label Label string for output
 */
void spi_dump_packet(const spi_command_packet_t* packet, const char* label);

/**
 * @brief Print command packet in detailed format
 *
 * Shows packet interpretation based on command type
 *
 * @param packet Pointer to packet to dump
 * @param is_tx true for TX packet, false for RX packet
 * @param label Label string for output
 */
void spi_dump_command_packet(const spi_command_packet_t* packet, bool is_tx, const char* label);

/**
 * @brief Convert packet from Little-Endian to Big-Endian for SPI transmission
 *
 * ESP32 is Little-Endian internally, but SPI protocol requires Big-Endian.
 * This function swaps bytes in all 16-bit words: 0x0001 -> 0x00 0x01
 *
 * @param packet Pointer to packet to convert (modified in-place)
 */
void spi_convert_to_big_endian(spi_command_packet_t* packet);

/**
 * @brief Convert packet from Big-Endian to Little-Endian after SPI reception
 *
 * Converts received Big-Endian SPI data back to ESP32 Little-Endian format
 *
 * @param packet Pointer to packet to convert (modified in-place)
 */
void spi_convert_from_big_endian(spi_command_packet_t* packet);

/**
 * @brief Get SPI transfer statistics
 * 
 * Returns transfer count and error statistics
 * 
 * @param total_transfers Pointer to store total transfer count
 * @param error_count Pointer to store error count
 */
void spi_get_stats(uint32_t* total_transfers, uint32_t* error_count);

/**
 * @brief Reset SPI statistics counters
 */
void spi_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif // SPI_INTERFACE_H
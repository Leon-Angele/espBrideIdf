

#ifndef SPI_TEST_H
#define SPI_TEST_H

#include "spi_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run comprehensive SPI interface tests
 * 
 * Tests union structure, memory alignment, and basic functionality
 * without requiring external hardware
 * 
 * @return true if all tests pass, false otherwise
 */
bool spi_run_self_tests(void);

/**
 * @brief Test union structure memory layout
 * 
 * Verifies that word and field access produce consistent results
 * 
 * @return true if memory layout is correct
 */
bool spi_test_union_layout(void);

/**
 * @brief Test packet validation functions
 * 
 * Tests various packet validation scenarios
 * 
 * @return true if validation works correctly
 */
bool spi_test_packet_validation(void);

/**
 * @brief Test SPI loopback (MISO connected to MOSI)
 * 
 * Performs actual SPI transactions with loopback connection
 * Requires hardware setup: connect GPIO2 (MISO) to GPIO7 (MOSI)
 * 
 * @return true if loopback test succeeds
 */
bool spi_test_hardware_loopback(void);

/**
 * @brief Performance benchmark test
 * 
 * Measures SPI transaction timing and throughput
 * 
 * @return true if performance meets expectations
 */
bool spi_test_performance(void);

#ifdef __cplusplus
}
#endif

#endif // SPI_TEST_H
# ESP32 SPI Master Interface Design

## Union-basierte Datenpaket-Struktur (Option B)

### Datenpaket Definition
```c
typedef union {
    // Raw 16-bit Word Array für direkten SPI Transfer
    uint16_t words[9];
    
    // Strukturierte High-Level Sicht
    struct {
        uint16_t status_word;    // Word 0: Status/Command
        uint32_t counter;        // Word 1-2: 32-bit Counter  
        float    data_1;         // Word 3-4: Float 1 (IEEE 754)
        float    data_2;         // Word 5-6: Float 2 (IEEE 754) 
        float    data_3;         // Word 7-8: Float 3 (IEEE 754)
    } __attribute__((packed)) fields;
    
    // Raw Byte Array für Debugging
    uint8_t bytes[18];
} spi_data_packet_t;
```

### Memory Layout Visualization
```
Byte:    0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17
Word:    [0   ] [1   ] [2   ] [3   ] [4   ] [5   ] [6   ] [7   ] [8   ]
Fields:  [stat] [counter    ] [data_1     ] [data_2     ] [data_3     ]
```

### Vorteile dieser Architektur

1. **Dual Access Pattern:**
   - `packet.words[0-8]` für direkten SPI Transfer
   - `packet.fields.counter` für typsicheren Zugriff

2. **Automatisches Memory Layout:**
   - Compiler handhabt Alignment automatisch
   - Keine manuellen Conversion-Funktionen nötig

3. **Debugging-freundlich:**
   - `packet.bytes[0-17]` für Hex-Dumps
   - Alle drei Sichten auf dieselben Daten

### SPI Master Configuration

#### Hardware Setup
```c
#define SPI_MASTER_HOST    SPI2_HOST
#define PIN_NUM_MISO       GPIO_NUM_2
#define PIN_NUM_MOSI       GPIO_NUM_7
#define PIN_NUM_CLK        GPIO_NUM_6
#define PIN_NUM_CS         GPIO_NUM_10

#define SPI_MASTER_FREQ_HZ 1000000  // 1MHz
```

#### Driver Configuration
```c
spi_bus_config_t bus_cfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096
};

spi_device_interface_config_t dev_cfg = {
    .clock_speed_hz = SPI_MASTER_FREQ_HZ,
    .mode = 0,                          // SPI Mode 0
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 7,
    .flags = 0                          // Full-Duplex
};
```

### Implementation Pattern

#### Basic Usage
```c
// Datenpaket erstellen
spi_data_packet_t tx_packet = {0};
tx_packet.fields.status_word = 0x1234;
tx_packet.fields.counter = inference_count;
tx_packet.fields.data_1 = ml_input;
tx_packet.fields.data_2 = ml_output;
tx_packet.fields.data_3 = confidence;

// SPI Transfer (Full-Duplex)
spi_data_packet_t rx_packet;
spi_transaction_t trans = {
    .length = 18 * 8,                   // 18 Bytes = 144 Bits
    .tx_buffer = tx_packet.words,       // Direkter Zugriff auf Words
    .rx_buffer = rx_packet.words
};

esp_err_t ret = spi_device_transmit(spi_device, &trans);

// Empfangene Daten verwenden
uint32_t received_counter = rx_packet.fields.counter;
float received_data1 = rx_packet.fields.data_1;
```

### Integration mit TensorFlow Lite

#### Main Loop Integration
```c
void loop() {
    // 1. Standard ML Inferenz
    float x = position * kXrange;
    int8_t x_quantized = x / input->params.scale + input->params.zero_point;
    input->data.int8[0] = x_quantized;
    
    interpreter->Invoke();
    
    int8_t y_quantized = output->data.int8[0];
    float y = (y_quantized - output->params.zero_point) * output->params.scale;
    
    // 2. SPI Datenpaket vorbereiten
    static uint32_t spi_counter = 0;
    spi_data_packet_t tx_packet = {0};
    
    tx_packet.fields.status_word = build_status_word();
    tx_packet.fields.counter = ++spi_counter;
    tx_packet.fields.data_1 = x;           // ML Input
    tx_packet.fields.data_2 = y;           // ML Output  
    tx_packet.fields.data_3 = calculate_confidence(x, y);
    
    // 3. Full-Duplex SPI Exchange
    spi_data_packet_t rx_packet;
    spi_full_duplex_exchange(&tx_packet, &rx_packet);
    
    // 4. Handle received sensor data
    if (rx_packet.fields.status_word & STATUS_DATA_VALID) {
        process_sensor_data(rx_packet.fields.data_1,
                           rx_packet.fields.data_2, 
                           rx_packet.fields.data_3);
    }
    
    // Standard output
    HandleOutput(x, y);
}
```

### Status Word Bit Definition
```c
#define STATUS_DATA_VALID    0x0001
#define STATUS_ERROR         0x0002  
#define STATUS_READY         0x0004
#define STATUS_COMMAND_MASK  0x00F0
#define STATUS_SEQUENCE_MASK 0xFF00

inline uint16_t build_status_word(void) {
    static uint8_t sequence = 0;
    return STATUS_READY | STATUS_DATA_VALID | ((++sequence) << 8);
}
```

### Performance Characteristics
```
Transfer Size:    18 Bytes (144 Bits)
SPI Clock:        1MHz
Transfer Time:    144µs per packet
Max Rate:         ~6.9 kHz (bei kontinuierlicher Übertragung)
FreeRTOS Cycle:   500ms (2Hz) - viel Headroom
```

### Error Handling
```c
esp_err_t spi_full_duplex_exchange(spi_data_packet_t* tx_packet,
                                   spi_data_packet_t* rx_packet) {
    spi_transaction_t trans = {
        .length = sizeof(spi_data_packet_t) * 8,
        .tx_buffer = tx_packet->words,
        .rx_buffer = rx_packet->words
    };
    
    esp_err_t ret = spi_device_transmit(spi_device, &trans);
    if (ret != ESP_OK) {
        MicroPrintf("SPI transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Optional: CRC/Checksum validation
    if (!validate_packet(rx_packet)) {
        MicroPrintf("Received packet validation failed");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}
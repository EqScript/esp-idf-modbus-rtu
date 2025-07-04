# esp-idf-modbus-rtu

A non-blocking Modbus RTU client (master) for the ESP-IDF framework.

This is a forked and refactored version of the original [esp32ModbusRTU](https://github.com/bertmelis/esp32ModbusRTU) library by Bert Melis, adapted to use the native ESP-IDF UART driver instead of the Arduino framework.

## Features

- **Modbus Client (Master):** For ESP32.
- **ESP-IDF Native:** Built for the ESP-IDF framework, using the native UART driver.
- **Non-Blocking API:** Requests are queued and handled by a dedicated FreeRTOS task, allowing your main application to run without blocking.
- **RS485 Half-Duplex Control:** Automatic control of the RTS (DE/RE) pin for RS485 transceivers, including support for inverted logic.
- **Event-Driven:** Use callbacks to handle incoming data and error conditions.

## Implemented Function Codes

- `0x02`: Read Discrete Inputs
- `0x03`: Read Holding Registers
- `0x04`: Read Input Registers
- `0x06`: Write Single Holding Register
- `0x10`: Write Multiple Holding Registers

## Installation

### PlatformIO

Add the library to your `platformio.ini` file:

```ini
lib_deps =
    https://github.com/EqScript/esp-idf-modbus-rtu.git
```

### Git Submodule

From your project's root directory:

```bash
git submodule add https://github.com/EqScript/esp-idf-modbus-rtu.git components/esp-idf-modbus-rtu
```

## Usage

Here is a basic example of how to set up and use the library:

```cpp
#include "esp_log.h"
#include "esp-idf-modbus-rtu.h"

static const char* TAG = "MODBUS_CLIENT";

esp32ModbusRTU modbus; // Use default UART2, pins 17(TX), 16(RX), no RTS

// Callback for successful data reception
void handleData(uint8_t serverAddress, esp32Modbus::FunctionCode fc, uint16_t address, uint8_t* data, size_t length) {
  ESP_LOGI(TAG, "Received data: id=0x%02x, fc=0x%02x, addr=0x%04x, len=%d", serverAddress, fc, address, length);
  // Process your data here
}

// Callback for errors
void handleError(esp32Modbus::Error error) {
  ESP_LOGE(TAG, "Error: 0x%02x", static_cast<uint8_t>(error));
}

extern "C" void app_main() {
    // For a board with an inverted RTS pin (e.g., MAX485)
    // esp32ModbusRTU modbus(UART_NUM_1, 22, 21, 23, 9600, true);

    modbus.onData(handleData);
    modbus.onError(handleError);
    modbus.begin();

    // Example: Read 2 holding registers from slave 1 at address 100
    modbus.readHoldingRegisters(1, 100, 2);
}
```

## API Reference

### Constructor

`esp32ModbusRTU(uart_port_t uart_port, int tx_pin, int rx_pin, int rts_pin, uint32_t baud_rate, bool rts_inverted = false)`

### Methods

- `void begin(int coreID = -1)`: Starts the Modbus task.
- `void onData(MBRTUOnData handler)`: Registers a callback for successful responses.
- `void onError(MBRTUOnError handler)`: Registers a callback for errors.
- `void setTimeOutValue(uint32_t tov)`: Sets the response timeout in milliseconds.
- `bool readHoldingRegisters(...)`, `bool writeSingleHoldingRegister(...)`, etc.: Queues a request to the Modbus master.

## Configuration

The following settings can be changed by defining them before including the library header, or by using compiler flags.

- `QUEUE_SIZE`: The maximum number of requests that can be queued. Defaults to `20`.
- `TIMEOUT_MS`: The default timeout for waiting for a response. Defaults to `5000` ms.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
#include "esp_log.h"
#include "espidfModbusRtu.h"

static const char* TAG = "MODBUS_CLIENT";

espidfModbusRtu modbus; // Use default UART2, pins 17(TX), 16(RX), no DE/RE control

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
    // For a board with separate DE/RE pins (e.g., MAX485) and inverted logic:
    // espidfModbusRtu modbus(UART_NUM_1, 22, 21, 23, 24, 9600, true); // UART1, TX=22, RX=21, DE=23, RE=24, 9600 baud, inverted DE/RE

    modbus.onData(handleData);
    modbus.onError(handleError);
    modbus.begin();

    // Example: Read 2 holding registers from slave 1 at address 100
    modbus.readHoldingRegisters(1, 100, 2);
}

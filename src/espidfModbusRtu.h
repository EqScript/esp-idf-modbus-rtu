/* esp32ModbusRTU

Copyright (c) 2018-2025 Bert Melis, EqScript

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef ESPIDFMODBUSRTU_H
#define ESPIDFMODBUSRTU_H

// #if defined ARDUINO_ARCH_ESP32

#ifndef QUEUE_SIZE
#define QUEUE_SIZE 20
#endif
#ifndef TIMEOUT_MS
#define TIMEOUT_MS 5000
#endif

#include <functional>

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <driver/gpio.h>
}

#include "esp32ModbusTypeDefs.h"
#include "ModbusMessage.h"

class espidfModbusRtu {
 public:
  explicit espidfModbusRtu(uart_port_t uart_port = UART_NUM_2, int tx_pin = 17, int rx_pin = 16, int de_pin = -1, int re_pin = -1, uint32_t baud_rate = 115200, bool de_re_inverted = false);
  ~espidfModbusRtu();
  void begin(int coreID = -1);
  bool readDiscreteInputs(uint8_t slaveAddress, uint16_t address, uint16_t numberCoils);
  bool readHoldingRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters);
  bool readInputRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters);
  bool writeSingleHoldingRegister(uint8_t slaveAddress, uint16_t address, uint16_t data);
  bool writeMultHoldingRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters, uint8_t* data);
  void onData(esp32Modbus::MBRTUOnData handler);
  void onError(esp32Modbus::MBRTUOnError handler);
  void setTimeOutValue(uint32_t tov);

 private:
  bool _addToQueue(esp32ModbusRTUInternals::ModbusRequest* request);
  static void _handleConnection(espidfModbusRtu* instance);
  void _send(uint8_t* data, uint8_t length);
  esp32ModbusRTUInternals::ModbusResponse* _receive(esp32ModbusRTUInternals::ModbusRequest* request);

 private:
  uint32_t m_timeOutValue;
  uart_port_t m_uartPort;
  int m_txPin;
  int m_rxPin;
  int m_dePin;
  int m_rePin;
  uint32_t m_baudRate;
  bool m_deReInverted;
  uint32_t m_lastMillis;
  uint32_t m_interval;
  TaskHandle_t m_task;
  QueueHandle_t m_queue;
  esp32Modbus::MBRTUOnData m_onData;
  esp32Modbus::MBRTUOnError m_onError;
};

// #endif

// #elif defined VITOWIFI_TEST

// #else

#pragma message "no suitable platform"

#endif

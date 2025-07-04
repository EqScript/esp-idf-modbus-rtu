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

#include "espidfModbusRtu.h"
#include "freertos/task.h"

using namespace esp32ModbusRTUInternals;  // NOLINT

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

#include "espidfModbusRtu.h"
#include "freertos/task.h"
#include "driver/gpio.h"

using namespace esp32ModbusRTUInternals;  // NOLINT

espidfModbusRtu::espidfModbusRtu(uart_port_t uart_port, int tx_pin, int rx_pin, int de_pin, int re_pin, uint32_t baud_rate, bool de_re_inverted) :
  m_timeOutValue(TIMEOUT_MS),
  m_uartPort(uart_port),
  m_txPin(tx_pin),
  m_rxPin(rx_pin),
  m_dePin(de_pin),
  m_rePin(re_pin),
  m_baudRate(baud_rate),
  m_deReInverted(de_re_inverted),
  m_lastMillis(0),
  m_interval(0),
  m_task(nullptr),
  m_queue(nullptr) {
    m_queue = xQueueCreate(QUEUE_SIZE, sizeof(ModbusRequest*));
}

espidfModbusRtu::~espidfModbusRtu() {
  if (m_task) {
    vTaskDelete(m_task);
  }
  if (m_queue) {
    vQueueDelete(m_queue);
  }
  uart_driver_delete(m_uartPort);
}

void espidfModbusRtu::begin(int coreID /* = -1 */) {
  uart_config_t uart_config = {
    .baud_rate = (int)m_baudRate,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  ESP_ERROR_CHECK(uart_param_config(m_uartPort, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(m_uartPort, m_txPin, m_rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(m_uartPort, 256, 256, 0, NULL, 0));

  if (m_dePin >= 0) {
    gpio_set_direction((gpio_num_t)m_dePin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)m_dePin, m_deReInverted ? 0 : 1); // Set DE to receive mode (inactive)
  }
  if (m_rePin >= 0) {
    gpio_set_direction((gpio_num_t)m_rePin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)m_rePin, m_deReInverted ? 1 : 0); // Set RE to receive mode (active)
  }

  xTaskCreatePinnedToCore((TaskFunction_t)&_handleConnection, "espidfModbusRtu", 4096, this, 5, &m_task, coreID >= 0 ? coreID : tskNO_AFFINITY);
  
  // silent interval is at least 3.5x character time
  m_interval = 35000 / m_baudRate;  // 3.5 * 1000 * 10 / baud
  if (m_interval == 0) m_interval = 1;  // minimum of 1msec interval
}

bool espidfModbusRtu::readDiscreteInputs(uint8_t slaveAddress, uint16_t address, uint16_t numberCoils) {
  ModbusRequest* request = new ModbusRequest02(slaveAddress, address, numberCoils);
  return _addToQueue(request);
}
bool espidfModbusRtu::readHoldingRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters) {
  ModbusRequest* request = new ModbusRequest03(slaveAddress, address, numberRegisters);
  return _addToQueue(request);
}

bool espidfModbusRtu::readInputRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters) {
  ModbusRequest* request = new ModbusRequest04(slaveAddress, address, numberRegisters);
  return _addToQueue(request);
}

bool espidfModbusRtu::writeSingleHoldingRegister(uint8_t slaveAddress, uint16_t address, uint16_t data) {
  ModbusRequest* request = new ModbusRequest06(slaveAddress, address, data);
  return _addToQueue(request);
}

bool espidfModbusRtu::writeMultHoldingRegisters(uint8_t slaveAddress, uint16_t address, uint16_t numberRegisters, uint8_t* data) {
  ModbusRequest* request = new ModbusRequest16(slaveAddress, address, numberRegisters, data);
  return _addToQueue(request);
}

void espidfModbusRtu::onData(esp32Modbus::MBRTUOnData handler) {
  m_onData = handler;
}

void espidfModbusRtu::onError(esp32Modbus::MBRTUOnError handler) {
  m_onError = handler;
}

bool espidfModbusRtu::_addToQueue(ModbusRequest* request) {
  if (!request) {
    return false;
  } else if (xQueueSend(m_queue, reinterpret_cast<void*>(&request), (TickType_t)0) != pdPASS) {
    delete request;
    return false;
  } else {
    return true;
  }
}

void espidfModbusRtu::_handleConnection(espidfModbusRtu* instance) {
  while (1) {
    ModbusRequest* request;
    if (pdTRUE == xQueueReceive(instance->m_queue, &request, portMAX_DELAY)) {  // block and wait for queued item
      instance->_send(request->getMessage(), request->getSize());
      ModbusResponse* response = instance->_receive(request);
      if (response->isSucces()) {
        if (instance->m_onData) instance->m_onData(response->getSlaveAddress(), response->getFunctionCode(), request->getAddress(), response->getData(), response->getByteCount());
      } else {
        if (instance->m_onError) instance->m_onError(response->getError());
      }
      delete request;  // object created in public methods
      delete response;  // object created in _receive()
    }
  }
}

void espidfModbusRtu::_send(uint8_t* data, uint8_t length) {
  while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - m_lastMillis < m_interval) {
    vTaskDelay(1);
  }
  if (m_dePin >= 0) gpio_set_level((gpio_num_t)m_dePin, m_deReInverted ? 1 : 0); // Set DE to transmit active
  if (m_rePin >= 0) gpio_set_level((gpio_num_t)m_rePin, m_deReInverted ? 0 : 1); // Set RE to transmit inactive
  uart_write_bytes(m_uartPort, (const char*)data, length);
  uart_wait_tx_done(m_uartPort, 100 / portTICK_PERIOD_MS); // Wait for TX to finish
  if (m_dePin >= 0) gpio_set_level((gpio_num_t)m_dePin, m_deReInverted ? 0 : 1); // Set DE to receive mode (inactive)
  if (m_rePin >= 0) gpio_set_level((gpio_num_t)m_rePin, m_deReInverted ? 1 : 0); // Set RE to receive mode (active)
  m_lastMillis = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void espidfModbusRtu::setTimeOutValue(uint32_t tov) {
  if (tov) m_timeOutValue = tov;
}

ModbusResponse* espidfModbusRtu::_receive(ModbusRequest* request) {
  ModbusResponse* response = new ModbusResponse(request->responseLength(), request);
  uint8_t buffer[256];
  int len = uart_read_bytes(m_uartPort, buffer, 256, m_timeOutValue / portTICK_PERIOD_MS);
  if (len > 0) {
    for (int i = 0; i < len; i++) {
      response->add(buffer[i]);
    }
  }
  m_lastMillis = xTaskGetTickCount() * portTICK_PERIOD_MS;
  return response;
}
  
/*
An Arduino library for Nordic Semiconductors' proprietary "UART/Serial Port Emulation over BLE" protocol, for ESP32.

* https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v14.0.0/ble_sdk_app_nus_eval.html
* https://learn.adafruit.com/introducing-adafruit-ble-bluetooth-low-energy-friend/uart-service
* https://github.com/sandeepmistry/arduino-BLEPeripheral/tree/master/examples/serial

Tested using UART console feature in [Adafruit Bluefruit LE Connect](https://apps.apple.com/at/app/adafruit-bluefruit-le-connect/id830125974).
*/

#ifndef __BLE_SERIAL_H__
#define __BLE_SERIAL_H__

#include <Arduino.h>
#include <BLEServer.h>

#define BLE_ATTRIBUTE_MAX_VALUE_LENGTH 20
#define BLE_SERIAL_RECEIVE_BUFFER_SIZE 256

template<size_t N> class ByteRingBuffer {
 private:
  uint8_t ringBuffer[N];
  size_t newestIndex = 0;
  size_t length = 0;
 public:
  void add(uint8_t value) {
    ringBuffer[newestIndex] = value;
    newestIndex = (newestIndex + 1) % N;
    length = min(length + 1, N);
  }
  int pop() { // pops the oldest value off the ring buffer
    if (length == 0) {
      return -1;
    }
    uint8_t result = ringBuffer[(N + newestIndex - length) % N];
    length -= 1;
    return result;
  }
  void clear() {
    newestIndex = 0;
    length = 0;
  }
  int get(size_t index) { // this.get(0) is the oldest value, this.get(this.getLength() - 1) is the newest value
    if (index < 0 || index >= length) {
      return -1;
    }
    return ringBuffer[(N + newestIndex - length + index) % N];
  }
  size_t getLength() { return length; }
};

class HardwareBLESerial : private BLECharacteristicCallbacks, private BLEServerCallbacks {
 public:
  // singleton instance getter
  static HardwareBLESerial& getInstance() {
    static HardwareBLESerial instance; // instantiated on first use, guaranteed to be destroyed
    return instance;
  }

  // similar to begin(), but also sets up ArduinoBLE for you, which you otherwise have to do manually
  // use this for simplicity, use begin() for flexibility (e.g., more than one service)
  bool beginAndSetupBLE(const char *name);

  void begin();
  void poll();
  void end();
  size_t available();
  int peek();
  int read();
  size_t write(uint8_t byte);
  void flush();

  size_t availableLines();
  size_t peekLine(char *buffer, size_t bufferSize);
  size_t readLine(char *buffer, size_t bufferSize);
  size_t print(const char *value);
  size_t println(const char *value);
  size_t print(char value);
  size_t println(char value);
  size_t print(int64_t value);
  size_t println(int64_t value);
  size_t print(uint64_t value);
  size_t println(uint64_t value);
  size_t print(double value);
  size_t println(double value);

  operator bool();
 private:
  HardwareBLESerial();
  HardwareBLESerial(HardwareBLESerial const &other) = delete;  // disable copy constructor
  void operator=(HardwareBLESerial const &other) = delete;  // disable assign constructor
  ~HardwareBLESerial();

  ByteRingBuffer<BLE_SERIAL_RECEIVE_BUFFER_SIZE> receiveBuffer;
  size_t numAvailableLines;

  unsigned long long lastFlushTime;
  size_t transmitBufferLength;
  uint8_t transmitBuffer[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];

  BLEServer* uartServer = nullptr;
  BLEService* uartService = nullptr;
  BLECharacteristic* receiveCharacteristic = nullptr;
  BLECharacteristic* transmitCharacteristic = nullptr;

  void onReceive(const uint8_t* data, size_t size);
  virtual void onWrite(BLECharacteristic* characteristic) override;
  virtual void onDisconnect(BLEServer* pServer) override;
};

#endif

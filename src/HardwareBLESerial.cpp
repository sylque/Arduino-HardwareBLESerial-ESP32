#include "HardwareBLESerial.h"

#include <BLE2902.h>
#include <BLEDevice.h>

HardwareBLESerial::HardwareBLESerial() {
  this->numAvailableLines = 0;
  this->transmitBufferLength = 0;
  this->lastFlushTime = 0;
}

HardwareBLESerial::~HardwareBLESerial() { this->end(); }

bool HardwareBLESerial::beginAndSetupBLE(const char *name) {
  BLEDevice::init(name);
  this->begin();
  return true;
}

void HardwareBLESerial::begin() {
  // Create the server
  this->uartServer = BLEDevice::createServer();
  this->uartServer->setCallbacks(this);

  // Create the UART service
  this->uartService =
      uartServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  // Create the RX charac
  this->receiveCharacteristic =
      uartService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
                                        BLECharacteristic::PROPERTY_WRITE_NR);
  receiveCharacteristic->setCallbacks(this);

  // Create the TX charac. IMPORTANT: it should have the BLE2902 descriptor
  this->transmitCharacteristic =
      uartService->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                                        BLECharacteristic::PROPERTY_NOTIFY);
  static BLE2902 s_ble2902;
  this->transmitCharacteristic->addDescriptor(&s_ble2902);

  // Start the service
  this->uartService->start();

  // Advertise. IMPORTANT: also advertizing the Uart service is required for the
  // Bluefruit Connect app to list the device as "Uart capable" (notice that
  // advertizing the Uart service is *not* required for the app to be able to
  // connect to the device and use all Uart features)
  BLEDevice::getAdvertising()->addServiceUUID(this->uartService->getUUID());
  BLEDevice::startAdvertising();
}

void HardwareBLESerial::poll() {
  if (millis() - this->lastFlushTime > 100) {
    flush();
  } else {
    // BLE.poll();
  }
}

void HardwareBLESerial::end() {
  if (this->receiveCharacteristic) {
    this->receiveCharacteristic->setCallbacks(nullptr);
  }
  flush();
  this->receiveBuffer.clear();
  delete uartServer;
  uartServer = nullptr;
  if (uartService) {
    uartService->stop();
  }  
  delete uartService;
  uartService = nullptr;
  delete receiveCharacteristic;
  receiveCharacteristic = nullptr;
  delete transmitCharacteristic;
  transmitCharacteristic = nullptr;
}

size_t HardwareBLESerial::available() {
  return this->receiveBuffer.getLength();
}

int HardwareBLESerial::peek() {
  if (this->receiveBuffer.getLength() == 0) return -1;
  return this->receiveBuffer.get(0);
}

int HardwareBLESerial::read() {
  int result = this->receiveBuffer.pop();
  if (result == (int)'\n') {
    this->numAvailableLines --;
  }
  return result;
}

size_t HardwareBLESerial::write(uint8_t byte) {
  // if (this->transmitCharacteristic->subscribed() == false) {
  //   return 0;
  // }
  this->transmitBuffer[this->transmitBufferLength] = byte;
  this->transmitBufferLength ++;
  if (this->transmitBufferLength == sizeof(this->transmitBuffer)) {
    flush();
  }
  return 1;
}

void HardwareBLESerial::flush() {
  if (this->transmitBufferLength > 0) {
    this->transmitCharacteristic->setValue(this->transmitBuffer, this->transmitBufferLength);
    this->transmitBufferLength = 0;
    this->transmitCharacteristic->notify();
  }
  this->lastFlushTime = millis();
  // BLE.poll();
}

size_t HardwareBLESerial::availableLines() {
  return this->numAvailableLines;
}

size_t HardwareBLESerial::peekLine(char *buffer, size_t bufferSize) {
  if (this->availableLines() == 0) {
    buffer[0] = '\0';
    return 0;
  }
  size_t i = 0;
  for (; i < bufferSize - 1; i ++) {
    int chr = this->receiveBuffer.get(i);
    if (chr == -1 || chr == '\n') {
      break;
    } else {
      buffer[i] = chr;
    }
  }
  buffer[i] = '\0';
  return i;
}

size_t HardwareBLESerial::readLine(char *buffer, size_t bufferSize) {
  if (this->availableLines() == 0) {
    buffer[0] = '\0';
    return 0;
  }
  size_t i = 0;
  for (; i < bufferSize - 1; i ++) {
    int chr = this->read();
    if (chr == -1 || chr == '\n') {
      break;
    } else {
      buffer[i] = chr;
    }
  }
  buffer[i] = '\0';
  return i;
}

size_t HardwareBLESerial::print(const char *str) {
  // if (this->transmitCharacteristic->subscribed() == false) {
  //   return 0;
  // }
  size_t written = 0;
  for (size_t i = 0; str[i] != '\0'; i ++) {
    written += this->write(str[i]);
  }
  return written;
}
size_t HardwareBLESerial::println(const char *str) { return this->print(str) + this->write('\n'); }

size_t HardwareBLESerial::print(char value) {
  char buf[2] = { value, '\0' };
  return this->print(buf);
}
size_t HardwareBLESerial::println(char value) { return this->print(value) + this->write('\n'); }

size_t HardwareBLESerial::print(int64_t value) {
  char buf[21]; snprintf(buf, 21, "%lld", value); // the longest representation of a uint64_t is for -2^63, 20 characters plus null terminator
  return this->print(buf);
}
size_t HardwareBLESerial::println(int64_t value) { return this->print(value) + this->write('\n'); }

size_t HardwareBLESerial::print(uint64_t value) {
  char buf[21]; snprintf(buf, 21, "%llu", value);  // the longest representation of a uint64_t is for 2^64-1, 20 characters plus null terminator
  return this->print(buf);
}
size_t HardwareBLESerial::println(uint64_t value) { return this->print(value) + this->write('\n'); }

size_t HardwareBLESerial::print(double value) {
  char buf[319]; snprintf(buf, 319, "%f", value); // the longest representation of a double is for -1e308, 318 characters plus null terminator
  return this->print(buf);
}
size_t HardwareBLESerial::println(double value) { return this->print(value) + this->write('\n'); }

bool HardwareBLESerial::availableCmds() {
  return this->peek() == '!';
}

// Returns an empty string if error
std::string HardwareBLESerial::readCmd() {
  if (!this->availableCmds()) {
    return "";
  }
  
  char buf[30];

  // Get the command start character
  buf[0] = this->read();
  
  // Get the command character
  buf[1] = this->read();

  // Get the payload size
  int payloadSize = 0;
  switch (buf[1]) {
    // Quaternion
    case 'Q':
      payloadSize = 4 * 4;
      break;
    // Accelerometer
    // Gyro
    // Magnetometer
    // Location
    case 'A':
    case 'G':
    case 'M':
    case 'L':
      payloadSize = 3 * 4;
      break;
    // Control pad
    case 'B':
      payloadSize = 2;
      break;
    default:
      return "";
  }
  
  // Read the payload and the CRC
  for (int i = 0; i < payloadSize + 1; ++i) {
    buf[i + 2] = this->read();
  }

  // Check the crc
  uint8_t sum = 0;
  for (int i = 0; i < payloadSize + 2; ++i) {
    sum += buf[i];
  }
  if (buf[payloadSize + 2] != (uint8_t)~sum) {
    return "";
  }

  return buf;
}

// Returns true if at least one device is connected, as master or slave
HardwareBLESerial::operator bool() {
  return this->uartServer ? this->uartServer->getConnectedCount() > 0 : false;
}

void HardwareBLESerial::onReceive(const uint8_t* data, size_t size) {
  for (size_t i = 0; i < min(size, sizeof(this->receiveBuffer)); i++) {
    this->receiveBuffer.add(data[i]);
    if (data[i] == '\n') {
      this->numAvailableLines ++;
    }
  }
}

void HardwareBLESerial::onWrite(BLECharacteristic *characteristic) {
  this->onReceive(characteristic->getData(), characteristic->getLength());
}

void HardwareBLESerial::onDisconnect(BLEServer *pServer) {
  BLEDevice::startAdvertising();
}

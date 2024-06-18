#pragma once
#include <Arduino.h>
#include <functional>
#include <map>
#include <vector>

class UartHandler {
private:
  HardwareSerial& serial;
  std::map<std::string, std::function<void(std::string)>> callbacksString;
  std::map<std::string, std::function<void(std::vector<uint8_t>)>> callbacksBytes;
  const char specialChar = ':';

  void onReceiveData() {
    if (serial.available()) {
      String message = serial.readStringUntil(',');

      int specialCharIndex = message.indexOf(specialChar);

      if (specialCharIndex == -1) {
        std::string key = message.c_str();
        std::string payload = serial.readStringUntil('\n').c_str();

        invokeStringCallback(key, payload);
      } else {
        std::string key = message.substring(0, specialCharIndex).c_str();
        int payloadSize = message.substring(specialCharIndex + 1).toInt();
        std::vector<uint8_t> payload;
        for (int i = 0; i < payloadSize; i++) {
          payload.push_back(serial.read());
        }
        invokeBytesCallback(key, payload);
      }
    }
  }

  void invokeStringCallback(const std::string& key, const std::string& payload) {
    auto it = callbacksString.find(key);
    if (it != callbacksString.end()) {
      it->second(payload);
    }
  }

  void invokeBytesCallback(const std::string& key, const std::vector<uint8_t>& payload) {
    auto it = callbacksBytes.find(key);
    if (it != callbacksBytes.end()) {
      it->second(payload);
    }
  }

public:
  UartHandler(HardwareSerial& serial) : serial(serial) {
    serial.onReceive(std::bind(&UartHandler::onReceiveData, this));
  }

  void attachStringCallback(const std::string& key, std::function<void(std::string)> callback) {
    callbacksString[key] = callback;
  }

  void attachBytesCallback(const std::string& key, std::function<void(std::vector<uint8_t>)> callback) {
    callbacksBytes[key] = callback;
  }

  void emit(const std::string& key, const std::string& payload) {
    std::string message = key + "," + payload + "\n";
    serial.write(message.c_str());
  }

  void emit(const std::string& key, const std::vector<uint8_t>& payload) {
    std::string message = key + ":" + std::to_string(payload.size()) + ",";
    serial.write(message.c_str());
    serial.write(payload.data(), payload.size());
  }
};
#include "Logger.h"

void Logger::log(std::string message) {
    uartHandler.emit("log", message.c_str());
}

void Logger::log(String message) {
    uartHandler.emit("log", message.c_str());
}
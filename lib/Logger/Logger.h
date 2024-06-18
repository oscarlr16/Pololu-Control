#pragma once
#include "Arduino.h"
#include "UartHandler.h"

extern UartHandler uartHandler;

class Logger {
public:
    Logger() {}
    static void log(std::string message);
    static void log(String message);
};
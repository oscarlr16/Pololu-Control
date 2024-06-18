#pragma once

enum class Direction {
    FORWARD,
    BACKWARD
};

class MotorDriverInterface {
public:
    MotorDriverInterface() {}
    virtual void run(double power) = 0;
    virtual void stop() = 0;
};
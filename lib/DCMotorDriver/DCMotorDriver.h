#pragma once
#include "MotorDriverInterface.h"
#include "Arduino.h"
#include "config.h"

class DCMotorDriver: public MotorDriverInterface {
public:
    DCMotorDriver(int pin_in1, int pin_in2, int pin_pwm, int pwm_channel, int pin_enc_a, int pin_enc_b);
    void run(double power) override;
    void stop() override;

    int getEncoderPinA() const;
    int getEncoderPinB() const;

private:
    int _pin_in1;
    int _pin_in2;
    int _pin_pwm;
    int _pwm_channel;
    int _pin_enc_a;
    int _pin_enc_b;
    Direction _direction;
};
#include "DCMotorDriver.h"
#include <Arduino.h>
#include <driver/ledc.h>

DCMotorDriver::DCMotorDriver(int pin_in1, int pin_in2, int pin_pwm, int pwm_channel, int pin_enc_a, int pin_enc_b) {
    _pin_in1 = pin_in1;
    _pin_in2 = pin_in2;
    _pin_pwm = pin_pwm;
    _pwm_channel = pwm_channel;
    _pin_enc_a = pin_enc_a;
    _pin_enc_b = pin_enc_b;

    _direction = Direction::FORWARD;

    pinMode(_pin_in1, OUTPUT);
    pinMode(_pin_in2, OUTPUT);
    pinMode(_pin_pwm, OUTPUT);

    ledcSetup(_pwm_channel, 32000, 10);  // Usa el canal especificado
    ledcAttachPin(_pin_pwm, _pwm_channel);
}

void DCMotorDriver::run(double power) {
    if (power > 0) {
        digitalWrite(_pin_in1, HIGH);
        digitalWrite(_pin_in2, LOW);
    } else if (power < 0) {
        digitalWrite(_pin_in1, LOW);
        digitalWrite(_pin_in2, HIGH);
    } else {
        stop();
        return;
    }

    int duty = abs(power) * 1023 / 100; 
    ledcWrite(_pwm_channel, duty);  // Usa el canal especificado
}

void DCMotorDriver::stop() {
    digitalWrite(_pin_in1, LOW);
    digitalWrite(_pin_in2, LOW);
    ledcWrite(_pwm_channel, 0);  // Usa el canal especificado
}

int DCMotorDriver::getEncoderPinA() const {
    return _pin_enc_a;
}

int DCMotorDriver::getEncoderPinB() const {
    return _pin_enc_b;
}

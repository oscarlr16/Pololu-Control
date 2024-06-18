#pragma once
#include "Arduino.h"
#include "DCMotorDriver.h"
#include <PID_v1.h>
#include <ESP32Encoder.h>

class DCMotorControl {
  public:
    DCMotorControl(DCMotorDriver &motor);
    ~DCMotorControl();

    void move_to(double positon);
    void set_speed(double speed);
    void stop();

    void set_pid_tunings(double Kp, double Ki, double Kd);
    void set_pid_sample_time(int sample_time);

    void get_pid_tunings(double &Kp, double &Ki, double &Kd);
    double get_pid_sample_time();

    double get_position(); // Nueva función para obtener la posición actual
    
    PID* _pid_controller;
    DCMotorDriver* _motor;
    ESP32Encoder _encoder;
    double _pid_setpoint = 0, _pid_output = 0;
    double _position = 0;

  private:

    double _sample_time;
    double _Kp, _Ki, _Kd;
};
#include "DCMotorControl.h"
#include "Logger.h"
#include "config.h"

#define DEFAULT_PID_SAMPLE_TIME 50
#define DEFAULT_PID_KP 0.7
#define DEFAULT_PID_KI 0.09
#define DEFAULT_PID_KD 0.02
#define DEFAULT_OUTPUT_MIN -100
#define DEFAULT_OUTPUT_MAX 100
#define POSITIVE_MAX_POWER 100
#define POSITIVE_MIN_POWER 10
#define NEGATIVE_MAX_POWER -100
#define NEGATIVE_MIN_POWER -10

void thread_motor_function(void* pvParameters);

DCMotorControl::DCMotorControl(DCMotorDriver &motor) {
    _motor = &motor;
    _pid_controller = new PID(&_position, &_pid_output, &_pid_setpoint, DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD, DIRECT);
    _pid_controller->SetMode(AUTOMATIC);
    set_pid_sample_time(DEFAULT_PID_SAMPLE_TIME);
    _pid_controller->SetOutputLimits(DEFAULT_OUTPUT_MIN, DEFAULT_OUTPUT_MAX);
    _encoder.attachFullQuad(motor.getEncoderPinA(), motor.getEncoderPinB());

    // create a thread to run the PID controller
    xTaskCreate(
        thread_motor_function,
        "motor_control",
        4096,
        this,
        5,
        NULL
    );

    Logger::log(String("DCMotorControl initialized"));
}

DCMotorControl::~DCMotorControl() {
    delete _pid_controller;
}

void DCMotorControl::set_pid_tunings(double Kp, double Ki, double Kd) {
    _pid_controller->SetTunings(Kp, Ki, Kd);
}

void DCMotorControl::set_pid_sample_time(int sample_time) {
    _pid_controller->SetSampleTime(sample_time);
    _sample_time = sample_time;
}

void DCMotorControl::get_pid_tunings(double &Kp, double &Ki, double &Kd) {
    Kp = _pid_controller->GetKp();
    Ki = _pid_controller->GetKi();
    Kd = _pid_controller->GetKd();
}

double DCMotorControl::get_pid_sample_time() {
    return _sample_time;
}

void DCMotorControl::move_to(double position) {
    _pid_setpoint = position;
}

double DCMotorControl::get_position() {
    return _position;
}

void thread_motor_function(void* pvParameters) {
    DCMotorControl* motor_control = (DCMotorControl*) pvParameters;
    for(;;) {
        motor_control->_position = motor_control->_encoder.getCount();
        motor_control->_pid_controller->Compute();

        double pid_pwm_output = motor_control->_pid_output;
        if (pid_pwm_output > 0) {
            pid_pwm_output = map(pid_pwm_output, 0, 100, POSITIVE_MIN_POWER, POSITIVE_MAX_POWER);
        } else {
            pid_pwm_output = map(pid_pwm_output, 0, -100, NEGATIVE_MIN_POWER, NEGATIVE_MAX_POWER);
        }

        motor_control->_motor->run(pid_pwm_output);
        Logger::log("PID output: " + String(pid_pwm_output));

        vTaskDelay(motor_control->get_pid_sample_time() / portTICK_PERIOD_MS);
    }
}

void DCMotorControl::set_speed(double speed) {
    Logger::log("Setting speed to: " + String(speed));
}

void DCMotorControl::stop() {
    Logger::log(String("Stopping motor"));
}

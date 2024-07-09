#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "DCMotorDriver.h"
#include "DCMotorControl.h"
#include "UartHandler.h"

// User Adjustable Variables
// You can adjust these variables according to your needs. Changing these values will affect how the motor operates.
// Position Limits mm
#define MOTOR_A_MAX 90.0
#define MOTOR_A_MIN 0.0
#define MOTOR_A_MID (MOTOR_A_MAX + MOTOR_A_MIN )/2.0

// Position Limits degrees
#define MOTOR_B_MAX 37.0/2
#define MOTOR_B_MIN -37.0/2
#define MOTOR_B_MID 0.0

// Counts per revolution (CPR) of the motor encoder
#define CPR 12 

/* -- MOTOR_A is the radius Drive --*/
// motor A ratios 5, 10 , 15
#define MOTOR_A_GEAR_RATIO 5
#define MOTOR_A_SHAFT_RATIO 20/14
#define MOTOR_A_FINAL_RATIO MOTOR_A_GEAR_RATIO*MOTOR_A_SHAFT_RATIO
// Linear advance per revolution in mm
#define LINEAR_ADVANCE_PER_REVOLUTION_MM 10.0 

/* -- MOTOR_B is the Angle Drive --*/
// motor B ratios 250, 298, 350
#define MOTOR_B_GEAR_RATIO 350
#define MOTOR_B_BELT_RATIO 50/12
#define MOTOR_B_FINAL_RATIO MOTOR_B_GEAR_RATIO*MOTOR_B_BELT_RATIO


// PID tunings for Motor A
#define MOTOR_A_P 0.7
#define MOTOR_A_I 0.001
#define MOTOR_A_D 0.01

// PID tunings for Motor B
// starting point 0.7,0.09,0.01
#define MOTOR_B_P 0.01
#define MOTOR_B_I 0.0
#define MOTOR_B_D 0.0

// --- End of User Adjustable Variables ---

// Create UART and motor objects
HardwareSerial uart_0(0);
UartHandler uartHandler(uart_0);
DCMotorDriver motorA(AIN1, AIN2, PWMA_PIN, 0, PINA_ENCODER_A, PINA_ENCODER_B); // PWM channel 0 for motor A
DCMotorDriver motorB(BIN1, BIN2, PWMB_PIN, 1, PINB_ENCODER_A, PINB_ENCODER_B); // PWM channel 1 for motor B
DCMotorControl motorControlA(motorA);
DCMotorControl motorControlB(motorB);

// Global variables for sine wave control
bool sineWaveActiveA = false; // Indicates if sine wave movement is active for motor A
bool sineWaveActiveB = false; // Indicates if sine wave movement is active for motor B
double sineAmplitudeA = 0; // Amplitude for sine wave movement of motor A
double sineAmplitudeB = 0; // Amplitude for sine wave movement of motor B
unsigned long startTimeA; // Start time for sine wave movement of motor A
unsigned long startTimeB; // Start time for sine wave movement of motor B

// Function declarations
void configurePins();
void initializeMotors();
void handleSerialInputOscar();
void handleSerialInputKWC();
void printMotorStatus();
double convertStepsToMM(double steps);
double convertMMToSteps(double mm);
double convertStepsToDegrees(double steps);
double convertDegreesToSteps(double degrees);

void setup() {
    Serial.begin(115200); // Initialize serial communication at 115200 baud
    configurePins(); // Configure the necessary pins
    initializeMotors(); // Initialize the motors
}

void loop() {
    u_int64_t now = millis();

    u_int64_t reportInterval = 300;
    static u_int64_t lastReport = 0;
    if (now > lastReport + reportInterval){
        lastReport = now;
        printMotorStatus(); // Print the current status of the motors
    }

    handleSerialInputOscar(); // Handle serial input from the user
    //handleSerialInputKWC();

    double freqA = 0.2; // Hz
    double freqB = 0.2; // Hz

    // add update rate code
    u_int64_t sineInterval = 50;
    static u_int64_t lastSine = 0;
    if (now > lastSine + sineInterval){
        lastSine = now;
        if (sineWaveActiveA) { // If sine wave movement is active for motor A
            unsigned long timeA = (millis() - startTimeA); // Calculate elapsed time in milliseconds
            double setpoint = (double)(MOTOR_A_MID) + sineAmplitudeA * sin((2.0 * PI) * ((double)(timeA)/1000.0)*freqA); // Calculate sine wave setpoint
            motorControlA.move_to(convertMMToSteps(setpoint)); // Move motor A to the calculated setpoint
        }
        if (sineWaveActiveB) { // If sine wave movement is active for motor B
        // Serial.println("=========================== sin B active =================");
            unsigned long timeB = (millis() - startTimeB); // Calculate elapsed time in milliseconds
            double setpointB = sineAmplitudeB * sin((2.0 * PI) * ((double)(timeB)/1000.0)*freqB); // Calculate sine wave setpoint
            motorControlB.move_to(convertDegreesToSteps(setpointB)); // Move motor B to the calculated setpoint
        }
    }
}

void configurePins() {
    pinMode(STBY_PIN, OUTPUT); // Set standby pin as output
    digitalWrite(STBY_PIN, HIGH); // Set standby pin to HIGH to enable motor driver
}

void initializeMotors() {
    motorControlA.set_pid_tunings(MOTOR_A_P, MOTOR_A_I, MOTOR_A_D); // Set PID tunings for motor A
    motorControlB.set_pid_tunings(MOTOR_B_P, MOTOR_B_I, MOTOR_B_D); // Set PID tunings for motor B
    motorControlA.move_to(0); // Move motor A to initial position 0
    motorControlB.move_to(0); // Move motor B to initial position 0
}


void handleSerialInputOscar() {
    if (Serial.available()) { // Check if data is available in the serial buffer
        String command = Serial.readStringUntil('\n'); // Read the incoming command until newline
        char motorId = command.charAt(0); // Get the motor ID (A or B)
        double value = 0;
        String func = "";

        // Check for valid command length
        if (command.length() > 1) {
            // Parse sine wave command
            if (command.indexOf("sin") != -1) {
                func = "sin";
                value = command.substring(1, command.indexOf("sin")).toDouble();
            } 
            // Parse direct move command
            else if (isDigit(command.charAt(1)) || (command.charAt(1) == '-' && isDigit(command.charAt(2)))) {
                value = command.substring(1).toDouble();
            } 
            // Parse stop command
            else {
                func = command.substring(1);
            }
        }

        // Handle sine wave movement commands
        if (func == "sin") {
            if (motorId == 'A') {
                sineWaveActiveA = true; // Activate sine wave movement for motor A
                sineAmplitudeA = value; // Set amplitude for motor A
                startTimeA = millis(); // Set start time for motor A
                Serial.println("Motor A moving in sine wave with amplitude " + String(value) + " mm");
            } else if (motorId == 'B') {
                sineWaveActiveB = true; // Activate sine wave movement for motor B
                sineAmplitudeB = value; // Set amplitude for motor B
                startTimeB = millis(); // Set start time for motor B
                Serial.println("Motor B moving in sine wave with amplitude " + String(value) + " degrees");
            } else {
                Serial.println("Invalid motor ID. Use 'A' for Motor A and 'B' for Motor B.");
            }
        }
        // Handle stop commands
        else if (command == "Astop") {
            sineWaveActiveA = false; // Deactivate sine wave movement for motor A
            motorControlA.stop(); // Stop motor A
            Serial.println("Motor A stopped");
        } else if (command == "Bstop") {
            sineWaveActiveB = false; // Deactivate sine wave movement for motor B
            motorControlB.stop(); // Stop motor B
            Serial.println("Motor B stopped");
        } else if (command == "stop") {
            sineWaveActiveA = false; // Deactivate sine wave movement for motor A
            sineWaveActiveB = false; // Deactivate sine wave movement for motor B
            motorControlA.stop(); // Stop motor A
            motorControlB.stop(); // Stop motor B
            Serial.println("Both motors stopped");
        }
        // Handle direct move commands
        else if (isDigit(command.charAt(1)) || (command.charAt(1) == '-' && isDigit(command.charAt(2)))) {
            if (motorId == 'A') {
                sineWaveActiveA = false; // Deactivate sine wave movement for motor A
                motorControlA.move_to(convertMMToSteps(value)); // Move motor A to the setpoint
                Serial.println("Motor A moving to " + String(value) + " mm");
            } else if (motorId == 'B') {
                sineWaveActiveB = false; // Deactivate sine wave movement for motor B
                motorControlB.move_to(convertDegreesToSteps(value)); // Move motor B to the setpoint
                Serial.println("Motor B moving to " + String(value) + " degrees");
            } else {
                Serial.println("Invalid motor ID. Use 'A' for Motor A and 'B' for Motor B.");
            }
        } else {
            Serial.println("Invalid command format.");
        }
    }
}



void handleSerialInputKWC() {
    if (Serial.available()) { // Check if data is available in the serial buffer
        String command = Serial.readStringUntil('\n'); // Read the incoming command until newline
        char motorId = (char)32 | command.charAt(0); // Get the motor ID (A or B)
        double value = command.substring(1, command.length() - 3).toDouble(); // Get the amplitude or setpoint value
        String func = command.substring(command.length() - 3); // Get the function (e.g., "sin")
        
        if (motorId != 'a' && motorId != 'b') {
            Serial.print(motorId);
            Serial.println(" --- Invalid motor ID. Use 'A' for Motor A and 'B' for Motor B.");
        }
        else if (func == "sin") { // motor ID OK
            // If the function is sine wave
            if (motorId == 'a') { // If the command is for motor A
                sineWaveActiveA = true; // Activate sine wave movement for motor A
                sineAmplitudeA = value; // Set amplitude for motor A
                startTimeA = millis(); // Set start time for motor A
                Serial.println("Motor A moving in sine wave with amplitude " + String(value) + " mm");
            }
            if (motorId == 'b') { // If the command is for motor B
                sineWaveActiveB = true; // Activate sine wave movement for motor B
                sineAmplitudeB = value; // Set amplitude for motor B
                startTimeB = millis(); // Set start time for motor B
                Serial.println("Motor B moving in sine wave with amplitude " + String(value) + " degrees");
            }
        }
        else if (func == "top"){
            if (motorId == 'a') { // If the command is for motor A
                sineWaveActiveA = false; // Deactivate sine wave movement for motor A
                motorControlA.stop();
                Serial.println("** STOP Motor A **");
            } 
            if (motorId == 'b') { // If the command is for motor B
                sineWaveActiveB = false; // Deactivate sine wave movement for motor B
                motorControlB.stop();
                Serial.println("** STOP Motor B **");
            }
        }
        else {
               // If the command is a direct move command
            value = command.substring(1).toDouble(); // Get the setpoint value after the motor ID
            
            if (motorId == 'a') { // If the command is for motor A
                sineWaveActiveA = false; // Deactivate sine wave movement for motor A
                motorControlA.move_to(convertMMToSteps(value)); // Move motor A to the setpoint
                Serial.println("Motor A moving to " + String(value) + " mm");
            } 
            if (motorId == 'b') { // If the command is for motor B
                sineWaveActiveB = false; // Deactivate sine wave movement for motor B
                motorControlB.move_to(convertDegreesToSteps(value)); // Move motor B to the setpoint
                Serial.println("Motor B moving to " + String(value) + " degrees");
            }
       }
    }
}


void printMotorStatus() {
    double positionA = motorControlA.get_position(); // Get the current position of motor A
    double positionB = motorControlB.get_position(); // Get the current position of motor B
    double setpointA = motorControlA._pid_setpoint; // Get the current setpoint of motor A
    double setpointB = motorControlB._pid_setpoint; // Get the current setpoint of motor B

    static uint8_t lineCounter = 0;
    if (lineCounter == 0){
        Serial.println ("  == MOTOR A ==\t\t\t== MOTOR B ==");
        Serial.println ("-- set,  pos mm \t--\t set,  pos degrees --");
    }
    lineCounter++;
    if (lineCounter > 16) lineCounter = 0;
// added postitionA (in steps) just for debugging
    String output = String(convertStepsToMM(setpointA)) + "\t\t" + String(convertStepsToMM(positionA)) + " \t "+ String(positionA) + " \t**\t " + 
                    String(convertStepsToDegrees(setpointB)) + "\t\t" + String(convertStepsToDegrees(positionB));
    Serial.println(output);
}

// === unit conversions ===
double convertStepsToMM(double steps) {     //  MOTOR_A radius Drive
    // Convert steps to millimeters based on CPR, gear reduction ratio, and linear advance per revolution
    return (steps / (CPR * MOTOR_A_FINAL_RATIO)) * LINEAR_ADVANCE_PER_REVOLUTION_MM;
}

double convertMMToSteps(double mm) {
    // Convert millimeters to steps based on CPR, gear reduction ratio, and linear advance per revolution
    return (mm / LINEAR_ADVANCE_PER_REVOLUTION_MM) * (CPR * MOTOR_A_FINAL_RATIO);
}

double convertStepsToDegrees(double steps) {
    // Convert steps to degrees based on CPR and gear reduction ratio
    return (steps / (CPR * MOTOR_B_FINAL_RATIO)) * 360.0;
}

double convertDegreesToSteps(double degrees) {
    // Convert degrees to steps based on CPR and gear reduction ratio
    return (degrees * CPR * MOTOR_B_FINAL_RATIO / 360.0);
}
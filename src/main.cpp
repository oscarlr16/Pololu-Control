#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "DCMotorDriver.h"
#include "DCMotorControl.h"
#include "UartHandler.h"

#include <CommandParser.h>
//commandparser might replace uartHandler- need to test

// User Adjustable Variables
// You can adjust these variables according to your needs. Changing these values will affect how the motor operates.
// Position Limits mm
#define MOTOR_A_MAX 90.0
#define MOTOR_A_MIN 0.0
#define MOTOR_A_MID (MOTOR_A_MAX + MOTOR_A_MIN )/2.0
#define MOTOR_A_MAX_AMPLITUDE 90.0

// Position Limits degrees
#define MOTOR_B_MAX 37.0/2
#define MOTOR_B_MIN -37.0/2
#define MOTOR_B_MID 0.0
#define MOTOR_B_MAX_AMPLITUDE 37.0

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
#define MOTOR_A_P 0.2
#define MOTOR_A_I 0.0001
#define MOTOR_A_D 0.01

// PID tunings for Motor B
// starting point 0.7,0.09,0.01
#define MOTOR_B_P 0.3
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

// all of the template arguments below are optional, but it is useful to adjust them to save memory (by lowering the limits) or allow larger inputs (by increasing the limits)
// limit number of commands to at most 5
// limit number of arguments per command to at most 3
// limit length of command names to 10 characters
// limit size of all arguments to 15 bytes (e.g., the argument "\x41\x42\x43" uses 14 characters to represent the string but is actually only 3 bytes, 0x41, 0x42, and 0x43)
// limit size of response strings to 64 bytes
typedef CommandParser<10, 4, 10, 15, 64> MyCommandParser;
// or just the default values
//typedef CommandParser<> MyCommandParser;
MyCommandParser parser;

// ======== Global variables ========
// for sine wave control
bool sineWaveActiveA = false; // Indicates if sine wave movement is active for motor A
bool sineWaveActiveB = false; // Indicates if sine wave movement is active for motor B
double sineAmplitudeA = 0; // Amplitude for sine wave movement of motor A
double sineAmplitudeB = 0; // Amplitude for sine wave movement of motor B
unsigned long startTimeA; // Start time for sine wave movement of motor A
unsigned long startTimeB; // Start time for sine wave movement of motor B

// motor global vars
struct motorData_t
{
  bool sinActive;
  double amplitude;
  bool moveActive;
  double target;
  bool stop;
};

struct motorData_t mtrAdat;
struct motorData_t mtrBdat;

    double freqA = 0.2; // Hz
    double freqB = 0.2; // Hz

// other global vars
char buffer[70];
u_int64_t now;
u_int64_t reportInterval = 3000;
enum rMode {PLOT, TERMINAL, NONE};
enum rMode reportMode = PLOT;
u_int64_t sineInterval = 25;  // make the interval configurable

// ======== Function declarations ========
void configurePins();
void initializeMotors();
void handleSerialInputOscar();
void plotMotorStatus();
double convertStepsToMM(double steps);
double convertMMToSteps(double mm);
double convertStepsToDegrees(double steps);
double convertDegreesToSteps(double degrees);

// callback functions
void cmd_test(MyCommandParser::Argument *args, char *response);
void cmdHelp(MyCommandParser::Argument *args, char *response);
void cmdStop(MyCommandParser::Argument *args, char *response);
void cmdMove(MyCommandParser::Argument *args, char *response);
void cmdSin(MyCommandParser::Argument *args, char *response);
void cmdStatus(MyCommandParser::Argument *args, char *response);
void cmdFreq(MyCommandParser::Argument *args, char *response);
void cmdPID(MyCommandParser::Argument *args, char *response);
void cmdReport(MyCommandParser::Argument *args, char *response);

void motorControls(void);  // uses globals

void setup() {
    Serial.begin(115200); // Initialize serial communication at 115200 baud

    Serial.println("Booting");
    Serial.println(__DATE__);
    configurePins(); // Configure the necessary pins
    initializeMotors(); // Initialize the motors

    // cmd from code example
    parser.registerCommand("TEST", "sdiu", &cmd_test);
    // base commands
    parser.registerCommand("HELP", "",    &cmdHelp);
    parser.registerCommand("STOP", "s",   &cmdStop);
    parser.registerCommand("MOVE", "sd",  &cmdMove);
    parser.registerCommand("SIN", "sd",   &cmdSin);
    // config commands
    parser.registerCommand("STATUS", "",  &cmdStatus);
    parser.registerCommand("FREQ", "sd",  &cmdFreq);
    parser.registerCommand("PID", "sddd", &cmdPID);
    parser.registerCommand("REPORT", "su", &cmdReport);

    mtrAdat.stop = true;
    mtrBdat.stop = true;
}

void loop() {
    now = millis();

    static u_int64_t lastReport = 0;
    if (now > lastReport + reportInterval){
        lastReport = now;
        switch (reportMode){
        case PLOT:
            plotMotorStatus(); // Plot the current status of the motors
            break;
        case TERMINAL:
            // need a new function for terminal report
        break;
        default:  // no report
            break;
        }
    }

    // handleSerialInputOscar(); // Handle serial input from the user

    // NOW USING CommandParser
  if (Serial.available()) {
    char line[128];
    size_t lineLength = Serial.readBytesUntil('\n', line, 127);
    line[lineLength] = '\0';

    char response[MyCommandParser::MAX_RESPONSE_SIZE];
    parser.processCommand(line, response);
    Serial.println(response);
  }

  motorControls();   


// MOVED TO MOTOR CONTROLS
    // add update rate code
/*    u_int64_t sineInterval = 25;
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
    }*/
}

void configurePins() {
    pinMode(STBY_PIN, OUTPUT); // Set standby pin as output
    digitalWrite(STBY_PIN, HIGH); // Set standby pin to HIGH to enable motor driver
    pinMode(2,OUTPUT);  //LED
}

void initializeMotors() {
    motorControlA.set_pid_tunings(MOTOR_A_P, MOTOR_A_I, MOTOR_A_D); // Set PID tunings for motor A
    motorControlB.set_pid_tunings(MOTOR_B_P, MOTOR_B_I, MOTOR_B_D); // Set PID tunings for motor B
    motorControlA.move_to(0); // Move motor A to initial position 0
    motorControlB.move_to(0); // Move motor B to initial position 0
}

void motorControls(void)
{ // call this every loop.   Use millis to control sine update rate.
  
  // TEST PRINTING
//  sprintf(buffer, "A    %d    %d    %d    %3.3f     %3.3f",\
    mtrAdat.sinActive,  mtrAdat.moveActive,  mtrAdat.stop,  mtrAdat.amplitude, mtrAdat.target);
  //Serial.println(buffer);  

    if(mtrAdat.stop){
        mtrAdat.sinActive = false;
        mtrAdat.moveActive = false;
        motorControlA.stop();
    }
    if(mtrAdat.sinActive & mtrAdat.moveActive){ // this is an error and should never happen
        motorControlA.stop();
        mtrAdat.stop = true;
    }
    if(mtrAdat.moveActive){
        motorControlA.move_to(convertMMToSteps(mtrAdat.target)); // Move motor A to the setpoint
    }
    
    if(mtrBdat.stop){
        mtrBdat.sinActive = false;
        mtrBdat.moveActive = false;
        motorControlB.stop();
    }
    if(mtrBdat.sinActive & mtrBdat.moveActive){ // this is an error and should never happen
        motorControlB.stop();
        mtrBdat.stop = true;
    }
    if(mtrBdat.moveActive){
        motorControlB.move_to(convertDegreesToSteps(mtrBdat.target)); // Move to the setpoint
    }


    
    static u_int64_t lastSine = 0;
    if (now > lastSine + sineInterval){
        lastSine = now;
        if (mtrAdat.sinActive) { // If sine wave movement is active for motor A
            unsigned long timeA = (millis() - startTimeA); // Calculate elapsed time in milliseconds
            double setpoint = (double)(MOTOR_A_MID) + mtrAdat.amplitude/2.0 * sin((2.0 * PI) * ((double)(timeA)/1000.0)*freqA); // Calculate sine wave setpoint
            motorControlA.move_to(convertMMToSteps(setpoint)); // Move motor A to the calculated setpoint
        }
        if (mtrBdat.sinActive) { // If sine wave movement is active for motor B
        // Serial.println("=========================== sin B active =================");
            unsigned long timeB = (millis() - startTimeB); // Calculate elapsed time in milliseconds
            double setpointB = mtrBdat.amplitude/2.0 * sin((2.0 * PI) * ((double)(timeB)/1000.0)*freqB); // Calculate sine wave setpoint
            motorControlB.move_to(convertDegreesToSteps(setpointB)); // Move motor B to the calculated setpoint
        }
    }

  
}  // uses globals

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


void plotMotorStatus() {
    double positionA = motorControlA.get_position();
    double positionB = motorControlB.get_position();
    double setpointA = motorControlA._pid_setpoint;
    double setpointB = motorControlB._pid_setpoint;

    String output = "A_set:" + String(convertStepsToMM(setpointA), 2) + 
                    ",A_pos:" + String(convertStepsToMM(positionA), 2) +
                    ",A_steps:" + String(positionA, 0) +
                    ",B_set:" + String(convertStepsToDegrees(setpointB), 2) +
                    ",B_pos:" + String(convertStepsToDegrees(positionB), 2);

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


// callback functions  // need to improve text handling of most callbacks with sprintf and/or flash not RAM

  
void cmd_test(MyCommandParser::Argument *args, char *response) {
  Serial.print("string: "); Serial.println(args[0].asString);
  Serial.print("double: "); Serial.println(args[1].asDouble);
  Serial.print("int64: "); Serial.println((int32_t)args[2].asInt64); // NOTE: on older AVR-based boards, Serial doesn't support printing 64-bit values, so we'll cast it down to 32-bit
  Serial.print("uint64: "); Serial.println((uint32_t)args[3].asUInt64); // NOTE: on older AVR-based boards, Serial doesn't support printing 64-bit values, so we'll cast it down to 32-bit
  strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmdHelp(MyCommandParser::Argument *args, char *response) {
  Serial.print("\
  === HELP ===\n\r\
  STOP A or B or *\n\r\
  MOVE A or B float\n\r\
  SIN A or B float\n\r\
  STATUS\n\r\
  FREQ A or B float(Hz)\n\r\
  PID A or B float float float\n\r\
  ===\n\r");
  Serial.println("test command: TEST <string> <double> <int64> <uint64>");
  Serial.println("example: TEST \"\\x41bc\\ndef\" -1.234e5 -123 123");
  Serial.println("example: MOVE A 21.5");
} 

void cmdStop(MyCommandParser::Argument *args, char *response) // add motor commands
{
  char motor_s[3];
  snprintf(motor_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare
  
  switch(motor_s[0]){
    case '*':
      Serial.print("Stopping ALL  ");
      Serial.print("Stopping B  ");
      mtrBdat.sinActive = false;
      mtrBdat.moveActive = false;
      mtrBdat.stop = true;
      // fall through to stop A
    case 'a':
    case 'A':
      //set motor command flags here
      Serial.print("Stopping A");
      mtrAdat.sinActive = false;
      mtrAdat.moveActive = false;
      mtrAdat.stop = true;
      break;
    case 'b':
    case 'B':
      Serial.print("Stopping B");
      mtrBdat.sinActive = false;
      mtrBdat.moveActive = false;
      mtrBdat.stop = true;
      break;
    default:
      sprintf(buffer, "  %s is not a valid input \n\r\
      Use  A or B or * \n\r",args[0].asString);
      Serial.println(buffer);
  }  
}

void cmdMove(MyCommandParser::Argument *args, char *response) // fix filters, add motor commands
{
  double target = args[1].asDouble;;
  char motor_s[3];
  snprintf(motor_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare
  
  switch(motor_s[0]){
    case 'a':
    case 'A':
      // filter for min & max allowable target positions
      target = fmin(MOTOR_A_MAX, target);  
      target = fmax(MOTOR_A_MIN, target);  
      //set motor command flags here
      mtrAdat.sinActive = false;
      mtrAdat.moveActive = true;
      mtrAdat.stop = false;
      mtrAdat.target = target;
      break;
    case 'b':
    case 'B':
      // filter for min & max allowable target positions
      target = fmin(MOTOR_B_MAX, target);  
      target = fmax(MOTOR_B_MIN, target);
      mtrBdat.sinActive = false;
      mtrBdat.moveActive = true;
      mtrBdat.stop = false;
      mtrBdat.target = target;
      break;
    default:
      sprintf(buffer, "  %s is not a valid input \n\r\
      Use  A or B\n\r",args[0].asString);
      Serial.println(buffer);
      return;
  }
  sprintf(buffer, "Moving %s to  %3.3f", args[0].asString, target);
  Serial.println(buffer);
}

void cmdSin(MyCommandParser::Argument *args, char *response)  // rewrite sin compute,  FIX filters, allow negagive amplitude, add motor commands
{
  double amplitude = args[1].asDouble;;
  char motor_s[3];
  snprintf(motor_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare
  
  switch(motor_s[0]){
    case 'a':
    case 'A':
      // filter for min & max allowable positions
            // filter for min & max allowable target positions
      amplitude = fmin(MOTOR_A_MAX_AMPLITUDE, amplitude);  
      amplitude = fmax(-MOTOR_A_MAX_AMPLITUDE, amplitude); 
      mtrAdat.sinActive = true;
      mtrAdat.moveActive = false;
      mtrAdat.stop = false;
      mtrAdat.amplitude = amplitude;
      break;
    case 'b':
    case 'B':
          // filter for min & max allowable target positions
      amplitude = fmin(MOTOR_B_MAX_AMPLITUDE, amplitude);
      amplitude = fmax(-MOTOR_B_MAX_AMPLITUDE, amplitude);
      mtrBdat.sinActive = true;
      mtrBdat.moveActive = false;
      mtrBdat.stop = false;
      mtrBdat.amplitude = amplitude;

      // filter for min & max allowable positions
      //sprintf(buffer, "~Sin %s Amplitude %3.3f", args[0].asString, args[1].asDouble);
      //Serial.println(buffer);
      //set flags and motor commands here
      break;
    default:
      sprintf(buffer, "  %s is not a valid input \n\r\
      Use  A or B\n\r",args[0].asString);
      Serial.println(buffer);
      return;
  }

  sprintf(buffer, "~Sin %s Amplitude %3.3f", args[0].asString, amplitude);
  Serial.println(buffer);
};

void cmdStatus(MyCommandParser::Argument *args, char *response) // get the actual numbers
{
  Serial.println("\n=== Status report === PID numbers are bogus!!");
  
  sprintf(buffer, "kPa=%3.3f \tkIa=%3.3f\tkDa=%3.3f\t FreqA=%3.3f Hz",  5.0, 0.6, 0.07, freqA); /// get the actual numbers
  Serial.println(buffer);
  buffer[0] = 0;
  
  sprintf(buffer, "kPb=%3.3f \tkIb=%3.3f\tkDb=%3.3f\t FreqB=%3.3f Hz",  1.0, 2.0 , 3.0 , freqB); /// get the actual numbers
  Serial.println(buffer);
  buffer[0] = 0;

  sprintf(buffer, "sine interval = %d ms,  Report Interval = %d ms,  PID Interval ?? ms" ,  sineInterval, reportInterval ); /// get the actual numbers
  Serial.println(buffer);
  Serial.println(reportInterval);
}



void cmdFreq(MyCommandParser::Argument *args, char *response) // REFACTOR add filters and get the actual variables
{
  char motor_s[3];
  snprintf(motor_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare
  
  switch(motor_s[0]){
    case 'a':
    case 'A':
      // filter for min & max allowable positions
      sprintf(buffer, "Set Frequency %s %3.3f (Hz)", args[0].asString, args[1].asDouble);
      Serial.println(buffer);  
      freqA = args[1].asDouble;  // SET FREQ HERE   
      break;
    case 'b':
    case 'B':
      // filter for min & max allowable positions
      sprintf(buffer, "Set Frequency %s %3.3f (Hz)", args[0].asString, args[1].asDouble);
      Serial.println(buffer);
      freqB = args[1].asDouble; // SET FREQ HERE
      break;
    default:
      sprintf(buffer, "  %s is not a valid input \n\r\
      Use  A or B or * \n\r",args[0].asString);
      Serial.println(buffer);
  }  
}

#define TESTNUM 0.01
void cmdPID(MyCommandParser::Argument *args, char *response)// This is a mess. // get the actual numbers
{
  char motor_s[3];
  snprintf(motor_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare
  
  switch(motor_s[0]){
    case 'a':
    case 'A':
      sprintf(buffer, "Default PID values: %s kP=%3.3f \tkI=%3.3f\tkD=%3.3f from #define",\
      args[0].asString, TESTNUM,  4, 5.0);// get the actual numbers from #defines
      Serial.println(buffer);
      buffer[0] = 0;

      Serial.println("current A PID values: ");  /// get the actual numbers from PID code
      Serial.println("numbers here");

      // consider filtering for min & max allowable values 
      break;

    case 'b':
    case 'B':
      sprintf(buffer, "Default PID values: %s kP=%3.3f \tkI=%3.3f\tkD=%3.3f from #define",\
      args[0].asString, TESTNUM,  4, 5.0);// get the actual numbers from #defines
      Serial.println(buffer);
      buffer[0] = 0;

      Serial.println("current B PID values: ");  /// get the actual numbers from PID code
      Serial.println("numbers here");

      // consider filtering for min & max allowable values 
      break;

    default:
      sprintf(buffer, "  %s is not a valid input \n\r\
      Use  A or B\n\r",args[0].asString);
      Serial.println(buffer);
      return;
  }  

  sprintf(buffer, "    New PID values: %s kP=%3.3f \tkI=%3.3f\tkD=%3.3f",\
    args[0].asString, args[1].asDouble, args[2].asDouble,args[3].asDouble); 
  Serial.println(buffer);
};

void cmdReport(MyCommandParser::Argument *args, char *response)
{
// P 100 = Plot 100 ms rate
// T 500 = Terminal readable half second rate
// N = None

    reportInterval = args[1].asUInt64;

    char rMode_s[3];
    snprintf(rMode_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare

    switch(rMode_s[0]){
        case 'p':
        case 'P':
        sprintf(buffer, "=== Report === Plot Mode %d ms", args[1].asUInt64);
        Serial.println(buffer);  
        reportMode = PLOT;
        break;
        case 't':
        case 'T':
        sprintf(buffer, "=== Report === Terminal report mode %d ms", args[1].asUInt64);
        Serial.println(buffer);
        reportMode = TERMINAL;
        break;
        default:
        sprintf(buffer, "=== Report === reporting off  %s  \n\r\
        Use  P or T  \n\r",args[0].asString);
        Serial.println(buffer);
        reportMode =  NONE;
    }  

}

// EOF
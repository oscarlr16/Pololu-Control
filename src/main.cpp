#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "DCMotorDriver.h"
#include "DCMotorControl.h"
#include "UartHandler.h"
#include "motorConfig.h"

#include <CommandParser.h>
//commandparser might replace uartHandler- need to test

// Create UART and motor objects
HardwareSerial uart_0(0);
UartHandler uartHandler(uart_0);

// Configure Control Pins and Encoder Inputs for Motor A
// Note: rotation must be set for different gear ratios.
// see https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors-rev-6-1.pdf
//  For odd number of gear pairs. eg.  15:1 100:1, 150:1 up to 1000:1
//DCMotorDriver motorA(AIN1, AIN2, PWMA_PIN, 0, PINA_ENCODER_A, PINA_ENCODER_B); // PWM channel 0 for motor A
//  For even number of gear pairs. eg. 5:1, 10:1, 30:1, 50:1 etc.
DCMotorDriver motorA(AIN2, AIN1, PWMA_PIN, PWMA_CHANNEL, PINA_ENCODER_B, PINA_ENCODER_A);  // adjusted IN & ENCODER to switch rotation
DCMotorControl motorControlA(motorA);

DCMotorDriver motorB(BIN1, BIN2, PWMB_PIN, PWMB_CHANNEL, PINB_ENCODER_A, PINB_ENCODER_B); 
DCMotorControl motorControlB(motorB);

// all of the template arguments below are optional, but it is useful to adjust them to save memory (by lowering the limits) or allow larger inputs (by increasing the limits)
// number of commands,  number of arguments per command, length of command names (characters), size of all arguments (bytes), size of response strings (bytes)
// (e.g., the argument "\x41\x42\x43" uses 14 characters to represent the string but is actually only 3 bytes, 0x41, 0x42, and 0x43)
typedef CommandParser<10, 4, 10, 15, 64> MyCommandParser;
//typedef CommandParser<> MyCommandParser;   // or just use the default values
MyCommandParser parser;

// ======== Global variables ========
// for sine wave control
bool sineWaveActiveA = false; // Indicates if sine wave movement is active for motor A
bool sineWaveActiveB = false; // Indicates if sine wave movement is active for motor B
double sineAmplitudeA = 0; // Amplitude for sine wave movement of motor A
double sineAmplitudeB = 0; // Amplitude for sine wave movement of motor B
unsigned long startTimeA; // Start time for sine wave movement of motor A
unsigned long startTimeB; // Start time for sine wave movement of motor B
double phaseAoffset = 90.0;  // offset in degrees

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
char buffer[80];
u_int64_t now;
u_int64_t loopPeriod;
u_int64_t reportInterval = 2000;
enum rMode_t {PLOT, TERMINAL, NONE};
rMode_t reportMode = PLOT;
u_int64_t sineInterval = 10;  // make the interval configurable  WAS 25

// ======== Function declarations ========
void configurePins();
void initializeMotors();
//  depricated void handleSerialInputOscar();
void printMotorStatus();
double convertStepsToMM(double steps);
double convertMMToSteps(double mm);
double convertStepsToDegrees(double steps);
double convertDegreesToSteps(double degrees);
void printStatus(void);

// callback functions
//void cmd_test(MyCommandParser::Argument *args, char *response);
void cmdHelp(MyCommandParser::Argument *args, char *response);
void cmdStop(MyCommandParser::Argument *args, char *response);
void cmdMove(MyCommandParser::Argument *args, char *response);
void cmdSin(MyCommandParser::Argument *args, char *response);
void cmdStatus(MyCommandParser::Argument *args, char *response);
void cmdFreq(MyCommandParser::Argument *args, char *response);
void cmdPID(MyCommandParser::Argument *args, char *response);
void cmdReport(MyCommandParser::Argument *args, char *response);
void cmdZero(MyCommandParser::Argument *args, char *response);
void cmdPhase(MyCommandParser::Argument *args, char *response);


void motorControls(void);  // uses globals

void setup() {
    Serial.begin(115200); // Initialize serial communication at 115200 baud

    Serial.println("Booting");
    Serial.println(__DATE__);
    configurePins(); // Configure the necessary pins
    initializeMotors(); // Initialize the motors

    // cmd from code example
//    parser.registerCommand("TEST", "sdiu", &cmd_test);
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

    parser.registerCommand("ZERO", "s", &cmdZero);
    parser.registerCommand("PHASE", "d",  &cmdPhase);

    mtrAdat.stop = true;
    mtrBdat.stop = true;

    printStatus();

    motorControlA.set_pid_sample_time(2) ;
    motorControlB.set_pid_sample_time(2) ;
}

void loop() {
    uint64_t oldNow = now;
    now = millis();
    loopPeriod = now - oldNow;

    static u_int64_t lastReport = 0;
    if (now > lastReport + reportInterval){
      lastReport = now;
      printMotorStatus(); 
    }

  if (Serial.available()) {  // NOW USING CommandParser
    char line[128];
    size_t lineLength = Serial.readBytesUntil('\n', line, 127);
    line[lineLength] = '\0';

    char response[MyCommandParser::MAX_RESPONSE_SIZE];
    parser.processCommand(line, response);
    Serial.println(response);
  }

  motorControls();   
} // END OF loop

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

  double setpoint;
  static u_int64_t lastSine = 0;
  if (now > lastSine + sineInterval){
    lastSine = now;
    if (mtrAdat.sinActive) { // If sine wave movement is active for motor A
      unsigned long timeA = (millis() - startTimeA); // Calculate elapsed time in milliseconds
      setpoint = (double)(MOTOR_A_MID) + mtrAdat.amplitude/2.0 * sin((2.0 * PI) * ((double)(timeA)/1000.0)*freqA + (PI * phaseAoffset/360.0)); // Calculate sine wave setpoint
      //setpoint = (double)(MOTOR_A_MID) + mtrAdat.amplitude/2.0 * sin((2.0 * PI)  + (PI * phaseAoffset / 360.0)); // PHASE TEST
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

void printMotorStatus() {
  double positionA = motorControlA.get_position();
  double positionB = motorControlB.get_position();
  double setpointA = motorControlA._pid_setpoint;
  double setpointB = motorControlB._pid_setpoint;

  String statOutput;

  switch (reportMode)
  {
  case PLOT:  // Plot for https://sekigon-gonnoc.github.io/web-serial-plotter/   CSV with labels  OR Arduino Serial Plotter
    statOutput = String(
                    "A_set:" + String(convertStepsToMM(setpointA), 2) +
                    ",A_pos:" + String(convertStepsToMM(positionA), 2) +
                    //",A_steps:" + String(positionA, 0) +
                    ",B_set:" + String(convertStepsToDegrees(setpointB), 2) +
                    ",B_pos:" + String(convertStepsToDegrees(positionB), 2));
    Serial.println(statOutput);
    break;

  case TERMINAL:
    statOutput = String(
                    String(convertStepsToMM(setpointA), 2) + "\t\t" +
                    String(convertStepsToMM(positionA), 2) + "\t\t" +
                    String(convertStepsToDegrees(setpointB), 2) + "\t\t" +
                    String(convertStepsToDegrees(positionB), 2) +"\t|| \t" +
                    String(loopPeriod));
    Serial.println(statOutput);
    break;

  default:
    break;
  }

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

/*  cmd_test from parser library
void cmd_test(MyCommandParser::Argument *args, char *response) {
  Serial.print("string: "); Serial.println(args[0].asString);
  Serial.print("double: "); Serial.println(args[1].asDouble);
  Serial.print("int64: "); Serial.println((int32_t)args[2].asInt64); // NOTE: on older AVR-based boards, Serial doesn't support printing 64-bit values, so we'll cast it down to 32-bit
  Serial.print("uint64: "); Serial.println((uint32_t)args[3].asUInt64); // NOTE: on older AVR-based boards, Serial doesn't support printing 64-bit values, so we'll cast it down to 32-bit
  strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
}
*/

void cmdHelp(MyCommandParser::Argument *args, char *response) {
  Serial.print("\
  === HELP ===\n\r\
  STOP A or B or *\n\r\
  MOVE A or B float\n\r\
  SIN A or B float\n\r\
  STATUS\n\r\
  FREQ A or B float(Hz)\n\r\
  PID A or B float float float\n\r\
  REPORT P or T or N, int    Plot, Terminal, None \n\r\
  ZERO A or B or *\n\r\
  PHASE float degrees"
  );
  /*Serial.println("test command: TEST <string> <double> <int64> <uint64>");
  Serial.println("example: TEST \"\\x41bc\\ndef\" -1.234e5 -123 123");
  Serial.println("example: MOVE A 21.5");*/
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

void cmdStatus(MyCommandParser::Argument *args, char *response) 
{
  printStatus();
}

void printStatus() // work around so that setup() can print STATUS at boot time. (without confusing parameter issues)
{
  double tempP; double tempI; double tempD;
  
  Serial.println("\n===== Status report ===== ");
  
  motorControlA.get_pid_tunings( tempP,  tempI,  tempD);
  sprintf(buffer, "kPa=%3.3f \tkIa=%3.3f\tkDa=%3.3f\t FreqA=%3.3f Hz",  tempP,  tempI,  tempD, freqA); /// get the actual numbers
  Serial.println(buffer);
  buffer[0] = 0;
  
  motorControlB.get_pid_tunings( tempP,  tempI,  tempD);
  sprintf(buffer, "kPb=%3.3f \tkIb=%3.3f\tkDb=%3.3f\t FreqB=%3.3f Hz",  tempP,  tempI,  tempD , freqB); /// get the actual numbers
  Serial.println(buffer);
  buffer[0] = 0;

  sprintf(buffer, "sine interval = %d ms" ,  sineInterval); 
  Serial.println(buffer);

  sprintf(buffer, "Report Interval = %d ms" ,  reportInterval ); 
  Serial.println(buffer);

  sprintf(buffer, "PID Interval ?? ms UNKNOWN"); /// get the actual numbers
  Serial.println(buffer);
  
  sprintf(buffer, "freqA = %f Hz" ,  freqA ); 
  Serial.println(buffer);
  sprintf(buffer, "freqB = %f Hz" ,  freqB ); 
  Serial.println(buffer);

}

void cmdFreq(MyCommandParser::Argument *args, char *response) //  add min/max filters
{
  char motor_s[3];
  snprintf(motor_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare
  
  switch(motor_s[0]){
    case 'a':
    case 'A':
      // filter for min & max
      sprintf(buffer, "Set Frequency %s %3.3f (Hz)", args[0].asString, args[1].asDouble);  
      freqA = args[1].asDouble;  // SET FREQ HERE   
      break;
    case 'b':
    case 'B':
      // filter for min & max
      sprintf(buffer, "Set Frequency %s %3.3f (Hz)", args[0].asString, args[1].asDouble);
      //Serial.println(buffer);
      freqB = args[1].asDouble; // SET FREQ HERE
      break;
    default:
      sprintf(buffer, "  %s is not a valid input \n\r\
      Use  A or B or * \n\r",args[0].asString);
      //Serial.println(buffer);
  }
  Serial.println(buffer);  
}


void cmdPID(MyCommandParser::Argument *args, char *response)// This is a mess. // get the actual numbers
{
  double tempP; double tempI; double tempD;
  char motor_s[3];
  snprintf(motor_s, 2, "%s", args[0].asString );  // make args[0] a char array so that switch has integers to compare
  
  switch(motor_s[0]){
    case 'a':
    case 'A':
      sprintf(buffer, "  Default PID values: %s  kP=%3.3f \tkI=%3.3f\tkD=%3.3f from #define",\
      args[0].asString, MOTOR_A_P, MOTOR_A_I, MOTOR_A_D);// get the actual numbers from #defines
      Serial.println(buffer);
      //buffer[0] = 0;

      motorControlA.get_pid_tunings( tempP,  tempI,  tempD);
      sprintf(buffer, "Current A PID values: %s  kP=%3.3f \tkI=%3.3f\tkD=%3.3f",\
      args[0].asString,  tempP, tempI, tempD );// get the actual numbers from #defines
      Serial.println(buffer);

      // consider filtering for min & max allowable values 
      motorControlA.set_pid_tunings(args[1].asDouble, args[2].asDouble, args[3].asDouble); // Set PID tunings
      break;

    case 'b':
    case 'B':
      sprintf(buffer, "  Default PID values: %s  kP=%3.3f \tkI=%3.3f\tkD=%3.3f from #define",\
      args[0].asString,  MOTOR_B_P, MOTOR_B_I, MOTOR_B_D);// get the actual numbers from #defines
      Serial.println(buffer);
      //buffer[0] = 0;

      motorControlB.get_pid_tunings( tempP,  tempI,  tempD);
      sprintf(buffer, "Current A PID values: %s  kP=%3.3f \tkI=%3.3f\tkD=%3.3f",\
      args[0].asString,  tempP, tempI, tempD );// get the actual numbers from #defines
      Serial.println(buffer);

      // consider filtering for min & max allowable values 
      motorControlB.set_pid_tunings(args[1].asDouble, args[2].asDouble, args[3].asDouble); // Set PID tunings
      break;

    default:
      sprintf(buffer, "  %s is not a valid input \n\r\
      Use  A or B\n\r",args[0].asString);
      Serial.println(buffer);
      return;
  }  

  sprintf(buffer, "      New PID values: %s  kP=%3.3f \tkI=%3.3f\tkD=%3.3f",\
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
        reportMode = PLOT;
        sprintf(buffer, "=== Report === Plot Mode %d ms", args[1].asUInt64);
        break;
      case 't':
      case 'T':
        reportMode = TERMINAL;
        Serial.println(buffer);
        sprintf(buffer, "=== Report === Terminal report mode %d ms", args[1].asUInt64);
        break;    
      default:
        Serial.print("=== Report === reporting off");
        sprintf(buffer, " - %s\n\r\
        Use P for WebPlot, T for Terminal, A for ArduinoPlot\n\r",args[0].asString);
        Serial.println(buffer);
        reportMode =  NONE;
    }  
    Serial.println(buffer);  
}

void cmdZero(MyCommandParser::Argument *args, char *response) // add motor commands
{
  Serial.println("This does nothing. Sorry");
}

void cmdPhase(MyCommandParser::Argument *args, char *response)
{
  phaseAoffset = args[0].asDouble;
  sprintf(buffer, "Phase offset %3.3f", phaseAoffset);
  Serial.println(buffer);
};

// EOF
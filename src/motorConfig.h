/*  motorConfig.h 
*   Motor configurations MH04 project*/

// Counts per revolution (CPR) of the motor encoders
#define CPR 12 

/* ====== MOTOR_A is the radius Drive ======*/
// motor A ratios 4.995, 9.96 , 15.25
#define MOTOR_A_GEAR_RATIO 9.96
#define MOTOR_A_SHAFT_RATIO 20/14
#define MOTOR_A_FINAL_RATIO MOTOR_A_GEAR_RATIO*MOTOR_A_SHAFT_RATIO
// Linear advance per main shaft revolution in mm
#define LINEAR_ADVANCE_PER_REVOLUTION_MM 8.0 

// Position Limits mm
// Origin (Zero) is fully retracted
// 90 is desired 75 is for testing
// #define MOTOR_A_MAX 90.0
#define MOTOR_A_MAX 75.0
#define MOTOR_A_MIN 0.0
#define MOTOR_A_MID (MOTOR_A_MAX + MOTOR_A_MIN )/2.0
#define MOTOR_A_MAX_AMPLITUDE MOTOR_A_MAX - MOTOR_A_MIN

/* ====== MOTOR_B is the Angle Drive ======*/
// motor B ratios 248.98, 297.92, 379.17
#define MOTOR_B_GEAR_RATIO 297.92
#define MOTOR_B_BELT_RATIO 50/12
#define MOTOR_B_FINAL_RATIO MOTOR_B_GEAR_RATIO*MOTOR_B_BELT_RATIO

// Position Limits degrees
// Origin (Zero) is the center line
#define MOTOR_B_MAX_AMPLITUDE 37.0
#define MOTOR_B_MAX MOTOR_B_MAX_AMPLITUDE/2
#define MOTOR_B_MIN -MOTOR_B_MAX_AMPLITUDE/2
#define MOTOR_B_MID 0.0


// initial PID tunings for Motor A (adjuatable)
#define MOTOR_A_P 0.28
#define MOTOR_A_I 0.15
#define MOTOR_A_D 0.01

// initial PID tunings for Motor B (adjuatable)
// starting point 0.7,0.09,0.01
#define MOTOR_B_P 0.3
#define MOTOR_B_I 0.0
#define MOTOR_B_D 0.0

// --- End of User Adjustable Variables ---

#ifndef ROBOT_H
#define ROBOT_H

#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <Wire.h> 
#include "utils.h"
#include "sensors.h"

#define PI 3.14159265359
#define Motor_L_f D3
#define Motor_L_r D2
#define Motor_R_f D4
#define Motor_R_r D5
#define SENSOR_PERIOD 0.020
#define SERIAL_BAUDRATE 115200
#define I2C_CLOCK_SPEED 800000L

#define ENCODER_L 2
#define ENCODER_R 7

#define K_COMP 0.68

#define WHEEL_RADIUS 0.040

struct K {
  float Kp = 0.0;
  float Ki = 0.0;
  float Kd = 0.0;
};

struct Error {
  float e = 0.0;
  float e_integ = 0.0;
  float e_prev = 0.0;
  float e_delta = 0.0;
  float e_delta_prev = 0.0;
}

K Kt;
K Kx;

// Encoder variables
int magnetStatus = 0;                                   //value of the status register (MD, ML, MH)
float degAngle; 
int rotations = 0;

float start_angle = 0;                                   // starting angle                          // for the display printing
float prev_angle = 0.0;
float wheel_angle = 0.0;

int quad_num = 0;                                 // quadrant IDs
int prev_quad_num = 0;                         // these are used for tracking the num_turns
float encoderTimer = 0;

// IMU data variables
float deltaT = 1;
float theta = 0.0, theta_prev = 0.0, theta_dot = 0.0, theta_integ = 0.0, theta_error =0.0, theta_prev_error = 0.0, theta_dot_prev = 0.0;
float x_vec[4];
float u =0.0;

int control_mode = 0; // Manual by default
int xspeed = 0;
int light_delay = 0;

float x = 0.0, x_prev = 0.0, x_dot = 0.0, x_integ = 0.0, x_error = 0.0, x_prev_error = 0.0, x_dot_prev = 0.0;
// PID parameters
float Kc = 0.0;
float previousError = 0.0;
float integral = 0.0;
unsigned long previousTime = 0;
float PID_output_max = 100;

// FIR Filter parameters
#define FILTER_ORDER 10  
#define Beta 0.24       
#define LAMBDA 0.5       // Decay rate
unsigned long t0;
unsigned long t1;
unsigned long dt;
float accelBuffer[FILTER_ORDER] = {0};  // Buffer to store past values
float previousValue = 0;

TimerDecorator imutimer("IMU_TIMER");

// Robot state machine
enum mode {
  IDLE, RUN
};

// BLE parameters
BLEService nanoService("13012F00-F8C3-4F4A-A8F4-15CD926DA146");
BLEStringCharacteristic pitch_char("13012F01-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic speed_char("13012F02-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16); 
BLEStringCharacteristic yaw_char("13012F03-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);

BLEStringCharacteristic control_com("13012F06-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K1_com("13012F07-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K2_com("13012F08-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K3_com("13012F09-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K4_com("13012F10-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K5_com("13012F11-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K6_com("13012F12-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K7_com("13012F13-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);

#endif
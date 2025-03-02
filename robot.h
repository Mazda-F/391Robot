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

// Encoder variables
int magnetStatus = 0;                                   //value of the status register (MD, ML, MH)
int lowbyte;                                            //raw angle bits[7:0]
word highbyte;                                          //raw angle bits[11:8]
int rawAngle;                                           //final raw angle bits[11:0]
float degAngle; 
const int sampleInterval = 9;                           // degrees
const int samples = int(360 / sampleInterval + 2);
float calibrationTable[samples];                        // for recording magnet angles every "sampleInterval" degrees
float interpolatedAngle;
float correctedAngle = 0;

float num_turns = 0;                                    // number of turns
float startAngle = 0;                                   // starting angle
float taredAngle = 0;                                   // tared angle - based on the startup value
float totalAngle = 0;                                   // total absolute angular displacement
float previousTotalAngle = 0;                           // for the display printing

int quad_num = 0;                                 // quadrant IDs
int prev_quad_num = 0;                         // these are used for tracking the num_turns
float encoderTimer = 0;

// Sensor data variables
float accelTheta;
float theta0;
float pitch;
float gyro_sample_rate;
float ax, ay, az, gx, gy, gz;
float deltaT;

int control_mode = 0; // Manual by default
int xspeed = 0;
int light_delay = 0;

// PID parameters
float Kp = 0.0, Ki = 0.0, Kd = 0.0;
float previousError = 0.0;
float integral = 0.0;
float k_comp = 0.68;
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

TimerDecorator imutimer("IMU_TIMER");

// Robot state machine
enum mode {
  IDLE, RUN
};

// BLE parameters
BLEService nanoService("13012F00-F8C3-4F4A-A8F4-15CD926DA146");
BLEStringCharacteristic pitchAngleCharacteristic("13012F01-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic speedCommandCharacteristic("13012F02-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16); 
BLEStringCharacteristic yawCommandCharacteristic("13012F03-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);

BLEStringCharacteristic control_com("13012F06-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic Kp_com("13012F07-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic Ki_com("13012F08-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic Kd_com("13012F09-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
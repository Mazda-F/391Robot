#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <Wire.h> 
#include "utils.h"
// #include "sensors.h"

#define PI 3.14159265359

#define Motor_L_f D3
#define Motor_L_r D2
#define Motor_R_f D4
#define Motor_R_r D5
#define SENSOR_PERIOD 0.020

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
float numberofTurns = 0;

float numberOfTurns = 0;                                // number of turns
float startAngle = 0;                                   // starting angle
float taredAngle = 0;                                   // tared angle - based on the startup value
float totalAngle = 0;                                   // total absolute angular displacement
float previousTotalAngle = 0;                           // for the display printing

int quadrantNumber = 0;                                 // quadrant IDs
int previousquadrantNumber = 0;                         // these are used for tracking the numberOfTurns
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
unsigned long t0;
unsigned long t1;
unsigned long dt;



TimerDecorator imutimer("IMU_TIMER");

//FIR Filter parameters
#define FILTER_ORDER 10  // Number of past samples
#define LAMBDA 0.5       // Decay rate
float accelBuffer[FILTER_ORDER] = {0};  // Buffer to store past values

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


void setup() {
  Serial.begin(115200);

  // start i2C with fast clock
  Wire.begin();                                        
  Wire.setClock(800000L);  
  checkMagnetPresence(); 
  ReadRawAngle();     
  startAngle = degAngle;                     

  if(!IMU.begin()){
    Serial.println("IMU Initialization Failed.");
    while(1);
  }

  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth Low Energy Module Failed.");
    while (1);
  }

  pinMode(Motor_L_f, OUTPUT);
  pinMode(Motor_L_r, OUTPUT);
  pinMode(Motor_R_f, OUTPUT);
  pinMode(Motor_R_r, OUTPUT);  

  // BLE Setup
  BLE.setLocalName("ROBOT_C4");
  BLE.setAdvertisedService(nanoService);
  nanoService.addCharacteristic(pitchAngleCharacteristic);
  nanoService.addCharacteristic(speedCommandCharacteristic);
  nanoService.addCharacteristic(yawCommandCharacteristic);
  nanoService.addCharacteristic(control_com);
  nanoService.addCharacteristic(Kp_com);
  nanoService.addCharacteristic(Ki_com);
  nanoService.addCharacteristic(Kd_com);
  BLE.addService(nanoService);
  BLE.advertise();
  Serial.println("BLE advertising...");
}

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &gyroSampleRate) {
    unsigned long t0, t1;
    t0 = millis();
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    gyroSampleRate = IMU.gyroscopeSampleRate();

}

float FIR(float newSample) {
    float sum = 0, weightSum = 0;

    // Shift buffer and compute weighted sum
    for (int i = FILTER_ORDER - 1; i > 0; i--) {
        accelBuffer[i] = accelBuffer[i - 1];
        float weight = exp(-LAMBDA * i);
        sum += accelBuffer[i] * weight;
        weightSum += weight;
    }

    accelBuffer[0] = newSample;
    sum += newSample;
    weightSum += 1;             

    return sum / weightSum;
}



void pidLoop() {
    unsigned long currentTime = millis();
    deltaT = (currentTime - previousTime) / 1000.0; // Convert to seconds
    previousTime = currentTime;

    // imutimer(readIMU, ax, ay, az, gx, gy, gz, gyro_sample_rate);
    readIMU(ax, ay, az, gx, gy, gz, gyro_sample_rate);
 
    // Angle calculation and complementary filter
    accelTheta = atan(ay/az) * (180/PI);
    float samplePeriod = 1 / gyro_sample_rate;
    float gyroTheta = accelTheta + gz * samplePeriod;
    float compTheta = k_comp * (gyroTheta) + (1-k_comp) * accelTheta + 2.0;
    //float FIR_Theta = FIR(compTheta);

    // PID calculations
    float setpoint = 0.0; // Target angle
    float error = setpoint - compTheta;
    integral += error * deltaT;
    float derivative = (error - previousError) / deltaT;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    // Reset integral if the error is very small
    if (abs(error) < 3) {
        integral = 0;
    }

    int motorSpeed = map(abs(output), 0, 100, 0, 255);
    if (control_mode) {
      motorSpeed = constrain(motorSpeed, 0, 255);
    }
    else {
      motorSpeed = 0;
    }

    if (output > 0) {
        analogWrite(Motor_L_r, motorSpeed);
        analogWrite(Motor_R_r, motorSpeed);
        analogWrite(Motor_L_f, 0);
        analogWrite(Motor_R_f, 0);
    } else {
        analogWrite(Motor_L_f, motorSpeed);
        analogWrite(Motor_R_f, motorSpeed);
        analogWrite(Motor_L_r, 0);
        analogWrite(Motor_R_r, 0);
    }
}

void readBluetoothBLE() {
  String control_mode_str = control_com.value();
  String Kp_str = Kp_com.value();
  String Ki_str = Ki_com.value();
  String Kd_str = Kd_com.value();
  control_mode = control_mode_str.toInt();
  float Kp_in = Kp_str.toFloat();
  float Ki_in = Ki_str.toFloat();
  float Kd_in = Kd_str.toFloat();
  
  // if (isnan(Kp_in)) Kp_in = 0;
  // if (isnan(Ki_in)) Ki_in = 0;
  // if (isnan(Kd_in)) Kd_in = 0;
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;
}



void loop() {
  BLEDevice central = BLE.central(); 
  // if a central is connected to peripheral:
    ReadRawAngle(); //ask the value from the sensor
    correctAngle(); //tare the value
    checkQuadrant();
    Serial.print("Raw Angle: ");
    Serial.print(rawAngle);
    Serial.print(" deg Angle: ");
    Serial.println(degAngle);
    delay(1000);

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {  
      // MAIN LOOP RUNTIME <= 25 ms
      // t0 = micros();
      pidLoop();
      readBluetoothBLE();
      // t1 = micros();
      // dt = t1 - t0;
      // Serial.println(dt);
      
    }

    Serial.println("Central device disconnected!");
  } 
}


void ReadRawAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
}

void correctAngle()
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}

void checkQuadrant()
{
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(correctedAngle >= 0 && correctedAngle <=90){
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180){
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270){
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360){
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4){
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1){
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition
    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}

void checkMagnetPresence() {  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!");
  delay(1000);  
}

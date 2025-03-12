#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include "sensors.h"

#define SERIAL_BAUDRATE 115200
#define I2C_CLOCK_SPEED 800000L
#define ENCODER_L 2
#define ENCODER_R 7
#define Motor_R_f D3
#define Motor_R_r D2
#define Motor_L_f D4
#define Motor_L_r D5
#define ENCODER_ADDRESS 0x36

float deg_angle, start_angle, prev_angle, x, corrected_angle, total_angle;
int magnetStatus;
unsigned long t0, t1;
int rots = 0;
int quad_num = 0;
int prev_quad_num = 0;
float num_turns = 0;
float wheel_angle = 0;
float maxx = 0;
int raw_angle;

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("Serial Started ...");
    Wire.begin(); // i2c                                        
    Wire.setClock(I2C_CLOCK_SPEED);  

    Serial.println("Calibrating Encoders...");
    // checkMagnetPresence(&magnetStatus); 
    deg_angle = ReadRawAngle(ENCODER_L);   
    start_angle = deg_angle;  
    prev_angle = start_angle; 
    total_angle = 0;               

    if (!IMU.begin()) { Serial.println("IMU Initialization Failed."); while(1); }
}

void loop() {

  Wire.beginTransmission(ENCODER_ADDRESS); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(ENCODER_ADDRESS, 1); //request from the sensor
  while(Wire.available() == 0); //wait until it becomes available 
  int lowbyte = Wire.read(); //Reading the data after the request
  //11:8 - 4 bits
  Wire.beginTransmission(ENCODER_ADDRESS);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(ENCODER_ADDRESS, 1);
  
  while(Wire.available() == 0);  
  word highbyte = Wire.read();
  highbyte = highbyte << 8; 
  raw_angle = highbyte | lowbyte; //int is 16 bits (as well as the word)
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  deg_angle = (raw_angle) * 0.087890625; 

  corrected_angle = deg_angle - start_angle;
  if(corrected_angle < 0) { 
    corrected_angle = corrected_angle + 360.0; 
  }

  if(corrected_angle >= 0 && corrected_angle <=90) quad_num = 1;
  if(corrected_angle > 90 && corrected_angle <=180) quad_num = 2;
  if(corrected_angle > 180 && corrected_angle <=270) quad_num = 3;
  if(corrected_angle > 270 && corrected_angle <360) quad_num = 4;
  int qn = quad_num;
  int prev_qn = prev_quad_num;
  if(qn != prev_qn) {   //if we changed quadrant
      if(qn == 1 && prev_qn == 4){
          num_turns++; // 4 --> 1 transition: CW rotation
      }
      if(qn == 4 && prev_qn == 1){
          num_turns--; // 1 --> 4 transition: CCW rotation
      }
      //this could be done between every quadrants so one can count every 1/4th of transition
      prev_quad_num = quad_num;  //update to the current quadrant
  }  
  total_angle = (num_turns*360) + corrected_angle;



  x = total_angle * 3.14159265/180.0 * 0.04;
  if (total_angle > maxx) {
    maxx = total_angle;
  }
  Serial.print(x); Serial.print("  "); 
  Serial.print(total_angle); Serial.print("  "); 
  Serial.print(quad_num); Serial.print("  "); 
  Serial.print(prev_quad_num); Serial.print("  "); 
  Serial.print(maxx); Serial.print("  "); 
  Serial.println(num_turns);
}
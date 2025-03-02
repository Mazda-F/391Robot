#include "sensors.h"

void checkMagnetPresence(int* magnetStatus) {  

    Serial.println("Finding Magnets...");
    //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly
    while((*magnetStatus & 32) != 32) { //while the magnet is not adjusted to the proper distance - 32: MD = 1
        *magnetStatus = 0; //reset reading
        Wire.beginTransmission(ENCODER_ADDRESS); //connect to the sensor
        Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
        Wire.endTransmission(); //end transmission
        Wire.requestFrom(0x36, 1); //request from the sensor
        while(Wire.available() == 0); //wait until it becomes available 
        *magnetStatus = Wire.read(); //Reading the data after the request
        //Serial.print("Magnet status: ");
        //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
    }      
    //Status register output: 0 0 MD ML MH 0 0 0  
    //MH: Too strong magnet - 100111 - DEC: 39 
    //ML: Too weak magnet - 10111 - DEC: 23     
    //MD: OK magnet - 110111 - DEC: 55
    Serial.println("Magnet found!");
    delay(1000);  
}

void checkQuadrant(float* corrected_angle, float* total_angle, float* num_turns, int* quad_num, int* prev_quad_num) {
    /*
    //Quadrants:
    4  |  1
    ---|---
    3  |  2
    */
    if(*corrected_angle >= 0 && *corrected_angle <=90) *quad_num = 1;
    if(*corrected_angle > 90 && *corrected_angle <=180) *quad_num = 2;
    if(*corrected_angle > 180 && *corrected_angle <=270) *quad_num = 3;
    if(*corrected_angle > 270 && *corrected_angle <360) *quad_num = 4;
    int qn = *quad_num;
    int prev_qn = *prev_quad_num;
    if(qn != prev_qn) {//if we changed quadrant
        if(qn == 1 && prev_qn == 4){
            *num_turns++; // 4 --> 1 transition: CW rotation
        }
        if(qn == 4 && prev_qn == 1){
            *num_turns--; // 1 --> 4 transition: CCW rotation
        }
        //this could be done between every quadrants so one can count every 1/4th of transition
        *prev_quad_num = *quad_num;  //update to the current quadrant
    }  
    *total_angle = (*num_turns*360) + *corrected_angle; //number of turns (+/-) plus the actual angle within the 0-360 range
}

/*
    #### Params:
    Input:
    Output: `corrected_angle` 
*/
void correctAngle(float* corrected_angle, float* deg_angle, float* start_angle) {
  
  *corrected_angle = *deg_angle - *start_angle; //this tares the position
  if(*corrected_angle < 0) { //if the calculated angle is negative, we need to "normalize" it
    *corrected_angle = *corrected_angle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else {
  }
}

/*
    ReadRawAngle
    Reads the raw angle from the magnetic encoder
*/
void ReadRawAngle(int* rawAngle, float* degAngle) { 
  //7:0 - bits
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
  *rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  *degAngle = (*rawAngle) * 0.087890625; 
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
}
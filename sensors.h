#ifndef SENSORS_H
#define SENSORS_H

#define ENCODER_ADDRESS 0x36

#include <ArduinoBLE.h>
#include <Wire.h> 

void checkMagnetPresence(int* magnetStatus);
float ReadRawAngle(uint8_t bus);
void getWheelAngle(float* total_angle, float* num_turns, int* quad_num, int* prev_quad_num, float start_angle);
#endif
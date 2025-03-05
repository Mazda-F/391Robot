#ifndef SENSORS_H
#define SENSORS_H

#define ENCODER_ADDRESS 0x36

#include <Arduino.h>
#include <Wire.h> 


void checkMagnetPresence(int* magnetStatus);
void checkQuadrant(float* corrected_angle, float* total_angle, float* num_turns, int* quad_num, int* prev_quad_num);
float ReadRawAngle(uint8_t bus);
float correctAngle(float deg_angle, float start_angle);
float getAngle(float start_angle, uint8_t bus);
float getDisplacement(int rotations, float prev_displacement, float prev_angle, float wheel_angle, float wheel_radius);

#endif
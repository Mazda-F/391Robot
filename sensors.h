#ifndef SENSORS_H
#define SENSORS_H

#define ENCODER_ADDRESS 0x36

#include <Arduino.h>
#include <Wire.h> 

void checkMagnetPresence(int* magnetStatus);
void checkQuadrant(float* corrected_angle, float* total_angle, float* num_turns, int* quad_num, int* prev_quad_num);
void correctAngle(float* corrected_angle, float* deg_angle, float* start_angle);
void ReadRawAngle(int* rawAngle, float* degAngle);

#endif
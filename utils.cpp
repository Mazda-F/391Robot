#include "utils.h"

TimerDecorator::TimerDecorator(const char* id) : function_id(id) {}

void Kprint(float Kp, float Ki, float Kd) {
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print(" Ki: ");
    Serial.print(Ki);
    Serial.print(" Kd: ");
    Serial.print(Kd);
    Serial.println("  ");
}

void updatePIDfromSerial(float* Kp, float* Ki, float* Kd, float* integral, 
    float* prev_error, float* prev_time, float* PID_output_max) {
      
      if (Serial.available()) {
           //read the input until newline
          String input = Serial.readStringUntil('\n'); 
          //remove whitespace
          input.trim();  
          float newKp, newKi, newKd, newOUTPUT;
  
          int valuesParsed = sscanf(input.c_str(), "%f %f %f %f", &newKp, &newKi, &newKd, &newOUTPUT);
  
          if (valuesParsed == 4) { 
              *Kp = newKp;
              *Ki = newKi;
              *Kd = newKd;
              *PID_output_max = newOUTPUT;
  
              *integral = 0;
              *prev_error = 0;
              *prev_time = millis();
  
              Serial.print("PID Updated: Kp = ");
              Serial.print(*Kp);
              Serial.print(", Ki = ");
              Serial.print(*Ki);
              Serial.print(", Kd = ");
              Serial.println(*Kd);
              Serial.print(", Output Max = ");
              Serial.print(newOUTPUT);
          } else {
              Serial.println("Invalid input. Enter values as: Kp Ki Kd Output");
          }
      }
  }
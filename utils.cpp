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

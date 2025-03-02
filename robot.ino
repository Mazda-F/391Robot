#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>

#define PI 3.1415926535897932384626433832795

#define Motor_L_f D3
#define Motor_L_r D2
#define Motor_R_f D4
#define Motor_R_r D5

float accelTheta;
float theta0;
float pitch;
int control_mode = 0; // Manual by default
int xspeed = 0;
int light_delay = 0;

// PID parameters
float Kp = 1, Ki = 0, Kd = 0;
float previousError = 0.0;
float integral = 0.0;
float k_comp = 0.68;
unsigned long previousTime = 0;
float PID_output_max = 100;

// FIR Filter parameters
#define FILTER_ORDER 10  
#define Beta 0.24       
float accelBuffer[FILTER_ORDER] = {0};  


void setup() {
  Serial.begin(115200);
  
  if(!IMU.begin()){
    Serial.println("IMU Initialization Failed.");
    while(1);
  }

  pinMode(Motor_L_f, OUTPUT);
  pinMode(Motor_L_r, OUTPUT);
  pinMode(Motor_R_f, OUTPUT);
  pinMode(Motor_R_r, OUTPUT);
}

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
}

float FIR(float newSample) {
    float sum = 0, weightSum = 0;

    for (int i = FILTER_ORDER - 1; i > 0; i--) {
        accelBuffer[i] = accelBuffer[i - 1];
        float weight = exp(-Beta * i);
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
    float deltaT = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    float ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);

    accelTheta = atan(ay/az) * (180/PI);
    float sampleRate = IMU.gyroscopeSampleRate();
    float samplePeriod = 1 / sampleRate;
    float gyroTheta = accelTheta + gz * samplePeriod;
    float compTheta = k_comp * (gyroTheta) + (1-k_comp) * accelTheta + 2.0; // Sensor Fusion Using Complementary Filter
    float FIR_Theta = FIR(compTheta); // Apply FIR filter with exponentially decaying weights

    float setpoint = 0.0; 
    float error = setpoint - compTheta;
    integral += error * deltaT;
    float derivative = (error - previousError) / deltaT;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    if (abs(error) < 3) {
        integral = 0;
    }

    Serial.println(output);

    int motorSpeed = map(abs(output), 0, PID_output_max, 0, 255);
    motorSpeed = constrain(motorSpeed, 0, 255);

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

void updatePIDfromSerial() {
    if (Serial.available()) {
         //read the input until newline
        String input = Serial.readStringUntil('\n'); 
        //remove whitespace
        input.trim();  
        float newKp, newKi, newKd, newOUTPUT;

        int valuesParsed = sscanf(input.c_str(), "%f %f %f %f", &newKp, &newKi, &newKd, &newOUTPUT);

        if (valuesParsed == 4) { 
            Kp = newKp;
            Ki = newKi;
            Kd = newKd;
            PID_output_max = newOUTPUT;

            integral = 0;
            previousError = 0;
            previousTime = millis();

            Serial.print("PID Updated: Kp = ");
            Serial.print(Kp);
            Serial.print(", Ki = ");
            Serial.print(Ki);
            Serial.print(", Kd = ");
            Serial.println(Kd);
            Serial.print(", Output Max = ");
            Serial.print(newOUTPUT);
        } else {
            Serial.println("Invalid input. Enter values as: Kp Ki Kd Output");
        }
    }
}

void loop() {
    updatePIDfromSerial();
    pidLoop();
}

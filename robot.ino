#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <math.h>

#define PI 3.14159265

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
float Kp = 0.0, Ki = 0.0, Kd = 0.0;
float previousError = 0.0;
float integral = 0.0;
float k_comp = 0.68;
unsigned long previousTime = 0;

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

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
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
    float deltaT = (currentTime - previousTime) / 1000.0; // Convert to seconds
    previousTime = currentTime;

    // Read sensor data
    float ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);

    // Angle calculation and complementary filter
    accelTheta = atan(ay/az) * (180/PI);
    float sampleRate = IMU.gyroscopeSampleRate();
    float samplePeriod = 1 / sampleRate;
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
    // Adjust motor speeds
    if (control_mode) {
      motorSpeed = constrain(motorSpeed, 0, 255);
    }
    else {
      motorSpeed = 0;
    }

    // Serial.print(compTheta);
    // Serial.print("  ");
    // Serial.print(output);
    // Serial.print("  ");
    // Serial.print(motorSpeed);
    // Serial.println("  ");
    // Serial.println("Kp: " + Kp.toString() + "Ki: " + Ki.toString() + "Kd: " + Kd.toString());
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print("Ki: ");
    Serial.print(Ki);
    Serial.print("Kd: ");
    Serial.print(Kd);
    Serial.println("  ");


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

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {  
      pidLoop();
      readBluetoothBLE();
      delay(5);
    }

    Serial.println("Central device disconnected!");
  } 
}

#include "robot.h"

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    Wire.begin(); // i2c                                        
    Wire.setClock(I2C_CLOCK_SPEED);  
    checkMagnetPresence(&magnetStatus); 
    ReadRawAngle(&rawAngle, &degAngle);   
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

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float* gyroSampleRate) {
    unsigned long t0, t1;
    t0 = millis();
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    *gyroSampleRate = IMU.gyroscopeSampleRate();
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
    readIMU(ax, ay, az, gx, gy, gz, &gyro_sample_rate);

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
        // ReadRawAngle(&rawAngle, &degAngle); //ask the value from the sensor
        // correctAngle(&correctedAngle, &degAngle, &startAngle); //tare the value
        // checkQuadrant(&correctedAngle, &totalAngle, &num_turns, &quad_num, &prev_quad_num);
        // Serial.print("Raw Angle: ");
        // Serial.print(rawAngle);
        // Serial.print(" deg Angle: ");
        // Serial.println(degAngle);
        // delay(1000);

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
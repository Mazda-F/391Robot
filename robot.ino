#include "robot.h"

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("Serial Started ...");
    Wire.begin(); // i2c                                        
    Wire.setClock(I2C_CLOCK_SPEED);  

    Serial.println("Calibrating Encoders...");
    // checkMagnetPresence(&magnetStatus); 
    degAngle = ReadRawAngle(ENCODER_L);   
    start_angle = degAngle;  
    prev_angle = start_angle;                

    if (!IMU.begin()) { Serial.println("IMU Initialization Failed."); while(1); }
    if (!BLE.begin()) { Serial.println("Starting Bluetooth Low Energy Module Failed."); while (1);}

    pinMode(Motor_L_f, OUTPUT);
    pinMode(Motor_L_r, OUTPUT);
    pinMode(Motor_R_f, OUTPUT);
    pinMode(Motor_R_r, OUTPUT);  
    BLE.setLocalName("ROBOT_C4");
    BLE.setAdvertisedService(nanoService);
    nanoService.addCharacteristic(pitch_char);
    nanoService.addCharacteristic(speed_char);
    nanoService.addCharacteristic(yaw_char);
    nanoService.addCharacteristic(control_com);
    nanoService.addCharacteristic(K1_com);
    nanoService.addCharacteristic(K2_com);
    nanoService.addCharacteristic(K3_com);
    nanoService.addCharacteristic(K4_com);
    nanoService.addCharacteristic(K5_com);
    nanoService.addCharacteristic(K6_com);
    nanoService.addCharacteristic(K7_com);
    BLE.addService(nanoService);
    BLE.advertise();
    Serial.println("BLE advertising...");

    t0 = millis();
    t1 = t0;
}


/*
    Returns the angle in RADIANS
*/
float getIMUPitch() {
    float ax, ay, az, gx, gy, gz;
    float accelTheta, gyro_sample_rate, gyro_sample_period, gyroTheta;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    gyro_sample_rate = IMU.gyroscopeSampleRate();
    accelTheta = atan(ay/az) * (180/PI);
    gyro_sample_period = 1 / gyro_sample_rate;
    gyroTheta = accelTheta + gz * gyro_sample_period;
    return (K_COMP * (gyroTheta) + (1-K_COMP) * accelTheta + 2.0) * PI/180; // Sensor Fusion Using Complementary Filter 
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

float IIR(float newSample, float *previousValue, float beta) {
    float filterVal = beta * (*previousValue) + (1-beta) * (newSample);
    (*previousValue) = filterVal; 
    return filterVal;
}

void pid_IMU() {
    unsigned long currentTime = millis();
    float deltaT = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    theta = getIMUPitch();
    theta_error = 0.0 - theta;
    theta_integ += theta_error * deltaT;
    theta_dot = (theta_error - theta_prev_error) / deltaT;
    theta_dot = IIR(theta_dot, &theta_dot_prev,  0.7260); // Apply FIR (low pass) filter with exponentially decaying weights to derivative with high frequency noise
    
    float output_t = Kt.Kp * theta_error + Kt.Ki * theta_integ + Kt.Kd * theta_dot;

    wheel_angle = getAngle(start_angle, ENCODER_L);
    x = getDisplacement(rotations, x_prev, prev_angle, wheel_angle, WHEEL_RADIUS);
    x_error = 0 - x;
    x_integ += x_error * deltaT;
    x_dot = (x-x_prev)/deltaT;
    x_dot = IIR(x_error, &x_dot_prev, 0.3077);

  
    float output_x = Kx.Kp * theta_error + Kx.Ki * theta_integ + Kx.Kd * theta_dot;
    
    float output_pid = output_t * Kc + output_x * (1-Kc);


    if (abs(output_pid) > 3.3) {
        theta_integ *= 0.01;
        x_integ *= 0.01;
    }
    drive_motors(output_pid);
}



// void pid_ENCODER() {
//     unsigned long currentTime = millis();
//     float deltaT = (currentTime - previousTime) / 1000.0;
//     previousTime = currentTime;



//     if (abs(error) < 3) {
//         integral = 0;
//     }
// }

// void motorControl() {
//     int motorSpeed = map(abs(output), 0, PID_output_max, 0, 255);
//     motorSpeed = constrain(motorSpeed, 0, 255);

//     if (output > 0) {
//         analogWrite(Motor_L_r, motorSpeed);
//         analogWrite(Motor_R_r, motorSpeed);
//         analogWrite(Motor_L_f, 0);
//         analogWrite(Motor_R_f, 0);
//     } else {
//         analogWrite(Motor_L_f, motorSpeed);
//         analogWrite(Motor_R_f, motorSpeed);
//         analogWrite(Motor_L_r, 0);
//         analogWrite(Motor_R_r, 0);
//     }
// }

void drive_motors(float pid_out) {
    int motorSpeed = abs(pid_out/3.3 * 255);
    // if (isnan(motorSpeed)) {
    //   motorSpeed = 0;
    // } 
    motorSpeed = constrain(motorSpeed, 0, 255);
    if (control_mode) {
      if (pid_out > 0) {
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
    else {
      degAngle = ReadRawAngle(ENCODER_L);   
      start_angle = degAngle;  
      prev_angle = start_angle;
      rotations = 0;
      x = 0.0;
      x_prev = 0.0; 
      x_dot = 0.0;
      analogWrite(Motor_L_f, 0);
      analogWrite(Motor_R_f, 0);
      analogWrite(Motor_L_r, 0);
      analogWrite(Motor_R_r, 0);
    }
}

// float LQR_calc(float x, float xdot, float t, float tdot) {
//     float output;
//     float K0 = 3541;
//     float K1 = -80.8;
//     float K2 = 68.3;
//     float K3 = -3.5;

//     output = x*K0 + xdot*K1 + t*K2 + tdot*K3;

//     return output;
// }

void readBluetoothBLE() {
    String control_mode_str = control_com.value();
    String K1 = K1_com.value();
    String K2 = K2_com.value();
    String K3 = K3_com.value();
    String K4 = K4_com.value();
    String K5 = K5_com.value();
    String K6 = K6_com.value();
    String K7 = K7_com.value();
    control_mode = control_mode_str.toInt();
    float K1_in = K1.toFloat();
    float K2_in = K2.toFloat();
    float K3_in = K3.toFloat();
    float K4_in = K4.toFloat();
    float K5_in = K5.toFloat();
    float K6_in = K6.toFloat();
    float K7_in = K7.toFloat();
    // if (isnan(Kp_in)) Kp_in = 0;
    // if (isnan(Ki_in)) Ki_in = 0;
    // if (isnan(Kd_in)) Kd_in = 0;
    Kt.Kp = K1_in;
    Kt.Ki = K2_in;
    Kt.Kd = K3_in;

    Kx.Kp = K4_in;
    Kx.Ki = K5_in;
    Kx.Kd = K6_in;

    Kc = K7_in;

    // Kprint4(Kp, Ki, Kd, Ko);
}



// void lqr_iter() {
//     t1 = millis();
//     dt = (t1 - t0) / 1000.0;
//     t0 = t1;
//     theta = getIMUPitch();
//     theta_dot = (theta - theta_prev) / dt;
//     theta_dot = FIR(theta_dot);
//     wheel_angle = getAngle(start_angle, ENCODER_L);
//     // Serial.println(wheel_angle);
//     x = getDisplacement(rotations, x_prev, prev_angle, wheel_angle, WHEEL_RADIUS);
//     x_dot = (x - x_prev) / dt;

//     u = x * Kp + x_dot * Ki + theta * Kd + theta_dot * Ko;
//     Serial.println(u);
//     drive_motors(u);
// }


void loop() {
    BLEDevice central = BLE.central(); 

    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());

    while (central.connected()) {  
        // MAIN LOOP RUNTIME <= 25 ms
        // t0 = micros();
        // pidLoop();
        pid_IMU();
        readBluetoothBLE();
      
    }
    analogWrite(Motor_L_f, 0);
    analogWrite(Motor_R_f, 0);
    analogWrite(Motor_L_r, 0);
    analogWrite(Motor_R_r, 0);

    Serial.println("Central device disconnected!");
    } 
}





// void loop() {
//     // updatePIDfromSerial(&Kp, &Ki, &Kd, &integral, &previousError, &previousTime, &PID_output_max);
//     pidLoop();
    

    
// }
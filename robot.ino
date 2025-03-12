#include "robot.h"

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("Serial Started ...");
  Wire.begin();                                        
  Wire.setClock(I2C_CLOCK_SPEED);  

  Serial.println("Calibrating Encoders...");
  deg_angle = ReadRawAngle(ENCODER_L);   
  start_angle = deg_angle;  
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

  t1 = millis();
  t0 = t1;
}

float getIMUPitch() { // Returns the angle in RADIANS
  float ax, ay, az, gx, gy, gz;
  float accelTheta, gyro_sample_rate, gyro_sample_period, gyroTheta;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  gyro_sample_rate = IMU.gyroscopeSampleRate();
  accelTheta = atan(ay/az) * (180/PI);
  gyro_sample_period = 1 / gyro_sample_rate;
  gyroTheta = accelTheta + gz * gyro_sample_period;
  return (K_COMP * (gyroTheta) + (1-K_COMP) * accelTheta + 2.0) * PI/180 - 0.04; 
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

  theta.prop = getIMUPitch();
  err_theta.prop = 0.0 - theta.prop;
  err_theta.integ += err_theta.prop * deltaT;
  err_theta.deriv = (err_theta.prop - err_theta.prev_prop) / deltaT;
  err_theta.deriv = IIR(err_theta.deriv, &err_theta.prev_deriv, 0.7260);
  
  getWheelAngle(&total_angle, &num_turns, &quad_num, &prev_quad_num, start_angle);
  wheel_angle = total_angle * PI/180.0;
  x.prop = wheel_angle * WHEEL_RADIUS;
  prev_angle = wheel_angle;
  err_x.prop = 0.0 - x.prop;
  err_x.integ += err_x.prop * deltaT;
  err_x.deriv = (err_x.prop - err_x.prev_prop) / deltaT;
  err_x.deriv = IIR(err_x.deriv, &err_x.prev_deriv, 0.3077);
  
  theta.prev_prop = theta.prop;
  x.prev_prop = x.prop;
  
  float output_t = Kt.Kp * err_theta.prop + Kt.Ki * err_theta.integ + Kt.Kd * err_theta.deriv;
  float output_x = Kx.Kp * err_x.prop + Kx.Ki * err_x.integ + Kx.Kd * err_x.deriv;
  
  float output_pid = -1.0*(output_t * Kc + output_x * (1-Kc));

  if (abs(output_pid) > 3.3) {
    err_theta.integ *= 0.01;
    err_x.integ *= 0.01;
  }
  drive_motors(output_pid);
}

void pid2() {
  unsigned long currentTime = millis();
  float deltaT = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  theta.prop = getIMUPitch();
  err_theta.prop = 0.0 - theta.prop;
  err_theta.integ += err_theta.prop * deltaT;
  err_theta.deriv = (err_theta.prop - err_theta.prev_prop) / deltaT;
  err_theta.deriv = IIR(err_theta.deriv, &err_theta.prev_deriv, 0.7260);
  
  getWheelAngle(&total_angle, &num_turns, &quad_num, &prev_quad_num, start_angle);
  wheel_angle = total_angle * PI/180.0;
  x.prop = wheel_angle * WHEEL_RADIUS;
  x.deriv = (x.prop - x.prev_prop)/ deltaT;

  prev_angle = wheel_angle;
  err_x.prop = 0.0 - x.deriv;
  err_x.integ += err_x.prop * deltaT;
  err_x.deriv = (err_x.prop - err_x.prev_prop) / deltaT;
  err_x.deriv = IIR(err_x.deriv, &err_x.prev_deriv, 0.3077);
  
  theta.prev_prop = theta.prop;
  x.prev_prop = x.prop;
  
  float output_t = Kt.Kp * err_theta.prop + Kt.Ki * err_theta.integ + Kt.Kd * err_theta.deriv;
  float output_x = Kx.Kp * err_x.prop + Kx.Ki * err_x.integ + Kx.Kd * err_x.deriv;
  
  float output_pid = (output_t * Kc + output_x * (1-Kc));

  if (abs(output_pid) > 3.3) {
    err_theta.integ *= 0.01;
    err_x.integ *= 0.01;
  }
  drive_motors(output_pid);


  // sprintf(strbuf, "X: % 7.2f  ", x.prop);
  // sprintf(strbuf2, "Theta: % 7.2f  ", theta.prop);
  // strcat(strbuf, strbuf2);
  // sprintf(strbuf2, "xPID: % 7.2f  ", output_x);
  // strcat(strbuf, strbuf2);
  // sprintf(strbuf2, "tPID: % 7.2f  ", output_t);
  // strcat(strbuf, strbuf2);
  // sprintf(strbuf2, "PID: % 7.2f  ", output_pid);
  // strcat(strbuf, strbuf2);
  // sprintf(strbuf2, "wheel_angle: % 7.2f  ", wheel_angle);
  // strcat(strbuf, strbuf2);
  // sprintf(strbuf2, "num_turns: % 7.2f  ", num_turns);
  // strcat(strbuf, strbuf2);
  // Serial.println(strbuf);
}

void lqr() {
  t1 = millis();
  float dt = (t1 - t0) / 1000.0;
  if (dt < 0.0001 || isnan(dt)) dt = 0.01;
  t0 = t1;
  theta.prop = getIMUPitch();
  theta.deriv = (theta.prop - theta.prev_prop) / (float)dt;
  theta.deriv = IIR(theta.deriv, &theta.prev_deriv, 0.5);
  theta.prev_prop = theta.prop;
  
  getWheelAngle(&total_angle, &num_turns, &quad_num, &prev_quad_num, start_angle);
  wheel_angle = total_angle * PI/180.0;
  x.prop = wheel_angle * WHEEL_RADIUS;
  x.deriv = (x.prop - x.prev_prop) / (float)dt;
  x.deriv = IIR(x.deriv, &x.prev_deriv, 0.3);
  x.prev_prop = x.prop;

  u = x.prop * Kt.Kp + x.deriv * Kt.Ki + theta.prop * Kt.Kd + theta.deriv * Kx.Kp;

  drive_motors(u);
}

void drive_motors(float pid_out) {
  int motorSpeed = abs(pid_out/3.3 * 255);
  int pwm_start = 0;
  if (isnan(motorSpeed)) {
    motorSpeed = 0;
  } 
  // pwm.prop = motorSpeed;
  // pwm.deriv = (pwm.prop - pwm.prev_prop);
  // pwm.prev_prop = pwm.prop;

  // if (pwm.deriv < 0) pwm_start = PWM_H2L;
  // else pwm_start = PWM_L2H;

  // motorSpeed = map(motorSpeed, 0, 255, pwm_start, 255);
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
    deg_angle = ReadRawAngle(ENCODER_L);   
    start_angle = deg_angle;  
    prev_angle = start_angle;
    rotations = 0;
    num_turns = 0;

    x.prop = 0.0;
    x.prev_prop = 0.0; 
    x.deriv = 0.0;
    x.prev_deriv = 0.0;

    err_x.integ = 0.0;
    err_theta.integ = 0.0;
    analogWrite(Motor_L_f, 0);
    analogWrite(Motor_R_f, 0);
    analogWrite(Motor_L_r, 0);
    analogWrite(Motor_R_r, 0);
  }
}

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
  Kt.Kp = K1_in;
  Kt.Ki = K2_in;
  Kt.Kd = K3_in;
  Kx.Kp = K4_in;
  Kx.Ki = K5_in;
  Kx.Kd = K6_in;

  Kc = K7_in;
}

void loop() {
  BLEDevice central = BLE.central(); 

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {  
      // MAIN LOOP RUNTIME <= 25 ms
      // pid_IMU();
      // lqr();
      pid2();
      readBluetoothBLE();
    }

  Serial.println("Central device disconnected!");
  analogWrite(Motor_L_f, 0);
  analogWrite(Motor_R_f, 0);
  analogWrite(Motor_L_r, 0);
  analogWrite(Motor_R_r, 0);
  } 
}
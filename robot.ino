#include <ArduinoBLE.h>
#include <math.h>
#include <Wire.h> 
#include "bmi270.h"
 

#define Motor_R_f D2
#define Motor_R_r D3
#define Motor_L_f D4
#define Motor_L_r D5
#define SENSOR_PERIOD 0.020
#define SERIAL_BAUDRATE 9600
#define I2C_CLOCK_SPEED 400000L

#define M_PI acos(-1.0)

#define INC_ADDRESS 0x68 //I2C 7bit address
#define CMD 0x7E
#define PWR_CTRL 0x7D
#define PWR_CONF 0x7C
#define ACC 0x40
#define GYRO 0x42
#define CHIP 0x00
#define DATA 0x0C
#define DRDY 0x1D
#define STATUS 0x03
#define WIRE Wire1 // Be careful!
uint8_t gyr_cas_factor_zx;

#define KCTRL_MATRIX 0.004 

#define ACC_STDDEV 3 * M_PI/180
#define GYR_STDDEV 4 * M_PI/180

// IMU
#define K_COMP 0.95
#define IMU_BETA 0.9
#define INIT_KERROR 2 * M_PI/180
bool init_gyro_flag = true;

int16_t ax16, ay16, az16, gx16, gy16, gz16;
float ax, ay, az, gx, gy, gz, a_angle, g_angle;
float ax_prev, ay_prev, az_prev = 0.0;
float g_angle_z, g_angle_dz, a_angle_z, comp_angle_z;
double angle_dt;
unsigned long angle_prev_time, angle_curr_time;
float g_sample_period;
float g_angle_prev;
float comp_angle;

float tilt_angle = 0.0; 
float pitch_a;
float p_cal, q_cal, r_cal;
float roll_rps, yaw_rps, pitch_rps;
float kalman_pitch = 0;
float kalman_pitch_un = INIT_KERROR*INIT_KERROR;

float kcomp = K_COMP;

// Wheel Encoder
#define ENCODER_L 2
#define ENCODER_R 1
#define MUX_ADDRESS 0x70
#define WHEEL_RADIUS 0.041
#define ENCODER_ADDRESS 0x36
#define WHEEL_DISTANCE 0.245
float encoderTimer = 0;
struct wheeldata {
    int quad_num = 0; int prev_quad_num = 0;
    float start_angle = 0.0; float prev_angle = 0.0; float wheel_angle = 0.0; float num_turns = 0; 
    float total_angle = 0.0; float deg_angle = 0.0; float x = 0.0;
};

// PID 
float Kc = 0.0;
float pid_dt = 0;
unsigned long pid_prev_time = 0;
unsigned long pid_curr_time = 0;
struct K { float Kp = 0.0; float Ki = 0.0; float Kd = 0.0; };
struct timevar { float integ = 0.0; float prop = 0.0; float deriv = 0.0; float dd = 0.0;  
    float prev_prop = 0.0; float prev_deriv = 0.0; float prev_dd = 0.0; float temp1 = 0.0; float temp2 = 0.0; float temp3 = 0.0;
};

K Kt;
K Kx;
timevar theta;
timevar x;
timevar yaw;
timevar err_theta;
timevar err_x;
timevar pwm;
wheeldata lwheel;
wheeldata rwheel;

// FIR 
#define FILTER_ORDER 10  
#define BETA 0.24       
#define LAMBDA 0.5                      // Decay rate
float accelBuffer[FILTER_ORDER] = {0};  // Buffer to store past values

// Debugging 
char strbuf[200];
char strbuf2[100];

#define N_FIR 250
#define BETA_FIR 0.993
float fir_coeffs[N_FIR];
float ax_vec[N_FIR];
float az_vec[N_FIR];
float gx_vec[N_FIR];
float gz_vec[N_FIR];
float l_encoder_vec[N_FIR]; 
float r_encoder_vec[N_FIR]; 

// TimerDecorator imutimer("IMU_TIMER");
enum mode {
  IDLE, RUN
};

// Bluetooth
int control_mode = 0; // default manual
BLEService nanoService("13012F00-F8C3-4F4A-A8F4-15CD926DA146");
BLEStringCharacteristic pitch_char("13012F01-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic speed_char("13012F02-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16); 
BLEStringCharacteristic yaw_char("13012F03-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic control_com("13012F06-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K1_com("13012F07-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K2_com("13012F08-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K3_com("13012F09-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K4_com("13012F10-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K5_com("13012F11-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K6_com("13012F12-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);
BLEStringCharacteristic K7_com("13012F13-F8C3-4F4A-A8F4-15CD926DA146", BLERead | BLEWrite, 16);


uint8_t readRegister8(uint8_t reg) {
    WIRE.beginTransmission(INC_ADDRESS);
    WIRE.write(reg);
    WIRE.endTransmission(false);
    WIRE.requestFrom(INC_ADDRESS, 1);  
    return WIRE.read();
}

void writeRegister8(uint16_t reg, uint8_t value) {
    WIRE.beginTransmission(INC_ADDRESS);
    WIRE.write(reg);
    WIRE.write(value);
    WIRE.endTransmission();
}

void initFIR(float* coeffs, float beta, int n) {
    int i; 
    float sum = 0.0;
    for (i = 0; i < n; i++) {
        coeffs[i] = beta;
        sum += beta;
        beta *= beta;
    }
    for (i = 0; i < n; i++) {
        coeffs[i] /= sum;
    }
}

void initIMU(){
    WIRE.begin();
    WIRE.setClock(400000);
    // initialization
    writeRegister8(PWR_CONF, 0x00); 
    delay(50);
    writeRegister8(0x59, 0x00);
    // load config
    for (int i=0; i<256; i++)
    {
        writeRegister8(0x5B, 0x00);
        writeRegister8(0x5C, i);

        WIRE.beginTransmission(INC_ADDRESS);
        WIRE.write(0x5E);
        WIRE.write(&bmi270_config_file[i*32], 32);
        WIRE.endTransmission();    
        delay(1);
    }
    writeRegister8(0x59, 0x01);
    // configuration
    writeRegister8(PWR_CTRL, 0x06); //enable
    writeRegister8(ACC, 0xAC); //ACC_CONF
    writeRegister8(0x41, 0x02); //ACC_CONF
    writeRegister8(GYRO, 0xE9); //GYRO_CONF  // 0xE9
    writeRegister8(PWR_CONF, 0x02); //disable power saving
    gyr_cas_factor_zx = (readRegister8(0x3C) & 0b01111111);
    delay(1000);
}

void readIMU() {
    WIRE.beginTransmission(INC_ADDRESS);
    WIRE.write(DATA);
    WIRE.endTransmission();
    WIRE.requestFrom(INC_ADDRESS, 12);
  
    ax16 =             (WIRE.read()   | WIRE.read() << 8); 
    ay16 =             (WIRE.read()   | WIRE.read() << 8); 
    az16 =             (WIRE.read()   | WIRE.read() << 8); 
    gx16 =             (WIRE.read()   | WIRE.read() << 8); 
    gy16 =             (WIRE.read()   | WIRE.read() << 8); 
    gz16 =             (WIRE.read()   | WIRE.read() << 8); // data 18 | 19
    gx16 = gx16 - ((int16_t) gyr_cas_factor_zx) * gz16/512;
}

float readRawAngle(int bus) {
    float deg_angle, raw_angle, corrected_angle;
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(1<<bus);
    Wire.endTransmission();

    Wire.beginTransmission(0x36);                         //connect to the sensor
    Wire.write(0x0D);                                     //figure 21 - register map: Raw angle (7:0)
    Wire.endTransmission();                               //end transmission
    Wire.requestFrom(0x36, 1);                            //request from the sensor
    while (Wire.available() == 0);                        //wait until it becomes available
    int lowbyte = Wire.read();                                //Reading the data after the request

    // ----- read high-order bits 11:8
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);                                     //figure 21 - register map: Raw angle (11:8)
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    while (Wire.available() == 0);
    word highbyte = Wire.read();

    // ----- combine bytes
    highbyte = highbyte << 8;                             // shift highbyte to left
    raw_angle = highbyte | lowbyte;                        // combine bytes to get 12-bit value 11:0
    deg_angle = raw_angle * 0.087890625;                    // 360/4096 = 0.087890625

    //Serial.print("Deg angle: ");
    //Serial.println(degAngle, 2);                          //absolute position of the encoder within the 0-360 circle
    return deg_angle;
}

void getWheelAngle(float* total_angle, float* num_turns, int* quad_num, int* prev_quad_num, float start_angle, int bus) {
    float deg_angle, raw_angle, corrected_angle;
    
    Wire.beginTransmission(MUX_ADDRESS);
    Wire.write(1<<bus);
    Wire.endTransmission();

    Wire.beginTransmission(ENCODER_ADDRESS); 
    Wire.write(0x0D); 
    Wire.endTransmission(); 
    Wire.requestFrom(ENCODER_ADDRESS, 1); 
    while(Wire.available() == 0); // blocking
    int lowbyte = Wire.read(); 
    Wire.beginTransmission(ENCODER_ADDRESS);
    Wire.write(0x0C);
    Wire.endTransmission();
    Wire.requestFrom(ENCODER_ADDRESS, 1);
    
    while(Wire.available() == 0);  
    word highbyte = Wire.read();
    highbyte = highbyte << 8; 
    raw_angle = highbyte | lowbyte;  
    deg_angle = (raw_angle) * 0.087890625;  // 360/(2^12) = 360/4096 = 0.087890625
    corrected_angle = deg_angle - start_angle;
    if(corrected_angle < 0) { 
      corrected_angle = corrected_angle + 360.0; 
    }
  
    if(corrected_angle >= 0 && corrected_angle <=90) (*quad_num) = 1;
    if(corrected_angle > 90 && corrected_angle <=180) (*quad_num) = 2;
    if(corrected_angle > 180 && corrected_angle <=270) (*quad_num) = 3;
    if(corrected_angle > 270 && corrected_angle <360) (*quad_num) = 4;
    int qn = *quad_num;
    int prev_qn = *prev_quad_num;
    // Serial.print(prev_qn);
    // Serial.print("\t");
    // Serial.print(qn);
    // Serial.print("\t");
    if(qn != prev_qn) {  
        if(qn == 1 && prev_qn == 4){
              (*num_turns) += 1.0;  // 4 --> 1 transition: CW rotation
        }
        if(qn == 4 && prev_qn == 1){
              (*num_turns) -= 1.0; // 1 --> 4 transition: CCW rotation
        }
        *prev_quad_num = *quad_num; 
    }  
    *total_angle = ((*num_turns)*360) + corrected_angle;
  }

void calibrateIMU() {
    for (int i = 0; i < 2000; i++) {
        readIMU();
        p_cal += ((float) gx16) / 16.384;
        q_cal += ((float) gy16) / 16.384;
        r_cal += ((float) gz16) / 16.384;
        delay(0.2);
    }
    p_cal /= 2000;
    q_cal /= 2000;
    r_cal /= 2000;
}

float getAngle() { // returns the angle IN RADIANS
    readIMU();
    ax = ((float) ax16) / 4096.0;
    ay = ((float) ay16) / 4096.0;
    az = ((float) az16) / 4096.0;
    pitch_rps = ((float) gy16) / 16.384 - q_cal;
    pitch_rps *= M_PI/180;

    angle_curr_time = millis();
    angle_dt = (angle_curr_time - angle_prev_time) / 1000.0;
    angle_prev_time = angle_curr_time;

    pitch_a = atan(-ax/sqrt(ay*ay + az*az));

    kalman_pitch = kalman_pitch + (float)angle_dt*pitch_rps;
    kalman_pitch_un = kalman_pitch_un + (float)angle_dt*(float)angle_dt*ACC_STDDEV*ACC_STDDEV;
    float kgain = kalman_pitch_un *  1/(1*kalman_pitch_un+GYR_STDDEV*GYR_STDDEV);
    kalman_pitch = kalman_pitch + kgain* (pitch_a - kalman_pitch);
    kalman_pitch_un = (1-kgain) * kalman_pitch_un;

    return kalman_pitch;
}

float FIR(float new_data, float* databuf, float* coeffs, int num_coeffs) {
    float sum = 0;
    for (int i = num_coeffs - 1; i > 0; i--) {
        databuf[i] = databuf[i - 1];
        sum += databuf[i] * coeffs[i];
    }
    databuf[0] = new_data;
    sum += new_data * coeffs[0];          
    return sum;
}

float IIR(float newSample, float *previousValue, float beta) {
    float filterVal = beta * (*previousValue) + (1-beta) * (newSample);
    (*previousValue) = filterVal; 
    return filterVal;
}

void PID_step() {
    pid_curr_time = millis();
    pid_dt = (pid_curr_time - pid_prev_time) / 1000.0;
    pid_prev_time = pid_curr_time;

    // theta.prop = getAngle() * 180/M_PI;
    theta.prop = getAngle();
    theta.prev_prop = theta.prop;

    err_theta.prop = 0.0 - theta.prop;
    err_theta.integ += err_theta.prop * pid_dt;
    err_theta.deriv = (err_theta.prop - err_theta.prev_prop) / pid_dt;
    err_theta.deriv = IIR(err_theta.deriv, &err_theta.prev_deriv, 0.7260); 

    err_theta.prev_prop = err_theta.prop;
    

    getWheelAngle(&(lwheel.total_angle), &(lwheel.num_turns), &(lwheel.quad_num), 
        &(lwheel.prev_quad_num), lwheel.start_angle, ENCODER_L);
    getWheelAngle(&(rwheel.total_angle), &(rwheel.num_turns), &(rwheel.quad_num), 
        &(rwheel.prev_quad_num), rwheel.start_angle, ENCODER_R);

    lwheel.wheel_angle = lwheel.total_angle * PI/180.0;
    rwheel.wheel_angle = -rwheel.total_angle * PI/180.0;
    lwheel.wheel_angle = FIR(lwheel.wheel_angle, l_encoder_vec, fir_coeffs, N_FIR);
    rwheel.wheel_angle = FIR(rwheel.wheel_angle, r_encoder_vec, fir_coeffs, N_FIR);
    lwheel.x = lwheel.wheel_angle * WHEEL_RADIUS;
    rwheel.x = rwheel.wheel_angle * WHEEL_RADIUS;
    
    x.prop = (lwheel.x + rwheel.x) / 2.0;
    x.deriv = (x.prop - x.prev_prop) / pid_dt;
    x.deriv = IIR(x.deriv, &x.temp1, 0.98);
    x.dd = (x.deriv - x.prev_deriv) / pid_dt;
    x.dd = IIR(x.dd, &x.temp2, 0.98);
    x.prev_prop = x.prop;
    x.prev_deriv = x.deriv;
    x.prev_dd = x.dd;
    
    yaw.prop = ((lwheel.x -  rwheel.x) / WHEEL_DISTANCE);
    yaw.deriv = (yaw.prop - yaw.prev_prop) / pid_dt;
    yaw.deriv = IIR(yaw.deriv, &yaw.temp1, 0.98);
    yaw.dd = (yaw.deriv - yaw.prev_deriv) / pid_dt;
    yaw.dd = IIR(yaw.dd, &yaw.temp2, 0.98);
    yaw.prev_prop = yaw.prop;
    yaw.prev_deriv = yaw.deriv;
    yaw.prev_dd = yaw.dd;

    err_x.prop = 0.0 - x.prop;
    err_x.integ += err_x.prop * pid_dt;
    err_x.deriv = (err_x.prop - err_x.prev_prop) / pid_dt;
    err_x.deriv = IIR(err_x.deriv, &err_x.prev_deriv, 0.3077);
    
    err_x.prev_prop = err_x.prop;

    float output_t = Kt.Kp * err_theta.prop + Kt.Ki * err_theta.integ + Kt.Kd * err_theta.deriv;
    float output_x = Kx.Kp * err_x.prop + Kx.Ki * err_x.integ + Kx.Kd * err_x.deriv;
    float output_pid = output_t + output_x;

    // Serial.print(-20);
    // Serial.print(" ");
    // Serial.print(20);
    // Serial.print(" ");
    // Serial.print(theta.prop * 180.0/M_PI);
    // Serial.print(" ");
    Serial.print(yaw.prop);
    Serial.print(" ");
    Serial.print(x.prop);
    Serial.print(" ");
    // Serial.print(g_angle);
    // Serial.print(" ");
    // Serial.print(yaw.dd*180.0/PI);
    // Serial.print(" ");
    // Serial.print(yaw.dd*180.0/PI - theta.prop);
    Serial.println(" ");
    driveMotors(output_pid);  
}

void driveMotors(float pid_out) {
    int motorSpeed = (int) abs(pid_out);
    motorSpeed = constrain(motorSpeed, 0, 255);
    motorSpeed = map(motorSpeed, 0, 255, 7, 255);

    if (control_mode) {
        if (pid_out > 0) {
            analogWrite(Motor_L_r, 255-motorSpeed);
            analogWrite(Motor_R_r, 255-motorSpeed);
            analogWrite(Motor_L_f, 255);
            analogWrite(Motor_R_f, 255);
        } else {
            analogWrite(Motor_L_f, 255-motorSpeed);
            analogWrite(Motor_R_f, 255-motorSpeed);
            analogWrite(Motor_L_r, 255);
            analogWrite(Motor_R_r, 255);
        }
    }
    else {
        // deg_angle = ReadRawAngle(ENCODER_L);   
        // start_angle = deg_angle;  
        // prev_angle = start_angle;
        // rotations = 0;
        // x.prop = 0.0;
        // x.prev_prop = 0.0; 
        // x.deriv = 0.0;
        // x.prev_deriv = 0.0;
        analogWrite(Motor_L_f, 255);
        analogWrite(Motor_R_f, 255);
        analogWrite(Motor_L_r, 255);
        analogWrite(Motor_R_r, 255);

        err_theta.integ = 0;
        err_x.integ = 0;
            
        lwheel.total_angle = 0.0;
        lwheel.num_turns = 0; 
        lwheel.prev_quad_num = 0;
        lwheel.deg_angle = 0.0;
        lwheel.x = 0.0;
        lwheel.quad_num = 0;

        rwheel.total_angle = 0.0;
        rwheel.num_turns = 0; 
        rwheel.prev_quad_num = 0;
        rwheel.deg_angle = 0.0;
        rwheel.x = 0.0;
        rwheel.quad_num = 0;
    }

    
}

void bluetooth() {
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

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("Serial Started ...");
    Serial.println("Calibrating Encoders...");

    
    Wire.begin();                                         // start i2C
    Wire.setClock(I2C_CLOCK_SPEED);     

    lwheel.start_angle = readRawAngle(ENCODER_L);  
    rwheel.start_angle = readRawAngle(ENCODER_R);  
    lwheel.prev_angle = lwheel.start_angle;                
    rwheel.prev_angle = rwheel.start_angle;                
    initIMU();
    calibrateIMU();

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
    pid_curr_time = millis();
    pid_prev_time = pid_curr_time;

    angle_curr_time = millis();
    angle_prev_time = angle_curr_time; 


    initFIR(fir_coeffs, BETA_FIR, N_FIR);
    for (int i = 0; i < N_FIR; i++) ax_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) az_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) gx_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) gz_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) l_encoder_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) r_encoder_vec[i] = 0.0;
    
    


}

void loop() {
    BLEDevice central = BLE.central(); 
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        while (central.connected()) {  
            PID_step();
            bluetooth();
        }
        analogWrite(Motor_L_f, 255);
        analogWrite(Motor_R_f, 255);
        analogWrite(Motor_L_r, 255);
        analogWrite(Motor_R_r, 255);
        Serial.println("Central device disconnected!");
    } 
}
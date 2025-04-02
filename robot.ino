#include <ArduinoBLE.h>
#include <math.h>
#include <Wire.h> 
#include "bmi270.h"
#include "Servo.h"

Servo servoMotor;
int servoPin = 9;

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
#define ENCODER_L 1
#define ENCODER_R 2
#define MUX_ADDRESS 0x70
#define WHEEL_RADIUS 0.041
#define ENCODER_ADDRESS 0x36
#define WHEEL_DISTANCE 0.245
float encoderTimer = 0;
struct wheeldata {
    int quad_num = 1; int prev_quad_num = 1;
    float start_angle = 0.0; float prev_angle = 0.0; float wheel_angle = 0.0; float num_turns = 0.0; 
    float total_angle = 0.0; float deg_angle = 0.0; float x = 0.0;
};

// PID 
float Kc = 0.0;
float pid_dt = 0;
unsigned long pid_prev_time = 0;
unsigned long pid_curr_time = 0;
struct K { float Kp = 0.0; float Ki = 0.0; float Kd = 0.0; float beta = 0.99;};
struct timevar { float integ = 0.0; float prop = 0.0; float deriv = 0.0; float dd = 0.0;  
    float prev_prop = 0.0; float prev_deriv = 0.0; float prev_dd = 0.0; float temp1 = 0.0; float temp2 = 0.0; float temp3 = 0.0;
};
float x_desired = 0.0;
float speed_desired = 0.0;
float yaw_desired = 0.0;
float steering_rate_desired = 0.0;

K Kt;
K Kx;
K Ky;
timevar theta;
timevar x;
timevar yaw;
timevar err_theta;
timevar err_x;
timevar err_yaw;
timevar pwm;
wheeldata lwheel;
wheeldata rwheel;
#define PWM_DEADZONE 5

// FIR 
#define FILTER_ORDER 10  
#define BETA 0.24       
#define LAMBDA 0.5                      // Decay rate
float accelBuffer[FILTER_ORDER] = {0};  // Buffer to store past values

// Debugging 
char strbuf[200];
char strbuf2[100];

#define N_FIR 250
#define N_FIR_WHEEL 10
#define BETA_FIR 0.993
float fir_coeffs[N_FIR];
float ax_vec[N_FIR];
float az_vec[N_FIR];
float gx_vec[N_FIR];
float gz_vec[N_FIR];
float wheel_fir_coeffs[N_FIR_WHEEL];
float l_encoder_vec[N_FIR_WHEEL]; 
float r_encoder_vec[N_FIR_WHEEL]; 

// TimerDecorator imutimer("IMU_TIMER");

// Bluetooth
#define NUM_DIN 15
#define NUM_DOUT 13
#define COMMAND_HOLD_MILLIS 10
int control_state = 0;
int prev_control_state = 0;
uint8_t dinbuff[NUM_DIN * 4];
uint8_t doutbuff[NUM_DOUT * 4];
float bt_din_buff[NUM_DIN];
float bt_dout_buff[NUM_DOUT];
BLEService nanoService("449f9707-8365-440d-94c0-25c7663b292f");
BLECharacteristic dout_com("b9a7479e-6475-4093-ae2a-6ee19eae177a", BLERead | BLEWrite, 12 + NUM_DOUT*4);
BLECharacteristic din_com("0d4e68bf-be73-4ddc-847c-ea40afaef5ef", BLERead | BLEWrite, 12 + NUM_DIN*4);


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
    delay(500);
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

void calibrateWheelAngle(float* start_angle, float* num_turns, int* quad_num, int* prev_quad_num, int bus) {
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

    *prev_quad_num = 1;
    *quad_num = 1;
    *start_angle = deg_angle;
}

void getWheelAngle(float* total_angle, float* num_turns, int* quad_num, int* prev_quad_num, float start_angle, int bus, bool reverse) {
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
    if (reverse) {
        corrected_angle = start_angle - deg_angle;
    } else {
        corrected_angle = deg_angle - start_angle;
    }

    if(corrected_angle < 0) { 
      corrected_angle = corrected_angle + 360.0; 
    }
  
    if(corrected_angle >= 0 && corrected_angle <=90) (*quad_num) = 1;
    if(corrected_angle > 90 && corrected_angle <=180) (*quad_num) = 2;
    if(corrected_angle > 180 && corrected_angle <=270) (*quad_num) = 3;
    if(corrected_angle > 270 && corrected_angle <360) (*quad_num) = 4;
    int qn = *quad_num;
    int prev_qn = *prev_quad_num;
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

    angle_curr_time = micros();
    angle_dt = (angle_curr_time - angle_prev_time) / 1000000.0;
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
    if (isnan(new_data)) {
        databuf[0] = databuf[1];
    } else {
        databuf[0] = new_data;
    }
    sum += new_data * coeffs[0];
    if (isnan(sum)) sum = 0.0;        
    return sum;
}

float IIR(float newSample, float *previousValue, float beta) {
    float filterVal = beta * (*previousValue) + (1-beta) * (newSample);
    (*previousValue) = filterVal; 
    if (isnan(filterVal)) {
        filterVal = 0.0;
        if (isnan(*previousValue)) {
            *previousValue = 0.0;
        }
        else {
            filterVal = *previousValue;
        }
    }
    
    return filterVal;
}

void PID_step() {
    pid_curr_time = micros();
    pid_dt = (pid_curr_time - pid_prev_time) / 1000000.0;
    if (pid_dt < 0.000001) {
        pid_dt = 0.001;
    }
    pid_prev_time = pid_curr_time;

    if (control_state) {
        x_desired += speed_desired * pid_dt;  
        yaw_desired += steering_rate_desired * pid_dt;
    } else {
        x_desired = 0.0;
        yaw_desired = 0.0;
    }

    /*** X ***/
    if (control_state == 1 && prev_control_state == 0) {
        calibrateWheelAngle(&(lwheel.start_angle), &(lwheel.num_turns), &(lwheel.quad_num), 
            &(lwheel.prev_quad_num), ENCODER_L);
        calibrateWheelAngle(&(rwheel.start_angle), &(rwheel.num_turns), &(rwheel.quad_num), 
            &(rwheel.prev_quad_num), ENCODER_R); 
        err_theta.integ = 0.0;
        err_x.integ = 0.0;
        err_yaw.integ = 0.0;
    }

    if (control_state) {
        getWheelAngle(&(lwheel.total_angle), &(lwheel.num_turns), &(lwheel.quad_num), 
            &(lwheel.prev_quad_num), lwheel.start_angle, ENCODER_L, false);
        getWheelAngle(&(rwheel.total_angle), &(rwheel.num_turns), &(rwheel.quad_num), 
            &(rwheel.prev_quad_num), rwheel.start_angle, ENCODER_R, true);
        lwheel.wheel_angle = lwheel.total_angle * M_PI/180.0;
        rwheel.wheel_angle = rwheel.total_angle * M_PI/180.0;
        lwheel.wheel_angle = FIR(lwheel.wheel_angle, l_encoder_vec, wheel_fir_coeffs, N_FIR_WHEEL);
        rwheel.wheel_angle = FIR(rwheel.wheel_angle, r_encoder_vec, wheel_fir_coeffs, N_FIR_WHEEL);
        lwheel.x = lwheel.wheel_angle * WHEEL_RADIUS;
        rwheel.x = rwheel.wheel_angle * WHEEL_RADIUS;
    
    } else {
        lwheel.total_angle      = 0.0;
        lwheel.wheel_angle      = 0.0;
        lwheel.num_turns        = 0; 
        lwheel.quad_num         = 1;
        lwheel.prev_quad_num    = 1;
        lwheel.deg_angle        = 0.0;
        lwheel.x                = 0.0;
        rwheel.total_angle      = 0.0;
        rwheel.wheel_angle      = 0.0;
        rwheel.num_turns        = 0; 
        rwheel.quad_num         = 1;
        rwheel.prev_quad_num    = 1;
        rwheel.deg_angle        = 0.0;
        rwheel.x                = 0.0;
    }
    
    x.prop = (lwheel.x + rwheel.x) / 2.0;
    x.deriv = (x.prop - x.prev_prop) / pid_dt;
    x.deriv = IIR(x.deriv, &x.prev_deriv, 0.98);
    x.prev_prop = x.prop;

    err_x.prop = x_desired - x.prop;
    err_x.integ += err_x.prop * pid_dt;
    err_x.deriv = (err_x.prop - err_x.prev_prop) / pid_dt;
    err_x.deriv = IIR(err_x.deriv, &err_x.prev_deriv, Kx.beta);
    err_x.prev_prop = err_x.prop;
    
    float x_pid_out = Kx.Kp * err_x.prop + Kx.Ki * err_x.integ + Kx.Kd * err_x.deriv;


    /*** THETA ***/
    // theta.prop = getAngle() * 180/M_PI;
    theta.prop = getAngle();
    if (isnan(theta.prop)) {
        theta.prop = theta.prev_prop;
    } else {
        theta.prev_prop = theta.prop;
    }

    err_theta.prop = x_pid_out - theta.prop;
    err_theta.integ += err_theta.prop * pid_dt;
    err_theta.deriv = (err_theta.prop - err_theta.prev_prop) / pid_dt;
    err_theta.deriv = IIR(err_theta.deriv, &err_theta.prev_deriv, Kt.beta); 
    err_theta.prev_prop = err_theta.prop;
    
    float theta_pid_out = Kt.Kp * err_theta.prop + Kt.Ki * err_theta.integ + Kt.Kd * err_theta.deriv;


    /*** YAW ***/
    yaw.prop = ((lwheel.x -  rwheel.x) / WHEEL_DISTANCE);
    yaw.deriv = (yaw.prop - yaw.prev_prop) / pid_dt;
    yaw.deriv = IIR(yaw.deriv, &yaw.prev_deriv, 0.98);
    yaw.prev_prop = yaw.prop;

    err_yaw.prop = yaw_desired - yaw.prop;
    err_yaw.integ += err_yaw.prop * pid_dt;
    err_yaw.deriv = (err_yaw.prop - err_yaw.prev_prop) / pid_dt;
    err_yaw.deriv = IIR(err_yaw.deriv, &err_yaw.prev_deriv, Ky.beta);
    err_yaw.prev_prop = err_yaw.prop;

    float yaw_pid_out = Ky.Kp * err_yaw.prop + Ky.Ki * err_yaw.integ + Ky.Kd * err_yaw.deriv;

    float left_motor_pwm = theta_pid_out - yaw_pid_out;
    float right_motor_pwm = theta_pid_out + yaw_pid_out;

    bt_dout_buff[0] = (float)theta.prop;
    bt_dout_buff[1] = (float)x.prop;
    bt_dout_buff[2] = (float)yaw.prop;
    bt_dout_buff[3] = (float)theta_pid_out;
    bt_dout_buff[4] = (float)x_pid_out;
    bt_dout_buff[5] = (float)yaw_pid_out;
    bt_dout_buff[6] = (float)x_desired;
    bt_dout_buff[7] = (float)yaw_desired;
    bt_dout_buff[8] = (float)lwheel.wheel_angle;
    bt_dout_buff[9] = (float)rwheel.wheel_angle;
    bt_dout_buff[10] = (float)left_motor_pwm;
    bt_dout_buff[11] = (float)right_motor_pwm;
    bt_dout_buff[12] = (float)pid_dt;
    // Serial.print(left_motor_pwm);
    // Serial.print("\t");
    
    // Serial.print(right_motor_pwm);
    // Serial.print("\t");
    // Serial.println(" ");
    if (isnan(left_motor_pwm)) {
        left_motor_pwm = 0.0;
    }
    if (isnan(right_motor_pwm)) {
        right_motor_pwm = 0.0;
    }
    driveMotors(left_motor_pwm, right_motor_pwm);  
    
}


void driveMotors(float left_motor_pwm, float right_motor_pwm) {
    left_motor_pwm = constrain(left_motor_pwm, -255, 255);
    right_motor_pwm = constrain(right_motor_pwm, -255, 255);
    int leftSpeed = map(abs(left_motor_pwm), 0, 255, PWM_DEADZONE, 255);
    int rightSpeed = map(abs(right_motor_pwm), 0, 255, PWM_DEADZONE, 255);

    if (control_state) {
        //// Left motor
        if (left_motor_pwm > 0) {  // Forward
            analogWrite(Motor_L_f, 255);
            analogWrite(Motor_L_r, 255-leftSpeed);
        } else {                   // Backward
            analogWrite(Motor_L_f, 255-leftSpeed);
            analogWrite(Motor_L_r, 255);
        }
        // Right motor
        if (right_motor_pwm > 0) {  // Forward
            analogWrite(Motor_R_f, 255);
            analogWrite(Motor_R_r, 255-rightSpeed);
        } else {                    // Backward
            analogWrite(Motor_R_f, 255-rightSpeed);
            analogWrite(Motor_R_r, 255);
        }
        
        digitalWrite(LEDR, HIGH);         
        digitalWrite(LEDG, LOW);        
        digitalWrite(LEDB, LOW);
        
    } else {
        analogWrite(Motor_L_f, 255);
        analogWrite(Motor_R_f, 255);
        analogWrite(Motor_L_r, 255);
        analogWrite(Motor_R_r, 255);

        digitalWrite(LEDR, LOW);         
        digitalWrite(LEDG, LOW);        
        digitalWrite(LEDB, HIGH);
    }
}

float n_to_beta(float n) {
    float beta = (n+0.2)/(n+1.12); // from the teachings of prof. leo stocco
    if (beta >= 1.0) {
        beta = 0.99;
    }
    return beta;
}

void bluetooth() {
    // read
    int intval;
    if (din_com.readValue(dinbuff, NUM_DIN*4) == NUM_DIN*4) {
        // the first 4 bytes belong to the control_command, which is an int
        prev_control_state = control_state;
        memcpy(&intval, dinbuff, sizeof(int));
        control_state = intval;
        if (isnan(control_state)) {
            control_state = prev_control_state;
        }
        
        // the remaining are floats
        for (int i = 1; i < NUM_DIN; i++) {
            memcpy(&bt_din_buff[i], dinbuff + i * sizeof(float), sizeof(float));
            if (isnan(bt_din_buff[i])) {
                bt_din_buff[i] = 0.0;
                // Serial.print("NAN");
            } 
        }
        speed_desired = bt_din_buff[1];
        steering_rate_desired = bt_din_buff[2];
        Kt.Kp = bt_din_buff[3];
        Kt.Ki = bt_din_buff[4];
        Kt.Kd = bt_din_buff[5];
        Kt.beta = n_to_beta(bt_din_buff[6]); // iir beta on derivative path
        Kx.Kp = bt_din_buff[7];
        Kx.Ki = bt_din_buff[8];
        Kx.Kd = bt_din_buff[9];
        Kx.beta = n_to_beta(bt_din_buff[10]);
        Ky.Kp = bt_din_buff[11];
        Ky.Ki = bt_din_buff[12];
        Ky.Kd = bt_din_buff[13];
        Ky.beta = n_to_beta(bt_din_buff[14]);
    } else {
        Serial.print(millis());
        Serial.print(" ");
        Serial.println("[ERROR]: Not enough bytes recieved for DATA IN");
        control_state = prev_control_state;
    }


    // write
    for (int i = 0; i < NUM_DOUT; i++) {
        if (isnan(bt_dout_buff[i])) {
            bt_dout_buff[i] = 0.0;
        }
        memcpy(doutbuff + i * sizeof(float), &bt_dout_buff[i], sizeof(float));
    }
    dout_com.writeValue(doutbuff, NUM_DOUT*sizeof(float), true);
}


void calibrateAll() {
    digitalWrite(LEDR, HIGH);         
    digitalWrite(LEDG, HIGH);        
    digitalWrite(LEDB, HIGH);

    calibrateWheelAngle(&(lwheel.start_angle), &(lwheel.num_turns), &(lwheel.quad_num), 
        &(lwheel.prev_quad_num), ENCODER_L);
    calibrateWheelAngle(&(rwheel.start_angle), &(rwheel.num_turns), &(rwheel.quad_num), 
        &(rwheel.prev_quad_num), ENCODER_R); 

    delay(250); digitalWrite(LEDR, LOW); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);

    initIMU();
    delay(250); digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);

    calibrateIMU();
    digitalWrite(LEDR, LOW); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);

    pid_curr_time = micros();
    pid_prev_time = pid_curr_time;
    angle_curr_time = micros();
    angle_prev_time = angle_curr_time; 
    initFIR(fir_coeffs, BETA_FIR, N_FIR);
    initFIR(wheel_fir_coeffs, n_to_beta(N_FIR_WHEEL), N_FIR_WHEEL);
    for (int i = 0; i < N_FIR; i++) ax_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) az_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) gx_vec[i] = 0.0;
    for (int i = 0; i < N_FIR; i++) gz_vec[i] = 0.0;
    for (int i = 0; i < N_FIR_WHEEL; i++) l_encoder_vec[i] = 0.0;
    for (int i = 0; i < N_FIR_WHEEL; i++) r_encoder_vec[i] = 0.0;

    for (int i = 0; i < NUM_DIN; i++) bt_din_buff[i] = 0.0;
}   

void lightshow() {
    digitalWrite(LEDR, LOW); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
    delay(100);
    digitalWrite(LEDR, HIGH); digitalWrite(LEDG, LOW); digitalWrite(LEDB, HIGH);
    delay(100);
    digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, LOW);
    delay(100);
    digitalWrite(LEDR, LOW); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
    delay(100);
    digitalWrite(LEDR, HIGH); digitalWrite(LEDG, LOW); digitalWrite(LEDB, HIGH);
    delay(100);
    digitalWrite(LEDR, HIGH); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, LOW);
    delay(100);
    digitalWrite(LEDR, LOW); digitalWrite(LEDG, HIGH); digitalWrite(LEDB, HIGH);
    delay(100);
}

void setup() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);       
    digitalWrite(LEDR, LOW);         
    digitalWrite(LEDG, HIGH);        
    digitalWrite(LEDB, HIGH);

    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("Serial Started ...");
    Serial.println("Calibrating Encoders...");

    Wire.begin();                                         // start i2C
    Wire.setClock(I2C_CLOCK_SPEED);    
    
    lightshow();

    // calibrateAll();

    if (!BLE.begin()) { 
        Serial.println("Starting Bluetooth Low Energy Module Failed.");
         while (1);
    }
    pinMode(Motor_L_f, OUTPUT);
    pinMode(Motor_L_r, OUTPUT);
    pinMode(Motor_R_f, OUTPUT);
    pinMode(Motor_R_r, OUTPUT); 

    servoMotor.attach(servoPin); 

    BLE.setLocalName("ROBOT_C4");
    BLE.setAdvertisedService(nanoService);
    nanoService.addCharacteristic(dout_com);
    nanoService.addCharacteristic(din_com);
    BLE.addService(nanoService);
    BLE.advertise();
    Serial.println("BLE advertising...");
}

void loop() {
    BLEDevice central = BLE.central(); 
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        calibrateAll();
        while (central.connected()) {  
            PID_step();
            bluetooth();
            
            servoMotor.write(55 - theta.prop * 180.0/M_PI);
        }
        analogWrite(Motor_L_f, 255);
        analogWrite(Motor_R_f, 255);
        analogWrite(Motor_L_r, 255);
        analogWrite(Motor_R_r, 255);
        digitalWrite(LEDR, LOW);         
        digitalWrite(LEDG, HIGH);        
        digitalWrite(LEDB, HIGH);
        Serial.println("Central device disconnected!");
    } 
}
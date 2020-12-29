#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

#include "KalmanFilter.h"

// Notes:
// - Requires Adafruit Unified Sensor with modified BMP sensor address

// To Do before Launch:
// - Reference value tuning (can change due to IMU mounting)
// - Zero values of servos tuning (can change due to servo and TVC mounting)
// - Uncomment parachute ejection section
// - Set true LOG macro

#define DEBUG false
#define LOG true
#define IMU_CALIBRATION true

#define P0 1021
#define DT_MILLIS 45

#define ACC_TH_START 20.0
#define ACC_TH_STOP 10.0
#define PARACHUTE_DELAY 3.0

#define LED_R_PIN 3
#define LED_G_PIN 7
#define LED_B_PIN 8

#define SOUND_PIN 2
#define SD_PIN 4

#define SERVO_1_PIN 9
#define SERVO_2_PIN 10
#define SERVO_3_PIN 5
#define SERVO_4_PIN 6

#define INNER_ZERO 60
#define OUTER_ZERO 75
#define SERVO_SATURATION 28
#define PAR_SERVO_CLOSED 0
#define PAR_SERVO_OPEN 30

#define MPU 0x68 // MPU6050 I2C address (alternative: 0x77)
#define BETA 0.8 // BETA = sqrt(3.0 / 4.0) * (PI * (gyroMeasError / 180.0)). Good tuning around 0.8, 1, 2.
#define LPF_GRAVITY_COMP_ALFA 0.98

#define LPF_CONTROL_ALFA 0.5
#define PHI_REF PI/2
#define THETA_REF 0.0
#define KP_PHI 88.0
#define KD_PHI 9.0
#define KI_PHI 2.0
#define KP_THETA 88.0
#define KD_THETA 9.0
#define KI_THETA 2.0

// Process
float clockStart = 0, clockEnd = 0, timeElapsed = 0;
float dt = DT_MILLIS * 0.001;
bool controlStarted = false, parachuteProcedureStarted = false;

// Servos
Servo inner_servo;
Servo outer_servo;
Servo parachute_servo;

// IMU sensor
float q_0 = 1, q_1 = 0, q_2 = 0, q_3 = 0;
float ax, ay, az, omega_x, omega_y, omega_z;
float ax_error = 0, ay_error = 0, az_error = 0, a_error_norm = 0, omega_x_error = 0, omega_y_error = 0, omega_z_error = 0;
float phi_raw, theta_raw, psi_raw, phi, theta;

KalmanFilter KF_phi = KalmanFilter(0.01, 0.01, 0.03); // Good tuning around (0.015, 0.015, 0.03)
KalmanFilter KF_theta = KalmanFilter(0.01, 0.01, 0.03);

// Gravity compensation
float g_comp_1_old = 0, g_comp_2_old = 9.81, g_comp_3_old = 0;
float g_comp_1, g_comp_2, g_comp_3;

// Pressure sensor
Adafruit_BMP280 bmp;
float temperature, pressure, altitude;

// Log
File log_file;

// Control: PID
float e_phi, e_phi_dot, e_phi_int, e_phi_old;
float e_theta, e_theta_dot, e_theta_int, e_theta_old;
float u1, u2, u1_old = 0, u2_old = 0;

void setup() {

    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    pinMode(SOUND_PIN, OUTPUT);

    inner_servo.attach(SERVO_1_PIN);
    outer_servo.attach(SERVO_2_PIN);
    parachute_servo.attach(SERVO_4_PIN);

    inner_servo.write(INNER_ZERO);
    outer_servo.write(OUTER_ZERO);
    parachute_servo.write(PAR_SERVO_CLOSED);

    Serial.begin(19200);

    Wire.begin();

    if(!SD.begin(SD_PIN)) {
        #if DEBUG
            Serial.println(F("Initialization failed: MicroSD Card Adapter"));
        #endif
        soundSignal(100, 30, 2);
        while(1);
    }

    log_file = SD.open("LOG.TXT", FILE_WRITE);

    if (!log_file) {
        #if DEBUG
            Serial.println(F("Initialization failed: Log file opening failed"));
        #endif
        soundSignal(100, 30, 3);
        while(1);
    }

    #if LOG
        log_file.println(F("-----"));
        log_file.println(F("[*] Starting launch procedure"));
        log_file.flush();
    #endif

    if (!bmp.begin()) {
        #if DEBUG
            Serial.println(F("Initialization failed: BMP280 sensor"));
        #endif

        #if LOG
            log_file.println(F("[!] Initializing BMP280 sensor: failed"));
            log_file.flush();
        #endif

        soundSignal(100, 30, 4);
        while (1);
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   // Operating Mode
        Adafruit_BMP280::SAMPLING_X2,               // Temp. oversampling
        Adafruit_BMP280::SAMPLING_X16,              // Pressure oversampling
        Adafruit_BMP280::FILTER_X16,                // Filtering
        Adafruit_BMP280::STANDBY_MS_500);           // Standby time

    #if LOG
        log_file.println(F("[*] Initializing BMP280 sensor: done"));
        log_file.flush();
    #endif

    // Initialize comunication
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Configure Accelerometer Sensitivity
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);               // Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);               // Set the register bits as 00010000 (+/- 8g full scale)
    Wire.endTransmission(true);

    // Configure Gyro Sensitivity
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);               // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);               // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);

    #if LOG
        log_file.println(F("[*] Initializing MPU6050 sensor: done"));
        log_file.flush();
    #endif

    #if IMU_CALIBRATION
        compute_IMU_error();
        delay(20);
    #endif

    #if LOG
        log_file.println(F("[*] Calibrating IMU: done"));
        log_file.println(F("\n[*] Starting measuring..."));
        log_file.println(F("ax, ay, az, q0, q1, q2, q3, phi_raw, theta_raw, psi_raw, phi, theta, temperature, pressure, altitude, u1, u2"));
        log_file.flush();
    #endif

    soundSignal(200, 200, 1);
}

void loop() {

    clockStart = millis();

    // Getting acceleration
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    ax = (Wire.read() << 8 | Wire.read()) / 4096.0 * 9.81;
    ay = (Wire.read() << 8 | Wire.read()) / 4096.0 * 9.81;
    az = (Wire.read() << 8 | Wire.read()) / 4096.0 * 9.81;

    // Getting attitude
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    omega_x = (((Wire.read() << 8 | Wire.read()) - omega_x_error) / 32.8) * PI / 180.0;
    omega_y = (((Wire.read() << 8 | Wire.read()) - omega_y_error) / 32.8) * PI / 180.0;
    omega_z = (((Wire.read() << 8 | Wire.read()) - omega_z_error) / 32.8) * PI / 180.0;

    updateFilter(omega_x, omega_y, omega_z, ax, ay, az);
    computeEuler();

    // Getting temperature, pressure, altitude
    temperature = bmp.readTemperature();    // celsius
    pressure = bmp.readPressure()/100;      // hPa
    altitude = bmp.readAltitude(P0);        // meters

    // Filtering
    phi = KF_phi.updateEstimate(phi_raw);
    theta = KF_theta.updateEstimate(theta_raw);

    // Gravity compensation
    acc_gravity_compensate(ax, ay, az);

    #if DEBUG
        Serial.print(ax); Serial.print(F("\t"));
        Serial.print(ay); Serial.print(F("\t"));
        Serial.print(az); Serial.print(F("\t"));
        Serial.print(phi_raw); Serial.print(F("\t"));
        Serial.print(theta_raw); Serial.print(F("\t"));
        Serial.print(psi_raw); Serial.print(F("\t"));
        Serial.print(phi); Serial.print(F("\t"));
        Serial.print(theta); Serial.print(F("\t"));
        Serial.print(temperature); Serial.print(F("\t"));
        Serial.print(pressure); Serial.print(F("\t"));
        Serial.print(altitude); Serial.print(F("\t"));
    #endif

    #if LOG
        log_file.print(ax); log_file.print(F(", "));
        log_file.print(ay); log_file.print(F(", "));
        log_file.print(az); log_file.print(F(", "));
        log_file.print(phi_raw); log_file.print(F(", "));
        log_file.print(theta_raw); log_file.print(F(", "));
        log_file.print(psi_raw); log_file.print(F(", "));
        log_file.print(phi); log_file.print(F(", "));
        log_file.print(theta); log_file.print(F(", "));
        log_file.print(temperature); log_file.print(F(", "));
        log_file.print(pressure); log_file.print(F(", "));
        log_file.print(altitude); log_file.print(F(", "));
        log_file.flush();
    #endif

    if ((ay > ACC_TH_START) && !controlStarted) {
        controlStarted = true;
        #if DEBUG
            Serial.println(F("Starting controller... "));
        #endif
        #if LOG
            log_file.println(F("[*] Starting controller... "));
            log_file.flush();
        #endif
    }

    if (controlStarted) {

        e_phi = PHI_REF - phi;
        e_phi_dot = (e_phi - e_phi_old) / dt;
        e_phi_int = e_phi_int + e_phi * dt;
        e_phi_old = e_phi;

        e_theta = THETA_REF - theta;
        e_theta_dot = (e_theta - e_theta_old) / dt;
        e_theta_int = e_theta_int + e_theta * dt;
        e_theta_old = e_theta;

        u1 = LPF_CONTROL_ALFA * u1_old + (1 - LPF_CONTROL_ALFA) * (KP_PHI * e_phi + KD_PHI * e_phi_dot + KI_PHI * e_phi_int);
        u2 = LPF_CONTROL_ALFA * u2_old + (1 - LPF_CONTROL_ALFA) * (KP_THETA * e_theta + KD_THETA * e_theta_dot + KI_THETA * e_theta_int);
        saturate(u1, SERVO_SATURATION);
        saturate(u2, SERVO_SATURATION);
        u1_old = u1;
        u2_old = u2;

        // Use for proper tuning
        //Serial.print(phi); Serial.print(F("\t")); Serial.print(theta); Serial.println("\t");
        //Serial.print(u1); Serial.print(F("\t")); Serial.print(u2); Serial.println("\t");

        inner_servo.write(INNER_ZERO - u1);
        outer_servo.write(OUTER_ZERO - u2);

        #if DEBUG
            Serial.print(u1); Serial.print(F("\t"));
            Serial.println(u2);
        #endif
        #if LOG
            log_file.print(u1); log_file.print(F(", "));
            log_file.println(u2);
            log_file.flush();
        #endif


        if ((ay < - ACC_TH_STOP) || (phi >= (PHI_REF + 1.0/4.0*PI)) || (phi <= (PHI_REF - 1.0/4.0*PI)) || (theta >= (THETA_REF + 1.0/4.0*PI)) || (theta <= (THETA_REF - 1.0/4.0*PI))) {
            #if DEBUG
                Serial.println(F("Starting parachute procedure... "));
            #endif
            #if LOG
                log_file.println(F("[*] Starting parachute procedure... "));
                log_file.flush();
            #endif
            controlStarted = false;
            parachuteProcedureStarted = true;
        }
    }

    if (parachuteProcedureStarted) {
        delay(1000 * PARACHUTE_DELAY);
        parachute_servo.write(PAR_SERVO_OPEN);
        parachuteProcedureStarted = false;
        #if DEBUG
            Serial.println(F("Parachute ejected "));
        #endif
        #if LOG
            log_file.println(F("[*] Parachute ejected "));
            log_file.flush();
        #endif
    }

    clockEnd = millis();
    timeElapsed = clockEnd - clockStart;
    if (DT_MILLIS > timeElapsed) delay(DT_MILLIS - timeElapsed);
}

void compute_IMU_error() {
    /*
    for (int i = 0; i < 500; i++) {
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);

        ax = (Wire.read() << 8 | Wire.read()) / 4096.0 * 9.81;
        ay = (Wire.read() << 8 | Wire.read()) / 4096.0 * 9.81;
        az = (Wire.read() << 8 | Wire.read()) / 4096.0 * 9.81;

        ax_error = ax_error + ax;
        ay_error = ay_error + ay;
        az_error = az_error + az;
    }

    ax_error = ax_error / 500.0;
    ay_error = ay_error / 500.0;
    az_error = az_error / 500.0;
    a_error_norm = sqrt(ax_error * ax_error + ay_error * ay_error + az_error * az_error);
    */

    for (int i = 0; i < 500; i++) {
        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);

        omega_x = Wire.read() << 8 | Wire.read();
        omega_y = Wire.read() << 8 | Wire.read();
        omega_z = Wire.read() << 8 | Wire.read();

        omega_x_error = omega_x_error + omega_x;
        omega_y_error = omega_y_error + omega_y;
        omega_z_error = omega_z_error + omega_z;
    }

    omega_x_error = omega_x_error / 500.0;
    omega_y_error = omega_y_error / 500.0;
    omega_z_error = omega_z_error / 500.0;
}

void updateFilter(float omega_x, float omega_y, float omega_z, float a_x, float a_y, float a_z) {

    float norm;

    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;
    float f_1, f_2, f_3;
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
    float nablaf_1, nablaf_2, nablaf_3, nablaf_4;

    float halfSEq_1 = 0.5 * q_0;
    float halfSEq_2 = 0.5 * q_1;
    float halfSEq_3 = 0.5 * q_2;
    float halfSEq_4 = 0.5 * q_3;
    float twoSEq_1 = 2.0 * q_0;
    float twoSEq_2 = 2.0 * q_1;
    float twoSEq_3 = 2.0 * q_2;

    //Normalise the accelerometer measurement. v
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;

    f_1 = twoSEq_2 * q_3 - twoSEq_1 * q_2 - a_x;
    f_2 = twoSEq_1 * q_1 + twoSEq_3 * q_3 - a_y;
    f_3 = 1.0 - twoSEq_2 * q_1 - twoSEq_3 * q_2 - a_z;

    J_11or24 = twoSEq_3;
    J_12or23 = 2 * q_3;
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    J_32 = 2 * J_14or21;
    J_33 = 2 * J_11or24;

    nablaf_1 = J_14or21 * f_2 - J_11or24 * f_1;
    nablaf_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    nablaf_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    nablaf_4 = J_14or21 * f_1 + J_11or24 * f_2;

    norm = sqrt(nablaf_1 * nablaf_1 + nablaf_2 * nablaf_2 + nablaf_3 * nablaf_3 + nablaf_4 * nablaf_4);
    nablaf_1 /= norm;
    nablaf_2 /= norm;
    nablaf_3 /= norm;
    nablaf_4 /= norm;

    SEqDot_omega_1 = -halfSEq_2 * omega_x - halfSEq_3 * omega_y - halfSEq_4 * omega_z;
    SEqDot_omega_2 = halfSEq_1 * omega_x + halfSEq_3 * omega_z - halfSEq_4 * omega_y;
    SEqDot_omega_3 = halfSEq_1 * omega_y - halfSEq_2 * omega_z + halfSEq_4 * omega_x;
    SEqDot_omega_4 = halfSEq_1 * omega_z + halfSEq_2 * omega_y - halfSEq_3 * omega_x;

    //Compute then integrate the estimated quaternion rate.
    q_0 += (SEqDot_omega_1 - (BETA * nablaf_1)) * dt;
    q_1 += (SEqDot_omega_2 - (BETA * nablaf_2)) * dt;
    q_2 += (SEqDot_omega_3 - (BETA * nablaf_3)) * dt;
    q_3 += (SEqDot_omega_4 - (BETA * nablaf_4)) * dt;

    //Normalise quaternion
    norm = sqrt(q_0 * q_0 + q_1 * q_1 + q_2 * q_2 + q_3 * q_3);
    q_0 /= norm;
    q_1 /= norm;
    q_2 /= norm;
    q_3 /= norm;
}

void computeEuler(void){

    phi_raw = atan2(2 * (q_0 * q_1 + q_2 * q_3), 1 - 2 * (q_1 * q_1 + q_2 * q_2));

    float sin_theta = (2 * (q_0 * q_2 - q_3 * q_1));
    if (abs(sin_theta) >= 1)
        theta_raw = copysign(PI / 2, sin_theta); // Use 90 degrees if out of range
    else
        theta_raw = asin(sin_theta);

    psi_raw = atan2(2 * (q_0 * q_3 + q_1 * q_2), 1 - 2 * (q_2 * q_2 + q_3 * q_3));
}

void acc_gravity_compensate(float &ax, float &ay, float &az) {

    g_comp_1 = LPF_GRAVITY_COMP_ALFA * g_comp_1_old + (1 - LPF_GRAVITY_COMP_ALFA) * 9.81 * (2 * (q_1 * q_3 - q_0 * q_2));
    g_comp_2 = LPF_GRAVITY_COMP_ALFA * g_comp_2_old + (1 - LPF_GRAVITY_COMP_ALFA) * 9.81 * (2 * (q_0 * q_1 + q_2 * q_3));
    g_comp_3 = LPF_GRAVITY_COMP_ALFA * g_comp_3_old + (1 - LPF_GRAVITY_COMP_ALFA) * 9.81 * (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3);

    ax = ax - g_comp_1;
    ay = ay - g_comp_2;
    az = az - g_comp_3;

    g_comp_1_old = g_comp_1;
    g_comp_2_old = g_comp_2;
    g_comp_3_old = g_comp_3;
}

void soundSignal(int on_time, int off_time, int rep) {

    // startup: 200, 200, 1
    // BMP 280 initialization error: 100, 30, 2
    // MicroSD Card Reader initialization error: 100, 30, 3

    for (int i = 0; i < rep; i++) {
        digitalWrite(SOUND_PIN, HIGH);
        delay(on_time);
        digitalWrite(SOUND_PIN, LOW);
        delay(off_time);
    }
}

void saturate(float &value, float saturation) {

    if (value > abs(saturation)) value = abs(saturation);
    else if (value < -abs(saturation)) value = -abs(saturation);
    else value = value;
}

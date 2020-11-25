/*
 * mpu6050.h
 *
 *  Created on: Jul 20, 2018
 *      Author: Lucas Tiziani
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "init.h"
#include <math.h>

// IMU calibration variables
#define GYRO_THRESHOLD 0.0          // (deg/s) gyro rotation threshold
#define IMU_CAL_CYCLES 200          // number of calibration cycles over which to record data

// IMU sensitivity values
#define IMU_GYRO_SENS_250 131.07    // (bits/deg/s) +/-250 deg/s gyro sensitivity
#define IMU_GYRO_SENS_500 65.54     // (bits/deg/s) +/-500 deg/s gyro sensitivity
#define IMU_GYRO_SENS_1000 32.77    // (bits/deg/s) +/-1000 deg/s gyro sensitivity
#define IMU_GYRO_SENS_2000 16.38    // (bits/deg/s) +/-2000 deg/s gyro sensitivity
#define IMU_ACCEL_SENS_2 16384.0    // (bits/g) +/-2g accelerometer sensitivity
#define IMU_ACCEL_SENS_4 8192.0     // (bits/g) +/-4g accelerometer sensitivity
#define IMU_ACCEL_SENS_8 4096.0     // (bits/g) +/-8g accelerometer sensitivity
#define IMU_ACCEL_SENS_16 2048.0    // (bits/g) +/-16g accelerometer sensitivity

// IMU registers
#define IMU_ADDR 0x68               // I2C address
#define IMU_PWR_MGMT_1 0x6B         // power management register 1
#define IMU_CONFIG 0x1A             // IMU configuration (DLPF)
#define IMU_GYRO_CONFIG 0x1B        // gyro configuration
#define IMU_ACCEL_CONFIG 0x1C       // accelerometer configuration
#define IMU_GRYO_XOUT1_REG 0x43     // first x-axis velocity data register
#define IMU_ACCEL_XOUT1_REG 0x3B    // first x-axis accelerometer data register

// constants
#define PI 3.142  // pi

// structures
struct imuStruct {
    struct {
        float gyro;      // (bits/deg/s) gyro sensitivity
        float accel;     // (bits/g) accelerometer sensitivity
    } sens;
    struct {
        float angVelOffset[3];  // (deg/s) gyro angular velocity offset
    } cal;
    struct {
        float angVel[3];    // (deg/s) angular velocity
        float accel[3];     // (g) acceleration
    } raw;
    struct {
        float gyro[3];      // (deg) angle from gyro data
        float accel[3];     // (deg) angle from accel data
        float fused[3][2];  // (deg) angle history from fusion of data
    } angle;
    struct {
        float fused[3];           // (deg/s) angular velocity
    } angVel;
};

// function prototypes
void IMU_Init(volatile struct imuStruct* imu, int dlpfCutoff, int gyroRange, int accelRange);
void IMU_ReadVals(volatile struct imuStruct* imu);
void IMU_CalcAngleGyro(volatile float* angVel, volatile float* angle, float tInteg);
void IMU_CalcAngleAccel(volatile float* accel, volatile float* angle);
void IMU_CalcAngleFused(volatile struct imuStruct* imu);
void IMU_Calibrate(volatile struct imuStruct* imu, int cycleDelay);
void IMU_SelfTest(void);


#endif /* MPU6050_H_ */

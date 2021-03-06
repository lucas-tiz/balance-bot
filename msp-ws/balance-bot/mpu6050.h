/**
* @file mpu6050.h
* @brief MPU-6050 IMU driver
*
* MPU-6050 gyro/accelerometer driver & processing
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/

#ifndef MPU6050_H_
#define MPU6050_H_


#include "i2c_cust.h"
#include <math.h>


/* Macros */
#define IMU_CAL_CYCLES 200

#define IMU_GYRO_THRESHOLD 0.0      // (deg/s) gyro rotation threshold

#define IMU_GYRO_SENS_250 131.07    // (bits/deg/s) +/-250 deg/s gyro sensitivity
#define IMU_GYRO_SENS_500 65.54     // (bits/deg/s) +/-500 deg/s gyro sensitivity
#define IMU_GYRO_SENS_1000 32.77    // (bits/deg/s) +/-1000 deg/s gyro sensitivity
#define IMU_GYRO_SENS_2000 16.38    // (bits/deg/s) +/-2000 deg/s gyro sensitivity
#define IMU_ACCEL_SENS_2 16384.0    // (bits/g) +/-2g accelerometer sensitivity
#define IMU_ACCEL_SENS_4 8192.0     // (bits/g) +/-4g accelerometer sensitivity
#define IMU_ACCEL_SENS_8 4096.0     // (bits/g) +/-8g accelerometer sensitivity
#define IMU_ACCEL_SENS_16 2048.0    // (bits/g) +/-16g accelerometer sensitivity

#define IMU_ADDR 0x68               // I2C address

#define IMU_REG_PWR_MGMT1 0x6B      // power management register 1
#define IMU_REG_CONFIG 0x1A         // IMU configuration (DLPF)
#define IMU_REG_GYRO_CONFIG 0x1B    // gyro configuration
#define IMU_REG_ACCEL_CONFIG 0x1C   // accelerometer configuration
#define IMU_REG_GRYO_XOUT1 0x43     // first x-axis velocity data register
#define IMU_REG_ACCEL_XOUT1 0x3B    // first x-axis accelerometer data register

#define PI 3.142  // pi


/* Data types */
typedef struct {
    struct {
        float gyro;      // (bits/deg/s) gyro sensitivity
        float accel;     // (bits/g) accelerometer sensitivity
    } sens;
    struct {
        float ang_vel_offset[3];  // (deg/s) gyro angular velocity offset
    } cal;
    struct {
        float ang_vel[3];    // (deg/s) angular velocity
        float accel[3];     // (g) acceleration
    } raw;
    struct {
        float gyro[3];      // (deg) angle from gyro data
        float accel[3];     // (deg) angle from accel data
        float fused[3][2];  // (deg) angle history from fusion of data
    } angle;
    struct {
        float fused[3];           // (deg/s) angular velocity
    } ang_vel;
} imu_t;


/* Function prototypes */
void IMU_init(volatile imu_t * imu, int cutoff_dlpf, int range_gyro, int range_accel);
void IMU_readVals(volatile imu_t * imu);
void IMU_calcAngleGyro(volatile float * ang_vel, volatile float * angle, float t_integ);
void IMU_calcAngleAccel(volatile float* accel, volatile float* angle);
void IMU_calcAngleFused(volatile imu_t * imu, float period_sense);
void IMU_calibrate(volatile imu_t * imu, const int cycles, const int delay);
void IMU_selfTest(void);


#endif /* MPU6050_H_ */

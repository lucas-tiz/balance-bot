/*
 * mpu6050.c
 *
 *  Created on: Jul 20, 2018
 *      Author: Lucas Tiziani
 */

#include "mpu6050.h"
#include "i2c_lt.h"

//TODO: imu self test

void IMU_Init(volatile struct imuStruct* imu, int dlpfCutoff, int gyroRange, int accelRange) {
    /* initialize MPU6050 IMU */
    I2C1_Write(IMU_ADDR, IMU_PWR_MGMT_1, 0x03);     // disable sleep, set clock source to z-axis gyroscope reference

    // set DLPF cutoff (0x00=260, 0x01=184, 0x02=94, 0x03=44, 0x04=21, 0x05=10, 0x06=5)
    uint8_t dlpfReg;
    switch (dlpfCutoff) {
        case 260:
            dlpfReg = 0x00;
        case 184:
            dlpfReg = 0x01;
        case 94:
            dlpfReg = 0x02;
        case 44:
            dlpfReg = 0x03;
        case 21:
            dlpfReg = 0x04;
        case 10:
            dlpfReg = 0x05;
        case 5:
            dlpfReg = 0x06;
    }
    I2C1_Write(IMU_ADDR, IMU_CONFIG, dlpfReg);

    // set full scale range of gyro (0x00=250, 0x08=500, 0x10=1000, 0x18=2000)
    uint8_t gyroReg;
    switch (gyroRange) {
        case 250:
            gyroReg = 0x00;
            imu->sens.gyro = IMU_GYRO_SENS_250;
        case 500:
            gyroReg = 0x08;
            imu->sens.gyro = IMU_GYRO_SENS_500;
        case 1000:
            gyroReg = 0x10;
            imu->sens.gyro = IMU_GYRO_SENS_1000;
        case 2000:
            gyroReg = 0x18;
            imu->sens.gyro = IMU_GYRO_SENS_2000;
    }
    I2C1_Write(IMU_ADDR, IMU_GYRO_CONFIG, gyroReg);

    // set full scale range of accelerometer (0x00=2, 0x08=4, 0x10=8, 0x18=16)
    uint8_t accelReg;
    switch (accelRange) {
        case 2:
            accelReg = 0x00;
            imu->sens.accel = IMU_ACCEL_SENS_2;
        case 4:
            accelReg = 0x08;
            imu->sens.accel = IMU_ACCEL_SENS_4;
        case 8:
            accelReg = 0x10;
            imu->sens.accel = IMU_ACCEL_SENS_8;
        case 16:
            accelReg = 0x18;
            imu->sens.accel = IMU_ACCEL_SENS_16;
    }
    I2C1_Write(IMU_ADDR, IMU_ACCEL_CONFIG, accelReg);
}


void IMU_ReadVals(volatile struct imuStruct* imu) {
    /* read angular velocities and accelerations from IMU */
    int16_t accelCounts[3], gyroCounts[3]; // initialize data arrays
    I2C1_burstRead2(IMU_ADDR, IMU_GRYO_XOUT1_REG, 6, gyroCounts); // read gyro counts
    I2C1_burstRead2(IMU_ADDR, IMU_ACCEL_XOUT1_REG, 6, accelCounts); // read acceleration counts

    int axis;
    for (axis = 0; axis < 3; axis++) {
        imu->raw.angVel[axis] = (float)gyroCounts[axis]/imu->sens.gyro - imu->cal.angVelOffset[axis]; // (deg/sec) corrected for steady-state gyro offset
        imu->raw.accel[axis] = (float)accelCounts[axis]/imu->sens.accel; // (g)
    }
}


void IMU_CalcAngleGyro(volatile float* angVel, volatile float* angle, float tInteg) {
    /* calculate pitch, roll, yaw from gyro data */
    static float angVelPrev[3];

    int axis;
    for (axis = 0; axis < 3; axis++) {
        if (angVel[axis] >= GYRO_THRESHOLD || angVel[axis] <= -GYRO_THRESHOLD) { // check gyro angular velocity threshold
            angle[axis] = angle[axis] + ((angVel[axis] + angVelPrev[axis])/2.0)*tInteg; // integrate angular velocity
        }
        angVelPrev[axis] = angVel[axis]; // set current ang vel as previous
    }
}


void IMU_CalcAngleAccel(volatile float* accel, volatile float* angle) {
    /* calculate pitch, yaw from accelerometer data */
    angle[0] = -atan2(accel[2], (accel[1]/fabs(accel[1]))*sqrt(pow(accel[0],2) + pow(accel[1],2)))*(180.0/PI); // pitch about x-axis //TODO: check this formula
    angle[2] = -atan2(-accel[0], accel[1])*(180.0/PI); // role about z-axis
}


void IMU_CalcAngleFused(volatile struct imuStruct* imu) {
    /* calculate orientation based on IMU data */
    //TODO: implement yaw, roll later if necessary

    IMU_CalcAngleGyro(imu->raw.angVel, imu->angle.gyro, 1/IMU_CALC_FREQ); // calculate angles from gyro data
    IMU_CalcAngleAccel(imu->raw.accel, imu->angle.accel);       // calculate angles from acceleration data

    // correct drift of gyro
    imu->angle.gyro[0] = 0.9996*imu->angle.gyro[0] + 0.0004*imu->angle.accel[0];

    // shift angular velocity history
    imu->angle.fused[0][1] = imu->angle.fused[0][0];

    // combine gyro and accelerometer data
    imu->angle.fused[0][0] = 0.90*imu->angle.gyro[0] + 0.10*imu->angle.accel[0]; // calculate complementary filtered pitch

    // calculate angular velocity
    imu->angVel.fused[0] = (imu->angle.fused[0][0] - imu->angle.fused[0][1])*IMU_CALC_FREQ;
}


void IMU_Calibrate(volatile struct imuStruct* imu, int cycleDelay) {
    /* initialize gyro values based on accelerometer values: must be stationary */
    float angVelCalData[IMU_CAL_CYCLES][3];  // initialize data array of angular velocities
    float accelCalData[IMU_CAL_CYCLES][3];   // initialize data array of accelerations

    int calCount; // zero calibration count
    int axis; // coordinate frame axis
    int i;
    for (calCount = 0; calCount < IMU_CAL_CYCLES; calCount++) {
        IMU_ReadVals(imu); // read angular velocities and accelerations
        for (axis = 0; axis < 3; axis++) {
            angVelCalData[calCount][axis] = imu->raw.angVel[axis];    // record ang vel data
            accelCalData[calCount][axis] = imu->raw.accel[axis];  // record accel data
        }
        for (i = 0; i < cycleDelay; i++); // delay
    }

    // calculate average value for each axis of accelerometer and gyro
    float angVelCalAvg[3] = {0,0,0};
    float accelCalAvg[3] = {0,0,0};
    for(axis = 0; axis < 3; axis++) {
        for (calCount = 0; calCount < IMU_CAL_CYCLES; calCount++) {
            angVelCalAvg[axis] = angVelCalAvg[axis] + angVelCalData[calCount][axis];
            accelCalAvg[axis] = accelCalAvg[axis] + accelCalData[calCount][axis];
        }

        angVelCalAvg[axis] = angVelCalAvg[axis]/(float)IMU_CAL_CYCLES; // calculate average angular velocity
        imu->cal.angVelOffset[axis] = angVelCalAvg[axis]; // set gyro ang vel offset to average
        accelCalAvg[axis] = accelCalAvg[axis]/(float)IMU_CAL_CYCLES; // calculate average accelerations
    }
    float angles[3];
    IMU_CalcAngleAccel(accelCalAvg, angles); // calculate orientation from average accelerations

    imu->angle.gyro[0] = angles[0]; // zero gyro pitch based on accelerometer
    imu->angle.gyro[2] = angles[2]; // zero gyro roll based on accelerometer
}


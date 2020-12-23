/**
* @file mpu6050.c
* @brief MPU-6050 IMU driver
*
* MPU-6050 gyro/accelerometer driver & processing
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/


/* TODO
 * pass IMU_CALC_FREQ as param?
 * implement yaw/roll
 * imu self test
 * make local functions (within module) static
 */


#include "mpu6050.h"


void IMU_init(volatile imu_t * imu, int cutoff_dlpf, int range_gyro, int range_accel) {
    /* Initialize MPU6050 IMU */
    I2Cc_write(IMU_ADDR, IMU_REG_PWR_MGMT1, 0x03); // disable sleep, set clock source to z-axis gyroscope reference

    // set DLPF cutoff (0x00=260, 0x01=184, 0x02=94, 0x03=44, 0x04=21, 0x05=10, 0x06=5)
    uint8_t reg_dlpf;
    switch (cutoff_dlpf) {
        case 260:
            reg_dlpf = 0x00;
        case 184:
            reg_dlpf = 0x01;
        case 94:
            reg_dlpf = 0x02;
        case 44:
            reg_dlpf = 0x03;
        case 21:
            reg_dlpf = 0x04;
        case 10:
            reg_dlpf = 0x05;
        case 5:
            reg_dlpf = 0x06;
    }
    I2Cc_write(IMU_ADDR, IMU_REG_CONFIG, reg_dlpf);

    // set full scale range of gyro (0x00=250, 0x08=500, 0x10=1000, 0x18=2000)
    uint8_t reg_gyro;
    switch (range_gyro) {
        case 250:
            reg_gyro = 0x00;
            imu->sens.gyro = IMU_GYRO_SENS_250;
        case 500:
            reg_gyro = 0x08;
            imu->sens.gyro = IMU_GYRO_SENS_500;
        case 1000:
            reg_gyro = 0x10;
            imu->sens.gyro = IMU_GYRO_SENS_1000;
        case 2000:
            reg_gyro = 0x18;
            imu->sens.gyro = IMU_GYRO_SENS_2000;
    }
    I2Cc_write(IMU_ADDR, IMU_REG_GYRO_CONFIG, reg_gyro);

    // set full scale range of accelerometer (0x00=2, 0x08=4, 0x10=8, 0x18=16)
    uint8_t reg_accel;
    switch (range_accel) {
        case 2:
            reg_accel = 0x00;
            imu->sens.accel = IMU_ACCEL_SENS_2;
        case 4:
            reg_accel = 0x08;
            imu->sens.accel = IMU_ACCEL_SENS_4;
        case 8:
            reg_accel = 0x10;
            imu->sens.accel = IMU_ACCEL_SENS_8;
        case 16:
            reg_accel = 0x18;
            imu->sens.accel = IMU_ACCEL_SENS_16;
    }
    I2Cc_write(IMU_ADDR, IMU_REG_ACCEL_CONFIG, reg_accel);
}


void IMU_readVals(volatile imu_t * imu) {
    /* Read angular velocities and accelerations from IMU */
    int16_t counts_accel[3], counts_gyro[3]; // initialize data arrays
    I2Cc_burstRead2(IMU_ADDR, IMU_REG_GRYO_XOUT1, 6, counts_gyro); // read gyro counts
    I2Cc_burstRead2(IMU_ADDR, IMU_REG_ACCEL_XOUT1, 6, counts_accel); // read acceleration counts

    int axis;
    for (axis = 0; axis < 3; axis++) {
        imu->raw.ang_vel[axis] = (float)counts_gyro[axis]/imu->sens.gyro
            - imu->cal.ang_vel_offset[axis]; // (deg/sec) corrected for steady-state gyro offset
        imu->raw.accel[axis] = (float)counts_accel[axis]/imu->sens.accel; // (g)
    }
}


void IMU_calcAngleGyro(volatile float * ang_vel, volatile float * angle, float t_integ) {
    /* Calculate pitch, roll, yaw from gyro data */
    static float ang_vel_prev[3];

    int axis;
    for (axis = 0; axis < 3; axis++) {
        if (ang_vel[axis] >= IMU_GYRO_THRESHOLD || ang_vel[axis] <= -IMU_GYRO_THRESHOLD) { // check gyro angular velocity threshold
            angle[axis] = angle[axis] + ((ang_vel[axis] + ang_vel_prev[axis])/2.0)*t_integ; // integrate angular velocity
        }
        ang_vel_prev[axis] = ang_vel[axis]; // set current ang vel as previous
    }
}


void IMU_calcAngleAccel(volatile float * accel, volatile float * angle) {
    /* Calculate pitch, yaw from accelerometer data */
    angle[0] = -atan2(accel[2], (accel[1]/fabs(accel[1]))*sqrt(pow(accel[0],2)
        + pow(accel[1],2)))*(180.0/PI); // pitch about x-axis //TODO: check this formula
    angle[2] = -atan2(-accel[0], accel[1])*(180.0/PI); // roll about z-axis
}


void IMU_calcAngleFused(volatile imu_t * imu) {
    /* Calculate orientation based on IMU data */
    //TODO: implement yaw, roll later if necessary

    IMU_calcAngleGyro(imu->raw.ang_vel, imu->angle.gyro, 1/IMU_CALC_FREQ); // calculate angles from gyro data
    IMU_calcAngleAccel(imu->raw.accel, imu->angle.accel);       // calculate angles from acceleration data

    // correct drift of gyro
    imu->angle.gyro[0] = 0.9996*imu->angle.gyro[0] + 0.0004*imu->angle.accel[0]; //TODO: configure fusion coefficients

    // shift angular velocity history
    imu->angle.fused[0][1] = imu->angle.fused[0][0];

    // combine gyro and accelerometer data
    imu->angle.fused[0][0] = 0.90*imu->angle.gyro[0] + 0.10*imu->angle.accel[0]; // calculate complementary filtered pitch

    // calculate angular velocity
    imu->ang_vel.fused[0] = (imu->angle.fused[0][0] - imu->angle.fused[0][1])*IMU_CALC_FREQ;
}


void IMU_calibrate(volatile imu_t * imu, int delay_cycle) {
    /* Initialize gyro values based on accelerometer values: must be stationary */
    float ang_vel_cal[IMU_CAL_CYCLES][3];  // initialize data array of angular velocities
    float accel_cal[IMU_CAL_CYCLES][3];   // initialize data array of accelerations

    int counts_cal; // zero calibration count
    int axis; // coordinate frame axis
    int i;
    for (counts_cal = 0; counts_cal < IMU_CAL_CYCLES; counts_cal++) {
        IMU_readVals(imu); // read angular velocities and accelerations
        for (axis = 0; axis < 3; axis++) {
            ang_vel_cal[counts_cal][axis] = imu->raw.ang_vel[axis];    // record ang vel data
            accel_cal[counts_cal][axis] = imu->raw.accel[axis];  // record accel data
        }
        for (i = 0; i < delay_cycle; i++); // delay
    }

    // calculate average value for each axis of accelerometer and gyro
    float ang_vel_cal_avg[3] = {0,0,0};
    float accel_cal_avg[3] = {0,0,0};
    for(axis = 0; axis < 3; axis++) {
        for (counts_cal = 0; counts_cal < IMU_CAL_CYCLES; counts_cal++) {
            ang_vel_cal_avg[axis] = ang_vel_cal_avg[axis] + ang_vel_cal[counts_cal][axis];
            accel_cal_avg[axis] = accel_cal_avg[axis] + accel_cal[counts_cal][axis];
        }

        ang_vel_cal_avg[axis] = ang_vel_cal_avg[axis]/(float)IMU_CAL_CYCLES; // calculate average angular velocity
        imu->cal.ang_vel_offset[axis] = ang_vel_cal_avg[axis]; // set gyro ang vel offset to average
        accel_cal_avg[axis] = accel_cal_avg[axis]/(float)IMU_CAL_CYCLES; // calculate average accelerations
    }
    float angles[3];
    IMU_calcAngleAccel(accel_cal_avg, angles); // calculate orientation from average accelerations

    imu->angle.gyro[0] = angles[0]; // zero gyro pitch based on accelerometer
    imu->angle.gyro[2] = angles[2]; // zero gyro roll based on accelerometer
}


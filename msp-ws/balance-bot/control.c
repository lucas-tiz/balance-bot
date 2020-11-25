/*
 * control.c
 *
 *  Created on: Jul 25, 2018
 *      Author: Lucas Tiziani
 */


#include "control.h"


void Motor_VelUpdate(struct motorStruct* motor, float motorVel) {
    /* motor velocity controller: PID */
    float err = motorVel - motor->velPID.velDes; // calculate proportional error
    motor->velPID.errInt += err; // integrate error
    float errDer = err - motor->velPID.errPrev; // derivative of error (time step is consistent so it's not required)
    motor->velPID.errPrev = err; // set previous error to current error

    // reset integral windup
    if ((motor->velPID.errInt)*(motor->velPID.kP) > 100.0) {
        motor->velPID.errInt = 100.0/(motor->velPID.kP);
    }
    else if ((motor->velPID.errInt)*(motor->velPID.kP) < -100.0) {
        motor->velPID.errInt = -100.0/(motor->velPID.kP);
    }

    float u = -(motor->velPID.kP)*err - (motor->velPID.kI)*(motor->velPID.errInt) - (motor->velPID.kD)*errDer; // calculate raw controller input

    //impose deadzone compensation
    if (u > 0) {
        u += motor->deadZone;
    }
    else if (u < 0)  {
        u -= motor->deadZone;
    }

    // impose saturation limits
    if (u > 100.0) {
        u = 100.0;
    }
    else if (u < -100.0) {
        u = -100.0;
    }

    // impose velocity dead-band
    if (abs(motor->velPID.velDes) < 1) {
        u = 0;
    }

    int duty = (int)(MOTOR_PERIOD*(u/100.0)); // calculate motor PWM timer duty cycle

//    debugVar = u; //TODO: debug only


    // set duty cycle depending on motor direction
    if (duty >= 0) { // if duty cycle is positive, go forward
        *(motor->duty[1]) = 0;  // set backward PWM signal to zero
        *(motor->duty[0]) = duty; // set forward PWM signal to duty cycle
        LED2_Set(LED_GREEN); //TODO: for debug only
    }
    else { // if duty cycle is negative, go backward
        *(motor->duty[0]) = 0;  // set backward PWM signal to zero
        *(motor->duty[1]) = -duty; // set forward PWM signal to duty cycle
        LED2_Set(LED_RED); //TODO: for debug only
    }
}


//
//
//// stop input voltage to motor
//void stopMotorInput(void)
//{
//    MOTOR_CW_DUTY_CYCLE = 0; // set CW PWM signal to zero
//    MOTOR_CCW_DUTY_CYCLE = 0; // set CCW PWM signal to zero
//}


// LQR controller: calculate desired angular velocity of base wheel
void Sys_ControlUpdate(void) {
    /* update motor velocity set-points based on sensor measurements*/

    //TODO: put gains somewhere
    static float kLqr[4] = {-0.1000, -0.8716, -410.4197, -71.6509};

    // create state vector of angles and angular velocities
    float x[4];
    x[0] = (encR.pos[0])*DEG_TO_RAD;           // alpha: wheel angle relative to chassis
    x[1] = (encR.velFilt)*DEG_TO_RAD;          // alpha vel: wheel angular velocity relative to chassis
    x[2] = (imu.angle.fused[0][0])*DEG_TO_RAD; // theta2: chassis angle
    x[3] = (imu.angVel.fused[0])*DEG_TO_RAD;   // theta2 vel: chassis angular velocity

    // calculate acceleration input
    int i;
    float alphaAccel = 0; // initialize desired wheel angular acceleration relative to chassis
    for (i = 0; i < 4; i++) {
        alphaAccel = alphaAccel - kLqr[i]*x[i]; // calculate acceleration from LQR state-feedback control law
    }

//    // impose acceleration limit
//    if (alphaDotDot_des > ACCEL_LIMIT)
//    {
//        alphaDotDot_des = ACCEL_LIMIT;
//    }
//    else if (alphaDotDot_des < -ACCEL_LIMIT)
//    {
//        alphaDotDot_des = -ACCEL_LIMIT;
//    }

    // integrate acceleration to get velocity input
    float alphaVel = 0;
    alphaVel = alphaVel + (alphaAccel/CONTROL_FREQ)*RAD_TO_DEG;

    debugVar = alphaVel;

//    // impose velocity limit
//    if (alphaDot_des > VEL_LIMIT)
//    {
//        alphaDot_des = VEL_LIMIT;
//    }
//    else if (alphaDot_des < -VEL_LIMIT)
//    {
//        alphaDot_des = -VEL_LIMIT;
//    }
}




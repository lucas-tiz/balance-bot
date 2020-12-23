/**
* @file motor.c
* @brief DC motor control
*
* DC motor velocity control
*
* @author Lucas Tiziani
* @date 2020-12-15
*
*/


#include "motor.h"


void Motor_velUpdate(motor_t * motor, float vel_motor) {
    /* Motor velocity controller: PID */
    float err = vel_motor - motor->pid_vel.vel_des;
    motor->pid_vel.err_int += err;
    float err_der = err - motor->pid_vel.err_prev; // assume consistent timestep
    motor->pid_vel.err_prev = err; // set previous error to current error

    // reset integral windup
    if ((motor->pid_vel.err_int)*(motor->pid_vel.k_p) > 100.0) {
        motor->pid_vel.err_int = 100.0/(motor->pid_vel.k_p);
    }
    else if ((motor->pid_vel.err_int)*(motor->pid_vel.k_p) < -100.0) {
        motor->pid_vel.err_int = -100.0/(motor->pid_vel.k_p);
    }

    // calculate raw control
    float u = -(motor->pid_vel.k_p)*err - (motor->pid_vel.k_i)*(motor->pid_vel.err_int)
        - (motor->pid_vel.k_d)*err_der;

    //impose deadzone compensation
    if (u > 0) {
        u += motor->deadzone;
    }
    else if (u < 0)  {
        u -= motor->deadzone;
    }

    // impose saturation limits
    if (u > 100.0) {
        u = 100.0;
    }
    else if (u < -100.0) {
        u = -100.0;
    }

    // impose velocity dead-band
    if (abs(motor->pid_vel.vel_des) < 1) {
        u = 0;
    }

    // set duty cycle depending on motor direction
    int duty = (int)(MOTOR_PERIOD*(u/100.0)); // calculate motor PWM timer duty cycle

    if (duty >= 0) { // if duty cycle is positive, go forward
        *(motor->reg_duty.back) = 0;  // set backward PWM signal to zero
        *(motor->reg_duty.forward) = duty; // set forward PWM signal to duty cycle
    }
    else { // if duty cycle is negative, go backward
        *(motor->reg_duty.back) = 0;  // set backward PWM signal to zero
        *(motor->reg_duty.forward) = -duty; // set forward PWM signal to duty cycle
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







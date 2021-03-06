/**
* @file motor.h
* @brief DC motor control
*
* DC motor velocity control
*
* @author Lucas Tiziani
* @date 2020-12-15
*
*/

#ifndef MOTOR_H_
#define MOTOR_H_


//#include "encoder.h"
#include "driverlib.h"
//#include "init.h"
#include "util.h"
#include <stdint.h>


//* Data types */
typedef struct {
//    const uint8_t pins[2]; // motor pins (0=forward, 1=backward)
//    volatile uint16_t* reg_duty[2]; // duty cycle timer registers corresponding to pins (pointers to registers)

    struct {
        volatile uint16_t * forward;
        volatile uint16_t * back;
    } reg_duty;

    const float deadzone; // (% duty cycle) motor dead zone
    struct {
        const float k_p; // motor velocity control proportional gain
        const float k_i; // motor velocity control integral gain
        const float k_d; // motor velocity control derivative
        volatile float vel_des; // motor velocity setpoint
        volatile float err_int; // motor velocity integrated error
        volatile float err_prev; // motor velocity previous error
    } pid_vel;
} motor_t;


/* Function prototypes */
void Motor_velUpdate(motor_t * motor, float vel_motor, int period_motor);


#endif /* MOTOR_H_ */

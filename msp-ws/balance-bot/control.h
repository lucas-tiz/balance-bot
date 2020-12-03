/*
 * control.h
 *
 *  Created on: Jul 25, 2018
 *      Author: Lucas Tiziani
 */

#ifndef CONTROL_H_
#define CONTROL_H_


#include "encoder.h"
#include "driverlib.h"
#include "init.h"
#include <stdint.h>

#define DEG_TO_RAD 0.0174533         // degrees to radians
#define RAD_TO_DEG 57.2958           // radians to degrees

// structures
struct motorStruct {
    const uint8_t pins[2]; // motor pins (0=forward, 1=backward)
    volatile uint16_t* duty[2]; // duty cycle timer registers corresponding to pins (pointers to registers)
    const float deadZone; // (% duty cycle) motor dead zone
    struct {
        const float kP; // motor velocity control proportional gain
        const float kI; // motor velocity control integral gain
        const float kD; // motor velocity control derivative
        volatile float velDes; // motor velocity setpoint
        volatile float errInt; // motor velocity integrated error
        volatile float errPrev; // motor velocity previous error
    } velPID;
};

// prototypes
void Motor_VelUpdate(struct motorStruct* motor, float motorVel);
void Sys_ControlUpdate(void);


#endif /* CONTROL_H_ */
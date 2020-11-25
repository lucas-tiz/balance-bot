/*
 * encoder.h
 *
 *  Created on: Jul 20, 2018
 *      Author: Lucas Tiziani
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "init.h"
#include "driverlib.h"
#include <stdint.h>

// encoder setup
#define NUM_ENC_CH 2                // number of encoder channels (A,B)
#define DEG_PER_COUNT 360.0/1400.0  // deg per count
#define LPF_ORDER 20                // low-pass velocity filter order

// structures
struct encStruct {
    uint8_t port;               // encoder port
    uint8_t pins[NUM_ENC_CH];   // encoder pins corresponding to encoder channels
    uint8_t prevState;          // previous encoder state
    int count;                  // encoder pulse count
    float pos[2];               // (deg) wheel angle history (current and previous)
    float vel[LPF_ORDER+1];     // (deg/s) wheel angular velocity history
    float velFilt;              // (deg/s) filtered wheel angular velocity
};


// function prototypes
void Enc_Initialize(volatile struct encStruct* enc);
void Enc_InterruptRoutine(volatile struct encStruct* enc);
void Enc_CalcAngle(volatile struct encStruct* enc);


#endif /* ENCODER_H_ */

/**
* @file enc.h
* @brief Encoder driver
*
* Encoder driver for reading angle
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/

#ifndef ENC_H_
#define ENC_H_


#include "driverlib.h"
#include <stdint.h>


/* Macros */
#define ENC_NUM_CH 2                    // number of encoder channels (A,B)
#define ENC_DEG_PER_COUNT 360.0/1400.0  // deg per count
#define ENC_LPF_ORDER 20                // low-pass velocity filter order
#define ENC_CALC_FREQ 100.0       // (Hz) frequency at which encoder angles are calculated
#define LPF_ORDER 20

/* Data types */
typedef struct {
    uint8_t port;             // encoder port
    uint8_t pins[ENC_NUM_CH]; // encoder pins corresponding to encoder channels
    volatile uint8_t state_prev;    // previous encoder state
    volatile int count;             // encoder pulse count
    float pos[2];                   // (deg) wheel angle history (current and previous)
    float vel[ENC_LPF_ORDER+1];     // (deg/s) wheel angular velocity history
    float vel_filt;                 // (deg/s) filtered wheel angular velocity
} enc_t;


/* Function prototypes */
void Enc_init(enc_t * enc);
void Enc_update(enc_t * enc);
void Enc_calcAngle(enc_t * enc);


#endif /* ENC_H_ */

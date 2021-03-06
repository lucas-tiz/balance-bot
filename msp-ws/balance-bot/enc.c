/**
* @file enc.c
* @brief Encoder driver
*
* Encoder driver for reading angle
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/


/* TODO
 * make encoder channels, degrees per count, LPF order configurable
 * move LPF coefficients to configuration
 * pass encoder read frequency to Enc_calcAngle
 */


#include "enc.h"


void Enc_init(enc_t * enc) {
    /* Initialize encoder based on current channel states */
    int i; // channel index
//    int pins_or; // bitwise OR of all encoder pins
    uint8_t state_chan[2]; // encoder channel states
    for (i = 0; i < 2; i++) { // loop over encoder channels A & B

        state_chan[i] = MAP_GPIO_getInputPinValue(enc->port, enc->pins[i]); // get encoder channel state
        if (state_chan[i] == GPIO_INPUT_PIN_LOW) { // if channel is low
            MAP_GPIO_interruptEdgeSelect(enc->port, enc->pins[i], GPIO_LOW_TO_HIGH_TRANSITION); // set low to high transition
        }
        else { // if channel is high
            MAP_GPIO_interruptEdgeSelect(enc->port, enc->pins[i], GPIO_HIGH_TO_LOW_TRANSITION); // set high to low transition
        }
//        pins_or |= enc->pins[i]; // add channel pin to bitwise OR
    }
    enc->state_prev = (state_chan[0] << 1) | state_chan[1]; // set previous encoder state to current encoder state
//        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, encPins[iEnc][2], GPIO_LOW_TO_HIGH_TRANSITION); // set low to high transition for channel X
//        pins_or |= encPins[iEnc][2]; // add channel pin to bitwise OR

//    MAP_GPIO_clearInterruptFlag(enc->port, pins_or); // clear all interrupt flags
//    MAP_GPIO_enableInterrupt(enc->port, pins_or); // enable GPIO interrupt
//    MAP_Interrupt_enableInterrupt(INT_PORT3); // enable interrupt in NVIC
}


void Enc_update(enc_t * enc) {
    /* encoder interrupt routine: determine which channel of encoder(s) caused interrupt, change encoder counts accordingly */
//    uint_fast16_t status = MAP_GPIO_getEnab00InterruptStatus(GPIO_PORT_P3); // get port 3 interrupt status
//
//    int iEnc; // encoder index
//    if (status <= (encPins[0][0] | encPins[0][1])) { // check which encoder (| encPins[0][2] for index channel X)
//        iEnc = 0; // interrupt is for encoder 0
//    }
//    else {
//        iEnc = 1; // interrupt is for encoder 1
//    }

    uint8_t state_chan[2] = {MAP_GPIO_getInputPinValue(enc->port, enc->pins[0]), // get encoder channels A, B values
                                MAP_GPIO_getInputPinValue(enc->port, enc->pins[1])};
    uint8_t state = (state_chan[0] << 1) | state_chan[1]; // combine channels into encoder state

//    if ((status & encPins[iEnc][2]) && (sysState[0] == 1)) { // if index interrupt
//        encCounts[iEnc] = 0; // reset encoder count
//    }... else

    uint8_t state_comb = (state << 2) | enc->state_prev; // combine current state and previous state
    int8_t inc_count; // encoder counter increment
    switch (state_comb)
    {
        case 0: // do nothing
            inc_count = 0;
            break;
        case 1: // count up
            inc_count = 1;
            break;
        case 2: // count down
            inc_count = -1;
            break;
        case 3: // do nothing - this is an error
            inc_count = 0;
            break;
        case 4: // count down
            inc_count = -1;
            break;
        case 5: // do nothing
            inc_count = 0;
            break;
        case 6: // do nothing - this is an error
            inc_count = 0;
            break;
        case 7: // count up
            inc_count = 1;
            break;
        case 8: // count up
            inc_count = 1;
            break;
        case 9: // do nothing - this is an error
            inc_count = 0;
            break;
        case 10: // do nothing
            inc_count = 0;
            break;
        case 11: // count down
            inc_count = -1;
            break;
        case 12: // do nothing - this is an error
            inc_count = 0;
            break;
        case 13: // count down
            inc_count = -1;
            break;
        case 14: // count up
            inc_count = 1;
            break;
        case 15: // do nothing
            inc_count = 0;
            break;
    }
    enc->count = enc->count + inc_count; // increment/decrement encoder count appropriately

    int i; // channel index
    for (i = 0; i < 2; i++) { // loop over encoder channels A & B
       if (state_chan[i] == GPIO_INPUT_PIN_LOW) { // if channel is low
           MAP_GPIO_interruptEdgeSelect(enc->port, enc->pins[i], GPIO_LOW_TO_HIGH_TRANSITION); // set low to high transition
       }
       else { // if channel is high
           MAP_GPIO_interruptEdgeSelect(enc->port, enc->pins[i], GPIO_HIGH_TO_LOW_TRANSITION); // set high to low transition
       }
    }

    enc->state_prev = state; // set previous encoder state to current encoder state
}


void Enc_calcAngle(enc_t * enc) {
    /* convert encoder counts to angle, differentiate angle, filter angular velocity  */
    static const float lpfCoeffs[ENC_LPF_ORDER+1] = {-0.00081606,-0.00348667,
        -0.00846865,-0.01369406,-0.01349162,0.00002764,0.03264918,0.08284379,
        0.13937502,0.18445740,0.20170049,0.18445740,0.13937502,0.08284379,
        0.03264918,0.00002764,-0.01349162,-0.01369406,-0.00846865,-0.00348667,
        -0.00081606};

    enc->pos[1] = enc->pos[0];// shift angle histories
    enc->pos[0] = (float)(enc->count)*ENC_DEG_PER_COUNT; // convert counts to angle

    // shift angular velocity histories
    int i = 0;
    for (i = 0; i < LPF_ORDER; i++) {
        enc->vel[LPF_ORDER-i] = enc->vel[ENC_LPF_ORDER-(i+1)]; // shift each value right
    }

    enc->vel[0] = (enc->pos[0]-enc->pos[1])*ENC_CALC_FREQ; // differentiate angle

    // calculate filtered angular velocity
    float tmp = 0;
    for (i = 0; i <= (LPF_ORDER+1); i++) {
        tmp = tmp + lpfCoeffs[i]*enc->vel[i];
    }
    enc->vel_filt = tmp;
}




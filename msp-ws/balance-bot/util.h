/**
* @file util.h
* @brief LED and delay utilities
*
* LED and delay utilities for MSP432
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/


#ifndef UTIL_H_
#define UTIL_H_


#include "driverlib.h"
#include <stdint.h>


/* Macros */
#define HZ_PER_MS 12000 // clock Hz per millisecond delay


/* Function prototypes */
void LED2_set(int state);
void delayMs(int clockFreq, int n);


#endif /* UTIL_H_ */

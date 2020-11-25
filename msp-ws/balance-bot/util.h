/*
 * util.h
 *
 *  Created on: Jul 24, 2018
 *      Author: Lucas Tiziani
 */

#ifndef UTIL_H_
#define UTIL_H_


#include "driverlib.h"
#include <stdint.h>

#define HZ_PER_MS 12000 // clock Hz per millisecond delay

// prototypes
void LED2_Set(int state);
void delayMs(int clockFreq, int n);


#endif /* UTIL_H_ */

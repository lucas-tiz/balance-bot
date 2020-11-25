/*
 * util.c
 *
 *  Created on: Jul 24, 2018
 *      Author: Lucas Tiziani
 */

#include "util.h"


void LED2_Set(int state) {
    /* LED2 state: 0 = OFF, 1 = RED, 2 = GREEN, 3 = BLUE */

    switch (state) {
        case 0:
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2); // turn all off
            break;
        case 1:
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
            break;
        case 2:
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
            break;
        case 3:
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
            break;
    }
}


void delayMs(int clockFreq, int n) {
    /* millisecond delay for 48 MHz clock */
    int i, j;

    for (j = 0; j < n; j++){
        for (i = (clockFreq/HZ_PER_MS); i > 0; i--); // delay 1 ms
    }
}

/*
 * uart_lt.c
 *
 *  Created on: Jul 24, 2018
 *      Author: Lucas Tiziani
 */

#include "uart_lt.h"


void UART_SendFloatArray(float* arr, int arrLen) {
    /* transmit float data array over UART */
    int i,j;
    char str[20];
    for (i = 0; i < arrLen; i++) { // loop over data in data vector
        sprintf(str, "%12.3f", arr[i]); // convert data float to string
        j = 0;
        while (str[j] != 0x00) {
            MAP_UART_transmitData(EUSCI_A0_BASE, str[j]); // transmit digit in data string
            while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmit buffer empty
            j++;
        }
        MAP_UART_transmitData(EUSCI_A0_BASE, 0x20); // transmit space between vector values
        while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmit buffer empty
    }
    MAP_UART_transmitData(EUSCI_A0_BASE, 0x0D); // transmit carriage return between data sets
    while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmit buffer empty
    MAP_UART_transmitData(EUSCI_A0_BASE, 0x0A); // transmit new line between data sets
    while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmit buffer empty
}


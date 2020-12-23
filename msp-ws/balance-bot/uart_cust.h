/**
* @file uart_cust.h
* @brief UART send/receive
*
* Send/receive strings via UART
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/


#ifndef UART_CUST_H_
#define UART_CUST_H_


#include "driverlib.h"
#include <stdio.h>


/* Function prototypes */
void UARTc_sendFloatArray(float* arr, int len_arr);


#endif /* UART_CUST_H_ */

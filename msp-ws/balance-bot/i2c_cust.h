/**
* @file i2c_cust.h
* @brief MSP432 I2C driver
*
* MSP432 I2C driver
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/

#ifndef I2C_CUST_H_
#define I2C_CUST_H_


//#include "driverlib.h"
#include "msp.h"
#include <stdint.h>


/* Function prototypes */
int I2Cc_write(int addr_slave, unsigned char addr_mem, unsigned char data);
int I2Cc_read(int addr_slave, unsigned char addr_mem, unsigned char* data);
int I2Cc_burstRead2(int addr_lsave, unsigned char addr_mem, int count_byte, int16_t* data);


#endif /* I2C_CUST_H_ */

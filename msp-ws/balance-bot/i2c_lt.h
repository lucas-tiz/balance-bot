/*
 * i2c_lucas.h
 *
 *  Created on: Jul 20, 2018
 *      Author: Lucas Tiziani
 */

#ifndef I2C_LT_H_
#define I2C_LT_H_

#include "driverlib.h"
#include <stdint.h>

// function prototypes
int I2C1_Write(int slaveAddr, unsigned char memAddr, unsigned char data);
int I2C1_Read(int slaveAddr, unsigned char memAddr, unsigned char* data);
int I2C1_burstRead2(int slaveAddr, unsigned char memAddr, int byteCount, int16_t* data);


#endif /*I2C_LUCAS_H_*/

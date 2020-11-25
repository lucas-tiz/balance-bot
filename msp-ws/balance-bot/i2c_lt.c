/*
 * i2c_lucas.c
 *
 *  Created on: Jul 20, 2018
 *      Author: Lucas Tiziani
 */

#include "i2c_lt.h"


int I2C1_Write(int slaveAddr, unsigned char memAddr, unsigned char data) {
    /* write single byte to I2C module 1 */
    EUSCI_B1->I2CSA = slaveAddr;    // set slave address
    EUSCI_B1->CTLW0 |= 0x0010;      // enable transmitter
    EUSCI_B1->CTLW0 |= 0x0002;      // generate START and send slave address

    while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
    EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
    EUSCI_B1->TXBUF = memAddr;      // send memory address to slave

    while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
    EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
    EUSCI_B1->TXBUF = data;         // send data to slave

    while(!(EUSCI_B1->IFG & 2));    // wait until last transmit is done
    EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
    EUSCI_B1->CTLW0 |= 0x0004;      // send STOP
    while(EUSCI_B1->CTLW0 & 4);     // wait until STOP is sent
    EUSCI_B1->IFG &= ~8;            // clear STOP interrupt flag

    return 0;                       // no error
}


int I2C1_Read(int slaveAddr, unsigned char memAddr, unsigned char* data) {
    /* read single byte from I2C module 1*/
    EUSCI_B1->I2CSA = slaveAddr;    // set slave address
    EUSCI_B1->CTLW0 |= 0x0010;      // enable transmitter
    EUSCI_B1->CTLW0 |= 0x0002;      // generate START and send slave address

    while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
    EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
    EUSCI_B1->TXBUF = memAddr;      // send memory address to slave

    while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
    EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
    EUSCI_B1->CTLW0 &= ~0x0010;     // enable receiver
    EUSCI_B1->CTLW0 |= 0x0002;      // generate RESTART and send slave address

    while(EUSCI_B1->CTLW0 & 2);     // wait until restart is sent
    EUSCI_B1->CTLW0 |= 0x0004;      // setup to generate STOP after byte is received

    while(!(EUSCI_B1->IFG & 1));    // wait until data is received
    *data = EUSCI_B1->RXBUF;        // read the received data
    EUSCI_B1->IFG &= ~1;            // clear receive interrupt flag
    while(EUSCI_B1->CTLW0 & 4);     // wait until STOP is sent
    EUSCI_B1->IFG &= ~8;            // clear STOP interrupt flag

    return 0;                       // no error
}


int I2C1_burstRead2(int slaveAddr, unsigned char memAddr, int byteCount, int16_t* data) {
    /* burst read 2 bytes */
    if (byteCount <= 0) {
        return -1;  // no read was performed
    }
     EUSCI_B1->I2CSA = slaveAddr;   // set slave address
     EUSCI_B1->CTLW0 |= 0x0010;     // enable transmitter
     EUSCI_B1->CTLW0 |= 0x0002;     // generate START and send slave address

     while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
     EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
     EUSCI_B1->TXBUF = memAddr;      // send memory address to slave

     while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
     EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
     EUSCI_B1->CTLW0 &= ~0x0010;     // enable receiver
     EUSCI_B1->CTLW0 |= 0x0002;      // generate RESTART and send slave address
     while(EUSCI_B1->CTLW0 & 2);     // wait until restart is sent

     int16_t tmp; // temporary data holder
     do {
         if (byteCount == 1) {          // when only one byte of data is left
             EUSCI_B1->CTLW0 |= 0x0004; // setup to generate STOP after byte is received
         }
         while(!(EUSCI_B1->IFG & 1));   // wait until data is received

         if (byteCount % 2 == 0){       // if first of 2 bytes
             tmp = EUSCI_B1->RXBUF;     // read the received data into temp var
         }
         else {                         // if last of 2 bytes
             *data++ = (((tmp) << 8) | EUSCI_B1->RXBUF); // combine bytes
         }
         EUSCI_B1->IFG &= ~1;           // clear receive interrupt flag
         byteCount--;
     } while (byteCount);

     while(EUSCI_B1->CTLW0 & 4)     // wait until stop is sent
     EUSCI_B1->IFG &= ~8;           // clear STOP interrupt flag

     return 0;                      // no error
}


/**
* @file i2c_cust.c
* @brief MSP432 I2C driver
*
* MSP432 I2C driver
*
* @author Lucas Tiziani
* @date 2020-12-19
*
*/

/* TODO
 * generalize to other EUSCI modules
 */

#include "i2c_cust.h"


int I2Cc_write(int addr_slave, unsigned char memAddr, unsigned char data) {
    /* write single byte to I2C module 1 */
    EUSCI_B1->I2CSA = addr_slave;    // set slave address
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

    return 0;
}


int I2Cc_read(int addr_slave, unsigned char memAddr, unsigned char* data) {
    /* read single byte from I2C module 1*/
    EUSCI_B1->I2CSA = addr_slave;    // set slave address
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

    return 0;
}


int I2Cc_burstRead2(int addr_slave, unsigned char addr_mem, int num_bytes, int16_t* data) {
    /* Burst read sets of 2 bytes */
    if (num_bytes <= 0) {
        return -1;  // no read was performed
    }
     EUSCI_B1->I2CSA = addr_slave;  // set slave address
     EUSCI_B1->CTLW0 |= 0x0010;     // enable transmitter
     EUSCI_B1->CTLW0 |= 0x0002;     // generate START and send slave address

     while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
     EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
     EUSCI_B1->TXBUF = addr_mem;     // send memory address to slave

     while(!(EUSCI_B1->IFG & 2));    // wait until ready to transmit
     EUSCI_B1->IFG &= ~2;            // clear transmit interrupt flag
     EUSCI_B1->CTLW0 &= ~0x0010;     // enable receiver
     EUSCI_B1->CTLW0 |= 0x0002;      // generate RESTART and send slave address
     while(EUSCI_B1->CTLW0 & 2);     // wait until restart is sent

     int16_t tmp; // temporary data holder
     do {
         if (num_bytes == 1) {          // when only one byte of data is left
             EUSCI_B1->CTLW0 |= 0x0004; // generate STOP after byte is received
         }
         while(!(EUSCI_B1->IFG & 1));   // wait until data is received

         if (num_bytes % 2 == 0){       // if first of 2 bytes
             tmp = EUSCI_B1->RXBUF;     // read the received data into temp var
         }
         else {                         // if last of 2 bytes
             *data++ = (((tmp) << 8) | EUSCI_B1->RXBUF); // combine bytes
         }
         EUSCI_B1->IFG &= ~1;           // clear receive interrupt flag
         num_bytes--;
     } while (num_bytes);

     while(EUSCI_B1->CTLW0 & 4)     // wait until stop is sent
     EUSCI_B1->IFG &= ~8;           // clear STOP interrupt flag

     return 0;
}


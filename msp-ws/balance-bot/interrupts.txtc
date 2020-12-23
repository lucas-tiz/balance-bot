/*
 * interrupts.c
 *
 *  Created on: Jul 24, 2018
 *      Author: Lucas
 */

#include "interrupts.h"

//void TA0_0_IRQHandler(void) {
//    /* timer 0 interrupt routine*/
//    MAP_Interrupt_disableMaster(); // disable interrupts
//
//    //TODO: implement controller here?
//
//    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
//    MAP_Interrupt_enableMaster(); // enable interrupts
//}


void TA1_0_IRQHandler(void) {
    /* timer 1 interrupt routine: read sensors*/
    MAP_Interrupt_disableMaster(); // disable interrupts
    static int count;

    IMU_ReadVals(&imu);     // update values from IMU
    IMU_CalcAngleFused(&imu);  // calculate IMU orientation

    Enc_CalcAngle(&encR); // calculate encoder angles
    Enc_CalcAngle(&encL); // calculate encoder angles

    Sys_ControlUpdate(); // update motor velocity setpoints
    Motor_VelUpdate(&motorR, encR.vel[0]); // update right motor velocity
    Motor_VelUpdate(&motorL, -(encL.vel[0])); // update left motor velocity

    count++; // increment data transmission counter
    imuUpdateFlag = 1; // raise  IMU update flag

    // transmit data at 20 Hz
    if (count == 5)
    {
        transmitDataFlag = 1;
        count = 0;
    }

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
    MAP_Interrupt_enableMaster(); // enable interrupts
}


void PORT3_IRQHandler(void) {
    /* encoder interrupt handler */
    MAP_Interrupt_disableMaster(); // disable interrupts
    uint_fast16_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3); // get port 3 interrupt status


    if (status <= (ENC_0_CH_A_PIN | ENC_0_CH_B_PIN)) { // check which encoder
        Enc_InterruptRoutine(&encR);
//        LED2_Set(LED_GREEN); // toggle green LED
    }
    else if (status <= (ENC_1_CH_A_PIN | ENC_1_CH_B_PIN)) {
        Enc_InterruptRoutine(&encL);
//        LED2_Set(LED_BLUE); // toggle green LED
    }

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status); // clear interrupt flag
    MAP_Interrupt_enableMaster(); // disable interrupts
}


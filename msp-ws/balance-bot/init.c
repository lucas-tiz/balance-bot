/*
 * init.c
 *
 *  Created on: Jul 23, 2018
 *      Author: Lucas Tiziani
 */


//#include "init.h"
//
//
//void Clock_Config(void) {
//    /* configure master and subsystem master clocks */
//    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1); // flash wait state required for 48 MHz frequency
//    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1); // flash wait state required for 48 MHz frequency
//    MAP_CS_setDCOFrequency(DCO_FREQ); // set DCO clock source frequency to 48 MHz
//    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, SMCLK_DIV);  // tie SMCLK to DCO, 4 divider
//}


//void TimerA0_Config(void) {
    /* configure timer A0 for motor PWM signals, 1kHz */    //TODO: play with frequency
//    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, MOTOR_RF_PIN, GPIO_PRIMARY_MODULE_FUNCTION); // configure PWM pin (TA0.1 --> P2.4)
//    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, MOTOR_RB_PIN, GPIO_PRIMARY_MODULE_FUNCTION); // configure PWM pin (TA0.2 --> P2.5)
//    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, MOTOR_LF_PIN, GPIO_PRIMARY_MODULE_FUNCTION); // configure PWM pin (TA0.2 --> P2.6)
//    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, MOTOR_LB_PIN, GPIO_PRIMARY_MODULE_FUNCTION); // configure PWM pin (TA0.2 --> P2.7)
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, MOTOR_RF_PIN); // set output low
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, MOTOR_RB_PIN); // set output low
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, MOTOR_LF_PIN); // set output low
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, MOTOR_LB_PIN); // set output low

//    Timer_A_PWMConfig pwmRightForwardConfig =    // right forward PWM signal
//    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
//        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // clock source divider
//        MOTOR_PERIOD,                       // period of timer A
//        TIMER_A_CAPTURECOMPARE_REGISTER_1,  // use capture compare register 1
//        TIMER_A_OUTPUTMODE_RESET_SET,       // use reset set output mode
//        0                                   // set initial duty cycle
//    };
//    Timer_A_PWMConfig pwmRightBackwardConfig =   // right back PWM signal
//    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
//        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // clock source divider
//        MOTOR_PERIOD,                       // period of timer A
//        TIMER_A_CAPTURECOMPARE_REGISTER_2,  // use capture compare register 2
//        TIMER_A_OUTPUTMODE_RESET_SET,       // use reset set output mode
//        0                                   // set initial duty cycle
//    };
//    Timer_A_PWMConfig pwmLeftForwardConfig =   // left forward PWM signal
//    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
//        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // clock source divider
//        MOTOR_PERIOD,                       // period of timer A
//        TIMER_A_CAPTURECOMPARE_REGISTER_3,  // use capture compare register 2
//        TIMER_A_OUTPUTMODE_RESET_SET,       // use reset set output mode
//        0                                   // set initial duty cycle
//    };
//    Timer_A_PWMConfig pwmLeftBackwardConfig =   // left back PWM signal
//    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
//        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // clock source divider
//        MOTOR_PERIOD,                       // period of timer A
//        TIMER_A_CAPTURECOMPARE_REGISTER_4,  // use capture compare register 2
//        TIMER_A_OUTPUTMODE_RESET_SET,       // use reset set output mode
//        0                                   // set initial duty cycle
//    };
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmRightForwardConfig);     // TA0.1 --> P2.4
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmRightBackwardConfig);    // TA0.2 --> P2.5
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmLeftForwardConfig);      // TA0.3 --> P2.6
//    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmLeftBackwardConfig);     // TA0.4 --> P2.7
//    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);           // start timer A0
//}


//void TimerA1_Config(void) {
//    /* configure timer A1 for sensor reading, 100 Hz */
//    const Timer_A_UpModeConfig timerSensorConfig = // configure timer A in up mode
//    {   TIMER_A_CLOCKSOURCE_SMCLK,          // tie timer A to SMCLK
//        0x02,                               // clock source divider
//        SMCLK_FREQ/IMU_CALC_FREQ/2,           // period of timer
//        TIMER_A_TAIE_INTERRUPT_DISABLE,     // disable timer A rollover interrupt
//        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // enable capture compare interrupt
//        TIMER_A_DO_CLEAR                    // clear counter upon initialization
//    };
//    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &timerSensorConfig);  // configure timer per above struct
//    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear timer interrupt flag
//    MAP_Interrupt_setPriority(INT_TA1_0, 0);    // set highest interrupt priority
//    MAP_Interrupt_enableInterrupt(INT_TA1_0);   // enable timer interrupt on NVIC module
//    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);  // start timer A1
//}


//void TimerA2_Config(void) {
//// configure timer to measure elapsed time between gyro reads
//const Timer_A_UpModeConfig timerGyroRead = // configure timer A in up mode
//{   TIMER_A_CLOCKSOURCE_SMCLK,          // tie timer A to SMCLK
//    0x02,                               // 2 clock source divider
//    65535,                              // period of timer
//    TIMER_A_TAIE_INTERRUPT_DISABLE,     // disable timer A rollover interrupt
//    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,// enable capture compare interrupt
//    TIMER_A_DO_CLEAR                    // clear counter upon initialization
//};
//    // set up timers
//    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &timerGyroRead); // configure timer A2 to count elapsed gyro read time
//    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);  // start timer A2
//}


//void LED1_Config(void) {
//    /* configure LED1 */
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
//}
//
//
//void LED2_Config(void) {
//    /* configure LED2 */
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 |GPIO_PIN2);
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0| GPIO_PIN1 |GPIO_PIN2);
//}


//void I2C_Config(void) {
//    /* configure I2C module 1 */
//    EUSCI_B1->CTLW0 |= 1;       // disable UCB1 during configuration
//    EUSCI_B1->CTLW0 = 0x0F81;   // 7-bit slave address, master, I2C, synch mode, use SMCLK
//    EUSCI_B1->BRW = 0x1E;       // set clock prescaler to get 400kHz baud rate
//    P6->SEL0 |= 0x30;           // configure P6.4, P6.5 for UCB1: set register bits 4 & 5
//    P6->SEL1 &= ~0x30;          // configure P6.4, P6.5 for UCB1: clear register bits 4 & 5
//    EUSCI_B1->CTLW0 &= ~1;      // enable UCB1 after configuration
//}


//void UART_Config(void) {
//    /* configure UART to 38400 baud rate*/
//    const eUSCI_UART_Config uartConfig =
//    {    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // clock source
//         19,                                            // clock prescalar
//         8,                                             // first mod reg
//         85,                                            // second mod reg
//         EUSCI_A_UART_NO_PARITY,                        // no parity
//         EUSCI_A_UART_LSB_FIRST,                        // LSB first
//         EUSCI_A_UART_ONE_STOP_BIT,                     // one stop bit
//         EUSCI_A_UART_MODE,                             // UART mode
//         EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // oversampling on
//    };
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, (GPIO_PIN2 | GPIO_PIN3), GPIO_PRIMARY_MODULE_FUNCTION); // set pins P1.2 and P1.3 to UART mode
//    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig); // configure UART module instance A0
//    MAP_UART_enableModule(EUSCI_A0_BASE); // enable UART module instance A0
//}


//void EncGPIO_Config(volatile struct encStruct* enc, uint8_t port, uint8_t pinA, uint8_t pinB) {
//    /* configure encoder pins and add port/pin info to encoder struct */
//    enc->port = port;
//    enc->pins[0] = pinA;
//    enc->pins[1] = pinB;
//
//    MAP_GPIO_setAsInputPinWithPullDownResistor(port, pinA | pinB); // encoder 1 channel A --> P3.2
//    MAP_GPIO_clearInterruptFlag(port, pinA | pinB); // | GPIO_PIN6 | GPIO_PIN7); // clear
//    MAP_Interrupt_setPriority(port, 0); // set highest interrupt priority
//    MAP_Interrupt_enableInterrupt(port);
//}

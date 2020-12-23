/*
 * init.h
 *
 *  Created on: Jul 23, 2018
 *      Author: Lucas Tiziani
 */

//#ifndef INIT_H_
//#define INIT_H_


//#include "control.h"
//#include "encoder.h"
//#include "i2c_lt.h"
//#include "interrupts.h"
//#include "mpu6050.h"
//#include "uart_lt.h"
//#include "util.h"
//
//#include "msp.h"
//#include "driverlib.h"
//#include <stdbool.h>
//#include <stdio.h>


//// clocks
//#define DCO_FREQ 48E+6  // clock frequency
//#define SMCLK_DIV CS_CLOCK_DIVIDER_4     // subsystem master clock divider
//#define SMCLK_FREQ DCO_FREQ/4   // subsystem master clock frequency
//
//// timers
//#define IMU_CALC_FREQ 100.0       // (Hz) frequency of IMU position calculations
//#define ENC_CALC_FREQ 100.0       // (Hz) frequency at which encoder angles are calculated
//#define CONTROL_FREQ 100.0        // (Hz) control update frequency
//#define MOTOR_FREQ 1000         // motor PWM timer frequency
//#define MOTOR_PERIOD SMCLK_FREQ/MOTOR_FREQ // motor PWM timer period
//
//// encoder ports/pins
//#define ENC_0_PORT GPIO_PORT_P3     // encoder 0 port
//#define ENC_0_CH_A_PIN GPIO_PIN2    // encoder 0 channel A pin
//#define ENC_0_CH_B_PIN GPIO_PIN3    // encoder 0 channel B pin
//#define ENC_1_PORT GPIO_PORT_P3     // encoder 1 port
//#define ENC_1_CH_A_PIN GPIO_PIN6    // encoder 1 channel A pin
//#define ENC_1_CH_B_PIN GPIO_PIN7    // encoder 1 channel B pin
//
//// motor pins/registers
//#define MOTOR_RF_PIN GPIO_PIN4  // right motor forward pin (TA0.1 --> P2.4)
//#define MOTOR_RB_PIN GPIO_PIN5  // right motor backward pin (TA0.2 --> P2.5)
//#define MOTOR_LF_PIN GPIO_PIN7  // left motor forward pin (TA0.4 --> P2.7)
//#define MOTOR_LB_PIN GPIO_PIN6  // left motor backward pin (TA0.3 --> P2.6)
//#define MOTOR_RF_DUTY &TA0CCR1   // right motor forward duty cycle timer register (TA0.1 --> P2.4)
//#define MOTOR_RB_DUTY &TA0CCR2   // right motor backward duty cycle timer register (TA0.2 --> P2.5)
//#define MOTOR_LF_DUTY &TA0CCR4   // left motor forward duty cycle timer register (TA0.4 --> P2.7)
//#define MOTOR_LB_DUTY &TA0CCR3   // left motor backward duty cycle timer register (TA0.3 --> P2.6)
//
//// LEDs
//#define LED_OFF 0
//#define LED_RED 1
//#define LED_GREEN 2
//#define LED_BLUE 3



//// global variables
//extern volatile struct imuStruct imu;  // imu struct
//extern volatile struct encStruct encR; // right encoder struct
//extern volatile struct encStruct encL; // left encoder struct
//extern struct motorStruct motorR; // right motor struct
//extern struct motorStruct motorL; // left motor struct
//
//extern volatile bool imuUpdateFlag;
//extern volatile bool transmitDataFlag;
//
//extern volatile int debugFlag;
//extern volatile float debugVar;



// prototypes
//void Clock_Config(void);
//void TimerA0_Config(void);
//void TimerA1_Config(void);
//void LED1_Config(void);
//void LED2_Config(void);
//void I2C_Config(void);
//void UART_Config(void);

//void EncGPIO_Config(volatile struct encStruct* enc, uint8_t port, uint8_t pinA, uint8_t pinB);


//#endif /* INIT_H_ */

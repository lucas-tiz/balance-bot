/**
 * @file main.c
 * @brief Balance two-wheeled robot
 *
 * Stabilize two-wheeled balancing robot in upright orientation
 *
 * @author Lucas Tiziani
 * @date 2020-12-01
 *
 */

/*TODO:
 * check interrupt durations on scope
 * error checking on start-up
 * simplify update IMU functions
 * calibrate accelerometer
 *
 */


/* Header files */
#include "control.h"
#include "encoder.h"
#include "i2c_lt.h"
#include "interrupts.h"
#include "mpu6050.h"
#include "uart_lt.h"
#include "util.h"

#include "msp.h"
#include "driverlib.h"
#include <stdbool.h>
#include <stdio.h>


/* Macros */
#define NDEBUG 1

// clocks
//#define DCO_FREQ 48E+6  // clock frequency
#define SMCLK_DIV CS_CLOCK_DIVIDER_4     // subsystem master clock divider
#define SMCLK_FREQ DCO_FREQ/4   // subsystem master clock frequency

// timers
#define IMU_CALC_FREQ 100.0       // (Hz) frequency of IMU position calculations
#define ENC_CALC_FREQ 100.0       // (Hz) frequency at which encoder angles are calculated
#define CONTROL_FREQ 100.0        // (Hz) control update frequency
#define MOTOR_FREQ 1000         // motor PWM timer frequency
#define MOTOR_PERIOD SMCLK_FREQ/MOTOR_FREQ // motor PWM timer period

// encoder ports/pins
#define ENC_0_PORT GPIO_PORT_P3     // encoder 0 port
#define ENC_0_CH_A_PIN GPIO_PIN2    // encoder 0 channel A pin
#define ENC_0_CH_B_PIN GPIO_PIN3    // encoder 0 channel B pin
#define ENC_1_PORT GPIO_PORT_P3     // encoder 1 port
#define ENC_1_CH_A_PIN GPIO_PIN6    // encoder 1 channel A pin
#define ENC_1_CH_B_PIN GPIO_PIN7    // encoder 1 channel B pin

// motor pins/registers
#define MOTOR_RF_PIN GPIO_PIN4 // right motor forward pin   (TA0.1 --> P2.4)
#define MOTOR_RB_PIN GPIO_PIN5 // right motor backward pin  (TA0.2 --> P2.5)
#define MOTOR_LF_PIN GPIO_PIN7 // left motor forward pin    (TA0.4 --> P2.7)
#define MOTOR_LB_PIN GPIO_PIN6 // left motor backward pin   (TA0.3 --> P2.6)
#define MOTOR_RF_DUTY &TA0CCR1 // right motor for duty reg  (TA0.1 --> P2.4)
#define MOTOR_RB_DUTY &TA0CCR2 // right motor back duty reg (TA0.2 --> P2.5)
#define MOTOR_LF_DUTY &TA0CCR4 // left motor for duty reg   (TA0.4 --> P2.7)
#define MOTOR_LB_DUTY &TA0CCR3 // left motor back duty reg  (TA0.3 --> P2.6)

// LEDs
#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3



/* Global variables */
volatile struct imuStruct imu;  // imu struct
volatile struct encStruct encR; // right encoder struct
volatile struct encStruct encL; // left encoder struct
struct motorStruct motorR; // right motor struct
struct motorStruct motorL; // left motor struct

struct motorStruct motorR = {               // right motor struct
    .pins = {MOTOR_RF_PIN, MOTOR_RB_PIN},   // motor pins (0=forward, 1=backward)
    .duty = {MOTOR_RF_DUTY, MOTOR_RB_DUTY}, // duty cycle timer registers corresponding to pins
    .deadZone = 14.17,                      // (% duty cycle) motor dead zone (1700/12000)
    .velPID = {.kP = 0.03,                  // motor velocity control proportional gain
               .kI = 0.005,                 // motor velocity control integral gain
               .kD = 0},};                  // motor velocity control derivative
struct motorStruct motorL = {               // left motor struct
    .pins = {MOTOR_LF_PIN, MOTOR_LB_PIN},   // motor pins (0=forward, 1=backward)
    .duty = {MOTOR_LF_DUTY, MOTOR_LB_DUTY}, // duty cycle timer registers corresponding to pins
    .deadZone = 17.50,                      // (% duty cycle) motor dead zone (2100/12000)
    .velPID = {.kP = 0.03,                  // motor velocity control proportional gain
               .kI = 0.005,                     // motor velocity control integral gain
               .kD = 0},};                  // motor velocity control derivative

volatile bool flag_imu_read = 0;
volatile bool flag_transmit = 0;

#ifndef NDEBUG
    volatile bool flag_debug = 0;
    volatile float debug = 0;
#endif





// prototypes
float* recordData(void);


void main(void) {
    MAP_WDT_A_holdTimer(); // hold the watchdog timer (stop from running)
    MAP_Interrupt_disableMaster(); // disable interrupts

    // Configure master and subsystem master clocks
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1); // higher voltage level for 48 MHz
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_setDCOFrequency(48E+6);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, 1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4);

    // Configure motor PWM with Timer A0
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, MOTOR_RF_PIN |
        MOTOR_RB_PIN | MOTOR_LF_PIN | MOTOR_LB_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, MOTOR_RF_PIN | MOTOR_RB_PIN |
        MOTOR_LF_PIN | MOTOR_LB_PIN);

    Timer_A_PWMConfig pwm_right_forward_config =
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0 // initial duty cycle
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_right_forward_config); // TA0.1 --> P2.4

    Timer_A_PWMConfig pwm_right_back_config =
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0 // initial duty cycle
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_right_back_config); // TA0.2 --> P2.5




    // configure
    LED1_Config();      // configure LED1
    LED2_Config();      // configure LED2
    I2C_Config();       // configure I2C
    UART_Config();      // configure UART
    TimerA0_Config();   // configure timer for motor PWM signals
    TimerA1_Config();   // configure timer for sensor calculations
    EncGPIO_Config(&encR, ENC_0_PORT, ENC_0_CH_A_PIN, ENC_0_CH_B_PIN); // configure encoder 0
    EncGPIO_Config(&encL, ENC_1_PORT, ENC_1_CH_A_PIN, ENC_1_CH_B_PIN); // configure encoder 1


    // initialize
    delayMs(DCO_FREQ, 500); // delay to let IMU power up

    LED2_Set(LED_GREEN);
    Enc_Initialize(&encR); // initialize encoder 0
    Enc_Initialize(&encL); // initialize encoder 1
    delayMs(DCO_FREQ, 100);
    LED2_Set(LED_OFF);

    LED2_Set(LED_RED);
    IMU_Init(&imu, 44, 1000, 2);   // initialize IMU
    delayMs(DCO_FREQ, 100);
    LED2_Set(LED_OFF);

    LED2_Set(LED_BLUE);
    IMU_Calibrate(&imu, 20000); // calibrate IMU
    LED2_Set(LED_OFF);

    LED2_Set(LED_GREEN);
    delayMs(DCO_FREQ, 2000); // allow time for positioning
    LED2_Set(LED_OFF);

    MAP_Interrupt_enableMaster(); // enable interrupts

    // run
    float* dataArr;
    while(1) {
        if (imuUpdateFlag == 1) {
//            IMU_CalcAngleFused(&imu);  // calculate IMU orientation
            imuUpdateFlag = 0;  // clear IMU update flag
        }
        if (transmitDataFlag == 1) {
            transmitDataFlag = 0; // clear flag
            dataArr = recordData();
            UART_SendFloatArray(dataArr, 9);

            debugFlag++;

            //TODO: for calibration
            if (debugFlag == 200) { // 10 seconds
                motorR.velPID.velDes = 800;
                motorL.velPID.velDes = -800;
//                *(motorL.duty[0]) = 2100;
            }
            if (debugFlag == 300) { // 15 seconds
                motorR.velPID.velDes = 0;
                motorL.velPID.velDes = 0;
//                *(motorL.duty[0]) = 0;
                debugFlag = 0;
            }

            LED2_Set(LED_GREEN); // toggle green LED
        }
        //TODO: add recalibration option
    }
}


float* recordData(void) {
    /* record data to transmit via UART */
    static float data[9];      // vector of data to send via UART

    MAP_Interrupt_disableMaster(); // disable interrupts to record data

    data[0] = encR.velFilt;
    data[1] = -encL.velFilt;
    data[2] = debugVar;

    data[3] = imu.raw.angVel[0];
    data[4] = imu.raw.angVel[1];
    data[5] = imu.raw.angVel[2];

    data[6] = imu.angle.gyro[0];
    data[7] = imu.angle.accel[0];
    data[8] = imu.angle.fused[0][0];

//    data[6] = imu.angle.fused[0][0];
//    data[8] = imu.angVel.fused[0];
//    data[7] = imu.angle.accel[0];

    MAP_Interrupt_enableMaster(); // enable interrupts

    return data;
}

